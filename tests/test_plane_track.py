'''the path plane_track flies a mission along, against ArduPlane's own
navigation and a flight flown through it'''

import json
import math
import os

import pytest

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import plane_track

mavlink = mavutil.mavlink
HOME = (-35.363262, 149.165238, 584.0)
PARAMS = {
    'AIRSPEED_CRUISE': 22.0, 'AIRSPEED_MIN': 10.0, 'AIRSPEED_MAX': 40.0,
    'NAVL1_PERIOD': 15.0, 'NAVL1_DAMPING': 0.75,
    'ROLL_LIMIT_DEG': 50.0, 'WP_RADIUS': 60.0, 'WP_LOITER_RAD': 80.0,
    'TECS_CLMB_MAX': 5.0, 'TECS_SINK_MAX': 5.0,
}


def offset(north, east, alt=100.0, base=HOME):
    '''(lat, lon, amsl) north and east metres of base, alt above it'''
    (lat, lon) = mp_util.gps_offset(base[0], base[1], east, north)
    return (lat, lon, base[2] + alt)


def waypoint(north, east, alt=100.0):
    (lat, lon, amsl) = offset(north, east, alt)
    return (mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, amsl, (0, 0, 0, 0))


def loiter_to_alt(north, east, alt, radius=0.0, xtrack=0.0):
    (lat, lon, amsl) = offset(north, east, alt)
    return (mavlink.MAV_CMD_NAV_LOITER_TO_ALT, lat, lon, amsl,
            (0, radius, 0, xtrack))


def loiter_turns(north, east, turns, radius, alt=100.0, xtrack=0.0):
    (lat, lon, amsl) = offset(north, east, alt)
    return (mavlink.MAV_CMD_NAV_LOITER_TURNS, lat, lon, amsl,
            (turns, 0, radius, xtrack))


def change_speed(speed):
    return (mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, None, (0, speed, -1, 0))


def local(point):
    '''metres (north, east) of HOME'''
    return (mp_util.gps_distance(HOME[0], HOME[1], point[0], HOME[1]) *
            (1 if point[0] >= HOME[0] else -1),
            mp_util.gps_distance(HOME[0], HOME[1], HOME[0], point[1]) *
            (1 if point[1] >= HOME[1] else -1))


def fly(items, params=PARAMS, home=HOME):
    track = plane_track.mission_track(home, items, params)
    assert track is not None
    return track


def distance_to_line(point, start, end):
    '''signed metres of point to the right of the line start->end'''
    (n, e) = (point[0] - start[0], point[1] - start[1])
    (dn, de) = (end[0] - start[0], end[1] - start[1])
    length = math.hypot(dn, de)
    return (n * de - e * dn) / length


class TestL1Control(object):
    '''the controller port, one step at a time'''

    def control(self):
        return plane_track.L1Control(15.0, 0.75, 0.0, 0.0)

    def test_on_the_track_it_holds_straight(self):
        l1 = self.control()
        l1.update_waypoint((0, 0), (20, 0), 0.0, (-500, 0), (500, 0), 0.1)
        assert l1.lateral_acceleration == pytest.approx(0.0, abs=1e-9)

    def test_off_the_track_it_turns_back_towards_it(self):
        l1 = self.control()
        # east of a track running north, it turns left
        l1.update_waypoint((0, 50), (20, 0), 0.0, (-500, 0), (500, 0), 0.1)
        assert l1.lateral_acceleration < 0
        l1.update_waypoint((0, -50), (20, 0), 0.0, (-500, 0), (500, 0), 0.1)
        assert l1.lateral_acceleration > 0

    def test_far_from_a_loiter_it_heads_for_the_centre(self):
        l1 = self.control()
        l1.update_loiter((-1000, 0), (20, 0), 0.0, (0, 0), 100.0, 1)
        assert not l1.circling
        assert l1.lateral_acceleration == pytest.approx(0.0, abs=1e-6)

    def test_on_the_circle_it_turns_at_the_circles_rate(self):
        l1 = self.control()
        # due west of the centre, flying north: clockwise round it
        l1.update_loiter((0, -100), (20, 0), 0.0, (0, 0), 100.0, 1)
        assert l1.circling
        assert l1.lateral_acceleration == pytest.approx(20.0 ** 2 / 100.0)

    def test_the_circle_flown_grows_with_altitude(self):
        l1 = self.control()
        assert l1.loiter_radius(100.0, 0.0, 20.0) == pytest.approx(100.0)
        assert l1.loiter_radius(100.0, 3000.0, 20.0) > 130.0


class TestMissionFlight(object):
    '''ArduPlane's mission logic, flown'''

    def test_it_flies_the_legs_between_waypoints(self):
        track = fly([waypoint(3000, 0), waypoint(3000, 3000)])
        points = [local(p) for p in track]
        # tracking the first leg north, well away from its ends
        on_leg = [p for p in points if 800 < p[0] < 2200 and p[1] < 1500]
        assert on_leg
        assert max(abs(p[1]) for p in on_leg) < 3.0
        # and it ends at the last one, which is done with once it is within
        # the distance it would start turning for whatever came next
        assert math.hypot(points[-1][0] - 3000, points[-1][1] - 3000) < 100.0

    def test_a_turn_is_cut_short_of_the_waypoint(self):
        track = fly([waypoint(3000, 0), waypoint(3000, 3000)])
        points = [local(p) for p in track]
        nearest = min(math.hypot(p[0] - 3000, p[1]) for p in points)
        # a 90 degree turn is started WP_RADIUS or the L1 distance out
        assert nearest > 20.0

    def test_a_leg_into_a_loiter_tracks_towards_its_centre(self):
        # out of one loiter and into another, the track is the line between
        # their centres rather than a tangent from one circle to the other,
        # until the aircraft is within three radii of the second centre: it
        # leaves the first circle a radius to one side of that line, and
        # pulls onto it
        first = (0.0, 2000.0)
        second = (6000.0, 2000.0)
        track = fly([loiter_turns(first[0], first[1], 1, 300),
                     loiter_turns(second[0], second[1], 1, 150),
                     waypoint(9000, 2000)])
        points = [local(p) for p in track]
        radius = plane_track.L1Control(15, 0.75, 0, 0).loiter_radius(150, 684, 22)
        approach = [p for p in points
                    if 2500 < p[0] < second[0] - 3 * radius - 300 and
                    abs(p[1] - 2000) < 500]
        assert approach
        assert max(abs(distance_to_line(p, first, second)) for p in approach) < 5.0

    def test_a_loiter_is_left_lined_up_on_what_comes_next(self):
        centre = (0.0, 3000.0)
        after = (3000.0, 3000.0)
        track = fly([loiter_turns(centre[0], centre[1], 1, 200),
                     waypoint(*after)])
        points = [local(p) for p in track]
        radius = plane_track.L1Control(15, 0.75, 0, 0).loiter_radius(200, 684, 22)
        leaving = [p for p in points
                   if p[0] > 400 and math.hypot(p[0] - centre[0], p[1] - centre[1]) > radius + 50]
        assert leaving
        # the leg is flown against the track out of the loiter's centre, so it
        # ends up on the line from the centre to the waypoint
        late = [p for p in leaving if 1500 < p[0] < 2500]
        assert max(abs(distance_to_line(p, centre, after)) for p in late) < 5.0

    def test_param4_crosstracks_from_where_the_loiter_is_left(self):
        centre = (0.0, 3000.0)
        after = (3000.0, 3000.0)
        from_centre = [local(p) for p in fly(
            [loiter_turns(centre[0], centre[1], 1, 200), waypoint(*after)])]
        from_exit = [local(p) for p in fly(
            [loiter_turns(centre[0], centre[1], 1, 200, xtrack=1), waypoint(*after)])]
        # from the exit the aircraft does not pull back onto the centre line,
        # so halfway along the leg it is still well to one side of it
        halfway = [p for p in from_exit if 1400 < p[0] < 1600]
        assert halfway
        assert min(abs(distance_to_line(p, centre, after)) for p in halfway) > 50.0
        halfway = [p for p in from_centre if 1400 < p[0] < 1600]
        assert max(abs(distance_to_line(p, centre, after)) for p in halfway) < 5.0

    def test_loiters_turn_the_way_their_radius_says(self):
        for (radius, clockwise) in ((200, True), (-200, False)):
            points = [local(p) for p in fly(
                [loiter_turns(0, 3000, 2, radius), waypoint(3000, 3000)])]
            circling = [p for p in points
                        if math.hypot(p[0], p[1] - 3000) < 400]
            swept = 0.0
            for (a, b) in zip(circling, circling[1:]):
                swept += mp_util.wrap_180(
                    math.degrees(math.atan2(b[1] - 3000, b[0])) -
                    math.degrees(math.atan2(a[1] - 3000, a[0])))
            # a bearing from the centre measured east of north grows clockwise
            if clockwise:
                assert swept > 600
            else:
                assert swept < -600

    def test_a_loiter_to_altitude_circles_until_it_gets_there(self):
        climb = 1000.0
        points = fly([loiter_to_alt(0, 3000, 100 + climb, 150),
                      waypoint(3000, 3000, 100 + climb)])
        centre = offset(0, 3000)
        on_circle = [p for p in points
                     if mp_util.gps_distance(p[0], p[1], centre[0], centre[1]) < 260]
        # climbing at 5 m/s it spends at least climb / 5 seconds up there
        # (less what it climbs on the way), a circle every 2 pi r / v seconds
        laps = len(on_circle) * plane_track.POINT_SPACING / (2 * math.pi * 170)
        assert laps > 2.0
        assert max(p[2] for p in points) == pytest.approx(HOME[2] + 100 + climb, abs=6.0)

    def test_a_faster_airspeed_takes_a_wider_line_through_a_turn(self):
        def overshoot(speed):
            items = [waypoint(3000, 0), waypoint(3000, 3000)]
            if speed is not None:
                items.insert(0, change_speed(speed))
            points = [local(p) for p in fly(items)]
            return max(p[0] for p in points)
        assert overshoot(35.0) > overshoot(None) + 10.0

    def test_only_an_airspeed_within_its_limits_changes_the_speed(self):
        def overshoot(params, change=None):
            items = [waypoint(3000, 0), waypoint(3000, 3000)]
            if change is not None:
                items.insert(0, (mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, None,
                                 change))
            return max(local(p)[0] for p in fly(items, params))
        params = PARAMS
        plain = overshoot(params)
        assert overshoot(params, (0, 35, -1, 0)) > plain + 10.0
        # a groundspeed is only a minimum ground speed to ArduPlane
        assert overshoot(params, (1, 35, -1, 0)) == pytest.approx(plain)
        # and it will not take an airspeed outside AIRSPEED_MIN/MAX
        assert overshoot(dict(params, AIRSPEED_MAX=30),
                         (0, 35, -1, 0)) == pytest.approx(plain)
        assert overshoot(params, (0, 5, -1, 0)) == pytest.approx(plain)
        # -2 goes back to AIRSPEED_CRUISE
        assert overshoot(params, (0, -2, -1, 0)) == pytest.approx(plain)

    def test_a_loiter_unlimited_ends_the_flight_on_its_circle(self):
        (lat, lon, amsl) = offset(0, 3000)
        track = fly([(mavlink.MAV_CMD_NAV_LOITER_UNLIM, lat, lon, amsl,
                      (0, 0, 150, 0)), waypoint(3000, 3000)])
        last = track[-1]
        radius = mp_util.gps_distance(last[0], last[1], lat, lon)
        assert 120 < radius < 220

    def test_what_it_cannot_fly_is_not_drawn(self):
        delay = (mavlink.MAV_CMD_NAV_DELAY, 0, 0, None, (10, 0, 0, 0))
        assert plane_track.mission_track(
            HOME, [waypoint(3000, 0), delay, waypoint(3000, 3000)], PARAMS) is None
        assert plane_track.mission_track(
            (0, 0, 0), [waypoint(3000, 0)], PARAMS) is None

    def test_a_waypoint_too_close_to_turn_for_is_passed(self):
        # ArduPlane counts a waypoint done once the aircraft is past it along
        # the leg, so one right beside the last does not leave it circling
        track = fly([waypoint(3000, 0), waypoint(3000, 20), waypoint(3020, 20)],
                    dict(PARAMS, ROLL_LIMIT_DEG=10.0))
        assert len(track) > 10

    def jump(self, target, repeats, command=mavlink.MAV_CMD_DO_JUMP):
        return (command, 0, 0, None, (target, repeats, 0, 0))

    def visits(self, track, north, east, within=80.0):
        '''how many separate times the track passes near a point'''
        (lat, lon, _) = offset(north, east)
        count = 0
        near = False
        for p in track:
            close = mp_util.gps_distance(p[0], p[1], lat, lon) < within
            if close and not near:
                count += 1
            near = close
        return count

    def test_a_jump_is_followed(self):
        # 1 and 2 are a circuit's first two corners; 3 jumps past a
        # waypoint which the mission only reaches by the jump back at 6
        items = [waypoint(2000, 0), waypoint(2000, 2000),
                 self.jump(5, -1), waypoint(-3000, -3000),
                 waypoint(0, 2000), self.jump(4, 1), waypoint(0, 4000)]
        track = fly(items)
        # never flown straight from 2 to 4: 4 is reached once, by the jump
        # back at 6, and 5 is flown before and after it
        assert self.visits(track, -3000, -3000) == 1
        assert self.visits(track, 0, 2000) == 2
        assert self.visits(track, 0, 4000) == 1

    def test_a_jump_is_drawn_once_however_often_it_repeats(self):
        items = [waypoint(2000, 0), waypoint(2000, 2000), waypoint(0, 2000),
                 self.jump(1, 50), waypoint(0, 4000)]
        track = fly(items)
        assert self.visits(track, 2000, 2000) == 2
        assert self.visits(track, 0, 4000) == 1

    def test_a_jump_repeated_for_ever_ends_the_flight(self):
        # the aircraft never gets past it: the circuit is drawn going round
        # the once, and nothing after the jump is drawn
        items = [waypoint(2000, 0), waypoint(2000, 2000), waypoint(0, 2000),
                 self.jump(1, -1), waypoint(0, 4000)]
        track = fly(items)
        assert self.visits(track, 2000, 2000) == 2
        assert self.visits(track, 0, 4000) == 0

    def test_a_jump_with_no_repeats_is_not_followed(self):
        # followed, it would skip the waypoint straight after it
        items = [waypoint(2000, 0), self.jump(4, 0), waypoint(2000, 2000),
                 waypoint(0, 2000)]
        assert self.visits(fly(items), 2000, 2000) == 1

    def test_a_jump_to_a_tag(self):
        tag = (600, 0, 0, None, (7, 0, 0, 0))
        items = [waypoint(2000, 0), self.jump(7, 1, command=601),
                 waypoint(-3000, -3000), tag, waypoint(0, 2000)]
        track = fly(items)
        assert self.visits(track, -3000, -3000) == 0
        assert self.visits(track, 0, 2000) == 1

    def test_a_jump_nowhere_ends_the_mission(self):
        # AP_Mission finds no next command, and the mission is over
        track = fly([waypoint(2000, 0), self.jump(9, 1), waypoint(0, 2000)])
        assert self.visits(track, 2000, 0) == 1
        assert self.visits(track, 0, 2000) == 0

    def takeoff(self, alt=50.0, command=mavlink.MAV_CMD_NAV_TAKEOFF):
        # a takeoff item placed at home, as a ground station puts it
        return (command, HOME[0], HOME[1], HOME[2] + alt, (0, 0, 0, 0))

    def climbed_out(self, track, alt=50.0):
        '''metres from home where the track first reaches the takeoff
        altitude'''
        for p in track:
            if p[2] >= HOME[2] + alt - 0.5:
                return mp_util.gps_distance(HOME[0], HOME[1], p[0], p[1])
        return None

    def test_a_fixed_wing_takeoff_climbs_out_at_full_rate(self):
        # towards the first waypoint, at TECS_CLMB_MAX: 50m at 5m/s is ten
        # seconds at 22m/s, not a slow approach to the altitude
        track = fly([self.takeoff(), waypoint(3000, 0)])
        distance = self.climbed_out(track)
        assert distance == pytest.approx(220.0, abs=40.0)
        (lat, lon, _) = track[-1]
        assert mp_util.gps_bearing(HOME[0], HOME[1], lat, lon) == pytest.approx(0.0, abs=5.0)

    def test_a_fixed_wing_takeoff_holds_the_course_it_is_given(self):
        # the course the aircraft had on the ground, not the way to the
        # first waypoint: east, with the waypoint to the north
        track = plane_track.mission_track(
            HOME, [self.takeoff(), waypoint(3000, 0)], PARAMS, heading=90.0)
        climbed = [local(p) for p in track if p[2] >= HOME[2] + 49.5][0]
        assert climbed[1] == pytest.approx(220.0, abs=40.0)
        assert abs(climbed[0]) < 5.0
        # and it only turns for the waypoint once the takeoff is done
        assert max(local(p)[1] for p in track) > 250.0

    def test_a_quadplane_takes_off_straight_up(self):
        params = dict(PARAMS, Q_ENABLE=1, Q_OPTIONS=0)
        for command in (mavlink.MAV_CMD_NAV_TAKEOFF,
                        mavlink.MAV_CMD_NAV_VTOL_TAKEOFF):
            # and sets off from overhead home towards the waypoint behind it
            track = fly([self.takeoff(command=command), waypoint(-3000, 0)],
                        params)
            assert self.climbed_out(track) == pytest.approx(0.0, abs=1.0)
            assert max(local(p)[0] for p in track) < 5.0
        # unless Q_OPTIONS lets NAV_TAKEOFF be a fixed-wing one
        track = fly([self.takeoff(), waypoint(3000, 0)],
                    dict(params, Q_OPTIONS=plane_track.Q_OPTION_ALLOW_FW_TAKEOFF))
        assert self.climbed_out(track) > 100.0

    def test_a_mission_started_away_from_home(self):
        # ArduPlane starts where the aircraft is when AUTO starts; home is
        # only where it came from
        start = offset(-1000, 500, 20)
        track = plane_track.mission_track(
            HOME, [waypoint(3000, 0), waypoint(3000, 3000)], PARAMS,
            start=start)
        assert track[0] == pytest.approx(start)
        # home still stands for home: a return to launch goes back there
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        track = plane_track.mission_track(
            HOME, [waypoint(3000, 0), rtl], PARAMS, start=start)
        home_ward = [p for p in track
                     if mp_util.gps_distance(p[0], p[1],
                                             HOME[0], HOME[1]) < 200.0]
        assert home_ward

    def test_a_return_to_launch_circles_where_it_returns_to(self):
        """ModeRTL::update circles at RTL_RADIUS, or WP_LOITER_RAD"""
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        for (rtl_radius, radius, clockwise) in ((0.0, 60.0, True),
                                                (250.0, 250.0, True),
                                                (-250.0, 250.0, False)):
            params = dict(PARAMS, RTL_RADIUS=rtl_radius)
            points = [local(p) for p in
                      fly([waypoint(3000, 0), rtl], params=params)]
            # once it is home for the last time, it is circling there
            home_ward = [i for (i, p) in enumerate(points)
                         if math.hypot(p[0], p[1]) > 600][-1]
            circling = points[home_ward:]
            circling = circling[len(circling) // 2:]
            # at the radius asked for, allowing the room an aircraft which
            # cannot bank that tightly takes
            for p in circling:
                assert radius <= math.hypot(p[0], p[1]) <= radius * 1.8
            swept = 0.0
            for (a, b) in zip(circling, circling[1:]):
                swept += mp_util.wrap_180(
                    math.degrees(math.atan2(b[1], b[0])) -
                    math.degrees(math.atan2(a[1], a[0])))
            # a bearing from the centre grows clockwise
            assert swept > 120 if clockwise else swept < -120

    def test_the_first_item_navigated_to(self):
        items = [change_speed(30), self.jump(4, 1), waypoint(2000, 0),
                 waypoint(3000, 0)]
        assert plane_track.first_navigation_item(items) == 4
        assert plane_track.first_navigation_item([change_speed(30)]) is None

    def test_a_mission_across_the_antimeridian(self):
        import time
        home = (10.0, 179.998, 0.0)
        east = mp_util.gps_newpos(home[0], home[1], 90, 300)
        assert east[1] < 0
        start = time.time()
        track = fly([(mavlink.MAV_CMD_NAV_WAYPOINT, east[0], east[1], 100.0,
                      (0, 0, 0, 0))], home=home)
        assert time.time() - start < 2.0
        # ending at the waypoint, having flown the short way round
        (lat, lon, _) = track[-1]
        assert mp_util.gps_distance(lat, lon, east[0], east[1]) < 100.0
        assert len(track) < 100

    def test_a_mission_too_long_to_fly_is_not(self):
        import time
        far = offset(2000 * 1000, 0)
        start = time.time()
        assert plane_track.mission_track(
            HOME, [waypoint(3000, 0),
                   (mavlink.MAV_CMD_NAV_WAYPOINT, far[0], far[1], far[2],
                    (0, 0, 0, 0))], PARAMS) is None
        assert time.time() - start < 0.5

    def test_a_mission_too_long_to_fly_is_not_flown_at_all(self, monkeypatch):
        # the check is there to save flying it: it must not be flown first
        flights = []
        monkeypatch.setattr(plane_track.MissionFlight, 'run',
                            lambda self: flights.append(self))
        far = offset(2000 * 1000, 0)
        assert plane_track.mission_track(
            HOME, [waypoint(3000, 0),
                   (mavlink.MAV_CMD_NAV_WAYPOINT, far[0], far[1], far[2],
                    (0, 0, 0, 0))], PARAMS) is None
        assert flights == []

    def test_a_mission_is_measured_from_where_it_starts(self):
        # the aircraft carried a long way from home before the mission
        # starts flies from there, so it is no longer than it looks
        start = offset(2000 * 1000, 0)
        items = [(mavlink.MAV_CMD_NAV_WAYPOINT, start[0], start[1], start[2],
                  (0, 0, 0, 0))]
        assert plane_track.least_flight_time(
            HOME, items, PARAMS, start=start) < 60.0
        assert plane_track.least_flight_time(HOME, items, PARAMS) > 4 * 3600
        assert plane_track.mission_track(HOME, items, PARAMS,
                                         start=start) is not None
        assert plane_track.mission_track(HOME, items, PARAMS) is None

    def test_a_jump_to_the_last_item_is_followed(self):
        # items are numbered from home, which is not among them, so the
        # last item is numbered as many as there are
        items = [waypoint(2000, 0), waypoint(2000, 2000),
                 (mavlink.MAV_CMD_DO_JUMP, 0, 0, None, (3, 1, 0, 0)),
                 waypoint(0, 2000)]
        flight = plane_track.MissionFlight((HOME[0], HOME[1]), HOME,
                                           list(items), PARAMS)
        assert flight.jump_target(2) == 2
        # and one past it goes nowhere
        beyond = list(items)
        beyond[2] = (mavlink.MAV_CMD_DO_JUMP, 0, 0, None, (5, 1, 0, 0))
        flight = plane_track.MissionFlight((HOME[0], HOME[1]), HOME,
                                           list(beyond), PARAMS)
        assert flight.jump_target(2) is None

    def test_the_flight_time_limit_holds_inside_an_item(self, monkeypatch):
        # a mission short enough in straight lines, but which takes longer
        # than the limit to fly, stops at the limit rather than flying on to
        # the end of the item it is in
        longest = []
        real = plane_track.MissionFlight.fly

        def timed(flight, *args, **kwargs):
            real(flight, *args, **kwargs)
            longest.append(flight.time)
        monkeypatch.setattr(plane_track.MissionFlight, 'fly', timed)
        monkeypatch.setattr(plane_track, 'MAX_FLIGHT_TIME', 250.0)
        items = [waypoint(8000, 0)]
        assert plane_track.mission_track(HOME, items, PARAMS) is None
        assert longest and max(longest) < 250.0 + plane_track.NAV_PERIOD * 2
        monkeypatch.setattr(plane_track, 'MAX_FLIGHT_TIME', 3600.0)
        assert plane_track.mission_track(HOME, items, PARAMS) is not None

    def test_a_return_to_launch_climbs_to_rtl_altitude(self):
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        items = [waypoint(3000, 0, alt=40), rtl]

        def arrives(params=None):
            return fly(items, dict(PARAMS, **(params or {})))[-1][2] - HOME[2]
        # ArduPlane's default is 100m above home; the height follows the
        # glide slope home a few seconds behind
        assert arrives() == pytest.approx(100, abs=10.0)
        assert arrives({'RTL_ALTITUDE': 250}) == pytest.approx(250, abs=15.0)
        # and a negative one comes home at the altitude it is at
        assert arrives({'RTL_ALTITUDE': -1}) == pytest.approx(40, abs=3.0)

    def test_a_loiter_time_holds_to_the_flight_time_limit(self, monkeypatch):
        longest = []
        real = plane_track.MissionFlight.fly

        def timed(flight, *args, **kwargs):
            real(flight, *args, **kwargs)
            longest.append(flight.time)
        monkeypatch.setattr(plane_track.MissionFlight, 'fly', timed)
        monkeypatch.setattr(plane_track, 'MAX_FLIGHT_TIME', 250.0)
        (lat, lon, amsl) = offset(2200, 0)
        items = [(mavlink.MAV_CMD_NAV_LOITER_TIME, lat, lon, amsl,
                  (100, 0, 0, 0)), waypoint(2200, 300)]
        params = dict(PARAMS, AIRSPEED_CRUISE=12.0, AIRSPEED_MAX=22.0)
        # plainly short enough to try, in straight lines
        assert plane_track.least_flight_time(HOME, items, params) < 250.0
        assert plane_track.mission_track(HOME, items, params) is None
        assert longest and max(longest) < 250.0 + plane_track.NAV_PERIOD * 2

    def test_the_least_flight_time_follows_jumps(self):
        # a jump straight over a waypoint a long way off: AP_Mission never
        # goes there, so neither is it counted
        far = offset(1000 * 1000, 0)
        jump = (mavlink.MAV_CMD_DO_JUMP, 0, 0, None, (3, 1, 0, 0))
        items = [jump, (mavlink.MAV_CMD_NAV_WAYPOINT, far[0], far[1], far[2],
                        (0, 0, 0, 0)), waypoint(1000, 0)]
        assert plane_track.least_flight_time(HOME, items, PARAMS) < 60.0
        assert plane_track.mission_track(HOME, items, PARAMS) is not None
        # and without the jump, it is far too far
        assert plane_track.least_flight_time(HOME, items[1:], PARAMS) > 20000.0

    def rtl_end(self, rally, params=None):
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        track = plane_track.mission_track(
            HOME, [waypoint(3000, 0), rtl], dict(PARAMS, **(params or {})),
            rally=rally)
        return track[-1]

    def test_a_return_to_launch_goes_to_the_nearest_rally_point(self):
        near = offset(2500, 500, 150)
        further = offset(-2000, 0, 150)
        end = self.rtl_end([further, near])
        assert mp_util.gps_distance(end[0], end[1], near[0], near[1]) < 150.0
        assert end[2] == pytest.approx(near[2], abs=15.0)
        # unless it is beyond RALLY_LIMIT_KM, when it goes home
        end = self.rtl_end([near], {'RALLY_LIMIT_KM': 0.3})
        assert mp_util.gps_distance(end[0], end[1], HOME[0], HOME[1]) < 150.0
        # or RALLY_INCL_HOME has it take home, being nearer
        end = self.rtl_end([further], {'RALLY_INCL_HOME': 1})
        assert mp_util.gps_distance(end[0], end[1], HOME[0], HOME[1]) < 150.0
        end = self.rtl_end([further])
        assert mp_util.gps_distance(end[0], end[1], further[0],
                                    further[1]) < 150.0

    def test_turns_are_flown_as_ap_mission_keeps_them(self):
        def turns(n):
            return fly([loiter_turns(2000, 0, n, 200), waypoint(2000, 3000)])
        # whole turns, the fraction dropped, as AP_Mission stores them
        assert turns(2.5) == turns(2.0)
        assert turns(2.5) != turns(3.0)

    def test_a_loiter_time_flies_the_vehicles_radius_whatever_it_asks(self):
        def loiter(param3):
            (lat, lon, amsl) = offset(2000, 0)
            return fly([(mavlink.MAV_CMD_NAV_LOITER_TIME, lat, lon, amsl,
                         (60, 0, param3, 0)), waypoint(2000, 3000)])
        assert loiter(300) == loiter(0)
        assert loiter(-300) != loiter(0)

    def test_no_glide_slope_with_alt_slope_min_zero(self):
        # a climb along a leg follows the slope to the waypoint, unless
        # ALT_SLOPE_MIN is zero or less, when it climbs as fast as it can
        items = [waypoint(0, 50, alt=100), waypoint(6000, 50, alt=400)]

        def halfway(params):
            track = fly(items, dict(PARAMS, CLIMB_SLOPE_HGT=0, **params))
            return min((abs(local(p)[0] - 3000), p[2]) for p in track)[1]
        sloped = halfway({})
        assert sloped == pytest.approx(HOME[2] + 250, abs=25.0)
        flat_out = halfway({'ALT_SLOPE_MIN': 0})
        assert flat_out == pytest.approx(HOME[2] + 400, abs=5.0)

    def test_the_altitude_of_a_rally_point(self):
        # above home unless the flags say otherwise
        assert plane_track.rally_amsl(100, 0, 584.0) == 684.0
        frame_valid = 1 << 2
        assert plane_track.rally_amsl(700, frame_valid | (0 << 3), 584.0) == 700.0
        assert plane_track.rally_amsl(100, frame_valid | (1 << 3), 584.0) == 684.0
        # above the EKF origin, and above the terrain under the point,
        # which the caller looks up
        assert plane_track.rally_amsl(100, frame_valid | (2 << 3), 584.0,
                                      origin_amsl=500.0) == 600.0
        assert plane_track.rally_amsl(100, frame_valid | (2 << 3), 584.0) is None
        assert plane_track.rally_amsl(100, frame_valid | (3 << 3), 584.0,
                                      terrain_amsl=300.0) == 400.0
        assert plane_track.rally_amsl(100, frame_valid | (3 << 3), 584.0) is None
        assert plane_track.rally_amsl(100, 0, None) is None
        # only a point above home moves when home does
        assert plane_track.rally_alt_is_fixed(0) is False
        assert plane_track.rally_alt_is_fixed(frame_valid | (1 << 3)) is False
        for frame in (0, 2, 3):
            assert plane_track.rally_alt_is_fixed(
                frame_valid | (frame << 3)) is True

    def test_a_rally_point_at_an_unknown_altitude_is_not_flown_to(self):
        # terrain nobody has, most likely: how high the aircraft flies the
        # return is anyone's guess, so the mission is not drawn flown
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        items = [waypoint(2000, 0), rtl]
        (lat, lon, amsl) = offset(2200, 200)
        assert plane_track.mission_track(HOME, items, PARAMS,
                                         rally=[(lat, lon, amsl)]) is not None
        assert plane_track.mission_track(HOME, items, PARAMS,
                                         rally=[(lat, lon, None)]) is None
        # one it does not go to does not matter
        far = offset(2000 * 100, 0)
        assert plane_track.mission_track(
            HOME, items, PARAMS, rally=[(far[0], far[1], None)]) is not None

    def test_the_altitude_an_item_with_no_position_is_flown_at(self):
        # Location::sanitize(): its own altitude, unless that is a relative 0
        assert plane_track.positionless_amsl(700.0, 0, 584.0) == 700.0
        assert plane_track.positionless_amsl(100.0, 3, 584.0) == 684.0
        assert plane_track.positionless_amsl(0.0, 3, 584.0) is None
        assert plane_track.positionless_amsl(100.0, 3, None) is None
        # the terrain under wherever the aircraft is, is not known here
        assert plane_track.positionless_amsl(100.0, 10, 584.0) is None
        assert plane_track.positionless_amsl(None, 0, 584.0) is None

    def test_without_parameters_arduplanes_defaults_fly(self):
        track = plane_track.mission_track(
            HOME, [waypoint(3000, 0), loiter_turns(3000, 3000, 1, 0)], {})
        assert track is not None


class TestFlownMission(object):
    """a QuadPlane SITL flight of ArduPilot's KalaupapaCanyonRun mission,
    which loiters up and down four times and runs a winding gorge between
    them: the path worked out for the mission against the one flown"""

    def fixture(self):
        path = os.path.join(os.path.dirname(__file__), 'missions',
                            'kalaupapa-canyon-run.json')
        with open(path) as f:
            return json.load(f)

    def distances(self, track, flown):
        '''metres from each flown point to the nearest leg of track'''
        (lat0, lon0) = (track[0][0], track[0][1])
        scale = math.cos(math.radians(lat0))

        def xy(p):
            return ((p[0] - lat0) * 111319.5, (p[1] - lon0) * 111319.5 * scale)
        segments = [(xy(a), xy(b)) for (a, b) in zip(track, track[1:])]
        out = []
        for p in flown:
            (pn, pe) = xy(p)
            best = None
            for ((an, ae), (bn, be)) in segments:
                (dn, de) = (bn - an, be - ae)
                squared = dn * dn + de * de
                t = 0.0
                if squared > 0:
                    t = min(max(((pn - an) * dn + (pe - ae) * de) / squared, 0.0), 1.0)
                d = math.hypot(pn - (an + t * dn), pe - (ae + t * de))
                if best is None or d < best:
                    best = d
            out.append(best)
        return sorted(out)

    def arrivals(self, path, items):
        '''for each item with a position, in mission order, the index into
        path of the first point near it: a waypoint's is within the distance
        a turn is started out, a loiter's within its circle'''
        out = []
        for (seq, (command, lat, lon, amsl, params)) in enumerate(items, 1):
            if (lat == 0 and lon == 0) or command in (
                    mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, mavlink.MAV_CMD_NAV_VTOL_LAND):
                continue
            near = 120.0
            if command == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
                near = abs(params[1]) * 1.4
            index = None
            for (i, point) in enumerate(path):
                if mp_util.gps_distance(point[0], point[1], lat, lon) < near:
                    index = i
                    break
            out.append((seq, index))
        return out

    def length(self, path, start, end):
        return sum(mp_util.gps_distance(a[0], a[1], b[0], b[1])
                   for (a, b) in zip(path[start:end], path[start + 1:end + 1]))

    def test_the_path_is_flown_in_the_order_flown(self):
        # the distances alone would not notice legs flown in the wrong order,
        # backwards or with extra loops, so long as the lines were there
        data = self.fixture()
        items = [(c, la, lo, a, tuple(p)) for (c, la, lo, a, p) in data['items']]
        track = plane_track.mission_track(tuple(data['home']), items,
                                          data['params'])
        flown = data['flown']
        drawn = self.arrivals(track, items)
        seen = self.arrivals(flown, items)
        # the flight kept ends short of the last waypoint
        seen = [(seq, i) for (seq, i) in seen if i is not None]
        order = [seq for (seq, i) in sorted(seen, key=lambda x: x[1])]
        assert order == [seq for (seq, i) in seen]
        assert None not in [i for (seq, i) in drawn]
        assert ([seq for (seq, i) in sorted(drawn, key=lambda x: x[1])] ==
                [seq for (seq, i) in drawn])
        # and about as far between each of them as was flown: an extra lap
        # of a loiter, or a leg flown twice, would not be
        drawn = dict(drawn)
        stages = [seq for (seq, i) in seen]
        total_drawn = self.length(track, drawn[stages[0]], drawn[stages[-1]])
        total_flown = self.length(flown, dict(seen)[stages[0]],
                                  dict(seen)[stages[-1]])
        assert total_drawn == pytest.approx(total_flown, rel=0.1)
        seen = dict(seen)
        for (a, b) in zip(stages, stages[1:]):
            flown_stage = self.length(flown, seen[a], seen[b])
            if flown_stage < 1000:
                continue
            assert self.length(track, drawn[a], drawn[b]) == pytest.approx(
                flown_stage, rel=0.2), (a, b)

    def test_the_path_is_the_one_flown(self):
        data = self.fixture()
        items = [(c, la, lo, a, tuple(p)) for (c, la, lo, a, p) in data['items']]
        track = plane_track.mission_track(tuple(data['home']), items,
                                          data['params'])
        assert track is not None
        distances = self.distances(track, data['flown'])
        median = distances[len(distances) // 2]
        p90 = distances[int(len(distances) * 0.9)]
        # the tangent-and-circle drawing this replaced was 6m out at the
        # median and 22m at the 90th percentile over this flight
        assert median < 2.0
        assert p90 < 8.0
        # a QuadPlane climbs out of its VTOL takeoff and transitions on the
        # way to the first loiter, which is not modelled; from the first
        # loiter on the path stays close all the way round
        assert distances[-1] < 80.0
        after_transition = self.distances(track, data['flown'][30:])
        assert after_transition[-1] < 30.0
