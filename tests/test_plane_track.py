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


class TestWaypointMaxRadius(object):
    """WP_MAX_RADIUS: a waypoint is not reached until the aircraft is that
    close to it, however far past it has flown (Plane::verify_nav_wp)"""

    def waypoint(self, north, east, passby=0):
        (lat, lon, amsl) = offset(north, east)
        return (mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, amsl, (0, 0, passby, 0))

    def corner(self, passby=0):
        '''a right-angle turn at 1000 north, 600 east'''
        return [self.waypoint(1000, 0), self.waypoint(1000, 600, passby),
                self.waypoint(0, 600)]

    def closest(self, track, north, east):
        (lat, lon, _) = offset(north, east)
        distances = [mp_util.gps_distance(p[0], p[1], lat, lon) for p in track]
        return (min(distances), distances.index(min(distances)))

    def test_the_corner_is_not_cut(self):
        # turning early, the aircraft passes wide of the corner
        (wide, _) = self.closest(fly(self.corner()), 1000, 600)
        assert wide > 20.0
        track = fly(self.corner(), dict(PARAMS, WP_MAX_RADIUS=20))
        (near, _) = self.closest(track, 1000, 600)
        assert near <= 20.0
        # and the mission goes on from there
        assert self.closest(track, 0, 600)[0] <= PARAMS['WP_RADIUS']

    def test_one_it_cannot_turn_tightly_enough_for_is_not_drawn(self):
        # a waypoint 100m to the side of one the aircraft arrives at going
        # the other way, with a turn some 85m across: held within 60m it
        # is reached, and within 30m it is circled for ever, as ArduPlane
        # warns, which leaves no path to draw
        params = dict(PARAMS, ROLL_LIMIT_DEG=30.0)
        items = [self.waypoint(1000, 0), self.waypoint(1000, 100),
                 self.waypoint(2000, 100)]
        track = fly(items, dict(params, WP_MAX_RADIUS=60))
        assert self.closest(track, 1000, 100)[0] <= 60.0
        assert plane_track.mission_track(
            HOME, items, dict(params, WP_MAX_RADIUS=30)) is None

    def test_a_pass_by_waypoint_is_overflown(self):
        # the finish line is past the pass-by point, so the aircraft is
        # never taken back to the waypoint: "it will overfly badly", as
        # ArduPlane has it
        params = dict(PARAMS, WP_MAX_RADIUS=30)
        items = [self.waypoint(1000, 0), self.waypoint(1000, 600, passby=100),
                 self.waypoint(1000, 0)]
        assert plane_track.mission_track(HOME, items, params) is None
        assert plane_track.mission_track(HOME, items, PARAMS) is not None

    def test_only_waypoints_are_held_to_it(self):
        # a landing passed wide of the radius is still done with
        (lat, lon, amsl) = offset(1200, 0, 0)
        items = [loiter_turns(1000, 0, 1, 80),
                 (mavlink.MAV_CMD_NAV_LAND, lat, lon, amsl, (0, 0, 0, 0))]
        track = fly(items)
        assert self.closest(track, 1200, 0)[0] > 1.0
        assert fly(items, dict(PARAMS, WP_MAX_RADIUS=1)) == track
        # as are loiters
        items = [loiter_turns(1000, 0, 1, 80), loiter_turns(1000, 600, 1, 80)]
        assert fly(items, dict(PARAMS, WP_MAX_RADIUS=1)) == fly(items)


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

    def test_a_return_to_launch_joins_its_circle(self):
        # ArduPlane circles from the start of an RTL, so the aircraft turns
        # onto the circle rather than flying over its middle first
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))

        def closest_on_return(params):
            distances = [math.hypot(*local(p))
                         for p in fly([waypoint(3000, 0), rtl], params=params)]
            return min(distances[distances.index(max(distances)):])
        for (params, radius) in ((dict(PARAMS, RTL_RADIUS=250), 250.0),
                                 (PARAMS, PARAMS['WP_LOITER_RAD'])):
            assert closest_on_return(params) > radius * 0.9
        # a plane given the parameter which only a QuadPlane takes is still
        # one of those
        assert closest_on_return(
            dict(PARAMS, RTL_RADIUS=250, Q_RTL_MODE=1)) > 225.0

    def test_a_quadplane_returns_to_land(self):
        '''QRTL, which Q_RTL_MODE 1 switches to on the way home and 3 goes
        to at once, lands where it returns to rather than circling'''
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        for (mode, alt) in ((1, 100.0), (3, 15.0)):
            params = dict(PARAMS, RTL_RADIUS=250, Q_ENABLE=1, Q_RTL_MODE=mode)
            track = fly([waypoint(3000, 0), rtl], params=params)
            assert math.hypot(*local(track[-1])) < 5.0
            # QRTL comes home at Q_RTL_ALT, and RTL at RTL_ALTITUDE
            assert track[-1][2] == pytest.approx(HOME[2] + alt, abs=10.0)
            # and it never went round
            assert all(local(p)[0] > -20.0 for p in track)

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

    def test_a_long_loiter_is_flown_through(self):
        # its turns are not taken for being stuck
        for (turns, radius) in ((100, 80), (40, 200)):
            track = fly([loiter_turns(3000, 0, turns, radius),
                         waypoint(3000, 3000)])
            (lat, lon, _) = offset(3000, 3000)
            assert mp_util.gps_distance(track[-1][0], track[-1][1],
                                        lat, lon) < 100.0
            circled = sum(mp_util.gps_distance(a[0], a[1], b[0], b[1])
                          for (a, b) in zip(track, track[1:]))
            assert circled > turns * 2 * math.pi * radius
        # nor is a long climb, longer than the leg to it and twenty laps
        track = fly([loiter_to_alt(500, 0, 6000, radius=80),
                     waypoint(3000, 3000, 6000)])
        assert max(p[2] for p in track) >= HOME[2] + 5990

    def test_a_jump_which_cannot_be_counted_is_not_flown(self):
        items = [waypoint(2000, 0),
                 (mavlink.MAV_CMD_DO_JUMP, 0, 0, None,
                  (float('nan'), 1, 0, 0)),
                 waypoint(0, 2000)]
        assert plane_track.mission_track(HOME, items, PARAMS) is None

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


class TestDrawnTrack(object):
    """where the maps get the path from, and what they do with it"""

    def items(self):
        '''a mission in the form the wp module's loader holds it: home, a
        speed change with no position, and two waypoints'''
        from types import SimpleNamespace
        wpoints = []
        rows = [(mavlink.MAV_CMD_NAV_WAYPOINT, offset(0, 0, 0)),
                (mavlink.MAV_CMD_DO_CHANGE_SPEED, (0, 0, 0)),
                (mavlink.MAV_CMD_NAV_WAYPOINT, offset(3000, 0)),
                (mavlink.MAV_CMD_NAV_WAYPOINT, offset(3000, 3000))]
        for (seq, (command, (lat, lon, amsl))) in enumerate(rows):
            params = (0, 30, -1, 0) if command == mavlink.MAV_CMD_DO_CHANGE_SPEED else (0, 0, 0, 0)
            wpoints.append(SimpleNamespace(
                seq=seq, command=command, x=lat, y=lon,
                z=0 if seq == 0 else amsl - HOME[2], frame=0 if seq == 0 else 3,
                param1=params[0], param2=params[1], param3=params[2],
                param4=params[3]))
        wpoints[0].z = HOME[2]
        return wpoints

    def live_module(self, vehicle, params=PARAMS, settle=True):
        '''the live map3d module, with only what send_mission() reads'''
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import Map3DModule
        module = Map3DModule.__new__(Map3DModule)
        wpoints = self.items()
        loader = SimpleNamespace(wpoints=wpoints, wp=lambda i: wpoints[i])
        module.mpstate = SimpleNamespace(
            mav_param=dict(params), vehicle_type=vehicle,
            module=lambda name: SimpleNamespace(wploader=loader))
        module.sent = []
        module.map = SimpleNamespace(
            set_mission=lambda items, track=None: module.sent.append(track))
        module.home_amsl = HOME[2]
        module.home_position = None
        module.reset_flown_track()
        module.map3d_settings = SimpleNamespace(missionpath='flown')
        module.armed = False
        module.ground_heading = None
        module.ground_heading_changed = False
        module.ground_heading_redrawn = 0
        module.default_circle_radius = lambda: 80.0
        module.terrain_alt = lambda lat, lon: None
        if settle:
            # the path is flown on a thread of its own, and drawn from the
            # idle task: wait for it and draw it, wherever the module sends
            # the mission, so these tests can look at what is drawn
            send_mission = module.send_mission

            def send_and_draw():
                send_mission()
                self.settle(module)
            module.send_mission = send_and_draw
        return module

    @staticmethod
    def settle(module, timeout=10.0):
        '''wait for the module's thread to fly the mission, and draw it'''
        import time
        deadline = time.time() + timeout
        while module.track_thread is not None:
            assert time.time() < deadline, 'the mission was never flown'
            time.sleep(0.001)
        module.draw_flown_track()

    def test_the_live_map_flies_a_planes_mission(self):
        module = self.live_module('plane')
        module.send_mission()
        track = module.sent[-1]
        assert track is not None
        assert track[0][:2] == pytest.approx(HOME[:2])
        # the speed change with no position of its own was flown too: at 30
        # m/s rather than 22 the turn at the first waypoint swings wider
        slow = self.live_module('plane', dict(PARAMS, AIRSPEED_CRUISE=22.0))
        slow.mpstate.module('wp').wploader.wpoints[1].param2 = -1
        slow.send_mission()
        assert (max(local(p)[0] for p in track) >
                max(local(p)[0] for p in slow.sent[-1]) + 5.0)

    def test_the_live_map_does_not_fly_the_mission_again_as_home_wanders(
            self, monkeypatch):
        # ArduPlane keeps setting home from the GPS every few seconds before
        # it arms, so home wanders by a metre or two each time
        calls = []
        real = plane_track.mission_track

        def counting(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', counting)
        module = self.live_module('plane')
        # a takeoff with no position of its own is flown from home, so it
        # wanders with it
        loader = module.mpstate.module('wp').wploader
        loader.wpoints[1].command = mavlink.MAV_CMD_NAV_TAKEOFF
        loader.wpoints[1].z = 30.0
        module.home_position = HOME[:2]
        module.send_mission()
        assert len(calls) == 1
        flown = module.sent[-1]
        (lat, lon) = mp_util.gps_newpos(HOME[0], HOME[1], 45, 3)
        module.home_position = (lat, lon)
        module.home_amsl = HOME[2] + 2
        module.send_mission()
        assert len(calls) == 1
        # the path it was flown with is moved to start at the new home, and
        # up with it, but still meets the waypoints, which have not moved
        moved = module.sent[-1]
        assert moved[0][0] == pytest.approx(lat, abs=1e-7)
        assert moved[0][1] == pytest.approx(lon, abs=1e-7)
        assert moved[0][2] == pytest.approx(flown[0][2] + 2)
        far = [i for (i, p) in enumerate(flown) if local(p)[1] > 1000][0]
        assert moved[far][:2] == pytest.approx(flown[far][:2], abs=1e-7)
        assert moved[far][2] == pytest.approx(flown[far][2] + 2)
        # but it moving further is a different flight
        (lat, lon) = mp_util.gps_newpos(HOME[0], HOME[1], 45, 50)
        module.home_position = (lat, lon)
        module.send_mission()
        assert len(calls) == 2
        module.home_amsl = HOME[2] + 30
        module.send_mission()
        assert len(calls) == 3

    def test_an_amsl_mission_is_not_flown_again_as_homes_altitude_wanders(
            self, monkeypatch):
        # an item at an altitude of its own does not move when home does,
        # so the mission is the same mission as home's altitude wanders
        calls = []
        real = plane_track.mission_track

        def counting(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', counting)
        module = self.live_module('plane')
        loader = module.mpstate.module('wp').wploader
        for w in loader.wpoints[1:]:
            if w.frame == 3:
                w.z += HOME[2]
                w.frame = 0
        module.home_position = HOME[:2]
        module.send_mission()
        assert len(calls) == 1
        for altitude in (0.3, -0.4, 0.2):
            module.home_amsl += altitude
            module.send_mission()
        assert len(calls) == 1
        # and the path it was drawn with stays where it was flown
        flown = [track for track in module.sent if track is not None][0]
        assert module.sent[-1][0][2] == pytest.approx(flown[0][2])
        # while a mission above home moves up with it
        relative = self.live_module('plane')
        relative.home_position = HOME[:2]
        relative.send_mission()
        moved_from = relative.sent[-1][0][2]
        relative.home_amsl += 0.3
        relative.send_mission()
        assert relative.sent[-1][0][2] == pytest.approx(moved_from + 0.3)
        assert len(calls) == 2

    def counted_flights(self, monkeypatch):
        calls = []
        real = plane_track.mission_track

        def counting(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', counting)
        return calls

    def test_a_mission_at_both_kinds_of_altitude(self, monkeypatch):
        # a path with some altitudes moving with home and some not cannot be
        # moved to fit a home which has: it is moved all the same while home
        # stays near, and flown again once home has gone further
        calls = self.counted_flights(monkeypatch)
        module = self.live_module('plane')
        loader = module.mpstate.module('wp').wploader
        # the last waypoint at an altitude of its own, the rest above home
        loader.wpoints[-1].z += HOME[2]
        loader.wpoints[-1].frame = 0
        module.home_position = HOME[:2]
        module.send_mission()
        assert len(calls) == 1
        flown = module.sent[-1]
        # ArduPlane sends home every few seconds before it arms, a few
        # centimetres different each time
        for altitude in (0.03, -0.02, 0.04):
            module.home_amsl += altitude
            module.send_mission()
        assert len(calls) == 1
        assert module.sent[-1][-1][2] == pytest.approx(flown[-1][2], abs=0.1)
        # further, and it is flown again, with the item with an altitude of
        # its own still drawn at it, rather than moved up with home
        module.home_amsl += 30.0
        module.send_mission()
        assert len(calls) == 2
        assert module.sent[-1][-1][2] == pytest.approx(flown[-1][2], abs=5.0)

    def test_an_item_with_no_altitude_to_fly_at_does_not_move_with_home(
            self, monkeypatch):
        # a speed change is at an altitude only because every item carries
        # one, and one with no position is flown wherever the aircraft is;
        # whatever the frame, neither makes the mission move with home
        speed = mavlink.MAV_CMD_DO_CHANGE_SPEED
        for (command, frame, z) in ((speed, 3, 0), (speed, 3, 50),
                                    (speed, 10, 0), (speed, 0, 0),
                                    (mavlink.MAV_CMD_NAV_LOITER_UNLIM, 3, 0),
                                    (mavlink.MAV_CMD_NAV_LOITER_UNLIM, 10, 50)):
            calls = self.counted_flights(monkeypatch)
            module = self.live_module('plane')
            loader = module.mpstate.module('wp').wploader
            for w in loader.wpoints[1:]:
                if w.command == speed:
                    (w.command, w.frame, w.z) = (command, frame, z)
                    w.param1 = w.param2 = w.param3 = w.param4 = 0
                else:
                    (w.frame, w.z) = (0, w.z + HOME[2])
            module.home_position = HOME[:2]
            for i in range(4):
                module.home_amsl = HOME[2] + 5.0 * (i % 2)
                module.send_mission()
            assert len(calls) == 1, (command, frame, z)
            # nor is the path drawn moved up and down with home
            flown = [track for track in module.sent if track is not None][0]
            assert module.home_amsl == HOME[2] + 5.0
            assert ([p[2] for p in module.sent[-1]] ==
                    pytest.approx([p[2] for p in flown])), (command, frame, z)
            monkeypatch.undo()

    def test_an_item_flown_above_home_moves_with_it(self, monkeypatch):
        # a loiter with no position of its own, above home, is flown at an
        # altitude which moves with home, among waypoints which do not
        calls = self.counted_flights(monkeypatch)
        module = self.live_module('plane')
        loader = module.mpstate.module('wp').wploader
        for w in loader.wpoints[1:]:
            if w.command == mavlink.MAV_CMD_DO_CHANGE_SPEED:
                (w.command, w.frame, w.z) = (
                    mavlink.MAV_CMD_NAV_LOITER_TURNS, 3, 50)
                (w.param1, w.param2, w.param3, w.param4) = (1, 0, 0, 0)
            else:
                (w.frame, w.z) = (0, w.z + HOME[2])
        module.home_position = HOME[:2]
        module.send_mission()
        module.home_amsl += 30.0
        module.send_mission()
        assert len(calls) == 2

    def test_the_live_map_takes_home_for_an_origin_it_has_not_been_told(self):
        from types import SimpleNamespace
        module = self.live_module('plane')
        frame_valid = 1 << 2
        rally = [SimpleNamespace(lat=int(HOME[0] * 1e7),
                                 lng=int(HOME[1] * 1e7), alt=100,
                                 flags=frame_valid | (2 << 3))]
        module.mpstate.module = lambda name: SimpleNamespace(
            rallyloader=SimpleNamespace(rally_count=lambda: 1,
                                        rally_point=lambda i: rally[0]))
        # ArduPilot logs the EKF origin and home together, and MAVProxy
        # keeps only the last of each type, so the origin is often unknown:
        # home stands in for it, and moves the point as home moves
        assert module.origin_amsl is None
        (point,) = module.rally_points((HOME[0], HOME[1], HOME[2]))
        assert point[2] == HOME[2] + 100
        assert point[3] is False

    def test_the_live_map_takes_a_rally_points_own_altitude_frame(self):
        from types import SimpleNamespace
        module = self.live_module('plane')
        frame_valid = 1 << 2
        rally = [SimpleNamespace(lat=int(HOME[0] * 1e7),
                                 lng=int(HOME[1] * 1e7), alt=100,
                                 flags=frame_valid | (frame << 3))
                 for frame in (1, 2, 3)]
        module.mpstate.module = lambda name: SimpleNamespace(
            wploader=module.mpstate.module('wp').wploader
            if name == 'wp' else None,
            rallyloader=SimpleNamespace(
                rally_count=lambda: len(rally),
                rally_point=lambda i: rally[i]))
        module.origin_amsl = 500.0
        module.terrain_alt = lambda lat, lon: 300.0
        points = module.rally_points((HOME[0], HOME[1], HOME[2]))
        assert [p[2] for p in points] == [HOME[2] + 100, 600.0, 400.0]
        # only the one above home moves when home does
        assert [p[3] for p in points] == [False, True, True]

    def test_an_item_with_no_position_keeps_its_altitude(self, monkeypatch):
        flown = []
        real = plane_track.mission_track

        def recording(home, items, *args, **kwargs):
            flown.append(items)
            return real(home, items, *args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', recording)
        module = self.live_module('plane')
        loader = module.mpstate.module('wp').wploader
        # a loiter to altitude where the aircraft is, 200m above home
        loader.wpoints[1].command = mavlink.MAV_CMD_NAV_LOITER_TO_ALT
        loader.wpoints[1].z = 200.0
        loader.wpoints[1].param2 = 0
        module.send_mission()
        assert flown[-1][0][3] == pytest.approx(HOME[2] + 200.0)

    def test_the_mission_is_flown_off_the_main_thread(self, monkeypatch):
        import threading
        release = threading.Event()
        flown_on = []
        real = plane_track.mission_track

        def slow(*args, **kwargs):
            flown_on.append(threading.current_thread())
            assert release.wait(10)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', slow)
        module = self.live_module('plane', settle=False)
        module.send_mission()
        # the mission is drawn at once, while its path is still being flown
        assert module.sent == [None]
        module.draw_flown_track()
        assert module.sent == [None]
        # sent again as it is flown, it is not flown twice over
        import time
        deadline = time.time() + 10
        while not flown_on:
            assert time.time() < deadline
            time.sleep(0.001)
        module.send_mission()
        release.set()
        self.settle(module)
        assert module.sent[-1] is not None
        assert flown_on[0] is not threading.main_thread()
        assert len(flown_on) == 1
        # and drawn with it straight away the next time, without flying it
        module.send_mission()
        assert module.sent[-1] is not None
        assert len(flown_on) == 1

    def flying_slowly(self, monkeypatch):
        '''plane_track.mission_track made to wait for a release, and a list
        of the calls made to it'''
        import threading
        release = threading.Event()
        calls = []
        real = plane_track.mission_track

        def slow(*args, **kwargs):
            calls.append(args)
            assert release.wait(10)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', slow)
        return (release, calls)

    def wait_for(self, condition):
        import time
        deadline = time.time() + 10
        while not condition():
            assert time.time() < deadline
            time.sleep(0.001)

    def test_home_wandering_as_the_mission_is_flown(self, monkeypatch):
        (release, calls) = self.flying_slowly(monkeypatch)
        module = self.live_module('plane', settle=False)
        module.home_position = HOME[:2]
        module.send_mission()
        self.wait_for(lambda: calls)
        # home wanders a few metres while the path is being flown: the path
        # is not flown again, and is drawn from where home is now
        (lat, lon) = mp_util.gps_newpos(HOME[0], HOME[1], 45, 3)
        module.home_position = (lat, lon)
        module.send_mission()
        release.set()
        self.settle(module)
        assert len(calls) == 1
        assert module.sent[-1][0][:2] == pytest.approx((lat, lon), abs=1e-7)

    def test_home_wandering_far_as_the_mission_is_flown(self, monkeypatch):
        (release, calls) = self.flying_slowly(monkeypatch)
        module = self.live_module('plane', settle=False)
        module.home_position = HOME[:2]
        module.send_mission()
        self.wait_for(lambda: calls)
        # a few metres at a time, but in all more than ten from the home it
        # is being flown from: that is flown again
        for metres in (4, 8, 13):
            (lat, lon) = mp_util.gps_newpos(HOME[0], HOME[1], 45, metres)
            module.home_position = (lat, lon)
            module.send_mission()
        release.set()
        self.settle(module)
        assert len(calls) == 2
        assert module.sent[-1][0][:2] == pytest.approx((lat, lon), abs=1e-7)

    def test_the_idle_task_draws_the_path_flown(self):
        import time
        module = self.live_module('plane', settle=False)
        module.map.is_alive = lambda: True
        module.map.check_events = lambda: []
        wp_module = module.mpstate.module('wp')
        wp_module.wploader.last_change = 1.0
        module.mpstate.module = lambda name: wp_module
        module.wp_change_time = 1.0
        module.kml_change_state = module._kml_state(wp_module)
        module.fence_change_time = module.rally_change_time = 0
        module.terrain_resolved = False
        module.send_mission()
        assert module.sent == [None]
        deadline = time.time() + 10
        while module.track_thread is not None:
            assert time.time() < deadline
            time.sleep(0.001)
        module.idle_task()
        assert module.sent[-1] is not None

    def test_a_path_for_a_mission_since_changed_is_not_drawn(self, monkeypatch):
        import threading
        import time
        (first, second) = (threading.Event(), threading.Event())
        releases = [first, second]
        calls = []
        real = plane_track.mission_track

        def slow(*args, **kwargs):
            calls.append(args)
            assert releases.pop(0).wait(10)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', slow)
        module = self.live_module('plane', settle=False)
        loader = module.mpstate.module('wp').wploader
        module.send_mission()
        # the mission changes while the first is being flown, rather than
        # before the thread has taken it, which it would simply replace
        self.wait_for(lambda: calls)
        loader.wpoints[3].y += 0.01
        module.send_mission()
        assert module.sent == [None, None]
        first.set()
        deadline = time.time() + 10
        while module.track_result is None:
            assert time.time() < deadline
            time.sleep(0.001)
        # the first mission's path is not drawn over the second mission
        module.draw_flown_track()
        assert module.sent == [None, None]
        second.set()
        self.settle(module)
        (lat, lon, _) = module.sent[-1][-1]
        assert mp_util.gps_distance(lat, lon, loader.wpoints[3].x,
                                    loader.wpoints[3].y) < 150.0

    def test_the_live_map_returns_to_its_rally_points(self, monkeypatch):
        from types import SimpleNamespace
        module = self.live_module('plane')
        loader = module.mpstate.module('wp').wploader
        loader.wpoints[3].command = mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        loader.wpoints[3].x = loader.wpoints[3].y = 0.0
        (lat, lon, _) = offset(2500, 800, 0)
        rally = SimpleNamespace(rally_points=[])
        rally.rally_count = lambda: len(rally.rally_points)
        rally.rally_point = lambda i: rally.rally_points[i]
        modules = SimpleNamespace(wploader=loader, rallyloader=rally)
        module.mpstate.module = lambda name: modules
        module.send_mission()
        (end_lat, end_lon, end_amsl) = module.sent[-1][-1]
        assert mp_util.gps_distance(end_lat, end_lon, HOME[0], HOME[1]) < 150
        # a rally point 150m above home, nearer than home is
        rally.rally_points.append(SimpleNamespace(
            lat=int(lat * 1e7), lng=int(lon * 1e7), alt=150, flags=0))
        module.send_mission()
        (end_lat, end_lon, end_amsl) = module.sent[-1][-1]
        assert mp_util.gps_distance(end_lat, end_lon, lat, lon) < 150
        assert end_amsl == pytest.approx(HOME[2] + 150, abs=15)

    def test_a_mission_which_will_not_fly_does_not_stop_the_next(self, monkeypatch):
        real = plane_track.mission_track
        failures = [RuntimeError('bad mission')]

        def failing(*args, **kwargs):
            if failures:
                raise failures.pop()
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', failing)
        module = self.live_module('plane')
        module.send_mission()
        assert module.sent[-1] is None
        loader = module.mpstate.module('wp').wploader
        loader.wpoints[3].y += 0.01
        module.send_mission()
        assert module.sent[-1] is not None

    def test_the_path_across_the_antimeridian_is_drawn_with_its_mission(self):
        pytest.importorskip("vtk")
        import vtk
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        home = (10.0, 179.998, 0.0)
        east = mp_util.gps_newpos(home[0], home[1], 90, 300)
        track = plane_track.mission_track(
            home, [(mavlink.MAV_CMD_NAV_WAYPOINT, east[0], east[1], 100.0,
                    (0, 0, 0, 0))], PARAMS)
        items = [MissionItem(home[0], home[1], 0.0, 0,
                             mavlink.MAV_CMD_NAV_WAYPOINT, 0),
                 MissionItem(east[0], east[1], 100.0, 0,
                             mavlink.MAV_CMD_NAV_WAYPOINT, 1)]
        for origin in (home, (east[0], east[1])):
            em = ElementManager(vtk.vtkRenderer(), origin[0], origin[1], 1.0)
            em.set_mission(items, track)
            (end, marker) = (em.mission_line[-1], em.mission_markers[-1])
            assert math.hypot(end[0] - marker[0], end[1] - marker[1]) < 100.0

    def test_the_live_map_leaves_other_vehicles_to_the_items(self):
        module = self.live_module('copter')
        module.send_mission()
        assert module.sent[-1] is None

    def test_an_unchanged_mission_is_not_flown_again(self, monkeypatch):
        calls = []
        real = plane_track.mission_track

        def counting(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', counting)
        module = self.live_module('plane')
        module.send_mission()
        module.send_mission()
        assert len(calls) == 1
        module.mpstate.mav_param['WP_RADIUS'] = 30.0
        module.send_mission()
        assert len(calls) == 2

    def test_the_live_map_takes_off_the_way_the_vehicle_points(self, monkeypatch):
        headings = []
        real = plane_track.mission_track

        def recording(home, items, params, heading=None, **kwargs):
            headings.append(heading)
            return real(home, items, params, heading, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', recording)
        module = self.live_module('plane')
        module.map.is_alive = lambda: True
        module.map.set_vehicle_type = lambda name: None
        module.icon_type = None
        mav = mavutil.mavlink

        def heartbeat(armed):
            return mav.MAVLink_heartbeat_message(
                mav.MAV_TYPE_FIXED_WING, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mav.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0, 0, 0, 3)

        def attitude(yaw):
            return mav.MAVLink_attitude_message(0, 0, 0, yaw, 0, 0, 0)

        def points(yaw, now):
            module.mavlink_packet(attitude(math.radians(yaw)))
            module.redraw_for_ground_heading(now)
        # on the ground, pointing east: the best guess there is at the course
        # it will take off on, and the mission is flown again from it without
        # waiting for anything else to change
        module.mavlink_packet(heartbeat(False))
        points(90, 100.0)
        assert headings == [90]
        # a degree or two of wander is not worth flying it again for
        points(92, 200.0)
        assert headings == [90]
        # turning round is, though not more often than every couple of seconds
        points(180, 200.5)
        assert headings == [90, 180]
        points(270, 201.0)
        assert headings == [90, 180]
        module.redraw_for_ground_heading(203.0)
        assert headings == [90, 180, 270]
        # once it is armed and flying, where it points is not the takeoff's,
        # and a ground station's heartbeat says nothing about the vehicle
        module.mavlink_packet(heartbeat(True))
        module.mavlink_packet(mav.MAVLink_heartbeat_message(
            mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
        points(0, 300.0)
        module.send_mission()
        assert headings == [90, 180, 270]

    def test_the_live_map_redraws_for_a_new_heading_on_its_own(self, monkeypatch):
        # nothing else has to happen for the module's idle task to do it
        headings = []
        real = plane_track.mission_track

        def recording(home, items, params, heading=None, **kwargs):
            headings.append(heading)
            return real(home, items, params, heading, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', recording)
        module = self.live_module('plane')
        module.send_mission()
        assert headings == [None]
        module.map.is_alive = lambda: True
        module.map.check_events = lambda: []
        wp_module = module.mpstate.module('wp')
        wp_module.wploader.last_change = 1.0
        module.mpstate.module = lambda name: wp_module
        module.wp_change_time = 1.0
        module.kml_change_state = module._kml_state(wp_module)
        module.fence_change_time = module.rally_change_time = 0
        module.terrain_resolved = False
        module.note_ground_heading(math.radians(45))
        module.idle_task()
        assert headings == [None, 45]

    def explorer(self):
        pytest.importorskip("wx")
        pytest.importorskip("lxml")
        import importlib.util
        path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                            'MAVProxy', 'tools', 'MAVExplorer.py')
        spec = importlib.util.spec_from_file_location('mavexplorer', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def log_mission(self, with_home=True):
        '''a log's CMD entries, as mission_from_log() keeps them'''
        cmds = {}
        for w in self.items():
            if w.seq == 0 and not with_home:
                cmds[0] = (0.0, 0.0, 0.0, 0, w.command, 0, (0, 0, 0, 0))
                continue
            cmds[w.seq] = (w.x, w.y, w.z, w.frame, w.command, w.seq,
                           (w.param1, w.param2, w.param3, w.param4))
        return cmds

    def test_mavexplorer_flies_a_planes_mission(self):
        mx = self.explorer()
        cmds = self.log_mission()
        mission = mx.resolve_mission_amsl(
            mx.mission_items_from_cmds(cmds), HOME[2], PARAMS,
            mavlink.MAV_TYPE_FIXED_WING)
        track = mx.plane_mission_track(cmds, mission, None, PARAMS,
                                       mavlink.MAV_TYPE_FIXED_WING)
        assert track is not None
        assert track[0][:2] == pytest.approx(HOME[:2])
        # the speed change is in the CMDs but not among the items drawn
        slow = dict(cmds)
        slow[1] = slow[1][:6] + ((0, -1, -1, 0),)
        slow = mx.plane_mission_track(slow, mission, None, PARAMS,
                                      mavlink.MAV_TYPE_FIXED_WING)
        assert (max(local(p)[0] for p in track) >
                max(local(p)[0] for p in slow) + 5.0)
        assert mx.plane_mission_track(cmds, mission, None, PARAMS,
                                      mavlink.MAV_TYPE_QUADROTOR) is None

    def test_mavexplorer_draws_no_path_for_a_mission_with_an_item_missing(self):
        # jumps, and where each item began, go by sequence number, which a
        # gap would put out by one for every item after it
        mx = self.explorer()
        cmds = self.log_mission()
        mission = mx.resolve_mission_amsl(
            mx.mission_items_from_cmds(cmds), HOME[2], PARAMS,
            mavlink.MAV_TYPE_FIXED_WING)
        assert mx.plane_mission_track(cmds, mission, None, PARAMS,
                                      mavlink.MAV_TYPE_FIXED_WING) is not None
        del cmds[1]
        mission = mx.resolve_mission_amsl(
            mx.mission_items_from_cmds(cmds), HOME[2], PARAMS,
            mavlink.MAV_TYPE_FIXED_WING)
        assert mx.plane_mission_track(cmds, mission, None, PARAMS,
                                      mavlink.MAV_TYPE_FIXED_WING) is None

    def test_mavexplorer_keeps_the_altitude_of_an_item_with_no_position(
            self, monkeypatch):
        flown = []
        real = plane_track.mission_track

        def recording(home, items, *args, **kwargs):
            flown.append(items)
            return real(home, items, *args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', recording)
        mx = self.explorer()
        cmds = self.log_mission()
        cmds[1] = (0.0, 0.0, 200.0, 3, mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 1,
                   (0, 0, 0, 0))
        mission = mx.resolve_mission_amsl(
            mx.mission_items_from_cmds(cmds), HOME[2], PARAMS,
            mavlink.MAV_TYPE_FIXED_WING)
        mx.plane_mission_track(cmds, mission, None, PARAMS,
                               mavlink.MAV_TYPE_FIXED_WING)
        assert flown[-1][0][3] == pytest.approx(HOME[2] + 200.0)

    def test_mavexplorer_starts_where_the_flight_did_with_no_home(self):
        mx = self.explorer()
        cmds = self.log_mission(with_home=False)
        started = offset(-500, -500, 0)
        mission = mx.resolve_mission_amsl(
            mx.mission_items_from_cmds(cmds, started_at=started[:2]),
            HOME[2], PARAMS, mavlink.MAV_TYPE_FIXED_WING)
        track = mx.plane_mission_track(cmds, mission, started, PARAMS,
                                       mavlink.MAV_TYPE_FIXED_WING)
        assert track is not None
        assert track[0][:2] == pytest.approx(started[:2])

    def log(self, *messages):
        '''a dataflash log handing back these messages in order'''
        queue = list(messages)

        class Log(object):
            def recv_match(self, type=None, condition=None):
                while queue:
                    m = queue.pop(0)
                    if type is None or m.get_type() in type:
                        return m
                return None
        return Log()

    def message(self, kind, **fields):
        from types import SimpleNamespace
        m = SimpleNamespace(_timestamp=0, **fields)
        m.get_type = lambda: kind
        return m

    def cmd(self, seq, command, lat, lon, alt):
        return self.message('CMD', CNum=seq, CId=command, Lat=lat, Lng=lon,
                            Alt=alt, Frame=3, Prm1=0, Prm2=0, Prm3=0, Prm4=0)

    def pos(self, north, east, alt=0.0):
        (lat, lon, amsl) = offset(north, east, alt)
        return self.message('POS', Lat=lat, Lng=lon, Alt=amsl)

    def mission_dump(self, new_mission=True):
        (lat, lon, _) = offset(3000, 0)
        return (([self.message('MSG', Message='New mission')] if new_mission
                 else []) +
                [self.cmd(0, mavlink.MAV_CMD_NAV_WAYPOINT, HOME[0], HOME[1], HOME[2]),
                 self.cmd(1, mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
                 self.cmd(2, mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, 100)])

    def test_mavexplorer_takes_the_takeoff_course_from_the_log(self):
        mx = self.explorer()
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            # taxied north a little before the mission started
            self.pos(10, 0), self.message('MSG', Message='Mission: 1 Takeoff'),
            self.pos(10, 10), self.pos(10, 25), self.pos(10, 45),
            self.pos(10, 80)]))
        (path, mission, cmds, started, _, _) = mx.mission_from_log(log)
        # east, from where the takeoff began, not from where the log did
        assert mx.takeoff_course(path, cmds, started) == pytest.approx(90.0, abs=1.0)
        assert mx.takeoff_course(path, cmds, {}) is None

    def test_mavexplorer_takes_the_takeoff_course_of_the_last_mission(self):
        # a log from before the logger wrote "New mission": the first item
        # of a mission written out again still starts it afresh, and where
        # the items of the mission before began has to go with it
        mx = self.explorer()
        log = self.log(*(
            [self.pos(0, 0)] + self.mission_dump(new_mission=False) +
            [self.message('MSG', Message='Mission: 1 Takeoff'),
             self.pos(0, 20), self.pos(0, 50)] +
            self.mission_dump(new_mission=False) +
            [self.pos(0, 60), self.message('MSG', Message='Mission: 1 Takeoff'),
             self.pos(40, 60), self.pos(80, 60)]))
        (path, mission, cmds, started, _, _) = mx.mission_from_log(log)
        course = mx.takeoff_course(path, cmds, started)
        # north, the second takeoff, not east, the first
        assert mp_util.wrap_180(course) == pytest.approx(0.0, abs=1.0)

    def test_mavexplorer_returns_to_the_rally_points_the_log_ends_with(self):
        mx = self.explorer()
        (lat, lon, _) = offset(2500, 800, 0)
        (old_lat, old_lon, _) = offset(-2000, 0, 0)

        def raly(seq, total, lat, lon, alt, **fields):
            return self.message('RALY', Tot=total, Seq=seq, Lat=lat, Lng=lon,
                                Alt=alt, **fields)
        dump = self.mission_dump()
        # home as the logger writes it, in absolute altitude
        dump[1].Frame = 0
        dump[-1] = self.cmd(2, mavlink.MAV_CMD_NAV_WAYPOINT,
                            *offset(3000, 0)[:2], 100)
        dump.append(self.cmd(3, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))
        log = self.log(*([self.pos(0, 0)] + dump + [
            raly(0, 2, old_lat, old_lon, 80, Flags=0),
            raly(1, 2, old_lat, old_lon, 80, Flags=0),
            # written out again, as one point
            raly(0, 1, lat, lon, 150, Flags=0)]))
        (path, mission, cmds, started, rally, _) = mx.mission_from_log(log)
        assert rally == [(lat, lon, 150, 0)]
        mission = mx.resolve_mission_amsl(mission, HOME[2], PARAMS,
                                          mavlink.MAV_TYPE_FIXED_WING)
        track = mx.plane_mission_track(cmds, mission, None, PARAMS,
                                       mavlink.MAV_TYPE_FIXED_WING,
                                       rally=rally)
        (end_lat, end_lon, end_amsl) = track[-1]
        assert mp_util.gps_distance(end_lat, end_lon, lat, lon) < 150
        assert end_amsl == pytest.approx(HOME[2] + 150, abs=15)

    def test_mavexplorer_keeps_rally_points_a_new_one_is_appended_to(self):
        """AP_Rally::append() logs the new point alone, with the new total"""
        mx = self.explorer()
        (lat, lon, _) = offset(2500, 800, 0)
        (other_lat, other_lon, _) = offset(-2000, 0, 0)
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            self.message('RALY', Tot=1, Seq=0, Lat=other_lat, Lng=other_lon,
                         Alt=80, Flags=0),
            self.message('RALY', Tot=2, Seq=1, Lat=lat, Lng=lon, Alt=150,
                         Flags=0)]))
        (path, mission, cmds, started, rally, _) = mx.mission_from_log(log)
        assert rally == [(other_lat, other_lon, 80, 0), (lat, lon, 150, 0)]
        # while a smaller table drops the points it no longer has
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            self.message('RALY', Tot=2, Seq=0, Lat=other_lat, Lng=other_lon,
                         Alt=80, Flags=0),
            self.message('RALY', Tot=2, Seq=1, Lat=lat, Lng=lon, Alt=150,
                         Flags=0),
            self.message('RALY', Tot=1, Seq=0, Lat=lat, Lng=lon, Alt=90,
                         Flags=0)]))
        (path, mission, cmds, started, rally, _) = mx.mission_from_log(log)
        assert rally == [(lat, lon, 90, 0)]

    def test_mavexplorer_waits_only_so_long_for_rally_terrain(
            self, monkeypatch):
        mx = self.explorer()
        from MAVProxy.modules.mavproxy_map3d import terrain
        asked = []

        def sample(lat, lon, **kwargs):
            asked.append(kwargs)
            return 300.0
        monkeypatch.setattr(terrain, 'sample_terrain', sample)
        frame_valid = 1 << 2
        amsl = mx.rally_point_amsl(HOME[0], HOME[1], 100,
                                   frame_valid | (3 << 3), HOME[2])
        assert amsl == 400.0
        assert asked == [{'timeout': mx.RALLY_TERRAIN_TIMEOUT}]
        assert 0 < mx.RALLY_TERRAIN_TIMEOUT <= 10.0
        # and an origin the log does not give falls back to home
        assert mx.rally_point_amsl(HOME[0], HOME[1], 100,
                                   frame_valid | (2 << 3),
                                   HOME[2]) == HOME[2] + 100

    def test_mavexplorer_forgets_rally_points_which_were_cleared(self):
        """the logger writes "New rally" before the whole table, even an
        empty one, which is all a cleared table is logged as"""
        mx = self.explorer()
        (lat, lon, _) = offset(2500, 800, 0)
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            self.message('RALY', Tot=1, Seq=0, Lat=lat, Lng=lon, Alt=150,
                         Flags=0),
            self.message('MSG', Message='New rally')]))
        (path, mission, cmds, started, rally, _) = mx.mission_from_log(log)
        assert rally == []

    def test_mavexplorer_measures_a_rally_point_from_the_ekf_origin(self):
        """ORGN type 0 is the EKF origin, type 1 home"""
        mx = self.explorer()
        (lat, lon, _) = offset(2500, 800, 0)
        frame_valid = 1 << 2
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            self.message('ORGN', Type=0, Lat=HOME[0], Lng=HOME[1],
                         Alt=HOME[2] - 50),
            self.message('ORGN', Type=1, Lat=HOME[0], Lng=HOME[1],
                         Alt=HOME[2]),
            self.message('RALY', Tot=1, Seq=0, Lat=lat, Lng=lon, Alt=120,
                         Flags=frame_valid | (2 << 3))]))
        (path, mission, cmds, started, rally, origin) = mx.mission_from_log(log)
        assert origin == (HOME[0], HOME[1], HOME[2] - 50)
        assert mx.rally_point_amsl(lat, lon, 120, frame_valid | (2 << 3),
                                   HOME[2], origin) == HOME[2] + 70

    def test_mavexplorer_takes_rally_points_from_an_older_log(self):
        """RALY had no Flags before 4.5, and one point set is logged alone"""
        mx = self.explorer()
        (lat, lon, _) = offset(2500, 800, 0)
        (other_lat, other_lon, _) = offset(-2000, 0, 0)

        def raly(seq, total, lat, lon, alt, **fields):
            return self.message('RALY', Tot=total, Seq=seq, Lat=lat, Lng=lon,
                                Alt=alt, **fields)
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            raly(0, 2, other_lat, other_lon, 80),
            raly(1, 2, lat, lon, 150),
            # each point set again on its own, which is logged alone: the
            # other points still stand, rather than being forgotten
            raly(1, 2, lat, lon, 160),
            raly(0, 2, other_lat, other_lon, 90)]))
        (path, mission, cmds, started, rally, _) = mx.mission_from_log(log)
        assert rally == [(other_lat, other_lon, 90, 0), (lat, lon, 160, 0)]

    def test_mavexplorer_starts_where_the_log_says_the_mission_did(self):
        mx = self.explorer()
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            # carried off to the east before the mission started
            self.pos(0, 600, 5), self.message('MSG', Message='Mission: 1 Takeoff'),
            self.pos(40, 600, 30), self.pos(80, 600, 60)]))
        (path, mission, cmds, started, _, _) = mx.mission_from_log(log)
        mission = mx.resolve_mission_amsl(mission, HOME[2], PARAMS,
                                          mavlink.MAV_TYPE_FIXED_WING)
        track = mx.plane_mission_track(cmds, mission, None, PARAMS,
                                       mavlink.MAV_TYPE_FIXED_WING,
                                       path, started)
        assert track[0] == pytest.approx(offset(0, 600, 5))
        # and takes off on the course it was flown on: north
        climbed = [p for p in track if p[2] >= HOME[2] + 49.5][0]
        assert local(climbed)[1] == pytest.approx(600.0, abs=5.0)
        # without the log to go on, it starts from home
        track = mx.plane_mission_track(cmds, mission, None, PARAMS,
                                       mavlink.MAV_TYPE_FIXED_WING)
        assert track[0][:2] == pytest.approx(HOME[:2])

    def test_mavexplorer_draws_the_path_from_where_the_log_started(self, monkeypatch):
        # the 3D map command itself, with a stand-in for the viewer
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import map3d
        mx = self.explorer()
        drawn = []

        class Viewer(object):
            def __init__(self, title=None):
                pass

            def set_mission(self, items, track=None):
                drawn.append(track)

            def __getattr__(self, name):
                return lambda *args, **kwargs: None
        monkeypatch.setattr(map3d, 'Map3D', Viewer)
        monkeypatch.setattr(map3d, 'missing_packages', lambda: [])
        log = self.log(*([self.pos(0, 0)] + self.mission_dump() + [
            self.pos(0, 600, 5), self.message('MSG', Message='Mission: 1 Takeoff'),
            self.pos(40, 600, 30), self.pos(80, 600, 60)]))
        log.rewind = lambda: None
        log.params = dict(PARAMS)
        log.mav_type = mavlink.MAV_TYPE_FIXED_WING
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(
            mlog=log, settings=SimpleNamespace(
                condition=None, showdirection=True, showlabels=False,
                labelsize=14, sync_xmap=False,
                missionpath='flown')),
            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [])
        mx.cmd_map3d([])
        (track,) = drawn
        assert track[0] == pytest.approx(offset(0, 600, 5))
        climbed = [p for p in track if p[2] >= HOME[2] + 49.5][0]
        assert local(climbed)[1] == pytest.approx(600.0, abs=5.0)

    def test_the_3d_map_draws_the_track_it_is_given(self):
        pytest.importorskip("vtk")
        import vtk
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        items = [MissionItem(p[0], p[1], p[2], 0, mavlink.MAV_CMD_NAV_WAYPOINT, i)
                 for (i, p) in enumerate((offset(0, 0), offset(3000, 0)))]
        track = [offset(0, 0), offset(1000, 50), offset(2000, 20), offset(3000, 0)]
        em = ElementManager(vtk.vtkRenderer(), HOME[0], HOME[1], 1.0)
        em.set_home(HOME[2])
        em.set_mission(items, track)
        assert em.mission_line == [em._enu(*p) for p in track]
        assert em.mission_markers == [em._enu(i.lat, i.lon, i.alt) for i in items]
        # and without one, the items are drawn as they always were
        em.set_mission(items)
        assert em.mission_line == [em._enu(i.lat, i.lon, i.alt) for i in items]


class TestMissionStyles(object):
    """the 3D map draws a mission as the path flown, as the geometry of its
    items, or plain: straight from item to item with a ring at each loiter"""

    RADIUS = 150.0

    def items(self):
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        (a, b, c) = (offset(0, 0), offset(2000, 0), offset(2000, 2000))
        return [
            MissionItem(a[0], a[1], a[2], 0, mavlink.MAV_CMD_NAV_WAYPOINT, 1),
            MissionItem(b[0], b[1], b[2], 0, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                        2, 1.0, self.RADIUS, 1.0),
            MissionItem(c[0], c[1], c[2], 0, mavlink.MAV_CMD_NAV_WAYPOINT, 3),
        ]

    def track(self):
        return [offset(0, 0), offset(1000, 30), offset(2000, 2000)]

    def elements(self):
        pytest.importorskip("vtk")
        import vtk
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        em = ElementManager(vtk.vtkRenderer(), HOME[0], HOME[1], 1.0)
        em.set_home(HOME[2])
        return em

    def distance_from_loiter(self, em, point):
        centre = em._enu(*offset(2000, 0))
        return math.hypot(point[0] - centre[0], point[1] - centre[1])

    def test_the_path_flown_is_drawn_by_default(self):
        em = self.elements()
        em.set_mission(self.items(), self.track())
        assert em.mission_line == [em._enu(*p) for p in self.track()]
        assert em.mission_rings == []

    def test_geometry(self):
        em = self.elements()
        em.set_mission(self.items(), self.track())
        em.set_mission_style('geometry')
        # round the circle rather than through its centre, with no ring
        assert em.mission_rings == []
        assert min(self.distance_from_loiter(em, p)
                   for p in em.mission_line) > self.RADIUS - 5.0
        # and a mission which comes with no path flown is drawn the same way
        # in the flown style
        geometry = em.mission_line
        em.set_mission_style('flown')
        em.set_mission(self.items())
        assert em.mission_line == geometry

    def test_plain(self):
        em = self.elements()
        items = self.items()
        em.set_mission(items, self.track())
        em.set_mission_style('plain')
        assert em.mission_line == [em._enu(i.lat, i.lon, i.alt) for i in items]
        (ring,) = em.mission_rings
        for point in ring:
            assert self.distance_from_loiter(em, point) == pytest.approx(
                self.RADIUS, abs=1.0)
            assert point[2] == pytest.approx(items[1].alt)
        # the ring is drawn as well as the line and the markers
        assert len(em.actors['mission']) == 3
        # a new mission is drawn in the style last asked for
        em.set_mission(items, self.track())
        assert len(em.mission_rings) == 1
        # and a style it does not know leaves it as it is
        em.set_mission_style('fancy')
        assert em.mission_style == 'plain'

    def test_the_viewer_is_told_the_style_and_the_mission(self, map3d_frame):
        # from the parent's Map3D, over the queue, to the child's frame and
        # its ElementManager: nothing but the queue stands in
        pytest.importorskip("wx")
        pytest.importorskip("vtk")
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d.map3d import Map3D
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
        sent = []
        viewer = Map3D.__new__(Map3D)
        viewer.child = SimpleNamespace(is_alive=lambda: True)
        viewer.object_queue = SimpleNamespace(put=sent.append)
        em = self.elements()
        frame = map3d_frame(em)
        items = self.items()
        viewer.set_mission_style('plain')
        viewer.set_mission(items, self.track())
        for msg in sent:
            Map3DFrame.handle(frame, msg)
        assert em.mission_style == 'plain'
        assert len(em.mission_rings) == 1
        sent[:] = []
        viewer.set_mission_style('flown')
        for msg in sent:
            Map3DFrame.handle(frame, msg)
        assert em.mission_line == [em._enu(*p) for p in self.track()]
        # and the map's own control shows the style it is drawing
        assert frame.style_choice.GetStringSelection() == 'flown'

    def live_module(self, style):
        module = TestDrawnTrack().live_module('plane')
        module.map3d_settings.missionpath = style
        return module

    def test_the_live_map_only_flies_the_mission_for_the_path_flown(self, monkeypatch):
        calls = []
        real = plane_track.mission_track

        def counting(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(plane_track, 'mission_track', counting)
        for style in ('geometry', 'plain'):
            module = self.live_module(style)
            module.send_mission()
            assert module.sent[-1] is None
        assert calls == []
        module = self.live_module('flown')
        module.send_mission()
        assert module.sent[-1] is not None

    def test_the_live_map_setting(self):
        from types import SimpleNamespace
        from MAVProxy.modules.lib import mp_settings
        from MAVProxy.modules.mavproxy_map3d.map3d import MISSION_STYLES
        module = TestDrawnTrack().live_module('plane')
        styles = []
        sends = []
        # settings of the kind the module makes, to see the choice enforced
        module.map3d_settings = mp_settings.MPSettings([
            ('fpvfov', float, 90.0), ('terrainbrightness', float, 1.25),
            ('terrainshading', bool, True), ('terrainwireframe', bool, False),
            ('showdirection', bool, True), ('showlabels', bool, False),
            ('labelsize', int, 14),
            mp_settings.MPSetting('missionpath', str, MISSION_STYLES[0],
                                  choice=MISSION_STYLES)])
        module.map = SimpleNamespace(
            is_alive=lambda: True, set_fpv_fov=lambda fov: None,
            set_mission_arrows=lambda enable: None,
            set_mission_labels=lambda enable: None,
            set_mission_label_size=lambda size: None,
            set_render_settings=lambda *args: None,
            set_mission_style=styles.append,
            set_mission=lambda items, track=None: sends.append(track))
        module.cmd_map3d(['set', 'missionpath', 'plain'])
        assert styles == ['plain']
        assert sends == [None]
        # back to the path flown, which is worked out again to draw
        module.cmd_map3d(['set', 'missionpath', 'flown'])
        assert styles == ['plain', 'flown']
        assert sends[-1] is not None
        # and a style there is not, is not taken
        module.cmd_map3d(['set', 'missionpath', 'fancy'])
        assert module.map3d_settings.missionpath == 'flown'

    def test_mavexplorer_draws_the_style_asked_for(self, monkeypatch):
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import map3d
        drawn = TestDrawnTrack()
        mx = drawn.explorer()
        views = []

        class Viewer(object):
            def __init__(self, title=None):
                self.styles = []
                self.missions = []
                views.append(self)

            def is_alive(self):
                return True

            def set_mission_style(self, style):
                self.styles.append(style)

            def set_mission(self, items, track=None):
                self.missions.append(track)

            def __getattr__(self, name):
                return lambda *args, **kwargs: None
        monkeypatch.setattr(map3d, 'Map3D', Viewer)
        monkeypatch.setattr(map3d, 'missing_packages', lambda: [])
        flights = []
        real = mx.plane_mission_track

        def counting(*args, **kwargs):
            flights.append(args)
            return real(*args, **kwargs)
        monkeypatch.setattr(mx, 'plane_mission_track', counting)
        log = drawn.log(*([drawn.pos(0, 0)] + drawn.mission_dump() + [
            drawn.message('MSG', Message='Mission: 1 Takeoff'),
            drawn.pos(40, 0, 30), drawn.pos(80, 0, 60)]))
        log.rewind = lambda: None
        log.params = dict(PARAMS)
        log.mav_type = mavlink.MAV_TYPE_FIXED_WING

        class Settings(SimpleNamespace):
            def command(self, args):
                setattr(self, args[0], args[1])
        settings = Settings(condition=None, showdirection=True,
                            showlabels=False, labelsize=14,
                            sync_xmap=False, missionpath='geometry')
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(
            mlog=log, settings=settings), raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [])
        mx.cmd_map3d([])
        (view,) = views
        assert view.styles == ['geometry']
        assert view.missions == [None]
        # the path flown was not worked out for a map not drawing it
        assert flights == []
        # asked for later, it is worked out and drawn in the view open
        mx.cmd_set(['missionpath', 'flown'])
        assert view.styles == ['geometry', 'flown']
        assert view.missions[-1] is not None
        assert len(flights) == 1
        # and only the once
        mx.cmd_set(['missionpath', 'plain'])
        mx.cmd_set(['missionpath', 'flown'])
        assert len(flights) == 1

    def test_the_control_on_the_map_draws_the_style_it_picks(self,
                                                             map3d_frame):
        em = self.elements()
        em.set_mission(self.items(), self.track())
        events = []
        frame = map3d_frame(em, events)
        frame.style_choice.SetStringSelection('plain')
        frame.on_style_choice(None)
        assert em.mission_style == 'plain'
        assert len(em.mission_rings) == 1
        # and the setting follows the control, so a view opened next draws
        # the mission the same way
        assert events == [('mission_style', 'plain')]

    def test_the_live_map_takes_the_style_picked_on_it(self):
        module = self.live_module('plain')
        module.map.is_alive = lambda: True
        module.map.check_events = lambda: [('mission_style', 'flown')]
        echoed = []
        module.map.set_mission_style = echoed.append
        # the rest of the idle task has nothing to do here
        module.send_kml = lambda kml_mod=None: None
        module.kml_change_state = None
        module.terrain_resolved = False
        module.idle_task()
        assert module.map3d_settings.missionpath == 'flown'
        # the view is told the setting it now has, whatever it was told since
        assert echoed == ['flown']
        # and the mission goes again, with the path flown worked out for it
        assert module.sent[-1] is not None

    def test_mavexplorer_takes_the_style_picked_on_a_view(self, monkeypatch):
        from types import SimpleNamespace
        mx = TestDrawnTrack().explorer()
        styles = []
        drawn = []
        view = SimpleNamespace(
            is_alive=lambda: True,
            check_events=lambda: [('mission_style', 'flown')],
            set_mission_arrows=lambda enable: None,
            set_mission_labels=lambda enable: None,
            set_mission_label_size=lambda size: None,
            set_mission_style=styles.append,
            set_mission=lambda items, track=None: drawn.append(track),
            mission_to_fly=(['an item'], lambda: ['a path flown']))
        settings = SimpleNamespace(showdirection=True, showlabels=False,
                                   labelsize=14, missionpath='geometry')
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(settings=settings),
                            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [view])
        mx.poll_map3d_views()
        assert settings.missionpath == 'flown'
        assert styles == ['flown']
        # the path flown is worked out only now that it is drawn, and once
        assert drawn == [['a path flown']]
        assert view.mission_to_fly is None

    def unflyable(self, module):
        '''give the live module's mission a NAV_DELAY, which cannot be flown
        through'''
        from types import SimpleNamespace
        loader = module.mpstate.module('wp').wploader
        loader.wpoints.append(SimpleNamespace(
            seq=len(loader.wpoints), command=mavlink.MAV_CMD_NAV_DELAY,
            x=0, y=0, z=0, frame=3, param1=10, param2=0, param3=0, param4=0))

    def test_the_live_map_says_when_a_mission_has_no_path_flown(self, capsys):
        module = self.live_module('flown')
        self.unflyable(module)
        module.send_mission()
        assert module.sent[-1] is None
        assert 'could not work out the path' in capsys.readouterr().out
        # once for the mission, however often it is sent, and when home
        # has moved far enough for it to be flown again
        module.send_mission()
        module.home_amsl += 50.0
        module.send_mission()
        assert module.sent[-1] is None
        assert capsys.readouterr().out == ''
        # and again for another which cannot be flown either
        module.mpstate.module('wp').wploader.wpoints[-1].param1 = 20
        module.send_mission()
        assert 'could not work out the path' in capsys.readouterr().out
        # a mission which can be flown says nothing
        module = self.live_module('flown')
        module.send_mission()
        assert module.sent[-1] is not None
        assert capsys.readouterr().out == ''

    def test_the_live_map_says_when_asked_for_a_path_it_cannot_draw(
            self, capsys):
        from types import SimpleNamespace
        from MAVProxy.modules import mavproxy_map3d
        module = TestDrawnTrack().live_module('copter')
        module.map3d_settings = mavproxy_map3d.make_settings()
        styles = []
        module.map = SimpleNamespace(
            is_alive=lambda: True, set_fpv_fov=lambda fov: None,
            set_mission_arrows=lambda enable: None,
            set_mission_labels=lambda enable: None,
            set_mission_label_size=lambda size: None,
            set_mission_style=styles.append,
            set_render_settings=lambda *args: None,
            set_mission=lambda items, track=None: module.sent.append(track))
        # flown is where it starts, which is not asking for it
        module.send_mission()
        assert capsys.readouterr().out == ''
        module.cmd_map3d(['set', 'missionpath', 'flown'])
        assert 'only a plane' in capsys.readouterr().out
        module.send_mission()
        assert capsys.readouterr().out == ''
        # nor is another style
        module.cmd_map3d(['set', 'missionpath', 'plain'])
        assert capsys.readouterr().out == ''
        # and from the view's own choice
        module.map.check_events = lambda: [('mission_style', 'flown')]
        module.send_kml = lambda kml_mod=None: None
        module.kml_change_state = None
        module.terrain_resolved = False
        module.idle_task()
        assert 'only a plane' in capsys.readouterr().out

    def test_mavexplorer_says_when_a_mission_has_no_path_flown(
            self, monkeypatch, capsys):
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import map3d
        drawn = TestDrawnTrack()
        mx = drawn.explorer()
        views = []

        class Viewer(object):
            def __init__(self, title=None):
                self.missions = []
                views.append(self)

            def is_alive(self):
                return True

            def set_mission(self, items, track=None):
                self.missions.append(track)

            def __getattr__(self, name):
                return lambda *args, **kwargs: None
        monkeypatch.setattr(map3d, 'Map3D', Viewer)
        monkeypatch.setattr(map3d, 'missing_packages', lambda: [])
        flights = []
        monkeypatch.setattr(mx, 'plane_mission_track',
                            lambda *args, **kwargs: flights.append(args))

        class Settings(SimpleNamespace):
            def command(self, args):
                setattr(self, args[0], args[1])

        def open_view(mav_type, style):
            log = drawn.log(*([drawn.pos(0, 0)] + drawn.mission_dump() + [
                drawn.message('MSG', Message='Mission: 1 Takeoff'),
                drawn.pos(40, 0, 30), drawn.pos(80, 0, 60)]))
            log.rewind = lambda: None
            log.params = dict(PARAMS)
            log.mav_type = mav_type
            settings = Settings(condition=None, showdirection=True,
                                showlabels=False, labelsize=14,
                                sync_xmap=False, missionpath=style)
            monkeypatch.setattr(mx, 'mestate', SimpleNamespace(
                mlog=log, settings=settings), raising=False)
            monkeypatch.setattr(mx, 'map3d_views', [])
            mx.cmd_map3d([])
            return views[-1]

        # a plane's mission which cannot be flown through
        view = open_view(mavlink.MAV_TYPE_FIXED_WING, 'flown')
        assert view.missions == [None]
        assert 'could not work out the path' in capsys.readouterr().out
        # and which is not flown again for nothing
        assert len(flights) == 1
        mx.cmd_set(['showdirection', 'true'])
        assert len(flights) == 1
        # a copter's, left as it starts, says nothing
        open_view(mavlink.MAV_TYPE_QUADROTOR, 'flown')
        assert capsys.readouterr().out == ''
        # but asked for, it does
        open_view(mavlink.MAV_TYPE_QUADROTOR, 'geometry')
        mx.cmd_set(['missionpath', 'flown'])
        assert 'could not work out the path' in capsys.readouterr().out
