'''the path plane_track flies a mission along, against ArduPlane's own
navigation and a flight flown through it'''

import math

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


# PARAMS for an aircraft whose TECS flies by an airspeed sensor
AIRSPEED_SENSOR = dict(PARAMS, ARSPD_TYPE=1, ARSPD_USE=1)


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


def up_to(track, north, east):
    '''the track as far as where it comes closest to the point north and
    east metres of HOME: the mission, without the return to launch which
    follows it once it is complete'''
    (lat, lon, _) = offset(north, east)
    distances = [mp_util.gps_distance(p[0], p[1], lat, lon) for p in track]
    return track[:distances.index(min(distances)) + 1]


def nearest(track, lat, lon):
    '''the point on track nearest lat, lon'''
    return min(track, key=lambda p: mp_util.gps_distance(p[0], p[1], lat, lon))


def arrival(track, lat, lon, within=100.0):
    '''the first point on track within so many metres of lat, lon: where it
    gets there, before it turns for whatever comes next'''
    return [p for p in track
            if mp_util.gps_distance(p[0], p[1], lat, lon) < within][0]


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


class TestVtolApproach(object):
    """a QuadPlane's VTOL landing flown in on a fixed-wing approach:
    Plane::verify_landing_vtol_approach"""

    QUADPLANE = dict(PARAMS, Q_ENABLE=1)

    def vtol_land(self, north, east, alt=60.0, option=1):
        (lat, lon, amsl) = offset(north, east, alt)
        return (mavlink.MAV_CMD_NAV_VTOL_LAND, lat, lon, amsl,
                (option, 0, 0, 0))

    def items(self, **kwargs):
        # out east and back to land at home
        return [waypoint(0, 2000, 150), self.vtol_land(0, 0, **kwargs)]

    def landed(self, track):
        '''where the fixed-wing flight hands over to the VTOL landing, from
        the landing, where the path ends; the course it was on there; and
        the path before, north and east of home'''
        points = [local(p) for p in track]
        end = points[-1]
        pad = [i for (i, p) in enumerate(points)
               if i > 10 and math.hypot(p[0] - end[0], p[1] - end[1]) < 0.5][0]
        (a, b) = (points[pad - 2], points[pad - 1])
        course = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        handover = (b[0] - end[0], b[1] - end[1])
        bearing = math.degrees(math.atan2(handover[1], handover[0]))
        return (handover, bearing, course, points[:pad])

    def swept(self, points):
        '''degrees swept clockwise about the pad'''
        total = 0.0
        for (a, b) in zip(points, points[1:]):
            total += mp_util.wrap_180(math.degrees(math.atan2(b[1], b[0])) -
                                      math.degrees(math.atan2(a[1], a[0])))
        return total

    def circling(self, before, radius):
        '''what was flown from coming back near the landing until the
        approach'''
        back = [i for (i, p) in enumerate(before)
                if i > len(before) // 3 and math.hypot(*p) < radius * 1.3]
        return before[back[0]:]

    def test_the_landing_is_circled_to_and_flown_in_to(self):
        params = dict(self.QUADPLANE, Q_FW_LND_APR_RAD=250)
        track = plane_track.mission_track(HOME, self.items(), params,
                                          approach=0.0)
        (handover, bearing, course, before) = self.landed(track)
        # the landing's altitude, come down to on the way in as ArduPlane
        # does, and the circle round it at Q_FW_LND_APR_RAD, clockwise:
        # from the east, a quarter of the way round to fly north
        assert min(p[2] for p in track[len(track) // 2:]) >= HOME[2] + 55.0
        circling = self.circling(before, 250)
        assert self.swept(circling) == pytest.approx(90.0, abs=30.0)
        assert max(math.hypot(*p) for p in circling) > 250
        # into the wind: north, from south of the landing, a stopping
        # distance or so out, and within the 30 degrees ArduPlane takes as
        # lined up
        assert mp_util.wrap_180(bearing - 180.0) == pytest.approx(0, abs=15)
        assert course == pytest.approx(0.0, abs=30.0)
        assert 100.0 < math.hypot(*handover) < 250.0
        # and it lands where the item is, at the altitude approached at
        assert math.hypot(*local(track[-1])) < 0.5
        assert track[-1][2] == pytest.approx(HOME[2] + 60.0, abs=1.0)

    def test_it_is_flown_into_the_wind(self):
        params = dict(self.QUADPLANE, Q_FW_LND_APR_RAD=250)
        for approach in (0.0, 90.0, -135.0):
            track = plane_track.mission_track(HOME, self.items(), params,
                                              approach=approach)
            (_, bearing, course, _) = self.landed(track)
            assert mp_util.wrap_180(course - approach) == pytest.approx(
                0.0, abs=30.0)
            assert mp_util.wrap_180(bearing - (approach + 180)) == \
                pytest.approx(0.0, abs=20.0)
        # with no wind to go on, ArduPlane's sum comes to due south
        track = plane_track.mission_track(HOME, self.items(), params)
        (_, bearing, course, _) = self.landed(track)
        assert mp_util.wrap_180(course - 180.0) == pytest.approx(0, abs=30)
        assert mp_util.wrap_180(bearing) == pytest.approx(0, abs=20)

    def test_the_circle_is_flown_the_way_its_radius_says(self):
        # from the east to fly east: half way round, either way
        for (radius, clockwise) in ((250, True), (-250, False)):
            params = dict(self.QUADPLANE, Q_FW_LND_APR_RAD=radius)
            track = plane_track.mission_track(HOME, self.items(), params,
                                              approach=90.0)
            swept = self.swept(self.circling(self.landed(track)[3], 250))
            assert swept > 150 if clockwise else swept < -150
        # and with no radius of its own, WP_LOITER_RAD's
        track = plane_track.mission_track(HOME, self.items(),
                                          self.QUADPLANE, approach=90.0)
        radius = PARAMS['WP_LOITER_RAD']
        circling = self.circling(self.landed(track)[3], radius)
        assert min(math.hypot(*p) for p in circling) < radius * 1.3
        assert self.swept(circling) > 150

    def test_what_asks_for_it(self):
        def circles(items, params):
            # coming from the east, only circling takes it west of the pad
            track = plane_track.mission_track(HOME, items, params,
                                              approach=0.0)
            return any(local(p)[1] < -40 for p in track)
        # param1, or Q_OPTIONS for every VTOL landing
        assert circles(self.items(option=1), self.QUADPLANE)
        assert circles(self.items(option=0),
                       dict(self.QUADPLANE, Q_OPTIONS=1 << 4))
        # but not a VTOL landing asked for neither way, which is flown
        # straight in, nor anything but a QuadPlane
        assert not circles(self.items(option=0), self.QUADPLANE)
        assert not circles(self.items(option=1), PARAMS)
        items = [waypoint(0, 2000, 150), self.vtol_land(0, 0, option=1)]
        assert plane_track.uses_vtol_approach(items, self.QUADPLANE)
        assert not plane_track.uses_vtol_approach(items, PARAMS)
        items[-1] = self.vtol_land(0, 0, option=0)
        assert not plane_track.uses_vtol_approach(items, self.QUADPLANE)
        assert plane_track.uses_vtol_approach(
            items, dict(self.QUADPLANE, Q_OPTIONS=1 << 4))

    def test_a_landing_where_the_aircraft_is_goes_out_to_the_circle(self):
        # a VTOL landing with no position of its own is where the aircraft
        # is when it starts, so the circle is started from its middle, and
        # the aircraft goes out to it before breaking out onto the approach
        # -- even arriving, as here, on the course it breaks out on: east,
        # for a clockwise circle and an approach south
        here = (mavlink.MAV_CMD_NAV_VTOL_LAND, 0.0, 0.0, HOME[2] + 60,
                (1, 0, 0, 0))
        params = dict(self.QUADPLANE, Q_FW_LND_APR_RAD=250)
        track = plane_track.mission_track(
            HOME, [waypoint(0, 2000, 60), here], params, approach=180.0)
        landing = local(track[-1])
        assert landing[1] > 1800
        points = [local(p) for p in track]
        arrived = [i for (i, p) in enumerate(points)
                   if math.hypot(p[0] - landing[0], p[1] - landing[1]) < 30]
        out = [math.hypot(p[0] - landing[0], p[1] - landing[1])
               for p in points[arrived[0]:]]
        assert max(out) > 245.0
        # and comes in from the north, into the wind
        last = points[-3]
        assert last[0] > landing[0] + 50

    def test_a_mission_which_runs_out_lands_on_the_approach_too(self):
        """the return a mission which runs out flies is landed as
        Q_RTL_MODE says, so such a mission is flown into the wind as
        much as one which returns by an item of its own"""
        params = dict(self.QUADPLANE, Q_RTL_MODE=2, Q_FW_LND_APR_RAD=200)
        items = [waypoint(0, 2000, 150), waypoint(500, 2000, 150)]
        assert plane_track.uses_vtol_approach(items, params)
        # and the course it is drawn on is the wind's
        into = [plane_track.mission_track(HOME, items, params,
                                          approach=course)[-3]
                for course in (45.0, 225.0)]
        assert mp_util.gps_distance(into[0][0], into[0][1],
                                    into[1][0], into[1][1]) > 100.0
        # where a mission the flight ends in does not return at all
        for ending in (self.vtol_land(0, 0, option=0),
                       (mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, None,
                        (0, 0, 0, 0)),
                       (mavlink.MAV_CMD_DO_JUMP, 0, 0, None, (1, -1, 0, 0))):
            assert not plane_track.uses_vtol_approach(items + [ending], params)
        # and a landing the flight never reaches, a jump going past it, does
        # not end anything: the mission still runs out
        landing = (mavlink.MAV_CMD_NAV_LAND, HOME[0], HOME[1], HOME[2],
                   (0, 0, 0, 0))
        jumped = [items[0], (mavlink.MAV_CMD_DO_JUMP, 0, 0, None,
                             (4, 1, 0, 0)), landing, items[1]]
        assert plane_track.uses_vtol_approach(jumped, params)
        # nor is an approach landing after one the flight ends at flown
        assert not plane_track.uses_vtol_approach(
            [items[0], landing, self.vtol_land(0, 0, option=1)], params)
        # and a mission nothing can make sense of has no path at all, so
        # asking it about the wind is answered rather than raising
        nonsense = [items[0], (mavlink.MAV_CMD_DO_JUMP, 0, 0, None,
                               (float('nan'), 1, 0, 0))]
        assert not plane_track.uses_vtol_approach(nonsense, params)
        assert plane_track.mission_track(HOME, nonsense, params) is None

    def test_a_return_to_launch_can_land_so(self):
        """Q_RTL_MODE 2: circle home at RTL_ALTITUDE, go down to Q_RTL_ALT,
        and come in on the approach"""
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        items = [waypoint(0, 2000, 150), rtl]
        params = dict(self.QUADPLANE, Q_RTL_MODE=2, Q_FW_LND_APR_RAD=200)
        assert plane_track.uses_vtol_approach(items, params)
        assert not plane_track.uses_vtol_approach(items, self.QUADPLANE)
        track = plane_track.mission_track(HOME, items, params, approach=45.0)
        (_, bearing, course, before) = self.landed(track)
        assert course == pytest.approx(45.0, abs=30.0)
        assert mp_util.wrap_180(bearing - 225.0) == pytest.approx(0, abs=20)
        # it was round home at RTL_ALTITUDE before coming down
        assert any(150 < math.hypot(*local(p)) < 300 and
                   p[2] > HOME[2] + 90.0 for p in track[len(track) // 2:])
        assert track[-1][2] == pytest.approx(HOME[2] + 15.0, abs=1.0)
        assert math.hypot(*local(track[-1])) < 0.5


class TestMissionFlight(object):
    '''ArduPlane's mission logic, flown'''

    def test_it_flies_the_legs_between_waypoints(self):
        track = fly([waypoint(3000, 0), waypoint(3000, 3000)])
        points = [local(p) for p in up_to(track, 3000, 3000)]
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
        points = [local(p) for p in up_to(track, *after)]
        radius = plane_track.L1Control(15, 0.75, 0, 0).loiter_radius(200, 684, 22)
        leaving = [p for p in points
                   if p[0] > 400 and math.hypot(p[0] - centre[0], p[1] - centre[1]) > radius + 50]
        assert leaving
        # the leg is flown against the track out of the loiter's centre, so it
        # ends up on the line from the centre to the waypoint
        late = [p for p in leaving if 1500 < p[0] < 2500]
        assert max(abs(distance_to_line(p, centre, after)) for p in late) < 5.0

    def test_a_loiter_lines_up_on_an_item_with_no_position_as_stored(self):
        # Plane::verify_loiter_heading heads for the next item's location as
        # the mission holds it, which is 0, 0 for one with no position: only
        # starting that item puts it where the aircraft is.  From here that
        # is west-north-west, as Location::get_distance_NE has it
        here = (mavlink.MAV_CMD_NAV_LOITER_TURNS, 0.0, 0.0, HOME[2] + 100,
                (1, 0, 80, 0))
        (clat, clon) = offset(1000, 0)[:2]
        scale = math.cos(math.radians(clat / 2.0))
        towards = math.degrees(math.atan2(-clon * scale, -clat))
        for radius in (100, -100):
            flight = plane_track.MissionFlight(
                (HOME[0], HOME[1]), HOME,
                [loiter_turns(1000, 0, 1, radius, xtrack=1), here], PARAMS)
            courses = []
            real = flight.fly_loiter

            def recording(index):
                if index == 1:
                    courses.append(math.degrees(flight.yaw))
                return real(index)
            flight.fly_loiter = recording
            assert flight.run() is not None
            (course,) = courses
            # within ArduPlane's 10 degrees, and the aircraft's turn since
            assert mp_util.wrap_180(course - towards) == pytest.approx(
                0, abs=15)

    def test_param4_crosstracks_from_where_the_loiter_is_left(self):
        centre = (0.0, 3000.0)
        after = (3000.0, 3000.0)
        from_centre = [local(p) for p in up_to(fly(
            [loiter_turns(centre[0], centre[1], 1, 200), waypoint(*after)]),
            *after)]
        from_exit = [local(p) for p in up_to(fly(
            [loiter_turns(centre[0], centre[1], 1, 200, xtrack=1), waypoint(*after)]),
            *after)]
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
        track = up_to(fly(items), 0, 4000)
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
        # AP_Mission finds no next command, and the mission is over: the
        # aircraft returns to launch
        track = fly([waypoint(2000, 0), self.jump(9, 1), waypoint(0, 2000)])
        assert self.visits(track, 2000, 0) == 1
        assert self.visits(track, 0, 2000) == 0
        assert math.hypot(*local(track[-1])) < PARAMS['WP_LOITER_RAD'] * 1.8

    def test_a_completed_mission_returns_to_launch(self):
        # Plane::exit_mission_callback changes mode to RTL, which circles home
        track = fly([waypoint(3000, 0), waypoint(3000, 3000)])
        assert self.visits(track, 3000, 3000) == 1
        circling = [local(p) for p in track[-20:]]
        for p in circling:
            assert math.hypot(*p) <= PARAMS['WP_LOITER_RAD'] * 1.8

    def test_a_speed_change_before_a_mission_runs_out_is_flown_home_at(self):
        """the items passed on the way to a mission's end are done before
        the return which follows it: a DO_CHANGE_SPEED among them is the
        speed the return is flown at, so its turn for home is wider or
        tighter than it would have been"""
        def turn(speed):
            items = [waypoint(1500, 0), waypoint(1500, 900)]
            if speed is not None:
                items.append(change_speed(speed))
            track = fly(items, dict(PARAMS, RTL_RADIUS=200))
            return max(local(p)[1] for p in track)
        unchanged = turn(None)
        assert turn(30.0) > unchanged + 25.0
        assert turn(12.0) < unchanged - 20.0

    def test_waypoints_closer_than_a_turn_are_passed_on_the_way_home(self):
        # a box smaller than the distance a turn is started from: each
        # waypoint is done with as soon as it is started, and the mission is
        # over in moments, so the path is the return to launch's circle
        box = [waypoint(50, -5), waypoint(50, 45), waypoint(0, 45),
               waypoint(0, -5)]
        track = fly(box, dict(PARAMS, WP_RADIUS=90.0))
        assert len(track) > 20
        far = max(math.hypot(*local(p)) for p in track)
        assert far > PARAMS['WP_LOITER_RAD'] * 0.9

    def test_a_mission_which_loops_for_ever_does_not_return(self):
        items = [waypoint(2000, 0), waypoint(2000, 2000), waypoint(0, 2000),
                 self.jump(1, -1)]
        track = fly(items)
        # it ends going round the loop, not circling home
        assert math.hypot(*local(track[-1])) > 1000

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

    def takeoff_at(self, pitch, alt=50.0):
        return (mavlink.MAV_CMD_NAV_TAKEOFF, HOME[0], HOME[1], HOME[2] + alt,
                (pitch, 0, 0, 0))

    def test_a_fixed_wing_takeoff_climbs_at_its_pitch(self):
        # Plane::takeoff_calc_pitch: with no airspeed sensor the aircraft
        # holds the item's pitch, 10 degrees here, and 4 where it gives none
        for (pitch, flown) in ((10, 10), (0, 4)):
            track = fly([self.takeoff_at(pitch), waypoint(3000, 0)])
            expected = 50.0 / math.tan(math.radians(flown))
            assert self.climbed_out(track) == pytest.approx(expected, rel=0.1)
            (lat, lon, _) = up_to(track, 3000, 0)[-1]
            assert mp_util.gps_bearing(HOME[0], HOME[1], lat, lon) == pytest.approx(0.0, abs=5.0)

    def test_a_sensor_the_throttle_turns_off_is_not_flown_by(self):
        # ARSPD_USE 2 is a glider's sensor behind the propeller, which is
        # not used while the throttle runs: the climb out is its pitch's
        track = fly([self.takeoff_at(10), waypoint(3000, 0)],
                    dict(AIRSPEED_SENSOR, ARSPD_USE=2))
        assert self.climbed_out(track) == pytest.approx(
            50.0 / math.tan(math.radians(10)), rel=0.1)

    def test_a_takeoff_with_an_airspeed_sensor_climbs_out_at_full_rate(self):
        # TECS flies it no lower than its pitch: at TECS_CLMB_MAX, 50m at
        # 5m/s is ten seconds at 22m/s, not a slow approach to the altitude
        track = fly([self.takeoff(), waypoint(3000, 0)], AIRSPEED_SENSOR)
        assert self.climbed_out(track) == pytest.approx(220.0, abs=40.0)
        # but a steeper pitch than that is still flown
        track = fly([self.takeoff_at(20), waypoint(3000, 0)], AIRSPEED_SENSOR)
        assert self.climbed_out(track) == pytest.approx(
            50.0 / math.tan(math.radians(20)), rel=0.1)

    def test_a_fixed_wing_takeoff_heads_for_the_next_waypoint(self):
        # it holds whatever course it was launched or rolled down the
        # runway on, which the way it pointed beforehand says little about:
        # north, for the waypoint, whatever heading it is given
        for heading in (None, 90.0):
            track = plane_track.mission_track(
                HOME, [self.takeoff(), waypoint(3000, 0)], AIRSPEED_SENSOR,
                heading=heading)
            climbed = [local(p) for p in track if p[2] >= HOME[2] + 49.5][0]
            assert climbed[0] == pytest.approx(220.0, abs=40.0)
            assert abs(climbed[1]) < 5.0
        # where a VTOL takeoff transitions the way it points
        track = plane_track.mission_track(
            HOME, [self.takeoff(command=mavlink.MAV_CMD_NAV_VTOL_TAKEOFF),
                   waypoint(3000, 0)], dict(PARAMS, Q_ENABLE=1), heading=90.0)
        assert local(track[2])[1] > 10.0

    def test_a_fixed_wing_takeoff_flies_the_course_it_was_flown_on(self):
        # a caller which has a flight of the takeoff to measure knows the
        # course it held, which the mission does not say
        for course in (90.0, 270.0):
            track = plane_track.mission_track(
                HOME, [self.takeoff(), waypoint(3000, 0)], AIRSPEED_SENSOR,
                heading=0.0, takeoff_course=course)
            climbed = [local(p) for p in track if p[2] >= HOME[2] + 49.5][0]
            east = 220.0 if course == 90.0 else -220.0
            assert climbed[1] == pytest.approx(east, abs=40.0)
            assert abs(climbed[0]) < 5.0

    def test_a_quadplane_takes_off_straight_up(self):
        params = dict(PARAMS, Q_ENABLE=1, Q_OPTIONS=0)
        for command in (mavlink.MAV_CMD_NAV_TAKEOFF,
                        mavlink.MAV_CMD_NAV_VTOL_TAKEOFF):
            # and sets off from overhead home towards the waypoint behind it
            track = up_to(fly([self.takeoff(command=command),
                               waypoint(-3000, 0)], params), -3000, 0)
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

    def test_qrtl_comes_home_on_its_own_altitude_profile(self):
        '''QRTL, which Q_RTL_MODE 3 goes to at once, comes in at
        RTL_ALTITUDE, not down a slope to Q_RTL_ALT, and drops to that only
        over the stretch its sink rate needs (ModeQRTL::
        update_target_altitude)'''
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        params = dict(PARAMS, RTL_RADIUS=250, Q_ENABLE=1, Q_RTL_MODE=3,
                      RTL_ALTITUDE=100, Q_RTL_ALT=15)

        def heights(start_alt):
            track = fly([waypoint(3000, 0, start_alt), rtl], params=params)
            turned = next(i for (i, p) in enumerate(track)
                          if local(p)[0] > 2900)
            return [(math.hypot(*local(p)), p[2] - HOME[2])
                    for p in track[turned:]]

        def at(heights, distance):
            return next(h for (d, h) in heights if d < distance)
        # from RTL_ALTITUDE: 2 * RTL_RADIUS, plus 85m at 3m/s (0.6 of
        # TECS_SINK_MAX) and AIRSPEED_CRUISE, out: 500m + 22m/s * 28.3s
        level = heights(100)
        assert all(h == pytest.approx(100, abs=1) for (d, h) in level
                   if 1150 < d < 2500)
        assert 30 < at(level, 700) < 90
        assert at(level, 20) == pytest.approx(15, abs=5)
        # from higher, down a slope to RTL_ALTITUDE by then
        high = heights(300)
        assert 150 < at(high, 2000) < 270
        assert at(high, 1150) == pytest.approx(100, abs=20)
        assert at(high, 20) == pytest.approx(15, abs=5)

    def test_qrtl_does_not_climb_back_up_to_its_slope(self):
        '''an aircraft which has got below QRTL's slope down to
        RTL_ALTITUDE is followed down, not sent back up to it'''
        params = dict(PARAMS, RTL_RADIUS=250, Q_ENABLE=1, Q_RTL_MODE=3,
                      RTL_ALTITUDE=100, Q_RTL_ALT=15)
        flight = plane_track.MissionFlight((HOME[0], HOME[1]), HOME, [],
                                           params)
        flight.qrtl = True
        flight.next_wp = (flight.home, HOME[2] + 15)
        # the approach began 3km out, 300m up, and the slope from there
        # is at 200m 2km out, where the aircraft is only 150m up
        flight.qrtl_start = (285.0, 3000.0)
        flight.position = (flight.home[0] + 2000.0, flight.home[1])
        flight.amsl = HOME[2] + 150
        flight.update_target_altitude()
        assert flight.target_amsl == pytest.approx(HOME[2] + 150)
        # and never below RTL_ALTITUDE while out there
        flight.amsl = HOME[2] + 50
        flight.update_target_altitude()
        assert flight.target_amsl == pytest.approx(HOME[2] + 100)

    def test_the_first_item_navigated_to(self):
        items = [change_speed(30), self.jump(4, 1), waypoint(2000, 0),
                 waypoint(3000, 0)]
        assert plane_track.first_navigation_item(items) == 4
        assert plane_track.first_navigation_item([change_speed(30)]) is None
        # a mission nothing can make sense of has no first item to navigate
        # to either, as it has no path: the caller is told so rather than
        # having the item raised at it
        nonsense = [self.jump(float('nan'), 1), waypoint(2000, 0)]
        assert plane_track.first_navigation_item(nonsense) is None
        assert plane_track.mission_track(HOME, nonsense, PARAMS) is None

    def test_a_mission_across_the_antimeridian(self):
        import time
        home = (10.0, 179.998, 0.0)
        east = mp_util.gps_newpos(home[0], home[1], 90, 300)
        assert east[1] < 0
        start = time.time()
        track = fly([(mavlink.MAV_CMD_NAV_WAYPOINT, east[0], east[1], 100.0,
                      (0, 0, 0, 0))], home=home)
        assert time.time() - start < 2.0
        # reaching the waypoint, having flown the short way round
        distances = [mp_util.gps_distance(p[0], p[1], east[0], east[1])
                     for p in track]
        assert min(distances) < 100.0
        assert distances.index(min(distances)) < 100

    def test_a_long_loiter_is_flown_through(self):
        # its turns are not taken for being stuck
        for (turns, radius) in ((100, 80), (40, 200)):
            track = fly([loiter_turns(3000, 0, turns, radius),
                         waypoint(3000, 3000)])
            (lat, lon, _) = offset(3000, 3000)
            end = up_to(track, 3000, 3000)[-1]
            assert mp_util.gps_distance(end[0], end[1], lat, lon) < 100.0
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

    def climbed_by(self, track, north):
        '''metres above home the track has climbed to by the time it is so
        far north'''
        return [p for p in track if local(p)[0] >= north][0][2] - HOME[2]

    def test_a_return_to_launch_climbs_at_once(self):
        # Plane::setup_alt_slope: RTL slopes only down to where it goes, and
        # climbs to it as fast as it can
        rtl = (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, None, (0, 0, 0, 0))
        track = fly([waypoint(-3000, 0, alt=40), rtl],
                    dict(PARAMS, RTL_ALTITUDE=200))
        home_ward = track[track.index(up_to(track, -3000, 0)[-1]):]
        # 160m at 5m/s is 32s, or 700m at 22m/s, not the whole way home
        assert self.climbed_by(home_ward, -2000) > 190

    def test_a_climb_in_auto_is_sloped_unless_asked_otherwise(self):
        items = [waypoint(1000, 0, alt=30), waypoint(4000, 0, alt=200)]
        # CLIMB_SLOPE_HGT and above: along the leg, halfway up it halfway
        sloped = fly(items)
        assert self.climbed_by(sloped, 2500) == pytest.approx(115, abs=15)
        # FLIGHT_OPTIONS' IMMEDIATE_CLIMB_IN_AUTO climbs as fast as it can
        at_once = fly(items, dict(
            PARAMS, FLIGHT_OPTIONS=plane_track.FLIGHT_OPTION_IMMEDIATE_CLIMB_IN_AUTO))
        assert self.climbed_by(at_once, 2500) > 190
        # and is still sloped on the way down
        down = [waypoint(1000, 0, alt=200), waypoint(4000, 0, alt=30)]
        for params in (PARAMS, dict(
                PARAMS, FLIGHT_OPTIONS=plane_track.FLIGHT_OPTION_IMMEDIATE_CLIMB_IN_AUTO)):
            assert self.climbed_by(fly(down, params), 2500) == pytest.approx(
                115, abs=15)

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


# SITL flights of ArduPilot autotest missions -- added by ArduPilot PR 34424,
# https://github.com/ArduPilot/ardupilot/pull/34424 -- recorded by
# missions/record_flight.py.  All were flown by ArduPlane V4.8.0-dev, which
# each recording's source says: the drawing follows what a version of
# ArduPlane does, so one which changes how it navigates -- as the QRTL
# approach did in 4.8 -- may want the drawing changed and these flown
# again.  Then how near the path drawn for each has to
# keep to it: the median and 90th percentile of the distance across from
# each point flown to the path drawn, the worst of them, and the worst after
# the first 30s, which takes in a QuadPlane's transition and a plane's
# takeoff; and the 90th percentile and worst of the height of each point
# flown above or below the nearest point of the path drawn
