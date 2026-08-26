'''geometry behind the mission rendering: arc waypoints and circling items'''

import math

import pytest

from MAVProxy.modules.lib import mp_util

# somewhere with a large longitude scale factor, to catch flat-earth mistakes
HERE = (-35.363262, 149.165238)


def radial_error(centre, radius, points):
    '''worst distance of points from a circle of radius about centre'''
    return max(abs(mp_util.gps_distance(centre[0], centre[1], p[0], p[1]) - radius)
               for p in points)


def swept_angle(centre, points):
    '''signed angle swept about centre, positive clockwise'''
    bearings = [mp_util.gps_bearing(centre[0], centre[1], p[0], p[1])
                for p in points]
    return sum(mp_util.wrap_180(bearings[i] - bearings[i-1])
               for i in range(1, len(bearings)))


class TestRhumbHelpers(object):

    def test_distance_takes_the_short_way_around(self):
        # a pair either side of the antimeridian is a short hop, not most of
        # the way around the globe
        d = mp_util.gps_distance(0, 179.9, 0, -179.9)
        assert d == pytest.approx(22263, abs=100)

    def test_bearing_takes_the_short_way_around(self):
        assert mp_util.gps_bearing(0, 179.9, 0, -179.9) == pytest.approx(90)
        assert mp_util.gps_bearing(0, -179.9, 0, 179.9) == pytest.approx(270)

    def test_ordinary_bearings_unchanged(self):
        (lat, lon) = HERE
        assert mp_util.gps_bearing(lat, lon, lat - 0.001, lon) == pytest.approx(180)
        assert mp_util.gps_bearing(lat, lon, lat, lon + 0.001) == pytest.approx(90)

    def test_newpos_round_trip(self):
        (lat, lon) = mp_util.gps_newpos(HERE[0], HERE[1], 37, 500)
        back = mp_util.gps_newpos(lat, lon, 37 + 180, 500)
        assert back[0] == pytest.approx(HERE[0])
        assert back[1] == pytest.approx(HERE[1])


class TestArcGeometry(object):

    @pytest.mark.parametrize("angle", [45, 90, 170, 270, 359,
                                       -45, -90, -170, -270, -359])
    def test_endpoints_and_radius(self, angle):
        start = HERE
        end = mp_util.gps_newpos(start[0], start[1], 75, 400)
        (centre, radius, _) = mp_util.arc_centre_and_radius(
            start[0], start[1], end[0], end[1], angle)
        # the chord subtends the swept angle at the centre
        chord = mp_util.gps_distance(start[0], start[1], end[0], end[1])
        assert radius == pytest.approx(
            chord / (2 * math.sin(math.radians(abs(angle) / 2.0))), rel=1e-6)
        # both endpoints sit on the circle, and so does every sample
        points = mp_util.arc_points(start, end, angle)
        assert points[0] == start
        assert points[-1] == end
        # relative, because these are rhumb lines: a near-full sweep implies a
        # circle tens of km across, where the flat-earth error is metres-ish
        assert radial_error(centre, radius, points) < 0.01 + radius * 1.0e-5

    @pytest.mark.parametrize("angle", [45, 90, 270, 359])
    def test_positive_is_clockwise(self, angle):
        start = HERE
        end = mp_util.gps_newpos(start[0], start[1], 75, 400)
        (centre, _, _) = mp_util.arc_centre_and_radius(
            start[0], start[1], end[0], end[1], angle)
        points = mp_util.arc_points(start, end, angle)
        assert swept_angle(centre, points) == pytest.approx(angle, abs=1.0)
        # ... and the mirror image sweeps the other way
        (centre, _, _) = mp_util.arc_centre_and_radius(
            start[0], start[1], end[0], end[1], -angle)
        points = mp_util.arc_points(start, end, -angle)
        assert swept_angle(centre, points) == pytest.approx(-angle, abs=1.0)

    def test_half_turn_centre_is_the_midpoint(self):
        start = HERE
        end = mp_util.gps_newpos(start[0], start[1], 20, 300)
        for angle in (180, -180):
            (centre, radius, _) = mp_util.arc_centre_and_radius(
                start[0], start[1], end[0], end[1], angle)
            assert radius == pytest.approx(150, rel=1e-3)
            assert mp_util.gps_distance(
                centre[0], centre[1], start[0], start[1]) == pytest.approx(150, rel=1e-3)

    @pytest.mark.parametrize("angle", [0, 360, -360, 720])
    def test_degenerate_sweeps_fall_back_to_a_line(self, angle):
        start = HERE
        end = mp_util.gps_newpos(start[0], start[1], 75, 400)
        assert mp_util.arc_centre_and_radius(
            start[0], start[1], end[0], end[1], angle) is None
        assert mp_util.arc_points(start, end, angle) == [start, end]

    def test_coincident_endpoints_fall_back_to_a_line(self):
        assert mp_util.arc_centre_and_radius(
            HERE[0], HERE[1], HERE[0], HERE[1], 90) is None
        assert mp_util.arc_points(HERE, HERE, 90) == [HERE, HERE]

    def test_arc_across_the_antimeridian(self):
        start = (0.0, 179.9)
        end = (0.0, -179.9)
        (centre, radius, _) = mp_util.arc_centre_and_radius(
            start[0], start[1], end[0], end[1], 90)
        # chord ~22.3km, so a 90 degree arc has a radius of chord/sqrt(2)
        assert radius == pytest.approx(15743, abs=100)
        points = mp_util.arc_points(start, end, 90)
        assert radial_error(centre, radius, points) < 0.01 + radius * 1.0e-5
        # and it stays near the antimeridian rather than wandering the globe
        for (lat, lon) in points:
            assert abs(mp_util.wrap_180(lon - 180.0)) < 1.0


class TestCirclingItems(object):

    def setup_method(self):
        from pymavlink import mavutil
        self.mavlink = mavutil.mavlink

    def test_radius_parameter_per_command(self):
        m = self.mavlink
        # (command, params, expected signed radius)
        cases = [
            (m.MAV_CMD_NAV_LOITER_UNLIM, (0, 0, 70, 0), 70),
            (m.MAV_CMD_NAV_LOITER_TURNS, (3, 0, -55, 0), -55),
            (m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0), 90),
            (m.MAV_CMD_NAV_LOITER_TO_ALT, (1, -65, 0, 0), -65),
            (m.MAV_CMD_DO_ORBIT, (80, 5, 0, 0), 80),
        ]
        for (command, params, expected) in cases:
            assert mp_util.mission_circle_radius(command, params) == expected

    def test_items_which_do_not_circle(self):
        m = self.mavlink
        for command in (m.MAV_CMD_NAV_WAYPOINT,
                        m.MAV_CMD_NAV_ARC_WAYPOINT,
                        m.MAV_CMD_NAV_TAKEOFF,
                        m.MAV_CMD_DO_JUMP):
            assert mp_util.mission_circle_radius(command, (1, 2, 3, 4)) is None

    def test_unset_radius_uses_the_vehicle_default(self):
        m = self.mavlink
        for params in ((0, 0, 0, 0), (0, 0, float('nan'), 0)):
            assert mp_util.mission_circle_radius(
                m.MAV_CMD_NAV_LOITER_UNLIM, params, 42) == 42
        # with no vehicle default there is no circle to draw
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_UNLIM, (0, 0, 0, 0)) is None

    def test_turn_counts(self):
        m = self.mavlink
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_NAV_LOITER_TURNS, (3, 0, 60, 0)) == 3
        # DO_ORBIT counts in radians
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_DO_ORBIT, (80, 5, 0, math.radians(270))) == pytest.approx(0.75)
        # circling forever, and items which do not count turns
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_DO_ORBIT, (80, 5, 0, 0)) is None
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0)) is None


class TestHoveringVehicles(object):

    def setup_method(self):
        from pymavlink import mavutil
        self.mavlink = mavutil.mavlink

    def test_which_vehicles_hover(self):
        m = self.mavlink
        for vehicle in (m.MAV_TYPE_QUADROTOR, m.MAV_TYPE_HEXAROTOR,
                        m.MAV_TYPE_HELICOPTER, m.MAV_TYPE_SUBMARINE,
                        'copter', 'sub'):
            assert mp_util.vehicle_hovers_to_loiter(vehicle)
        for vehicle in (m.MAV_TYPE_FIXED_WING, m.MAV_TYPE_VTOL_QUADROTOR,
                        m.MAV_TYPE_GROUND_ROVER, 'plane', 'rover'):
            assert not mp_util.vehicle_hovers_to_loiter(vehicle)
        # not knowing the vehicle leaves the circle drawn
        assert not mp_util.vehicle_hovers_to_loiter(None)

    def test_a_hovering_vehicle_only_circles_for_some_items(self):
        m = self.mavlink
        # it flies these as circles ...
        for (command, params) in ((m.MAV_CMD_NAV_LOITER_TURNS, (2, 0, 60, 0)),
                                  (m.MAV_CMD_DO_ORBIT, (80, 5, 0, 0))):
            assert mp_util.mission_circle_radius(
                command, params, vehicle=m.MAV_TYPE_QUADROTOR) is not None
        # ... and holds position for these, climbing straight up rather than
        # spiralling for LOITER_TO_ALT
        for (command, params) in ((m.MAV_CMD_NAV_LOITER_UNLIM, (0, 0, 70, 0)),
                                  (m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0)),
                                  (m.MAV_CMD_NAV_LOITER_TO_ALT, (1, -65, 0, 0))):
            assert mp_util.mission_circle_radius(
                command, params, vehicle=m.MAV_TYPE_QUADROTOR) is None
            # a forward-flight vehicle circles for all of them
            assert mp_util.mission_circle_radius(
                command, params, vehicle=m.MAV_TYPE_FIXED_WING) is not None

    def test_unknown_vehicle_keeps_the_old_behaviour(self):
        m = self.mavlink
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TO_ALT, (1, -65, 0, 0)) == -65


class TestVehicleRates(object):

    PLANE = {'TECS_CLMB_MAX': 5.0, 'TECS_SINK_MAX': 4.0,
             'AIRSPEED_CRUISE': 22.0, 'WP_LOITER_RAD': 90.0}
    COPTER = {'WP_SPD_UP': 2.5, 'WP_SPD_DN': 1.5, 'WP_SPD': 10.0}
    OLDER = {'WPNAV_SPEED_UP': 250.0, 'WPNAV_SPEED_DN': 150.0,
             'TRIM_ARSPD_CM': 2200.0}

    def test_param_value_accepts_a_mapping_or_a_callable(self):
        assert mp_util.param_value(self.PLANE, 'TECS_CLMB_MAX') == 5.0
        assert mp_util.param_value(self.PLANE.get, 'TECS_CLMB_MAX') == 5.0
        assert mp_util.param_value(self.PLANE, 'NO_SUCH_PARAM') is None
        assert mp_util.param_value(None, 'TECS_CLMB_MAX') is None

    def test_rates_from_forward_flight_parameters(self):
        assert mp_util.vehicle_climb_rate(self.PLANE) == 5.0
        assert mp_util.vehicle_climb_rate(self.PLANE, descending=True) == 4.0
        assert mp_util.vehicle_cruise_speed(self.PLANE) == 22.0

    def test_rates_from_multicopter_parameters(self):
        assert mp_util.vehicle_climb_rate(self.COPTER) == 2.5
        assert mp_util.vehicle_climb_rate(self.COPTER, descending=True) == 1.5
        assert mp_util.vehicle_cruise_speed(self.COPTER) == 10.0

    def test_older_centimetre_parameters_are_scaled(self):
        assert mp_util.vehicle_climb_rate(self.OLDER) == 2.5
        assert mp_util.vehicle_climb_rate(self.OLDER, descending=True) == 1.5
        assert mp_util.vehicle_cruise_speed(self.OLDER) == 22.0

    def test_no_parameters_at_all(self):
        for params in ({}, None):
            assert mp_util.vehicle_climb_rate(params) is None
            assert mp_util.vehicle_climb_rate(params, descending=True) is None
            assert mp_util.vehicle_cruise_speed(params) is None


class TestLoiterToAltTurns(object):

    PLANE = TestVehicleRates.PLANE

    def test_turns_follow_the_configured_rates(self):
        # a turn at 90m radius and 22m/s takes 2*pi*90/22 = 25.7s, and climbs
        # 5m/s * 25.7s = 128.5m, so 300m of climb is a little over two turns
        turns = mp_util.loiter_to_alt_turns(90, 300, self.PLANE)
        assert turns == pytest.approx(300.0 / (5.0 * 2 * math.pi * 90 / 22.0))
        assert turns == pytest.approx(2.334, abs=0.01)

    def test_descending_uses_the_sink_rate(self):
        # TECS_SINK_MAX is 4m/s against a 5m/s climb, so descending takes
        # proportionally longer
        climb = mp_util.loiter_to_alt_turns(90, 300, self.PLANE)
        sink = mp_util.loiter_to_alt_turns(90, -300, self.PLANE)
        assert sink == pytest.approx(climb * 5.0 / 4.0)

    def test_one_turn_when_the_rates_are_unknown(self):
        assert mp_util.loiter_to_alt_turns(90, 300, {}) == 1.0
        assert mp_util.loiter_to_alt_turns(90, 300, None) == 1.0
        # ... and when there is nothing to work from at all
        assert mp_util.loiter_to_alt_turns(0, 300, self.PLANE) == 1.0
        assert mp_util.loiter_to_alt_turns(90, None, self.PLANE) == 1.0
        assert mp_util.loiter_to_alt_turns(90, 300, {}, default_turns=3) == 3

    def test_absurd_turn_counts_are_clamped(self):
        # a huge climb should not draw a spiral of hundreds of turns, and the
        # vehicle flies part of a circle however little is left to do
        assert mp_util.loiter_to_alt_turns(90, 1000000, self.PLANE) == 20.0
        assert mp_util.loiter_to_alt_turns(90, 0.001, self.PLANE) == 0.25

    def test_the_approach_leg_takes_some_of_the_climb(self):
        # the vehicle is already climbing on the way to the loiter point, so
        # a long leg leaves less to do on the circle
        near = mp_util.loiter_to_alt_turns(90, 300, self.PLANE, 100)
        far = mp_util.loiter_to_alt_turns(90, 300, self.PLANE, 1000)
        assert far < near < mp_util.loiter_to_alt_turns(90, 300, self.PLANE)
        # a leg long enough to do all the climbing still draws part of a turn
        assert mp_util.loiter_to_alt_turns(90, 300, self.PLANE, 100000) == 0.25

    def test_a_plane_cruises_down_at_the_minimum_sink_rate(self):
        # SINK_MAX is a limit the vehicle will not exceed rather than the rate
        # it descends at, so SINK_MIN wins when both are set
        params = dict(self.PLANE, TECS_SINK_MIN=2.0, TECS_SINK_MAX=5.0)
        assert mp_util.vehicle_climb_rate(params, descending=True) == 2.0


class TestPolygonBounds(object):

    def test_bounds_cover_the_arc_and_not_just_the_chord(self):
        pytest.importorskip("cv2")
        from MAVProxy.modules.mavproxy_map.mp_slipmap_util import SlipPolygon
        start = HERE
        end = mp_util.gps_newpos(start[0], start[1], 90, 1100)
        args = ('key', [start, end], 'layer', (255, 255, 255), 2)
        chord = SlipPolygon(*args).bounds()
        arc = SlipPolygon(*args, arcs={0: 270}).bounds()
        # the chord is due east, so its box has no height at all
        assert chord[2] == pytest.approx(0)
        assert arc[2] > 0.01
        # and the arc box contains every point of the arc
        for (lat, lon) in mp_util.arc_points(start, end, 270):
            assert arc[0] <= lat <= arc[0] + arc[2]
            assert arc[1] <= lon <= arc[1] + arc[3]
