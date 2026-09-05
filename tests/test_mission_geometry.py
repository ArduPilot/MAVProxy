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


class TestSpiral(object):

    def spiral(self, radius, turns):
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.elements import spiral_latlon
        return spiral_latlon(HERE, radius, turns)

    def test_stays_at_the_radius(self):
        for (la, lo) in self.spiral(90.0, 2.5):
            assert mp_util.gps_distance(HERE[0], HERE[1], la, lo) == \
                pytest.approx(90.0, abs=0.05)

    @pytest.mark.parametrize("turns", [0.25, 1.0, 2.5])
    def test_sweeps_the_requested_turns(self, turns):
        for radius in (90.0, -90.0):
            points = self.spiral(radius, turns)
            swept = swept_angle(HERE, points)
            expected = turns * 360.0 * (1 if radius > 0 else -1)
            assert swept == pytest.approx(expected, abs=1.0)

    def test_both_ends_are_included(self):
        points = self.spiral(90.0, 1.0)
        # a whole turn returns to where it started, but as a separate point so
        # an altitude can be walked along the spiral
        assert points[0] == pytest.approx(points[-1])

    def test_circle_is_unchanged_by_the_shared_ring_maths(self):
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.elements import circle_latlon
        ring = circle_latlon(HERE, 100.0)
        assert len(ring) == 64
        for (la, lo) in ring:
            assert mp_util.gps_distance(HERE[0], HERE[1], la, lo) == \
                pytest.approx(100.0, abs=0.05)


class TestContinuousTrack(object):
    """the map3d mission line is the path the vehicle is expected to fly, so
    it has to run through the circles rather than to their centres, joining
    and leaving them along a tangent"""

    def build(self, radius, turns, entry_alt=100.0, target_alt=100.0,
              second_radius=None):
        pytest.importorskip("vtk")
        import vtk
        from pymavlink import mavutil
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        from MAVProxy.modules.mavproxy_map3d.terrain import enu
        approach = mp_util.gps_newpos(HERE[0], HERE[1], 180, 800)
        after = mp_util.gps_newpos(HERE[0], HERE[1], 90, 800)
        second_command = (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
                          if second_radius
                          else mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        items = [
            MissionItem(approach[0], approach[1], entry_alt, 3,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1),
            MissionItem(HERE[0], HERE[1], target_alt, 3,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 2,
                        1.0, radius, turns),
            MissionItem(after[0], after[1], target_alt, 3, second_command, 3,
                        0.0, second_radius),
        ]
        em = ElementManager(vtk.vtkRenderer(), HERE[0], HERE[1], 1.0)
        em.set_home(584.0)
        em.set_mission(items)
        actors = em.actors['mission']
        pts = actors[0].GetMapper().GetInput().GetPoints()
        line = [pts.GetPoint(i) for i in range(pts.GetNumberOfPoints())]
        centre = enu(HERE[0], HERE[1], 0.0, HERE[0], HERE[1])
        return actors, line, centre

    def on_circle(self, line, centre, radius):
        return [i for i, p in enumerate(line)
                if abs(math.hypot(p[0]-centre[0], p[1]-centre[1]) - radius) < 0.5]

    def join_angle(self, line, centre, outside, touch):
        # angle between the leg and the radius at the touch point: 90 for a
        # tangent, 0 or 180 for a leg aimed at the centre
        leg = (line[touch][0]-line[outside][0], line[touch][1]-line[outside][1])
        radial = (line[touch][0]-centre[0], line[touch][1]-centre[1])
        dot = leg[0]*radial[0] + leg[1]*radial[1]
        cosang = dot / (math.hypot(*leg) * math.hypot(*radial))
        return math.degrees(math.acos(max(-1.0, min(1.0, cosang))))

    def test_the_track_is_one_line_with_the_circle_in_it(self):
        (actors, line, centre) = self.build(80.0, 2.0)
        # one polyline and one set of markers: no circle drawn off on its own
        assert len(actors) == 2
        on = self.on_circle(line, centre, 80.0)
        assert len(on) > 40
        # and those points are a single unbroken run within the line
        assert on[-1] - on[0] + 1 == len(on)
        assert on[0] > 0                  # a leg comes in
        assert on[-1] < len(line) - 1     # and a leg goes out

    def test_the_line_never_reaches_the_centre(self):
        (actors, line, centre) = self.build(80.0, 2.0)
        nearest = min(math.hypot(p[0]-centre[0], p[1]-centre[1]) for p in line)
        # the vehicle circles the point rather than overflying it
        assert nearest == pytest.approx(80.0, abs=0.5)

    def test_the_circle_is_joined_and_left_along_a_tangent(self):
        for radius in (80.0, -80.0):
            (actors, line, centre) = self.build(radius, 2.0)
            on = self.on_circle(line, centre, 80.0)
            joined = self.join_angle(line, centre, on[0]-1, on[0])
            left = self.join_angle(line, centre, on[-1]+1, on[-1])
            assert joined == pytest.approx(90.0, abs=1.0)
            assert left == pytest.approx(90.0, abs=1.0)

    def test_the_leg_between_two_circles_touches_both(self):
        # the leg out of one loiter and into the next is tangent to each,
        # whichever way the two of them turn
        for second in (150.0, -150.0):
            (actors, line, centre) = self.build(80.0, 2.0, second_radius=second)
            on = self.on_circle(line, centre, 80.0)
            left = self.join_angle(line, centre, on[-1]+1, on[-1])
            assert left == pytest.approx(90.0, abs=1.0)

    def test_a_spiral_draws_its_turns(self):
        (actors, line, centre) = self.build(80.0, 2.0,
                                            entry_alt=100.0, target_alt=300.0)
        on = self.on_circle(line, centre, 80.0)
        bearings = [math.degrees(math.atan2(line[i][0]-centre[0],
                                            line[i][1]-centre[1])) % 360
                    for i in on]
        swept = sum(mp_util.wrap_180(bearings[i]-bearings[i-1])
                    for i in range(1, len(bearings)))
        # the sweep is stretched to leave on the tangent to whatever follows,
        # so it lands near the turns asked for rather than exactly on them
        assert abs(swept) / 360.0 == pytest.approx(2.0, abs=0.5)
        assert on[-1] - on[0] + 1 == len(on)

    def test_the_approach_leg_shows_its_share_of_the_climb(self):
        (actors, line, centre) = self.build(80.0, 2.0,
                                            entry_alt=100.0, target_alt=300.0)
        on = self.on_circle(line, centre, 80.0)
        zs = [p[2] for p in line]
        assert zs[0] == pytest.approx(684.0)
        assert zs[-1] == pytest.approx(884.0)
        # the vehicle is already climbing on the way there, so the leg has
        # done some of it by the time the circle is joined
        assert 684.0 < zs[on[0]] < 884.0
        # the climb runs one way throughout, with no step back at the joins
        assert all(zs[i] >= zs[i-1] for i in range(1, len(zs)))
        # and it is spread evenly around the spiral rather than in jumps
        around = [abs(zs[i]-zs[i-1]) for i in range(on[0]+1, on[-1]+1)]
        assert max(around) - min(around) < 0.5

    def test_a_level_loiter_stays_level(self):
        (actors, line, centre) = self.build(80.0, None)
        zs = [p[2] for p in line]
        assert max(zs) == pytest.approx(min(zs))


class TestDirectionArrows(object):
    """map3d can show which way the mission is flown, toggled by a setting"""

    def build(self, arrows=None):
        pytest.importorskip("vtk")
        import vtk
        from pymavlink import mavutil
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        legs = [mp_util.gps_newpos(HERE[0], HERE[1], b, d)
                for (b, d) in ((0, 0), (0, 900), (90, 900), (180, 900))]
        items = [MissionItem(la, lo, 100.0, 3,
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, i)
                 for (i, (la, lo)) in enumerate(legs)]
        em = ElementManager(vtk.vtkRenderer(), HERE[0], HERE[1], 1.0)
        em.set_home(584.0)
        if arrows is not None:
            em.set_mission_arrows(arrows)
        em.set_mission(items)
        return em

    def cones(self, em):
        glyph = em.actors['mission'][1].GetMapper()
        producer = glyph.GetInputConnection(0, 0).GetProducer()
        producer.Update()
        return producer.GetInput()

    def test_off_by_default(self):
        em = self.build()
        # just the line and the markers
        assert len(em.actors['mission']) == 2

    def test_turning_them_on_and_off(self):
        em = self.build()
        em.set_mission_arrows(True)
        assert len(em.actors['mission']) == 3
        em.set_mission_arrows(False)
        assert len(em.actors['mission']) == 2

    def test_the_setting_is_remembered_across_a_new_mission(self):
        # asked for before the mission arrives, they still appear
        em = self.build(arrows=True)
        assert len(em.actors['mission']) == 3

    def test_they_sit_on_the_track_pointing_the_way_it_is_flown(self):
        em = self.build(arrows=True)
        source = self.cones(em)
        line = em.mission_line
        vectors = source.GetPointData().GetVectors()
        assert source.GetNumberOfPoints() > 5

        def distance_to_segment(p, a, b):
            ab = [b[i]-a[i] for i in range(3)]
            length = sum(c*c for c in ab)
            if length == 0:
                return math.dist(p, a)
            t = sum((p[i]-a[i])*ab[i] for i in range(3)) / length
            t = max(0.0, min(1.0, t))
            return math.dist(p, [a[i] + ab[i]*t for i in range(3)])

        for i in range(source.GetNumberOfPoints()):
            point = source.GetPoint(i)
            heading = vectors.GetTuple3(i)
            assert math.hypot(*heading) == pytest.approx(1.0)
            nearest = min(range(1, len(line)),
                          key=lambda k: distance_to_segment(point, line[k-1],
                                                            line[k]))
            (a, b) = (line[nearest-1], line[nearest])
            assert distance_to_segment(point, a, b) < 0.01
            length = math.dist(a, b)
            along = [(b[j]-a[j])/length for j in range(3)]
            agreement = sum(along[j]*heading[j] for j in range(3))
            assert agreement == pytest.approx(1.0, abs=1e-6)

    def test_a_mission_with_nowhere_to_go_draws_none(self):
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.elements import _arrows
        assert _arrows([(0.0, 0.0, 0.0)], (1.0, 1.0, 1.0)) is None
        assert _arrows([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
                       (1.0, 1.0, 1.0)) is None


class TestCrosstrackRejoin(object):
    """ArduPlane crosstracks the leg out of a loiter against a track from the
    loiter's centre, not from the tangent the vehicle left on, unless the item
    asks otherwise with param4.  So the vehicle comes off the circle a radius
    or so to one side of that track and pulls back onto it"""

    PARAMS = {'NAVL1_PERIOD': 20.0, 'NAVL1_DAMPING': 0.75,
              'AIRSPEED_CRUISE': 22.0}

    def test_the_l1_distance_comes_from_the_parameters(self):
        # 1/pi times damping, period and speed, as the controller computes it
        assert mp_util.vehicle_track_convergence(self.PARAMS) == \
            pytest.approx(0.3183099 * 0.75 * 20.0 * 22.0)
        # damping has a standard value if it is not set
        assert mp_util.vehicle_track_convergence(
            {'NAVL1_PERIOD': 20.0, 'AIRSPEED_CRUISE': 22.0}) == \
            pytest.approx(0.3183099 * 0.75 * 20.0 * 22.0)
        assert mp_util.vehicle_track_convergence({}) is None
        assert mp_util.vehicle_track_convergence(
            {'NAVL1_PERIOD': 20.0}) is None

    def build(self, converge):
        pytest.importorskip("vtk")
        import vtk
        from pymavlink import mavutil
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        from MAVProxy.modules.mavproxy_map3d.terrain import enu
        approach = mp_util.gps_newpos(HERE[0], HERE[1], 180, 1500)
        target = mp_util.gps_newpos(HERE[0], HERE[1], 45, 3000)
        items = [
            MissionItem(approach[0], approach[1], 100.0, 3,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1),
            MissionItem(HERE[0], HERE[1], 100.0, 3,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, 2,
                        2.0, 150.0, None, converge),
            MissionItem(target[0], target[1], 100.0, 3,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3),
        ]
        em = ElementManager(vtk.vtkRenderer(), HERE[0], HERE[1], 1.0)
        em.set_home(584.0)
        em.set_mission(items)
        pts = em.actors['mission'][0].GetMapper().GetInput().GetPoints()
        line = [pts.GetPoint(i) for i in range(pts.GetNumberOfPoints())]
        centre = enu(HERE[0], HERE[1], 0.0, HERE[0], HERE[1])
        end = enu(target[0], target[1], 0.0, HERE[0], HERE[1])
        return line, centre, end

    def exit_index(self, line, centre, radius=150.0):
        """index of the point where the drawn path leaves the circle.

        The rejoin hugs the circle as it departs, so points just after the
        exit can also sit on the radius; take the end of the longest unbroken
        run of them, which is the arc itself
        """
        on = [i for i, p in enumerate(line)
              if abs(math.hypot(p[0]-centre[0], p[1]-centre[1]) - radius) < 1.0]
        best = (0, on[0])
        (start, previous) = (on[0], on[0])
        for i in on[1:] + [None]:
            if i is None or i != previous + 1:
                if previous - start >= best[0]:
                    best = (previous - start, previous)
                start = i
            previous = i
        return best[1]

    def crosstrack(self, line, centre, end):
        """crosstrack error of each point after the circle, against the track
        that runs from the loiter centre to the next waypoint"""
        (ux, uy) = (end[0]-centre[0], end[1]-centre[1])
        length = math.hypot(ux, uy)
        (ux, uy) = (ux/length, uy/length)

        errors = []
        for p in line[self.exit_index(line, centre):]:
            (dx, dy) = (p[0]-centre[0], p[1]-centre[1])
            if dx*ux + dy*uy > length:
                break
            errors.append(abs(dx*(-uy) + dy*ux))
        return errors

    def test_it_pulls_back_onto_the_centre_track(self):
        converge = mp_util.vehicle_track_convergence(self.PARAMS)
        (line, centre, end) = self.build(converge)
        errors = self.crosstrack(line, centre, end)
        # it leaves the circle about a radius off the track ...
        assert errors[0] == pytest.approx(150.0, abs=5.0)
        # ... and is back on it by the end
        assert errors[-1] < 1.0
        # closing steadily rather than jumping
        assert all(errors[i] <= errors[i-1] + 1e-6
                   for i in range(1, len(errors)))

    def test_it_closes_faster_than_flying_straight_would(self):
        converge = mp_util.vehicle_track_convergence(self.PARAMS)
        (line, centre, end) = self.build(converge)
        errors = self.crosstrack(line, centre, end)
        # flying straight from the exit closes the error linearly over the
        # leg; the controller pulls it in sooner than that
        half = len(errors) // 2
        assert errors[half] < errors[0] * 0.5

    def test_it_leaves_along_the_tangent(self):
        # the regression that matters: an exit that starts turning towards the
        # track straight away kinks the path and cuts back inside the circle
        # the vehicle has just left
        converge = mp_util.vehicle_track_convergence(self.PARAMS)
        (line, centre, end) = self.build(converge)

        exit_index = self.exit_index(line, centre)

        def radius_of(p):
            return math.hypot(p[0]-centre[0], p[1]-centre[1])

        def heading(a, b):
            return math.degrees(math.atan2(b[0]-a[0], b[1]-a[1])) % 360

        def turn_at(i):
            change = heading(line[i], line[i+1]) - heading(line[i-1], line[i])
            return abs(mp_util.wrap_180(change))
        # the circle is drawn as short chords, so it turns a little at every
        # point; leaving it should be no sharper than that
        around_the_circle = turn_at(exit_index - 4)
        assert turn_at(exit_index) < around_the_circle + 2.0
        # and it barely grazes the circle it has left on the way out.  It
        # does dip in a little, since the track it is rejoining runs through
        # the middle, but starting off on the tangent keeps that to a metre
        # or two rather than the tens of metres a straight-to-the-track
        # departure cuts across
        assert min(radius_of(p) for p in line[exit_index:]) > 145.0

    def test_crosstracking_from_the_exit_flies_straight_there(self):
        # param4 set means the leg is measured from where the circle was left,
        # so there is nothing to pull back onto
        (line, centre, end) = self.build(None)
        errors = self.crosstrack(line, centre, end)
        # just the exit point itself, then straight off to the waypoint
        assert len(errors) == 1


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
