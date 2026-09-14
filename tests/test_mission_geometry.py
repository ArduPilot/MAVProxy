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


def waypoint(seq, command, x, y, z, frame=3, params=(0, 0, 0, 0)):
    '''a mission item the way the wp module's loader holds one'''
    from types import SimpleNamespace
    return SimpleNamespace(seq=seq, command=command, x=x, y=y, z=z,
                           frame=frame, param1=params[0], param2=params[1],
                           param3=params[2], param4=params[3])


def live_mission_items(wpoints, home_amsl=584.0, params=None,
                       vehicle='plane', default_radius=60.0):
    '''the MissionItems the live map3d module sends for a mission.

    The module is built without the MAVProxy around it: only what
    send_mission() reads is supplied, and terrain is never available
    '''
    from types import SimpleNamespace
    from MAVProxy.modules.mavproxy_map3d import Map3DModule
    module = Map3DModule.__new__(Map3DModule)
    loader = SimpleNamespace(wpoints=wpoints, wp=lambda i: wpoints[i])
    module.mpstate = SimpleNamespace(
        mav_param=params or {}, vehicle_type=vehicle,
        module=lambda name: SimpleNamespace(wploader=loader))
    sent = []
    module.map = SimpleNamespace(set_mission=sent.extend)
    module.home_amsl = home_amsl
    module.home_position = None
    module.default_circle_radius = lambda: default_radius
    module.terrain_alt = lambda lat, lon: None
    module.send_mission()
    return sent


class TestProjection(object):
    """the local frame everything in the 3D map is drawn in"""

    def test_the_antimeridian_is_crossed_the_short_way(self):
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.terrain import enu
        lat = -16.5
        (east, _, _) = enu(lat, -179.999, 0.0, lat, 179.998)
        (west, _, _) = enu(lat, 179.998, 0.0, lat, -179.999)
        metres = math.radians(0.003) * 6378137.0 * math.cos(math.radians(lat))
        assert east == pytest.approx(metres, rel=0.01)
        assert west == pytest.approx(-metres, rel=0.01)
        # and away from it, as it always was
        (e, n, u) = enu(HERE[0] + 0.01, HERE[1] + 0.01, 5.0, HERE[0], HERE[1])
        assert e > 0 and n > 0 and u == 5.0

    # an origin just west of the antimeridian, and a point just east of it
    SEAM = (-16.5, 179.998)

    def terrain(self):
        pytest.importorskip("vtk")
        from quantized_mesh_tile.global_geodetic import GlobalGeodetic
        from MAVProxy.modules.mavproxy_map3d.terrain import TerrainManager
        manager = TerrainManager.__new__(TerrainManager)
        (manager.lat0, manager.lon0) = self.SEAM
        manager.g = GlobalGeodetic(True)
        (manager.zoom_fine, manager.lod_min) = (12, 8)
        (manager.ring, manager.fine_radius) = (1, 2)
        manager.tiles = {}
        return manager

    def test_the_camera_looks_across_the_antimeridian(self):
        from types import SimpleNamespace
        manager = self.terrain()
        east = math.radians(0.003) * 6378137.0 * math.cos(math.radians(self.SEAM[0]))
        for (focal_east, lon) in ((east, -179.999), (-east, 179.995)):
            camera = SimpleNamespace(focal=(focal_east, 0.0, 0.0))
            (lat, focal_lon) = manager.focal_latlon(camera)
            assert (lat, focal_lon) == pytest.approx((self.SEAM[0], lon), abs=1e-6)
        # and the tiles wanted about a point just east of it are those either
        # side of it, not the ones at the western edge of the western tile row
        (lat, lon) = manager.focal_latlon(SimpleNamespace(focal=(east, 0.0, 0.0)))
        fine = sorted(x for (z, x, y) in manager.desired_set(lat, lon)
                      if z == manager.zoom_fine)
        columns = manager.g.GetNumberOfXTilesAtZoom(manager.zoom_fine)
        assert set(fine) == {0, 1, 2, columns - 2, columns - 1}

    def test_terrain_nearest_the_camera_is_fetched_first_across_the_antimeridian(self):
        from types import SimpleNamespace
        manager = self.terrain()
        queued = []
        manager.jobs = SimpleNamespace(put=queued.append)
        (manager.inflight, manager.mesh_revision) = (set(), 0)
        east = math.radians(0.003) * 6378137.0 * math.cos(math.radians(self.SEAM[0]))
        manager.update(SimpleNamespace(
            focal=(east, 0.0, 0.0), pos=(0.0, 0.0, 0.0),
            cam=SimpleNamespace(GetViewAngle=lambda: 30.0)))
        (x, row) = manager.g.LonLatToTile(-179.999, self.SEAM[0], manager.zoom_fine)
        columns = manager.g.GetNumberOfXTilesAtZoom(manager.zoom_fine)
        order = [x for (_, _, (z, x, y)) in queued
                 if (z, y) == (manager.zoom_fine, row)]
        # either side of the antimeridian in turn, not all of the east first
        assert order == [0, columns - 1, 1, columns - 2, 2]

    def test_terrain_heights_are_found_either_side_of_the_antimeridian(self):
        from types import SimpleNamespace
        manager = self.terrain()
        west_tile = SimpleNamespace(bbox=(170.0, -20.0, 180.0, -10.0),
                                    height_at=lambda e, n: 10.0)
        east_tile = SimpleNamespace(bbox=(-180.0, -20.0, -170.0, -10.0),
                                    height_at=lambda e, n: 20.0)
        manager.tiles = {(12, 1, 0): west_tile, (12, 0, 0): east_tile}
        # a ring about a point near it runs on past 180 rather than wrapping
        assert manager.height_at(-16.5, 180.001) == 20.0
        assert manager.height_at(-16.5, -180.0) == 10.0
        assert manager.height_at(-16.5, 179.999) == 10.0
        assert manager.height_at(-16.5, -179.999) == 20.0

    def test_terrain_is_sampled_either_side_of_the_antimeridian(self, monkeypatch):
        pytest.importorskip("vtk")
        import numpy as np
        from MAVProxy.modules.mavproxy_map3d import terrain
        fetched = []

        def decode(z, x, y):
            fetched.append((z, x, y))
            return {"bbox": (-180.0, -20.0, -170.0, -10.0),
                    "verts": np.array([(-180.0, -10.0, 1.0), (-170.0, -10.0, 1.0),
                                       (-180.0, -20.0, 1.0), (-170.0, -20.0, 1.0)])}
        monkeypatch.setattr(terrain, 'decode_terrain', decode)
        monkeypatch.setattr(terrain, '_sample_cache', {})
        assert terrain.sample_terrain(-16.5, 180.001) == pytest.approx(1.0)
        assert fetched == [(12,) + terrain.GlobalGeodetic(True).LonLatToTile(
            -179.999, -16.5, 12)]

    def test_the_view_is_turned_across_the_antimeridian(self):
        pytest.importorskip("vtk")
        pytest.importorskip("wx")
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
        looked = []
        frame = SimpleNamespace(
            terrain=SimpleNamespace(lat0=self.SEAM[0], lon0=self.SEAM[1]),
            tc=SimpleNamespace(look_at=lambda focal, dist=None: looked.append(focal)),
            state=SimpleNamespace(zexag=1.0), on_camera_change=lambda: None)
        Map3DFrame.look_at_latlon(frame, self.SEAM[0], -179.999, 0.0)
        east = math.radians(0.003) * 6378137.0 * math.cos(math.radians(self.SEAM[0]))
        assert looked[0] == pytest.approx((east, 0.0, 0.0), abs=1.0)

    def test_draped_lines_are_sampled_the_short_way_round(self):
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
        em = ElementManager.__new__(ElementManager)
        (em.lat0, em.lon0) = self.SEAM
        points = [(self.SEAM[0], 179.998), (self.SEAM[0], -179.998)]
        samples = list(em._terrain_samples(points, closed=False))
        assert len(samples) > 2
        for (lat, lon) in samples:
            assert 0.0 <= mp_util.wrap_180(lon - 179.998) <= 0.004 + 1e-9

    def test_mavexplorer_looks_at_a_flight_across_the_antimeridian(self):
        pytest.importorskip("wx")
        pytest.importorskip("lxml")
        import importlib.util
        import os
        path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                            'MAVProxy', 'tools', 'MAVExplorer.py')
        spec = importlib.util.spec_from_file_location('mavexplorer', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        # starting just east of it, and going further west than east
        flight = [(-16.5, -179.9995, 30.0), (-16.5, 179.99, 20.0),
                  (-16.49, -179.995, 25.0)]
        (lat, lon, ground, span) = module.path_view(flight)
        assert lat == pytest.approx(-16.49667, abs=1e-4)
        assert lon == pytest.approx(179.9985, abs=1e-4)
        assert ground == 20.0
        east_west = math.radians(0.015) * mp_util.radius_of_earth * math.cos(
            math.radians(-16.49))
        assert span == pytest.approx(east_west, rel=0.01)


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
            # LOITER_TIME has no radius of its own: see below
            (m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0), None),
            (m.MAV_CMD_NAV_LOITER_TO_ALT, (1, -65, 0, 0), -65),
            (mp_util.MAV_CMD_DO_ORBIT, (80, 5, 0, 0), 80),
        ]
        for (command, params, expected) in cases:
            assert mp_util.mission_circle_radius(command, params) == expected

    def test_items_which_do_not_circle(self):
        m = self.mavlink
        for command in (m.MAV_CMD_NAV_WAYPOINT,
                        mp_util.MAV_CMD_NAV_ARC_WAYPOINT,
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

    def test_a_loiter_time_radius_of_one_is_a_direction(self):
        m = self.mavlink
        # ArduPilot cannot store a radius for LOITER_TIME and hands back +-1
        # to say which way round it flies, so the vehicle's own radius is the
        # size and param3 only picks the direction
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 1, 0), 60) == 60
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, -1, 0), 60) == -60
        # and one uploaded with a real radius is flown at the vehicle's all
        # the same: ArduPlane's verify_loiter_time() calls update_loiter(0)
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, -90, 0), 60) == -60
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0), 60) == 60

    def test_a_radius_of_a_metre_or_less_is_the_vehicles(self):
        m = self.mavlink
        # update_loiter() takes a radius of a metre or less as unset, for
        # every loiter item, keeping the direction the item asked for
        for command in (m.MAV_CMD_NAV_LOITER_UNLIM, m.MAV_CMD_NAV_LOITER_TURNS):
            assert mp_util.mission_circle_radius(
                command, (1, 0, 1, 0), 80) == 80
            assert mp_util.mission_circle_radius(
                command, (1, 0, -1, 0), 80) == -80
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TO_ALT, (0, -0.5, 0, 0), 80) == -80
        # and a radius which is really a radius is left alone
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TURNS, (1, 0, -90, 0), 80) == -90

    def test_a_vehicle_radius_of_a_metre_or_less_is_arduplanes_default(self):
        m = self.mavlink
        # with WP_LOITER_RAD a metre or less, update_loiter() flies
        # LOITER_RADIUS_DEFAULT, 60m, the way WP_LOITER_RAD's sign says
        for command in (m.MAV_CMD_NAV_LOITER_UNLIM, m.MAV_CMD_NAV_LOITER_TIME):
            assert mp_util.mission_circle_radius(
                command, (30, 0, 0, 0), 0) == 60
            assert mp_util.mission_circle_radius(
                command, (30, 0, 0, 0), -1) == -60
            assert mp_util.mission_circle_radius(
                command, (30, 0, -1, 0), 1) == -60

    def test_a_loiter_time_goes_the_vehicles_way_unless_told_otherwise(self):
        m = self.mavlink
        # ArduPlane's update_loiter() flies counter-clockwise when the item
        # asked for it, and otherwise the way WP_LOITER_RAD's sign says: so
        # a vehicle whose own radius is negative circles counter-clockwise
        # for an item handing back +1, or one uploaded with no radius at all
        for param3 in (0, 1):
            assert mp_util.mission_circle_radius(
                m.MAV_CMD_NAV_LOITER_TIME, (30, 0, param3, 0), -60) == -60
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, -1, 0), -60) == -60
        # and a clockwise vehicle is only turned round by the item asking
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 0, 0), 60) == 60
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, -1, 0), 60) == -60
        # and with no vehicle radius to take, nothing is drawn rather than a
        # circle a metre across
        assert mp_util.mission_circle_radius(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 1, 0)) is None

    def test_turn_counts(self):
        m = self.mavlink
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_NAV_LOITER_TURNS, (3, 0, 60, 0)) == 3
        # DO_ORBIT counts in radians
        assert mp_util.mission_circle_turns(
            mp_util.MAV_CMD_DO_ORBIT, (80, 5, 0, math.radians(270))) == pytest.approx(0.75)
        # circling forever, and items which do not count turns
        assert mp_util.mission_circle_turns(
            mp_util.MAV_CMD_DO_ORBIT, (80, 5, 0, 0)) is None
        assert mp_util.mission_circle_turns(
            m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0)) is None

    def test_which_items_crosstrack_from_their_centre(self):
        m = self.mavlink
        # param4 of these picks the track the next leg is flown against: 0
        # asks for one out of the loiter centre, 1 for the exit location
        for command in (m.MAV_CMD_NAV_LOITER_TURNS,
                        m.MAV_CMD_NAV_LOITER_TIME,
                        m.MAV_CMD_NAV_LOITER_TO_ALT):
            assert mp_util.mission_crosstracks_from_centre(
                command, (1, 0, 60, 0))
            assert not mp_util.mission_crosstracks_from_centre(
                command, (1, 0, 60, 1))
            # an item which does not say gets what the vehicle does by default
            assert mp_util.mission_crosstracks_from_centre(
                command, (1, 0, 60, float('nan')))

    def test_items_whose_param4_is_not_a_crosstrack_choice(self):
        m = self.mavlink
        # LOITER_UNLIM is never left, and its param4 is a yaw angle; DO_ORBIT
        # counts its turns there.  Neither is asking for a track out
        for command in (m.MAV_CMD_NAV_LOITER_UNLIM, mp_util.MAV_CMD_DO_ORBIT):
            for param4 in (0, 1, 90, math.radians(270)):
                assert not mp_util.mission_crosstracks_from_centre(
                    command, (80, 5, 60, param4))
        # nor is an item which does not circle at all
        assert not mp_util.mission_crosstracks_from_centre(
            m.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0))


class TestLogMissionItems(object):
    """what MAVExplorer hands the 3D map for the mission items in a log.

    The drawing tests below build their MissionItems by hand, so they say
    nothing about whether anything fills them in: this covers the other end
    """

    def module(self):
        pytest.importorskip("wx")
        pytest.importorskip("lxml")
        import importlib.util
        import os
        path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                            'MAVProxy', 'tools', 'MAVExplorer.py')
        # MAVProxy/tools is not a package, so the tool is loaded by path
        spec = importlib.util.spec_from_file_location('mavexplorer', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def resolve(self, mission, params=None):
        return self.module().resolve_mission_amsl(mission, 584.0, params or {})

    def item(self, seq, command, params, alt=100.0):
        return (HERE[0], HERE[1] + seq * 0.01, alt, 3, command, seq, params)

    def test_a_takeoff_with_no_position_climbs_from_home(self):
        from pymavlink import mavutil
        m = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        home = (HERE[0], HERE[1], 584.0, 0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, (0, 0, 0, 0))
        takeoff = (0.0, 0.0, 30.0, 3, m, 1, (0, 0, 0, 0))
        out = self.module().mission_items_from_cmds({0: home, 1: takeoff})
        assert len(out) == 2
        # the takeoff is drawn as the climb from home it is
        assert out[1][0] == HERE[0]
        assert out[1][1] == HERE[1]
        assert out[1][2] == 30.0
        assert out[1][4] == m

    def test_an_item_with_nowhere_to_go_is_left_out(self):
        from pymavlink import mavutil
        home = (HERE[0], HERE[1], 584.0, 0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, (0, 0, 0, 0))
        # a jump carries no position and is not a takeoff, so it is not drawn
        jump = (0.0, 0.0, 0.0, 3, mavutil.mavlink.MAV_CMD_DO_JUMP, 1,
                (0, 0, 0, 0))
        out = self.module().mission_items_from_cmds({0: home, 1: jump})
        assert [i[5] for i in out] == [0]
        # and with nowhere to climb from, nor is a takeoff
        takeoff = (0.0, 0.0, 30.0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1,
                   (0, 0, 0, 0))
        out = self.module().mission_items_from_cmds({1: takeoff})
        assert out == []

    def test_a_mission_uploaded_before_home_was_known(self):
        from pymavlink import mavutil
        # the autopilot keeps its own home in item 0, so a mission uploaded
        # before it had one leaves that item empty as well
        home = (0.0, 0.0, 0.0, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                (0, 0, 0, 0))
        takeoff = (0.0, 0.0, 30.0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1,
                   (0, 0, 0, 0))
        out = self.module().mission_items_from_cmds(
            {0: home, 1: takeoff}, started_at=HERE)
        # the empty home is not drawn, but the takeoff climbs from where the
        # flight started
        assert [i[5] for i in out] == [1]
        assert (out[0][0], out[0][1]) == HERE

    def test_a_takeoff_climbs_from_where_it_was_flown(self):
        from pymavlink import mavutil
        # a vehicle need not take off from the home a mission was uploaded
        # with; where the log says the takeoff began wins over home
        home = (HERE[0], HERE[1], 584.0, 0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, (0, 0, 0, 0))
        takeoff = (0.0, 0.0, 30.0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1,
                   (0, 0, 0, 0))
        elsewhere = mp_util.gps_newpos(HERE[0], HERE[1], 45, 2000)
        out = self.module().mission_items_from_cmds(
            {0: home, 1: takeoff}, flown_from={1: elsewhere})
        assert (out[1][0], out[1][1]) == elsewhere

    def test_the_turns_an_item_asks_for_are_carried_through(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        out = self.resolve([
            self.item(0, m.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0)),
            self.item(1, m.MAV_CMD_NAV_LOITER_TURNS, (3, 0, 60, 0)),
            self.item(2, mp_util.MAV_CMD_DO_ORBIT, (60, 5, 0, math.radians(270))),
            self.item(3, m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 60, 0)),
        ])
        assert out[1].circle_turns == 3
        assert out[2].circle_turns == pytest.approx(0.75)
        # circling until its time is up is not a number of turns
        assert out[3].circle_turns is None

    def test_a_loiter_to_alt_works_its_own_turns_out(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        params = {'AIRSPEED_CRUISE': 20.0, 'TECS_CLMB_MAX': 5.0}
        out = self.resolve([
            self.item(0, m.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), alt=100.0),
            self.item(1, m.MAV_CMD_NAV_LOITER_TO_ALT, (0, 60, 0, 0),
                      alt=700.0),
        ], params)
        # this one has no turn count of its own: what is left to climb on
        # arrival decides it.  600m at 5m/s is 120s of climbing, of which
        # the 900m approach does 45s, leaving four turns of a 60m circle
        assert out[1].circle_turns == pytest.approx(4.0, abs=0.2)

    def log(self, *messages):
        '''a log which hands back these messages in order'''
        class Log(object):
            def __init__(self, messages):
                self.messages = list(messages)

            def recv_match(self, type=None, condition=None):
                while self.messages:
                    m = self.messages.pop(0)
                    if type is None or m.get_type() in type:
                        return m
                return None
        return Log(messages)

    def message(self, kind, **fields):
        from types import SimpleNamespace
        m = SimpleNamespace(_timestamp=0, **fields)
        m.get_type = lambda: kind
        return m

    def dump(self, where, count, command=16):
        '''what the logger writes for a mission of count items at where'''
        out = [self.message('MSG', Message='New mission')]
        for seq in range(count):
            p = mp_util.gps_newpos(where[0], where[1], 90, 100 * seq)
            out.append(self.message('CMD', CNum=seq, CId=command, Lat=p[0],
                                    Lng=p[1], Alt=100.0, Frame=3, Prm1=0,
                                    Prm2=0, Prm3=0, Prm4=0))
        return out

    def test_a_log_draws_the_last_mission_it_holds(self):
        first = HERE
        second = mp_util.gps_newpos(HERE[0], HERE[1], 0, 5000)
        (path, mission) = self.module().mission_from_log(
            self.log(*(self.dump(first, 4) + self.dump(second, 2))))
        # just the second mission, even though it is the shorter of the two
        assert len(mission) == 2
        assert mission[0][0] == pytest.approx(second[0])

    def test_a_cleared_mission_draws_nothing(self):
        # clearing the mission writes the message and then no items at all
        (path, mission) = self.module().mission_from_log(
            self.log(*(self.dump(HERE, 4) +
                       [self.message('MSG', Message='New mission')])))
        assert mission == []

    def test_the_log_says_where_a_takeoff_began(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        moved = mp_util.gps_newpos(HERE[0], HERE[1], 45, 2000)

        def cmd(seq, command, lat, lng, alt):
            return self.message('CMD', CNum=seq, CId=command, Lat=lat,
                                Lng=lng, Alt=alt, Frame=3, Prm1=0, Prm2=0,
                                Prm3=0, Prm4=0)
        (path, mission) = self.module().mission_from_log(self.log(
            self.message('POS', Lat=HERE[0], Lng=HERE[1], Alt=584.0),
            self.message('MSG', Message='New mission'),
            cmd(0, m.MAV_CMD_NAV_WAYPOINT, HERE[0], HERE[1], 584.0),
            cmd(1, m.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 30.0),
            # the vehicle is carried somewhere else before it flies
            self.message('POS', Lat=moved[0], Lng=moved[1], Alt=600.0),
            self.message('MSG', Message='Mission: 1 Takeoff'),
            self.message('POS', Lat=moved[0], Lng=moved[1], Alt=630.0),
        ))
        assert (mission[1][0], mission[1][1]) == moved

    def test_a_log_from_before_the_logger_said_new_mission(self):
        # without the message, a mission's first item still starts it again
        first = [m for m in self.dump(HERE, 4) if m.get_type() == 'CMD']
        second = [m for m in self.dump(
            mp_util.gps_newpos(HERE[0], HERE[1], 0, 5000), 2)
            if m.get_type() == 'CMD']
        (path, mission) = self.module().mission_from_log(
            self.log(*(first + second)))
        assert len(mission) == 2


class TestLiveMissionTurns(object):
    """the turns the live map3d module hands the viewer"""

    def test_the_turns_an_item_asks_for_are_carried_through(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        at = [mp_util.gps_newpos(HERE[0], HERE[1], 90, 400 * i)
              for i in range(3)]
        items = live_mission_items([
            waypoint(0, m.MAV_CMD_NAV_WAYPOINT, at[0][0], at[0][1], 100.0),
            waypoint(1, m.MAV_CMD_NAV_LOITER_TURNS, at[1][0], at[1][1],
                     300.0, params=(3, 0, 60, 0)),
            waypoint(2, m.MAV_CMD_NAV_LOITER_TIME, at[2][0], at[2][1],
                     300.0, params=(30, 0, 60, 0)),
        ])
        assert items[1].circle_turns == 3
        # circling until its time is up is not a number of turns
        assert items[2].circle_turns is None


class TestLiveMissionAltitudes(object):
    """the map3d module measures the climb into a loiter itself, to work out
    how many turns it takes, so it has to resolve the frames first"""

    def module(self, home_amsl):
        from MAVProxy.modules.mavproxy_map3d import Map3DModule
        # the module talks to a live MAVProxy, which is not what is under
        # test here: only the altitude it hands the turn count
        module = Map3DModule.__new__(Map3DModule)
        module.home_amsl = home_amsl
        return module

    def test_frames_resolve_to_amsl(self):
        module = self.module(584.0)
        for frame in (0, 5):
            assert module.item_amsl(700.0, frame) == 700.0
        for frame in (3, 6):
            assert module.item_amsl(100.0, frame) == 684.0

    def test_a_terrain_altitude_without_terrain_is_unknown(self):
        # send_mission() turns a terrain-frame item into AMSL when it has the
        # terrain height; one still in frame 10 or 11 has no known height,
        # and home is not a stand-in for the ground under it
        module = self.module(584.0)
        for frame in (10, 11):
            assert module.item_amsl(100.0, frame) is None

    def test_a_climb_across_two_frames(self):
        module = self.module(584.0)
        # 600m AMSL to 100m above a home at 584m is a climb of 84m, not the
        # 500m descent the raw item altitudes look like
        first = module.item_amsl(600.0, 0)
        second = module.item_amsl(100.0, 3)
        assert second - first == pytest.approx(84.0)

    def test_send_mission_measures_the_climb_in_one_frame(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        params = {'AIRSPEED_CRUISE': 20.0, 'TECS_CLMB_MAX': 5.0,
                  'TECS_SINK_MIN': 2.0}
        loiter_at = mp_util.gps_newpos(HERE[0], HERE[1], 90, 300)
        items = live_mission_items([
            waypoint(0, m.MAV_CMD_NAV_WAYPOINT, HERE[0], HERE[1], 600.0,
                     frame=0),
            waypoint(1, m.MAV_CMD_NAV_LOITER_TO_ALT, loiter_at[0],
                     loiter_at[1], 400.0, frame=3, params=(0, 60, 0, 0)),
        ], home_amsl=584.0, params=params)
        # 600m AMSL up to 400m above a 584m home is a climb of 384m; the raw
        # altitudes would have it a 200m descent, which takes more turns
        climb = mp_util.loiter_to_alt_turns(60, 384.0, params, 300.0)
        descent = mp_util.loiter_to_alt_turns(60, -200.0, params, 300.0)
        assert abs(climb - descent) > 0.5
        assert items[1].circle_turns == pytest.approx(climb, rel=0.01)

    def test_without_a_home_a_relative_altitude_is_unknown(self):
        module = self.module(None)
        assert module.item_amsl(100.0, 3) is None
        # an AMSL item still stands on its own
        assert module.item_amsl(700.0, 0) == 700.0


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
                                  (mp_util.MAV_CMD_DO_ORBIT, (80, 5, 0, 0))):
            assert mp_util.mission_circle_radius(
                command, params, vehicle=m.MAV_TYPE_QUADROTOR) is not None
        # ... and holds position for these, climbing straight up rather than
        # spiralling for LOITER_TO_ALT
        for (command, params) in ((m.MAV_CMD_NAV_LOITER_UNLIM, (0, 0, 70, 0)),
                                  (m.MAV_CMD_NAV_LOITER_TIME, (30, 0, 90, 0)),
                                  (m.MAV_CMD_NAV_LOITER_TO_ALT, (1, -65, 0, 0))):
            assert mp_util.mission_circle_radius(
                command, params, 60, vehicle=m.MAV_TYPE_QUADROTOR) is None
            # a forward-flight vehicle circles for all of them
            assert mp_util.mission_circle_radius(
                command, params, 60, vehicle=m.MAV_TYPE_FIXED_WING) is not None

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


class TestProducersCrosstrack(object):
    """what each producer of MissionItems makes of param4.  Only the loiters
    ArduPlane lets go of carry a crosstrack choice there; LOITER_UNLIM
    never leaves, and DO_ORBIT counts its turns in param4, so either
    producer reading param4 as a crosstrack choice for those would draw a
    pull-back after a loiter the vehicle never leaves"""

    PARAMS = {'NAVL1_PERIOD': 17.0, 'AIRSPEED_CRUISE': 20.0}

    def cases(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        # (command, params, whether the leg out is pulled back to the centre)
        return [
            (m.MAV_CMD_NAV_LOITER_TURNS, (1, 0, 60, 0), True),
            (m.MAV_CMD_NAV_LOITER_TURNS, (1, 0, 60, 1), False),
            (m.MAV_CMD_NAV_LOITER_UNLIM, (0, 0, 60, 0), False),
        ]

    def test_the_live_module(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        at = [mp_util.gps_newpos(HERE[0], HERE[1], 90, 400 * i)
              for i in range(3)]
        for (command, params, pulled_back) in self.cases():
            items = live_mission_items([
                waypoint(0, m.MAV_CMD_NAV_WAYPOINT, at[0][0], at[0][1], 100.0),
                waypoint(1, command, at[1][0], at[1][1], 100.0, params=params),
                waypoint(2, m.MAV_CMD_NAV_WAYPOINT, at[2][0], at[2][1], 100.0),
            ], params=self.PARAMS)
            assert (items[1].exit_converge is not None) == pulled_back, command

    def test_mavexplorer(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        log = TestLogMissionItems()
        for (command, params, pulled_back) in self.cases():
            items = log.resolve([
                log.item(0, m.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0)),
                log.item(1, command, params),
                log.item(2, m.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0)),
            ], self.PARAMS)
            assert (items[1].exit_converge is not None) == pulled_back, command


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
