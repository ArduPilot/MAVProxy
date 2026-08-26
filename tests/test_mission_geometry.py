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
