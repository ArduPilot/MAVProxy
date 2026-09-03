#!/usr/bin/env python3
"""Regression tests for finite, well-ordered camera ground footprints."""

import math
import os
from pathlib import Path
import sys
import unittest


os.environ.setdefault("MAVLINK20", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from MAVProxy.modules.lib import camera_projection  # noqa: E402
from MAVProxy.modules.lib import mp_util  # noqa: E402


class FlatElevationModel:
    def GetElevation(self, _latitude, _longitude):
        return 0.0


class CameraProjectionTest(unittest.TestCase):
    latitude = -35.0
    longitude = 149.0
    altitude = 100.0

    def projection(self, hfov=88, width=1920, height=1080):
        params = camera_projection.CameraParams(
            xresolution=width, yresolution=height, FOV=hfov)
        return camera_projection.CameraProjection(
            params, elevation_model=FlatElevationModel())

    def assert_sane(self, points, max_range):
        self.assertGreaterEqual(len(points), 4)
        self.assertEqual(points[0], points[-1])
        for latitude, longitude in points:
            self.assertTrue(math.isfinite(latitude))
            self.assertTrue(math.isfinite(longitude))
            self.assertLessEqual(
                mp_util.gps_distance(self.latitude, self.longitude,
                                     latitude, longitude),
                max_range)

        def side(a, b, c):
            return ((b[0] - a[0]) * (c[1] - a[1]) -
                    (b[1] - a[1]) * (c[0] - a[0]))

        def crosses(a, b, c, d):
            return (side(a, b, c) * side(a, b, d) < 0 and
                    side(c, d, a) * side(c, d, b) < 0)

        self.assertFalse(crosses(points[0], points[1],
                                 points[2], points[3]))
        self.assertFalse(crosses(points[1], points[2],
                                 points[3], points[0]))

    def test_nadir_projection_is_finite(self):
        points = self.projection().get_projection(
            self.latitude, self.longitude, self.altitude,
            0, -90, 0, max_range=10000)
        self.assert_sane(points, 10000)

    def test_partial_sky_view_uses_earth_frame_range_clip(self):
        # The old implementation moved sky-facing corners independently and
        # returned a crossed, globe-spanning polygon.
        points = self.projection().get_projection(
            self.latitude, self.longitude, self.altitude,
            45, 0, 0, max_range=10000)
        self.assert_sane(points, 10000)

    def test_near_horizon_projection_obeys_range_limit(self):
        projection = self.projection()
        points = projection.get_projection(
            self.latitude, self.longitude, self.altitude,
            -35, -64, 0)
        self.assert_sane(points, 10000)

    def test_earth_clip_preserves_nested_fields_of_view(self):
        wide = self.projection(88, 1920, 1080)
        narrow = self.projection(24, 1280, 720)

        def local_ne(point):
            distance = mp_util.gps_distance(
                self.latitude, self.longitude, *point)
            bearing = math.radians(mp_util.gps_bearing(
                self.latitude, self.longitude, *point))
            return (distance * math.cos(bearing),
                    distance * math.sin(bearing))

        def inside_convex(point, polygon):
            signs = []
            for index, start in enumerate(polygon):
                end = polygon[(index + 1) % len(polygon)]
                cross = ((end[0] - start[0]) * (point[1] - start[1]) -
                         (end[1] - start[1]) * (point[0] - start[0]))
                if abs(cross) > 1.0e-4:
                    signs.append(cross > 0)
            return not signs or all(sign == signs[0] for sign in signs)

        for roll, pitch in ((0, -90), (45, 0), (-35, -64), (30, -30)):
            wide_points = wide.get_projection(
                self.latitude, self.longitude, self.altitude,
                roll, pitch, 0, max_range=1000)
            narrow_points = narrow.get_projection(
                self.latitude, self.longitude, self.altitude,
                roll, pitch, 0, max_range=1000)
            wide_ne = [local_ne(point) for point in wide_points[:-1]]
            narrow_ne = [local_ne(point) for point in narrow_points[:-1]]
            self.assertTrue(all(inside_convex(point, wide_ne)
                                for point in narrow_ne))

    def test_attitude_grid_never_returns_invalid_polygon(self):
        for hfov, width, height in ((88, 1920, 1080), (24, 1280, 720)):
            projection = self.projection(hfov, width, height)
            for roll in range(-180, 181, 15):
                for pitch in range(-90, 31, 5):
                    points = projection.get_projection(
                        self.latitude, self.longitude, self.altitude,
                        roll, pitch, 0, max_range=10000)
                    if points is not None:
                        self.assert_sane(points, 10000)


if __name__ == "__main__":
    unittest.main()
