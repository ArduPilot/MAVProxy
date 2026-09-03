#!/usr/bin/env python3
"""Focused tests for SIYI map projection integration."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from MAVProxy.modules.mavproxy_SIYI import SIYIModule  # noqa: E402


class FakeMap:
    def __init__(self):
        self.objects = {}

    def add_object(self, obj):
        self.objects[obj.key] = obj


class SIYIProjectionTest(unittest.TestCase):
    @mock.patch("MAVProxy.modules.mavproxy_SIYI.camera_projection.CameraProjection")
    def test_projection_uses_range_limit_without_vertex_markers(self,
                                                                projection_type):
        points = [(-35.0, 149.0), (-35.0, 149.1),
                  (-35.1, 149.1), (-35.0, 149.0)]
        projection_type.return_value.get_projection.return_value = points
        map_display = FakeMap()
        terrain = SimpleNamespace(ElevationModel=object())
        messages = {
            "ATTITUDE": SimpleNamespace(yaw=math.radians(20)),
            "GLOBAL_POSITION_INT": SimpleNamespace(
                lat=-350000000, lon=1490000000),
            "GPS_RAW_INT": SimpleNamespace(alt=600000),
        }
        siyi = SimpleNamespace(
            siyi_settings=SimpleNamespace(
                mount_alt=2.0, fov_max_range=4321.0),
            mpstate=SimpleNamespace(map=map_display),
            master=SimpleNamespace(messages=messages),
            module=lambda name: terrain if name == "terrain" else None,
            get_fov_attitude=lambda: (1.0, -30.0, 5.0))

        SIYIModule.show_fov1(siyi, 88.0, "FOV_RGB", 16.0/9.0,
                             (0, 128, 128))

        projection_type.return_value.get_projection.assert_called_once_with(
            -35.0, 149.0, 602.0, 1.0, -30.0, 25.0,
            max_range=4321.0)
        polygon = map_display.objects["FOV_RGB"]
        self.assertEqual(polygon.points, points)
        self.assertFalse(polygon._showcircles)


if __name__ == "__main__":
    unittest.main()
