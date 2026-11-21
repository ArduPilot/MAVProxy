"""a home-centred fence circle is drawn only once home is known"""
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules.mavproxy_map import MapModule, MAV_CMD_NAV_FENCE_HOME_CIRCLE_INCLUSION
# MapModule.__init__ imports this, making it a global of the map package
from MAVProxy.modules.mavproxy_map import mp_slipmap  # noqa: F401
from pymavlink import mavutil


def circle(seq, command, radius, lat=0.0, lng=0.0):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        1, 1, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        command,
        0, 0,
        radius, 0, 0, 0,
        int(lat*1e7), int(lng*1e7), 0)


class MapHomeCircleTests(unittest.TestCase):
    def setUp(self):
        self.map = MapModule.__new__(MapModule)
        self.map.map = mock.Mock()
        self.map.home_pos = ()
        self.home_circle = circle(5, MAV_CMD_NAV_FENCE_HOME_CIRCLE_INCLUSION, 500)
        self.other_circle = circle(4, mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, 10, -35.28, 149.005)

    def drawn(self):
        return {c.args[0].key: c.args[0].latlon for c in self.map.map.add_object.call_args_list}

    def test_home_circle_skipped_until_home_is_known(self):
        self.map.display_polyfences_circles([self.home_circle, self.other_circle], (0, 255, 96))
        drawn = self.drawn()
        self.assertNotIn('5:circle', drawn)
        self.assertIn('4:circle', drawn)

    def test_home_circle_drawn_at_home(self):
        self.map.home_pos = (-35.36, 148.71)
        self.map.display_polyfences_circles([self.home_circle], (0, 255, 96))
        self.assertEqual(self.drawn()['5:circle'], (-35.36, 148.71))


if __name__ == '__main__':
    unittest.main()
