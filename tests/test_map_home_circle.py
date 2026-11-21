"""a home-centred fence circle is drawn only once home is known"""
from pathlib import Path
import sys
from types import SimpleNamespace
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
        self.map.map.icon.return_value = SimpleNamespace(shape=(32, 32, 3))
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

    def with_master(self, master):
        self.map.mpstate = SimpleNamespace(master=lambda: master)

    def test_home_received_before_map_loaded_is_used(self):
        home = mavutil.mavlink.MAVLink_home_position_message(
            -353648096, 1487110656, 971170, 0, 0, 0, [1, 0, 0, 0], 0, 0, 0)
        self.with_master(SimpleNamespace(messages={'HOME_POSITION': home}))
        self.map.load_received_home_position()
        expected = (home.latitude*1.0e-7, home.longitude*1.0e-7)
        self.assertEqual(self.map.home_pos, expected)
        self.assertEqual(self.drawn()['HOME_POSITION'], expected)
        self.map.display_polyfences_circles([self.home_circle], (0, 255, 96))
        self.assertEqual(self.drawn()['5:circle'], expected)

    def test_no_home_received_leaves_home_unknown(self):
        for master in (None, SimpleNamespace(messages={})):
            with self.subTest(master=master):
                self.with_master(master)
                self.map.load_received_home_position()
                self.assertEqual(self.map.home_pos, ())
                self.map.map.add_object.assert_not_called()


if __name__ == '__main__':
    unittest.main()
