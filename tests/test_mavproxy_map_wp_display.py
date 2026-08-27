import importlib.util
import inspect
import types
import unittest
from pathlib import Path

from pymavlink import mavutil


# Some developer environments have a released MAVProxy imported by a pytest
# plugin before collection begins.  Load the worktree file explicitly so the
# tests always exercise the code they accompany.
MAP_MODULE_PATH = (Path(__file__).resolve().parents[1] /
                    'MAVProxy/modules/mavproxy_map/__init__.py')
MAP_SPEC = importlib.util.spec_from_file_location(
    'mavproxy_map_under_test', MAP_MODULE_PATH)
mavproxy_map = importlib.util.module_from_spec(MAP_SPEC)
MAP_SPEC.loader.exec_module(mavproxy_map)


class FakeWPLoader:
    '''Minimal stand-in for pymavlink's MAVWPLoader: colour_for_wp() and
    label_for_waypoint() only ever call wploader.wp(n) to get the command
    for a given waypoint index.'''
    def __init__(self, commands):
        self._commands = commands

    def wp(self, n):
        return types.SimpleNamespace(command=self._commands[n])


class FakeWPModule:
    def __init__(self, commands):
        self.wploader = FakeWPLoader(commands)


def make_map_module(commands):
    '''Build a MapModule instance without running its real __init__ (which
    needs a live mpstate/wx environment), just enough state for
    colour_for_wp()/label_for_waypoint() to run.'''
    MapModule = mavproxy_map.MapModule
    inst = object.__new__(MapModule)
    inst._colour_for_wp_command = {
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: (0, 255, 255),
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT: (64, 255, 64),
        mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT: (255, 0, 255),
    }
    inst._label_suffix_for_wp_command = {
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT: "SW",
        mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT: "AW",
    }
    fake_wp_module = FakeWPModule(commands)
    inst.module = lambda name: fake_wp_module
    return inst


class TestMapWpCommandDicts(unittest.TestCase):
    '''regression test for MAVProxy issue #1739: MAV_CMD_NAV_ARC_WAYPOINT
    must get a colour/label distinct from a plain waypoint's, matching the
    existing precedent for MAV_CMD_NAV_SPLINE_WAYPOINT.'''

    def test_init_source_registers_arc_waypoint_entries(self):
        # MapModule.__init__ builds self._colour_for_wp_command and
        # self._label_suffix_for_wp_command inline; full instantiation
        # needs a live mpstate/wx environment (not available headlessly),
        # so check the real source directly rather than a hand-duplicated
        # copy of the dicts, to catch a regression if the entries are
        # ever removed from __init__ itself.
        source = inspect.getsource(mavproxy_map.MapModule.__init__)
        self.assertIn(
            "mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT: (255, 0, 255)",
            source)
        self.assertIn(
            'mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT: "AW"',
            source)
        self.assertEqual(mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 36)

    def test_colour_for_wp_distinguishes_arc_from_plain_waypoint(self):
        commands = {
            0: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1: mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT,
        }
        inst = make_map_module(commands)
        plain_colour = inst.colour_for_wp(0)
        arc_colour = inst.colour_for_wp(1)
        self.assertNotEqual(plain_colour, arc_colour)
        self.assertEqual(arc_colour, (255, 0, 255))

    def test_label_for_waypoint_arc_waypoint_gets_suffix(self):
        commands = {
            0: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1: mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT,
        }
        inst = make_map_module(commands)
        self.assertEqual(inst.label_for_waypoint(0), "0")
        self.assertEqual(inst.label_for_waypoint(1), "1(AW)")


if __name__ == '__main__':
    unittest.main()
