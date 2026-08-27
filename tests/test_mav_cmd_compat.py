import tempfile
import os
import unittest

from pymavlink import mavutil
from pymavlink import mavwp

from MAVProxy.modules.lib import mav_cmd_compat
from MAVProxy.modules.mavproxy_misseditor import me_defines


class TestMavCmdCompat(unittest.TestCase):
    def test_attribute_registered(self):
        self.assertEqual(mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 36)
        self.assertEqual(mav_cmd_compat.MAV_CMD_NAV_ARC_WAYPOINT, 36)

    def test_enum_entry_registered(self):
        self.assertIn(36, mavutil.mavlink.enums['MAV_CMD'])
        entry = mavutil.mavlink.enums['MAV_CMD'][36]
        self.assertIs(entry.has_location, True)
        self.assertIn(1, entry.param)
        self.assertIn(5, entry.param)
        self.assertIn(6, entry.param)
        self.assertIn(7, entry.param)

    def test_misseditor_command_list(self):
        self.assertEqual(me_defines.miss_cmds.get(36), 'NAV_ARC_WAYPOINT')

    def test_misseditor_column_labels(self):
        labels = me_defines.get_column_labels('NAV_ARC_WAYPOINT')
        self.assertEqual(labels.get(1), 'Angle(deg)')
        self.assertEqual(labels.get(5), 'Lat')
        self.assertEqual(labels.get(6), 'Lon')
        self.assertEqual(labels.get(7), 'Alt')

    def test_wploader_is_location_command(self):
        loader = mavwp.MAVWPLoader()
        self.assertTrue(loader.is_location_command(36))

    def test_wploader_is_location_wp(self):
        loader = mavwp.MAVWPLoader()
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT,
            0, 1, 90.0, 0, 0, 0, -35.0, 149.0, 50.0)
        self.assertTrue(loader.is_location_wp(wp))


class TestArcWaypointMissionRoundTrip(unittest.TestCase):
    '''regression test for MAVProxy issue #1739: a MAV_CMD_NAV_ARC_WAYPOINT
    mission item must not be silently dropped from the drawn mission path,
    and its parameters must survive a save/load round trip.'''

    def _mission_item(self, seq, command, param1, lat, lon, alt):
        return mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, 0, 1, param1, 0, 0, 0, lat, lon, alt)

    def test_round_trip_preserves_arc_waypoint(self):
        loader = mavwp.MAVWPLoader()
        loader.add(self._mission_item(
            0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -35.0, 149.0, 20.0))
        loader.add(self._mission_item(
            1, mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90.0,
            -35.0005, 149.0005, 25.0))
        loader.add(self._mission_item(
            2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -35.001, 149.001, 20.0))

        fd, path = tempfile.mkstemp(suffix='.txt')
        os.close(fd)
        try:
            loader.save(path)

            reloaded = mavwp.MAVWPLoader()
            reloaded.load(path)

            self.assertEqual(reloaded.count(), 3)
            commands = [reloaded.wp(i).command for i in range(3)]
            self.assertEqual(commands, [
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            ])

            arc_wp = reloaded.wp(1)
            self.assertAlmostEqual(arc_wp.param1, 90.0, places=4)
            self.assertAlmostEqual(arc_wp.x, -35.0005, places=6)
            self.assertAlmostEqual(arc_wp.y, 149.0005, places=6)
            self.assertAlmostEqual(arc_wp.z, 25.0, places=4)

            # the actual #1739 bug: the arc waypoint used to be silently
            # excluded from the drawn mission path because pymavlink didn't
            # recognise it as a location-bearing command.
            self.assertEqual(reloaded.view_indexes(), [0, 1, 2])
        finally:
            os.remove(path)


if __name__ == '__main__':
    unittest.main()
