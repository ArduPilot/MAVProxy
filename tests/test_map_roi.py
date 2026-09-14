"""ROI commands require a known terrain altitude and successful MAVLink send."""
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules.mavproxy_map import MapModule
from pymavlink import mavutil


class MapROITests(unittest.TestCase):
    def setUp(self):
        self.output = io.BytesIO()
        self.mav = mavutil.mavlink.MAVLink(self.output)
        self.elevation = mock.Mock()
        self.terrain = SimpleNamespace(ElevationModel=self.elevation)
        self.map = MapModule.__new__(MapModule)
        self.map.mpstate = SimpleNamespace(
            click_location=(-35.27845654154013, 148.95347139693465),
            settings=SimpleNamespace(target_system=1, target_component=1),
            master=lambda: SimpleNamespace(mav=self.mav),
            module=lambda name: self.terrain if name == 'terrain' else None)
        self.previous_roi = (-35.0, 149.0, 600.0)
        self.map.current_ROI = self.previous_roi

    def test_unavailable_terrain_does_not_send_or_change_roi(self):
        for altitude in (None, float('nan'), float('inf'), -float('inf')):
            with self.subTest(altitude=altitude):
                self.elevation.GetElevation.return_value = altitude
                with mock.patch('builtins.print') as report:
                    self.map.cmd_set_roi([])
                self.assertIn('terrain elevation unavailable', report.call_args.args[0])
                self.assertEqual(self.output.getvalue(), b'')
                self.assertEqual(self.map.current_ROI, self.previous_roi)

    def test_missing_terrain_module_is_reported(self):
        self.terrain = None
        with mock.patch('builtins.print') as report:
            self.map.cmd_set_roi([])
        report.assert_called_once_with('Unable to set ROI: terrain module is not loaded')
        self.assertEqual(self.output.getvalue(), b'')
        self.assertEqual(self.map.current_ROI, self.previous_roi)

    def test_known_elevation_sends_packable_roi_including_sea_level(self):
        for altitude in (624.5, 0.0, -15.0):
            with self.subTest(altitude=altitude):
                self.output.seek(0)
                self.output.truncate()
                self.elevation.GetElevation.return_value = altitude
                with mock.patch('builtins.print'):
                    self.map.cmd_set_roi([])
                message = mavutil.mavlink.MAVLink(None).parse_buffer(self.output.getvalue())[0]
                self.assertEqual(message.command, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
                self.assertEqual(message.frame, mavutil.mavlink.MAV_FRAME_GLOBAL)
                lat, lon = self.map.mpstate.click_location
                self.assertEqual((message.x, message.y, message.z), (int(lat*1e7), int(lon*1e7), altitude))
                self.assertEqual(self.map.current_ROI, (lat, lon, altitude))

    def test_failed_send_preserves_previous_roi(self):
        self.elevation.GetElevation.return_value = 624.5
        with mock.patch.object(self.mav, 'command_int_send', side_effect=OSError('link closed')), \
                mock.patch('builtins.print'):
            with self.assertRaisesRegex(OSError, 'link closed'):
                self.map.cmd_set_roi([])
        self.assertEqual(self.map.current_ROI, self.previous_roi)


if __name__ == '__main__':
    unittest.main()
