"""Draw uses the requested altitude/frame and remembers the dialog choices."""
from types import SimpleNamespace
import unittest
from unittest import mock

from pymavlink import mavutil

from MAVProxy.modules import mavproxy_wp
from MAVProxy.modules.lib import mp_menu


class WPDrawTests(unittest.TestCase):
    def setUp(self):
        self.draw_lines = mock.Mock()
        self.wp = mavproxy_wp.WPModule.__new__(mavproxy_wp.WPModule)
        self.wp.mpstate = SimpleNamespace(
            settings=SimpleNamespace(wpalt=100, terrainalt='False',
                                     target_system=1, target_component=1),
            map_functions={'draw_lines': self.draw_lines})
        self.wp.draw_frame = None
        self.wp.wploader_by_sysid = {}
        self.home = mavutil.mavlink.MAVLink_mission_item_message(
            1, 1, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0, -35.0, 149.0, 600)
        self.wp.get_WP0 = mock.Mock(return_value=self.home)
        self.wp.send_all_waypoints = mock.Mock()
        self.points = [(-35.01, 149.01), (-35.02, 149.02)]

    def draw(self, args):
        with mock.patch('builtins.print'):
            self.wp.cmd_draw(args)
        callback = self.draw_lines.call_args.args[0]
        callback(self.points)

    def assert_waypoints(self, altitude, frame):
        loader = self.wp.wploader
        self.assertEqual(loader.wp(0).z, 600)
        self.assertEqual(loader.wp(0).frame, mavutil.mavlink.MAV_FRAME_GLOBAL)
        for index, point in enumerate(self.points, start=loader.count() - 2):
            waypoint = loader.wp(index)
            self.assertEqual((waypoint.x, waypoint.y), point)
            self.assertEqual(waypoint.z, altitude)
            self.assertEqual(waypoint.frame, frame)
        self.wp.send_all_waypoints.assert_called()

    def test_single_altitude_argument_is_used(self):
        self.draw(['75'])
        self.assert_waypoints(75, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_all_frames_and_session_defaults(self):
        for name, frame in [('AboveHome', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                            ('AGL', mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT),
                            ('AMSL', mavutil.mavlink.MAV_FRAME_GLOBAL)]:
            with self.subTest(frame=name):
                self.draw(['250', name])
                self.assert_waypoints(250, frame)
                self.draw([])
                self.assert_waypoints(250, frame)

    def test_default_terrain_frame_and_explicit_override(self):
        self.wp.settings.terrainalt = 'True'
        self.draw([])
        self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.draw(['80', 'AboveHome'])
        self.assert_waypoints(80, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_automatic_frame_tracks_terrain_changes(self):
        self.wp.settings.terrainalt = 'Auto'
        self.wp.get_mav_param = mock.Mock(return_value=0)
        self.draw([])
        self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.wp.get_mav_param.return_value = 1
        self.draw([])
        self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.wp.settings.terrainalt = 'False'
        self.draw(['75'])
        self.assert_waypoints(75, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.wp.settings.terrainalt = 'True'
        self.draw([])
        self.assert_waypoints(75, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.assertIsNone(self.wp.draw_frame)

    def test_explicit_frame_is_remembered_until_default_selected(self):
        self.draw(['80', 'AMSL'])
        self.wp.settings.terrainalt = 'True'
        self.draw(['90'])
        self.assert_waypoints(90, mavutil.mavlink.MAV_FRAME_GLOBAL)
        self.draw(['95', 'Default'])
        self.assert_waypoints(95, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.wp.settings.terrainalt = 'False'
        self.draw([])
        self.assert_waypoints(95, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_active_draw_keeps_frame_resolved_at_start(self):
        with mock.patch('builtins.print'):
            self.wp.cmd_draw([])
        self.wp.settings.terrainalt = 'True'
        self.draw_lines.call_args.args[0](self.points)
        self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_invalid_arguments_do_not_start_drawing(self):
        for args in [['bad'], ['75', 'bad'], ['75', 'AGL', 'extra']]:
            with self.subTest(args=args), mock.patch('builtins.print'):
                self.wp.cmd_draw(args)
                self.draw_lines.assert_not_called()
                self.assertEqual(self.wp.settings.wpalt, 100)
                self.assertIsNone(self.wp.draw_frame)

    @unittest.skipUnless(mavproxy_wp.mp_util.has_wxpython, 'requires wxPython')
    def test_draw_dialog_remembers_choices_and_cancel_preserves_them(self):
        from MAVProxy.modules.lib import wx_loader

        def draw_handler():
            return next(item.handler for item in self.wp.gui_menu_items()
                        if item.name == 'Draw')

        wx = mock.MagicMock()
        for name in ('VERTICAL', 'HORIZONTAL', 'ALL', 'ALIGN_CENTER_VERTICAL',
                     'EXPAND', 'OK', 'CANCEL', 'ALIGN_CENTER', 'ID_OK'):
            setattr(wx, name, 1)
        wx.Dialog.return_value.ShowModal.return_value = wx.ID_OK
        wx.TextCtrl.return_value.GetValue.return_value = '275'
        wx.Choice.return_value.GetSelection.return_value = 3

        with mock.patch.object(wx_loader, 'wx', wx), \
                mock.patch.dict(mp_menu.last_dropdown_selection, {}, clear=True), \
                mock.patch.dict(mp_menu.last_value_selection, {}, clear=True):
            handler = draw_handler()
            self.assertEqual(handler.dropdown_options, ['Default', 'AboveHome', 'AGL', 'AMSL'])
            self.draw(handler.call().split())
            self.assert_waypoints(275, mavutil.mavlink.MAV_FRAME_GLOBAL)

            # Menu reconstruction must preserve accepted choices as well.
            wx.Dialog.return_value.ShowModal.return_value = 0
            self.assertIsNone(draw_handler().call())
            self.assertEqual(wx.TextCtrl.call_args.kwargs['value'], '275')
            wx.Choice.return_value.SetSelection.assert_called_with(3)
            self.assertEqual(mp_menu.last_value_selection[handler.title], '275')
            self.assertEqual(mp_menu.last_dropdown_selection[handler.title], 3)

    @unittest.skipUnless(mavproxy_wp.mp_util.has_wxpython, 'requires wxPython')
    def test_dialog_default_resolves_after_parameters_arrive(self):
        from MAVProxy.modules.lib import wx_loader

        self.wp.settings.terrainalt = 'Auto'
        self.wp.get_mav_param = mock.Mock(return_value=0)
        handler = next(item.handler for item in self.wp.gui_menu_items()
                       if item.name == 'Draw')
        self.wp.get_mav_param.return_value = 1

        wx = mock.MagicMock()
        for name in ('VERTICAL', 'HORIZONTAL', 'ALL', 'ALIGN_CENTER_VERTICAL',
                     'EXPAND', 'OK', 'CANCEL', 'ALIGN_CENTER', 'ID_OK'):
            setattr(wx, name, 1)
        wx.Dialog.return_value.ShowModal.return_value = wx.ID_OK
        wx.TextCtrl.return_value.GetValue.return_value = '100'
        wx.Choice.return_value.GetSelection.side_effect = (
            lambda: wx.Choice.return_value.SetSelection.call_args.args[0])

        with mock.patch.object(wx_loader, 'wx', wx), \
                mock.patch.dict(mp_menu.last_dropdown_selection, {}, clear=True), \
                mock.patch.dict(mp_menu.last_value_selection, {}, clear=True):
            self.draw(handler.call().split())
            self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
            self.wp.settings.terrainalt = 'False'
            self.draw(handler.call().split())
            self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
            self.wp.settings.terrainalt = 'True'
            self.draw(handler.call().split())
            self.assert_waypoints(100, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)


if __name__ == '__main__':
    unittest.main()
