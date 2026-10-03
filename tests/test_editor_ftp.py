"""Editor transfer regressions using real parameter/mission FTP encoders."""
import io
from pathlib import Path
import queue
import struct
import sys
import threading
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink import mavparm, mavutil, mavwp  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega  # noqa: E402
from MAVProxy.modules import mavproxy_param, mavproxy_wp  # noqa: E402
from MAVProxy.modules import mavproxy_fence, mavproxy_rally  # noqa: E402
from MAVProxy.modules.mavproxy_paramedit import param_editor, ph_event  # noqa: E402
from MAVProxy.modules.mavproxy_misseditor import mission_editor, me_event  # noqa: E402


class EditorFTPTests(unittest.TestCase):
    def setUp(self):
        dialect = mock.patch.object(mavutil, 'mavlink', ardupilotmega)
        dialect.start()
        self.addCleanup(dialect.stop)
        self.ftp = mock.Mock()
        self.master = mock.Mock()
        self.modules = {'ftp': self.ftp}
        self.params = mavparm.MAVParmDict()
        self.params.update({'CHANGED': 1.0, 'UNCHANGED': 2.0, 'OTHER': 3.0})
        self.state = types.SimpleNamespace(
            module=self.modules.get, master=lambda: self.master,
            mav_master=[self.master], mav_param=self.params,
            settings=types.SimpleNamespace(target_system=1, target_component=1,
                                           param_ftp=True, wp_use_mission_int=True),
            status=types.SimpleNamespace(logdir=None), console=mock.Mock(),
            vehicle_name='ArduCopter', logqueue=None)
        self.pstate = mavproxy_param.ParamState(
            self.params, None, 'ArduCopter', 'mav.parm', self.state, (1, 1))
        self.param = mavproxy_param.ParamModule.__new__(mavproxy_param.ParamModule)
        self.param.mpstate = self.state
        self.param.pstate = {(1, 1): self.pstate}
        self.modules['param'] = self.param
        with mock.patch.object(param_editor.ParamEditorMain, 'start'):
            self.editor = param_editor.ParamEditorMain(self.state)
        self.editor.gui_event_queue = queue.Queue()
        self.wp = mavproxy_wp.WPModule.__new__(mavproxy_wp.WPModule)
        self.wp.mpstate = self.state
        self.wp.wploader_by_sysid = {1: mavwp.MAVWPLoader()}
        self.wp.loading_waypoints = False
        self.modules['wp'] = self.wp
        self.mission = types.SimpleNamespace(
            mpstate=self.state, gui_event_queue=queue.Queue(),
            gui_event_queue_lock=threading.Lock(), num_wps_expected=0,
            wps_received={})
        self.mission_events = queue.Queue()
        self.mission_thread = mission_editor.MissionEditorEventThread(
            self.mission, self.mission_events, threading.Lock())

    def run_param_event(self, kind, process_ftp=True, **args):
        events = queue.Queue()
        events.put(ph_event.ParamEditorEvent(kind, **args))
        worker = param_editor.ParamEditorEventThread(self.editor, events, threading.Lock())
        with mock.patch.object(param_editor.time, 'sleep',
                               side_effect=lambda _: setattr(worker, 'time_to_quit', True)):
            worker.run()
        if process_ftp:
            self.editor.process_ftp_requests()

    def run_mission_events(self, events, process_ftp=True):
        for event in events:
            self.mission_events.put(event)
        self.mission_thread.time_to_quit = False
        with mock.patch.object(mission_editor.time, 'sleep', side_effect=lambda _:
                               setattr(self.mission_thread, 'time_to_quit', True)):
            self.mission_thread.run()
        self.mission_thread.time_to_quit = False
        if process_ftp:
            self.mission_thread.process_ftp_requests()

    def param_download(self, values):
        data = struct.pack('<HHH', 0x671b, len(values), len(values))
        for name, value in values.items():
            data += struct.pack('<BB', 4, (len(name)-1) << 4)
            data += name.encode() + struct.pack('<f', value)
        return io.BytesIO(data)

    def assert_param_status(self, message):
        event = self.editor.gui_event_queue.get_nowait()
        self.assertEqual(event.get_type(), ph_event.PEGE_FTP_TRANSFER)
        self.assertIn(message, event.get_arg('message'))
        self.assertTrue(self.editor.gui_event_queue.empty())

    def assert_mission_status(self, success, message):
        event = self.mission.gui_event_queue.get_nowait()
        self.assertEqual(event.get_type(), me_event.MEGE_FTP_TRANSFER)
        self.assertEqual(event.get_arg('success'), success)
        self.assertIn(message, event.get_arg('message'))
        self.assertTrue(self.mission.gui_event_queue.empty())

    def test_param_write_button_uses_param_set_with_either_checkbox_state(self):
        from MAVProxy.modules.mavproxy_paramedit.param_editor_frame import ParamEditorFrame
        for use_ftp in (True, False):
            with self.subTest(use_ftp=use_ftp):
                frame = types.SimpleNamespace(
                    checkbox_mavftp=mock.Mock(), modified_param={'CHANGED': 4.0, 'OTHER': 5.0},
                    event_queue=queue.Queue(), display_list=mock.Mock(), ftp_transfer_started=mock.Mock())
                frame.checkbox_mavftp.GetValue.return_value = use_ftp
                frame.display_list.GetNumberRows.return_value = 0
                ParamEditorFrame.write_param(frame, mock.Mock())
                event = frame.event_queue.get_nowait()
                self.assertEqual(event.get_type(), ph_event.PEE_WRITE_PARAM)
                frame.ftp_transfer_started.assert_not_called()
                self.run_param_event(event.get_type(), **event.arg_dict)
                self.assertEqual(self.master.param_set_send.call_args_list,
                                 [mock.call('CHANGED', 4.0), mock.call('OTHER', 5.0)])
                self.master.param_set_send.reset_mock()
                self.assertEqual(self.editor.paramchanged, frame.modified_param)
                self.assertTrue(self.editor.gui_event_queue.empty())
                self.ftp.cmd_put.assert_not_called()
                self.ftp.cmd_get.assert_not_called()
                self.master.param_fetch_all.assert_not_called()
                self.master.mav.param_request_read_send.assert_not_called()

    def test_param_write_acknowledges_only_matching_param_value(self):
        self.run_param_event(ph_event.PEE_WRITE_PARAM, modparam={'CHANGED': 4.0})
        for value in (1.0, 4.0):
            m = mavutil.mavlink.MAVLink_param_value_message(
                b'CHANGED', value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 3, 0)
            self.editor.process_mavlink_packet(m)
            if value == 1.0:
                self.assertEqual(self.editor.paramchanged, {'CHANGED': 4.0})
                self.assertTrue(self.editor.gui_event_queue.empty())
            else:
                self.assertEqual(self.editor.paramchanged, {})
                event = self.editor.gui_event_queue.get_nowait()
                self.assertEqual(event.get_type(), ph_event.PEGE_WRITE_SUCC)
                self.assertEqual(event.get_arg('paramvalue'), 4.0)

    def test_param_fetch_ftp_updates_editor_even_without_logqueue(self):
        self.state.settings.param_ftp = False
        self.pstate.ftp_failed = True
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
        self.master.param_fetch_all.assert_not_called()
        # One int8 parameter named NEW.
        data = struct.pack('<HHHBB', 0x671b, 1, 1, 1, 2 << 4) + b'NEW\x07'
        self.ftp.cmd_get.call_args.kwargs['callback'](io.BytesIO(data))
        event = self.editor.gui_event_queue.get_nowait()
        self.assertEqual(event.get_type(), ph_event.PEGE_READ_PARAM)
        self.assertEqual(event.get_arg('param'), {'NEW': 7})
        self.assertEqual(event.get_arg('pstatus'), (1, 1))
        self.assertEqual(self.params, {'NEW': 7})
        self.assert_param_status('Read succeeded (1 parameters)')

    def test_param_fetch_without_ftp_overrides_global_setting_and_retries(self):
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=False)
        self.master.param_fetch_all.assert_called_once_with()
        self.ftp.cmd_get.assert_not_called()
        self.assertFalse(self.pstate.use_ftp())
        self.pstate.fetch_all(self.master)
        self.ftp.cmd_get.assert_called_once()

    def test_failed_param_fetch_does_not_replace_editor_contents(self):
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
        self.ftp.cmd_get.call_args.kwargs['callback'](None)
        self.assert_param_status('Read failed')
        self.assertEqual(self.params['CHANGED'], 1.0)

    def mission_item_event(self, seq):
        return me_event.MissionEditorEvent(
            me_event.MEE_WRITE_WP_NUM, num=seq, frame=3, cmd_id=16,
            p1=0, p2=0, p3=0, p4=0, lat=-35.0 + seq, lon=149.0, alt=100 + seq)

    def test_mission_ftp_waits_for_all_rows_and_round_trips_into_editor(self):
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=True, count=2),
            self.mission_item_event(0)])
        self.ftp.cmd_put.assert_not_called()
        self.run_mission_events([self.mission_item_event(1)])
        self.ftp.cmd_put.assert_called_once()
        self.master.waypoint_count_send.assert_not_called()
        self.master.mav.send.assert_not_called()
        data = self.ftp.cmd_put.call_args.kwargs['fh'].getvalue()
        self.assertEqual(struct.unpack('<HHHHH', data[:10]), (0x763d, 0, 0, 0, 2))
        self.wp.wploader.clear()
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
        # Simulate completion while the editor event thread is running.
        self.mission_thread.time_to_quit = False
        self.ftp.cmd_get.call_args.kwargs['callback'](io.BytesIO(data))
        self.run_mission_events([])
        events = []
        while not self.mission.gui_event_queue.empty():
            events.append(self.mission.gui_event_queue.get_nowait())
        self.assertEqual([e.get_type() for e in events], [
            me_event.MEGE_FTP_MISSION])
        loader = events[0].get_arg('wploader')
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(1).x, -34.0)
        self.assertEqual(loader.wp(1).z, 101)

    def test_mission_standard_transfer_paths(self):
        with mock.patch.object(self.wp, 'cmd_wp') as command:
            self.run_mission_events([
                me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=False)])
            command.assert_called_once_with(['list'])
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=False, count=1),
            self.mission_item_event(0)])
        self.master.waypoint_count_send.assert_called_once_with(1)
        self.master.mav.send.assert_called_once()
        self.ftp.cmd_get.assert_not_called()
        self.ftp.cmd_put.assert_not_called()

    def test_empty_and_failed_mission_downloads(self):
        for data in (None, io.BytesIO(struct.pack('<HHHHH', 0x763d, 0, 0, 0, 0))):
            self.run_mission_events([
                me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
            self.mission_thread.time_to_quit = False
            self.ftp.cmd_get.call_args.kwargs['callback'](data)
            self.run_mission_events([])
            if data is None:
                self.assert_mission_status(False, 'Read failed')
            else:
                event = self.mission.gui_event_queue.get_nowait()
                self.assertEqual(event.get_type(), me_event.MEGE_FTP_MISSION)
                self.assertEqual(event.get_arg('wploader').count(), 0)
                self.assertTrue(self.mission.gui_event_queue.empty())

    def test_mission_write_completion_reports_success_and_failure(self):
        for result in (None, 48):
            self.run_mission_events([
                me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=True, count=1),
                self.mission_item_event(0)])
            self.mission_thread.time_to_quit = False
            self.ftp.cmd_put.call_args.kwargs['callback'](result)
            self.assert_mission_status(result is not None,
                                       'Write failed' if result is None else 'Write succeeded')

    def test_missing_ftp_module_reports_failures_in_both_editors(self):
        del self.modules['ftp']
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
        self.assert_param_status('Read failed')
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
        self.assert_mission_status(False, 'Read failed')
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=True, count=1),
            self.mission_item_event(0)])
        self.assert_mission_status(False, 'Write failed')

    def test_invalid_downloads_report_failure_without_replacing_data(self):
        for data in (b'', struct.pack('<HHHBB', 0x671b, 1, 1, 4, 0)):
            self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
            self.ftp.cmd_get.call_args.kwargs['callback'](io.BytesIO(data))
            self.assert_param_status('Read failed')
            self.assertEqual(self.params['CHANGED'], 1.0)
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=False, count=1),
            self.mission_item_event(0)])
        for data in (b'', struct.pack('<HHHHH', 0x763d, 0, 0, 0, 1)):
            self.run_mission_events([
                me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
            self.mission_thread.time_to_quit = False
            self.ftp.cmd_get.call_args.kwargs['callback'](io.BytesIO(data))
            self.assert_mission_status(False, 'Read failed')
            self.assertEqual(self.wp.wploader.count(), 1)

    def test_transfer_start_exceptions_are_reported(self):
        self.ftp.cmd_get.side_effect = RuntimeError('Disconnected')
        self.ftp.cmd_put.side_effect = RuntimeError('Disconnected')
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
        self.assert_param_status('Read failed: Disconnected')
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
        self.assert_mission_status(False, 'Read failed: Disconnected')
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=True, count=1),
            self.mission_item_event(0)])
        self.assert_mission_status(False, 'Write failed: Disconnected')

    def test_defaults_follow_vehicle_and_refresh_without_serializing_live_state(self):
        self.pstate.default_params = {'CHANGED': 0.0}
        self.editor.update_default_params()
        event = self.editor.gui_event_queue.get_nowait()
        self.assertEqual(event.get_type(), ph_event.PEGE_DEFAULTS)
        self.assertEqual(event.get_arg('defaults'), {'CHANGED': 0.0})
        self.assertIsNot(event.get_arg('defaults'), self.pstate.default_params)
        self.editor.update_default_params()
        self.assertTrue(self.editor.gui_event_queue.empty())
        self.pstate.default_params = {'CHANGED': 1.0}
        self.editor.update_default_params()
        self.assertEqual(self.editor.gui_event_queue.get_nowait().get_arg('defaults'), {'CHANGED': 1.0})
        self.state.settings.target_system = 2
        self.editor.update_default_params()
        self.assertEqual(self.editor.gui_event_queue.get_nowait().get_arg('defaults'), {})

    def test_ftpload_cli_requires_a_filename_for_all_mission_types(self):
        for cls in (mavproxy_wp.WPModule, mavproxy_fence.FenceModule, mavproxy_rally.RallyModule):
            module = cls.__new__(cls)
            module.mpstate = self.state
            with mock.patch('builtins.print') as report:
                module.cmd_wp(['ftpload'])
            report.assert_called_once_with('Usage: %s ftpload FILENAME' % module.command_name())
        self.ftp.cmd_put.assert_not_called()

    def test_explicit_editor_upload_can_clear_a_mission(self):
        self.wp.ftp_upload(mavwp.MAVWPLoader())
        data = self.ftp.cmd_put.call_args.kwargs['fh'].getvalue()
        self.assertEqual(struct.unpack('<HHHHH', data), (0x763d, 0, 0, 0, 0))

    def test_parameter_fetch_after_vehicle_switch_is_discarded(self):
        self.run_param_event(ph_event.PEE_FETCH, use_ftp=True)
        callback = self.ftp.cmd_get.call_args.kwargs['callback']
        self.state.settings.target_system = 2
        self.state.vehicle_name = 'ArduPlane'
        callback(self.param_download({'CHANGED': 9.0}))
        self.assert_param_status('Read discarded: vehicle changed')

    def test_editor_threads_defer_ftp_submission_to_main_loop(self):
        main_thread = threading.get_ident()
        called_from = []
        self.ftp.cmd_get.side_effect = lambda *a, **kw: called_from.append(threading.get_ident())
        self.ftp.cmd_put.side_effect = lambda *a, **kw: called_from.append(threading.get_ident())
        for operation in ('Read', 'Write'):
            if operation == 'Read':
                worker = threading.Thread(target=lambda: self.run_param_event(
                    ph_event.PEE_FETCH, process_ftp=False, use_ftp=True))
                worker.start()
                worker.join(5)
                self.assertFalse(worker.is_alive())
            events = [me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)]
            if operation == 'Write':
                events = [me_event.MissionEditorEvent(me_event.MEE_WRITE_WPS, use_ftp=True, count=1),
                          self.mission_item_event(0)]
            worker = threading.Thread(target=lambda: self.run_mission_events(events, process_ftp=False))
            worker.start()
            worker.join(5)
            self.assertFalse(worker.is_alive())
            self.assertEqual(called_from, [])
            self.editor.process_ftp_requests()
            self.mission_thread.process_ftp_requests()
            self.assertEqual(called_from, [main_thread] * (2 if operation == 'Read' else 1))
            called_from.clear()

    def test_vehicle_switch_before_submission_cancels_queued_transfers(self):
        self.run_param_event(ph_event.PEE_FETCH, process_ftp=False, use_ftp=True)
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)], process_ftp=False)
        self.state.settings.target_system = 2
        self.editor.process_ftp_requests()
        self.mission_thread.process_ftp_requests()
        self.ftp.cmd_get.assert_not_called()
        self.assert_param_status('Read cancelled: vehicle changed')
        self.assert_mission_status(False, 'Read cancelled: vehicle changed')

    def test_mission_download_after_vehicle_switch_does_not_replace_loader(self):
        self.run_mission_events([
            me_event.MissionEditorEvent(me_event.MEE_READ_WPS, use_ftp=True)])
        self.state.settings.target_system = 2
        callback = self.ftp.cmd_get.call_args.kwargs['callback']
        callback(io.BytesIO(struct.pack('<HHHHH', 0x763d, 0, 0, 0, 0)))
        self.assertNotIn(2, self.wp.wploader_by_sysid)
        self.assert_mission_status(False, 'Read result discarded: vehicle changed')


if __name__ == '__main__':
    unittest.main()
