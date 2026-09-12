"""Camera XML rules and extended parameter transactions (no camera or wx needed)."""
import io
import lzma
import os
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

os.environ.setdefault('MAVLINK20', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules.mavproxy_camera.definition import CameraDefinition, decode_value, TYPES
from MAVProxy.modules.mavproxy_camera.parameters import CameraParameters
from pymavlink import mavutil

XML = b'''<mavlinkcamera>
<definition version="1"><vendor>Test</vendor><model>Camera</model></definition>
<parameters>
<parameter name="CAM_MODE" type="uint32" default="0" control="0">
<description>Mode</description><options><option name="Photo" value="0"/>
<option name="Video" value="1"><parameterranges>
<parameterrange parameter="ISO" condition="AUTO=0 AND GAIN&gt;2">
<roption name="Low" value="100"/></parameterrange></parameterranges></option></options></parameter>
<parameter name="AUTO" type="bool" default="1"><description>Automatic</description>
<updates><update>ISO</update><update>AUTO</update></updates><options>
<option name="On" value="1"><exclusions><exclusions><exclude>ISO</exclude></exclusions></exclusions></option>
<option name="Off" value="0"/></options></parameter>
<parameter name="ISO" type="uint32" default="100"><description>ISO</description>
<options><option name="100" value="100"/><option name="400" value="400"/></options></parameter>
<parameter name="GAIN" type="float" default="1" min="0" max="10" step="0.1"><description>Gain</description></parameter>
<parameter name="SERIAL" type="uint64" default="0" readonly="1"><description>Serial</description></parameter>
<parameter name="ACTION" type="bool" default="0" writeonly="1"><description>Action</description></parameter>
<parameter name="DATA" type="custom"><description>Vendor data</description></parameter>
</parameters><localization><locale name="de_DE"><strings original="Automatic" translated="Automatisch"/>
<strings original="Off" translated="Aus"/></locale></localization></mavlinkcamera>'''


def packet(name, value, param_type=5, result=None):
    fmt = {v[0]: v[1] for v in TYPES.values()}[param_type]
    raw = struct.pack('<' + fmt, value).ljust(128, b'\0')
    if result is None:
        return mavutil.mavlink.MAVLink_param_ext_value_message(name.encode(), raw, param_type, 7, 0)
    return mavutil.mavlink.MAVLink_param_ext_ack_message(name.encode(), raw, param_type, result)


class DefinitionTest(unittest.TestCase):
    def test_order_controls_localization_and_nested_exclusions(self):
        definition = CameraDefinition(XML, 'de_DE')
        self.assertEqual(list(definition.parameters)[:3], ['CAM_MODE', 'AUTO', 'ISO'])
        self.assertEqual(definition.parameters['AUTO'].description, 'Automatisch')
        self.assertEqual(definition.parameters['AUTO'].options[-1][0], 'Aus')
        controls = definition.controls({'AUTO': 1})
        self.assertNotIn('ISO', controls)
        self.assertNotIn('CAM_MODE', controls)
        self.assertNotIn('DATA', controls)
        self.assertIn('ISO', definition.controls({'AUTO': 0}))

    def test_conditional_options_restore_and_numeric_comparisons(self):
        definition = CameraDefinition(XML)
        values = dict(CAM_MODE=1, AUTO=0, GAIN=10)
        self.assertEqual(definition.controls(values)['ISO'], [('Low', 100)])
        values['GAIN'] = 1
        self.assertEqual(len(definition.controls(values)['ISO']), 2)
        self.assertFalse(definition.condition('__import__(os)', values))
        self.assertFalse(definition.condition('MISSING=0', values))
        self.assertTrue(definition.condition('AUTO=1 OR GAIN=1', values))

    def test_compressed_and_invalid_definitions(self):
        self.assertEqual(len(CameraDefinition(lzma.compress(XML)).parameters), 7)
        for data in [b'<html/>', b'<mavlinkcamera>', XML.replace(b'name="ISO" type', b'name="AUTO" type')]:
            with self.assertRaises(ValueError):
                CameraDefinition(data)

    def test_validation_range_step_integer_overflow_and_nan(self):
        definition = CameraDefinition(XML)
        gain = definition.parameters['GAIN']
        self.assertAlmostEqual(gain.validate('2.3', []), 2.3, places=5)
        for value in ['nan', 'inf', '-1', '11', '2.35']:
            with self.assertRaises(ValueError):
                gain.validate(value, [])
        with self.assertRaises(ValueError):
            definition.parameters['ISO'].validate('200', [('Low', 100)])
        with self.assertRaises(ValueError):
            definition.parameters['ISO'].convert(2 ** 32)

    def test_all_wire_types_survive_real_mavlink_roundtrip(self):
        values = [255, -128, 65535, -32768, 0x80000000, -2147483648,
                  2 ** 64 - 1, -2 ** 63, 0.95, -12345.6789]
        for param_type, value in enumerate(values, 1):
            msg = packet('TEST', value, param_type)
            encoded = msg.pack(mavutil.mavlink.MAVLink(None))
            received = mavutil.mavlink.MAVLink(None).parse_char(encoded)
            decoded = decode_value(received)
            if param_type >= 9:
                self.assertAlmostEqual(decoded, value, places=6)
            else:
                self.assertEqual(decoded, value)


class ParameterTest(unittest.TestCase):
    def setUp(self):
        self.module = mock.Mock()
        self.camera = SimpleNamespace(system_id=17, component_id=100, definition=None,
                                      label=lambda: 'Camera')
        self.parameters = CameraParameters(self.module, self.camera)
        self.parameters.definition = CameraDefinition(XML)
        self.parameters.values.update(AUTO=0, ISO=100, CAM_MODE=0, GAIN=1)

    def test_dialog_refresh_bulk_requests_the_camera_component(self):
        p = self.parameters
        p.errors['ISO'] = 'No response; use Refresh to retry'
        p.dialog = mock.Mock()
        p.dialog.events.return_value = [('refresh',)]
        p.idle()
        self.module.master.mav.param_ext_request_list_send.assert_called_once_with(17, 100)
        self.assertFalse(p.errors)
        self.assertIn('ISO', p.reads)
        self.module.master.mav.param_ext_request_read_send.assert_not_called()

    def test_missing_definition_parameter_in_complete_camera_list(self):
        p = self.parameters
        p.request_all()
        # The camera exposes two parameters, including one absent from XML,
        # but does not expose ISO. XML order cannot stand in for wire indexes.
        for index, name in enumerate(['AUTO', 'EXTRA']):
            msg = packet(name, 0, 1)
            msg.param_count = 2
            msg.param_index = index
            p.packet(msg)
        p.reads = {'ISO': [0, p.MAX_ATTEMPTS]}
        p.idle()
        self.assertIn('definition mismatch', p.errors['ISO'])
        # Later periodic updates must not claim all settings are up to date.
        msg = packet('AUTO', 0, 1)
        msg.param_count = 2
        p.packet(msg)
        self.assertIn('unavailable', p.status)

    def test_incomplete_list_is_a_timeout_not_a_definition_mismatch(self):
        p = self.parameters
        p.request_all()
        msg = packet('AUTO', 0, 1)
        msg.param_count = 2
        p.packet(msg)
        p.reads = {'ISO': [0, p.MAX_ATTEMPTS]}
        p.idle()
        self.assertIn('No response', p.errors['ISO'])
        p.request_all()
        self.assertIsNone(p.reported_count)
        self.assertFalse(p.reported_parameters)

    def test_changed_parameter_count_invalidates_previous_inventory(self):
        p = self.parameters
        msg = packet('AUTO', 0, 1)
        msg.param_count = 1
        p.packet(msg)
        self.assertIn('mismatch', p._read_timeout_error('ISO'))
        msg.param_count = 2
        p.packet(msg)
        self.assertIn('No response', p._read_timeout_error('ISO'))

    def test_write_binary_target_pending_and_ack_dependent_updates(self):
        p = self.parameters
        p.set_value('AUTO', 1)
        sent = self.module.master.mav.param_ext_set_send.call_args.args
        self.assertEqual(sent[:3], (17, 100, b'AUTO'))
        self.assertEqual(sent[3], b'\x01' + b'\0' * 127)
        self.assertEqual(sent[4], 1)
        self.assertEqual(p.values['AUTO'], 0)
        with self.assertRaises(ValueError):
            p.set_value('AUTO', 0)
        p.packet(packet('AUTO', 1, 1, mavutil.mavlink.PARAM_ACK_ACCEPTED))
        self.assertEqual(p.values['AUTO'], 1)
        self.assertIn('ISO', p.reads)
        self.assertIn('AUTO', p.reads)  # self-update actions must reread
        self.assertFalse(p.pending)
        self.assertNotIn('ISO', p.definition.controls(p.values))

    def test_failed_ack_restores_camera_value_and_reports_error(self):
        p = self.parameters
        p.set_value('ISO', 400)
        p.packet(packet('ISO', 100, 5, mavutil.mavlink.PARAM_ACK_VALUE_UNSUPPORTED))
        self.assertEqual(p.values['ISO'], 100)
        self.assertIn('UNSUPPORTED', p.errors['ISO'])
        self.assertIn('ISO', p.reads)

    def test_in_progress_does_not_resend_and_has_deadline(self):
        p = self.parameters
        p.set_value('ISO', 400)
        p.packet(packet('ISO', 400, 5, mavutil.mavlink.PARAM_ACK_IN_PROGRESS))
        p.idle()
        self.assertEqual(self.module.master.mav.param_ext_set_send.call_count, 1)
        p.pending['ISO']['deadline'] = 0
        p.idle()
        self.assertNotIn('ISO', p.pending)
        self.assertIn('timed out', p.errors['ISO'])
        self.assertEqual(self.module.master.mav.param_ext_set_send.call_count, 1)

    def test_retries_bounded_and_missing_reads_recovered(self):
        p = self.parameters
        p.request_all()
        self.assertNotIn('ACTION', p.reads)
        p.packet(packet('ISO', 400))
        self.assertNotIn('ISO', p.reads)
        p.set_value('ISO', 100)
        for _ in range(3):
            p.pending['ISO']['deadline'] = 0
            p.idle()
        self.assertEqual(self.module.master.mav.param_ext_set_send.call_count, 3)
        self.assertFalse(p.pending)
        self.assertIn('timed out', p.errors['ISO'])

    def test_unread_readonly_hidden_and_invalid_type(self):
        p = self.parameters
        for name in ['SERIAL', 'CAM_MODE']:
            with self.assertRaises(ValueError):
                p.set_value(name, 1)
        p.values.pop('ISO')
        with self.assertRaises(ValueError):
            p.set_value('ISO', 100)
        p.set_value('ACTION', 1)
        p.packet(packet('GAIN', 1, 5))
        self.assertEqual(p.values['GAIN'], 1)
        self.assertIn('different parameter type', p.errors['GAIN'])

    def test_ftp_uses_camera_target_and_stale_download_is_ignored(self):
        p = self.parameters
        p.load('mavftp:///camera.xml.xz')
        ftp = self.module.module.return_value
        args, kwargs = ftp.cmd_get.call_args
        self.assertEqual(args, (['/camera.xml.xz'],))
        self.assertEqual((kwargs['target_system'], kwargs['target_component']), (17, 100))
        callback = kwargs['callback']
        p.load('')
        callback(io.BytesIO(lzma.compress(XML)))
        p.idle()
        self.assertIsNone(p.definition)

    def test_qgc_mftp_component_uri(self):
        self.parameters.load('mftp://[;comp=101]/camera.xml.xz')
        args, kwargs = self.module.module.return_value.cmd_get.call_args
        self.assertEqual(args, (['/camera.xml.xz'],))
        self.assertEqual(kwargs['target_system'], 17)
        self.assertEqual(kwargs['target_component'], 101)

    def test_definition_loaded_requests_parameters_and_repeated_info_is_quiet(self):
        p = self.parameters
        info = SimpleNamespace(cam_definition_uri='mavftp:///camera.xml', cam_definition_version=1)
        p.information(info)
        self.module.module.return_value.cmd_get.call_args.kwargs['callback'](io.BytesIO(XML))
        import time
        deadline = time.monotonic() + 2
        while p.loading and time.monotonic() < deadline:
            p.idle()
            time.sleep(0.001)
        self.module.master.mav.param_ext_request_list_send.assert_called_once_with(17, 100)
        p.information(info)
        self.assertEqual(self.module.module.return_value.cmd_get.call_count, 1)


if __name__ == '__main__':
    unittest.main()
