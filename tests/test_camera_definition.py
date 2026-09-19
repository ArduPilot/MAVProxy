"""Camera XML rules and extended parameter transactions (no camera or wx needed)."""
import io
from decimal import Decimal
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
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
from MAVProxy.modules.mavproxy_camera.definition import (CameraDefinition, Parameter, decode_value, TYPES, equal_value,
    definition_bytes, download_definition, MAX_DEFINITION_SIZE, _DefinitionRedirectHandler)
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


class DefinitionValidationTest(unittest.TestCase):
    def parameter(self, attributes, content=''):
        return Parameter(ET.fromstring('<parameter name="X" %s>%s</parameter>' %
                                       (attributes, content)), lambda s: s)

    def test_integer_step_uses_exact_arithmetic_at_large_values(self):
        p = self.parameter('type="uint64" min="0" step="10"')
        for value in [19, 10009, 100009, 999999, 2 ** 64 - 1]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                p.validate(value, [])
        self.assertEqual(p.validate(2 ** 64 - 6, []), 2 ** 64 - 6)
        self.assertEqual(p.validate(1000000, []), 1000000)

    def test_all_thermal_float_slider_positions_are_accepted(self):
        for lower, upper, step in [('-100', '100', '0.01'),
                                   ('-273.15', '1000', '0.1'),
                                   ('-0.3', '0.3', '0.1')]:
            p = self.parameter('type="float" min="%s" max="%s" step="%s"' %
                               (lower, upper, step))
            count = int((Decimal(upper) - Decimal(lower)) / Decimal(step))
            for i in range(count + 1):
                # Text-entry values and wx slider arithmetic must both work.
                text = str(Decimal(lower) + i * Decimal(step))
                self.assertEqual(p.validate(text, []), p.convert(text))
                p.validate(float(lower) + i * float(step), [])
            self.assertEqual(p.minimum, float(lower))
            self.assertEqual(p.step, float(step))
            with self.assertRaises(ValueError):
                p.validate(str(Decimal(lower) + Decimal(step) / 2), [])

    def test_double_steps_and_wire_rounded_bounds(self):
        p = self.parameter('type="double" min="-0.3" max="0.3" step="0.1"')
        p.validate(0, [])
        p.validate(-0.3 + 3 * 0.1, [])
        with self.assertRaises(ValueError):
            p.validate('0.05', [])
        p = self.parameter('type="float" min="0.7" max="0.9" step="0.1"')
        for value in ('0.7', '0.8', '0.9'):
            p.validate(value, [])
        for attrs in ['min="2" max="1"', 'step="0"', 'step="-1"']:
            with self.assertRaises(ValueError):
                self.parameter('type="float" ' + attrs)

    def test_adjacent_float32_options_remain_distinct(self):
        adjacent = struct.unpack('<f', struct.pack('<I', 0x3f800001))[0]
        self.assertFalse(equal_value(1.0, adjacent))
        p = self.parameter('type="float"',
                           '<options><option name="One" value="1"/></options>')
        with self.assertRaises(ValueError):
            p.validate(adjacent, p.options)
        xml = XML.replace(b'name="AUTO" type="bool"', b'name="AUTO" type="float"')
        definition = CameraDefinition(xml)
        controls = definition.controls({'AUTO': adjacent})
        self.assertIn('ISO', controls)  # Only AUTO=1 excludes ISO.
        self.assertFalse(definition.condition('AUTO=1', {'AUTO': adjacent}))

    def test_custom_numeric_attributes_and_options_do_not_break_other_controls(self):
        xml = XML.replace(b'<parameter name="DATA" type="custom">',
                          b'<parameter name="DATA" type="custom" min="vendor" '
                          b'max="data" step="whatever" default="opaque">'
                          b'<options><option name="Binary" value="opaque"/></options>')
        definition = CameraDefinition(xml)
        custom = definition.parameters['DATA']
        self.assertIsNone(custom.minimum)
        self.assertFalse(custom.options)
        self.assertIn('GAIN', definition.controls({}))
        self.assertNotIn('DATA', definition.controls({}))

    def test_dtd_and_entity_expansion_are_rejected_in_multiple_encodings(self):
        for dtd in ['<!DOCTYPE mavlinkcamera [<!ENTITY x "' + 'A' * 1000 + '">]>',
                    '<!DOCTYPE mavlinkcamera SYSTEM "file:///etc/passwd">',
                    '<!DOCTYPE mavlinkcamera [<!ENTITY x SYSTEM "file:///etc/passwd">]>']:
            xml = dtd + '<mavlinkcamera><definition><model>&x;</model></definition></mavlinkcamera>'
            for encoding in ('utf-8', 'utf-16'):
                with self.subTest(encoding=encoding), self.assertRaises(ValueError):
                    CameraDefinition(xml.encode(encoding))

    def test_size_limits_xz_expansion_and_truncated_streams(self):
        with mock.patch('MAVProxy.modules.mavproxy_camera.definition.MAX_DEFINITION_SIZE', 1024):
            for data in [b'x' * 1025, lzma.compress(b'x' * 1025), lzma.compress(b'<x/>')[:-3]]:
                with self.assertRaises(ValueError):
                    definition_bytes(data)
            self.assertEqual(definition_bytes(lzma.compress(b'x' * 1024)), b'x' * 1024)

    def test_custom_nonfinite_and_old_pymavlink_decoding(self):
        self.assertEqual(decode_value(SimpleNamespace(param_type=11, _param_value_raw=b'a\0b')),
                         b'a\0b')
        for value in [float('nan'), float('inf'), -float('inf')]:
            with self.assertRaises(ValueError):
                decode_value(packet('GAIN', value, 9))
        with self.assertRaisesRegex(ValueError, 'upgrade pymavlink'):
            decode_value(SimpleNamespace(param_type=9, param_value='lost binary bytes'))


class HTTPDefinitionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ('/redirect', '/ftp-redirect'):
                    self.send_response(302)
                    self.send_header('Location', '/camera.xz' if self.path == '/redirect'
                                     else 'ftp://127.0.0.1/unwanted.xml')
                    self.end_headers()
                else:
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(lzma.compress(XML) if self.path == '/camera.xz' else XML)

            def log_message(self, *_args):
                pass
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = 'http://127.0.0.1:%u' % cls.server.server_port

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def test_http_xml_compression_and_allowed_redirect(self):
        for path in ('/camera.xml', '/camera.xz', '/redirect'):
            self.assertEqual(len(CameraDefinition(download_definition(self.url + path)).parameters), 7)

    def test_http_size_cap_and_unsupported_schemes(self):
        with mock.patch('MAVProxy.modules.mavproxy_camera.definition.MAX_DEFINITION_SIZE', 64):
            with self.assertRaises(ValueError):
                download_definition(self.url + '/camera.xml')
        for uri in ['file:///etc/passwd', 'ftp://127.0.0.1/camera.xml', self.url + '/ftp-redirect']:
            with self.assertRaises(ValueError):
                download_definition(uri)

    def test_https_redirect_is_allowed(self):
        from urllib.request import Request
        redirect = _DefinitionRedirectHandler().redirect_request(
            Request(self.url), None, 302, 'Found', {}, 'https://camera.example/definition.xml')
        self.assertEqual(redirect.full_url, 'https://camera.example/definition.xml')


class ParameterTest(unittest.TestCase):
    def setUp(self):
        self.module = mock.Mock()
        self.camera = SimpleNamespace(system_id=17, component_id=100, definition=None,
                                      label=lambda: 'Camera')
        self.parameters = CameraParameters(self.module, self.camera)
        self.parameters.definition = CameraDefinition(XML)
        self.parameters.values.update(AUTO=0, ISO=100, CAM_MODE=0, GAIN=1)

    def wait_for_load(self):
        deadline = time.monotonic() + 3
        while self.parameters.loading and time.monotonic() < deadline:
            self.parameters.idle()
            time.sleep(0.001)
        self.assertFalse(self.parameters.loading)

    def test_invalid_ftp_names_are_rejected_before_submission(self):
        for uri in ['mavftp:///caf%C3%A9.xml', 'mftp:///bad%00.xml',
                    'mftp:///' + 'x' * 239, 'mftp:///bad%ff.xml']:
            self.parameters.load(uri)
            self.parameters.idle()
            self.assertIn('Invalid MAVFTP', self.parameters.status)
        self.module.module.return_value.cmd_get.assert_not_called()

    def test_ftp_limit_and_submission_failure_are_reported(self):
        self.module.module.return_value.cmd_get.side_effect = RuntimeError('failed to start')
        self.parameters.load('mftp:///camera.xml')
        self.parameters.idle()
        self.assertEqual(self.module.module.return_value.cmd_get.call_args.kwargs['max_size'],
                         MAX_DEFINITION_SIZE)
        self.assertIn('failed to start', self.parameters.status)

    def test_local_override_and_rejected_scheme(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'camera.xml.xz'
            filename.write_bytes(lzma.compress(XML))
            self.parameters.load(str(filename), local=True)
            self.wait_for_load()
            self.assertEqual(len(self.parameters.definition.parameters), 7)
        self.parameters.load('file:///etc/passwd')
        self.parameters.idle()
        self.assertIn('Unsupported', self.parameters.status)

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

    @mock.patch('MAVProxy.modules.mavproxy_camera.settings_dialog.CameraSettingsDialog')
    @mock.patch('MAVProxy.modules.lib.mp_util.has_wxpython', True)
    def test_dialog_refresh_refetches_xml_and_rebuilds_controls(self, dialog_class):
        p = self.parameters
        p.identity = ('mftp:///camera.xml', 1)
        old_dialog = mock.Mock()
        old_dialog.events.return_value = [('refresh',), ('set', 'AUTO', 1)]
        p.dialog = old_dialog
        p.idle()
        old_dialog.events.return_value = []
        old_dialog.close.assert_not_called()
        self.assertIs(p.dialog, old_dialog)
        self.assertTrue(p.loading)
        self.module.master.mav.param_ext_set_send.assert_not_called()
        self.module.master.mav.param_ext_request_list_send.assert_not_called()
        request = self.module.module.return_value.cmd_get.call_args
        self.assertEqual(request.args, (['/camera.xml'],))
        self.assertEqual(request.kwargs['target_system'], 17)
        self.assertEqual(request.kwargs['target_component'], 100)
        changed = XML.replace(b'</parameters>', b'''<parameter name="NEW" type="bool">
            <description>New control</description></parameter></parameters>''')
        request.kwargs['callback'](io.BytesIO(changed))
        dialog_class.return_value.events.return_value = []
        self.wait_for_load()
        old_dialog.close.assert_called_once()
        self.assertIn('NEW', p.definition.parameters)
        self.assertIn('NEW', p.reads)
        self.module.master.mav.param_ext_request_list_send.assert_called_once_with(17, 100)
        dialog_class.assert_called_once()
        rows = dialog_class.call_args.args[1]['rows']
        self.assertIn('NEW', [row['name'] for row in rows])

    @mock.patch('MAVProxy.modules.mavproxy_camera.settings_dialog.CameraSettingsDialog')
    def test_refresh_unchanged_xml_keeps_window_and_reads_values(self, dialog_class):
        p = self.parameters
        p.identity = ('mftp:///camera.xml', 1)
        old_dialog = mock.Mock()
        p.dialog = old_dialog
        for data in (XML, lzma.compress(XML)):
            with self.subTest(compressed=data != XML):
                old_dialog.events.return_value = [('refresh',)]
                p.idle()
                old_dialog.events.return_value = []
                self.assertIs(p.dialog, old_dialog)
                self.assertEqual(p.values['ISO'], 100)
                self.module.module.return_value.cmd_get.call_args.kwargs['callback'](io.BytesIO(data))
                self.wait_for_load()
                self.assertIs(p.dialog, old_dialog)
                old_dialog.close.assert_not_called()
                dialog_class.assert_not_called()
                self.assertIsNone(p.refresh_fallback)
                self.assertIn('ISO', p.reads)
                self.module.master.mav.param_ext_request_list_send.assert_called_with(17, 100)
        self.assertEqual(self.module.master.mav.param_ext_request_list_send.call_count, 2)

    @mock.patch('MAVProxy.modules.mavproxy_camera.parameters.download_definition')
    def test_refresh_preserves_local_definition_override(self, download):
        p = self.parameters
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'camera.xml'
            filename.write_bytes(XML)
            p.load(str(filename), local=True)
            self.wait_for_load()
            filename.write_bytes(XML.replace(b'<description>Gain</description>',
                                            b'<description>Changed gain</description>'))
            p.dialog = mock.Mock()
            p.dialog.events.return_value = [('refresh',)]
            with mock.patch.object(p, 'open_dialog'):
                p.idle()
                p.dialog.events.return_value = []
                self.wait_for_load()
            self.assertEqual(p.definition.parameters['GAIN'].description, 'Changed gain')
            download.assert_not_called()

    @mock.patch('MAVProxy.modules.mavproxy_camera.settings_dialog.CameraSettingsDialog')
    @mock.patch('MAVProxy.modules.lib.mp_util.has_wxpython', True)
    def test_failed_refresh_keeps_dialog_available_for_retry(self, dialog_class):
        p = self.parameters
        p.identity = ('mftp:///camera.xml', 1)
        previous = p.definition
        old_dialog = mock.Mock()
        p.dialog = old_dialog
        p.dialog.events.return_value = [('refresh',)]
        p.idle()
        p.dialog.events.return_value = []
        self.module.module.return_value.cmd_get.call_args.kwargs['callback'](None)
        dialog_class.return_value.events.return_value = []
        p.idle()
        self.assertIs(p.definition, previous)
        self.assertIs(p.dialog, old_dialog)
        p.packet(packet('AUTO', 1, 1))
        self.assertIn('Definition load failed', p.snapshot()['status'])
        self.assertTrue(all(not row['enabled'] for row in p.snapshot()['rows']))
        with self.assertRaisesRegex(ValueError, 'Refresh'):
            p.set_value('ACTION', 1)
        p.dialog.events.return_value = [('refresh',)]
        p.idle()
        p.dialog.events.return_value = []
        self.module.module.return_value.cmd_get.call_args.kwargs['callback'](io.BytesIO(XML))
        dialog_class.return_value.events.return_value = []
        self.wait_for_load()
        self.assertIsNone(p.refresh_fallback)
        self.assertIs(p.dialog, old_dialog)
        old_dialog.close.assert_not_called()
        dialog_class.assert_not_called()
        self.module.master.mav.param_ext_request_list_send.assert_called_once_with(17, 100)

    @mock.patch('MAVProxy.modules.mavproxy_camera.settings_dialog.CameraSettingsDialog')
    def test_closing_dialog_during_refresh_does_not_reopen_it(self, dialog_class):
        p = self.parameters
        p.identity = ('mftp:///camera.xml', 1)
        p.dialog = mock.Mock()
        p.dialog.events.return_value = [('refresh',)]
        p.idle()
        p.dialog.is_alive.return_value = False
        p.idle()
        changed = XML.replace(b'<description>Gain</description>', b'<description>Changed gain</description>')
        self.module.module.return_value.cmd_get.call_args.kwargs['callback'](io.BytesIO(changed))
        self.wait_for_load()
        self.assertIsNone(p.dialog)
        self.assertFalse(p.open_when_ready)
        dialog_class.assert_not_called()

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
