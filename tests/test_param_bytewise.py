"""Parameter encoding selection and exact set/ack handling.

AP_FLAKE8_CLEAN
"""
import io
import struct
import unittest
from unittest.mock import Mock, patch

from pymavlink import mavutil, mavparm
from pymavlink.dialects.v20 import ardupilotmega as dialect
from MAVProxy.modules.mavproxy_param import ParamState


class BytewiseParamsTest(unittest.TestCase):
    def setUp(self):
        self.master = Mock()
        self.master.mavlink20.return_value = True
        self.master.target_system = 1
        self.master.target_component = 1
        self.master.mav.srcSystem = 255
        self.master.mav.srcComponent = 190
        self.state = ParamState(mavparm.MAVParmDict(), None, 'ArduRover', 'mav.parm', Mock(), (1, 1))
        self.state.autopilot_type_by_sysid[1] = dialect.MAV_AUTOPILOT_ARDUPILOTMEGA

    def test_negotiated_selection(self):
        self.assertIsNone(self.state.wire_param_type(self.master, 'TEST', 42, dialect.MAV_PARAM_TYPE_INT32))
        self.state.supports_bytewise_by_sysid[1] = True
        for logical, wire in ((6, 12), (5, 13)):
            self.assertEqual(self.state.wire_param_type(self.master, 'TEST', 42, logical), wire)
        self.master.mavlink20.return_value = False
        self.assertIsNone(self.state.wire_param_type(self.master, 'TEST', 42, 6))
        self.master.mavlink20.return_value = True
        self.state.autopilot_type_by_sysid[1] = dialect.MAV_AUTOPILOT_PX4
        self.assertEqual(self.state.wire_param_type(self.master, 'TEST', 42, 6), 6)

    def test_exact_set_and_ack(self):
        for ptype, value, fmt in ((12, 0x7f800001, '<i'), (13, 2**32-1, '<I'),
                                  (8, -2**63, '<q'), (7, 2**64-1, '<Q')):
            pending = ParamState.ParamSet(self.master, 'TEST', str(value), param_type=ptype)
            pending.send_set()
            self.master.param_fetch_one.assert_called_with('TEST')
            args, kwargs = self.master.param_set_send.call_args
            self.assertEqual(pending.target_value(), value)
            data = kwargs.get('parm_raw', kwargs.get('extended_data'))
            self.assertEqual(data, struct.pack(fmt, value))
            wrong = self.ext_message(0, pending.extended_type) if pending.extended_type else None
            right = self.ext_message(value, pending.extended_type) if pending.extended_type else None
            self.assertFalse(pending.handle_PARAM_VALUE(wrong, value-1))
            self.assertTrue(pending.handle_PARAM_VALUE(right, value))

    def test_decode_records_logical_type(self):
        msg = dialect.MAVLink_param_value_message(b'TEST', 0, 12, 1, 0)
        msg.set_raw_field_bytes(0, struct.pack('<i', 0x7f800001))
        tx = dialect.MAVLink(io.BytesIO(), srcSystem=1, srcComponent=1)
        rx = dialect.MAVLink(io.BytesIO())
        decoded = rx.parse_char(msg.pack(tx))
        self.assertEqual(self.state.handle_bytewise_param_value(decoded), 0x7f800001)
        self.assertEqual(self.state.mav_param.param_types['TEST'], mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.assertTrue(self.state.mav_param.target_supports_bytewise)

    def test_fractional_integer_rejected(self):
        self.master.reset_mock()
        pending = ParamState.ParamSet(self.master, 'TEST', '1.5', param_type=12)
        pending.send_set()
        self.master.param_set_send.assert_not_called()
        self.assertEqual(pending.attempts_remaining, 0)

    def ext_message(self, value, subtype):
        data = mavutil.encode_param_extended(value, subtype).ljust(128, b'\x00')
        msg = dialect.MAVLink_param_value_message(b'TEST', float('nan'), 11, 1, 0, subtype, data)
        return self.roundtrip(msg)

    def roundtrip(self, msg, component=1):
        tx = dialect.MAVLink(io.BytesIO(), srcSystem=1, srcComponent=component)
        return dialect.MAVLink(io.BytesIO()).parse_char(msg.pack(tx))

    def test_real64_and_custom(self):
        for logical, subtype, value in ((10, 3, 1.0000000000000002), (10, 3, -0.0),
                                        (10, 3, float('nan')), (11, 4, bytes(range(128)))):
            pending = ParamState.ParamSet(self.master, 'TEST', value, param_type=logical, extended_type=subtype)
            pending.send_set()
            ack = self.ext_message(value, subtype)
            self.assertTrue(pending.handle_PARAM_VALUE(ack, mavutil.decode_param_value(ack)))
            self.state.handle_bytewise_param_value(ack)
            self.assertEqual(self.state.mav_param.param_types['TEST'], logical)
            self.assertEqual(self.state.mav_param.param_extended_types['TEST'], subtype)
            ack.extended_data = bytes(128)
            self.assertFalse(pending.handle_PARAM_VALUE(ack, 0))

    def test_progress_preserves_download_and_extends_timeout(self):
        pending = ParamState.ParamSet(self.master, 'TEST', 42, param_type=12, attempts=1)
        with patch('time.time', return_value=10):
            pending.send_set()
        self.state.parameters_to_set['TEST'] = pending
        self.state.mav_param['TEST'] = 41
        self.state.mav_param.param_types['TEST'] = 6
        self.state.mav_param_count = 20
        self.state.fetch_set = {5}
        progress = dialect.MAVLink_param_value_message(b'TEST', float('nan'), 14, 999, 5)
        with patch('time.time', return_value=10.8):
            self.state.handle_mavlink_packet(self.master, self.roundtrip(progress, component=2))
            self.assertEqual(pending.request_sent, 10)
            self.state.handle_mavlink_packet(self.master, self.roundtrip(progress))
        with patch('time.time', return_value=11.5):
            self.assertFalse(pending.expired())
        with patch('time.time', return_value=12):
            self.assertTrue(pending.expired())
        self.assertEqual(self.state.mav_param['TEST'], 41)
        self.assertEqual(self.state.mav_param.param_types['TEST'], 6)
        self.assertEqual(self.state.mav_param_count, 20)
        self.assertEqual(self.state.fetch_set, {5})
        self.assertFalse(self.state.mav_param_set)
        error = dialect.MAVLink_param_error_message(255, 190, b'TEST', -1, 9)
        error.target_system = 254
        self.state.handle_mavlink_packet(self.master, self.roundtrip(error))
        self.assertIn('TEST', self.state.parameters_to_set)
        error.target_system = 255
        self.state.handle_mavlink_packet(self.master, self.roundtrip(error))
        self.assertFalse(self.state.parameters_to_set)
        self.assertEqual(self.state.mav_param['TEST'], 41)

    def test_custom_hex_input(self):
        pending = ParamState.ParamSet(self.master, 'TEST', 'hex:0001ff', param_type=11, extended_type=4)
        pending.send_set()
        self.assertEqual(self.master.param_set_send.call_args.kwargs['extended_data'],
                         bytes([0, 1, 255]) + bytes(125))
        self.master.reset_mock()
        pending = ParamState.ParamSet(self.master, 'TEST', 'hex:zz', param_type=11, extended_type=4)
        pending.send_set()
        self.master.param_set_send.assert_not_called()

    def test_exact_display(self):
        self.state.param_types['TEST'] = 10
        self.assertEqual(self.state.format_param_value('TEST', 1.0000000000000002), '1.0000000000000002')
        self.assertEqual(self.state.format_param_value('TEST', b'\x00\xff'), 'hex:00ff')


if __name__ == '__main__':
    unittest.main()
