"""Parameter encoding selection and exact set/ack handling.

AP_FLAKE8_CLEAN
"""
import io
import struct
import unittest
from unittest.mock import Mock

from pymavlink import mavutil, mavparm
from pymavlink.dialects.v20 import ardupilotmega as dialect
from MAVProxy.modules.mavproxy_param import ParamState


class BytewiseParamsTest(unittest.TestCase):
    def setUp(self):
        self.master = Mock()
        self.master.mavlink20.return_value = True
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
            self.assertFalse(pending.handle_PARAM_VALUE(None, value-1))
            self.assertTrue(pending.handle_PARAM_VALUE(None, value))

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


if __name__ == '__main__':
    unittest.main()
