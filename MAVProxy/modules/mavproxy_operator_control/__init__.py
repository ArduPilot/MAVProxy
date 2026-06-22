#!/usr/bin/env python3
'''
MAVProxy operator control module.

Implements the GCS operator control protocol using
MAV_CMD_REQUEST_OPERATOR_CONTROL and CONTROL_STATUS.

AP_FLAKE8_CLEAN
'''

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

MAV_CMD_REQUEST_OPERATOR_CONTROL = 32100
GCS_CONTROL_STATUS_FLAGS_SYSTEM_MANAGER = 1
GCS_CONTROL_STATUS_FLAGS_TAKEOVER_ALLOWED = 2


class OperatorControlModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OperatorControlModule, self).__init__(
            mpstate, "operator_control", "GCS operator control", public=True)
        self.manager_system = None
        self.manager_component = None
        self.control_status = None
        self._last_gcs_main = None
        self._last_flags = None

        self.add_command('oc', self.cmd_oc, 'operator control commands', [
            'request [allow_takeover=0|1]',
            'release',
            'grant',
            'deny',
            'status',
        ])

    def _flags_str(self, flags):
        parts = []
        if flags & GCS_CONTROL_STATUS_FLAGS_SYSTEM_MANAGER:
            parts.append('SYSTEM_MANAGER')
        if flags & GCS_CONTROL_STATUS_FLAGS_TAKEOVER_ALLOWED:
            parts.append('TAKEOVER_ALLOWED')
        return '|'.join(parts) if parts else 'none'

    def _result_str(self, result):
        result_map = {
            mavutil.mavlink.MAV_RESULT_ACCEPTED: 'ACCEPTED',
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
            mavutil.mavlink.MAV_RESULT_DENIED: 'DENIED',
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED: 'UNSUPPORTED',
            mavutil.mavlink.MAV_RESULT_FAILED: 'FAILED',
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS: 'IN_PROGRESS',
        }
        return result_map.get(result, 'UNKNOWN(%d)' % result)

    def _parse_named_args(self, args, valid_keys):
        parsed = {}
        for arg in args:
            if '=' not in arg:
                print("oc: expected key=value, got: %s" % arg)
                return None
            key, val = arg.split('=', 1)
            if key not in valid_keys:
                print("oc: unknown parameter '%s'" % key)
                return None
            try:
                parsed[key] = int(val)
            except ValueError:
                print("oc: '%s' must be an integer" % key)
                return None
        return parsed

    def cmd_oc(self, args):
        '''operator control command'''
        usage = "Usage: oc <request|release|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == 'request':
            self._cmd_request(args[1:])
        elif args[0] == 'release':
            self._cmd_release()
        elif args[0] == 'grant':
            self._cmd_grant()
        elif args[0] == 'deny':
            self._cmd_request([])
        elif args[0] == 'status':
            self._cmd_status()
        else:
            print(usage)

    def _cmd_request(self, args):
        if self.manager_system is None:
            print("oc: no system manager found yet (waiting for CONTROL_STATUS)")
            return
        parsed = self._parse_named_args(args, {'allow_takeover'})
        if parsed is None:
            return
        allow_takeover = parsed.get('allow_takeover', 0)
        self.master.mav.command_long_send(
            self.manager_system,
            self.manager_component,
            MAV_CMD_REQUEST_OPERATOR_CONTROL,
            0,
            1.0,
            float(allow_takeover),
            10.0,
            float(self.settings.source_system),
            0.0, 0.0, 0.0)
        print("oc: requesting control (manager sysid=%d compid=%d allow_takeover=%d)" %
              (self.manager_system, self.manager_component, allow_takeover))

    def _cmd_grant(self):
        '''re-request control with allow_takeover=1, giving the pending requester a 10s window'''
        self._cmd_request(['allow_takeover=1'])

    def _cmd_release(self):
        if self.manager_system is None:
            print("oc: no system manager found yet (waiting for CONTROL_STATUS)")
            return
        self.master.mav.command_long_send(
            self.manager_system,
            self.manager_component,
            MAV_CMD_REQUEST_OPERATOR_CONTROL,
            0,
            0.0, 0.0, 0.0,
            float(self.settings.source_system),
            0.0, 0.0, 0.0)
        print("oc: releasing control")

    def _cmd_status(self):
        if self.control_status is None:
            print("oc: no CONTROL_STATUS received yet")
            return
        m = self.control_status
        secondary = [x for x in m.gcs_secondary if x != 0]
        print("oc status: manager=sysid:%d/compid:%d  flags=%s  gcs_main=%d  gcs_secondary=%s" % (
            self.manager_system, self.manager_component,
            self._flags_str(m.flags),
            m.gcs_main,
            str(secondary) if secondary else 'none'))

    def mavlink_packet(self, m):
        mtype = m.get_type()

        if mtype == 'CONTROL_STATUS':
            if not (m.flags & GCS_CONTROL_STATUS_FLAGS_SYSTEM_MANAGER):
                return
            self.manager_system = m.get_srcSystem()
            self.manager_component = m.get_srcComponent()
            self.control_status = m
            if m.gcs_main != self._last_gcs_main or m.flags != self._last_flags:
                self._last_gcs_main = m.gcs_main
                self._last_flags = m.flags
                secondary = [x for x in m.gcs_secondary if x != 0]
                takeover = bool(m.flags & GCS_CONTROL_STATUS_FLAGS_TAKEOVER_ALLOWED)
                print("oc: CONTROL_STATUS gcs_main=%d secondary=%s takeover_allowed=%s" % (
                    m.gcs_main,
                    str(secondary) if secondary else 'none',
                    takeover))

        elif mtype == 'COMMAND_ACK':
            if m.command == MAV_CMD_REQUEST_OPERATOR_CONTROL:
                print("oc: REQUEST_OPERATOR_CONTROL ack: %s" % self._result_str(m.result))

        elif mtype == 'COMMAND_LONG':
            if (m.command == MAV_CMD_REQUEST_OPERATOR_CONTROL and
                    m.target_system == self.settings.source_system):
                requester = int(m.param4)
                print("oc: takeover request from GCS sysid=%d  (use 'oc grant', 'oc deny', or 'oc release')" % requester)
                self.master.mav.command_ack_send(
                    MAV_CMD_REQUEST_OPERATOR_CONTROL,
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    255, 0,
                    m.get_srcSystem(),
                    m.get_srcComponent())


def init(mpstate):
    '''initialise module'''
    return OperatorControlModule(mpstate)
