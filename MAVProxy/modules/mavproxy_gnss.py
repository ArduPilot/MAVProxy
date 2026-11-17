#!/usr/bin/env python3
'''
GNSS receiver status

Tabulates the GNSS message the vehicle sends for each of its GNSS
receivers, alongside the receiver's GPSn_TYPE, where it is connected
and the driver ArduPilot detected for it (if MAVProxy saw the
"GPS n: detected" message), as of the latest message for each
receiver.

AP_FLAKE8_CLEAN
'''

import math
import re
import time

from MAVProxy.modules.lib import mp_module

# GPSn_TYPE values, from the ArduPilot parameter documentation
gps_type_names = {
    0: 'None',
    1: 'AUTO',
    2: 'uBlox',
    5: 'NMEA',
    6: 'SiRF',
    7: 'HIL',
    8: 'SwiftNav',
    9: 'DroneCAN',
    10: 'SBF',
    11: 'GSOF',
    13: 'ERB',
    14: 'MAVLink',
    15: 'NOVA',
    16: 'HemisphereNMEA',
    17: 'uBlox-MB-Base',
    18: 'uBlox-MB-Rover',
    19: 'MSP',
    20: 'AllyStar',
    21: 'ExternalAHRS',
    22: 'DroneCAN-MB-Base',
    23: 'DroneCAN-MB-Rover',
    24: 'UnicoreNMEA',
    25: 'UnicoreMB-NMEA',
    26: 'SBF-DualAntenna',
    27: 'Blended',
    100: 'SITL',
}

# GPSn_TYPE values for receivers connected over DroneCAN
dronecan_types = {9, 22, 23}

# GPSn_TYPE values which do not use a serial port; ArduPilot assigns
# the serial ports with GPS protocol to the other instances in order
non_serial_types = {0, 7, 9, 14, 19, 21, 22, 23, 27}

fix_type_names = {
    0: 'NoGPS',
    1: 'NoFix',
    2: '2D',
    3: '3D',
    4: 'DGPS',
    5: 'RTKFloat',
    6: 'RTKFixed',
    7: 'Static',
    8: 'PPP',
}


class GNSSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GNSSModule, self).__init__(mpstate, "gnss", "GNSS receiver status", public=True)
        self.gnss = {}  # (sysid, id) -> (time received, GNSS message)
        self.detected = {}  # (sysid, id) -> driver name from "GPS n: detected NAME"
        self.add_command('gnss', self.cmd_gnss, "GNSS receiver status", ['status'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: gnss <status>"

    def cmd_gnss(self, args):
        '''handle the gnss command'''
        if len(args) == 0 or args[0] != "status":
            print(self.usage())
            return
        # start on a new line, as the prompt may be on the current one:
        print('\n' + self.status())

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype == 'GNSS':
            self.gnss[(m.get_srcSystem(), m.id)] = (time.time(), m)
        elif mtype == 'STATUSTEXT':
            # ArduPilot announces the driver it found for each receiver:
            match = re.match(r'GPS (\d+): detected (.*)', m.text)
            if match is not None:
                self.detected[(m.get_srcSystem(), int(match.group(1))-1)] = match.group(2)

    def param(self, name):
        '''return an integer parameter of the primary vehicle, or None'''
        value = self.get_mav_param(name, None)
        if value is None:
            return None
        return int(value)

    def gps_serial_ports(self):
        '''return the numbers of the serial ports with GPS protocol, in the
        order ArduPilot assigns them to GPS instances'''
        ports = []
        for n in range(10):
            if self.param('SERIAL%u_PROTOCOL' % n) == 5:
                ports.append(n)
        return ports

    def connections(self, instances):
        '''return a dictionary from GPS instance (0-based) to a description
        of where the receiver is connected'''
        ret = {}
        serial_ports = self.gps_serial_ports()
        uart_idx = 0
        for i in instances:
            gps_type = self.param('GPS%u_TYPE' % (i+1))
            if gps_type is None:
                ret[i] = ''
            elif gps_type in dronecan_types:
                node = self.param('GPS%u_CAN_NODEID' % (i+1))
                ret[i] = 'DroneCAN node %s' % ('?' if not node else node)
            elif gps_type == 27:
                ret[i] = 'blended'
            elif gps_type in non_serial_types:
                ret[i] = ''
            else:
                if uart_idx < len(serial_ports):
                    ret[i] = 'SERIAL%u' % serial_ports[uart_idx]
                else:
                    ret[i] = 'no serial port'
                uart_idx += 1
        return ret

    @staticmethod
    def metres(value):
        '''format a float field which is NaN when unknown'''
        if value is None or math.isnan(value):
            return '?'
        return '%.2f' % value

    @staticmethod
    def yaw_string(yaw):
        '''format the GNSS yaw field'''
        if yaw == 65535:
            return '-'
        if yaw == 65534:
            return 'unavail'
        return '%.1f' % (yaw * 0.01)

    def status(self):
        '''return a table of the primary vehicle's GNSS receivers'''
        sysid = self.target_system
        now = time.time()
        seen = {i: v for ((s, i), v) in self.gnss.items() if sysid == 0 or s == sysid}

        # every configured instance, plus any we have GNSS for:
        instances = set(seen.keys())
        for i in range(16):
            gps_type = self.param('GPS%u_TYPE' % (i+1))
            if gps_type is not None and gps_type != 0:
                instances.add(i)
        instances = sorted(instances)
        if len(instances) == 0:
            return 'No GNSS messages or configured GPS parameters'

        connected = self.connections(range(max(instances)+1))
        header = ['id', 'param', 'type', 'connection', 'detected', 'fix', 'sats', 'h_acc', 'v_acc',
                  'h_geoid', 'h_ellips', 'undul', 'yaw', 'age']
        rows = []
        for i in instances:
            gps_type = self.param('GPS%u_TYPE' % (i+1))
            if gps_type is None:
                param = '-'
                type_name = '?'
            else:
                param = 'GPS%u %u' % (i+1, gps_type)
                type_name = gps_type_names.get(gps_type, '?')
            detected = ''
            for ((s, d), name) in self.detected.items():
                if d == i and (sysid == 0 or s == sysid):
                    detected = name
            row = [str(i), param, type_name, connected.get(i, ''), detected]
            if i not in seen:
                row.extend(['no GNSS'] + [''] * 7 + ['-'])
                rows.append(row)
                continue
            (t, m) = seen[i]
            undulation = None
            if not math.isnan(m.height_above_geoid) and not math.isnan(m.height_above_ellipsoid):
                undulation = m.height_above_ellipsoid - m.height_above_geoid
            row.extend([
                fix_type_names.get(m.fix_type, str(m.fix_type)),
                '?' if m.satellites_used == 255 else str(m.satellites_used),
                self.metres(m.h_acc),
                self.metres(m.v_acc),
                self.metres(m.height_above_geoid),
                self.metres(m.height_above_ellipsoid),
                self.metres(undulation),
                self.yaw_string(m.yaw),
                '%.1fs' % (now - t),
            ])
            rows.append(row)

        widths = [max(len(r[c]) for r in [header] + rows) for c in range(len(header))]
        lines = []
        for r in [header] + rows:
            lines.append('  '.join(r[c].ljust(widths[c]) for c in range(len(header))).rstrip())
        return '\n'.join(lines)


def init(mpstate):
    '''initialise module'''
    return GNSSModule(mpstate)
