#!/usr/bin/env python3
'''
GNSS receiver status

Tabulates the GNSS message the vehicle sends for each of its GNSS
receivers, alongside the receiver's GPSn_TYPE, where it is connected
and the driver ArduPilot detected for it (if MAVProxy saw the
"GPS n: detected" message), as of the latest message for each
receiver.

"gnss probe" asks each DroneCAN GPS node for its node information
(uavcan.protocol.GetNodeInfo) through the vehicle's MAVLink CAN
forwarding, and adds the device name and software version to the
table.  CAN forwarding goes to one client at a time, so the probe
takes it over from any other user (such as the DroneCAN GUI tool) for
a few seconds.

AP_FLAKE8_CLEAN
'''

import math
import re
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

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


# DroneCAN service type ID of uavcan.protocol.GetNodeInfo
GETNODEINFO_ID = 1


class NodeInfo(object):
    '''what a DroneCAN node said about itself in a GetNodeInfo response'''
    def __init__(self, bus, response):
        self.bus = bus
        self.name = bytes(response.name).rstrip(b'\0').decode('utf-8', errors='replace')
        sw = response.software_version
        self.sw_version = '%u.%u' % (sw.major, sw.minor)
        self.vcs_commit = sw.vcs_commit if sw.optional_field_flags & 1 else None
        hw = response.hardware_version
        self.hw_version = '%u.%u' % (hw.major, hw.minor)
        self.unique_id = ''.join('%02x' % b for b in hw.unique_id)


class GNSSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GNSSModule, self).__init__(mpstate, "gnss", "GNSS receiver status", public=True)
        self.gnss = {}  # (sysid, id) -> (time received, GNSS message)
        self.detected = {}  # (sysid, id) -> driver name from "GPS n: detected NAME"
        self.node_info = {}  # (sysid, node ID) -> NodeInfo
        self.probe = None  # state of a running "gnss probe"
        self.gnss_settings = mp_settings.MPSettings([
            # the DroneCAN node ID the probe sends its requests from:
            ('probe_node_id', int, 126),
        ])
        self.add_command('gnss', self.cmd_gnss, "GNSS receiver status", ['status', 'probe', 'set (GNSSSETTING)'])
        self.add_completion_function('(GNSSSETTING)', self.gnss_settings.completion)

    def usage(self):
        '''show help on command line options'''
        return "Usage: gnss <status|probe|set>"

    def cmd_gnss(self, args):
        '''handle the gnss command'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            # start on a new line, as the prompt may be on the current one:
            print('\n' + self.status())
        elif args[0] == "probe":
            self.start_probe()
        elif args[0] == "set":
            self.gnss_settings.command(args[1:])
        else:
            print(self.usage())

    def dronecan_nodes(self):
        '''return {node ID: GPS instance} for the primary vehicle's DroneCAN GPSs'''
        nodes = {}
        for i in range(16):
            if self.param('GPS%u_TYPE' % (i+1)) not in dronecan_types:
                continue
            node = self.param('GPS%u_CAN_NODEID' % (i+1)) or self.param('GPS%u_CAN_OVRIDE' % (i+1))
            if node:
                nodes[node] = i
        return nodes

    def dronecan_buses(self):
        '''return the (0-based) CAN buses with a DroneCAN driver'''
        buses = []
        for port in range(1, 4):
            driver = self.param('CAN_P%u_DRIVER' % port)
            if driver is None or driver == 0:
                continue
            if self.param('CAN_D%u_PROTOCOL' % driver) == 1:
                buses.append(port-1)
        return buses

    def start_probe(self):
        '''start asking the DroneCAN GPS nodes for their node information'''
        try:
            import dronecan  # noqa:F401
        except ImportError:
            print("gnss: probe needs the dronecan Python package")
            return
        if self.probe is not None:
            print("gnss: probe already running")
            return
        nodes = self.dronecan_nodes()
        buses = self.dronecan_buses()
        if len(nodes) == 0 or len(buses) == 0:
            print("gnss: no DroneCAN GPS nodes or DroneCAN buses in the parameters (fetched yet?)")
            return
        print("gnss: probing DroneCAN nodes %s on CAN bus %s" % (
            ' '.join(str(n) for n in sorted(nodes)),
            ' '.join(str(b+1) for b in buses)))
        self.probe = {
            'sysid': self.target_system,
            'nodes': set(nodes.keys()),
            'buses': buses,
            'bus_index': -1,
            'attempt': 0,
            'last_send': 0,
            'last_forward': 0,
            'transfer_id': 0,
            'partial': {},  # source node ID -> transfer being reassembled
        }
        self.probe_next_bus()

    def probe_next_bus(self):
        '''start probing the next bus, or finish'''
        p = self.probe
        p['bus_index'] += 1
        p['attempt'] = 0
        p['last_send'] = 0
        p['partial'] = {}
        if p['bus_index'] >= len(p['buses']) or self.probe_pending() == set():
            self.finish_probe()
            return
        bus = p['buses'][p['bus_index']]
        # forward only GetNodeInfo frames, so the responses are not lost
        # among the rest of the traffic on the bus:
        ids = [GETNODEINFO_ID] + [0] * 15
        self.master.mav.can_filter_modify_send(self.target_system, self.target_component,
                                               bus+1, mavutil.mavlink.CAN_FILTER_REPLACE, 1, ids)
        self.send_can_forward(bus+1)

    def probe_pending(self):
        '''return the node IDs the probe has no response from'''
        p = self.probe
        return {n for n in p['nodes'] if (p['sysid'], n) not in self.node_info}

    def send_can_forward(self, bus):
        '''ask the vehicle to forward CAN frames from bus (1-based, 0 to stop) to us'''
        self.master.mav.command_long_send(self.target_system, self.target_component,
                                          mavutil.mavlink.MAV_CMD_CAN_FORWARD, 0,
                                          bus, 0, 0, 0, 0, 0, 0)
        self.probe['last_forward'] = time.time()

    def send_getnodeinfo(self, bus, node):
        '''send a GetNodeInfo request, which is a single frame with an
        empty payload, to node on bus (0-based)'''
        p = self.probe
        priority = 30
        can_id = ((priority << 24) | (GETNODEINFO_ID << 16) | (1 << 15) | (node << 8) | (1 << 7) |
                  self.gnss_settings.probe_node_id)
        tail = 0xC0 | p['transfer_id']  # start and end of transfer
        p['transfer_id'] = (p['transfer_id'] + 1) % 32
        data = [tail] + [0] * 7
        self.master.mav.can_frame_send(self.target_system, self.target_component, bus, 1,
                                       can_id | (1 << 31), data)

    def finish_probe(self):
        '''stop probing, clear the forwarding filter and show the results'''
        p = self.probe
        for bus in p['buses']:
            # an empty list removes the filter for any other user of forwarding:
            self.master.mav.can_filter_modify_send(self.target_system, self.target_component,
                                                   bus+1, mavutil.mavlink.CAN_FILTER_REPLACE, 0, [0] * 16)
        missing = self.probe_pending()
        self.probe = None
        lines = ['gnss: probe finished']
        for node in sorted(p['nodes']):
            info = self.node_info.get((p['sysid'], node))
            if info is None:
                lines.append('  node %u: no response' % node)
                continue
            commit = '' if info.vcs_commit is None else ' (%08x)' % info.vcs_commit
            lines.append('  node %u (bus %u): %s software %s%s hardware %s unique ID %s' % (
                node, info.bus+1, info.name, info.sw_version, commit, info.hw_version, info.unique_id))
        if missing:
            lines.append('  no response from node %s' % ' '.join(str(n) for n in sorted(missing)))
        print('\n'.join(lines))

    def idle_task(self):
        '''run the probe'''
        p = self.probe
        if p is None:
            return
        now = time.time()
        bus = p['buses'][p['bus_index']]
        if now - p['last_forward'] > 1:
            self.send_can_forward(bus+1)
        if now - p['last_send'] < 1:
            return
        pending = self.probe_pending()
        if len(pending) == 0 or p['attempt'] >= 3:
            self.probe_next_bus()
            return
        for node in sorted(pending):
            self.send_getnodeinfo(bus, node)
        p['attempt'] += 1
        p['last_send'] = now

    def handle_can_frame(self, m):
        '''handle a forwarded CAN frame, looking for GetNodeInfo responses'''
        p = self.probe
        if p is None or (p['sysid'] != 0 and m.get_srcSystem() != p['sysid']):
            return
        can_id = m.id & 0x1FFFFFFF
        if (not (can_id & (1 << 7)) or          # not a service frame
                can_id & (1 << 15) or           # a request
                (can_id >> 16) & 0xFF != GETNODEINFO_ID or
                (can_id >> 8) & 0x7F != self.gnss_settings.probe_node_id):
            return
        source = can_id & 0x7F
        data = bytes(m.data[:m.len])
        if len(data) == 0:
            return
        tail = data[-1]
        start = bool(tail & 0x80)
        end = bool(tail & 0x40)
        toggle = bool(tail & 0x20)
        transfer_id = tail & 0x1F
        partial = p['partial']
        if start:
            partial[source] = {'transfer_id': transfer_id, 'toggle': toggle, 'data': bytearray(data[:-1])}
        else:
            t = partial.get(source)
            if t is None or t['transfer_id'] != transfer_id or t['toggle'] == toggle:
                # a frame was lost
                partial.pop(source, None)
                return
            t['toggle'] = toggle
            t['data'].extend(data[:-1])
        if not end:
            return
        t = partial.pop(source)
        payload = t['data']
        import dronecan
        from dronecan import transport
        T = dronecan.uavcan.protocol.GetNodeInfo
        if not start:
            # multi-frame transfer: the first two bytes are the transfer CRC
            crc = payload[0] | (payload[1] << 8)
            payload = payload[2:]
            if dronecan.dsdl.common.crc16_from_bytes(bytes(payload), initial=T.base_crc) != crc:
                return
        response = T.Response()
        try:
            response._unpack(transport.bits_from_bytes(bytearray(payload)), tao=True)
        except Exception:
            return
        self.node_info[(p['sysid'], source)] = NodeInfo(m.bus, response)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ('CAN_FRAME', 'CANFD_FRAME'):
            self.handle_can_frame(m)
        elif mtype == 'GNSS':
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
        header = ['id', 'param', 'type', 'connection', 'detected', 'device', 'fix', 'sats', 'h_acc', 'v_acc',
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
            device = ''
            if gps_type in dronecan_types:
                node = self.param('GPS%u_CAN_NODEID' % (i+1)) or self.param('GPS%u_CAN_OVRIDE' % (i+1))
                info = self.node_info.get((sysid, node))
                if info is not None:
                    device = '%s %s' % (info.name, info.sw_version)
            row = [str(i), param, type_name, connected.get(i, ''), detected, device]
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
