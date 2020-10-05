#!/usr/bin/env python
'''enable run-time addition and removal of master link, just like --master on the cnd line'''
''' TO USE:
    link add 10.11.12.13:14550
    link list
    link remove 3      # to remove 3rd output
'''

from pymavlink import mavutil
import time, struct, math, sys, fnmatch, traceback, json

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

dataPackets = frozenset(['BAD_DATA','LOG_DATA'])
delayedPackets = frozenset([ 'MISSION_CURRENT', 'SYS_STATUS', 'VFR_HUD',
                  'GPS_RAW_INT', 'SCALED_PRESSURE', 'GLOBAL_POSITION_INT',
                  'NAV_CONTROLLER_OUTPUT' ])
activityPackets = frozenset([ 'HEARTBEAT', 'GPS_RAW_INT', 'GPS_RAW', 'GLOBAL_POSITION_INT', 'SYS_STATUS' ])

preferred_ports = [
    '*FTDI*',
    "*Arduino_Mega_2560*",
    "*3D*",
    "*USB_to_UART*",
    '*Ardu*',
    '*PX4*',
    '*Hex_*',
    '*Holybro_*',
    '*mRo*',
    '*FMU*']

class LinkModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(LinkModule, self).__init__(mpstate, "link", "link control", public=True, multi_vehicle=True)
        self.add_command('link', self.cmd_link, "link control",
                         ["<list|ports>",
                          'add (SERIALPORT)',
                          'attributes (LINK) (ATTRIBUTES)',
                          'remove (LINKS)'])
        self.add_command('vehicle', self.cmd_vehicle, "vehicle control")
        self.add_command('alllinks', self.cmd_alllinks, "send command on all links")
        self.no_fwd_types = set()
        self.no_fwd_types.add("BAD_DATA")
        self.add_completion_function('(SERIALPORT)', self.complete_serial_ports)
        self.add_completion_function('(LINKS)', self.complete_links)
        self.add_completion_function('(LINK)', self.complete_links)
        self.last_altitude_announce = 0.0
        self.vehicle_list = set()

        self.menu_added_console = False
        if mp_util.has_wxpython:
            self.menu_add = MPMenuSubMenu('Add', items=[])
            self.menu_rm = MPMenuSubMenu('Remove', items=[])
            self.menu = MPMenuSubMenu('Link',
                                      items=[self.menu_add,
                                             self.menu_rm,
                                             MPMenuItem('Ports', 'Ports', '# link ports'),
                                             MPMenuItem('List', 'List', '# link list'),
                                             MPMenuItem('Status', 'Status', '# link')])
            self.last_menu_update = 0

    def idle_task(self):
        '''called on idle'''
        if mp_util.has_wxpython and (not self.menu_added_console and self.module('console') is not None):
            self.menu_added_console = True
            # we don't dynamically update these yet due to a wx bug
            self.menu_add.items = [ MPMenuItem(p, p, '# link add %s' % p) for p in self.complete_serial_ports('') ]
            self.menu_rm.items = [ MPMenuItem(p, p, '# link remove %s' % p) for p in self.complete_links('') ]
            self.module('console').add_menu(self.menu)
        for m in self.mpstate.mav_master:
            m.source_system = self.settings.source_system
            m.mav.srcSystem = m.source_system
            m.mav.srcComponent = self.settings.source_component
        # don't let pending statustext wait forever for last chunk:
        for src in self.status.statustexts_by_sysidcompid:
            msgids = self.status.statustexts_by_sysidcompid[src].keys()
            for msgid in msgids:
                pending = self.status.statustexts_by_sysidcompid[src][msgid]
                if time.time() - pending.last_chunk_time > 1:
                    self.emit_accumulated_statustext(src, msgid, pending)

    def complete_serial_ports(self, text):
        '''return list of serial ports'''
        ports = mavutil.auto_detect_serial(preferred_list=preferred_ports)
        return [ p.device for p in ports ]

    def complete_links(self, text):
        '''return list of links'''
        try:
            ret = [ m.address for m in self.mpstate.mav_master ]
            for m in self.mpstate.mav_master:
                ret.append(m.address)
                if hasattr(m, 'label'):
                    ret.append(m.label)
            return ret
        except Exception as e:
            print("Caught exception: %s" % str(e))

    def cmd_link(self, args):
        '''handle link commands'''
        if len(args) < 1:
            self.show_link()
        elif args[0] == "list":
            self.cmd_link_list()
        elif args[0] == "add":
            if len(args) != 2:
                print("Usage: link add LINK")
                print('Usage: e.g. link add 127.0.0.1:9876')
                print('Usage: e.g. link add 127.0.0.1:9876:{"label":"rfd900"}')
                return
            self.cmd_link_add(args[1:])
        elif args[0] == "attributes":
            if len(args) != 3:
                print("Usage: link attributes LINK ATTRIBUTES")
                print('Usage: e.g. link attributes rfd900 {"label":"bob"}')
                return
            self.cmd_link_attributes(args[1:])
        elif args[0] == "ports":
            self.cmd_link_ports()
        elif args[0] == "remove":
            if len(args) != 2:
                print("Usage: link remove LINK")
                return
            self.cmd_link_remove(args[1:])
        else:
            print("usage: link <list|add|remove|attributes>")

    def show_link(self):
        '''show link information'''
        for master in self.mpstate.mav_master:
            linkdelay = (self.status.highest_msec.get(self.target_system,0) - master.highest_msec.get(self.target_system,0))*1.0e-3
            if master.linkerror:
                status = "DOWN"
            else:
                status = "OK"
            sign_string = ''
            try:
                if master.mav.signing.sig_count:
                    if master.mav.signing.secret_key is None:
                        # unsigned/reject counts are not updated if we
                        # don't have a signing secret
                        sign_string = ", (no-signing-secret)"
                    else:
                        sign_string = ", unsigned %u reject %u" % (master.mav.signing.unsigned_count, master.mav.signing.reject_count)
            except AttributeError as e:
                # some mav objects may not have a "signing" attribute
                pass
            print("link %s %s (%u packets, %u bytes, %.2fs delay, %u lost, %.1f%% loss, rate:%uB/s%s)" % (self.link_label(master),
                                                                                    status,
                                                                                    self.status.counters['MasterIn'][master.linknum],
                                                                                    self.status.bytecounters['MasterIn'][master.linknum].total(),
                                                                                    linkdelay,
                                                                                    master.mav_loss,
                                                                                    master.packet_loss(),
                                                                                    self.status.bytecounters['MasterIn'][master.linknum].rate(),
                                                                                    sign_string))

    def cmd_alllinks(self, args):
        '''send command on all links'''
        saved_target = self.mpstate.settings.target_system
        print("Sending to: ", self.vehicle_list)
        for v in sorted(self.vehicle_list):
            self.cmd_vehicle([str(v)])
            self.mpstate.functions.process_stdin(' '.join(args), True)
        self.cmd_vehicle([str(saved_target)])
        
    def cmd_link_list(self):
        '''list links'''
        print("%u links" % len(self.mpstate.mav_master))
        for i in range(len(self.mpstate.mav_master)):
            conn = self.mpstate.mav_master[i]
            if hasattr(conn, 'label'):
                print("%u (%s): %s" % (i, conn.label, conn.address))
            else:
                print("%u: %s" % (i, conn.address))

    def parse_link_attributes(self, some_json):
        '''return a dict based on some_json (empty if json invalid)'''
        try:
            return json.loads(some_json)
        except ValueError:
            print('Invalid JSON argument: {0}'.format(some_json))
        return {}

    def parse_link_descriptor(self, descriptor):
        '''parse e.g. 'udpin:127.0.0.1:9877:{"foo":"bar"}' into
        python structure ("udpin:127.0.0.1:9877", {"foo":"bar"})'''
        optional_attributes = {}
        link_components = descriptor.split(":{", 1)
        device = link_components[0]
        if (len(link_components) == 2 and link_components[1].endswith("}")):
            # assume json
            some_json = "{" + link_components[1]
            optional_attributes = self.parse_link_attributes(some_json)
        return (device, optional_attributes)

    def apply_link_attributes(self, conn, optional_attributes):
        for attr in optional_attributes:
            print("Applying attribute to link: %s = %s" % (attr, optional_attributes[attr]))
            setattr(conn, attr, optional_attributes[attr])

    def link_add(self, descriptor, force_connected=False):
        '''add new link'''
        try:
            (device, optional_attributes) = self.parse_link_descriptor(descriptor)
            print("Connect %s source_system=%d" % (device, self.settings.source_system))
            try:
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                  source_system=self.settings.source_system,
                                                  baud=self.settings.baudrate,
                                                  force_connected=force_connected)
            except Exception as e:
                # try the same thing but without force-connected for
                # backwards-compatability
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                  source_system=self.settings.source_system,
                                                  baud=self.settings.baudrate)
            conn.mav.srcComponent = self.settings.source_component
        except Exception as msg:
            print("Failed to connect to %s : %s" % (descriptor, msg))
            return False
        if self.settings.rtscts:
            conn.set_rtscts(True)
        conn.mav.set_callback(self.master_callback, conn)
        if hasattr(conn.mav, 'set_send_callback'):
            conn.mav.set_send_callback(self.master_send_callback, conn)
        conn.linknum = len(self.mpstate.mav_master)
        conn.linkerror = False
        conn.link_delayed = False
        conn.last_heartbeat = 0
        conn.last_message = 0
        conn.highest_msec = {}
        conn.target_system = self.settings.target_system
        self.apply_link_attributes(conn, optional_attributes)
        self.mpstate.mav_master.append(conn)
        self.status.counters['MasterIn'].append(0)
        self.status.bytecounters['MasterIn'].append(self.status.ByteCounter())
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        return True

    def cmd_link_add(self, args):
        '''add new link'''
        descriptor = args[0]
        print("Adding link %s" % descriptor)
        self.link_add(descriptor)

    def link_attributes(self, link, attributes):
        i = self.find_link(link)
        if i is None:
            print("Connection (%s) not found" % (link,))
            return
        conn = self.mpstate.mav_master[i]
        atts = self.parse_link_attributes(attributes)
        self.apply_link_attributes(conn, atts)

    def cmd_link_attributes(self, args):
        '''change optional link attributes'''
        link = args[0]
        attributes = args[1]
        print("Setting link %s attributes (%s)" % (link, attributes))
        self.link_attributes(link, attributes)

    def cmd_link_ports(self):
        '''show available ports'''
        ports = mavutil.auto_detect_serial(preferred_list=preferred_ports)
        for p in ports:
            print("%s : %s : %s" % (p.device, p.description, p.hwid))

    def find_link(self, device):
        '''find a device based on number, name or label'''
        for i in range(len(self.mpstate.mav_master)):
            conn = self.mpstate.mav_master[i]
            if (str(i) == device or
                conn.address == device or
                getattr(conn, 'label', None) == device):
                return i
        return None

    def cmd_link_remove(self, args):
        '''remove an link'''
        device = args[0]
        if len(self.mpstate.mav_master) <= 1:
            print("Not removing last link")
            return
        i = self.find_link(device)
        if i is None:
            return
        conn = self.mpstate.mav_master[i]
        print("Removing link %s" % conn.address)
        try:
            try:
                mp_util.child_fd_list_remove(conn.port.fileno())
            except Exception:
                pass
            self.mpstate.mav_master[i].close()
        except Exception as msg:
            print(msg)
            pass
        self.mpstate.mav_master.pop(i)
        self.status.counters['MasterIn'].pop(i)
        self.status.bytecounters['MasterIn'].pop(i)
        # renumber the links
        for j in range(len(self.mpstate.mav_master)):
            conn = self.mpstate.mav_master[j]
            conn.linknum = j

    def get_usec(self):
        '''time since 1970 in microseconds'''
        return int(time.time() * 1.0e6)

    def master_send_callback(self, m, master):
        '''called on sending a message'''
        if self.status.watch is not None:
            for msg_type in self.status.watch:
                if fnmatch.fnmatch(m.get_type().upper(), msg_type.upper()):
                    self.mpstate.console.writeln('> '+ str(m))
                    break

        mtype = m.get_type()
        if mtype != 'BAD_DATA' and self.mpstate.logqueue:
            usec = self.get_usec()
            usec = (usec & ~3) | 3 # linknum 3
            self.mpstate.logqueue.put(bytearray(struct.pack('>Q', usec) + m.get_msgbuf()))

    def handle_msec_timestamp(self, m, master):
        '''special handling for MAVLink packets with a time_boot_ms field'''

        if m.get_type() == 'GLOBAL_POSITION_INT':
            # this is fix time, not boot time
            return

        msec = m.time_boot_ms
        sysid = m.get_srcSystem()
        if msec + 30000 < master.highest_msec.get(sysid,0):
            self.say('Time has wrapped')
            print('Time has wrapped', msec, master.highest_msec.get(sysid,0))
            self.status.highest_msec[sysid] = msec
            for mm in self.mpstate.mav_master:
                mm.link_delayed = False
                mm.highest_msec[sysid] = msec
            return

        # we want to detect when a link is delayed
        master.highest_msec[sysid] = msec
        if msec > self.status.highest_msec.get(sysid,0):
            self.status.highest_msec[sysid] = msec
        if msec < self.status.highest_msec.get(sysid,0) and len(self.mpstate.mav_master) > 1 and self.mpstate.settings.checkdelay:
            master.link_delayed = True
        else:
            master.link_delayed = False

    def colors_for_severity(self, severity):
        severity_colors = {
            # tuple is (fg, bg) (as in "white on red")
            mavutil.mavlink.MAV_SEVERITY_EMERGENCY: ('white', 'red'),
            mavutil.mavlink.MAV_SEVERITY_ALERT: ('white', 'red'),
            mavutil.mavlink.MAV_SEVERITY_CRITICAL: ('white', 'red'),
            mavutil.mavlink.MAV_SEVERITY_ERROR: ('black', 'orange'),
            mavutil.mavlink.MAV_SEVERITY_WARNING: ('black', 'orange'),
            mavutil.mavlink.MAV_SEVERITY_NOTICE: ('black', 'yellow'),
            mavutil.mavlink.MAV_SEVERITY_INFO: ('white', 'green'),
            mavutil.mavlink.MAV_SEVERITY_DEBUG: ('white', 'green'),
        }
        try:
            return severity_colors[severity]
        except Exception as e:
            print("Exception: %s" % str(e))
            return ('white', 'red')

    def report_altitude(self, altitude):
        '''possibly report a new altitude'''
        master = self.master
        if getattr(self.console, 'ElevationMap', None) is not None and self.mpstate.settings.basealt != 0:
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0)*1.0e-7
            lon = master.field('GLOBAL_POSITION_INT', 'lon', 0)*1.0e-7
            alt1 = self.console.ElevationMap.GetElevation(lat, lon)
            if alt1 is not None:
                alt2 = self.mpstate.settings.basealt
                altitude += alt2 - alt1
        self.status.altitude = altitude
        altitude_converted = self.height_convert_units(altitude)
        if (int(self.mpstate.settings.altreadout) > 0 and
            math.fabs(altitude_converted - self.last_altitude_announce) >=
            int(self.settings.altreadout)):
            self.last_altitude_announce = altitude_converted
            rounded_alt = int(self.settings.altreadout) * ((self.settings.altreadout/2 + int(altitude_converted)) / int(self.settings.altreadout))
            self.say("height %u" % rounded_alt, priority='notification')


    def emit_accumulated_statustext(self, key, id, pending):
        out = pending.accumulated_statustext()
        if out != self.status.last_apm_msg or time.time() > self.status.last_apm_msg_time+2:
            (fg, bg) = self.colors_for_severity(pending.severity)
            out = pending.accumulated_statustext()
            self.mpstate.console.writeln("APM: %s" % out, bg=bg, fg=fg)
            self.status.last_apm_msg = out
            self.status.last_apm_msg_time = time.time()
        del self.status.statustexts_by_sysidcompid[key][id]

    def master_msg_handling(self, m, master):
        '''link message handling for an upstream link'''
        if self.settings.target_system != 0 and m.get_srcSystem() != self.settings.target_system:
            # don't process messages not from our target
            if m.get_type() == "BAD_DATA":
                if self.mpstate.settings.shownoise and mavutil.all_printable(m.data):
                    out = m.data
                    if type(m.data) == bytearray:
                        out = m.data.decode('ascii')
                    self.mpstate.console.write(out, bg='red')
            return

        if self.settings.target_system != 0 and master.target_system != self.settings.target_system:
            # keep the pymavlink level target system aligned with the MAVProxy setting
            master.target_system = self.settings.target_system

        if self.settings.target_component != 0 and master.target_component != self.settings.target_component:
            # keep the pymavlink level target component aligned with the MAVProxy setting
            print("change target_component %u" % self.settings.target_component)
            master.target_component = self.settings.target_component
            
        mtype = m.get_type()

        if mtype == 'HEARTBEAT' and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            if self.settings.target_system == 0 and self.settings.target_system != m.get_srcSystem():
                self.settings.target_system = m.get_srcSystem()
                self.say("online system %u" % self.settings.target_system,'message')
                for mav in self.mpstate.mav_master:
                    mav.target_system = self.settings.target_system

            if self.status.heartbeat_error:
                self.status.heartbeat_error = False
                self.say("heartbeat OK")
            if master.linkerror:
                master.linkerror = False
                self.say("link %s OK" % (self.link_label(master)))
            self.status.last_heartbeat = time.time()
            master.last_heartbeat = self.status.last_heartbeat

            armed = self.master.motors_armed()
            if armed != self.status.armed:
                self.status.armed = armed
                if armed:
                    self.say("ARMED")
                else:
                    self.say("DISARMED")

            if master.flightmode != self.status.flightmode:
                self.status.flightmode = master.flightmode
                if self.mpstate.functions.input_handler is None:
                    self.set_prompt(self.status.flightmode + "> ")

            if master.flightmode != self.status.last_mode_announced and time.time() > self.status.last_mode_announce + 2:
                    self.status.last_mode_announce = time.time()
                    self.status.last_mode_announced = master.flightmode
                    self.say("Mode " + self.status.flightmode)

            if m.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
                self.mpstate.vehicle_type = 'plane'
                self.mpstate.vehicle_name = 'ArduPlane'
            elif m.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                            mavutil.mavlink.MAV_TYPE_SURFACE_BOAT]:
                self.mpstate.vehicle_type = 'rover'
                self.mpstate.vehicle_name = 'APMrover2'
            elif m.type in [mavutil.mavlink.MAV_TYPE_SUBMARINE]:
                self.mpstate.vehicle_type = 'sub'
                self.mpstate.vehicle_name = 'ArduSub'
            elif m.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
                self.mpstate.vehicle_type = 'copter'
                self.mpstate.vehicle_name = 'ArduCopter'
            elif m.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
                self.mpstate.vehicle_type = 'antenna'
                self.mpstate.vehicle_name = 'AntennaTracker'

        elif mtype == 'STATUSTEXT':

            class PendingText(object):
                def __init__(self):
                    self.expected_count = None
                    self.severity = None
                    self.chunks = {}
                    self.start_time = time.time()
                    self.last_chunk_time = time.time()

                def add_chunk(self, m): # m is a statustext message
                    self.severity = m.severity
                    self.last_chunk_time = time.time()
                    if hasattr(m, 'chunk_seq'):
                        # mavlink extensions are present.
                        chunk_seq = m.chunk_seq
                        mid = m.id
                    else:
                        # Note that m.id may still exist!  It will
                        # contain the value 253, STATUSTEXT's mavlink
                        # message id.  Thus our reliance on the
                        # presence of chunk_seq.
                        chunk_seq = 0
                        mid = 0
                    self.chunks[chunk_seq] = m.text

                    if len(m.text) != 50 or mid == 0:
                        self.expected_count = chunk_seq + 1;

                def complete(self):
                    return (self.expected_count is not None and
                            self.expected_count == len(self.chunks))

                def accumulated_statustext(self):
                    next_expected_chunk = 0
                    out = ""
                    for chunk_seq in sorted(self.chunks.keys()):
                        if chunk_seq != next_expected_chunk:
                            out += " ... "
                            next_expected_chunk = chunk_seq
                        out += self.chunks[chunk_seq]
                        next_expected_chunk += 1

                    return out

            key = "%s.%s" % (m.get_srcSystem(), m.get_srcComponent())
            if key not in self.status.statustexts_by_sysidcompid:
                self.status.statustexts_by_sysidcompid[key] = {}
            if hasattr(m, 'chunk_seq'):
                mid = m.id
            else:
                # m.id will have the value of 253, STATUSTEXT mavlink id
                mid = 0
            if mid not in self.status.statustexts_by_sysidcompid[key]:
                self.status.statustexts_by_sysidcompid[key][mid] = PendingText()

            pending = self.status.statustexts_by_sysidcompid[key][mid]
            pending.add_chunk(m)
            if pending.complete():
                # we have all of the chunks!
                self.emit_accumulated_statustext(key, mid, pending)

        elif mtype == "VFR_HUD":
            have_gps_lock = False
            if 'GPS_RAW' in self.status.msgs and self.status.msgs['GPS_RAW'].fix_type == 2:
                have_gps_lock = True
            elif 'GPS_RAW_INT' in self.status.msgs and self.status.msgs['GPS_RAW_INT'].fix_type == 3:
                have_gps_lock = True
            if have_gps_lock and not self.status.have_gps_lock and m.alt != 0:
                self.say("GPS lock at %u meters" % m.alt, priority='notification')
                self.status.have_gps_lock = True

        elif mtype == "GPS_RAW":
            if self.status.have_gps_lock:
                if m.fix_type != 2 and not self.status.lost_gps_lock and (time.time() - self.status.last_gps_lock) > 3:
                    self.say("GPS fix lost")
                    self.status.lost_gps_lock = True
                if m.fix_type == 2 and self.status.lost_gps_lock:
                    self.say("GPS OK")
                    self.status.lost_gps_lock = False
                if m.fix_type == 2:
                    self.status.last_gps_lock = time.time()

        elif mtype == "GPS_RAW_INT":
            if self.status.have_gps_lock:
                if m.fix_type < 3 and not self.status.lost_gps_lock and (time.time() - self.status.last_gps_lock) > 3:
                    self.say("GPS fix lost")
                    self.status.lost_gps_lock = True
                if m.fix_type >= 3 and self.status.lost_gps_lock:
                    self.say("GPS OK")
                    self.status.lost_gps_lock = False
                if m.fix_type >= 3:
                    self.status.last_gps_lock = time.time()

        elif mtype == "NAV_CONTROLLER_OUTPUT" and self.status.flightmode == "AUTO" and self.mpstate.settings.distreadout:
            rounded_dist = int(m.wp_dist/self.mpstate.settings.distreadout)*self.mpstate.settings.distreadout
            if math.fabs(rounded_dist - self.status.last_distance_announce) >= self.mpstate.settings.distreadout:
                if rounded_dist != 0:
                    self.say("%u" % rounded_dist, priority="progress")
            self.status.last_distance_announce = rounded_dist

        elif mtype == "GLOBAL_POSITION_INT":
            self.report_altitude(m.relative_alt*0.001)

        elif mtype == "COMPASSMOT_STATUS":
            print(m)

        elif mtype == "SIMSTATE":
            self.mpstate.is_sitl = True

        elif mtype == "ATTITUDE":
            att_time = m.time_boot_ms * 0.001
            self.mpstate.attitude_time_s = max(self.mpstate.attitude_time_s, att_time)
            if self.mpstate.attitude_time_s - att_time > 120:
                # cope with wrap
                self.mpstate.attitude_time_s = att_time

        elif mtype == "COMMAND_ACK":
            try:
                cmd = mavutil.mavlink.enums["MAV_CMD"][m.command].name
                cmd = cmd[8:]
                res = mavutil.mavlink.enums["MAV_RESULT"][m.result].name
                res = res[11:]
                self.mpstate.console.writeln("Got COMMAND_ACK: %s: %s" % (cmd, res))
            except Exception:
                self.mpstate.console.writeln("Got MAVLink msg: %s" % m)

            if m.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.say("Calibrated")
                elif m.result == mavutil.mavlink.MAV_RESULT_FAILED:
                    self.say("Calibration failed")
                elif m.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    self.say("Calibration unsupported")
                elif m.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    self.say("Calibration temporarily rejected")
                else:
                    self.say("Calibration response (%u)" % m.result)
        elif mtype == "MISSION_ACK":
            try:
                t = mavutil.mavlink.enums["MAV_MISSION_TYPE"][m.mission_type].name
                t = t[12:]
                res = mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name
                res = res[12:]
                self.mpstate.console.writeln("Got MISSION_ACK: %s: %s" % (t, res))
            except Exception as e:
                self.mpstate.console.writeln("Got MAVLink msg: %s" % m)
        else:
            #self.mpstate.console.writeln("Got MAVLink msg: %s" % m)
            pass

        if self.status.watch is not None:
            for msg_type in self.status.watch:
                if fnmatch.fnmatch(mtype.upper(), msg_type.upper()):
                    self.mpstate.console.writeln('< '+ str(m))
                    break

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()

        if type == 'HEARTBEAT':
            sysid = msg.get_srcSystem()
            if not sysid in self.vehicle_list and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.vehicle_list.add(sysid)

    def master_callback(self, m, master):
        '''process mavlink message m on master, sending any messages to recipients'''

        # see if it is handled by a specialised sysid connection
        sysid = m.get_srcSystem()
        mtype = m.get_type()
        if sysid in self.mpstate.sysid_outputs:
            self.mpstate.sysid_outputs[sysid].write(m.get_msgbuf())
            if mtype == "GLOBAL_POSITION_INT":
                for modname in 'map', 'asterix', 'NMEA', 'NMEA2':
                    mod = self.module(modname)
                    if mod is not None:
                        mod.set_secondary_vehicle_position(m)
            return

        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)
        self.status.counters['MasterIn'][master.linknum] += 1

        if mtype == 'GLOBAL_POSITION_INT':
            # send GLOBAL_POSITION_INT to 2nd GCS for 2nd vehicle display
            for sysid in self.mpstate.sysid_outputs:
                self.mpstate.sysid_outputs[sysid].write(m.get_msgbuf())

            if self.mpstate.settings.fwdpos:
                for link in self.mpstate.mav_master:
                    if link != master:
                        link.write(m.get_msgbuf())

        # and log them
        if mtype not in dataPackets and self.mpstate.logqueue:
            # put link number in bottom 2 bits, so we can analyse packet
            # delay in saved logs
            usec = self.get_usec()
            usec = (usec & ~3) | master.linknum
            self.mpstate.logqueue.put(bytearray(struct.pack('>Q', usec) + m.get_msgbuf()))

        # keep the last message of each type around
        self.status.msgs[mtype] = m
        instance_field = getattr(m, '_instance_field', None)
        if mtype not in self.status.msg_count:
            self.status.msg_count[mtype] = 0
        self.status.msg_count[mtype] += 1

        if instance_field is not None:
            instance_value = getattr(m, instance_field, None)
            if instance_value is not None:
                mtype_instance = "%s[%s]" % (mtype, instance_value)
                self.status.msgs[mtype_instance] = m
                if mtype_instance not in self.status.msg_count:
                    self.status.msg_count[mtype_instance] = 0
                self.status.msg_count[mtype_instance] += 1
        
        if m.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_GIMBAL and mtype == 'HEARTBEAT':
            # silence gimbal heartbeat packets for now
            return

        if getattr(m, 'time_boot_ms', None) is not None and self.settings.target_system == m.get_srcSystem():
            # update link_delayed attribute
            self.handle_msec_timestamp(m, master)

        if mtype in activityPackets:
            if master.linkerror:
                master.linkerror = False
                self.say("link %s OK" % (self.link_label(master)))
            self.status.last_message = time.time()
            master.last_message = self.status.last_message

        if master.link_delayed and self.mpstate.settings.checkdelay:
            # don't process delayed packets that cause double reporting
            if mtype in delayedPackets:
                return

        self.master_msg_handling(m, master)

        # don't pass along bad data
        if mtype != 'BAD_DATA':
            # pass messages along to listeners, except for REQUEST_DATA_STREAM, which
            # would lead a conflict in stream rate setting between mavproxy and the other
            # GCS
            if self.mpstate.settings.mavfwd_rate or mtype != 'REQUEST_DATA_STREAM':
                if mtype not in self.no_fwd_types:
                    for r in self.mpstate.mav_outputs:
                        r.write(m.get_msgbuf())

            sysid = m.get_srcSystem()
            target_sysid = self.target_system

            # pass to modules
            for (mod,pm) in self.mpstate.modules:
                if not hasattr(mod, 'mavlink_packet'):
                    continue
                if not mod.multi_vehicle and sysid != target_sysid:
                    # only pass packets not from our target to modules that
                    # have marked themselves as being multi-vehicle capable
                    continue
                try:
                    mod.mavlink_packet(m)
                except Exception as msg:
                    if self.mpstate.settings.moddebug == 1:
                        print(msg)
                    elif self.mpstate.settings.moddebug > 1:
                        exc_type, exc_value, exc_traceback = sys.exc_info()
                        traceback.print_exception(exc_type, exc_value, exc_traceback,
                                                  limit=2, file=sys.stdout)

    def cmd_vehicle(self, args):
        '''handle vehicle commands'''
        if len(args) < 1:
            print("Usage: vehicle SYSID[:COMPID]")
            return
        a = args[0].split(':')
        self.mpstate.settings.target_system = int(a[0])
        if len(a) > 1:
            self.mpstate.settings.target_component = int(a[1])

        # change default link based on most recent HEARTBEAT
        best_link = 0
        best_timestamp = 0
        for i in range(len(self.mpstate.mav_master)):
            m = self.mpstate.mav_master[i]
            m.target_system = self.mpstate.settings.target_system
            m.target_component = self.mpstate.settings.target_component
            if 'HEARTBEAT' in m.messages:
                stamp = m.messages['HEARTBEAT']._timestamp
                src_system = m.messages['HEARTBEAT'].get_srcSystem()
                if stamp > best_timestamp:
                    best_link = i
                    best_timestamp = stamp
            m.link_delayed = False                    
        self.mpstate.settings.link = best_link + 1
        print("Set vehicle %s (link %u)" % (args[0], best_link+1))

def init(mpstate):
    '''initialise module'''
    return LinkModule(mpstate)
