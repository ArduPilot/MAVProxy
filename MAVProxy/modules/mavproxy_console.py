"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys, math, time, re

from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.mp_menu import *

class DisplayItem:
    def __init__(self, fmt, expression, row):
        self.expression = expression.strip('"\'')
        self.format = fmt.strip('"\'')
        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        self.msg_types = set(re.findall(re_caps, expression))
        print(self.expression, self.msg_types)
        self.row = row

class ConsoleModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ConsoleModule, self).__init__(mpstate, "console", "GUI console", public=True, multi_vehicle=True)
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0
        self.max_link_num = 0
        self.last_sys_status_health = 0
        self.last_sys_status_errors_announce = 0
        self.user_added = {}
        self.safety_on = False
        self.add_command('console', self.cmd_console, "console module", ['add','list','remove'])
        mpstate.console = wxconsole.MessageConsole(title='Console')

        # setup some default status information
        mpstate.console.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
        mpstate.console.set_status('SysID', '', row=0, fg='blue')
        mpstate.console.set_status('ARM', 'ARM', fg='grey', row=0)
        mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
        mpstate.console.set_status('GPS2', '', fg='red', row=0)
        mpstate.console.set_status('Vcc', 'Vcc: --', fg='red', row=0)
        mpstate.console.set_status('Radio', 'Radio: --', row=0)
        mpstate.console.set_status('INS', 'INS', fg='grey', row=0)
        mpstate.console.set_status('MAG', 'MAG', fg='grey', row=0)
        mpstate.console.set_status('AS', 'AS', fg='grey', row=0)
        mpstate.console.set_status('RNG', 'RNG', fg='grey', row=0)
        mpstate.console.set_status('AHRS', 'AHRS', fg='grey', row=0)
        mpstate.console.set_status('EKF', 'EKF', fg='grey', row=0)
        mpstate.console.set_status('LOG', 'LOG', fg='grey', row=0)
        mpstate.console.set_status('Heading', 'Hdg ---/---', row=2)
        mpstate.console.set_status('Alt', 'Alt ---', row=2)
        mpstate.console.set_status('AGL', 'AGL ---/---', row=2)
        mpstate.console.set_status('AirSpeed', 'AirSpeed --', row=2)
        mpstate.console.set_status('GPSSpeed', 'GPSSpeed --', row=2)
        mpstate.console.set_status('Thr', 'Thr ---', row=2)
        mpstate.console.set_status('Roll', 'Roll ---', row=2)
        mpstate.console.set_status('Pitch', 'Pitch ---', row=2)
        mpstate.console.set_status('Wind', 'Wind ---/---', row=2)
        mpstate.console.set_status('WP', 'WP --', row=3)
        mpstate.console.set_status('WPDist', 'Distance ---', row=3)
        mpstate.console.set_status('WPBearing', 'Bearing ---', row=3)
        mpstate.console.set_status('AltError', 'AltError --', row=3)
        mpstate.console.set_status('AspdError', 'AspdError --', row=3)
        mpstate.console.set_status('FlightTime', 'FlightTime --', row=3)
        mpstate.console.set_status('ETR', 'ETR --', row=3)
        mpstate.console.set_status('Params', 'Param ---/---', row=3)

        mpstate.console.ElevationMap = mp_elevation.ElevationModel()

        self.vehicle_list = []
        self.vehicle_menu = None
        self.vehicle_name_by_sysid = {}
        self.component_name = {}
        self.last_param_sysid_timestamp = None

        # create the main menu
        if mp_util.has_wxpython:
            self.menu = MPMenuTop([])
            self.add_menu(MPMenuSubMenu('MAVProxy',
                                        items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                               MPMenuItem('Map', 'Load Map', '# module load map')]))
            self.vehicle_menu = MPMenuSubMenu('Vehicle', items=[])
            self.add_menu(self.vehicle_menu)

    def cmd_console(self, args):
        usage = 'usage: console <add|list|remove>'
        if len(args) < 1:
            print(usage)
            return
        cmd = args[0]
        if cmd == 'add':
            if len(args) < 4:
                print("usage: console add ID FORMAT EXPRESSION <row>")
                return
            if len(args) > 4:
                row = int(args[4])
            else:
                row = 4
            self.user_added[args[1]] = DisplayItem(args[2], args[3], row)
        elif cmd == 'list':
            for k in sorted(self.user_added.keys()):
                d = self.user_added[k]
                print("%s : FMT=%s EXPR=%s ROW=%u" % (k, d.format, d.expression, d.row))
        elif cmd == 'remove':
            if len(args) < 2:
                print("usage: console remove ID")
                return
            id = args[1]
            if id in self.user_added:
                self.user_added.pop(id)
        else:
            print(usage)

    def add_menu(self, menu):
        '''add a new menu'''
        self.menu.add(menu)
        self.mpstate.console.set_menu(self.menu, self.menu_callback)

    def remove_menu(self, menu):
        '''add a new menu'''
        self.menu.remove(menu)
        self.mpstate.console.set_menu(self.menu, self.menu_callback)

    def unload(self):
        '''unload module'''
        self.mpstate.console.close()
        self.mpstate.console = textconsole.SimpleConsole()

    def menu_callback(self, m):
        '''called on menu selection'''
        if m.returnkey.startswith('# '):
            cmd = m.returnkey[2:]
            if m.handler is not None:
                if m.handler_result is None:
                    return
                cmd += m.handler_result
            self.mpstate.functions.process_stdin(cmd)
        if m.returnkey == 'menuSettings':
            wxsettings.WXSettings(self.settings)


    def estimated_time_remaining(self, lat, lon, wpnum, speed):
        '''estimate time remaining in mission in seconds'''
        idx = wpnum
        if wpnum >= self.module('wp').wploader.count():
            return 0
        distance = 0
        done = set()
        while idx < self.module('wp').wploader.count():
            if idx in done:
                break
            done.add(idx)
            w = self.module('wp').wploader.wp(idx)
            if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
                idx = int(w.param1)
                continue
            idx += 1
            if (w.x != 0 or w.y != 0) and w.command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                                                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]:
                distance += mp_util.gps_distance(lat, lon, w.x, w.y)
                lat = w.x
                lon = w.y
                if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    break
        return distance / speed

    def vehicle_type_string(self, hb):
        '''return vehicle type string from a heartbeat'''
        if hb.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            return 'Plane'
        if hb.type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            return 'Rover'
        if hb.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
            return 'Boat'
        if hb.type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
            return 'Sub'
        if hb.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                           mavutil.mavlink.MAV_TYPE_COAXIAL,
                           mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                           mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                           mavutil.mavlink.MAV_TYPE_TRICOPTER,
                           mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
            return "Copter"
        if hb.type == mavutil.mavlink.MAV_TYPE_HELICOPTER:
            return "Heli"
        if hb.type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
            return "Tracker"
        return "UNKNOWN(%u)" % hb.type

    def component_type_string(self, hb):
        # note that we rely on vehicle_type_string for basic vehicle types
        if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
            return "GCS"
        elif hb.type == mavutil.mavlink.MAV_TYPE_GIMBAL:
            return "Gimbal"
        elif hb.type == mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER:
            return "CC"
        elif hb.type == mavutil.mavlink.MAV_TYPE_GENERIC:
            return "Generic"
        return self.vehicle_type_string(hb)

    def update_vehicle_menu(self):
        '''update menu for new vehicles'''
        self.vehicle_menu.items = []
        for s in sorted(self.vehicle_list):
            clist = self.module('param').get_component_id_list(s)
            if len(clist) == 1:
                name = 'SysID %u: %s' % (s, self.vehicle_name_by_sysid[s])
                self.vehicle_menu.items.append(MPMenuItem(name, name, '# vehicle %u' % s))
            else:
                for c in sorted(clist):
                    try:
                        name = 'SysID %u[%u]: %s' % (s, c, self.component_name[s][c])
                    except KeyError as e:
                        name = 'SysID %u[%u]: ?' % (s,c)
                    self.vehicle_menu.items.append(MPMenuItem(name, name, '# vehicle %u:%u' % (s,c)))
        self.mpstate.console.set_menu(self.menu, self.menu_callback)
    
    def add_new_vehicle(self, hb):
        '''add a new vehicle'''
        if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
            return
        sysid = hb.get_srcSystem()
        self.vehicle_list.append(sysid)
        self.vehicle_name_by_sysid[sysid] = self.vehicle_type_string(hb)
        self.update_vehicle_menu()

    def check_critical_error(self, msg):
        '''check for any error bits being set in SYS_STATUS'''
        errors = msg.errors_count1 | (msg.errors_count2<<16)
        if errors == 0:
            return
        now = time.time()
        if now - self.last_sys_status_errors_announce > self.mpstate.settings.sys_status_error_warn_interval:
            self.last_sys_status_errors_announce = now
            self.say("Critical failure 0x%x sysid=%u" % (errors, msg.get_srcSystem()))

        
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if not isinstance(self.console, wxconsole.MessageConsole):
            return
        if not self.console.is_alive():
            self.mpstate.console = textconsole.SimpleConsole()
            return
        type = msg.get_type()
        sysid = msg.get_srcSystem()

        if type == 'HEARTBEAT':
            if not sysid in self.vehicle_list:
                self.add_new_vehicle(msg)
            if sysid not in self.component_name:
                self.component_name[sysid] = {}
            compid = msg.get_srcComponent()
            if compid not in self.component_name[sysid]:
                self.component_name[sysid][compid] = self.component_type_string(msg)
                self.update_vehicle_menu()

        if self.last_param_sysid_timestamp != self.module('param').new_sysid_timestamp:
            '''a new component ID has appeared for parameters'''
            self.last_param_sysid_timestamp = self.module('param').new_sysid_timestamp
            self.update_vehicle_menu()

        if type in ['RADIO', 'RADIO_STATUS']:
            # handle RADIO msgs from all vehicles
            if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
                fg = 'red'
            else:
                fg = 'black'
            self.console.set_status('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)

        if type == 'SYS_STATUS':
            self.check_critical_error(msg)

        if not self.is_primary_vehicle(msg):
            # don't process msgs from other than primary vehicle, other than
            # updating vehicle list
            return

        master = self.master
        # add some status fields
        if type in [ 'GPS_RAW_INT', 'GPS2_RAW' ]:
            if type == 'GPS_RAW_INT':
                field = 'GPS'
                prefix = 'GPS:'
            else:
                field = 'GPS2'
                prefix = 'GPS2'
            nsats = msg.satellites_visible
            fix_type = msg.fix_type
            if fix_type >= 3:
                self.console.set_status(field, '%s OK%s (%u)' % (prefix, fix_type, nsats), fg='green')
            else:
                self.console.set_status(field, '%s %u (%u)' % (prefix, fix_type, nsats), fg='red')
            if type == 'GPS_RAW_INT':
                vfr_hud_heading = master.field('VFR_HUD', 'heading', None)
                gps_heading = int(msg.cog * 0.01)
                if vfr_hud_heading is None:
                    vfr_hud_heading = '---'
                else:
                    vfr_hud_heading = '%3u' % vfr_hud_heading
                self.console.set_status('Heading', 'Hdg %s/%3u' %
                                        (vfr_hud_heading, gps_heading))
        elif type == 'VFR_HUD':
            if master.mavlink10():
                alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
            else:
                alt = master.field('GPS_RAW', 'alt', 0)
            home = self.module('wp').get_home()
            if home is not None:
                home_lat = home.x
                home_lng = home.y
            else:
                home_lat = None
                home_lng = None
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
            lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
            rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
            agl_alt = None
            if self.settings.basealt != 0:
                agl_alt = self.console.ElevationMap.GetElevation(lat, lng)
                if agl_alt is not None:
                    agl_alt = self.settings.basealt - agl_alt
            else:
                try:
                    agl_alt_home = self.console.ElevationMap.GetElevation(home_lat, home_lng)
                except Exception as ex:
                    print(ex)
                    agl_alt_home = None
                if agl_alt_home is not None:
                    agl_alt = self.console.ElevationMap.GetElevation(lat, lng)
                if agl_alt is not None:
                    agl_alt = agl_alt_home - agl_alt
            if agl_alt is not None:
                agl_alt += rel_alt
                vehicle_agl = master.field('TERRAIN_REPORT', 'current_height', None)
                if vehicle_agl is None:
                    vehicle_agl = '---'
                else:
                    vehicle_agl = self.height_string(vehicle_agl)
                self.console.set_status('AGL', 'AGL %s/%s' % (self.height_string(agl_alt), vehicle_agl))
            self.console.set_status('Alt', 'Alt %s' % self.height_string(rel_alt))
            self.console.set_status('AirSpeed', 'AirSpeed %s' % self.speed_string(msg.airspeed))
            self.console.set_status('GPSSpeed', 'GPSSpeed %s' % self.speed_string(msg.groundspeed))
            self.console.set_status('Thr', 'Thr %u' % msg.throttle)
            t = time.localtime(msg._timestamp)
            flying = False
            if self.mpstate.vehicle_type == 'copter':
                flying = self.master.motors_armed()
            else:
                flying = msg.groundspeed > 3
            if flying and not self.in_air:
                self.in_air = True
                self.start_time = time.mktime(t)
            elif flying and self.in_air:
                self.total_time = time.mktime(t) - self.start_time
                self.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
            elif not flying and self.in_air:
                self.in_air = False
                self.total_time = time.mktime(t) - self.start_time
                self.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
        elif type == 'ATTITUDE':
            self.console.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
            self.console.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))
        elif type in ['SYS_STATUS']:
            sensors = { 'AS'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                        'MAG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'INS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS,
                        'RC'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
                        'TERR' : mavutil.mavlink.MAV_SYS_STATUS_TERRAIN,
                        'RNG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION,
                        'LOG'  : mavutil.mavlink.MAV_SYS_STATUS_LOGGING,
            }
            announce = [ 'RC' ]
            for s in sensors.keys():
                bits = sensors[s]
                present = ((msg.onboard_control_sensors_present & bits) == bits)
                enabled = ((msg.onboard_control_sensors_enabled & bits) == bits)
                healthy = ((msg.onboard_control_sensors_health & bits) == bits)
                if not present:
                    fg = 'black'
                elif not enabled:
                    fg = 'grey'
                elif not healthy:
                    fg = 'red'
                else:
                    fg = 'green'
                # for terrain show yellow if still loading
                if s == 'TERR' and fg == 'green' and master.field('TERRAIN_REPORT', 'pending', 0) != 0:
                    fg = 'yellow'
                self.console.set_status(s, s, fg=fg)
            for s in announce:
                bits = sensors[s]
                enabled = ((msg.onboard_control_sensors_enabled & bits) == bits)
                healthy = ((msg.onboard_control_sensors_health & bits) == bits)
                was_healthy = ((self.last_sys_status_health & bits) == bits)
                if enabled and not healthy and was_healthy:
                    self.say("%s fail" % s)
            self.last_sys_status_health = msg.onboard_control_sensors_health

            if ((msg.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) == 0):
                self.safety_on = True
            else:
                self.safety_on = False                

        elif type == 'WIND':
            self.console.set_status('Wind', 'Wind %u/%s' % (msg.direction, self.speed_string(msg.speed)))

        elif type == 'EKF_STATUS_REPORT':
            highest = 0.0
            vars = ['velocity_variance',
                    'pos_horiz_variance',
                    'pos_vert_variance',
                    'compass_variance',
                    'terrain_alt_variance']
            for var in vars:
                v = getattr(msg, var, 0)
                highest = max(v, highest)
            if highest >= 1.0:
                fg = 'red'
            elif highest >= 0.5:
                fg = 'orange'
            else:
                fg = 'green'
            self.console.set_status('EKF', 'EKF', fg=fg)

        elif type == 'HWSTATUS':
            if msg.Vcc >= 4600 and msg.Vcc <= 5300:
                fg = 'green'
            else:
                fg = 'red'
            self.console.set_status('Vcc', 'Vcc %.2f' % (msg.Vcc * 0.001), fg=fg)
        elif type == 'POWER_STATUS':
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_CHANGED:
                fg = 'red'
            else:
                fg = 'green'
            status = 'PWR:'
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_USB_CONNECTED:
                status += 'U'
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_BRICK_VALID:
                status += 'B'
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_SERVO_VALID:
                status += 'S'
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_OVERCURRENT:
                status += 'O1'
            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:
                status += 'O2'
            self.console.set_status('PWR', status, fg=fg)
            self.console.set_status('Srv', 'Srv %.2f' % (msg.Vservo*0.001), fg='green')
        elif type == 'HEARTBEAT':
            fmode = master.flightmode
            if self.settings.vehicle_name:
                fmode = self.settings.vehicle_name + ':' + fmode
            self.console.set_status('Mode', '%s' % fmode, fg='blue')
            if len(self.vehicle_list) > 1:
                self.console.set_status('SysID', 'Sys:%u' % sysid, fg='blue')
            if self.master.motors_armed():
                arm_colour = 'green'
            else:
                arm_colour = 'red'
            armstring = 'ARM'
            # add safety switch state
            if self.safety_on:
                armstring += '(SAFE)'
            self.console.set_status('ARM', armstring, fg=arm_colour)
            if self.max_link_num != len(self.mpstate.mav_master):
                for i in range(self.max_link_num):
                    self.console.set_status('Link%u'%(i+1), '', row=1)
                self.max_link_num = len(self.mpstate.mav_master)
            for m in self.mpstate.mav_master:
                if self.mpstate.settings.checkdelay:
                    linkdelay = (self.mpstate.status.highest_msec.get(sysid, 0) - m.highest_msec.get(sysid,0))*1.0e-3
                else:
                    linkdelay = 0
                linkline = "Link %s " % (self.link_label(m))
                fg = 'dark green'
                if m.linkerror:
                    linkline += "down"
                    fg = 'red'
                else:
                    packets_rcvd_percentage = 100
                    if (m.mav_count+m.mav_loss) != 0: #avoid divide-by-zero
                        packets_rcvd_percentage = (100.0 * m.mav_count) / (m.mav_count + m.mav_loss)

                    linkbits = ["%u pkts" % m.mav_count,
                                "%u lost" % m.mav_loss,
                                "%.2fs delay" % linkdelay,
                    ]
                    try:
                        if m.mav.signing.sig_count:
                            # other end is sending us signed packets
                            if not m.mav.signing.secret_key:
                                # we've received signed packets but
                                # can't verify them
                                fg = 'orange'
                                linkbits.append("!KEY")
                            elif not m.mav.signing.sign_outgoing:
                                # we've received signed packets but aren't
                                # signing outselves; this can lead to hairloss
                                fg = 'orange'
                                linkbits.append("!SIGNING")
                            if m.mav.signing.badsig_count:
                                fg = 'orange'
                                linkbits.append("%u badsigs" % m.mav.signing.badsig_count)
                    except AttributeError as e:
                        # mav.signing.sig_count probably doesn't exist
                        pass

                    linkline += "OK {rcv_pct:.1f}% ({bits})".format(
                        rcv_pct=packets_rcvd_percentage,
                        bits=", ".join(linkbits))

                    if linkdelay > 1 and fg == 'dark green':
                        fg = 'orange'

                self.console.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
        elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
            wpmax = self.module('wp').wploader.count()
            if wpmax > 0:
                wpmax = "/%u" % wpmax
            else:
                wpmax = ""
            self.console.set_status('WP', 'WP %u%s' % (msg.seq, wpmax))
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
            lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
            if lat != 0 and lng != 0:
                airspeed = master.field('VFR_HUD', 'airspeed', 30)
                if abs(airspeed - self.speed) > 5:
                    self.speed = airspeed
                else:
                    self.speed = 0.98*self.speed + 0.02*airspeed
                self.speed = max(1, self.speed)
                time_remaining = int(self.estimated_time_remaining(lat, lng, msg.seq, self.speed))
                self.console.set_status('ETR', 'ETR %u:%02u' % (time_remaining/60, time_remaining%60))

        elif type == 'NAV_CONTROLLER_OUTPUT':
            self.console.set_status('WPDist', 'Distance %s' % self.dist_string(msg.wp_dist))
            self.console.set_status('WPBearing', 'Bearing %u' % msg.target_bearing)
            if msg.alt_error > 0:
                alt_error_sign = "(L)"
            else:
                alt_error_sign = "(H)"
            if msg.aspd_error > 0:
                aspd_error_sign = "(L)"
            else:
                aspd_error_sign = "(H)"
            if math.isnan(msg.alt_error):
                alt_error = "NaN"
            else:
                alt_error = "%s%s" % (self.height_string(msg.alt_error), alt_error_sign)
            self.console.set_status('AltError', 'AltError %s' % alt_error)
            self.console.set_status('AspdError', 'AspdError %s%s' % (self.speed_string(msg.aspd_error*0.01), aspd_error_sign))

        elif type == 'PARAM_VALUE':
            rec, tot = self.module('param').param_status()
            self.console.set_status('Params', 'Param %u/%u' % (rec,tot))

        for id in self.user_added.keys():
            if type in self.user_added[id].msg_types:
                d = self.user_added[id]
                val = mavutil.evaluate_expression(d.expression, self.master.messages)
                try:
                    self.console.set_status(id, d.format % val, row = d.row)
                except Exception as ex:
                    print(ex)
                    pass

def init(mpstate):
    '''initialise module'''
    return ConsoleModule(mpstate)
