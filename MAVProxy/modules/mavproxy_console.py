"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys, math, time

from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.mp_menu import *

class ConsoleModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ConsoleModule, self).__init__(mpstate, "console", "GUI console", public=True)
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0
        mpstate.console = wxconsole.MessageConsole(title='Console')

        # setup some default status information
        mpstate.console.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
        mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
        mpstate.console.set_status('Vcc', 'Vcc: --', fg='red', row=0)
        mpstate.console.set_status('Radio', 'Radio: --', row=0)
        mpstate.console.set_status('INS', 'INS', fg='grey', row=0)
        mpstate.console.set_status('MAG', 'MAG', fg='grey', row=0)
        mpstate.console.set_status('AS', 'AS', fg='grey', row=0)
        mpstate.console.set_status('AHRS', 'AHRS', fg='grey', row=0)
        mpstate.console.set_status('Battery', 'Batt --', row=1)
        mpstate.console.set_status('Heading', 'Hdg ---/---', row=2)
        mpstate.console.set_status('Alt', 'Alt ---', row=2)
        mpstate.console.set_status('AGL', 'AGL ---', row=2)
        mpstate.console.set_status('AirSpeed', 'AirSpeed --', row=2)
        mpstate.console.set_status('GPSSpeed', 'GPSSpeed --', row=2)
        mpstate.console.set_status('Thr', 'Thr ---', row=2)
        mpstate.console.set_status('Roll', 'Roll ---', row=2)
        mpstate.console.set_status('Pitch', 'Pitch ---', row=2)
        mpstate.console.set_status('WP', 'WP --', row=3)
        mpstate.console.set_status('WPDist', 'Distance ---', row=3)
        mpstate.console.set_status('WPBearing', 'Bearing ---', row=3)
        mpstate.console.set_status('AltError', 'AltError --', row=3)
        mpstate.console.set_status('AspdError', 'AspdError --', row=3)
        mpstate.console.set_status('FlightTime', 'FlightTime --', row=3)
        mpstate.console.set_status('ETR', 'ETR --', row=3)

        mpstate.console.ElevationMap = mp_elevation.ElevationModel()

        # create the main menu
        self.menu = MPMenuTop([])
        self.add_menu(MPMenuSubMenu('MAVProxy',
                                    items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                           MPMenuItem('Map', 'Load Map', '# module load map')]))

    def add_menu(self, menu):
        '''add a new menu'''
        self.menu.add(menu)
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

    def vcell_to_battery_percent(self, vcell):
        '''convert a cell voltage to a percentage battery level'''
        if vcell > 4.1:
            # above 4.1 is 100% battery
            return 100.0
        elif vcell > 3.81:
            # 3.81 is 17% remaining, from flight logs
            return 17.0 + 83.0 * (vcell - 3.81) / (4.1 - 3.81)
        elif vcell > 3.2:
            # below 3.2 it degrades fast. It's dead at 3.2
            return 0.0 + 17.0 * (vcell - 3.20) / (3.81 - 3.20)
        # it's dead or disconnected
        return 0.0

    def battery_update(self, SYS_STATUS):
        '''update battery level'''

        # main flight battery
        self.mpstate.status.battery_level = SYS_STATUS.battery_remaining
        self.mpstate.status.voltage_level = SYS_STATUS.voltage_battery
        self.mpstate.status.current_battery = SYS_STATUS.current_battery

        # avionics battery
        if not 'AP_ADC' in self.mpstate.status.msgs:
            return
            rawvalue = float(self.mpstate.status.msgs['AP_ADC'].adc2)
            INPUT_VOLTAGE = 4.68
            VOLT_DIV_RATIO = 3.56
            voltage = rawvalue*(INPUT_VOLTAGE/1024.0)*VOLT_DIV_RATIO
            vcell = voltage / self.mpstate.settings.numcells

            avionics_battery_level = vcell_to_battery_percent(vcell)

            if self.mpstate.status.avionics_battery_level == -1 or abs(avionics_battery_level-self.mpstate.status.avionics_battery_level) > 70:
                self.mpstate.status.avionics_battery_level = avionics_battery_level
            else:
                self.mpstate.status.avionics_battery_level = (95*self.mpstate.status.avionics_battery_level + 5*avionics_battery_level)/100

    def battery_report(self):
        batt_mon = self.get_mav_param('BATT_MONITOR',0)

        #report voltage level only 
        if batt_mon == 3:
            self.mpstate.console.set_status('Battery', 'Batt: %.2fV' % (float(self.mpstate.status.voltage_level) / 1000.0), row=1)
        elif batt_mon == 4:
            self.mpstate.console.set_status('Battery', 'Batt: %u%%/%.2fV %.1fA' % (self.mpstate.status.battery_level, (float(self.mpstate.status.voltage_level) / 1000.0), self.mpstate.status.current_battery / 100.0 ), row=1)

            rbattery_level = int((self.mpstate.status.battery_level+5)/10)*10

            if self.mpstate.settings.battwarn > 0 and time.time() > self.mpstate.status.last_battery_announce_time + 60*self.mpstate.settings.battwarn:
                self.mpstate.status.last_battery_announce_time = time.time()
                if rbattery_level != self.mpstate.status.last_battery_announce:
                    self.say("Flight battery %u percent" % rbattery_level, priority='notification')
                    self.mpstate.status.last_battery_announce = rbattery_level
                if rbattery_level <= 20:
                    self.say("Flight battery warning")

        else:
            #clear battery status
            self.mpstate.console.set_status('Battery')

        # avionics battery reporting disabled for now
        return
        avionics_rbattery_level = int((self.mpstate.status.avionics_battery_level+5)/10)*10

        if avionics_rbattery_level != self.mpstate.status.last_avionics_battery_announce:
            self.say("Avionics Battery %u percent" % avionics_rbattery_level, priority='notification')
            self.mpstate.status.last_avionics_battery_announce = avionics_rbattery_level
        if avionics_rbattery_level <= 20:
            self.say("Avionics battery warning")

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if not isinstance(self.console, wxconsole.MessageConsole):
            return
        if not self.console.is_alive():
            self.mpstate.console = textconsole.SimpleConsole()
            return
        type = msg.get_type()

        master = self.master
        # add some status fields
        if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
            if type == "GPS_RAW":
                num_sats1 = master.field('GPS_STATUS', 'satellites_visible', 0)
            else:
                num_sats1 = msg.satellites_visible
            num_sats2 = master.field('GPS2_RAW', 'satellites_visible', -1)
            if num_sats2 == -1:
                sats_string = "%u" % num_sats1
            else:
                sats_string = "%u/%u" % (num_sats1, num_sats2)
            if ((msg.fix_type == 3 and master.mavlink10()) or
                (msg.fix_type == 2 and not master.mavlink10())):
                self.console.set_status('GPS', 'GPS: OK (%s)' % sats_string, fg='green')
            else:
                self.console.set_status('GPS', 'GPS: %u (%s)' % (msg.fix_type, sats_string), fg='red')
            if master.mavlink10():
                gps_heading = int(self.mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01)
            else:
                gps_heading = self.mpstate.status.msgs['GPS_RAW'].hdg
            self.console.set_status('Heading', 'Hdg %s/%u' % (master.field('VFR_HUD', 'heading', '-'), gps_heading))
        elif type == 'VFR_HUD':
            if master.mavlink10():
                alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
            else:
                alt = master.field('GPS_RAW', 'alt', 0)
            if self.module('wp').wploader.count() > 0:
                wp = self.module('wp').wploader.wp(0)
                home_lat = wp.x
                home_lng = wp.y
            else:
                home_lat = master.field('HOME', 'lat') * 1.0e-7
                home_lng = master.field('HOME', 'lon') * 1.0e-7
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
            lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
            rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
            if self.settings.basealt != 0:
                agl_alt = self.settings.basealt - self.console.ElevationMap.GetElevation(lat, lng)
            else:
                agl_alt = self.console.ElevationMap.GetElevation(home_lat, home_lng) - self.console.ElevationMap.GetElevation(lat, lng)
            agl_alt += rel_alt
            self.console.set_status('AGL', 'AGL %u' % agl_alt)
            self.console.set_status('Alt', 'Alt %u' % rel_alt)
            self.console.set_status('AirSpeed', 'AirSpeed %u' % msg.airspeed)
            self.console.set_status('GPSSpeed', 'GPSSpeed %u' % msg.groundspeed)
            self.console.set_status('Thr', 'Thr %u' % msg.throttle)
            t = time.localtime(msg._timestamp)
            if msg.groundspeed > 3 and not self.in_air:
                self.in_air = True
                self.start_time = time.mktime(t)
            elif msg.groundspeed > 3 and self.in_air:
                self.total_time = time.mktime(t) - self.start_time
                self.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
            elif msg.groundspeed < 3 and self.in_air:
                self.in_air = False
                self.total_time = time.mktime(t) - self.start_time
                self.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
        elif type == 'ATTITUDE':
            self.console.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
            self.console.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))
        elif type in ['SYS_STATUS']:
            sensors = { 'AS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                        'MAG' : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'INS' : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS}
            for s in sensors.keys():
                bits = sensors[s]
                present = ((msg.onboard_control_sensors_enabled & bits) == bits)
                healthy = ((msg.onboard_control_sensors_health & bits) == bits)
                if not present:
                    fg = 'grey'
                elif not healthy:
                    fg = 'red'
                else:
                    fg = 'green'
                self.console.set_status(s, s, fg=fg)

            self.battery_update(msg)
            self.battery_report()

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
        elif type in ['RADIO', 'RADIO_STATUS']:
            if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
                fg = 'red'
            else:
                fg = 'black'
            self.console.set_status('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)
        elif type == 'HEARTBEAT':
            self.console.set_status('Mode', '%s' % master.flightmode, fg='blue')
            for m in self.mpstate.mav_master:
                linkdelay = (self.mpstate.status.highest_msec - m.highest_msec)*1.0e-3
                linkline = "Link %u " % (m.linknum+1)
                if m.linkerror:
                    linkline += "down"
                    fg = 'red'
                else:
                    linkline += "OK (%u pkts, %.2fs delay, %u lost)" % (m.mav_count, linkdelay, m.mav_loss)
                    if linkdelay > 1:
                        fg = 'orange'
                    else:
                        fg = 'darkgreen'
                self.console.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
        elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
            self.console.set_status('WP', 'WP %u' % msg.seq)
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
            self.console.set_status('WPDist', 'Distance %u' % msg.wp_dist)
            self.console.set_status('WPBearing', 'Bearing %u' % msg.target_bearing)
            if msg.alt_error > 0:
                alt_error_sign = "L"
            else:
                alt_error_sign = "H"
            if msg.aspd_error > 0:
                aspd_error_sign = "L"
            else:
                aspd_error_sign = "H"
            self.console.set_status('AltError', 'AltError %d%s' % (msg.alt_error, alt_error_sign))
            self.console.set_status('AspdError', 'AspdError %.1f%s' % (msg.aspd_error*0.01, aspd_error_sign))

def init(mpstate):
    '''initialise module'''
    return ConsoleModule(mpstate)
