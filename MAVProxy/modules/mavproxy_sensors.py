#!/usr/bin/env python
'''monitor sensor consistancy'''

import time, math
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module


def angle_diff(angle1, angle2):
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

class sensors_report(object):
    def __init__(self):
        self.last_report = 0
        self.ok = True
        self.value = 0

class SensorsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SensorsModule, self).__init__(mpstate, "sensors", "monitor sensor consistancy")
        self.add_command('sensors', self.cmd_sensors, "show key sensors")
        self.add_command('speed', self.cmd_speed, "enable/disable speed report")

        self.last_report = 0
        self.ok = True
        self.value = 0
        self.ground_alt = 0
        self.gps_alt = 0
        self.max_speed = 0
        self.last_watch = 0
        self.reports = {}
        self.reports['heading'] = sensors_report()
        self.reports['altitude'] = sensors_report()
        self.reports['speed'] = sensors_report()

        from MAVProxy.modules.lib.mp_settings import MPSetting
        self.settings.append(MPSetting('speedreporting', bool, False, 'Speed Reporting', tab='Sensors'))

        if 'GPS_RAW' in self.status.msgs:
            # cope with reload
            gps = mpstate.status.msgs['GPS_RAW']
            self.ground_alt = gps.alt - self.status.altitude

        if 'GPS_RAW_INT' in self.status.msgs:
            # cope with reload
            gps = mpstate.status.msgs['GPS_RAW_INT']
            self.ground_alt = (gps.alt / 1.0e3) - self.status.altitude

    def cmd_sensors(self, args):
        '''show key sensors'''
        if self.master.WIRE_PROTOCOL_VERSION == '1.0':
            gps_heading = self.status.msgs['GPS_RAW_INT'].cog * 0.01
        else:
            gps_heading = self.status.msgs['GPS_RAW'].hdg

        self.console.writeln("heading: %u/%u   alt: %u/%u  r/p: %u/%u speed: %u/%u  thr: %u" % (
            self.status.msgs['VFR_HUD'].heading,
            gps_heading,
            self.status.altitude,
            self.gps_alt,
            math.degrees(self.status.msgs['ATTITUDE'].roll),
            math.degrees(self.status.msgs['ATTITUDE'].pitch),
            self.status.msgs['VFR_HUD'].airspeed,
            self.status.msgs['VFR_HUD'].groundspeed,
            self.status.msgs['VFR_HUD'].throttle))


    def cmd_speed(self, args):
        '''enable/disable speed report'''
        self.settings.set('speedreporting', not self.settings.speedreporting)
        if self.settings.speedreporting:
            self.console.writeln("Speed reporting enabled", bg='yellow')
        else:
            self.console.writeln("Speed reporting disabled", bg='yellow')

    def report(self, name, ok, msg=None, deltat=20):
        '''report a sensor error'''
        r = self.reports[name]
        if time.time() < r.last_report + deltat:
            r.ok = ok
            return
        r.last_report = time.time()
        if ok and not r.ok:
            self.say("%s OK" % name)
        r.ok = ok
        if not r.ok:
            self.say(msg)

    def report_change(self, name, value, maxdiff=1, deltat=10):
        '''report a sensor change'''
        r = self.reports[name]
        if time.time() < r.last_report + deltat:
            return
        r.last_report = time.time()
        if math.fabs(r.value - value) < maxdiff:
            return
        r.value = value
        self.say("%s %u" % (name, value))

    def check_heading(self, m):
        '''check heading discrepancy'''
        if 'GPS_RAW' in self.status.msgs:
            gps = self.status.msgs['GPS_RAW']
            if gps.v < 3:
                return
            diff = math.fabs(angle_diff(m.heading, gps.hdg))
        elif 'GPS_RAW_INT' in self.status.msgs:
            gps = self.status.msgs['GPS_RAW_INT']
            if gps.vel < 300:
                return
            diff = math.fabs(angle_diff(m.heading, gps.cog / 100.0))
        else:
            return
        self.report('heading', diff < 20, 'heading error %u' % diff)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'VFR_HUD' and ('GPS_RAW' in self.status.msgs or 'GPS_RAW_INT' in self.status.msgs):
            self.check_heading(m)
            if self.settings.speedreporting:
                if m.airspeed != 0:
                    speed = m.airspeed
                else:
                    speed = m.groundspeed
                self.report_change('speed', speed, maxdiff=2, deltat=2)
        if self.status.watch == "sensors" and time.time() > self.sensors_state.last_watch + 1:
            self.sensors_state.last_watch = time.time()
            self.cmd_sensors([])

def init(mpstate):
    '''initialise module'''
    return SensorsModule(mpstate)
