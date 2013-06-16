#!/usr/bin/env python
'''monitor sensor consistancy'''

import time, math
from pymavlink import mavutil

mpstate = None

class sensors_report(object):
    def __init__(self):
        self.last_report = 0
        self.ok = True
        self.value = 0

class sensors_state(object):
    def __init__(self):
        self.ground_alt = 0
        self.gps_alt = 0
        self.max_speed = 0
        self.last_watch = 0
        self.speed_report = False
        self.reports = {}
        self.reports['heading'] = sensors_report()
        self.reports['altitude'] = sensors_report()
        self.reports['speed'] = sensors_report()

def name():
    '''return module name'''
    return "sensors"

def description():
    '''return module description'''
    return "monitor sensor consistancy"

def cmd_sensors(args):
    '''show key sensors'''
    if mpstate.master().WIRE_PROTOCOL_VERSION == '1.0':
        gps_heading = mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01
    else:
        gps_heading = mpstate.status.msgs['GPS_RAW'].hdg
        
    mpstate.console.writeln("heading: %u/%u   alt: %u/%u  r/p: %u/%u speed: %u/%u  thr: %u" % (
        mpstate.status.msgs['VFR_HUD'].heading,
        gps_heading,
        mpstate.status.altitude,
        mpstate.sensors_state.gps_alt,
        math.degrees(mpstate.status.msgs['ATTITUDE'].roll),
        math.degrees(mpstate.status.msgs['ATTITUDE'].pitch),
        mpstate.status.msgs['VFR_HUD'].airspeed,
        mpstate.status.msgs['VFR_HUD'].groundspeed,
        mpstate.status.msgs['VFR_HUD'].throttle))
                     

def cmd_speed(args):
    '''enable/disable speed report'''
    mpstate.sensors_state.speed_report = not mpstate.sensors_state.speed_report
    if mpstate.sensors_state.speed_report:
        mpstate.console.writeln("Speed reporting enabled", bg='yellow')
    else:
        mpstate.console.writeln("Speed reporting disabled", bg='yellow')

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.sensors_state = sensors_state()
    mpstate.command_map['sensors'] = (cmd_sensors, "show key sensors")
    mpstate.command_map['speed'] = (cmd_speed, "enable/disable speed report")

    if 'GPS_RAW' in mpstate.status.msgs:
        # cope with reload
        gps = mpstate.status.msgs['GPS_RAW']
        mpstate.sensors_state.ground_alt = gps.alt - mpstate.status.altitude

    if 'GPS_RAW_INT' in mpstate.status.msgs:
        # cope with reload
        gps = mpstate.status.msgs['GPS_RAW_INT']
        mpstate.sensors_state.ground_alt = (gps.alt/1.0e3) - mpstate.status.altitude

def angle_diff(angle1, angle2):
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

def report(name, ok, msg=None, deltat=20):
    '''report a sensor error'''
    r = mpstate.sensors_state.reports[name]
    if time.time() < r.last_report + deltat:
        r.ok = ok
        return
    r.last_report = time.time()
    if ok and not r.ok:
        mpstate.functions.say("%s OK" % name)
    r.ok = ok
    if not r.ok:
        mpstate.functions.say(msg)

def report_change(name, value, maxdiff=1, deltat=10):
    '''report a sensor change'''
    r = mpstate.sensors_state.reports[name]
    if time.time() < r.last_report + deltat:
        return
    r.last_report = time.time()
    if math.fabs(r.value - value) < maxdiff:
        return
    r.value = value
    mpstate.functions.say("%s %u" % (name, value))

def check_heading(m):
    '''check heading discrepancy'''
    if 'GPS_RAW' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW']
        if gps.v < 3:
            return
        diff = math.fabs(angle_diff(m.heading, gps.hdg))
    elif 'GPS_RAW_INT' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW_INT']
        if gps.vel < 300:
            return
        diff = math.fabs(angle_diff(m.heading, gps.cog/100.0))
    else:
        return
    report('heading', diff<20, 'heading error %u' % diff)

def check_altitude(m):
    '''check altitude discrepancy'''
    if 'GPS_RAW' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW']
        if gps.fix_type != 2:
            return
        v = gps.v
        alt = gps.alt
    elif 'GPS_RAW_INT' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW_INT']
        if gps.fix_type != 3:
            return
        v = gps.vel/100
        alt = gps.alt/1000
    else:
        return

    if v > mpstate.sensors_state.max_speed:
        mpstate.sensors_state.max_speed = v
    if mpstate.sensors_state.max_speed < 5:
        mpstate.sensors_state.ground_alt = alt
        return
    mpstate.sensors_state.gps_alt = alt - mpstate.sensors_state.ground_alt
    diff = math.fabs(mpstate.sensors_state.gps_alt - mpstate.status.altitude)
    report('altitude', diff<30, 'altitude error %u' % diff)

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    if m.get_type() == 'VFR_HUD' and ( 'GPS_RAW' in mpstate.status.msgs or 'GPS_RAW_INT' in mpstate.status.msgs ):
        check_heading(m)
        check_altitude(m)
        if mpstate.sensors_state.speed_report:
            if m.airspeed != 0:
                speed = m.airspeed
            else:
                speed = m.groundspeed
            report_change('speed', speed, maxdiff=2, deltat=2)
    if mpstate.status.watch == "sensors" and time.time() > mpstate.sensors_state.last_watch+1:
        mpstate.sensors_state.last_watch = time.time()
        cmd_sensors([])
