#!/usr/bin/env python
'''
CUAV mission control
Andrew Tridgell
'''

import sys, os, time
from cuav.lib import cuav_util
from pymavlink import mavutil

mpstate = None

class module_state(object):
    def __init__(self):
        self.stage = "PREFLIGHT"
        self.check_time = time.time()
        self.stage = None
        mpstate.console.set_status('Stage', 'Stage: --', row=8, fg='red')
        mpstate.console.set_status('Bottle', 'Bottle: --', row=8, fg='green')
        mpstate.console.set_status('BottleConfirm', 'Con: --', row=8, fg='green')

def name():
    '''return module name'''
    return "cuav"

def description():
    '''return module description'''
    return "cuav mission module"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.cuav_state = module_state()

def unload():
    '''unload module'''
    pass

def check_parms(parms, set=False):
    '''check parameter settings'''
    state = mpstate.cuav_state
    master = mpstate.master()
    for p in parms.keys():
        v = mpstate.mav_param.get(p, None)
        if v is None:
            continue
        if abs(v - parms[p]) > 0.0001:
            if set:
                mpstate.console.writeln('Setting %s to %.1f (currently %.1f)' % (p, parms[p], v), fg='blue')
            else:
                mpstate.console.writeln('%s should be %.1f (currently %.1f)' % (p, parms[p], v), fg='blue')
            if set:
                master.param_set_send(p, parms[p])

def check_preflight():
    '''check preflight parameters'''
    state = mpstate.cuav_state
    master = mpstate.master()
    # check relative alt hasn't drifted
    if abs(master.field('GLOBAL_POSITION_INT', 'relative_alt', 0))*0.001 >= 5:
        mpstate.console.writeln('Baro cal needed', fg='blue')
    if master.field('MISSION_CURRENT', 'seq', 0) != 0:
        mpstate.console.writeln('Mission should be reset', fg='blue')
    if mpstate.status.fenceloader.count() < 5:
        mpstate.console.writeln('fence needs to be loaded', fg='blue')        
    parms = { 
              'TRIM_THROTTLE' : 40,
              'RC3_MIN'       : 1080.0  }
    check_parms(parms)
    parms = {
        'STICK_MIXING'  : 1.0,
        'THR_MIN'       : 5.0,
        'COMPASS_USE'   : 1.0,
        'KFF_RDDRMIX'   : 0.0,
        'RC7_FUNCTION'  : 0.0,
        'WP_RADIUS'     : 70,
        'ALT_OFFSET'    : 0.0,
        "SR0_EXTRA1"    : 5.0,
        "SR0_EXTRA2"    : 3.0,
        "SR0_EXTRA3"    : 2.0,
        "SR0_EXT_STAT"  : 4.0,
        "SR0_PARAMS"    : 10.0,
        "SR0_POSITION"  : 4.0,
        "SR0_RAW_CTRL"  : 2.0,
        "SR0_RAW_SENS"  : 2.0,
        "SR0_RC_CHAN"   : 2.0,
        "SR3_EXTRA1"    : 1.0,
        "SR3_EXTRA2"    : 1.0,
        "SR3_EXTRA3"    : 1.0,
        "SR3_EXT_STAT"  : 3.0,
        "SR3_PARAMS"    : 10.0,
        "SR3_POSITION"  : 3.0,
        "SR3_RAW_CTRL"  : 1.0,
        "SR3_RAW_SENS"  : 1.0,
        "SR3_RC_CHAN"   : 1.0 }
    check_parms(parms, set=True)
              

def check_joe_approach():
    '''check joe approach'''
    parms = { 'WP_RADIUS'     : 30 }
    check_parms(parms, set=True)

def check_land_approach():
    '''check land approach'''
    parms = { 'STICK_MIXING'  : 1.0,
              'THR_MIN'       : 5.0,
              'KFF_RDDRMIX'   : 0.0 }
    check_parms(parms, set=True)

def check_landing():
    '''check landing'''
    parms = { 'STICK_MIXING'  : 1.0,
              'THR_MIN'       : 0.0 }
    check_parms(parms, set=True)

def check_landed():
    '''check landed'''
    parms = { 'THR_MIN'  : 0.0,
              'RC3_MIN'  : 1020}
    check_parms(parms, set=True)

def check_search():
    '''check search'''
    state = mpstate.cuav_state
    master = mpstate.master()
    parms = { 'WP_RADIUS'     : 70,
              'STICK_MIXING'  : 0,
              'KFF_RDDRMIX'   : 0.1,
              'THR_MIN'       : 5.0 }
    check_parms(parms, set=True)

def check_takeoff():
    '''check takeoff'''
    state = mpstate.cuav_state
    if mpstate.settings.mavfwd != 0:
        mpstate.console.writeln('mavfwd should be zero', fg='blue')
        

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.cuav_state
    if m.get_type() == "SERVO_OUTPUT_RAW":
        bottle = m.servo7_raw
        if bottle == 1000:
            mpstate.console.set_status('Bottle', 'Bottle: HELD', row=8, fg='green')
        elif bottle == 1430:
            mpstate.console.set_status('Bottle', 'Bottle: DROP', row=8, fg='red')
        else:
            mpstate.console.set_status('Bottle', 'Bottle: %u' % bottle, row=8, fg='red')
    if m.get_type() == "SYS_STATUS":
        voltage = m.voltage_battery * 0.001
        if voltage < 1.0:
            mpstate.console.set_status('BottleConfirm', 'Con: DROP', row=8, fg='red')
        elif voltage > 3.0 and voltage < 5.5:
            mpstate.console.set_status('BottleConfirm', 'Con: HELD', row=8, fg='green')
        else:
            mpstate.console.set_status('BottleConfirm', 'Con: %.1f' % voltage, row=8, fg='blue')
    if m.get_type() == "WIND":
        mpstate.console.set_status('Wind', 'Wind: %.1f/%u' % (m.speed,int(m.direction+0.5)), row=8)
    if time.time() - state.check_time > 2:
        state.check_time = time.time()
        master = mpstate.master()
        if mpstate.status.flightmode == "MANUAL" and master.field('VFR_HUD', 'groundspeed', 0) < 2:
            # we're in preflight
            state.stage = "PREFLIGHT"
            check_preflight()
        if mpstate.status.flightmode == "AUTO":
            height = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0)*0.001
            groundspeed = master.field('VFR_HUD', 'groundspeed', 0)
            fix_type = master.field('GPS_RAW_INT', 'fix_type', 0)
            if fix_type != 3:
                check_parms({ "COMPASS_USE":1}, set=True)
                
            # we're in auto flight, see what target we have
            wpnum = master.field('MISSION_CURRENT', 'seq', 0)
            if wpnum > 0 and mpstate.status.wploader.count() > wpnum + 3:
                wp0 = mpstate.status.wploader.wp(wpnum)
                wp1 = mpstate.status.wploader.wp(wpnum+1)
                wp2 = mpstate.status.wploader.wp(wpnum+2)
                wp3 = mpstate.status.wploader.wp(wpnum+3)
                if wp1.command == MAV_CMD_DO_SET_SERVO or wp2.command == MAV_CMD_DO_SET_SERVO:
                    state.stage = "JOE_APPROACH"
                    check_joe_approach()
                elif wp1.command == MAV_CMD_DO_CHANGE_SPEED and wp2.command == MAV_CMD_DO_CHANGE_SPEED and wp3.command == MAV_CMD_NAV_LAND:
                    state.stage = "LAND_APPROACH"
                    check_land_approach()
                elif wp0.command == MAV_CMD_NAV_LAND and groundspeed < 14 and height < 20:
                    state.stage = "LANDED"
                    check_landed()
                elif wp0.command == MAV_CMD_NAV_LAND:
                    state.stage = "LANDING"
                    check_landing()
                elif wp0.command == MAV_CMD_NAV_TAKEOFF:
                    state.stage = "TAKEOFF"
                    check_takeoff()
                elif groundspeed > 20 and height > 50:
                    state.stage = "SEARCH"
                    check_search()
        mpstate.console.set_status('Stage', 'Stage: %s' % state.stage, row=8, fg='blue')
