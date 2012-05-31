"""
  MAVProxy checklists

  uses lib/libchecklist.py for UI
"""

import os, sys, math

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import libchecklist

def name():
    '''return module name'''
    return "checklist"

def description():
    '''return module description'''
    return "Checklists for the APM"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.checklist = libchecklist.UI()
        

def unload():
    '''unload module'''
    mpstate.checklist = None
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    if not isinstance(mpstate.checklist, libchecklist.UI):
        return
    if not mpstate.checklist.is_alive():
        mpstate.libchecklist = None
        return

    type = msg.get_type()

    

    '''beforeEngineList - APM booted'''
    if mpstate.status.heartbeat_error = True:
        mpstate.checklist.set_status("APM Booted", 0)
    else:
        mpstate.checklist.set_status("APM Booted", 1)

    '''beforeEngineList - Altitude lock'''
    if mpstate.status.altitude > 0 and mpstate.status.altitude < 5000:
        mpstate.checklist.set_status("Altitude lock", 1)
    else:
        mpstate.checklist.set_status("Altitude lock", 0)

    '''beforeEngineList - Flight mode MANUAL'''
    if mpstate.status.flightmode == "MANUAL":
        mpstate.checklist.set_status("Flight mode MANUAL", 1)
    else:
        mpstate.checklist.set_status("Flight mode MANUAL", 0)

    '''beforeEngineList - Trim set from controller'''
    if int(mpstate.mav_param['RC1_TRIM']) == 0:
        mpstate.checklist.set_status("Trim set from controller", 0)
    elif int(mpstate.mav_param['RC2_TRIM']) == 0:
        mpstate.checklist.set_status("Trim set from controller", 0)
    elif int(mpstate.mav_param['RC3_TRIM']) == 0:
        mpstate.checklist.set_status("Trim set from controller", 0)
    elif int(mpstate.mav_param['RC4_TRIM']) == 0:
        mpstate.checklist.set_status("Trim set from controller", 0)
    else:
        mpstate.checklist.set_status("Trim set from controller", 1)

    '''beforeEngineList - GPS lock'''
    if (not 'GPS_RAW_INT' in mpstate.status.msgs) or mpstate.status.msgs['GPS_RAW'].fix_type != 2:
        mpstate.checklist.set_status("GPS lock", 0)
    else:
        mpstate.checklist.set_status("GPS lock", 1)

    '''beforeEngineList - Radio Links > 6db margin TODO: figure out how to read db levels'''
    for master in mpstate.mav_master:
        mpstate.checklist.set_status("Radio Links > 6db margin", 1)
        if master.linkerror or master.link_delayed:
            mpstate.checklist.set_status("Radio Links > 6db margin", 0)

    '''beforeEngineList - Avionics Battery'''
    if mpstate.status.avionics_battery_level > 40:
        mpstate.checklist.set_status("Avionics Battery", 1)
    else:
        mpstate.checklist.set_status("Avionics Battery", 0)

    '''beforeEngineList - Compass Offsets'''
    if int(mpstate.mav_param['SET_MAG_OFFSETS']) == 0:
        mpstate.checklist.set_status("Compass Offsets", 0)
    else:
        mpstate.checklist.set_status("Compass Offsets", 1)

    '''beforeEngineList - Accelerometers Calibrated'''
    '''if (something):
        mpstate.checklist.set_status("Compass Offsets", 0)
    else:
        mpstate.checklist.set_status("Compass Offsets", 1)'''

    '''beforeEngineList - Gyros Calibrated'''
    '''if (something):
        mpstate.checklist.set_status("Gyros Offsets", 0)
    else:
        mpstate.checklist.set_status("Gyros Offsets", 1)'''

    '''beforeEngineList - Aircraft Params Loaded'''
    '''if (something):
        mpstate.checklist.set_status("Aircraft Params Loaded", 0)
    else:
        mpstate.checklist.set_status("Aircraft Params Loaded", 1)'''

    '''beforeEngineList - Waypoints Loaded'''
    if (mpstate.status.wploader.count() == 0:
        mpstate.checklist.set_status("Waypoints Loaded", 0)
    else:
        mpstate.checklist.set_status("Waypoints Loaded", 1)






