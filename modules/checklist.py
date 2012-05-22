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

    master = mpstate.master()

    mpstate.checklist.set_status("Compass Offsets", 1)

    # add some status fields
    if type in ['HEARTBEAT']:
        '''print "Got packet"'''
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
    elif type == 'ATTITUDE':
        '''print "Got other packet"'''
        if mpstate.status.avionics_battery_level > 40:
            mpstate.checklist.set_status("Avionics Battery", 1)
        else:
            mpstate.checklist.set_status("Avionics Battery", 0)




