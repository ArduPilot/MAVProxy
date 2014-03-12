#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil

def name():
    '''return module name'''
    return "mode"

def description():
    '''return module description'''
    return "mode handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.command_map['mode'] = (cmd_mode, "mode change")
    mpstate.command_map['guided'] = (cmd_guided, "fly to a clicked location on map")

def cmd_mode(args):
    '''set arbitrary mode'''
    mode_mapping = mpstate.master().mode_mapping()
    if mode_mapping is None:
        print('No mode mapping available')
        return
    if len(args) != 1:
        print('Available modes: ', mode_mapping.keys())
        return
    mode = args[0].upper()
    if mode not in mode_mapping:
        print('Unknown mode %s: ' % mode)
        return
    mpstate.master().set_mode(mode_mapping[mode])

def unknown_command(args):
    '''handle mode switch by mode name as command'''
    mode_mapping = mpstate.master().mode_mapping()
    mode = args[0].upper()
    if mode in mode_mapping:
        mpstate.master().set_mode(mode_mapping[mode])
        return True
    return False

def cmd_guided(args):
    '''set GUIDED target'''
    if len(args) != 1:
        print("Usage: guided ALTITUDE")
        return
    try:
        latlon = mpstate.map_state.click_position
    except Exception:
        print("No map available")
        return
    if latlon is None:
        print("No map click position available")
        return        
    altitude = int(args[0])
    print("Guided %s %d" % (str(latlon), altitude))
    mpstate.master().mav.mission_item_send(mpstate.status.target_system,
                                           mpstate.status.target_component,
                                           0,
                                           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                           2, 0, 0, 0, 0, 0,
                                           latlon[0], latlon[1], altitude)
    
