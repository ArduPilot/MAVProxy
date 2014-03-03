#!/usr/bin/env python
'''
Antenna tracker control module
This module catches MAVLINK_MSG_ID_GLOBAL_POSITION_INT
and sends them to a MAVlink connected antenna tracker running
ardupilot AntennaTracker
Mike McCauley, based on earlier work by Andrew Tridgell
June 2012
'''

import sys, os, time
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil

mpstate = None

class module_state(object):
    def __init__(self):
        self.connection = None
        self.settings = mp_settings.MPSettings(
            [ ('port', str, "/dev/ttyUSB0"),
              ('baudrate', int, 57600)
              ]
            )

def name():
    '''return module name'''
    return "tracker"

def description():
    '''return module description'''
    return "antenna tracker control module"

def cmd_tracker(args):
    '''set address to contact the tracker'''
    state = mpstate.tracker_state
    if args[0] == "start":
        cmd_tracker_start()
    elif args[0] == "set":
        if len(args) < 3:
            state.settings.show_all()
        else:
            state.settings.set(args[1], args[2])

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.tracker_state = module_state()
    mpstate.command_map['tracker'] = (cmd_tracker, "antenna tracker control module")

def unload():
    '''unload module'''
    pass

def mavlink_packet(m):
    '''handle an incoming mavlink packet. Rlay it to the tracker if it is a GLOBAL_POSITION_INT'''
    if m.get_type() == 'GLOBAL_POSITION_INT':
        if (mpstate.tracker_state.connection == None):
            return
        mpstate.tracker_state.connection.mav.global_position_int_send(m.time_boot_ms, m.lat, m.lon, m.alt, m.relative_alt, m.vx, m.vy, m.vz, m.hdg)

def cmd_tracker_start():
    if mpstate.tracker_state.settings.port == None:
        print("tracker port not set")
        return
    print("connecting to tracker %s at %d" % (mpstate.tracker_state.settings.port, mpstate.tracker_state.settings.baudrate))
    mpstate.tracker_state.connection = mavutil.mavlink_connection(mpstate.tracker_state.settings.port, 
                                                                  autoreconnect=True, 
                                                                  baud=mpstate.tracker_state.settings.baudrate)
