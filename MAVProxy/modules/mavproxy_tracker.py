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
    elif args[0] == 'arm':
        cmd_tracker_arm()
    elif args[0] == 'disarm':
        cmd_tracker_disarm()
    elif args[0] == 'level':
        cmd_tracker_level()
    else:
        print("usage: tracker <start|set|arm|disarm|level>")

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.tracker_state = module_state()
    mpstate.command_map['tracker'] = (cmd_tracker, "antenna tracker control module")

def unload():
    '''unload module'''
    pass

def tracker_callback(m, connection):
    '''process mavlink message m from tracker'''
    connection.post_message(m)

def tracker_send_callback(m, master):
    '''called on sending a message to the tracker'''

def mavlink_packet(m):
    '''handle an incoming mavlink packet from the master vehicle. Relay it to the tracker if it is a GLOBAL_POSITION_INT'''
    if m.get_type() == 'GLOBAL_POSITION_INT':
        if (mpstate.tracker_state.connection == None):
            return
        mpstate.tracker_state.connection.mav.global_position_int_send(m.time_boot_ms, m.lat, m.lon, m.alt, m.relative_alt, m.vx, m.vy, m.vz, m.hdg)

def cmd_tracker_start():
    if mpstate.tracker_state.settings.port == None:
        print("tracker port not set")
        return
    print("connecting to tracker %s at %d" % (mpstate.tracker_state.settings.port, mpstate.tracker_state.settings.baudrate))
    m = mavutil.mavlink_connection(mpstate.tracker_state.settings.port, 
                                   autoreconnect=True, 
                                   baud=mpstate.tracker_state.settings.baudrate)
    m.mav.set_callback(tracker_callback, m)
    if hasattr(m.mav, 'set_send_callback'):
        m.mav.set_send_callback(tracker_send_callback, m)
    m.linknum = len(mpstate.mav_master)
    m.linkerror = False
    m.link_delayed = False
    m.last_heartbeat = 0
    m.last_message = 0
    m.highest_msec = 0
    mpstate.mav_master.append(m)
    mpstate.tracker_state.connection = m

def cmd_tracker_arm():
    '''Enable the servos in the tracker so the antenna will move'''
    if (mpstate.tracker_state.connection):
        mpstate.tracker_state.connection.arducopter_arm()

def cmd_tracker_disarm():
    '''Disable the servos in the tracker so the antenna will not move'''
    if (mpstate.tracker_state.connection):
        mpstate.tracker_state.connection.arducopter_disarm()

def cmd_tracker_level():
    '''Calibrate the accelerometers. Disarm and move the antenna level first'''
    if (mpstate.tracker_state.connection):
        mpstate.tracker_state.connection.calibrate_level()

