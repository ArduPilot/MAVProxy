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
from MAVProxy.modules import mavproxy_map
from pymavlink import mavutil

mpstate = None

class tracker_state(object):
    def __init__(self):
        self.connection = None
        self.settings = mp_settings.MPSettings(
            [ ('port', str, "/dev/ttyUSB0"),
              ('baudrate', int, 57600),
              ('debug', int, 0)
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
    mpstate.tracker_state = tracker_state()
    mpstate.command_map['tracker'] = (cmd_tracker, "antenna tracker control module")

def unload():
    '''unload module'''
    pass

def mavlink_packet(m):
    '''handle an incoming mavlink packet from the master vehicle. Relay it to the tracker
    if it is a GLOBAL_POSITION_INT'''
    state = mpstate.tracker_state
    if not state.connection:
        return
    if m.get_type() in ['GLOBAL_POSITION_INT', 'SCALED_PRESSURE']:
        state.connection.mav.send(m)

def idle_task():
    '''called in idle time'''
    state = mpstate.tracker_state
    if not state.connection:
        return

    # check for a mavlink message from the tracker
    m = state.connection.recv_msg()
    if m is None:
        return

    if state.settings.debug:
        print m

    if m.get_type() == 'GLOBAL_POSITION_INT':
        (state.lat, state.lon, state.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
        if state.lat != 0 or state.lon != 0:
            mavproxy_map.create_vehicle_icon('AntennaTracker', 'red', follow=False, vehicle_type='antenna')
            mpstate.map.set_position('AntennaTracker', (state.lat, state.lon), rotation=state.heading)
    

def cmd_tracker_start():
    state = mpstate.tracker_state
    if state.settings.port == None:
        print("tracker port not set")
        return
    print("connecting to tracker %s at %d" % (state.settings.port, state.settings.baudrate))
    m = mavutil.mavlink_connection(state.settings.port, 
                                   autoreconnect=True, 
                                   baud=state.settings.baudrate)
    state.connection = m

def cmd_tracker_arm():
    '''Enable the servos in the tracker so the antenna will move'''
    state = mpstate.tracker_state
    if not state.connection:
        print("tracker not connected")
        return
    state.connection.arducopter_arm()

def cmd_tracker_disarm():
    '''Disable the servos in the tracker so the antenna will not move'''
    state = mpstate.tracker_state
    if not state.connection:
        print("tracker not connected")
        return
    state.connection.arducopter_disarm()

def cmd_tracker_level():
    '''Calibrate the accelerometers. Disarm and move the antenna level first'''
    state = mpstate.tracker_state
    if not state.connection:
        print("tracker not connected")
        return
    state.connection.calibrate_level()
