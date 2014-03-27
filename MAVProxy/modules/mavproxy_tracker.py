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

from MAVProxy.modules.lib import mp_module

# this should be in mavutil.py
mode_mapping_antenna = {
    'MANUAL' : 0, 
    'AUTO' : 10,
    'INITIALISING' : 16
    }

class TrackerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TrackerModule, self).__init__(mpstate, "tracker", "antenna tracker control module")
        self.connection = None
        self.settings = mp_settings.MPSettings(
            [ ('port', str, "/dev/ttyUSB0"),
              ('baudrate', int, 57600),
              ('debug', int, 0)
              ]
            )
        self.add_command('tracker', self.cmd_tracker,
                         "antenna tracker control module",
                         ['<start|arm|disarm|level|param|mode|position>',
                          'set (TRACKERSETTING)'])
        self.add_completion_function('(TRACKERSETTING)', self.settings.completion)

    def cmd_tracker_position(args):
        '''tracker manual positioning commands'''
        state = mpstate.tracker_state
        if not state.connection:
            print("tracker not connected")
            return
        positions = [0, 0, 0, 0, 0] # x, y, z, r, buttons. only position[0] (yaw) and position[1] (pitch) are currently used
        for i in range(0, 4):
            if len(args) > i:
                positions[i] = int(args[i]) # default values are 0
        state.connection.mav.manual_control_send(state.connection.target_system,
                                                 positions[0], positions[1],
                                                 positions[2], positions[3],
                                                 positions[4])
    
    def cmd_tracker(args):
        '''tracker command parser'''
        if args[0] == "start":
            cmd_tracker_start()
        elif args[0] == "set":
            state.settings.command(args[1:])
        elif args[0] == 'arm':
            cmd_tracker_arm()
        elif args[0] == 'disarm':
            cmd_tracker_disarm()
        elif args[0] == 'level':
            cmd_tracker_level()
        elif args[0] == 'param':
            cmd_tracker_param(args[1:])
        elif args[0] == 'mode':
            cmd_tracker_mode(args[1:])
        elif args[0] == 'position':
            cmd_tracker_position(args[1:])
        else:
            print("usage: tracker <start|set|arm|disarm|level|param set NAME VALUE|mode MODE>")

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet from the master vehicle. Relay it to the tracker
        if it is a GLOBAL_POSITION_INT'''
        if not self.connection:
            return
        if m.get_type() in ['GLOBAL_POSITION_INT', 'SCALED_PRESSURE']:
            self.connection.mav.send(m)
    
    def idle_task(self):
        '''called in idle time'''
        if not self.connection:
            return
    
        # check for a mavlink message from the tracker
        m = self.connection.recv_msg()
        if m is None:
            return
    
        if self.settings.debug:
            print m
    
        if m.get_type() == 'GLOBAL_POSITION_INT':
            (self.lat, self.lon, self.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
            if self.lat != 0 or self.lon != 0:
                mavproxy_map.create_vehicle_icon('AntennaTracker', 'red', follow=False, vehicle_type='antenna')
                self.mpstate.map.set_position('AntennaTracker', (self.lat, self.lon), rotation=self.heading)
        
    
    def cmd_tracker_start(self):
        if self.settings.port == None:
            print("tracker port not set")
            return
        print("connecting to tracker %s at %d" % (self.settings.port, self.settings.baudrate))
        m = mavutil.mavlink_connection(self.settings.port, 
                                       autoreconnect=True, 
                                       baud=self.settings.baudrate)
        self.connection = m
    
    def cmd_tracker_arm(self):
        '''Enable the servos in the tracker so the antenna will move'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.arducopter_arm()
    
    def cmd_tracker_disarm(self):
        '''Disable the servos in the tracker so the antenna will not move'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.arducopter_disarm()
    
    def cmd_tracker_level(self):
        '''Calibrate the accelerometers. Disarm and move the antenna level first'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.calibrate_level()

    def cmd_tracker_param(args):
        '''Parameter commands'''
        if args[0] == "set" and len(args) > 2:
            cmd_tracker_param_set(args[1], args[2])
        else:
            print("usage: tracker param set PARAMNAME VALUE")

    def cmd_tracker_param_set(name, value, retries=3):
        '''Parameter setting'''
        state = mpstate.tracker_state
        if not state.connection:
            print("tracker not connected")
            return
        return mpstate.mav_param.mavset(state.connection, name.upper(), value, retries=retries)
        
    def cmd_tracker_mode(args):
        '''mode commands'''
        state = mpstate.tracker_state
        if not state.connection:
            print("tracker not connected")
            return
        mode = args[0].upper()
        if mode == "MANUAL":
            state.connection.set_mode_manual()
        elif mode == "AUTO":
            state.connection.set_mode_auto()
        else:
            print('Unknown mode %s: ' % mode)


def init(mpstate):
    '''initialise module'''
    return TrackerModule(mpstate)
