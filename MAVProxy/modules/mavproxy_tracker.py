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
from MAVProxy.modules.mavproxy_param import ParamState

# this should be in mavutil.py
mode_mapping_antenna = {
    'MANUAL' : 0,
    'AUTO' : 10,
    'INITIALISING' : 16
    }

class TrackerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        from pymavlink import mavparm
        super(TrackerModule, self).__init__(mpstate, "tracker", "antenna tracker control module")
        self.connection = None
        self.tracker_param = mavparm.MAVParmDict()
        self.pstate = ParamState(self.tracker_param, self.logdir, self.vehicle_name, 'tracker.parm')
        self.tracker_settings = mp_settings.MPSettings(
            [ ('port', str, "/dev/ttyUSB0"),
              ('baudrate', int, 57600),
              ('debug', int, 0)
              ]
            )
        self.add_command('tracker', self.cmd_tracker,
                         "antenna tracker control module",
                         ['<start|arm|disarm|level|mode|position|calpress|mode>',
                          'set (TRACKERSETTING)',
                          'param <set|show|fetch|help> (TRACKERPARAMETER)',
                          'param (TRACKERSETTING)'])
        self.add_completion_function('(TRACKERSETTING)', self.tracker_settings.completion)
        self.add_completion_function('(TRACKERPARAMETER)', self.complete_parameter)

    def complete_parameter(self, text):
        '''complete a tracker parameter'''
        return self.tracker_param.keys()

    def find_connection(self):
        '''find an antenna tracker connection if possible'''
        if self.connection is not None:
            return self.connection
        for m in self.mpstate.mav_master:
            if 'HEARTBEAT' in m.messages:
                if m.messages['HEARTBEAT'].type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
                    return m
        return None

    def cmd_tracker(self, args):
        '''tracker command parser'''
        usage = "usage: tracker <start|set|arm|disarm|level|param|mode|position> [options]"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "start":
            self.cmd_tracker_start()
        elif args[0] == "set":
            self.tracker_settings.command(args[1:])
        elif args[0] == 'arm':
            self.cmd_tracker_arm()
        elif args[0] == 'disarm':
            self.cmd_tracker_disarm()
        elif args[0] == 'level':
            self.cmd_tracker_level()
        elif args[0] == 'param':
            self.cmd_tracker_param(args[1:])
        elif args[0] == 'mode':
            self.cmd_tracker_mode(args[1:])
        elif args[0] == 'position':
            self.cmd_tracker_position(args[1:])
        elif args[0] == 'calpress':
            self.cmd_tracker_calpress(args[1:])
        else:
            print(usage)

    def cmd_tracker_position(self, args):
        '''tracker manual positioning commands'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        positions = [0, 0, 0, 0, 0] # x, y, z, r, buttons. only position[0] (yaw) and position[1] (pitch) are currently used
        for i in range(0, 4):
            if len(args) > i:
                positions[i] = int(args[i]) # default values are 0
        connection.mav.manual_control_send(connection.target_system,
                                           positions[0], positions[1],
                                           positions[2], positions[3],
                                           positions[4])

    def cmd_tracker_calpress(self, args):
        '''calibrate barometer on tracker'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        connection.calibrate_pressure()

    def cmd_tracker_mode(self, args):
        '''set arbitrary mode'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        mode_mapping = connection.mode_mapping()
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
        connection.set_mode(mode_mapping[mode])

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet from the master vehicle. Relay it to the tracker
        if it is a GLOBAL_POSITION_INT'''
        if m.get_type() in ['GLOBAL_POSITION_INT', 'SCALED_PRESSURE']:
            connection = self.find_connection()
            if not connection:
                return
            if m.get_srcSystem() != connection.target_system:
                connection.mav.send(m)

    def idle_task(self):
        '''called in idle time'''
        if not self.connection:
            return

        # check for a mavlink message from the tracker
        m = self.connection.recv_msg()
        if m is None:
            return

        if self.tracker_settings.debug:
            print(m)

        self.pstate.handle_mavlink_packet(self.connection, m)
        self.pstate.fetch_check(self.connection)

        if self.module('map') is None:
            return

        if m.get_type() == 'GLOBAL_POSITION_INT':
            (self.lat, self.lon, self.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
            if self.lat != 0 or self.lon != 0:
                self.module('map').create_vehicle_icon('AntennaTracker', 'red', follow=False, vehicle_type='antenna')
                self.mpstate.map.set_position('AntennaTracker', (self.lat, self.lon), rotation=self.heading)


    def cmd_tracker_start(self):
        if self.tracker_settings.port == None:
            print("tracker port not set")
            return
        if self.connection is not None:
            self.connection.close()
            self.connection = None
            print("Closed old connection")
        print("connecting to tracker %s at %d" % (self.tracker_settings.port,
                                                  self.tracker_settings.baudrate))
        m = mavutil.mavlink_connection(self.tracker_settings.port,
                                       autoreconnect=True,
                                       source_system=self.settings.source_system,
                                       baud=self.tracker_settings.baudrate)
        m.mav.srcComponent = self.settings.source_component
        if self.logdir:
            m.setup_logfile(os.path.join(self.logdir, 'tracker.tlog'))
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

    def cmd_tracker_param(self, args):
        '''Parameter commands'''
        if not self.connection:
            print("tracker not connected")
            return
        self.pstate.handle_command(self.connection, self.mpstate, args)

def init(mpstate):
    '''initialise module'''
    return TrackerModule(mpstate)
