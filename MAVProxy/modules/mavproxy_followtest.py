#!/usr/bin/env python
'''
 test follow-me options in ArduPilot
 Andrew Tridgell
 September 2016
'''

import sys, os, time, math
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap
from pymavlink import mavutil
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class FollowTestModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FollowTestModule, self).__init__(mpstate, "followtest", "followtest module")
        self.add_command('followtest', self.cmd_followtest, "followtest control",
                         ['set (FOLLOWSETTING)'])
        self.follow_settings = mp_settings.MPSettings([("radius", float, 100.0),
                                                       ("altitude", float, 50.0),
                                                       ("speed", float, 10.0),
                                                       ("type", str, 'guided'),
                                                       ("vehicle_throttle", float, 0.5),
                                                       ("disable_msg", bool, False)])
        self.add_completion_function('(FOLLOWSETTING)', self.follow_settings.completion)
        self.target_pos = None
        self.last_update = 0
        self.circle_dist = 0
        
    def cmd_followtest(self, args):
        '''followtest command parser'''
        usage = "usage: followtest <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.follow_settings.command(args[1:])
        else:
            print(usage)

    def update_target(self, time_boot_ms):
        '''update target on map'''
        if not self.mpstate.map:
            # don't draw if no map
            return
        if not 'HOME_POSITION' in self.master.messages:
            return

        home_position = self.master.messages['HOME_POSITION']

        now = time_boot_ms * 1.0e-3
        dt = now - self.last_update
        if dt < 0:
            dt = 0
        self.last_update = now

        self.circle_dist += dt * self.follow_settings.speed

        # assume a circle for now
        circumference = math.pi * self.follow_settings.radius * 2
        rotations = math.fmod(self.circle_dist, circumference) / circumference
        angle = math.pi * 2 * rotations
        self.target_pos = mp_util.gps_newpos(home_position.latitude*1.0e-7,
                                             home_position.longitude*1.0e-7,
                                             math.degrees(angle),
                                             self.follow_settings.radius)

        icon = self.mpstate.map.icon('camera-small-red.png')
        (lat, lon) = (self.target_pos[0], self.target_pos[1])
        self.mpstate.map.add_object(mp_slipmap.SlipIcon('followtest',
                                                        (lat, lon),
                                                        icon, layer='FollowTest', rotation=0, follow=False))
        

    def idle_task(self):
        '''update vehicle position'''
        pass


    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if not self.mpstate.map:
            # don't draw if no map
            return

        if m.get_type() != 'GLOBAL_POSITION_INT':
            return
        self.update_target(m.time_boot_ms)

        if self.target_pos is None:
            return

        if self.follow_settings.disable_msg:
            return

        if self.follow_settings.type == 'guided':
            # send normal guided mode packet
            self.master.mav.mission_item_int_send(self.settings.target_system,
                                                  self.settings.target_component,
                                                  0,
                                                  self.module('wp').get_default_frame(),
                                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                  2, 0, 0, 0, 0, 0,
                                                  int(self.target_pos[0]*1.0e7), int(self.target_pos[1]*1.0e7),
                                                  self.follow_settings.altitude)

        elif self.follow_settings.type == 'yaw':
            # display yaw from vehicle to target
            vehicle = (m.lat*1.0e-7, m.lon*1.0e-7)
            vehicle_yaw = math.degrees(self.master.field('ATTITUDE', 'yaw', 0))
            target_bearing = mp_util.gps_bearing(vehicle[0], vehicle[1], self.target_pos[0], self.target_pos[1])
            # wrap the angle from -180 to 180 thus commanding the vehicle to turn left or right
            # note its in centi-degrees so *100
            relyaw = mp_util.wrap_180(target_bearing - vehicle_yaw) * 100

            self.master.mav.command_long_send(self.settings.target_system,
                                                  self.settings.target_component,
                                                  mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED, 0,
                                                  relyaw,
                                                  self.follow_settings.vehicle_throttle,
                                                  0, 0, 0, 0, 0)

def init(mpstate):
    '''initialise module'''
    return FollowTestModule(mpstate)
