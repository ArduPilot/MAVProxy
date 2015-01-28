#!/usr/bin/env python
'''
gimbal control module
Andrew Tridgell
January 2015
'''

import sys, os, time
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
from pymavlink.rotmat import Vector3
from math import radians

class GimbalModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GimbalModule, self).__init__(mpstate, "gimbal", "gimbal control module")
        self.add_command('gimbal', self.cmd_gimbal, "gimbal link control",
                         ['<rate|status>'])

    def cmd_gimbal(self, args):
        '''control gimbal'''
        usage = 'Usage: gimbal <rate|status>'
        if len(args) == 0:
            print(usage)
            return
        if args[0] == 'rate':
            self.cmd_gimbal_rate(args)
        elif args[0] == 'status':
            self.cmd_gimbal_status(args)

    def cmd_gimbal_rate(self, args):
        '''control gimbal rate'''
        if len(args) != 4:
            print("usage: gimbal rate YAW ROLL PITCH")
            return
        (yaw, roll, pitch) = (float(args[1]), float(args[2]), float(args[3]))
        self.master.mav.gimbal_control_send(self.target_system,
                                            mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                                            radians(roll),
                                            radians(pitch),
                                            radians(yaw),
                                            0, 0, 0)
    def cmd_gimbal_status(self, args):
        '''show gimbal status'''
        master = self.master
        if 'GIMBAL_REPORT' in master.messages:
            print(master.messages['GIMBAL_REPORT'])
        else:
            print("No GIMBAL_REPORT messages")


def init(mpstate):
    '''initialise module'''
    return GimbalModule(mpstate)
