#!/usr/bin/env python
'''command long'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class CmdlongModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CmdlongModule, self).__init__(mpstate, "cmdlong")
        self.add_command('setspeed', self.cmd_do_change_speed, "do_change_speed")
        self.add_command('setyaw', self.cmd_condition_yaw, "condition_yaw")
        self.add_command('takeoff', self.cmd_takeoff, "takeoff")

    def cmd_takeoff(self, args):
        '''take off'''
        if ( len(args) != 1):
            print("Usage: takeoff ALTITUDE_IN_METERS")
            return
        
        if (len(args) == 1):
            altitude = float(args[0])
            print("Take Off started")
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                0, # confirmation
                0, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                altitude) # param7

    def cmd_do_change_speed(self, args):
        '''speed value'''
        if ( len(args) != 1):
            print("Usage: speed SPEED_VALUE")
            return
        
        if (len(args) == 1):
            speed = float(args[0])
            print("SPEED %s" % (str(speed)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                0, # confirmation
                0, # param1
                speed, # param2 (Speed value)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def cmd_condition_yaw(self, args):
        '''yaw angle angular_speed angle_mode'''
        if ( len(args) != 3):
            print("Usage: yaw ANGLE ANGULAR_SPEED MODE:[0 absolute / 1 relative]")
            return
        
        if (len(args) == 3):
            angle = float(args[0])
            angular_speed = float(args[1])
            angle_mode = float(args[2])
            print("ANGLE %s" % (str(angle)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                0, # confirmation
                angle, # param1 (angle value)
                angular_speed, # param2 (angular speed value)
                0, # param3
                angle_mode, # param4 (mode: 0->absolute / 1->relative)
                0, # param5
                0, # param6
                0) # param7

def init(mpstate):
    '''initialise module'''
    return CmdlongModule(mpstate)
