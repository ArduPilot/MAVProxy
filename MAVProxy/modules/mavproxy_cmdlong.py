#!/usr/bin/env python
'''command long'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class CmdlongModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CmdlongModule, self).__init__(mpstate, "cmdlong")
        self.add_command('speed', self.cmd_do_change_speed, "do_change_speed")

    def cmd_do_change_speed(self, args):
        '''speed value'''
        if ( len(args) != 1):
            print("Usage: speed SPEED_VALUE")
            return
        
        if (len(args) == 1):
            speed = float(args[0])
            print("SPEED %s" % (str(speed)))
            self.master.mav.command_long_send(
                self.status.target_system,  # target_system
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

def init(mpstate):
    '''initialise module'''
    return CmdlongModule(mpstate)
