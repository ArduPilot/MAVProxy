#!/usr/bin/env python
'''arm/disarm command handling'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

arming_masks = {
    "all"     : 0x0001,
    "baro"    : 0x0002,
    "compass" : 0x0004,
    "gps"     : 0x0008,
    "ins"     : 0x0010,
    "params"  : 0x0020,
    "rc"      : 0x0040,
    "voltage" : 0x0080,
    "battery" : 0x0100
    }

class ArmModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ArmModule, self).__init__(mpstate, "arm", "arm/disarm handling")
        self.add_command('arm', self.cmd_arm,      'arm motors', ['check <all|baro|compass|gps|ins|params|rc|voltage|battery>',
                                      'uncheck <all|baro|compass|gps|ins|params|rc|voltage|battery>',
                                      'list',
                                      'throttle',
                                      'safetyon',
                                      'safetyoff'])
        self.add_command('disarm', self.cmd_disarm,   'disarm motors')


    def cmd_arm(self, args):
        '''arm commands'''
        usage = "usage: arm <check|uncheck|list|throttle|safetyon|safetyoff>"
        checkables = "<all|baro|compass|gps|ins|params|rc|voltage|battery>"

        if len(args) <= 0:
            print(usage)
            return

        if args[0] == "check":
            if (len(args) < 2):
                print("usage: arm check", checkables)
                return

            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            name = args[1].lower()
            if name == 'all':
                for name in arming_masks.keys():
                    arming_mask |= arming_masks[name]
            elif name in arming_masks:
                arming_mask |= arming_masks[name]
            else:
                print("unrecognized arm check:", name)
                return
            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "uncheck":
            if (len(args) < 2):
                print("usage: arm uncheck", checkables)
                return

            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            name = args[1].lower()
            if name == 'all':
                arming_mask = 0
            elif name in arming_masks:
                arming_mask &= ~arming_masks[name]
            else:
                print("unrecognized arm check:", args[1])
                return

            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "list":
            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            if arming_mask == 0:
                print("NONE")
            for name in arming_masks.keys():
                if arming_masks[name] & arming_mask:
                    print(name)
            return

        if args[0] == "throttle":
            self.master.arducopter_arm()
            return

        if args[0] == "safetyon":
            self.master.mav.set_mode_send(self.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          1)
            return

        if args[0] == "safetyoff":
            self.master.mav.set_mode_send(self.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          0)
            return

        print(usage)

    def cmd_disarm(self, args):
        '''disarm motors'''
        p2 = 0
        if len(args) == 1 and args[0] == 'force':
            p2 = 21196
        self.master.mav.command_long_send(
            self.target_system,  # target_system
            0,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0, # confirmation
            0, # param1 (0 to indicate disarm)
            p2, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

def init(mpstate):
    '''initialise module'''
    return ArmModule(mpstate)
