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
    "battery" : 0x0100,
    "airspeed": 0x0200,
    "logging" : 0x0400,
    "switch"  : 0x0800,
    "gps_config": 0x1000,
    }

class ArmModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ArmModule, self).__init__(mpstate, "arm", "arm/disarm handling")
        checkables = "<" + "|".join(arming_masks.keys()) + ">"
        self.add_command('arm', self.cmd_arm,      'arm motors', ['check ' + self.checkables(),
                                      'uncheck ' + self.checkables(),
                                      'list',
                                      'throttle',
                                      'safetyon',
                                      'safetyoff'])
        self.add_command('disarm', self.cmd_disarm,   'disarm motors')
        self.was_armed = False

    def checkables(self):
        return "<" + "|".join(arming_masks.keys()) + ">"

    def cmd_arm(self, args):
        '''arm commands'''
        usage = "usage: arm <check|uncheck|list|throttle|safetyon|safetyoff>"

        if len(args) <= 0:
            print(usage)
            return

        if args[0] == "check":
            if (len(args) < 2):
                print("usage: arm check " + self.checkables())
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
                print("usage: arm uncheck " + self.checkables())
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

    def all_checks_enabled(self):
        ''' returns true if the UAV is skipping any arming checks'''
        arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
        if arming_mask == 1:
            return True
        for bit in arming_masks.values():
            if not arming_mask & bit and bit != 1:
                return False
        return True

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == 'HEARTBEAT' and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            armed = self.master.motors_armed()
            if armed != self.was_armed:
                self.was_armed = armed
                if armed and not self.all_checks_enabled():
                    self.say("Arming checks disabled")

def init(mpstate):
    '''initialise module'''
    return ArmModule(mpstate)
