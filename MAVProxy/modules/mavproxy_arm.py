#!/usr/bin/env python3
'''arm/disarm command handling'''

import copy

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

# note that the number of bits here is contrained by the float
# transport mechanism.  25 bits is the limit.  Given the transport
# limit, if all bits are set, or all but the lowest bit it set, it is
# reasonable to set the mask to "all" for simplicity.
arming_masks = {
    "all"     :    1 <<  0,
    "baro"    :    1 <<  1,
    "compass" :    1 <<  2,
    "gps"     :    1 <<  3,
    "ins"     :    1 <<  4,
    "params"  :    1 <<  5,
    "rc"      :    1 <<  6,
    "voltage" :    1 <<  7,
    "battery" :    1 <<  8,
    "airspeed":    1 <<  9,
    "logging" :    1 << 10,
    "switch"  :    1 << 11,
    "gps_config":  1 << 12,
    "system":      1 << 13,
    "mission":     1 << 14,
    "rangefinder": 1 << 15,
    "unknown16":   1 << 16,
    "unknown17":   1 << 17,
    "unknown18":   1 << 18,
    "unknown19":   1 << 19,
    "unknown20":   1 << 20,
    "unknown21":   1 << 21,
    "unknown22":   1 << 22,
    "unknown23":   1 << 23,
    "unknown24":   1 << 24,
}
arming_skpcheck_bits = copy.copy(arming_masks)
del arming_skpcheck_bits["all"]

# on the assumption we may not always know about all arming bits, we
# use this "full" mask to transition from using 0x1 (all checks
# enabled) to having checks disabled by turning its bit off.
full_arming_mask = 0b1111111111111111111111110

class ArmModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ArmModule, self).__init__(mpstate, "arm", "arm/disarm handling", public=True)
        self.add_command(
            'arm',
            self.cmd_arm,
            'arm motors', [
                'check ' + self.checkables(),
                'uncheck ' + self.checkables(),
                'skip ' + self.skip_checkables(),
                'unskip ' + self.skip_checkables(),
                'list',
                'throttle',
                'safetyon',
                'safetystatus',
                'safetyoff',
                'bits',
                'prearms',
            ])
        self.add_command('disarm', self.cmd_disarm,   'disarm motors')
        self.was_armed = False

    def checkables(self):
        return "<" + "|".join(arming_masks.keys()) + ">"

    def skip_checkables(self):
        return "<" + "|".join(arming_skpcheck_bits.keys()) + ">"

    def cmd_arm(self, args):
        '''arm commands'''
        usage = "usage: arm <check|uncheck|skip|unskip|list|throttle|safetyon|safetyoff|safetystatus|bits|prearms>"

        if len(args) <= 0:
            print(usage)
            return

        if args[0] == "skip":
            return self.arm_skip(args[1:])

        if args[0] == "unskip":
            return self.arm_unskip(args[1:])

        if args[0] == "check":
            if (len(args) < 2):
                print("usage: arm check " + self.checkables())
                return

            arming_mask = self.get_mav_param("ARMING_CHECK")
            if arming_mask is None:
                print("ARMING_CHECK parameter not available")
                return
            arming_mask = int(arming_mask)
            name = args[1].lower()
            if name == 'all':
                arming_mask = 1
            elif name in arming_masks:
                arming_mask |= arming_masks[name]
            else:
                print("unrecognized arm check:", name)
                return
            if (arming_mask & ~0x1) == full_arming_mask:
                arming_mask = 0x1
            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "uncheck":
            if (len(args) < 2):
                print("usage: arm uncheck " + self.checkables())
                return

            arming_mask = self.get_mav_param("ARMING_CHECK")
            if arming_mask is None:
                print("ARMING_CHECK parameter not available")
                return
            arming_mask = int(arming_mask)
            name = args[1].lower()
            if name == 'all':
                arming_mask = 0
            elif name in arming_masks:
                if arming_mask == arming_masks["all"]:
                    arming_mask = full_arming_mask
                arming_mask &= ~arming_masks[name]
            else:
                print("unrecognized arm check:", args[1])
                return

            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "list":
            arming_check = self.get_mav_param("ARMING_CHECK")
            if arming_check is not None:
                return self.arming_list_ARMING_CHECK()
            arming_skip = self.get_mav_param("ARMING_SKIPCHK")
            if arming_skip is not None:
                return self.arming_list_ARMING_SKIPCHK()


        if args[0] == "bits":
            for mask in sorted(arming_masks, key=lambda x : arming_masks[x]):
                print("%s" % mask)
            return

        if args[0] == "prearms":
            self.master.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, # command
                0, # confirmation
                0, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
            return

        if args[0] == "throttle":
            p2 = 0
            if len(args) == 2 and args[1] == 'force':
                p2 = 2989
            self.master.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                1, # param1 (1 to indicate arm)
                p2, # param2  (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
            return

        if args[0] == "safetyon":
            self.master.mav.set_mode_send(self.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          1)
            return

        if args[0] == "safetystatus":
            try:
                sys_status = self.master.messages['SYS_STATUS']
            except KeyError:
                print("Unknown; no SYS_STATUS")
                return
            if sys_status.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                print("Safety is OFF (vehicle is dangerous)")
            else:
                print("Safety is ON (vehicle allegedly safe)")
            return

        if args[0] == "safetyoff":
            self.master.mav.set_mode_send(self.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          0)
            return

        print(usage)

    def arming_list_ARMING_CHECK(self):
        arming_mask = int(self.get_mav_param("ARMING_CHECK"))
        if arming_mask == 0:
            print("NONE")
        for name in sorted(arming_masks, key=lambda x : arming_masks[x]):
            if arming_masks[name] & arming_mask:
                print(name)

    def arming_list_ARMING_SKIPCHK(self):
        arming_skip = int(self.get_mav_param("ARMING_SKIPCHK"))
        for name in sorted(arming_masks, key=lambda x : arming_masks[x]):
            bang = ""
            if arming_masks[name] & arming_skip:
                bang = "!"
            print(f"{bang}{name}")

    def _arm_skip(self, what, args):
        if (len(args) != 1):
            print(f"usage: arm {what} {self.skip_checkables()}")
            return

        arming_skip = self.get_mav_param("ARMING_SKIPCHK")
        if arming_skip is None:
            print("ARMING_SKIPCHK parameter not available")
            return
        arming_skip = int(arming_skip)
        name = args[0].lower()
        if name not in arming_masks:
            print(f"unrecognized arm check: {name}")
            return
        if what == "skip":
            arming_skip |= arming_masks[name]
        elif what == "unskip":
            arming_skip &= ~arming_masks[name]
        else:
            raise ValueError(what)

        self.param_set("ARMING_SKIPCHK", arming_skip)

    def arm_skip(self, args):
        return self._arm_skip("skip", args)

    def arm_unskip(self, args):
        return self._arm_skip("unskip", args)

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
        arming_check = self.get_mav_param("ARMING_CHECK")
        if arming_check is not None:
            return self.all_checks_enabled_ARMING_CHECK()
        skipchk = self.get_mav_param("ARMING_SKIPCHK")
        if skipchk is not None:
            return self.all_checks_enabled_ARMING_SKIPCHK()

        # antenna tracker doesn't have either parameter
        return False

    def all_checks_enabled_ARMING_CHECK(self):
        arming_mask = int(self.get_mav_param("ARMING_CHECK"))
        if arming_mask == 1:
            return True
        for bit in arming_masks.values():
            if not arming_mask & bit and bit != 1:
                return False
        return True

    def all_checks_enabled_ARMING_SKIPCHK(self):
        arming_skip_mask = int(self.get_mav_param("ARMING_SKIPCHK"))
        return arming_skip_mask == 0

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == 'HEARTBEAT' and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            armed = self.master.motors_armed()
            if armed != self.was_armed:
                self.was_armed = armed
                if armed and not self.all_checks_enabled():
                    self.say("Arming checks disabled")
                ice_enable = self.get_mav_param('ICE_ENABLE', 0)
                if ice_enable == 1:
                    rc = self.master.messages["RC_CHANNELS"]
                    v = self.mav_param.get('ICE_START_CHAN', None)
                    if v is None:
                        return
                    v = getattr(rc, 'chan%u_raw' % v)
                    if v <= 1300:
                        self.say("ICE Disabled")


def init(mpstate):
    '''initialise module'''
    return ArmModule(mpstate)
