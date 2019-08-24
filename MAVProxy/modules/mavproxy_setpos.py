"""
setpos command for SET_POSITION_TARGET_LOCAL_NED
"""

import math

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil


class SetPosModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SetPosModule, self).__init__(mpstate, "SetPos", "SetPos", public=False)
        self.add_command('setpos', self.cmd_setpos, "set local pos")

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        pass

    def cmd_setpos(self, args):
        '''set local position'''
        if len(args) < 3:
            print("Usage: setpos dX dY dZ dYaw")
            return
        dX = float(args[0])
        dY = float(args[1])
        dZ = float(args[2])
        if len(args) >= 4:
            dYaw = math.radians(float(args[3]))
        else:
            dYaw = 0.0
        self.master.mav.set_position_target_local_ned_send(
            0, # timestamp
            self.target_system, # target system_id
            self.target_component, # target component id
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b1111101111111000, # mask specifying use-only-x-y-z-yaw
            dX, # x
            dY, # y
            -dZ,# z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            dYaw, # yaw
            0, # yawrate
            )


def init(mpstate):
    '''initialise module'''
    return SetPosModule(mpstate)
