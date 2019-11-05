"""
setpos command for SET_POSITION_TARGET_LOCAL_NED
"""

import math
import time

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_settings


class SetPosModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SetPosModule, self).__init__(mpstate, "SetPos", "SetPos", public=False)
        self.add_command('setpos', self.cmd_setpos, "set local pos")
        self.add_command('hop', self.cmd_hop, "hop position")
        self.hop = None
        self.hop_stage = 0
        self.hop_last_time = time.time()
        self.hop_settings = mp_settings.MPSettings(
            [('height', float, 0.7),
             ('takeoff_delay', float, 4.0),
             ('move_delay', float, 4.0)])
        self.add_command('hop', self.cmd_hop, 'HOP control',
                         ["set (HOPSETTING)"])
        self.add_completion_function('(HOPSETTING)',
                                     self.hop_settings.completion)

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

    def cmd_hop(self, args):
        '''start a hop'''
        if len(args) > 0 and args[0] == "set":
            self.hop_settings.command(args[1:])
            return
        if len(args) < 2:
            print("Usage: hop dX dY dYaw")
            return
        dX = float(args[0])
        dY = float(args[1])
        if len(args) > 2:
            dYaw = float(args[2])
        else:
            dYaw = 0
        self.hop_stage = 0
        self.hop_last_time = time.time()
        self.hop = [dX, dY, dYaw]

    def idle_task(self):
        '''run commands when idle'''
        if self.hop is None:
            return
        if time.time() - self.hop_last_time < 0.2:
            return
        if self.status.flightmode != 'GUIDED':
            self.module('mode').cmd_mode(['GUIDED'])
            self.hop_last_time = time.time()
            return
        if not self.master.motors_armed():
            self.module('arm').cmd_arm(['throttle'])
            self.hop_last_time = time.time()
            return
        now = time.time()
        if self.hop_stage == 0:
            self.module('cmdlong').cmd_takeoff([self.hop_settings.height])
            self.hop_last_time = time.time()
            self.hop_stage += 1
        if self.hop_stage == 1:
            if now - self.hop_last_time < self.hop_settings.takeoff_delay:
                return
            dX = self.hop[0]
            dY = self.hop[1]
            dYaw = self.hop[2]
            self.cmd_setpos([dX, dY, 0, dYaw])
            self.hop_last_time = time.time()
            self.hop_stage += 1
        if self.hop_stage == 2:
            if now - self.hop_last_time < self.hop_settings.move_delay:
                return
            self.module('mode').cmd_mode(['LAND'])
            self.hop = None


def init(mpstate):
    '''initialise module'''
    return SetPosModule(mpstate)
