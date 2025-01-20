#!/usr/bin/env python

import time, math
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting


class TestModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TestModule, self).__init__(mpstate, "followme", "followme module")
        self.add_command(
            "followme", self.followme, "show some information about follow me"
        )
        self.add_command(
            "followme2", self.followme2, "show some information about follow me"
        )

    def followme(self, args):
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            0,
            0,
            10,
            10,
            16,
            0,
            0,
        )

    def followme2(self, args):
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            0,
            150,
            10,
            10,
            16,
            0,
            0,
        )

    def mavlink_packet(self, m):
        """handle a mavlink packet"""


def init(mpstate):
    """initialise module"""
    return TestModule(mpstate)
