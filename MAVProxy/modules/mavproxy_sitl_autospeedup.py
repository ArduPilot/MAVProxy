#!/usr/bin/env python
'''sitl_autospeedup Module
Peter Barker, January 2022

This module is designed to work with ArduPilot's SITL.

It will detect a vehicle which is relatively freshly booted and
currently not passing prearm checks.  If detected, it will change the
`SIM_SPEEDUP` parameter to a large value.  Once the bit goes true it
will drop the speedup again.

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class sitl_autospeedup(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(sitl_autospeedup, self).__init__(mpstate, "sitl_autospeedup", "")

        self.sitl_autospeedup_settings = mp_settings.MPSettings(
            [ ('speedup', int, 100),
              ('max_time_boot', int, 30),
              ('verbose', bool, False),
          ])

        self.old_speedup = None
        self.last_sent_request_sys_status = 0

    def usage(self):
        '''show help on command line options'''
        return "Usage: sitl_autospeedup <set>"

    def cmd_sitl_autospeedup(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "set":
            self.sitl_autospeedup_settings.command(args[1:])
        else:
            print(self.usage())

    def send_request_sys_status(self):
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, # command
            0, # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if not self.mpstate.sitl_output:
            return

        try:
            self.master.messages['HEARTBEAT']
        except KeyError:
            return
        try:
            SYS_STATUS = self.master.messages['SYS_STATUS']
        except KeyError:
            # Copter does not persist streamrates set via
            # request_data_stream, and MAVProxy only sets them at a
            # relatively slow rate.  If we don't go and ask for
            # SYS_STATUS it can take several seconds for us to get
            # one...
            now = time.time()
            if now - self.last_sent_request_sys_status > 0.5:
                self.send_request_sys_status()
                self.last_sent_request_sys_status = now
            return
        parameter_should_be_set = (
            self.mpstate.attitude_time_s < self.sitl_autospeedup_settings.max_time_boot and
            ((SYS_STATUS.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK) == 0))
        new_speedup = None
#        print("Should: %s t=%u h=%u" % (str(parameter_should_be_set), self.mpstate.attitude_time_s, (SYS_STATUS.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)))
        if parameter_should_be_set and self.old_speedup is None:
            self.old_speedup = self.get_mav_param("SIM_SPEEDUP")
            new_speedup = self.sitl_autospeedup_settings.speedup
        elif not parameter_should_be_set and self.old_speedup is not None:
            new_speedup = self.old_speedup
            self.old_speedup = None

        if new_speedup is not None:
            self.say("sitl_autospeedup: SIM_SPEEDUP to %u" % (new_speedup, ))
            self.param_set("SIM_SPEEDUP", new_speedup)

def init(mpstate):
    '''initialise module'''
    return sitl_autospeedup(mpstate)
