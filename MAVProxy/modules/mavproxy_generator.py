#!/usr/bin/env python
'''
Generator Module - module for generators reporting via the GENERATOR_STATUS message
Peter Barker, Jun 2020
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


class generator(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(generator, self).__init__(mpstate, "generator", "")

        self.generator_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('generator',
                         self.cmd_generator,
                         "generator module",
                         ['status','set (LOGSETTING)'])
        self.console_row = 6
        self.console.set_status('Generator', 'No generator messages', row=self.console_row)
        self.last_seen_generator_message = 0
        self.mpstate = mpstate
        self.last_set_interval_sent = 0

    def usage(self):
        '''show help on command line options'''
        return "Usage: generator <status|set>"

    def cmd_generator(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.generator_settings.command(args[1:])
        else:
            print(self.usage())

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GENERATOR_STATUS':
            if self.generator_settings.verbose:
                print("Got generator message")
            self.last_seen_generator_message = time.time()
            error_string = ""
            errors = []
            prefix = "MAV_GENERATOR_STATUS_FLAG_"
            len_prefix = len(prefix)
            flags = []
            for i in range(64):
                if m.status & (1<<i):
                    try:
                        name = mavutil.mavlink.enums["MAV_GENERATOR_STATUS_FLAG"][1<<i].name
                        if name.startswith(prefix):
                            name = name[len_prefix:]
                    except KeyError:
                        name = "UNKNOWN=%u" % (1<<i)
                    flags.append(name)
            self.console.set_status(
                'Generator',
                'Generator: RPM:%u current:%u volts:%f flags:%s runtime:%u maint:%d' %
                (m.generator_speed,
                 m.load_current,
                 m.bus_voltage,
                 ",".join(flags),
                 m.runtime,
                 m.time_until_maintenance,
                 ),
                row=self.console_row)

        now  = time.time()
        if now - self.last_seen_generator_message > 10:
            # request the message once per second:
            if now - self.last_set_interval_sent > 1:
                self.last_set_interval_sent = now
                self.master.mav.command_long_send(
                    self.mpstate.settings.target_system,
                    self.mpstate.settings.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,  # confirmation
                    mavutil.mavlink.MAVLINK_MSG_ID_GENERATOR_STATUS,  # msg id
                    100000,  # interval
                    0,  # p3
                    0,  # p4
                    0,  # p5
                    0,  # p6
                    0)  # p7


def init(mpstate):
    '''initialise module'''
    return generator(mpstate)
