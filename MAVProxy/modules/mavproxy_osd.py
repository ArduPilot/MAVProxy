#!/usr/bin/env python
'''
OSD Module
Andy Piper, August 2020, Vendee, France

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

class osd(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise OSD module"""
        super(osd, self).__init__(mpstate, "osd", "")

        self.request_id = 1
        self.osd_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('osd', self.cmd_osd, "OSD module", ['param-set', 'param-show', 'set (OSDSETTING)'])
        self.type_map = {
            mavutil.mavlink.OSD_PARAM_NONE : "NONE",
            mavutil.mavlink.OSD_PARAM_SERIAL_PROTOCOL : "SERIAL_PROTOCOL",
            mavutil.mavlink.OSD_PARAM_SERVO_FUNCTION : "SERVO_FUNCTION",
            mavutil.mavlink.OSD_PARAM_AUX_FUNCTION : "AUX_FUNCTION",
            mavutil.mavlink.OSD_PARAM_FLIGHT_MODE : "FLIGHT_MODE",
            mavutil.mavlink.OSD_PARAM_FAILSAFE_ACTION : "FAILSAFE_ACTION",
            mavutil.mavlink.OSD_PARAM_FAILSAFE_ACTION_1 : "FAILSAFE_ACTION_1",
            mavutil.mavlink.OSD_PARAM_FAILSAFE_ACTION_2 : "FAILSAFE_ACTION_2" }
        self.invtype_map = { v : k for k, v in self.type_map.items()}

    def usage(self):
        '''show help on command line options'''
        return "Usage: osd <param-set|param-show|set>"

    def cmd_osd(self, args):
        '''control behaviour of the OSD module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "param-set":
            self.param_set(args[1:])
        elif args[0] == "param-show":
            self.param_show(args[1:])
        elif args[0] == "set":
            self.osd_settings.command(args[1:])
        else:
            print(self.usage())

    def param_set(self, args):
        '''sets an OSD parameter'''
        if len(args) < 3 or len(args) > 6:
            print("param-set <screen> <index> <name> (<type> | <min> <max> <increment>)")
            return
        screen = int(args[0], 0)
        index = int(args[1], 0)
        name = args[2]
        type = mavutil.mavlink.OSD_PARAM_NONE
        min_value = 0.
        max_value = 0.
        increment = 0.
        # config type implies the ranges are pre-defined
        if len(args) > 3:
            type = self.string_to_config_type(args[3])
            # can't have config type and ranges
            if type is not None and len(args) > 4:
                print("param-set <screen> <index> <name> (<type> | <min> <max> <increment>)")
                return

        if len(args) > 3 and type is None:
            type = mavutil.mavlink.OSD_PARAM_NONE
            min_value = float(args[3])
            if len(args) > 4:
                max_value = float(args[4])
            if len(args) > 5:
                increment = float(args[5])

        if sys.version_info.major >= 3:
            name = bytearray(name, 'ascii')

        self.master.mav.osd_param_config_send(self.target_system,
                                        self.target_component,
                                        self.request_id,
                                        screen,
                                        index,
                                        name,
                                        type,
                                        min_value,
                                        max_value,
                                        increment)
        self.request_id += 1

    def param_show(self, args):
        '''show an OSD parameter'''
        if len(args) != 2:
            print("param-show <screen> <index>")
            return
        screen = int(args[0], 0)
        index = int(args[1], 0)

        self.master.mav.osd_param_show_config_send(self.target_system,
                                        self.target_component,
                                        self.request_id,
                                        screen,
                                        index)
        self.request_id += 1

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        if mtype == "OSD_PARAM_CONFIG_REPLY" or mtype == "OSD_PARAM_SHOW_CONFIG_REPLY":
            if m.result == mavutil.mavlink.OSD_PARAM_INVALID_PARAMETER:
                print("OSD request %u failed: invalid parameter" % (m.request_id))
            elif m.result != 0:
                print("OSD request %u failed: %u" % (m.request_id, m.result))
            else:
                if mtype == "OSD_PARAM_CONFIG_REPLY":
                    print("OSD parameter set")
                else:
                    print("%s %f %f %f %s" % (m.param_id, m.min_value, m.max_value, m.increment,
                        self.config_type_to_string(m.config_type)))

    def config_type_to_string(self, config_type):
        '''convert config type to a string'''
        if config_type in self.type_map:
            return self.type_map[config_type]
        return None

    def string_to_config_type(self, config_str):
        '''convert a string to a config type'''
        if config_str in self.invtype_map:
            return self.invtype_map[config_str]
        return None

def init(mpstate):
    '''initialise OSD module'''
    return osd(mpstate)
