#!/usr/bin/env python
'''
Msg Module
Peter barker, September 2016

Simple "msg" command sends statustext

'''

from pymavlink import mavutil
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class msg(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(msg, self).__init__(mpstate, "msg", "")
        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.msg_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('msg', self.cmd_msg, "statustext sending", [])

    def usage(self):
        '''show help on command line options'''
        return "Usage: msg message"

    def cmd_msg(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        txt = ' '.join(args)
        self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                        txt)

def init(mpstate):
    '''initialise module'''
    return msg(mpstate)
