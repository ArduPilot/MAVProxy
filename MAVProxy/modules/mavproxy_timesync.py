#!/usr/bin/env python

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class TimeSyncModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TimeSyncModule, self).__init__(mpstate, "timesync")
        self.add_command('timesync', self.cmd_timesync, "timesync")

    def cmd_timesync(self, args):
        import time
        if ( len(args) != 1):
            print("Usage: timesync CURRENT_TIME")
            return

        time = int(args[0])
        self.master.mav.timesync_send(0, time)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'TIMESYNC':
            print m

def init(mpstate):
    '''initialise module'''
    return TimeSyncModule(mpstate)
