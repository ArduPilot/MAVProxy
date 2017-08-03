#!/usr/bin/env python

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class SyncModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SyncModule, self).__init__(mpstate, "sync")
        self.add_command('sync', self.cmd_sync, "sync")

    def cmd_sync(self, args):
        import time
        if ( len(args) != 1):
            print("Usage: sync CURRENT_TIME")
            return

        time = int(args[0])
        self.master.mav.timesync_send(0, time)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'TIMESYNC':
            print m

def init(mpstate):
    '''initialise module'''
    return SyncModule(mpstate)
