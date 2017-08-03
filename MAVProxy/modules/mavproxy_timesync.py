#!/usr/bin/env python

from __future__ import print_function

"""
Currently what I do is run a sync every second and keep the following:
time_send
time_receive

As a result the delta between GCS time and drone time is:
dtime = (time_send + time_receive) / 2 - drone_time;

And when doing that a lot the dtime will average to the correct
delta. As a result it is now possible to know exactly when a
"GLOBAL_POSITION_INT" was created in GCS time.
"""

from MAVProxy.modules.lib import mp_module


class TimeSyncModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TimeSyncModule, self).__init__(mpstate, "timesync")
        self.add_command('timesync', self.cmd_timesync, "timesync")

    def cmd_timesync(self, args):
        if (len(args) != 1):
            print("Usage: timesync CURRENT_TIME")
            return

        self.master.mav.timesync_send(0, int(args[0]))

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'TIMESYNC':
            print(m)


def init(mpstate):
    '''initialise module'''
    return TimeSyncModule(mpstate)
