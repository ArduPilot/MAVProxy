#!/usr/bin/env python
'''
System_Time Module
Peter barker, May 2018

Send mavlink SYSTEM_TIME messages via mavlink

'''

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class system_time(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(system_time, self).__init__(mpstate, "system_time", "")
        self.last_sent = 0
        self.last_sent_ts1 = 0
        self.last_sent_timesync = 0
        self.module_load_time = time.time()

        self.system_time_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ('interval_timesync', int, 10),
             ('interval', int, 10)])
        self.add_command('system_time',
                         self.cmd_system_time,
                         "system_time module",
                         ['status', 'set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: system_time <status|set>"

    def cmd_system_time(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.system_time_settings.command(args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        return "Perfectly happy.  All is good with the world"

    def uptime(self):
        '''return system uptime in ms'''
        return int(1000*(time.time()-self.module_load_time))

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()

        if now-self.last_sent > self.system_time_settings.interval:
            self.last_sent = now
            time_us = time.time() * 1000000
            if self.system_time_settings.verbose:
                print("ST: Sending system time: (%u/%u)" %
                      (time_us, self.uptime(),))
            self.master.mav.system_time_send(time_us,
                                             self.uptime())

        if (now-self.last_sent_timesync >
            self.system_time_settings.interval_timesync):
            self.last_sent_timesync = now
            time_ns = time.time() * 1000000000
            time_ns += 1234
            if self.system_time_settings.verbose:
                print("ST: Sending timesync request")
            self.master.mav.timesync_send(0, time_ns)
            self.last_sent_ts1 = time_ns

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'SYSTEM_TIME':
            if self.system_time_settings.verbose:
                print("ST: Received from (%u/%u): %s" %
                      (m.get_srcSystem(), m.get_srcComponent(), m))
        if m.get_type() == 'TIMESYNC':
            if m.tc1 == 0:
                # this is a request for a timesync response
                time_ns = time.time() * 1000000000
                time_ns += 1234
                if True or self.system_time_settings.verbose:
                    if self.system_time_settings.verbose:
                        print("ST: received timesync; sending response: %u" %
                              (time_ns))
                    self.master.mav.timesync_send(time_ns,
                                                  m.ts1)
            else:
                if m.ts1 == self.last_sent_ts1:
                    # we sent this one!
                    now_ns = time.time() * 1000000000
                    now_ns += 1234
                    if self.system_time_settings.verbose:
                        print("ST: timesync response: sysid=%u latency=%fms" %
                              (m.get_srcSystem(),
                               (now_ns-self.last_sent_ts1)/1000000.0))

def init(mpstate):
    '''initialise module'''
    return system_time(mpstate)
