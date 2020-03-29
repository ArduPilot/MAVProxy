#!/usr/bin/env python
'''
Message Rate viewer
Peter Barker, December 2018

Simply display message rates
'''

import time

from MAVProxy.modules.lib import mp_module


class messagerate(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(messagerate, self).__init__(mpstate, "messagerate", "")
        self.counts = {}
        self.buckets = []
        self.max_buckets = 5

        self.last_calc = time.time()
        self.add_command('messagerate',
                         self.cmd_messagerate,
                         "messagerate module",
                         ['status', 'reset'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: messagerate <status>"

    def cmd_messagerate(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "reset":
            self.reset()
        else:
            print(self.usage())

    def reset(self):
        '''reset rates'''
        self.counts = {}
        self.buckets = []
        self.last_calc = time.time()

    def status(self):
        '''returns rates'''
        counts = {}
        for bucket in self.buckets:
            for x in bucket:
                if not x in counts:
                    counts[x] = 0
                counts[x] += bucket[x]

        ret = ""
        mtypes = counts.keys()
        mtypes = sorted(mtypes)
        for mtype in mtypes:
            ret += "%s: %0.1f/s\n" % (mtype,
                                      counts[mtype]/float(len(self.buckets)))
        return ret

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        time_delta = now - self.last_calc
        if time_delta > 1:
            self.last_calc = now
            self.buckets.append(self.counts)
            self.counts = {}
            if len(self.buckets) > self.max_buckets:
                self.buckets = self.buckets[-self.max_buckets:]

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        if mtype not in self.counts:
            self.counts[mtype] = 0
        self.counts[mtype] += 1


def init(mpstate):
    '''initialise module'''
    return messagerate(mpstate)
