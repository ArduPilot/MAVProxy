#!/usr/bin/env python
'''
Message Rate viewer
Peter Barker, December 2018

Simply display message rates
'''

import time

from MAVProxy.modules.lib import mp_module
import statistics
import struct
import numpy as np

class messagestats(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(messagestats, self).__init__(mpstate, "messagestats", "")
        
        self.max_timestamps = 20
        self.msg_stats_timeout = 60.0
        self.msg_timestamps = {}
        self.msg_type_lengths = {}

        self.last_calc = time.time()
        self.add_command('messagestats',
                         self.cmd_messagestats,
                         "messagestats module",
                         ['show', 'reset', 'len'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: messagestats <show|reset|len (length)|timeout>"

    def cmd_messagestats(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "show":
            print(self.stats())
        elif args[0] == "reset":
            self.reset()
        elif args[0] == "len":
            if len(args) == 2:
                self.max_timestamps = int(args(1))
            print("messagestats: maximum message timestamps = %d" % self.max_timestamps)
        elif args[0] == "timeout":
            if len(args) == 2:
                self.msg_stats_timeout = float(args[1])
            print("messagerate stats: message timeout = %d" % self.stats_age_timeout)
        else:
            print(self.usage())

    def reset(self):
        '''reset message timestamps'''
        self.msg_timestamps = {}


    def stats(self):
        ret = "\n\n"
        ret += "Message stats - Max timestamps:%u   Msg timeout:%fs\n" % (self.max_timestamps, self.msg_stats_timeout)
        mtypes = sorted(self.msg_timestamps.keys())
        total_datarate = 0
        now = time.time()
        for mtype in mtypes:
            timestamps = np.array(self.msg_timestamps[mtype])
            timestamp_ages = now - timestamps 
            timestamp_not_old = timestamp_ages < self.msg_stats_timeout
            filtered_timestamps = timestamps[timestamp_not_old]
            tstamps_count = len(filtered_timestamps)
            if tstamps_count >= 2:
                periods = filtered_timestamps[1:] - filtered_timestamps[0:-1]  

                period_mean = statistics.mean(periods)
                mean_rate = 1.0 / period_mean
                size = self.msg_type_lengths[mtype]
                mean_bitrate = mean_rate * size
                period_min = min(periods)
                period_max = max(periods)
                age = now - timestamps[-1]
                total_datarate += mean_rate * size
                period_stdev = 0.0
                period_stdev_norm = 0
                if tstamps_count > 8:
                    period_stdev = statistics.stdev(periods)
                    period_stdev_norm = period_stdev / period_mean
                  
                ret += "%25s: %3uB  age:%6.1fs  %5uBps  Period(ms)(%5.0f:mean %5.0u:max  %5.0u:min %5.2f:stddev_norm)    PERIODS:" % (mtype, size, age, mean_bitrate, period_mean*1000.0, period_max*1000, period_min*1000, period_stdev_norm)
                for period in periods:
                    ret += "  %5.0fms" % (period*1000)
                ret += "\n"
        if total_datarate < 1000.0:
            ret += "\n%25s: %0.2fBps\n" %("Total mean", total_datarate)
        else:
            ret += "\n%25s: %0.2fkBps\n" %("Total mean", total_datarate*0.001)
        return ret

#     def idle_task(self):
#         '''called rapidly by mavproxy'''


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        
        if mtype not in self.msg_timestamps.keys():
            self.msg_timestamps[mtype] = []
            try:
                length = struct.calcsize(m.format) + 9
            except:
                length = 9
            self.msg_type_lengths[mtype] = length
            
        self.msg_timestamps[mtype].append(time.time())
        self.msg_timestamps[mtype] = self.msg_timestamps[mtype][-self.max_timestamps:]


def init(mpstate):
    '''initialise module'''
    return messagestats(mpstate)
