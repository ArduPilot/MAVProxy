#!/usr/bin/env python
'''
Message Rate viewer
Peter Barker, December 2018

Simply display message rates
'''

import time

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

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
                         ['status', 'reset', 'set', 'get'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: messagerate <status | reset | set(msg)(rate) | get(msg)>"

    def cmd_messagerate(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "reset":
            self.reset()
        elif args[0] == "get":
            if len(args) != 2:
                print(self.usage())
                return
            message_name = args[1]
            mavlink_map = mavutil.mavlink.mavlink_map
            for msg_id in mavlink_map.keys():
                mav_cmd_name = mavlink_map[msg_id].name
                if mav_cmd_name == message_name:
                    self.master.mav.command_long_send(
                        self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                        0,
                        msg_id, 0, 0, 0, 0, 0, 0)
                    return
            print("Unknown message ID:%s" % message_name)
        elif args[0] == "set":
            if len(args) < 3:
                print(self.usage())
                return
            message_name = args[1]
            message_rate = float(args[2])
            priority = 0
            if len(args) > 3:
              priority = int(args[3])
            mavlink_map = mavutil.mavlink.mavlink_map
            for msg_id in mavlink_map.keys():
                mav_cmd_name = mavlink_map[msg_id].name
                if mav_cmd_name == message_name:
                    self.master.mav.command_long_send(
                        self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        msg_id, (int) (1E6/message_rate), priority, 0, 0, 0, 0)
                    return
            print("Unknown message ID:%s" % message_name)
            
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
        
        mavlink_map = mavutil.mavlink.mavlink_map
              
        if mtype ==  'MESSAGE_INTERVAL':
            if  m.message_id in mavlink_map:
                print("Msg:%s  rate:%0.2fHz" % (mavlink_map[m.message_id].name, (1E6/m.interval_us) ) )
            else:
                print("Msg ID:%s  rate:%0.2fHz" % (m.message_id, (1E6/m.interval_us) ) )


def init(mpstate):
    '''initialise module'''
    return messagerate(mpstate)
