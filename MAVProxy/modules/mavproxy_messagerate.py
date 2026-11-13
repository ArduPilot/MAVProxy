#!/usr/bin/env python3
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
        super().__init__(mpstate, "messagerate", "")
        self.counts = {}
        self.buckets = []
        self.max_buckets = 5

        self.last_calc = time.time()
        # intervals asked for with MAV_CMD_REQUEST_MESSAGE and not yet
        # received: (target system, target component, message ID) -> time
        self.pending_gets = {}
        # (sysid, compid) of vehicles known to answer
        # MAV_CMD_REQUEST_MESSAGE with MESSAGE_INTERVAL
        self.supports_request_message = set()
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
                msg_type = mavlink_map[msg_id]
                mav_cmd_name = msg_type.msgname if hasattr(msg_type, "msgname") else msg_type.name
                if mav_cmd_name == message_name:
                    self.request_message_interval(msg_id)
                    return
            print("Unknown message ID:%s" % message_name)
        elif args[0] == "set":
            if len(args) < 3:
                print(self.usage())
                return
            message_name = args[1]
            message_rate = float(args[2])
            # Special handling for -1 and 0 which correspond to "stop sending" and "reset to default rate"
            message_rate = message_rate if message_rate <= 0 else 1E6/message_rate
            priority = 0
            if len(args) > 3:
              priority = int(args[3])
            mavlink_map = mavutil.mavlink.mavlink_map
            for msg_id in mavlink_map.keys():
                msg_type = mavlink_map[msg_id]
                mav_cmd_name = msg_type.msgname if hasattr(msg_type, "msgname") else msg_type.name
                if mav_cmd_name == message_name:
                    self.master.mav.command_long_send(
                        self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        msg_id, (int) (message_rate), priority, 0, 0, 0, 0)
                    return
            print("Unknown message ID:%s" % message_name)

        else:
            print(self.usage())

    def request_message_interval(self, msg_id):
        '''ask for the interval of msg_id with MAV_CMD_REQUEST_MESSAGE;
        MAV_CMD_GET_MESSAGE_INTERVAL is deprecated, so it is only sent if
        the vehicle does not support the newer method (see mavlink_packet)'''
        target_system = self.settings.target_system
        target_component = self.settings.target_component
        self.pending_gets[(target_system, target_component, msg_id)] = time.time()
        self.master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_MESSAGE_INTERVAL, msg_id, 0, 0, 0, 0, 0)

    def get_message_interval_fallback(self, msg_id, target_system, target_component):
        '''ask for the interval of msg_id with the deprecated
        MAV_CMD_GET_MESSAGE_INTERVAL, for vehicles which cannot supply
        MESSAGE_INTERVAL in response to MAV_CMD_REQUEST_MESSAGE (e.g.
        ArduPilot before 4.7, PX4 before 1.17)'''
        self.master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id, 0, 0, 0, 0, 0, 0)

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
        # forget requests which were never answered
        for key in [k for k, sent in self.pending_gets.items() if now - sent > 5]:
            del self.pending_gets[key]
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

        if mtype == 'COMMAND_ACK':
            if (m.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE and
                    m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED):
                # a vehicle which supports this sends MESSAGE_INTERVAL
                # before the ACK, so any request to this sender still
                # pending got no answer; ask those with the old command.
                # A sender which has already answered this way refused
                # only that one request (e.g. for a message it can't
                # send), so the others are left to be answered.
                source = (m.get_srcSystem(), m.get_srcComponent())
                if source in self.supports_request_message:
                    return
                for key in list(self.pending_gets.keys()):
                    sysid, compid, msg_id = key
                    if sysid != source[0] or compid not in (0, source[1]):
                        continue
                    del self.pending_gets[key]
                    self.get_message_interval_fallback(msg_id, sysid, compid)
            return

        if mtype ==  'MESSAGE_INTERVAL':
            sysid, compid = m.get_srcSystem(), m.get_srcComponent()
            for key in ((sysid, compid, m.message_id), (sysid, 0, m.message_id)):
                if self.pending_gets.pop(key, None) is not None:
                    self.supports_request_message.add((sysid, compid))
            if  m.message_id in mavlink_map:
                msg_type =  mavlink_map[m.message_id]
                mav_cmd_name = msg_type.msgname if hasattr(msg_type, "msgname") else msg_type.name
            else:
                mav_cmd_name = "ID:%s" % m.message_id
            print("Msg:%s  rate:%s" % (mav_cmd_name, self.interval_us_to_rate_string(m.interval_us)))

    @staticmethod
    def interval_us_to_rate_string(interval_us):
        '''convert a MESSAGE_INTERVAL interval_us to a human-readable rate'''
        if interval_us == -1:
            return "disabled"
        if interval_us == 0:
            return "not available"
        if interval_us < 0:
            return "invalid (%d)" % interval_us
        return "%0.2fHz" % (1E6/interval_us)


def init(mpstate):
    '''initialise module'''
    return messagerate(mpstate)
