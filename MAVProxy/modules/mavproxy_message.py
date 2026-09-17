#!/usr/bin/env python3
'''
Arbitrary Message Module
Peter Barker, September 2017

'''

import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class message(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(message, self).__init__(mpstate, "message", "")
        self.status_callcount = 0
        self.boredom_interval = 10  # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.message_settings = mp_settings.MPSettings(
            [('verbose', bool, False)])
        self.add_command('message', self.cmd_message, "message module", [])

    def usage(self):
        '''show help on command line options'''
        return "Usage: message TYPE ARG..."

    def cmd_message(self, args):
        if len(args) == 0:
            print(self.usage())
        else:
            packettype = args[0]
            methodname = packettype.lower() + "_send"
            transformed = [eval(x) for x in args[1:]]
            try:
                method = getattr(self.master.mav, methodname)
            except AttributeError:
                print("Unable to find %s" % methodname)
                return
            # pymavlink requires bytes for char-array fields; encode
            # any str supplied for one of those:
            msg_class = getattr(
                mavutil.mavlink,
                "MAVLink_%s_message" % packettype.lower(),
                None)
            if msg_class is not None:
                for i, fieldtype in enumerate(msg_class.fieldtypes):
                    if (fieldtype == "char" and
                            i < len(transformed) and
                            isinstance(transformed[i], str)):
                        transformed[i] = transformed[i].encode("utf8")
            method(*transformed)


def init(mpstate):
    '''initialise module'''
    return message(mpstate)


# STABILIZE> message MISSION_CLEAR_ALL int(1) int(1)
# STABILIZE> Got MAVLink msg: MISSION_ACK {target_system : 255,
# target_component : 0, type : 0}
