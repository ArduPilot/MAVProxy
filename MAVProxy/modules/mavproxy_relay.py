#!/usr/bin/env python
'''relay handling module'''

import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class RelayModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RelayModule, self).__init__(mpstate, "relay")
        self.add_command('relay', self.cmd_relay, "relay commands")
        self.add_command('servo', self.cmd_servo, "servo commands")
        self.add_command('motortest', self.cmd_motortest, "motortest commands")

    def cmd_relay(self, args):
        '''set relays'''
        if len(args) == 0 or args[0] not in ['set', 'repeat']:
            print("Usage: relay <set|repeat>")
            return
        if args[0] == "set":
            if len(args) < 3:
                print("Usage: relay set <RELAY_NUM> <0|1>")
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
                                                   int(args[1]), int(args[2]),
                                                   0, 0, 0, 0, 0)
        if args[0] == "repeat":
            if len(args) < 4:
                print("Usage: relay repeat <RELAY_NUM> <COUNT> <PERIOD>")
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY, 0,
                                                   int(args[1]), int(args[2]), float(args[3]),
                                                   0, 0, 0, 0)

    def cmd_servo(self, args):
        '''set servos'''
        if len(args) == 0 or args[0] not in ['set', 'repeat']:
            print("Usage: servo <set|repeat>")
            return
        if args[0] == "set":
            if len(args) < 3:
                print("Usage: servo set <SERVO_NUM> <PWM>")
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                                   int(args[1]), int(args[2]),
                                                   0, 0, 0, 0, 0)
        if args[0] == "repeat":
            if len(args) < 5:
                print("Usage: servo repeat <SERVO_NUM> <PWM> <COUNT> <PERIOD>")
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0,
                                                   int(args[1]), int(args[2]), int(args[3]), float(args[4]),
                                                   0, 0, 0)


    def cmd_motortest(self, args):
        '''run motortests on copter'''
        if len(args) < 4:
            print("Usage: motortest motornum type(0=percent, 1=PWM, 2=RC-passthru) value timeout(s) <count>")
            return
        if len(args) == 5:
            count = int(args[4])
        else:
            count = 0
        self.master.mav.command_long_send(self.target_system,
                                          0,
                                          mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                          int(args[0]), int(args[1]), float(args[2]), int(args[3]), count,
                                          0, 0)


def init(mpstate):
    '''initialise module'''
    return RelayModule(mpstate)
