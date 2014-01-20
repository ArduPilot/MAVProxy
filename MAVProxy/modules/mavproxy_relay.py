#!/usr/bin/env python
'''relay handling module'''

import time
from pymavlink import mavutil

def name():
    '''return module name'''
    return "relay"

def description():
    '''return module description'''
    return "relay handling"

def cmd_relay(args):
    '''set relays'''
    if len(args) == 0 or args[0] not in ['set', 'repeat']:
        print("Usage: relay <set|repeat>")
        return
    if args[0] == "set":
        if len(args) < 3:
            print("Usage: relay set <RELAY_NUM> <0|1>")
            return
        mpstate.master().mav.command_long_send(mpstate.status.target_system,
                                               mpstate.status.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
                                               int(args[1]), int(args[2]),
                                               0, 0, 0, 0, 0)
    if args[0] == "repeat":
        if len(args) < 4:
            print("Usage: relay repeat <RELAY_NUM> <COUNT> <PERIOD>")
            return
        mpstate.master().mav.command_long_send(mpstate.status.target_system,
                                               mpstate.status.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY, 0,
                                               int(args[1]), int(args[2]), float(args[3]),
                                               0, 0, 0, 0)

def cmd_servo(args):
    '''set servos'''
    if len(args) == 0 or args[0] not in ['set', 'repeat']:
        print("Usage: servo <set|repeat>")
        return
    if args[0] == "set":
        if len(args) < 3:
            print("Usage: servo set <SERVO_NUM> <PWM>")
            return
        mpstate.master().mav.command_long_send(mpstate.status.target_system,
                                               mpstate.status.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                               int(args[1]), int(args[2]),
                                               0, 0, 0, 0, 0)
    if args[0] == "repeat":
        if len(args) < 5:
            print("Usage: servo repeat <SERVO_NUM> <PWM> <COUNT> <PERIOD>")
            return
        mpstate.master().mav.command_long_send(mpstate.status.target_system,
                                               mpstate.status.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0,
                                               int(args[1]), int(args[2]), int(args[3]), float(args[4]),
                                               0, 0, 0)
        
              

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.command_map['relay'] = (cmd_relay, "relay commands")
    mpstate.command_map['servo'] = (cmd_servo, "servo commands")

def unload():
    '''unload module'''
    pass
