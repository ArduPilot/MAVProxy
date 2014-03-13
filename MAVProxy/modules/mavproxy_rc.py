#!/usr/bin/env python
'''rc command handling'''

import time, os, struct
from pymavlink import mavutil

class rc_state(object):
    def __init__(self):
        self.override = [ 0 ] * 8
        self.last_override = [ 0 ] * 8
        self.override_counter = 0        

def name():
    '''return module name'''
    return "rc"

def description():
    '''return module description'''
    return "rc command handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.rc_state = rc_state()
    state = mpstate.rc_state
    mpstate.command_map['rc']     = (cmd_rc, "RC input control")
    mpstate.command_map['switch'] = (cmd_switch, "flight mode switch control")
    mpstate.completions['rc'] = ['<1|2|3|4|5|6|7|8|all>']
    mpstate.completions['switch'] = ['<0|1|2|3|4|5|6>']
    if mpstate.sitl_output:
        state.override_period = mavutil.periodic_event(20)
    else:
        state.override_period = mavutil.periodic_event(1)


def idle_task():
    state = mpstate.rc_state
    if state.override_period.trigger():
        if (state.override != [ 0 ] * 8 or
            state.override != state.last_override or
            state.override_counter > 0):
            state.last_override = state.override[:]
            send_rc_override()
            if state.override_counter > 0:
                state.override_counter -= 1



def send_rc_override():
    '''send RC override packet'''
    state = mpstate.rc_state
    if mpstate.sitl_output:
        buf = struct.pack('<HHHHHHHH',
                          *state.override)
        mpstate.sitl_output.write(buf)
    else:
        mpstate.master().mav.rc_channels_override_send(mpstate.status.target_system,
                                                       mpstate.status.target_component,
                                                       *state.override)

def cmd_switch(args):
    '''handle RC switch changes'''
    state = mpstate.rc_state
    mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
    if len(args) != 1:
        print("Usage: switch <pwmvalue>")
        return
    value = int(args[0])
    if value < 0 or value > 6:
        print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
        return
    if mpstate.vehicle_type == 'copter':
        default_channel = 5
    else:
        default_channel = 8
    if mpstate.vehicle_type == 'rover':
        flite_mode_ch_parm = int(mpstate.functions.get_mav_param("MODE_CH", default_channel))
    else:
        flite_mode_ch_parm = int(mpstate.functions.get_mav_param("FLTMODE_CH", default_channel))
    state.override[flite_mode_ch_parm-1] = mapping[value]
    state.override_counter = 10
    send_rc_override()
    if value == 0:
        print("Disabled RC switch override")
    else:
        print("Set RC switch override to %u (PWM=%u channel=%u)" % (
            value, mapping[value], flite_mode_ch_parm))

def cmd_rc(args):
    '''handle RC value override'''
    state = mpstate.rc_state
    if len(args) != 2:
        print("Usage: rc <channel|all> <pwmvalue>")
        return
    value = int(args[1])
    if value == -1:
        value = 65535
    if args[0] == 'all':
        for i in range(8):
            state.override[i] = value
    else:
        channel = int(args[0])
        state.override[channel-1] = value
        if channel < 1 or channel > 8:
            print("Channel must be between 1 and 8 or 'all'")
            return
    state.override_counter = 10
    send_rc_override()

