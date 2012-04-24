#!/usr/bin/env python
''' simple bottle drop module'''

import time

mpstate = None
hold_pwm = 983
release_pwm = 1776
drop_channel = 5
drop_time = 2.0

class drop_state(object):
    def __init__(self):
        self.waiting = False
        self.start_drop = 0

def name():
    '''return module name'''
    return "drop"

def description():
    '''return module description'''
    return "bottle drop control"

def cmd_drop(args):
    '''drop a bottle'''
    mpstate.drop_state.start_drop = time.time()
    mpstate.drop_state.waiting = True
    mpstate.status.override[drop_channel-1] = release_pwm
    mpstate.override_period.force()
    print("started drop")

def check_drop(m):
    '''check if drop is complete'''
    if mpstate.drop_state.waiting and time.time() > mpstate.drop_state.start_drop+drop_time:
        mpstate.status.override[drop_channel-1] = 0
        mpstate.drop_state.waiting = False
        mpstate.override_period.force()
        print("drop complete")
        

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.drop_state = drop_state()
    mpstate.command_map['drop'] = (cmd_drop, "drop bottle")
    print("drop initialised")

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    if m.get_type() == 'RC_CHANNELS_RAW':
        check_drop(m)
    if m.get_type() == 'PARAM_VALUE':
        if str(m.param_id) == 'RC5_FUNCTION' and m.param_value != 1.0:
            print("DROP WARNING: RC5_FUNCTION=%u" % m.param_value)
