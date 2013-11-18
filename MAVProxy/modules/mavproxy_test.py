#!/usr/bin/env python
'''test flight for DCM noise'''

import time, math


mpstate = None

def name():
    '''return module name'''
    return "test"

def description():
    '''return module description'''
    return "test flight"

def enum(**enums):
    return type('Enum', (), enums)

TestState = enum(INIT=1, FBWA=2, AUTO=3)

class test_state(object):
    def __init__(self):
        self.state = TestState.INIT

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.test_state = test_state()
    print("Module test loaded")

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    if mpstate.test_state.state == TestState.INIT:
        if mpstate.status.flightmode == "MANUAL":
            mpstate.functions.process_stdin("switch 4")
            mpstate.functions.process_stdin("rc 2 1300")
            mpstate.functions.process_stdin("rc 3 2000")
            mpstate.functions.process_stdin("module load sensors")
            mpstate.functions.process_stdin("watch sensors")
            mpstate.functions.process_stdin("wp list")
            mpstate.test_state.state = TestState.FBWA
    if mpstate.test_state.state == TestState.FBWA:
        if mpstate.status.altitude > 60:
            mpstate.functions.process_stdin("rc 2 1500")
            mpstate.functions.process_stdin("auto")
            mpstate.test_state.state = TestState.AUTO
