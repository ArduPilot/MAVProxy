#!/usr/bin/env python
'''test flight for DCM noise'''

import time, math

def enum(**enums):
    return type('Enum', (), enums)

TestState = enum(INIT=1, FBWA=2, AUTO=3)

from MAVProxy.modules.lib import mp_module

class TestModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TestModule, self).__init__(mpstate, "test", "test flight")
        self.state = TestState.INIT
        print("Module test loaded")

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if self.state == TestState.INIT:
            if self.status.flightmode == "MANUAL":
                self.mpstate.functions.process_stdin("switch 4")
                self.mpstate.functions.process_stdin("rc 2 1300")
                self.mpstate.functions.process_stdin("rc 3 2000")
                self.mpstate.functions.process_stdin("module load sensors")
                self.mpstate.functions.process_stdin("watch sensors")
                self.mpstate.functions.process_stdin("wp list")
                self.state = TestState.FBWA
        if self.state == TestState.FBWA:
            if self.status.altitude > 60:
                self.mpstate.functions.process_stdin("rc 2 1500")
                self.mpstate.functions.process_stdin("auto")
                self.state = TestState.AUTO

def init(mpstate):
    '''initialise module'''
    return TestModule(mpstate)
