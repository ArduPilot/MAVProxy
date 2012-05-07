"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys

mpstate = None
print 'foo'

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import wxconsole

def name():
    '''return module name'''
    return "console"

def description():
    '''return module description'''
    return "GUI console"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.console_saved = mpstate.console
    mpstate.console = wxconsole.MessageConsole()

def unload():
    '''unload module'''
    mpstate.console = mpstate.console_saved
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    pass

