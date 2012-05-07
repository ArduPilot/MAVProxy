"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import wxconsole, textconsole

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
    mpstate.console = wxconsole.MessageConsole()

def unload():
    '''unload module'''
    mpstate.console = textconsole.SimpleConsole()
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    if not isinstance(mpstate.console, wxconsole.MessageConsole):
        return
    if not mpstate.console.is_alive():
        mpstate.console = textconsole.SimpleConsole()
        return
    type = msg.get_type()
    if type == 'GPS_RAW':
        if msg.fix_type == 2:
            mpstate.console.set_status('GPS', 'GPS: OK', fg='green')
        else:
            mpstate.console.set_status('GPS', 'GPS: %u' % msg.fix_type, fg='red')

