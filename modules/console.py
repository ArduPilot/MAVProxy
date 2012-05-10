"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys, math

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

    # setup some default status information
    mpstate.console.set_status('Mode', 'UNKNOWN', row=0)
    mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
    mpstate.console.set_status('Heading', 'Hdg ---/---', row=1)
    mpstate.console.set_status('Alt', 'Alt ---/---', row=1)
    mpstate.console.set_status('Speed', 'Speed --/--', row=1)
    mpstate.console.set_status('Thr', 'Thr ---', row=1)
    mpstate.console.set_status('Roll', 'Roll ---', row=1)
    mpstate.console.set_status('Pitch', 'Pitch ---', row=1)
        

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

    master = mpstate.master()
    # add some status fields
    if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
        if ((msg.fix_type == 3 and master.mavlink10()) or
            (msg.fix_type == 2 and not master.mavlink10())):
            mpstate.console.set_status('GPS', 'GPS: OK', fg='green')
        else:
            mpstate.console.set_status('GPS', 'GPS: %u' % msg.fix_type, fg='red')
        if master.mavlink10():
            gps_heading = mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01
        else:
            gps_heading = mpstate.status.msgs['GPS_RAW'].hdg
        mpstate.console.set_status('Heading', 'Hdg %s/%u' % (master.field('VFR_HUD', 'heading', '-'), gps_heading))
    if type == 'VFR_HUD':
        mpstate.console.set_status('Mode', '%s' % master.flightmode)
        mpstate.console.set_status('Alt', 'Alt %u/%u' % (mpstate.status.altitude, master.field('GPS_RAW', 'alt', '-')))
        mpstate.console.set_status('Speed', 'Speed %u/%u' % (msg.airspeed, msg.groundspeed))
        mpstate.console.set_status('Thr', 'Thr %u' % msg.throttle)
    if type == 'ATTITUDE':
        mpstate.console.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
        mpstate.console.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))

        
        
