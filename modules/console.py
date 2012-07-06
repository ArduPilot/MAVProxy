"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys, math, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import wxconsole, textconsole

class module_state(object):
    def __init__(self):
        '''Flight time vars'''
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0

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
    mpstate.console_state = module_state()
    mpstate.console = wxconsole.MessageConsole(title='Console')

    # setup some default status information
    mpstate.console.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
    mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
    mpstate.console.set_status('Vcc', 'Vcc: --', fg='red', row=0)
    mpstate.console.set_status('Radio', 'Radio: --', row=0)
    mpstate.console.set_status('Heading', 'Hdg ---/---', row=2)
    mpstate.console.set_status('Alt', 'Alt ---/---', row=2)
    mpstate.console.set_status('AirSpeed', 'AirSpeed --', row=2)
    mpstate.console.set_status('GPSSpeed', 'GPSSpeed --', row=2)
    mpstate.console.set_status('Thr', 'Thr ---', row=2)
    mpstate.console.set_status('Roll', 'Roll ---', row=2)
    mpstate.console.set_status('Pitch', 'Pitch ---', row=2)
    mpstate.console.set_status('WP', 'WP --', row=3)
    mpstate.console.set_status('WPDist', 'Distance ---', row=3)
    mpstate.console.set_status('WPBearing', 'Bearing ---', row=3)
    mpstate.console.set_status('AltError', 'AltError --', row=3)
    mpstate.console.set_status('AspdError', 'AspdError --', row=3)
    mpstate.console.set_status('FlightTime', 'FlightTime --', row=3)
       

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
    state = mpstate.console_state
    # add some status fields
    if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
        if ((msg.fix_type == 3 and master.mavlink10()) or
            (msg.fix_type == 2 and not master.mavlink10())):
            mpstate.console.set_status('GPS', 'GPS: OK', fg='green')
        else:
            mpstate.console.set_status('GPS', 'GPS: %u' % msg.fix_type, fg='red')
        if master.mavlink10():
            gps_heading = int(mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01)
        else:
            gps_heading = mpstate.status.msgs['GPS_RAW'].hdg
        mpstate.console.set_status('Heading', 'Hdg %s/%u' % (master.field('VFR_HUD', 'heading', '-'), gps_heading))
    elif type == 'VFR_HUD':
        mpstate.console.set_status('Mode', '%s' % master.flightmode, fg='blue')
        if master.mavlink10():
            alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
        else:
            alt = master.field('GPS_RAW', 'alt', 0)
        mpstate.console.set_status('Alt', 'Alt %u/%.0f' % (mpstate.status.altitude, alt))
        mpstate.console.set_status('AirSpeed', 'AirSpeed %u' % msg.airspeed)
        mpstate.console.set_status('GPSSpeed', 'GPSSpeed %u' % msg.groundspeed)
        mpstate.console.set_status('Thr', 'Thr %u' % msg.throttle)
        t = time.localtime(msg._timestamp)
        if msg.groundspeed > 3 and not state.in_air:
            state.in_air = True
            state.start_time = time.mktime(t)
        elif msg.groundspeed > 3 and state.in_air:
            state.total_time = time.mktime(t) - state.start_time
            mpstate.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(state.total_time)/60, int(state.total_time)%60))
        elif msg.groundspeed < 3 and state.in_air:
            state.in_air = False
            state.total_time = time.mktime(t) - state.start_time
            mpstate.console.set_status('FlightTime', 'FlightTime %u:%02u' % (int(state.total_time)/60, int(state.total_time)%60))
    elif type == 'ATTITUDE':
        mpstate.console.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
        mpstate.console.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))
    elif type == 'HWSTATUS':
        if msg.Vcc >= 4600 and msg.Vcc <= 5100:
            fg = 'green'
        else:
            fg = 'red'
        mpstate.console.set_status('Vcc', 'Vcc %.2f' % (msg.Vcc * 0.001), fg=fg)
    elif type == 'RADIO':
        if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
            fg = 'red'
        else:
            fg = 'black'
        mpstate.console.set_status('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)
    elif type == 'HEARTBEAT':
        for m in mpstate.mav_master:
            linkdelay = (mpstate.status.highest_usec - m.highest_usec)*1e-6            
            linkline = "Link %u " % (m.linknum+1)
            if m.linkerror:
                linkline += "down"
                fg = 'red'
            elif master.link_delayed:
                linkline += "delayed %.2fs" % linkdelay
                fg = 'yellow'
            else:
                linkline += "OK (%u pkts, %.2fs delay, %u lost)" % (m.mav_count, linkdelay, m.mav_loss)
                fg = 'darkgreen'
            mpstate.console.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
    elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
        mpstate.console.set_status('WP', 'WP %u' % msg.seq)
    elif type == 'NAV_CONTROLLER_OUTPUT':
        mpstate.console.set_status('WPDist', 'Distance %u' % msg.wp_dist)
        mpstate.console.set_status('WPBearing', 'Bearing %u' % msg.target_bearing)
        mpstate.console.set_status('AltError', 'AltError %d' % msg.alt_error)
        mpstate.console.set_status('AspdError', 'AspdError %.1f' % (msg.aspd_error*0.01))
