"""
  MAVProxy console

  uses lib/console.py for display
"""

import os, sys, math, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import wxconsole, textconsole, mp_elevation, mavutil, mp_util

class module_state(object):
    def __init__(self):
        '''Flight time vars'''
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0

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
    mpstate.console.set_status('Alt', 'Alt ---', row=2)
    mpstate.console.set_status('AGL', 'AGL ---', row=2)
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
    mpstate.console.set_status('ETR', 'ETR --', row=3)

    mpstate.console.ElevationMap = mp_elevation.ElevationModel()


def unload():
    '''unload module'''
    mpstate.console = textconsole.SimpleConsole()

def estimated_time_remaining(lat, lon, wpnum, speed):
    '''estimate time remaining in mission in seconds'''
    idx = wpnum
    if wpnum >= mpstate.status.wploader.count():
        return 0
    distance = 0
    done = set()
    while idx < mpstate.status.wploader.count():
        if idx in done:
            break
        done.add(idx)
        w = mpstate.status.wploader.wp(idx)
        if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
            idx = int(w.param1)
            continue
        idx += 1
        if (w.x != 0 or w.y != 0) and w.command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                                                    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                                    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                                                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]:
            distance += mp_util.gps_distance(lat, lon, w.x, w.y)
            lat = w.x
            lon = w.y
            if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                break
    return distance / speed
        
        
        
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
        if type == "GPS_RAW":
            num_sats = master.field('GPS_STATUS', 'satellites_visible', 0)
        else:
            num_sats = msg.satellites_visible
        if ((msg.fix_type == 3 and master.mavlink10()) or
            (msg.fix_type == 2 and not master.mavlink10())):
            mpstate.console.set_status('GPS', 'GPS: OK (%u)' % num_sats, fg='green')
        else:
            mpstate.console.set_status('GPS', 'GPS: %u (%u)' % (msg.fix_type, num_sats), fg='red')
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
        if mpstate.status.wploader.count() > 0:
            wp = mpstate.status.wploader.wp(0)
            home_lat = wp.x
            home_lng = wp.y
        else:
            home_lat = master.field('HOME', 'lat') * 1.0e-7
            home_lng = master.field('HOME', 'lon') * 1.0e-7
        lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
        lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
        rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
        if mpstate.settings.basealt != 0:
            agl_alt = mpstate.settings.basealt - mpstate.console.ElevationMap.GetElevation(lat, lng)
        else:
            agl_alt = mpstate.console.ElevationMap.GetElevation(home_lat, home_lng) - mpstate.console.ElevationMap.GetElevation(lat, lng)
        agl_alt += rel_alt
        mpstate.console.set_status('AGL', 'AGL %u' % agl_alt)
        mpstate.console.set_status('Alt', 'Alt %u' % rel_alt)
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
        if msg.Vcc >= 4600 and msg.Vcc <= 5300:
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
            linkdelay = (mpstate.status.highest_msec - m.highest_msec)*1.0e-3
            linkline = "Link %u " % (m.linknum+1)
            if m.linkerror:
                linkline += "down"
                fg = 'red'
            else:
                linkline += "OK (%u pkts, %.2fs delay, %u lost)" % (m.mav_count, linkdelay, m.mav_loss)
                if linkdelay > 1:
                    fg = 'yellow'
                else:
                    fg = 'darkgreen'
            mpstate.console.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
    elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
        mpstate.console.set_status('WP', 'WP %u' % msg.seq)
        lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
        lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
        if lat != 0 and lng != 0:
            airspeed = master.field('VFR_HUD', 'airspeed', 30)
            if abs(airspeed - state.speed) > 5:
                state.speed = airspeed
            else:
                state.speed = 0.98*state.speed + 0.02*airspeed
            state.speed = max(1, state.speed)
            time_remaining = int(estimated_time_remaining(lat, lng, msg.seq, state.speed))
            mpstate.console.set_status('ETR', 'ETR %u:%02u' % (time_remaining/60, time_remaining%60))
            
    elif type == 'NAV_CONTROLLER_OUTPUT':
        mpstate.console.set_status('WPDist', 'Distance %u' % msg.wp_dist)
        mpstate.console.set_status('WPBearing', 'Bearing %u' % msg.target_bearing)
        mpstate.console.set_status('AltError', 'AltError %d' % msg.alt_error)
        mpstate.console.set_status('AspdError', 'AspdError %.1f' % (msg.aspd_error*0.01))
