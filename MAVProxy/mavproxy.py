#!/usr/bin/env python
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

'''

import sys, os, struct, math, time, socket
import fnmatch, errno, threading
import serial, Queue, select

import select

# allow running without installing
#sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))


from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import mp_settings

class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        if opts.quadcopter:
            self.rc_throttle = [ 0.0, 0.0, 0.0, 0.0 ]
        else:
            self.rc_aileron  = 0
            self.rc_elevator = 0
            self.rc_throttle = 0
            self.rc_rudder   = 0
        self.gps	 = None
        self.msgs = {}
        self.msg_count = {}
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.setup_mode = opts.setup
        self.wp_op = None
        self.wp_save_filename = None
        self.wploader = mavwp.MAVWPLoader()
        self.fenceloader = mavwp.MAVFenceLoader()
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.mav_error = 0
        self.target_system = 1
        self.target_component = 1
        self.rallyloader = mavwp.MAVRallyLoader(self.target_system, self.target_component)
        self.speech = None
        self.altitude = 0
        self.last_altitude_announce = 0.0
        self.last_distance_announce = 0.0
        self.last_battery_announce = 0
        self.last_avionics_battery_announce = 0
        self.battery_level = -1
        self.avionics_battery_level = -1
        self.last_waypoint = 0
        self.exit = False
        self.override = [ 0 ] * 8
        self.last_override = [ 0 ] * 8
        self.override_counter = 0
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        self.logdir = None
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = False
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.highest_msec = 0
        self.fence_enabled = False
        self.last_fence_breach = 0
        self.last_fence_status = 0
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        self.watch = None
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        self.last_seq = 0
        self.fetch_one = 0
        self.armed = False

    def show(self, f, pattern=None):
        '''write status to status.txt'''
        if pattern is None:
            f.write('Counters: ')
            for c in self.counters:
                f.write('%s:%s ' % (c, self.counters[c]))
            f.write('\n')
            f.write('MAV Errors: %u\n' % self.mav_error)
            f.write(str(self.gps)+'\n')
        for m in sorted(self.msgs.keys()):
            if pattern is not None and not fnmatch.fnmatch(str(m).upper(), pattern.upper()):
                continue
            f.write("%u: %s\n" % (self.msg_count[m], str(self.msgs[m])))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()

class MAVFunctions(object):
    pass

class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.map = None
        self.map_functions = {}
        self.settings = mp_settings.MPSettings(
            [ ('link', int, 1),
              ('altreadout', int, 10),
              ('distreadout', int, 200),
              ('battreadout', int, 0),
              ('heartbeat', int, 1),
              ('numcells', int, 1),
              ('speech', int, 0),
              ('mavfwd', int, 1),
              ('mavfwd_rate', int, 0),
              ('streamrate', int, 4),
              ('streamrate2', int, 4),
              ('heartbeatreport', int, 1),
              ('moddebug', int, 0),
              ('rc1mul', int, 1),
              ('rc2mul', int, 1),
              ('rc4mul', int, 1),
              ('shownoise', int, 1),
              ('basealt', int, 0),
              ('wpalt', int, 100),
              ('parambatch', int, 10)]
            )
        self.status = MPStatus()

        # master mavlink device
        self.mav_master = None

        # mavlink outputs
        self.mav_outputs = []

        # SITL output
        self.sitl_output = None

        self.mav_param = mavparm.MAVParmDict()
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.modules = []
        self.functions = MAVFunctions()
        self.functions.say = say
        self.functions.process_stdin = process_stdin
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}

    def master(self):
        '''return the currently chosen mavlink master object'''
        if self.settings.link > len(self.mav_master):
            self.settings.link = 1

        # try to use one with no link error
        if not self.mav_master[self.settings.link-1].linkerror:
            return self.mav_master[self.settings.link-1]
        for m in self.mav_master:
            if not m.linkerror:
                return m
        return self.mav_master[self.settings.link-1]


def get_usec():
    '''time since 1970 in microseconds'''
    return int(time.time() * 1.0e6)

class rline(object):
    '''async readline abstraction'''
    def __init__(self, prompt):
        import threading
        self.prompt = prompt
        self.line = None
        try:
            import readline
        except Exception:
            pass

    def set_prompt(self, prompt):
        if prompt != self.prompt:
            self.prompt = prompt
            sys.stdout.write(prompt)

def say(text, priority='important'):
    '''speak some text'''
    ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
    mpstate.console.writeln(text)
    if mpstate.settings.speech:
        import speechd
        mpstate.status.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
        mpstate.status.speech.set_output_module('festival')
        mpstate.status.speech.set_language('en')
        mpstate.status.speech.set_priority(priority)
        mpstate.status.speech.set_punctuation(speechd.PunctuationMode.SOME)
        mpstate.status.speech.speak(text)
        mpstate.status.speech.close()

def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)


def send_rc_override():
    '''send RC override packet'''
    if mpstate.sitl_output:
        buf = struct.pack('<HHHHHHHH',
                          *mpstate.status.override)
        mpstate.sitl_output.write(buf)
    else:
        mpstate.master().mav.rc_channels_override_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         *mpstate.status.override)

def cmd_switch(args):
    '''handle RC switch changes'''
    mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
    if len(args) != 1:
        print("Usage: switch <pwmvalue>")
        return
    value = int(args[0])
    if value < 0 or value > 6:
        print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
        return
    if opts.quadcopter:
        default_channel = 5
    else:
        default_channel = 8
    flite_mode_ch_parm = int(get_mav_param("FLTMODE_CH", default_channel))
    mpstate.status.override[flite_mode_ch_parm-1] = mapping[value]
    mpstate.status.override_counter = 10
    send_rc_override()
    if value == 0:
        print("Disabled RC switch override")
    else:
        print("Set RC switch override to %u (PWM=%u channel=%u)" % (
            value, mapping[value], flite_mode_ch_parm))

def cmd_rc(args):
    '''handle RC value override'''
    if len(args) != 2:
        print("Usage: rc <channel|all> <pwmvalue>")
        return
    value   = int(args[1])
    if value == -1:
        value = 65535
    if args[0] == 'all':
        for i in range(8):
            mpstate.status.override[i] = value
    else:
        channel = int(args[0])
        mpstate.status.override[channel-1] = value
        if channel < 1 or channel > 8:
            print("Channel must be between 1 and 8 or 'all'")
            return
    mpstate.status.override_counter = 10
    send_rc_override()

def cmd_loiter(args):
    '''set LOITER mode'''
    mpstate.master().set_mode_loiter()

def cmd_auto(args):
    '''set AUTO mode'''
    mpstate.master().set_mode_auto()

def cmd_ground(args):
    '''do a ground start mode'''
    mpstate.master().calibrate_imu()

def cmd_level(args):
    '''run a accel level'''
    mpstate.master().calibrate_level()

def cmd_mode(args):
    '''set arbitrary mode'''
    mode_mapping = mpstate.master().mode_mapping()
    if mode_mapping is None:
        print('No mode mapping available')
        return
    if len(args) != 1:
        print('Available modes: ', mode_mapping.keys())
        return
    mode = args[0].upper()
    if mode not in mode_mapping:
        print('Unknown mode %s: ' % mode)
        return
    mpstate.master().set_mode(mode_mapping[mode])

def cmd_accelcal(args):
    '''do a full 3D accel calibration'''
    mav = mpstate.master()
    # ack the APM to begin 3D calibration of accelerometers
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                              0, 0, 0, 0, 1, 0, 0)
    count = 0
    # we expect 6 messages and acks
    while count < 6:
        m = mav.recv_match(type='STATUSTEXT', blocking=True)
        text = str(m.text)
        if not text.startswith('Place APM'):
            continue
        # wait for user to hit enter
        mpstate.rl.line = None
        while mpstate.rl.line is None:
            time.sleep(0.1)
        mpstate.rl.line = None
        count += 1
        # tell the APM that we've done as requested
        mav.mav.command_ack_send(count, 1)


def cmd_reboot(args):
    '''reboot autopilot'''
    mpstate.master().reboot_autopilot()

def cmd_calpressure(args):
    '''calibrate pressure sensors'''
    mpstate.master().calibrate_pressure()

def cmd_rtl(args):
    '''set RTL mode'''
    mpstate.master().set_mode_rtl()

def cmd_manual(args):
    '''set MANUAL mode'''
    mpstate.master().set_mode_manual()

def cmd_servo(args):
    '''set a servo'''
    if len(args) != 2:
        print("Usage: servo <channel> <pwmvalue>")
        return
    channel = int(args[0])
    value   = int(args[1])
    mpstate.master().set_servo(channel, value)

def cmd_fbwa(args):
    '''set FBWA mode'''
    mpstate.master().set_mode_fbwa()

def cmd_guided(args):
    '''set GUIDED target'''
    if len(args) != 1:
        print("Usage: guided ALTITUDE")
        return
    try:
        latlon = mpstate.map_state.click_position
    except Exception:
        print("No map available")
        return
    if latlon is None:
        print("No map click position available")
        return        
    altitude = int(args[0])
    print("Guided %s %d" % (str(latlon), altitude))
    mpstate.master().mav.mission_item_send(mpstate.status.target_system,
                                           mpstate.status.target_component,
                                           0,
                                           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                           2, 0, 0, 0, 0, 0,
                                           latlon[0], latlon[1], altitude)

def process_waypoint_request(m, master):
    '''process a waypoint request from the master'''
    if (not mpstate.status.loading_waypoints or
        time.time() > mpstate.status.loading_waypoint_lasttime + 10.0):
        mpstate.status.loading_waypoints = False
        mpstate.console.error("not loading waypoints")
        return
    if m.seq >= mpstate.status.wploader.count():
        mpstate.console.error("Request for bad waypoint %u (max %u)" % (m.seq, mpstate.status.wploader.count()))
        return
    wp = mpstate.status.wploader.wp(m.seq)
    wp.target_system = mpstate.status.target_system
    wp.target_component = mpstate.status.target_component
    master.mav.send(mpstate.status.wploader.wp(m.seq))
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.console.writeln("Sent waypoint %u : %s" % (m.seq, mpstate.status.wploader.wp(m.seq)))
    if m.seq == mpstate.status.wploader.count() - 1:
        mpstate.status.loading_waypoints = False
        mpstate.console.writeln("Sent all %u waypoints" % mpstate.status.wploader.count())

def load_waypoints(filename):
    '''load waypoints from a file'''
    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component
    try:
        mpstate.status.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u waypoints from %s" % (mpstate.status.wploader.count(), filename))

    mpstate.master().waypoint_clear_all_send()
    if mpstate.status.wploader.count() == 0:
        return

    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(mpstate.status.wploader.count())

def update_waypoints(filename, wpnum):
    '''update waypoints from a file'''
    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component
    try:
        mpstate.status.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    if mpstate.status.wploader.count() == 0:
        print("No waypoints found in %s" % filename)
        return
    if wpnum == -1:
        print("Loaded %u updated waypoints from %s" % (mpstate.status.wploader.count(), filename))
    elif wpnum >= mpstate.status.wploader.count():
        print("Invalid waypoint number %u" % wpnum)
        return
    else:
        print("Loaded updated waypoint %u from %s" % (wpnum, filename))

    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    if wpnum == -1:
        start = 0
        end = mpstate.status.wploader.count()-1
    else:
        start = wpnum
        end = wpnum
    mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         start, end)

def save_waypoints(filename):
    '''save waypoints to a file'''
    try:
        mpstate.status.wploader.save(filename)
    except Exception, msg:
        print("Failed to save %s - %s" % (filename, msg))
        return
    print("Saved %u waypoints to %s" % (mpstate.status.wploader.count(), filename))

def wp_draw_callback(points):
    '''callback from drawing waypoints'''
    if len(points) < 3:
        return
    from MAVProxy.modules.lib import mp_util
    home = mpstate.status.wploader.wp(0)
    mpstate.status.wploader.clear()
    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component
    mpstate.status.wploader.add(home)
    for p in points:
        mpstate.status.wploader.add_latlonalt(p[0], p[1], mpstate.settings.wpalt)
    mpstate.master().waypoint_clear_all_send()
    if mpstate.status.wploader.count() == 0:
        return
    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(mpstate.status.wploader.count())

def wp_loop():
    '''close the loop on a mission'''
    loader = mpstate.status.wploader
    if loader.count() < 2:
        print("Not enough waypoints (%u)" % loader.count())
        return
    wp = loader.wp(loader.count()-2)
    if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
        print("Mission is already looped")
        return
    wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_DO_JUMP,
                                                      0, 1, 1, -1, 0, 0, 0, 0, 0)
    loader.add(wp)
    loader.add(loader.wp(1))
    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(mpstate.status.wploader.count())
    print("Closed loop on mission")

def set_home_location():
    '''set home location from last map click'''
    try:
        latlon = mpstate.map_state.click_position
    except Exception:
        print("No map available")
        return
    lat = float(latlon[0])
    lon = float(latlon[1])
    if mpstate.status.wploader.count() == 0:
        mpstate.status.wploader.add_latlonalt(lat, lon, 0)
    w = mpstate.status.wploader.wp(0)
    w.x = lat
    w.y = lon
    mpstate.status.wploader.set(w, 0)
    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         0, 0)
    

def cmd_wp(args):
    '''waypoint commands'''
    if len(args) < 1:
        print("usage: wp <list|load|update|save|set|clear|loop>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: wp load <filename>")
            return
        load_waypoints(args[1])
    elif args[0] == "update":
        if len(args) < 2:
            print("usage: wp update <filename> <wpnum>")
            return
        if len(args) == 3:
            wpnum = int(args[2])
        else:
            wpnum = -1
        update_waypoints(args[1], wpnum)
    elif args[0] == "list":
        mpstate.status.wp_op = "list"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: wp save <filename>")
            return
        mpstate.status.wp_save_filename = args[1]
        mpstate.status.wp_op = "save"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "savelocal":
        if len(args) != 2:
            print("usage: wp savelocal <filename>")
            return
        mpstate.status.wploader.save(args[1])
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: wp show <filename>")
            return
        mpstate.status.wploader.load(args[1])
    elif args[0] == "set":
        if len(args) != 2:
            print("usage: wp set <wpindex>")
            return
        mpstate.master().waypoint_set_current_send(int(args[1]))
    elif args[0] == "clear":
        mpstate.master().waypoint_clear_all_send()
    elif args[0] == "draw":
        if not 'draw_lines' in mpstate.map_functions:
            print("No map drawing available")
            return        
        if mpstate.status.wploader.count() == 0:
            print("Need home location - refresh waypoints")
            return
        mpstate.map_functions['draw_lines'](wp_draw_callback)
        print("Drawing waypoints on map")
    elif args[0] == "sethome":
        set_home_location()        
    elif args[0] == "loop":
        wp_loop()        
    else:
        print("Usage: wp <list|load|save|set|show|clear|draw|loop>")


def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print("usage: script <filename>")
        return

    run_script(args[0])


def fetch_fence_point(i):
    '''fetch one fence point'''
    mpstate.master().mav.fence_fetch_point_send(mpstate.status.target_system,
                                                mpstate.status.target_component, i)
    tstart = time.time()
    p = None
    while time.time() - tstart < 1:
        p = mpstate.master().recv_match(type='FENCE_POINT', blocking=False)
        if p is not None:
            break
        time.sleep(0.1)
        continue
    if p is None:
        mpstate.console.error("Failed to fetch point %u" % i)
        return None
    return p



def send_fence():
    '''send fence points from fenceloader'''
    # must disable geo-fencing when loading
    action = get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
    param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
    param_set('FENCE_TOTAL', mpstate.status.fenceloader.count())
    for i in range(mpstate.status.fenceloader.count()):
        p = mpstate.status.fenceloader.point(i)
        mpstate.master().mav.send(p)
        p2 = fetch_fence_point(i)
        if p2 is None:
            param_set('FENCE_ACTION', action)
            return
        if (p.idx != p2.idx or
            abs(p.lat - p2.lat) >= 0.00003 or
            abs(p.lng - p2.lng) >= 0.00003):
            print("Failed to send fence point %u" % i)
            param_set('FENCE_ACTION', action)
            return
    param_set('FENCE_ACTION', action)

def load_fence(filename):
    '''load fence points from a file'''
    try:
        mpstate.status.fenceloader.target_system = mpstate.status.target_system
        mpstate.status.fenceloader.target_component = mpstate.status.target_component
        mpstate.status.fenceloader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u geo-fence points from %s" % (mpstate.status.fenceloader.count(), filename))
    send_fence()


def list_fence(filename):
    '''list fence points, optionally saving to a file'''

    mpstate.status.fenceloader.clear()
    count = get_mav_param('FENCE_TOTAL', 0)
    if count == 0:
        print("No geo-fence points")
        return
    for i in range(int(count)):
        p = fetch_fence_point(i)
        if p is None:
            return
        mpstate.status.fenceloader.add(p)

    if filename is not None:
        try:
            mpstate.status.fenceloader.save(filename)
        except Exception, msg:
            print("Unable to save %s - %s" % (filename, msg))
            return
        print("Saved %u geo-fence points to %s" % (mpstate.status.fenceloader.count(), filename))
    else:
        for i in range(mpstate.status.fenceloader.count()):
            p = mpstate.status.fenceloader.point(i)
            mpstate.console.writeln("lat=%f lng=%f" % (p.lat, p.lng))
    if mpstate.status.logdir != None:
        fencetxt = os.path.join(mpstate.status.logdir, 'fence.txt')
        mpstate.status.fenceloader.save(fencetxt)
        print("Saved fence to %s" % fencetxt)


def fence_draw_callback(points):
    '''callback from drawing a fence'''
    if len(points) < 3:
        return
    from MAVProxy.modules.lib import mp_util
    mpstate.status.fenceloader.clear()
    mpstate.status.fenceloader.target_system = mpstate.status.target_system
    mpstate.status.fenceloader.target_component = mpstate.status.target_component
    bounds = mp_util.polygon_bounds(points)
    (lat, lon, width, height) = bounds
    center = (lat+width/2, lon+height/2)
    mpstate.status.fenceloader.add_latlon(center[0], center[1])
    for p in points:
        mpstate.status.fenceloader.add_latlon(p[0], p[1])
    # close it
    mpstate.status.fenceloader.add_latlon(points[0][0], points[0][1])
    send_fence()

def cmd_fence(args):
    '''geo-fence commands'''
    if len(args) < 1:
        print("usage: fence <list|load|save|clear|draw>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: fence load <filename>")
            return
        load_fence(args[1])
    elif args[0] == "list":
        list_fence(None)
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: fence save <filename>")
            return
        list_fence(args[1])
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: fence show <filename>")
            return
        mpstate.status.fenceloader.load(args[1])
    elif args[0] == "draw":
        if not 'draw_lines' in mpstate.map_functions:
            print("No map drawing available")
            return        
        mpstate.map_functions['draw_lines'](fence_draw_callback)
        print("Drawing fence on map")
    elif args[0] == "clear":
        param_set('FENCE_TOTAL', 0)
    else:
        print("Usage: fence <list|load|save|show|clear|draw>")


def fetch_rally_point(i):
    '''fetch one rally point'''
    mpstate.master().mav.rally_fetch_point_send(mpstate.status.target_system,
                                                mpstate.status.target_component, i)
    tstart = time.time()
    p = None
    while time.time() - tstart < 1:
        p = mpstate.master().recv_match(type='RALLY_POINT', blocking=False)
        if p is not None:
            break
        time.sleep(0.1)
        continue
    if p is None:
        mpstate.console.error("Failed to fetch rally point %u" % i)
        return None
    return p

def send_rally_points():
    '''send rally points from fenceloader'''
    param_set('RALLY_TOTAL', mpstate.status.rallyloader.rally_count())
    for i in range(mpstate.status.rallyloader.rally_count()):
        p = mpstate.status.rallyloader.rally_point(i)
        mpstate.master().mav.send(p)
        
        #Don't know why this check was being done, but not discounting yet...
        #Will remove before committing if I don't discover why.
        #p2 = fetch_rally_point(i)
        #
        #if (p.idx != p2.idx or
        #    abs(p.lat - p2.lat) >= 0.00003 or
        #    abs(p.lng - p2.lng) >= 0.00003):
        #    print("Failed to send fence point %u" % i)
        #    param_set('FENCE_ACTION', action)
        #    return

    #TODO rally land points

def list_rally_points():
    mpstate.status.rallyloader.clear()
    rally_count = get_mav_param('RALLY_TOTAL', 0)
    if rally_count == 0:
        print("No rally points")
        return
    for i in range(int(rally_count)):
        p = fetch_rally_point(i)
        if p is None:
            return
        mpstate.status.rallyloader.append_rally_point(p)

    #TODO: rally_land points

    for i in range(mpstate.status.rallyloader.rally_count()):
        p = mpstate.status.rallyloader.rally_point(i)
        mpstate.console.writeln("lat=%f lng=%f alt=%f break_alt=%f land_dir=%f" % (p.lat * 1e-7, p.lng * 1e-7, p.alt, p.break_alt, p.land_dir))

    #TODO: rally_land points

def cmd_rally(args):
    '''rally point commands'''
    #TODO: add_land arg
    if(len(args) < 1):
        print("Usage: rally <list|load|save|add|clear>")
        return

    elif(args[0] == "add"):
        if (len(args) < 2):
            print("Usage: rally add ALT <BREAK_ALT> <LAND_HDG>")
            return

        if (mpstate.status.rallyloader.rally_count() > 4):
            print ("Only 5 rally points possible per flight plan.")
            return

        try:
            latlon = mpstate.map_state.click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return

        alt = float(args[1]);
        break_alt = 0.0
        land_hdg = 0.0;
        if (len(args) > 2):
            break_alt = float(args[2]);
        if (len(args) > 3):
            land_hdg = float(args[3]);

        mpstate.status.rallyloader.create_and_append_rally_point(latlon[0] * 1e7, latlon[1] * 1e7, alt, break_alt, land_hdg, 0)

        send_rally_points();

        print("Added Rally point at %s %f" % (str(latlon), alt))
    
    elif args[0] == "clear":
        mpstate.status.rallyloader.clear()
        param_set('RALLY_TOTAL', 0)

    elif(args[0] == "list"):
        list_rally_points()

    elif(args[0] == "load"):
        if (len(args) < 2):
            print("Usage: rally load filename")

        try:
            mpstate.status.rallyloader.load(args[1])
        except Exception, msg:
            print("Unable to load %s - %s" % (args[1], msg))
            return
    
        send_rally_points()

        print("Loaded %u rally points from %s" % (mpstate.status.rallyloader.rally_count(), args[1]))

    elif(args[0] == "save"):
        if (len(args) < 2):
            print("Usage: rally save filename");

        mpstate.status.rallyloader.save(args[1]);

        print "Saved rally file ", args[1];

    else:
        #TODO: add_land arg
        print("Usage: rally <list|load|save|add|clear>")

def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)


def cmd_param(args):
    '''control parameters'''
    param_wildcard = "*"
    if len(args) < 1:
        print("usage: param <fetch|edit|set|show|diff>")
        return
    if args[0] == "fetch":
        if len(args) == 1:
            mpstate.master().param_fetch_all()
            mpstate.mav_param_set = set()
            print("Requested parameter list")
        else:
            for p in mpstate.mav_param.keys():
                if fnmatch.fnmatch(p, args[1].upper()):
                    mpstate.master().param_fetch_one(p)
                    mpstate.status.fetch_one += 1
                    print("Requested parameter %s" % p)
    elif args[0] == "save":
        if len(args) < 2:
            print("usage: param save <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.save(args[1], param_wildcard, verbose=True)
    elif args[0] == "diff":
        if len(args) < 2:
            if opts.aircraft is not None:
                filename = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(mpstate.status.logdir))), 'mavdefault.txt')
            else:
                print("Usage: param diff <filename>")
        else:
            filename = args[1]
        if len(args) == 3:
            wildcard = args[2]
        else:
            wildcard = '*'
        mpstate.mav_param.diff(filename, wildcard=wildcard)
    elif args[0] == "set":
        if len(args) != 3:
            print("Usage: param set PARMNAME VALUE")
            return
        param = args[1]
        value = args[2]
        if not param.upper() in mpstate.mav_param:
            print("Unable to find parameter '%s'" % param)
            return
        param_set(param, value)
    elif args[0] == "load":
        if len(args) < 2:
            print("Usage: param load <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.load(args[1], param_wildcard, mpstate.master())
    elif args[0] == "show":
        if len(args) > 1:
            pattern = args[1]
        else:
            pattern = "*"
        mpstate.mav_param.show(pattern)
    else:
        print("Unknown subcommand '%s' (try 'fetch', 'save', 'set', 'show', 'load')" % args[0])

def cmd_set(args):
    '''control mavproxy options'''
    if len(args) == 0:
        mpstate.settings.show_all()
        return

    if getattr(mpstate.settings, args[0], None) is None:
        print("Unknown setting '%s'" % args[0])
        return
    if len(args) == 1:
        mpstate.settings.show(args[0])
    else:
        mpstate.settings.set(args[0], args[1])

tune_options = {
    'None':             '0',
    'StabRollPitchkP':  '1',
    'RateRollPitchkP':  '4',
    'RateRollPitchkI':  '6',
    'RateRollPitchkD':  '21',
    'StabYawkP':        '3',
    'RateYawkP':        '6',
    'RateYawkD':        '26',
    'AltitudeHoldkP':   '14',
    'ThrottleRatekP':   '7',
    'ThrottleRatekD':   '37',
    'ThrottleAccelkP':  '34',
    'ThrottleAccelkI':  '35',
    'ThrottleAccelkD':  '36',
    'LoiterPoskP':      '12',
    'LoiterRatekP':     '22',
    'LoiterRatekI':     '28',
    'LoiterRatekD':     '29',
    'WPSpeed':          '10',
    'AcroRollPitch kP': '25',
    'AcroYawkP':        '40',
    'RelayOnOff':       '9',
    'HeliExtGyro':      '13',
    'OFLoiterkP':       '17',
    'OFLoiterkI':       '18',
    'OFLoiterkD':       '19',
    'AHRSYawkP':        '30',
    'AHRSkP':           '31',
    'INAV_TC':          '32',
    'Declination':      '38',
    'CircleRate':       '39',
    'SonarGain':       '41',
}

def tune_show():
    opt_num = str(int(get_mav_param('TUNE')))
    option = None
    for k in tune_options.keys():
        if opt_num == tune_options[k]:
            option = k
            break
    else:
        print("TUNE is currently set to unknown value " + opt_num)
        return
    low = get_mav_param('TUNE_LOW')
    high = get_mav_param('TUNE_HIGH')
    print("TUNE is currently set to %s LOW=%f HIGH=%f" % (option, low/1000, high/1000))

def tune_option_validate(option):
    for k in tune_options:
        if option.upper() == k.upper():
            return k
    return None

# TODO: Check/show the limits of LOW and HIGH
def cmd_tuneopt(args):
    '''Select option for Tune Pot on Channel 6 (quadcopter only)'''
    usage = "usage: tuneopt <set|show|reset|list>"
    if not opts.quadcopter or 'TUNE' not in mpstate.mav_param:
        print("This command is only available for quadcopter")
        return
    if len(args) < 1:
        print(usage)
        return
    if args[0].lower() == 'reset':
        param_set('TUNE', '0')
    elif args[0].lower() == 'set':
        if len(args) < 4:
            print('Usage: tuneopt set OPTION LOW HIGH')
            return
        option = tune_option_validate(args[1])
        if not option:
            print('Invalid Tune option: ' + args[1])
            return
        low = args[2]
        high = args[3]
        param_set('TUNE', tune_options[option])
        param_set('TUNE_LOW', float(low) * 1000)
        param_set('TUNE_HIGH', float(high) * 1000)
    elif args[0].lower() == 'show':
        tune_show()
    elif args[0].lower() == 'list':
        print("Options available:")
        for s in sorted(tune_options.keys()):
            print('  ' + s)
    else:
        print(usage)

def cmd_status(args):
    '''show status'''
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern)

def cmd_bat(args):
    '''show battery levels'''
    print("Flight battery:   %u%%" % mpstate.status.battery_level)
    print("Avionics battery: %u%%" % mpstate.status.avionics_battery_level)

def cmd_alt(args):
    '''show altitude'''
    print("Altitude:  %.1f" % mpstate.status.altitude)


def cmd_up(args):
    '''adjust TRIM_PITCH_CD up by 5 degrees'''
    if len(args) == 0:
        adjust = 5.0
    else:
        adjust = float(args[0])
    old_trim = get_mav_param('TRIM_PITCH_CD', None)
    if old_trim is None:
        print("Existing trim value unknown!")
        return
    new_trim = int(old_trim + (adjust*100))
    if math.fabs(new_trim - old_trim) > 1000:
        print("Adjustment by %d too large (from %d to %d)" % (adjust*100, old_trim, new_trim))
        return
    print("Adjusting TRIM_PITCH_CD from %d to %d" % (old_trim, new_trim))
    param_set('TRIM_PITCH_CD', new_trim)

aux_options = {
    "Nothing":"0",
    "Flip":"2",
    "SimpleMode":"3",
    "RTL":"4",
    "SaveTrim":"5",
    "SaveWP":"7",
    "MultiMode":"8",
    "CameraTrigger":"9",
    "Sonar":"10",
    "Fence":"11",
    "ResetYaw":"12",
    "SuperSimpleMode":"13",
    "AcroTrainer":"14",
    "Auto":"16",
    "AutoTune":"17",
    "Land":"18"
}

def aux_show(channel):
    param = "CH%s_OPT" % channel
    opt_num = str(int(get_mav_param(param)))
    option = None
    for k in aux_options.keys():
        if opt_num == aux_options[k]:
            option = k
            break
    else:
        print("AUX Channel is currently set to unknown value " + opt_num)
        return
    print("AUX Channel is currently set to " + option)

def aux_option_validate(option):
    for k in aux_options:
        if option.upper() == k.upper():
            return k
    return None

def cmd_auxopt(args):
    '''handle AUX switches (CH7, CH8) settings'''
    if not opts.quadcopter or 'CH7_OPT' not in mpstate.mav_param:
        print("This command is only available for quadcopter")
        return
    if len(args) == 0 or args[0] not in ('set', 'show', 'reset', 'list'):
        print("Usage: auxopt set|show|reset|list")
        return
    if args[0] == 'list':
        print("Options available:")
        for s in sorted(aux_options.keys()):
            print('  ' + s)
    elif args[0] == 'show':
        if len(args) > 2 and args[1] not in ['7', '8', 'all']:
            print("Usage: auxopt show [7|8|all]")
            return
        if len(args) < 2 or args[1] == 'all':
            aux_show('7')
            aux_show('8')
            return
        aux_show(args[1])
    elif args[0] == 'reset':
        if len(args) < 2 or args[1] not in ['7', '8', 'all']:
            print("Usage: auxopt reset 7|8|all")
            return
        if args[1] == 'all':
            param_set('CH7_OPT', '0')
            param_set('CH8_OPT', '0')
            return
        param = "CH%s_OPT" % args[1]
        param_set(param, '0')
    elif args[0] == 'set':
        if len(args) < 3 or args[1] not in ['7', '8']:
            print("Usage: auxopt set 7|8 OPTION")
            return
        option = aux_option_validate(args[2])
        if not option:
            print("Invalid option " + args[2])
            return
        param = "CH%s_OPT" % args[1]
        param_set(param, aux_options[option])
    else:
        print("Usage: auxopt set|show|list")

def cmd_setup(args):
    mpstate.status.setup_mode = True
    mpstate.rl.set_prompt("")


def cmd_reset(args):
    print("Resetting master")
    mpstate.master().reset()

def cmd_link(args):
    for master in mpstate.mav_master:
        linkdelay = (mpstate.status.highest_msec - master.highest_msec)*1.0e-3
        if master.linkerror:
            print("link %u down" % (master.linknum+1))
        else:
            print("link %u OK (%u packets, %.2fs delay, %u lost, %.1f%% loss)" % (master.linknum+1,
                                                                                  mpstate.status.counters['MasterIn'][master.linknum],
                                                                                  linkdelay,
                                                                                  master.mav_loss,
                                                                                  master.packet_loss()))

def cmd_watch(args):
    '''watch a mavlink packet pattern'''
    if len(args) == 0:
        mpstate.status.watch = None
        return
    mpstate.status.watch = args[0]
    print("Watching %s" % mpstate.status.watch)

def cmd_module(args):
    '''module commands'''
    usage = "usage: module <list|load|reload|unload>"
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        for m in mpstate.modules:
            print("%s: %s" % (m.name(), m.description()))
    elif args[0] == "load":
        if len(args) < 2:
            print("usage: module load <name>")
            return
        modname = args[1]
        modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
        for modpath in modpaths:
            try:
                m = import_package(modpath)
                if m in mpstate.modules:
                    raise RuntimeError("module %s already loaded" % (modname,))
                m.init(mpstate)
                mpstate.modules.append(m)
                print("Loaded module %s" % (modname,))
                return
            except ImportError, msg:
                ex = msg
        print("Failed to load module: %s" % ex)
    elif args[0] == "reload":
        if len(args) < 2:
            print("usage: module reload <name>")
            return
        modname = args[1]
        for m in mpstate.modules:
            if m.name() == modname:
                try:
                    m.unload()
                except Exception:
                    pass
                reload(m)
                m.init(mpstate)
                print("Reloaded module %s" % modname)
                return
        print("Unable to find module %s" % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print("usage: module unload <name>")
            return
        modname = os.path.basename(args[1])
        for m in mpstate.modules:
            if m.name() == modname:
                m.unload()
                mpstate.modules.remove(m)
                print("Unloaded module %s" % modname)
                return
        print("Unable to find module %s" % modname)
    else:
        print(usage)


def cmd_alias(args):
    '''alias commands'''
    usage = "usage: alias <add|remove|list>"
    if len(args) < 1 or args[0] == "list":
        if len(args) >= 2:
            wildcard = args[1].upper()
        else:
            wildcard = '*'
        for a in sorted(mpstate.aliases.keys()):
            if fnmatch.fnmatch(a.upper(), wildcard):
                print("%-15s : %s" % (a, mpstate.aliases[a]))
    elif args[0] == "add":
        if len(args) < 3:
            print(usage)
            return
        a = args[1]
        mpstate.aliases[a] = ' '.join(args[2:])
    elif args[0] == "remove":
        if len(args) != 2:
            print(usage)
            return
        a = args[1]
        if a in mpstate.aliases:
            mpstate.aliases.pop(a)
        else:
            print("no alias %s" % a)
    else:
        print(usage)
        return

def cmd_arm(args):
    '''arm commands'''
    usage = "usage: arm <check|uncheck|list|throttle>"
    checkables = "<all|baro|compass|gps|ins|params|rc|voltage|battery>"

    if len(args) > 0:
        if args[0] == "check":
            if (len(args) < 2):
                print "usage: arm check", checkables
                return
            
            arming_mask = int(mpstate.mav_param["ARMING_CHECK"])

            if (args[1] == "all"):
                arming_mask |= 0x0001
            elif (args[1] == "baro"):
                arming_mask |= 0x0002
            elif (args[1] == "compass"):
                arming_mask |= 0x0004
            elif (args[1] == "gps"):
                arming_mask |= 0x0008
            elif (args[1] == "ins"):
                arming_mask |= 0x0010
            elif (args[1] == "params"):
                arming_mask |= 0x0020
            elif (args[1] == "rc"):
                arming_mask |= 0x0030
            elif (args[1] == "voltage"):
                arming_mask |= 0x0040
            elif (args[1] == "battery"):
                arming_mask |= 0x0100
            else:
                print "unrecognized arm check:", args[1]
                return

            param_set("ARMING_CHECK", arming_mask)
        elif args[0] == "uncheck":
            if (len(args) < 2):
                print "usage: arm uncheck", checkables
                return

            arming_mask = int(mpstate.mav_param["ARMING_CHECK"])

            if (args[1] == "all"):
                arming_mask ^= 0x0001
            elif (args[1] == "baro"):
                arming_mask ^= 0x0002
            elif (args[1] == "compass"):
                arming_mask ^= 0x0004
            elif (args[1] == "gps"):
                arming_mask ^= 0x0008
            elif (args[1] == "ins"):
                arming_mask ^= 0x0010
            elif (args[1] == "params"):
                arming_mask ^= 0x0020
            elif (args[1] == "rc"):
                arming_mask ^= 0x0040
            elif (args[1] == "voltage"):
                arming_mask ^= 0x0080
            elif (args[1] == "battery"):
                arming_mask ^= 0x0100
            else:
                print "unrecognized arm check:", args[1]
                return

            param_set("ARMING_CHECK", arming_mask)

        elif args[0] == "list":
            arming_mask = int(mpstate.mav_param["ARMING_CHECK"])
            if (arming_mask & 0x0001 != 0):
                print "ALL"
            if (arming_mask & 0x0002 != 0):
                print "BARO"
            if (arming_mask & 0x0004 != 0):
                print "COMPASS"
            if (arming_mask & 0x0008 != 0):
                print "GPS"
            if (arming_mask & 0x0010 != 0):
                print "INS"
            if (arming_mask & 0x0020 != 0):
                print "PARAMETERS"
            if (arming_mask & 0x0040 != 0):
                print "RC"
            if (arming_mask & 0x0080 != 0):
                print "VOLTAGE"
            if (arming_mask & 0x0100 != 0):
                print "BATTERY"

        elif args[0] == "throttle":
            mpstate.master().arducopter_arm()
        else:
            print(usage)
            return
    else:
        print(usage)
        return


def cmd_time(args):
  '''show autopilot time'''
  tusec = mpstate.master().field('SYSTEM_TIME', 'time_unix_usec', 0)
  if tusec == 0:
      print("No SYSTEM_TIME time available")
      return
  print("%s (%s)\n" % (time.ctime(tusec * 1.0e-6), time.ctime()))


def cmd_disarm(args):
  '''disarm motors'''
  mpstate.master().arducopter_disarm()

# http://stackoverflow.com/questions/211100/pythons-import-doesnt-work-as-expected
# has info on why this is necessary.

def import_package(name):
    """Given a package name like 'foo.bar.quux', imports the package
    and returns the desired module."""
    mod = __import__(name)
    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod


command_map = {
    'switch'  : (cmd_switch,   'set RC switch (1-5), 0 disables'),
    'rc'      : (cmd_rc,       'override a RC channel value'),
    'wp'      : (cmd_wp,       'waypoint management'),
    'script'  : (cmd_script,   'run a script of MAVProxy commands'),
    'fence'   : (cmd_fence,    'geo-fence management'),
    'rally'   : (cmd_rally,    'rally point management'),
    'param'   : (cmd_param,    'manage APM parameters'),
    'tuneopt' : (cmd_tuneopt,  'Select option for Tune Pot on Channel 6 (quadcopter only)'),
    'setup'   : (cmd_setup,    'go into setup mode'),
    'reset'   : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'status'  : (cmd_status,   'show status'),
    'auto'    : (cmd_auto,     'set AUTO mode'),
    'mode'    : (cmd_mode,     'set a mode'),
    'ground'  : (cmd_ground,   'do a ground start'),
    'level'   : (cmd_level,    'set level on a multicopter'),
    'accelcal': (cmd_accelcal, 'do 3D accelerometer calibration'),
    'calpress': (cmd_calpressure,'calibrate pressure sensors'),
    'loiter'  : (cmd_loiter,   'set LOITER mode'),
    'rtl'     : (cmd_rtl,      'set RTL mode'),
    'manual'  : (cmd_manual,   'set MANUAL mode'),
    'fbwa'    : (cmd_fbwa,     'set FBWA mode'),
    'guided'  : (cmd_guided,   'set GUIDED target'),
    'set'     : (cmd_set,      'mavproxy settings'),
    'bat'     : (cmd_bat,      'show battery levels'),
    'alt'     : (cmd_alt,      'show relative altitude'),
    'link'    : (cmd_link,     'show link status'),
    'servo'   : (cmd_servo,    'set a servo value'),
    'reboot'  : (cmd_reboot,   'reboot the autopilot'),
    'up'      : (cmd_up,       'adjust TRIM_PITCH_CD up by 5 degrees'),
    'auxopt'  : (cmd_auxopt,   'select option for aux switches on CH7 and CH8 (ArduCopter only)'),
    'watch'   : (cmd_watch,    'watch a MAVLink pattern'),
    'module'  : (cmd_module,   'module commands'),
    'alias'   : (cmd_alias,    'command aliases'),
    'arm'     : (cmd_arm,      'Copter/Plane arm motors'),
    'time'    : (cmd_time,     'Show autopilot time'),
    'disarm'  : (cmd_disarm,   'Copter/Plane disarm motors')
    }

def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)
    line = line.strip()

    if mpstate.status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            mpstate.status.setup_mode = False
            mpstate.status.flightmode = "MAV"
            mpstate.rl.set_prompt("MAV> ")
            return
        if line != '+++':
            line += '\r'
        for c in line:
            time.sleep(0.01)
            mpstate.master().write(c)
        return

    if not line:
        return

    args = line.split()
    cmd = args[0]
    while cmd in mpstate.aliases:
        line = mpstate.aliases[cmd]
        args = line.split() + args[1:]
        cmd = args[0]
        
    if cmd == 'help':
        k = command_map.keys()
        k.sort()
        for cmd in k:
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    if not cmd in command_map:
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command: %s" % str(e))


def scale_rc(servo, min, max, param):
    '''scale a PWM value'''
    # default to servo range of 1000 to 2000
    min_pwm  = get_mav_param('%s_MIN'  % param, 0)
    max_pwm  = get_mav_param('%s_MAX'  % param, 0)
    if min_pwm == 0 or max_pwm == 0:
        return 0
    if max_pwm == min_pwm:
        p = 0.0
    else:
        p = (servo-min_pwm) / float(max_pwm-min_pwm)
    v = min + p*(max-min)
    if v < min:
        v = min
    if v > max:
        v = max
    return v


def system_check():
    '''check that the system is ready to fly'''
    ok = True

    if mavutil.mavlink.WIRE_PROTOCOL_VERSION == '1.0':
        if not 'GPS_RAW_INT' in mpstate.status.msgs:
            say("WARNING no GPS status")
            return
        if mpstate.status.msgs['GPS_RAW_INT'].fix_type != 3:
            say("WARNING no GPS lock")
            ok = False
    else:
        if not 'GPS_RAW' in mpstate.status.msgs and not 'GPS_RAW_INT' in mpstate.status.msgs:
            say("WARNING no GPS status")
            return
        if mpstate.status.msgs['GPS_RAW'].fix_type != 2:
            say("WARNING no GPS lock")
            ok = False

    if not 'PITCH_MIN' in mpstate.mav_param:
        say("WARNING no pitch parameter available")
        return

    if int(mpstate.mav_param['PITCH_MIN']) > 1300:
        say("WARNING PITCH MINIMUM not set")
        ok = False

    if not 'ATTITUDE' in mpstate.status.msgs:
        say("WARNING no attitude recorded")
        return

    if math.fabs(mpstate.status.msgs['ATTITUDE'].pitch) > math.radians(5):
        say("WARNING pitch is %u degrees" % math.degrees(mpstate.status.msgs['ATTITUDE'].pitch))
        ok = False

    if math.fabs(mpstate.status.msgs['ATTITUDE'].roll) > math.radians(5):
        say("WARNING roll is %u degrees" % math.degrees(mpstate.status.msgs['ATTITUDE'].roll))
        ok = False

    if ok:
        say("All OK SYSTEM READY TO FLY")


def beep():
    f = open("/dev/tty", mode="w")
    f.write(chr(7))
    f.close()

def vcell_to_battery_percent(vcell):
    '''convert a cell voltage to a percentage battery level'''
    if vcell > 4.1:
        # above 4.1 is 100% battery
        return 100.0
    elif vcell > 3.81:
        # 3.81 is 17% remaining, from flight logs
        return 17.0 + 83.0 * (vcell - 3.81) / (4.1 - 3.81)
    elif vcell > 3.2:
        # below 3.2 it degrades fast. It's dead at 3.2
        return 0.0 + 17.0 * (vcell - 3.20) / (3.81 - 3.20)
    # it's dead or disconnected
    return 0.0


def battery_update(SYS_STATUS):
    '''update battery level'''

    # main flight battery
    mpstate.status.battery_level = SYS_STATUS.battery_remaining

    # avionics battery
    if not 'AP_ADC' in mpstate.status.msgs:
        return
    rawvalue = float(mpstate.status.msgs['AP_ADC'].adc2)
    INPUT_VOLTAGE = 4.68
    VOLT_DIV_RATIO = 3.56
    voltage = rawvalue*(INPUT_VOLTAGE/1024.0)*VOLT_DIV_RATIO
    vcell = voltage / mpstate.settings.numcells

    avionics_battery_level = vcell_to_battery_percent(vcell)

    if mpstate.status.avionics_battery_level == -1 or abs(avionics_battery_level-mpstate.status.avionics_battery_level) > 70:
        mpstate.status.avionics_battery_level = avionics_battery_level
    else:
        mpstate.status.avionics_battery_level = (95*mpstate.status.avionics_battery_level + 5*avionics_battery_level)/100



def battery_report():
    '''report battery level'''
    if int(mpstate.settings.battreadout) == 0:
        return

    mpstate.console.set_status('Battery', 'Battery: %u' % mpstate.status.battery_level, row=0)

    rbattery_level = int((mpstate.status.battery_level+5)/10)*10

    if rbattery_level != mpstate.status.last_battery_announce:
        say("Flight battery %u percent" % rbattery_level, priority='notification')
        mpstate.status.last_battery_announce = rbattery_level
    if rbattery_level <= 20:
        say("Flight battery warning")

    # avionics battery reporting disabled for now
    return
    avionics_rbattery_level = int((mpstate.status.avionics_battery_level+5)/10)*10

    if avionics_rbattery_level != mpstate.status.last_avionics_battery_announce:
        say("Avionics Battery %u percent" % avionics_rbattery_level, priority='notification')
        mpstate.status.last_avionics_battery_announce = avionics_rbattery_level
    if avionics_rbattery_level <= 20:
        say("Avionics battery warning")


def handle_msec_timestamp(m, master):
    '''special handling for MAVLink packets with a time_boot_ms field'''

    if m.get_type() == 'GLOBAL_POSITION_INT':
        # this is fix time, not boot time
        return

    msec = m.time_boot_ms
    if msec + 30000 < master.highest_msec:
        say('Time has wrapped')
        print('Time has wrapped', msec, master.highest_msec)
        mpstate.status.highest_msec = msec
        for mm in mpstate.mav_master:
            mm.link_delayed = False
            mm.highest_msec = msec
        return

    # we want to detect when a link is delayed
    master.highest_msec = msec
    if msec > mpstate.status.highest_msec:
        mpstate.status.highest_msec = msec
    if msec < mpstate.status.highest_msec and len(mpstate.mav_master) > 1:
        master.link_delayed = True
    else:
        master.link_delayed = False

def report_altitude(altitude):
    '''possibly report a new altitude'''
    master = mpstate.master()
    if getattr(mpstate.console, 'ElevationMap', None) is not None and mpstate.settings.basealt != 0:
        lat = master.field('GLOBAL_POSITION_INT', 'lat', 0)*1.0e-7
        lon = master.field('GLOBAL_POSITION_INT', 'lon', 0)*1.0e-7
        alt1 = mpstate.console.ElevationMap.GetElevation(lat, lon)
        alt2 = mpstate.settings.basealt
        altitude += alt2 - alt1
    mpstate.status.altitude = altitude
    if (int(mpstate.settings.altreadout) > 0 and
        math.fabs(mpstate.status.altitude - mpstate.status.last_altitude_announce) >= int(mpstate.settings.altreadout)):
        mpstate.status.last_altitude_announce = mpstate.status.altitude
        rounded_alt = int(mpstate.settings.altreadout) * ((mpstate.settings.altreadout/2 + int(mpstate.status.altitude)) / int(mpstate.settings.altreadout))
        say("height %u" % rounded_alt, priority='notification')


def master_send_callback(m, master):
    '''called on sending a message'''
    mtype = m.get_type()

    if mtype != 'BAD_DATA' and mpstate.logqueue:
        usec = get_usec()
        usec = (usec & ~3) | 3 # linknum 3
        mpstate.logqueue.put(str(struct.pack('>Q', usec) + m.get_msgbuf()))


def master_callback(m, master):
    '''process mavlink message m on master, sending any messages to recipients'''

    if getattr(m, '_timestamp', None) is None:
        master.post_message(m)
    mpstate.status.counters['MasterIn'][master.linknum] += 1

    if getattr(m, 'time_boot_ms', None) is not None:
        # update link_delayed attribute
        handle_msec_timestamp(m, master)

    mtype = m.get_type()

    # and log them
    if mtype not in ['BAD_DATA','LOG_DATA'] and mpstate.logqueue:
        # put link number in bottom 2 bits, so we can analyse packet
        # delay in saved logs
        usec = get_usec()
        usec = (usec & ~3) | master.linknum
        mpstate.logqueue.put(str(struct.pack('>Q', usec) + m.get_msgbuf()))

    if mtype in [ 'HEARTBEAT', 'GPS_RAW_INT', 'GPS_RAW', 'GLOBAL_POSITION_INT', 'SYS_STATUS' ]:
        if master.linkerror:
            master.linkerror = False
            say("link %u OK" % (master.linknum+1))
        mpstate.status.last_message = time.time()
        master.last_message = mpstate.status.last_message

    if master.link_delayed:
        # don't process delayed packets that cause double reporting
        if mtype in [ 'MISSION_CURRENT', 'SYS_STATUS', 'VFR_HUD',
                      'GPS_RAW_INT', 'SCALED_PRESSURE', 'GLOBAL_POSITION_INT',
                      'NAV_CONTROLLER_OUTPUT' ]:
            return

    if mtype == 'HEARTBEAT' and m.get_srcSystem() != 255:
        if (mpstate.status.target_system != m.get_srcSystem() or
            mpstate.status.target_component != m.get_srcComponent()):
            mpstate.status.target_system = m.get_srcSystem()
            mpstate.status.target_component = m.get_srcComponent()
            say("online system %u component %u" % (mpstate.status.target_system, mpstate.status.target_component),'message')
            if len(mpstate.mav_param_set) == 0 or len(mpstate.mav_param_set) != mpstate.mav_param_count:
                master.param_fetch_all()

        if mpstate.status.heartbeat_error:
            mpstate.status.heartbeat_error = False
            say("heartbeat OK")
        if master.linkerror:
            master.linkerror = False
            say("link %u OK" % (master.linknum+1))

        mpstate.status.last_heartbeat = time.time()
        master.last_heartbeat = mpstate.status.last_heartbeat

        armed = mpstate.master().motors_armed()
        if armed != mpstate.status.armed:
            mpstate.status.armed = armed
            if armed:
                say("ARMED")
            else:
                say("DISARMED")
        
    elif mtype == 'STATUSTEXT':
        if m.text != mpstate.status.last_apm_msg or time.time() > mpstate.status.last_apm_msg_time+2:
            mpstate.console.writeln("APM: %s" % m.text, bg='red')
            mpstate.status.last_apm_msg = m.text
            mpstate.status.last_apm_msg_time = time.time()
    elif mtype == 'PARAM_VALUE':
        param_id = "%.16s" % m.param_id
        if m.param_index != -1 and m.param_index not in mpstate.mav_param_set:
            added_new_parameter = True
            mpstate.mav_param_set.add(m.param_index)
        else:
            added_new_parameter = False
        if m.param_count != -1:
            mpstate.mav_param_count = m.param_count
        mpstate.mav_param[str(param_id)] = m.param_value
        if mpstate.status.fetch_one > 0:
            mpstate.status.fetch_one -= 1
            mpstate.console.writeln("%s = %f" % (param_id, m.param_value))
        if added_new_parameter and len(mpstate.mav_param_set) == m.param_count:
            mpstate.console.writeln("Received %u parameters" % m.param_count)
            if mpstate.status.logdir != None:
                mpstate.mav_param.save(os.path.join(mpstate.status.logdir, 'mav.parm'), '*', verbose=True)

    elif mtype == 'SERVO_OUTPUT_RAW':
        if opts.quadcopter:
            mpstate.status.rc_throttle[0] = scale_rc(m.servo1_raw, 0.0, 1.0, param='RC3')
            mpstate.status.rc_throttle[1] = scale_rc(m.servo2_raw, 0.0, 1.0, param='RC3')
            mpstate.status.rc_throttle[2] = scale_rc(m.servo3_raw, 0.0, 1.0, param='RC3')
            mpstate.status.rc_throttle[3] = scale_rc(m.servo4_raw, 0.0, 1.0, param='RC3')
        else:
            mpstate.status.rc_aileron  = scale_rc(m.servo1_raw, -1.0, 1.0, param='RC1') * mpstate.settings.rc1mul
            mpstate.status.rc_elevator = scale_rc(m.servo2_raw, -1.0, 1.0, param='RC2') * mpstate.settings.rc2mul
            mpstate.status.rc_throttle = scale_rc(m.servo3_raw, 0.0, 1.0, param='RC3')
            mpstate.status.rc_rudder   = scale_rc(m.servo4_raw, -1.0, 1.0, param='RC4') * mpstate.settings.rc4mul
            if mpstate.status.rc_throttle < 0.1:
                mpstate.status.rc_throttle = 0

    elif mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
        if mpstate.status.wp_op is None:
            mpstate.console.error("No waypoint load started")
        else:
            mpstate.status.wploader.clear()
            mpstate.status.wploader.expected_count = m.count
            mpstate.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                             time.asctime(time.localtime(m._timestamp)),
                                                                             time.asctime()))
            master.waypoint_request_send(0)

    elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and mpstate.status.wp_op != None:
        if m.seq > mpstate.status.wploader.count():
            mpstate.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, mpstate.status.wploader.count()))
        elif m.seq < mpstate.status.wploader.count():
            # a duplicate
            pass
        else:
            mpstate.status.wploader.add(m)
        if m.seq+1 < mpstate.status.wploader.expected_count:
            master.waypoint_request_send(m.seq+1)
        else:
            if mpstate.status.wp_op == 'list':
                for i in range(mpstate.status.wploader.count()):
                    w = mpstate.status.wploader.wp(i)
                    print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                        w.command, w.frame, w.x, w.y, w.z,
                        w.param1, w.param2, w.param3, w.param4,
                        w.current, w.autocontinue))
                if mpstate.status.logdir != None:
                    waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
                    save_waypoints(waytxt)
                    print("Saved waypoints to %s" % waytxt)
            elif mpstate.status.wp_op == "save":
                save_waypoints(mpstate.status.wp_save_filename)
            mpstate.status.wp_op = None

    elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
        process_waypoint_request(m, master)

    elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
        if m.seq != mpstate.status.last_waypoint:
            mpstate.status.last_waypoint = m.seq
            say("waypoint %u" % m.seq,priority='message')

    elif mtype == "SYS_STATUS":
        battery_update(m)
        if master.flightmode != mpstate.status.flightmode and time.time() > mpstate.status.last_mode_announce + 2:
            mpstate.status.flightmode = master.flightmode
            mpstate.status.last_mode_announce = time.time()
            mpstate.rl.set_prompt(mpstate.status.flightmode + "> ")
            say("Mode " + mpstate.status.flightmode)

    elif mtype == "VFR_HUD":
        have_gps_fix = False
        if 'GPS_RAW' in mpstate.status.msgs and mpstate.status.msgs['GPS_RAW'].fix_type == 2:
            have_gps_fix = True
        if 'GPS_RAW_INT' in mpstate.status.msgs and mpstate.status.msgs['GPS_RAW_INT'].fix_type == 3:
            have_gps_fix = True
        if have_gps_fix and not mpstate.status.have_gps_lock:
                say("GPS lock at %u meters" % m.alt, priority='notification')
                mpstate.status.have_gps_lock = True

    elif mtype == "GPS_RAW":
        if mpstate.status.have_gps_lock:
            if m.fix_type != 2 and not mpstate.status.lost_gps_lock and (time.time() - mpstate.status.last_gps_lock) > 3:
                say("GPS fix lost")
                mpstate.status.lost_gps_lock = True
            if m.fix_type == 2 and mpstate.status.lost_gps_lock:
                say("GPS OK")
                mpstate.status.lost_gps_lock = False
            if m.fix_type == 2:
                mpstate.status.last_gps_lock = time.time()

    elif mtype == "GPS_RAW_INT":
        if mpstate.status.have_gps_lock:
            if m.fix_type != 3 and not mpstate.status.lost_gps_lock and (time.time() - mpstate.status.last_gps_lock) > 3:
                say("GPS fix lost")
                mpstate.status.lost_gps_lock = True
            if m.fix_type == 3 and mpstate.status.lost_gps_lock:
                say("GPS OK")
                mpstate.status.lost_gps_lock = False
            if m.fix_type == 3:
                mpstate.status.last_gps_lock = time.time()

    elif mtype == "NAV_CONTROLLER_OUTPUT" and mpstate.status.flightmode == "AUTO" and mpstate.settings.distreadout:
        rounded_dist = int(m.wp_dist/mpstate.settings.distreadout)*mpstate.settings.distreadout
        if math.fabs(rounded_dist - mpstate.status.last_distance_announce) >= mpstate.settings.distreadout:
            if rounded_dist != 0:
                say("%u" % rounded_dist, priority="progress")
            mpstate.status.last_distance_announce = rounded_dist

    elif mtype == "FENCE_STATUS":
        if not mpstate.status.fence_enabled:
            mpstate.status.fence_enabled = True
            say("fence enabled")
        if mpstate.status.last_fence_breach != m.breach_time:
            say("fence breach")
        if mpstate.status.last_fence_status != m.breach_status:
            if m.breach_status == mavutil.mavlink.FENCE_BREACH_NONE:
                say("fence OK")
        mpstate.status.last_fence_breach = m.breach_time
        mpstate.status.last_fence_status = m.breach_status

    elif mtype == "GLOBAL_POSITION_INT":
        report_altitude(m.relative_alt*0.001)

    elif mtype == "BAD_DATA":
        if mpstate.settings.shownoise and mavutil.all_printable(m.data):
            mpstate.console.write(str(m.data), bg='red')
    elif mtype in [ "COMMAND_ACK", "MISSION_ACK" ]:
        mpstate.console.writeln("Got MAVLink msg: %s" % m)
    else:
        #mpstate.console.writeln("Got MAVLink msg: %s" % m)
        pass

    if mpstate.status.watch is not None:
        if fnmatch.fnmatch(m.get_type().upper(), mpstate.status.watch.upper()):
            mpstate.console.writeln(m)

    # keep the last message of each type around
    mpstate.status.msgs[m.get_type()] = m
    if not m.get_type() in mpstate.status.msg_count:
        mpstate.status.msg_count[m.get_type()] = 0
    mpstate.status.msg_count[m.get_type()] += 1

    # don't pass along bad data
    if mtype != "BAD_DATA":
        # pass messages along to listeners, except for REQUEST_DATA_STREAM, which
        # would lead a conflict in stream rate setting between mavproxy and the other
        # GCS
        if mpstate.settings.mavfwd_rate or mtype != 'REQUEST_DATA_STREAM':
            for r in mpstate.mav_outputs:
                r.write(m.get_msgbuf())

        # pass to modules
        for mod in mpstate.modules:
            if not hasattr(mod, 'mavlink_packet'):
                continue
            try:
                mod.mavlink_packet(m)
            except Exception, msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    import traceback
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)

def process_master(m):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        return
    if mpstate.logqueue_raw:
        mpstate.logqueue_raw.put(str(s))

    if mpstate.status.setup_mode:
        sys.stdout.write(str(s))
        sys.stdout.flush()
        return

    if m.first_byte and opts.auto_protocol:
        m.auto_mavlink_version(s)
    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)
            if msg.get_type() == "BAD_DATA":
                if opts.show_errors:
                    mpstate.console.writeln("MAV error: %s" % msg)
                mpstate.status.mav_error += 1



def process_mavlink(slave):
    '''process packets from MAVLink slaves, forwarding to the master'''
    try:
        buf = slave.recv()
    except socket.error:
        return
    try:
        if slave.first_byte and opts.auto_protocol:
            slave.auto_mavlink_version(buf)
        msgs = slave.mav.parse_buffer(buf)
    except mavutil.mavlink.MAVError as e:
        mpstate.console.error("Bad MAVLink slave message from %s: %s" % (slave.address, e.message))
        return
    if msgs is None:
        return
    if mpstate.settings.mavfwd and not mpstate.status.setup_mode:
        for m in msgs:
            mpstate.master().write(m.get_msgbuf())
    mpstate.status.counters['Slave'] += 1


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def log_writer():
    '''log writing thread'''
    while True:
        mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue_raw.empty():
            mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue.empty():
            mpstate.logfile.write(mpstate.logqueue.get())
        mpstate.logfile.flush()
        mpstate.logfile_raw.flush()

def open_logs():
    '''open log files'''
    if opts.append_log or opts.continue_mode:
        mode = 'a'
    else:
        mode = 'w'
    logfile = opts.logfile
    if opts.aircraft is not None:
        dirname = "%s/logs/%s" % (opts.aircraft, time.strftime("%Y-%m-%d"))
        mkdir_p(dirname)
        highest = None
        for i in range(1, 10000):
            fdir = os.path.join(dirname, 'flight%u' % i)
            if not os.path.exists(fdir):
                break
            highest = fdir
        if mpstate.continue_mode and highest is not None:
            fdir = highest
        elif os.path.exists(fdir):
            print("Flight logs full")
            sys.exit(1)
        mkdir_p(fdir)
        print(fdir)
        logfile = os.path.join(fdir, 'flight.tlog')
        mpstate.status.logdir = fdir
    mpstate.logfile_name = logfile
    mpstate.logfile = open(logfile, mode=mode)
    mpstate.logfile_raw = open(logfile+'.raw', mode=mode)
    print("Logging to %s" % logfile)

    # queues for logging
    mpstate.logqueue = Queue.Queue()
    mpstate.logqueue_raw = Queue.Queue()

    # use a separate thread for writing to the logfile to prevent
    # delays during disk writes (important as delays can be long if camera
    # app is running)
    t = threading.Thread(target=log_writer)
    t.daemon = True
    t.start()

def set_stream_rates():
    '''set mavlink stream rates'''
    if (not msg_period.trigger() and
        mpstate.status.last_streamrate1 == mpstate.settings.streamrate and
        mpstate.status.last_streamrate2 == mpstate.settings.streamrate2):
        return
    mpstate.status.last_streamrate1 = mpstate.settings.streamrate
    mpstate.status.last_streamrate2 = mpstate.settings.streamrate2
    for master in mpstate.mav_master:
        if master.linknum == 0:
            rate = mpstate.settings.streamrate
        else:
            rate = mpstate.settings.streamrate2
        if rate != -1:
            master.mav.request_data_stream_send(mpstate.status.target_system, mpstate.status.target_component,
                                                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                rate, 1)

def check_link_status():
    '''check status of master links'''
    tnow = time.time()
    if mpstate.status.last_message != 0 and tnow > mpstate.status.last_message + 5:
        say("no link")
        mpstate.status.heartbeat_error = True
    for master in mpstate.mav_master:
        if not master.linkerror and tnow > master.last_message + 5:
            say("link %u down" % (master.linknum+1))
            master.linkerror = True

def send_heartbeat(master):
    if master.mavlink10():
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
    else:
        MAV_GROUND = 5
        MAV_AUTOPILOT_NONE = 4
        master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)

def periodic_tasks():
    '''run periodic checks'''
    if mpstate.status.setup_mode:
        return

    if mpstate.settings.heartbeat != 0:
        heartbeat_period.frequency = mpstate.settings.heartbeat

    if heartbeat_period.trigger() and mpstate.settings.heartbeat != 0:
        mpstate.status.counters['MasterOut'] += 1
        for master in mpstate.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    if param_period.trigger():
        if len(mpstate.mav_param_set) == 0:
            mpstate.master().param_fetch_all()
        elif mpstate.mav_param_count != 0 and len(mpstate.mav_param_set) != mpstate.mav_param_count:
            if mpstate.master().time_since('PARAM_VALUE') >= 1:
                diff = set(range(mpstate.mav_param_count)).difference(mpstate.mav_param_set)
                count = 0
                while len(diff) > 0 and count < mpstate.settings.parambatch:
                    idx = diff.pop()
                    mpstate.master().param_fetch_one(idx)
                    count += 1

        # cope with packet loss fetching mission
        if mpstate.master().time_since('MISSION_ITEM') >= 2 and mpstate.status.wploader.count() < getattr(mpstate.status.wploader,'expected_count',0):
            seq = mpstate.status.wploader.count()
            print("re-requesting WP %u" % seq)
            mpstate.master().waypoint_request_send(seq)

    if battery_period.trigger():
        battery_report()

    if mpstate.override_period.trigger():
        if (mpstate.status.override != [ 0 ] * 8 or
            mpstate.status.override != mpstate.status.last_override or
            mpstate.status.override_counter > 0):
            mpstate.status.last_override = mpstate.status.override[:]
            send_rc_override()
            if mpstate.status.override_counter > 0:
                mpstate.status.override_counter -= 1


    # call optional module idle tasks. These are called at several hundred Hz
    for m in mpstate.modules:
        if hasattr(m, 'idle_task'):
            try:
                m.idle_task()
            except Exception, msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    import traceback
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)


def main_loop():
    '''main processing loop'''
    if not mpstate.status.setup_mode and not opts.nowait:
        for master in mpstate.mav_master:
            send_heartbeat(master)
            master.wait_heartbeat()
            if len(mpstate.mav_param) < 10 or not mpstate.continue_mode:
                mpstate.mav_param_set = set()
                master.param_fetch_all()
        set_stream_rates()

    while True:
        if mpstate is None or mpstate.status.exit:
            return
        if mpstate.rl.line is not None:
            cmds = mpstate.rl.line.split(';')
            for c in cmds:
                process_stdin(c)
            mpstate.rl.line = None

        for master in mpstate.mav_master:
            if master.fd is None:
                if master.port.inWaiting() > 0:
                    process_master(master)

        periodic_tasks()

        rin = []
        for master in mpstate.mav_master:
            if master.fd is not None:
                rin.append(master.fd)
        for m in mpstate.mav_outputs:
            rin.append(m.fd)
        if rin == []:
            time.sleep(0.0001)
            continue

        for fd in mpstate.select_extra:
            rin.append(fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], 0.0001)
        except select.error:
            continue

        if mpstate is None:
            return

        for fd in rin:
            for master in mpstate.mav_master:
                if fd == master.fd:
                    process_master(master)
                    continue
            for m in mpstate.mav_outputs:
                if fd == m.fd:
                    process_mavlink(m)
                    continue

            # this allow modules to register their own file descriptors
            # for the main select loop
            if fd in mpstate.select_extra:
                try:
                    # call the registered read function
                    (fn, args) = mpstate.select_extra[fd]
                    fn(args)
                except Exception, msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)



def input_loop():
    '''wait for user input'''
    while True:
        while mpstate.rl.line is not None:
            time.sleep(0.01)
        try:
            line = raw_input(mpstate.rl.prompt)
        except EOFError:
            mpstate.status.exit = True
            sys.exit(1)
        mpstate.rl.line = line


def run_script(scriptfile):
    '''run a script file'''
    try:
        f = open(scriptfile, mode='r')
    except Exception:
        return
    mpstate.console.writeln("Running script %s" % scriptfile)
    for line in f:
        line = line.strip()
        if line == "" or line.startswith('#'):
            continue
        if line.startswith('@'):
            line = line[1:]
        else:
            mpstate.console.writeln("-> %s" % line)
        process_stdin(line)
    f.close()


if __name__ == '__main__':

    from optparse import OptionParser
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master",dest="master", action='append', help="MAVLink master port", default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="master port baud rate", default=115200)
    parser.add_option("--out",   dest="output", help="MAVLink output port",
                      action='append', default=[])
    parser.add_option("--sitl", dest="sitl",  default=None, help="SITL output port")
    parser.add_option("--streamrate",dest="streamrate", default=4, type='int',
                      help="MAVLink stream rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=1, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=1, help='MAVLink target master component')
    parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
                      default='mav.tlog')
    parser.add_option("-a", "--append-log", dest="append_log", help="Append to log files",
                      action='store_true', default=False)
    parser.add_option("--quadcopter", dest="quadcopter", help="use quadcopter controls",
                      action='store_true', default=False)
    parser.add_option("--setup", dest="setup", help="start in setup mode",
                      action='store_true', default=False)
    parser.add_option("--nodtr", dest="nodtr", help="disable DTR drop on close",
                      action='store_true', default=False)
    parser.add_option("--show-errors", dest="show_errors", help="show MAVLink error packets",
                      action='store_true', default=False)
    parser.add_option("--speech", dest="speech", help="use text to speach",
                      action='store_true', default=False)
    parser.add_option("--num-cells", dest="num_cells", help="number of LiPo battery cells",
                      type='int', default=0)
    parser.add_option("--aircraft", dest="aircraft", help="aircraft name", default=None)
    parser.add_option("--cmd", dest="cmd", help="initial commands", default=None)
    parser.add_option("--console", action='store_true', help="use GUI console")
    parser.add_option("--map", action='store_true', help="load map module")
    parser.add_option(
        '--load-module',
        action='append',
        default=[],
        help='Load the specified module. Can be used multiple times, or with a comma separated list')
    parser.add_option("--mav09", action='store_true', default=False, help="Use MAVLink protocol 0.9")
    parser.add_option("--auto-protocol", action='store_true', default=False, help="Auto detect MAVLink protocol version")
    parser.add_option("--nowait", action='store_true', default=False, help="don't wait for HEARTBEAT on startup")
    parser.add_option("--continue", dest='continue_mode', action='store_true', default=False, help="continue logs")
    parser.add_option("--dialect",  default="ardupilotmega", help="MAVLink dialect")

    (opts, args) = parser.parse_args()

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    from pymavlink import mavutil, mavwp, mavparm
    mavutil.set_dialect(opts.dialect)

    # global mavproxy state
    mpstate = MPState()
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode

    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        say('Startup')

    if not opts.master:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*"])
        if len(serial_list) == 1:
            opts.master = [serial_list[0].device]
        else:
            print('''
Please choose a MAVLink master with --master
For example:
    --master=com14
    --master=/dev/ttyUSB0
    --master=127.0.0.1:14550

Auto-detected serial ports are:
''')
            for port in serial_list:
                print("%s" % port)
            sys.exit(1)

    # container for status information
    mpstate.status.target_system = opts.TARGET_SYSTEM
    mpstate.status.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []

    # open master link
    for mdev in opts.master:
        m = mavutil.mavlink_connection(mdev, autoreconnect=True, baud=opts.baudrate)
        m.mav.set_callback(master_callback, m)
        if hasattr(m.mav, 'set_send_callback'):
            m.mav.set_send_callback(master_send_callback, m)
        m.linknum = len(mpstate.mav_master)
        m.linkerror = False
        m.link_delayed = False
        m.last_heartbeat = 0
        m.last_message = 0
        m.highest_msec = 0
        mpstate.mav_master.append(m)
        mpstate.status.counters['MasterIn'].append(0)

    # log all packets from the master, for later replay
    open_logs()

    if mpstate.continue_mode and mpstate.status.logdir != None:
        parmfile = os.path.join(mpstate.status.logdir, 'mav.parm')
        if os.path.exists(parmfile):
            mpstate.mav_param.load(parmfile)
            for m in mpstate.mav_master:
                m.param_fetch_complete = True
        waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
        if os.path.exists(waytxt):
            mpstate.status.wploader.load(waytxt)
            print("Loaded waypoints from %s" % waytxt)
        fencetxt = os.path.join(mpstate.status.logdir, 'fence.txt')
        if os.path.exists(fencetxt):
            mpstate.status.fenceloader.load(fencetxt)
            print("Loaded fence from %s" % fencetxt)

    # open any mavlink UDP ports
    for p in opts.output:
        mpstate.mav_outputs.append(mavutil.mavlink_connection(p, baud=opts.baudrate, input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.numcells = opts.num_cells
    mpstate.settings.speech = opts.speech
    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    msg_period = mavutil.periodic_event(1.0/15)
    param_period = mavutil.periodic_event(1)
    log_period = mavutil.periodic_event(2)
    heartbeat_period = mavutil.periodic_event(1)
    battery_period = mavutil.periodic_event(0.1)
    if mpstate.sitl_output:
        mpstate.override_period = mavutil.periodic_event(20)
    else:
        mpstate.override_period = mavutil.periodic_event(1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.rl = rline("MAV> ")
    if opts.setup:
        mpstate.rl.set_prompt("")

    if 'HOME' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['HOME'], ".mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)

    if opts.aircraft is not None:
        start_script = os.path.join(opts.aircraft, "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
        else:
            print("no script %s" % start_script)

    if not opts.setup:
        # some core functionality is in modules
        standard_modules = ['log']
        for m in standard_modules:
            process_stdin('module load %s' % m)

    if opts.console:
        process_stdin('module load console')

    if opts.map:
        process_stdin('module load map')

    for module in opts.load_module:
        modlist = module.split(',')
        for mod in modlist:
            process_stdin('module load %s' % mod)

    if opts.cmd is not None:
        cmds = opts.cmd.split(';')
        for c in cmds:
            process_stdin(c)

    # run main loop as a thread
    mpstate.status.thread = threading.Thread(target=main_loop)
    mpstate.status.thread.daemon = True
    mpstate.status.thread.start()

    # use main program for input. This ensures the terminal cleans
    # up on exit
    try:
        input_loop()
    except KeyboardInterrupt:
        print("exiting")
        mpstate.status.exit = True
        sys.exit(1)
