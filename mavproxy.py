#!/usr/bin/env python
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

RTL
wind baro
upgrade eeprom
MODE parameters
reset now on format version
FLTMODE_1

'''

import sys, os, struct, math, time, socket
import fnmatch, errno
from curses import ascii

# find the mavlink.py module
for d in [ 'pymavlink', '../pymavlink' ]:
    if os.path.exists(d):
        sys.path.insert(0, d)
import mavlink, readline, select

def kt2mps(x):
    '''knots to meters per second'''
    return float(x)*0.514444444

def deg2rad(x):
    '''degrees to radians'''
    return (float(x) / 360.0) * 2.0 * math.pi

def ft2m(x):
    '''feet to meters'''
    return float(x) * 0.3048

def get_usec():
    '''time since 1970 in microseconds'''
    return long(time.time() * float(1000*1000))

class rline(object):
    '''async readline abstraction'''
    def __init__(self, handler, prompt, *args, **kwargs):
        import ctypes
        def callback(line):
            handler(self, line, *args, **kwargs)
        self.args = args
        self.kwargs = kwargs
        for lib in [ 'libreadline.so.6', 'libreadline.so.5', 'libreadline.so' ]:
            try:
                self.rl_lib = ctypes.cdll.LoadLibrary(lib)
                break
            except:
                pass
        if self.rl_lib is None:
            raise RuntimeError("Unable to find readline library")
        self.cHandler = ctypes.CFUNCTYPE(None, ctypes.c_char_p)(callback)
        self.rl_lib.rl_callback_handler_install(prompt, self.cHandler)
    def set_prompt(self, prompt):
        self.rl_lib.rl_set_prompt(prompt)
        self.rl_lib.rl_redisplay(prompt)
    def read_char(self):
        self.rl_lib.rl_callback_read_char()
    def cleanup(self):
        self.rl_lib.rl_cleanup_after_signal()
    def add_history(self, line):
        self.rl_lib.add_history(line)

def say(text, priority):
    '''speak some text'''
    ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
    print(text)
    if opts.speech:
        import speechd
        status.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
        status.speech.set_output_module('festival')
        status.speech.set_language('en')
	status.speech.set_priority(priority)
        status.speech.set_punctuation(speechd.PunctuationMode.SOME)
        status.speech.speak(text)
        status.speech.close()


class periodic_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.last_time = time.time()
    def trigger(self):
        '''return True if we should trigger now'''
        tnow = time.time()
        if self.last_time + (1.0/self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False

class status(object):
    '''hold status information about the master'''
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
        self.counters = {'MasterIn' : 0, 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.setup_mode = opts.setup
        self.wp_op = None
        self.wp_save_filename = None
        self.wpoints = []
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.mav_error = 0
        self.in_mavlink = False
        self.master_buffer = ""
        self.show_pwm = False
        self.target_system = -1
        self.target_component = -1
        self.speech = None
        self.mode_string = None
        self.first_altitude = 0.0
        self.last_altitude_announce = 0.0
        self.last_battery_announce = 0
        self.last_waypoint = 0

    def show(self, f):
        '''write status to status.txt'''
        f.write('Counters: ')
        for c in status.counters:
            f.write('%s:%u ' % (c, status.counters[c]))
        f.write('\n')
        f.write('MAV Errors: %u\n' % status.mav_error)
        f.write(str(self.gps)+'\n')
        for m in status.msgs:
                f.write("%u: %s\n" % (status.msg_count[m], str(status.msgs[m])))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()

# current MAV master parameters
mav_param = {}

def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    global mav_param
    if not param in mav_param:
        return default
    return mav_param[param]
    

def control_set(mav_master, name, channel, args):
    '''set a fixed RC control PWM value'''
    if len(args) != 1:
        print("Usage: %s <pwmvalue>" % name)
        return
    values = [ 65535 ] * 8
    values[channel-1] = int(args[0])
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    
def cmd_roll(args, rl, mav_master):
    control_set(mav_master, 'roll', 1, args)

def cmd_pitch(args, rl, mav_master):
    control_set(mav_master, 'pitch', 2, args)

def cmd_rudder(args, rl, mav_master):
    control_set(mav_master, 'rudder', 4, args)

def cmd_throttle(args, rl, mav_master):
    control_set(mav_master, 'throttle', 3, args)


def cmd_switch(args, rl, mav_master):
    '''handle RC switch changes'''
    mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
    if len(args) != 1:
        print("Usage: switch <pwmvalue>")
        return
    value = int(args[0])
    if value < 0 or value > 6:
        print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
        return
    flite_mode_ch_parm = int(get_mav_param("FLTMODE_CH", 8))
    values = [ 65535 ] * 8
    values[flite_mode_ch_parm-1] = mapping[value]
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    if value == 0:
        print("Disabled RC switch override")
    else:
        print("Set RC switch override to %u (PWM=%u)" % (value, mapping[value]))

def cmd_trim(args, rl, mav_master):
    '''trim aileron, elevator and rudder to current values'''
    if not 'RC_CHANNELS_RAW' in status.msgs:
        print("No RC_CHANNELS_RAW to trim with")
        return
    m = status.msgs['RC_CHANNELS_RAW']

    mav_master.mav.param_set_send(status.target_system,
                                  status.target_component,
                                  'ROLL_TRIM',
                                  m.chan1_raw)
    mav_master.mav.param_set_send(status.target_system,
                                  status.target_component,
                                  'PITCH_TRIM',
                                  m.chan2_raw)
    mav_master.mav.param_set_send(status.target_system,
                                  status.target_component,
                                  'YAW_TRIM',
                                  m.chan4_raw)
    print("Trimmed to aileron=%u elevator=%u rudder=%u" % (
        m.chan1_raw, m.chan2_raw, m.chan4_raw))
    

def cmd_rc(args, rl, mav_master):
    '''handle RC value override'''
    if len(args) != 2:
        print("Usage: rc <channel> <pwmvalue>")
        return
    channel = int(args[0])
    value   = int(args[1])
    if value == -1:
        value = 65535
    if channel < 1 or channel > 8:
        print("Channel must be between 1 and 8")
        return
    values = [ 65535 ] * 8
    values[channel-1] = value
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)

def cmd_disarm(args, rl, mav_master):
    '''disarm the motors'''
    values = [ 65535 ] * 8
    values[3] = 1000
    values[2] = 1000
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)

def cmd_arm(args, rl, mav_master):
    '''arm the motors'''
    values = [ 65535 ] * 8
    values[3] = 2000
    values[2] = 1000
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    time.sleep(2)
    values[3] = 1500
    values[2] = 1000
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)
    mav_master.mav.rc_channels_override_send(status.target_system, status.target_component, *values)


def cmd_loiter(args, rl, mav_master):
    '''set LOITER mode'''
    MAV_ACTION_LOITER = 27
    mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_LOITER)

def cmd_auto(args, rl, mav_master):
    '''set AUTO mode'''
    MAV_ACTION_SET_AUTO = 13
    mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_SET_AUTO)

def cmd_ground(args, rl, mav_master):
    '''do a ground start mode'''
    MAV_ACTION_CALIBRATE_GYRO = 17
    mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_CALIBRATE_GYRO)

def cmd_rtl(args, rl, mav_master):
    '''set RTL mode'''
    MAV_ACTION_RETURN = 3
    mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_RETURN)

def cmd_manual(args, rl, mav_master):
    '''set MANUAL mode'''
    MAV_ACTION_SET_MANUAL = 12
    mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_SET_MANUAL)



def process_waypoint_request(m, mav_master):
    '''process a waypoint request from the master'''
    if (not status.loading_waypoints or
        time.time() > status.loading_waypoint_lasttime + 10.0):
        return
    if m.seq >= len(status.wpoints):
        print("Request for bad waypoint %u (max %u)" % (m.seq, len(status.wpoints)))
        return
    mav_master.mav.send(status.wpoints[m.seq])
    status.loading_waypoint_lasttime = time.time()
    if m.seq == len(status.wpoints) - 1:
        status.loading_waypoints = False
        print("Sent all %u waypoints" % len(status.wpoints))
    else:
        print("Sent waypoint %u : %s" % (m.seq, status.wpoints[m.seq]))


def read_waypoint_v100(line):
    '''read a version 100 waypoint'''
    cmdmap = {
        2 : mavlink.MAV_CMD_NAV_TAKEOFF,
        3 : mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        4 : mavlink.MAV_CMD_NAV_LAND,
        24: mavlink.MAV_CMD_NAV_TAKEOFF,
        26: mavlink.MAV_CMD_NAV_LAND,
        25: mavlink.MAV_CMD_NAV_WAYPOINT ,
        27: mavlink.MAV_CMD_NAV_LOITER_UNLIM
        }
    a = line.split()
    if len(a) != 13:
        raise RuntimeError("invalid waypoint line with %u values" % len(a))
    w = mavlink.MAVLink_waypoint_message(status.target_system, status.target_component,
                                         int(a[0]),    # seq
                                         int(a[1]),    # frame
                                         int(a[2]),    # action
                                         int(a[7]),    # current
                                         int(a[12]),   # autocontinue
                                         float(a[5]),  # param1,
                                         float(a[6]),  # param2,
                                         float(a[3]),  # param3
                                         float(a[4]),  # param4
                                         float(a[9]),  # x, latitude
                                         float(a[8]),  # y, longitude
                                         float(a[10])  # z
                                         )
    if not w.command in cmdmap:
        print("Unknown v100 waypoint action %u" % w.command)
        return None
    
    w.command = cmdmap[w.command]
    return w

def read_waypoint_v110(line):
    '''read a version 110 waypoint'''
    a = line.split()
    if len(a) != 12:
        raise RuntimeError("invalid waypoint line with %u values" % len(a))
    w = mavlink.MAVLink_waypoint_message(status.target_system, status.target_component,
                                         int(a[0]),    # seq
                                         int(a[2]),    # frame
                                         int(a[3]),    # command
                                         int(a[1]),    # current
                                         int(a[11]),   # autocontinue
                                         float(a[4]),  # param1,
                                         float(a[5]),  # param2,
                                         float(a[6]),  # param3
                                         float(a[7]),  # param4
                                         float(a[8]),  # x (latitude)
                                         float(a[9]),  # y (longitude)
                                         float(a[10])  # z (altitude)
                                         )
    return w


def load_waypoints(filename):
    '''load waypoints from a file'''
    f = open(filename, mode='r')
    version_line = f.readline().strip()
    if version_line == "QGC WPL 100":
        readfn = read_waypoint_v100
    elif version_line == "QGC WPL 110":
        readfn = read_waypoint_v110
    else:
        print("Unsupported waypoint format '%s'" % version_line)
        f.close()
        return

    status.wpoints = []

    for line in f:
        w = readfn(line)
        if w is not None:
            w.seq = len(status.wpoints)
            status.wpoints.append(w)
    f.close()
    print("Loaded %u waypoints from %s" % (len(status.wpoints), filename))

    mav_master.mav.waypoint_clear_all_send(status.target_system, status.target_component)
    if len(status.wpoints) == 0:
        return

    status.loading_waypoints = True
    status.loading_waypoint_lasttime = time.time()
    mav_master.mav.waypoint_count_send(status.target_system, status.target_component, len(status.wpoints))

def save_waypoints(filename):
    '''save waypoints to a file'''
    f = open(filename, mode='w')
    f.write("QGC WPL 110\n")
    for w in status.wpoints:
        f.write("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u\n" % (
            w.seq, w.current, w.frame, w.command,
            w.param1, w.param2, w.param3, w.param4,
            w.x, w.y, w.z, w.autocontinue))
    f.close()
    print("Saved %u waypoints to %s" % (len(status.wpoints), filename))
             

def cmd_wp(args, rl, mav_master):
    '''waypoint commands'''
    if len(args) < 1:
        print("usage: wp <list|load|save|set|clear>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: wp load <filename>")
            return
        load_waypoints(args[1])
    elif args[0] == "list":
        status.wp_op = "list"
        mav_master.mav.waypoint_request_list_send(status.target_system, status.target_component)
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: wp save <filename>")
            return
        status.wp_save_filename = args[1]
        status.wp_op = "save"
        mav_master.mav.waypoint_request_list_send(status.target_system, status.target_component)
    elif args[0] == "set":
        if len(args) != 2:
            print("usage: wp set <wpindex>")
            return
        mav_master.mav.waypoint_set_current_send(status.target_system, status.target_component, int(args[1]))
    elif args[0] == "clear":
        mav_master.mav.waypoint_clear_all_send(status.target_system, status.target_component)
    else:
        print("Usage: wp <list|load|save|set|clear>")


def param_save(filename, wildcard):
    '''save parameters to a file'''
    f = open(filename, mode='w')
    k = mav_param.keys()
    k.sort()
    count = 0
    for p in k:
        if p and fnmatch.fnmatch(str(p), wildcard):
            f.write("%-15.15s %f\n" % (p, mav_param[p]))
            count += 1
    f.close()
    print("Saved %u parameters to %s" % (count, filename))


def param_load_file(filename, wildcard, mav_master):
    '''load parameters from a file'''
    f = open(filename, mode='r')
    count = 0
    changed = 0
    for line in f:
        line = line.strip()
        if line[0] == "#":
            continue
        a = line.split()
        if len(a) != 2:
            print("Invalid line: %s" % line)
            continue
        if not fnmatch.fnmatch(a[0], wildcard):
            continue
        if math.fabs(mav_param[a[0]] - float(a[1])) > 0.000001:
            mav_master.mav.param_set_send(status.target_system,
                                          status.target_component, a[0], float(a[1]))
            changed += 1
            time.sleep(0.1)
            print("changed %s from %f to %f" % (a[0], mav_param[a[0]], float(a[1])))
        count += 1
    f.close()
    print("Loaded %u parameters from %s (changed %u)" % (count, filename, changed))
    

param_wildcard = "*"

def cmd_param(args, rl, mav_master):
    '''control parameters'''
    if len(args) < 1:
        print("usage: param <fetch|edit|set|show|store>")
        return
    if args[0] == "fetch":
        mav_master.mav.param_request_list_send(status.target_system, status.target_component)
        print("Requested parameter list")
    elif args[0] == "save":
        if len(args) < 2:
            print("usage: param save <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        param_save(args[1], param_wildcard)
    elif args[0] == "set":
        if len(args) != 3:
            print("Usage: param set PARMNAME VALUE")
            return
        param = args[1]
        value = args[2]
        if not param in mav_param:
            print("Unable to find parameter '%s'" % param)
            return
        mav_master.mav.param_set_send(status.target_system,
                                      status.target_component, param, float(value))
    elif args[0] == "load":
        if len(args) < 2:
            print("Usage: param load <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        param_load_file(args[1], param_wildcard, mav_master);
    elif args[0] == "show":
        if len(args) > 1:
            pattern = args[1]
        else:
            pattern = "*"
        k = mav_param.keys()
        k.sort()
        for p in k:
            if fnmatch.fnmatch(str(p), pattern):
                print("%-15.15s %f" % (p, mav_param[p]))
    elif args[0] == "store":
        MAV_ACTION_STORAGE_WRITE = 15
        mav_master.mav.action_send(status.target_system, status.target_component, MAV_ACTION_STORAGE_WRITE)
    else:
        print("Unknown subcommand '%s' (try 'fetch', 'save', 'set', 'show', 'load' or 'store')" % args[0]);


def cmd_status(args, rl, mav_master):
    '''show status'''
    status.show(sys.stdout)

def cmd_pwm(args, rl, mav_master):
    '''show PWM values'''
    status.show_pwm = not status.show_pwm


def cmd_setup(args, rl, mav_master):
    status.setup_mode = True
    rl.set_prompt("")


def cmd_reset(args, rl, mav_master):
    print("Resetting master")
    mav_master.reset()

command_map = {
    'roll'    : (cmd_roll,     'set fixed roll PWM'),
    'pitch'   : (cmd_pitch,    'set fixed pitch PWM'),
    'rudder'  : (cmd_rudder,   'set fixed rudder PWM'),
    'throttle': (cmd_throttle, 'set fixed throttle PWM'),
    'switch'  : (cmd_switch,   'set RC switch (1-5), 0 disables'),
    'rc'      : (cmd_rc,       'override a RC channel value'),
    'wp'      : (cmd_wp,       'waypoint management'),
    'param'   : (cmd_param,    'manage APM parameters'),
    'setup'   : (cmd_setup,    'go into setup mode'),
    'reset'   : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'd'       : (cmd_disarm,   'disarm the motors'),
    'disarm'  : (cmd_disarm,   'disarm the motors'),
    'arm'     : (cmd_arm,      'arm the motors'),
    'status'  : (cmd_status,   'show status'),
    'pwm'     : (cmd_pwm,      'show PWM input'),
    'trim'    : (cmd_trim,     'trim aileron, elevator and rudder to current values'),
    'auto'    : (cmd_auto,     'set AUTO mode'),
    'ground'  : (cmd_ground,   'do a ground start'),
    'loiter'  : (cmd_loiter,   'set LOITER mode'),
    'rtl'     : (cmd_rtl,      'set RTL mode'),
    'manual'  : (cmd_manual,   'set MANUAL mode'),
    };

def process_stdin(rl, line, mav_master):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)
    line = line.strip()

    if status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            status.setup_mode = False
            rl.set_prompt("MAV> ")
            return
        rl.add_history(line)
        mav_master.write(line + '\r')
        return

    if not line:
        return

    rl.add_history(line)
    args = line.split(" ")
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
    fn(args[1:], rl, mav_master)


class mavfd(object):
    '''a generic mavlink port'''
    def __init__(self, fd, address):
        self.fd = fd
        self.address = address


class mavserial(mavfd):
    '''a serial mavlink port'''
    def __init__(self, device, baud=115200):
        import serial
        self.baud = baud
        self.device = device
        self.port = serial.Serial(self.device, self.baud, timeout=0, dsrdtr=not opts.nodtr)

        mavfd.__init__(self, self.port.fileno(), device)

        if opts.nodtr:
            # prevent DTR reset on close
            import termios
            tattr = termios.tcgetattr(self.fd)
            tattr[2] &= ~termios.HUPCL
            termios.tcsetattr(self.fd, termios.TCSANOW, tattr)

        self.mav = mavlink.MAVLink(self, srcSystem=opts.SOURCE_SYSTEM)
        self.mav.robust_parsing = True
        self.logfile = None
        self.logfile_raw = None

    def read(self):
        return self.port.read()

    def recv(self):
        return self.read()

    def write(self, buf):
        try:
            return self.port.write(buf)
        except OSError:
            self.reset()
            return -1

    def reset(self):
        import serial
        self.port.close()
        while True:
            try:
                self.port = serial.Serial(self.device, self.baud, timeout=0, dsrdtr=not opts.nodtr)
                self.fd = self.port.fileno()
                if opts.nodtr:
                    # prevent DTR reset on close
                    import termios
                    tattr = termios.tcgetattr(self.fd)
                    tattr[2] &= ~termios.HUPCL
                    termios.tcsetattr(self.fd, termios.TCSANOW, tattr)
                return
            except Exception, msg:
                print("Failed to reopen %s - %s" % (self.device, msg))
                time.sleep(1)
        

class mavudp(mavfd):
    '''a UDP mavlink socket'''
    def __init__(self, device, input=False):
        a = device.split(':')
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if input:
            self.port.bind((a[0], int(a[1])))
            self.connected = False
        else:
            self.port.connect((a[0], int(a[1])))
            self.connected = True
        self.port.setblocking(0)
        self.last_address = None
        mavfd.__init__(self, self.port.fileno(), device)
        self.mav = mavlink.MAVLink(self, srcSystem=opts.SOURCE_SYSTEM)
    def recv(self):
        data, self.last_address = self.port.recvfrom(300)
        return data
    def write(self, buf):
        try:
            if self.connected:
                self.port.send(buf)
            else:
                self.port.sendto(buf, self.last_address)
        except socket.error:
            pass

def scale_rc(servo, min, max, min_pwm=1000, max_pwm=2000, param=None):
    '''scale a PWM value'''
    # default to servo range of 1000 to 2000
    if param:
        min_pwm  = get_mav_param('%s_MIN'  % param, min_pwm)
        max_pwm  = get_mav_param('%s_MAX'  % param, max_pwm)
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
    

def all_printable(buf):
    '''see if a string is all printable'''
    for c in buf:
        if not ascii.isprint(c) and not c in ['\r', '\n', '\t']:
            return False
    return True

def system_check():
    '''check that the system is ready to fly'''
    ok = True

    if not 'GPS_RAW' in status.msgs:
        say("WARNING no GPS status",'important')
        return
    
    if status.msgs['GPS_RAW'].fix_type != 2:
        say("WARNING no GPS lock",'important')
        ok = False

    if not 'PITCH_MIN' in mav_param:
        say("WARNING no pitch parameter available",'important')
        return
        
    if int(mav_param['PITCH_MIN']) > 1300:
        say("WARNING PITCH MINIMUM not set",'important')
        ok = False

    if not 'ATTITUDE' in status.msgs:
        say("WARNING no attitude recorded",'important')
        return

    if math.fabs(status.msgs['ATTITUDE'].pitch) > math.radians(5):
        say("WARNING pitch is %u degrees" % math.degrees(status.msgs['ATTITUDE'].pitch),'important')
        ok = False

    if math.fabs(status.msgs['ATTITUDE'].roll) > math.radians(5):
        say("WARNING roll is %u degrees" % math.degrees(status.msgs['ATTITUDE'].roll),'important')
        ok = False

    if ok:
        say("All OK SYSTEM READY TO FLY",'important')


def mode_string(mode, nav_mode):
    '''work out autopilot mode'''
    MAV_MODE_MANUAL = 2
    MAV_MODE_GUIDED = 3
    MAV_MODE_AUTO = 4
    MAV_MODE_TEST1 = 5
    MAV_MODE_TEST2 = 6
    MAV_MODE_TEST3 = 7
    MAV_NAV_LIFTOFF = 1
    MAV_NAV_HOLD = 2
    MAV_NAV_WAYPOINT = 3
    MAV_NAV_VECTOR = 4
    MAV_NAV_RETURNING = 5
    MAV_NAV_LANDING = 6
    MAV_NAV_LOST = 7
    MAV_NAV_LOITER = 8
    cmode = (mode, nav_mode)
    if opts.quadcopter:
        mapping = {
            (100, 4)    : "STABILIZE",
            (102, 4)    : "SIMPLE",
            (101, 4)    : "ACRO",
            (4,   2)    : "LOITER",
            (4,   5)    : "RTL",
            (4,   3)    : "AUTO",
            }
    else:
        mapping = {
            (MAV_MODE_MANUAL, MAV_NAV_VECTOR)    : "MANUAL",
            (MAV_MODE_TEST3,  MAV_NAV_VECTOR)    : "CIRCLE",
            (MAV_MODE_GUIDED, MAV_NAV_VECTOR)    : "GUIDED",
            (MAV_MODE_TEST1,  MAV_NAV_VECTOR)    : "STABILIZE",
            (MAV_MODE_TEST2,  MAV_NAV_LIFTOFF)   : "FBWA",
            (MAV_MODE_AUTO,   MAV_NAV_WAYPOINT)  : "AUTO",
            (MAV_MODE_AUTO,   MAV_NAV_RETURNING) : "RTL",
            (MAV_MODE_AUTO,   MAV_NAV_LOITER)    : "LOITER",
            (MAV_MODE_AUTO,   MAV_NAV_LIFTOFF)   : "TAKEOFF",
            (MAV_MODE_AUTO,   MAV_NAV_LANDING)   : "LANDING",
            }
    if cmode in mapping:
        return mapping[cmode]
    return "Mode(%s,%s)" % cmode

def beep():
    f = open("/dev/tty", mode="w")
    f.write(chr(7))
    f.close()

def battery_report():
    '''report battery level'''
    if not 'SYS_STATUS' in status.msgs:
        return

    if opts.num_cells == 0:
        return

    voltage = status.msgs['SYS_STATUS'].vbat / (opts.num_cells * 1000.0)
    levels = [ 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1 ]
    battery_level = 100
    for i in range(0, len(levels)):
        if voltage <= levels[i]:
            battery_level = 10 * i
            break
    if battery_level != status.last_battery_announce:
        say("Battery %u percent" % battery_level,'notification')
        status.last_battery_announce = battery_level
    if battery_level <= 20:
        say("battery warning",'important')

    

def master_callback(m, master, recipients):
    '''process mavlink message m on master, sending any messages to recipients'''

    status.counters['MasterIn'] += 1

    mtype = m.get_type()
    if mtype == 'HEARTBEAT':
        if (status.target_system != m.get_srcSystem() or
            status.target_component != m.get_srcComponent()):
            status.target_system = m.get_srcSystem()
            status.target_component = m.get_srcComponent()
            say("online system %u component %u" % (status.target_system, status.target_component),'message')
    elif mtype == 'STATUSTEXT':
        print("APM: %s" % m.text)
    elif mtype == 'PARAM_VALUE':
        mav_param[str(m.param_id)] = m.param_value
        if m.param_index+1 == m.param_count:
            print("Received %u parameters" % m.param_count)

    elif mtype == 'SERVO_OUTPUT_RAW':
        if opts.quadcopter:
            status.rc_throttle[0] = scale_rc(m.servo1_raw, 0.0, 1.0, param='RC3')
            status.rc_throttle[1] = scale_rc(m.servo2_raw, 0.0, 1.0, param='RC3')
            status.rc_throttle[2] = scale_rc(m.servo3_raw, 0.0, 1.0, param='RC3')
            status.rc_throttle[3] = scale_rc(m.servo4_raw, 0.0, 1.0, param='RC3')
        else:
            status.rc_aileron  = scale_rc(m.servo1_raw, -1.0, 1.0, param='RC1')
            status.rc_elevator = scale_rc(m.servo2_raw, -1.0, 1.0, param='RC2')
            status.rc_throttle = scale_rc(m.servo3_raw, 0.0, 1.0, param='RC3')
            status.rc_rudder   = scale_rc(m.servo4_raw, -1.0, 1.0, param='RC4')
            if status.rc_throttle < 0.1:
                status.rc_throttle = 0

    elif mtype == 'WAYPOINT_COUNT' and status.wp_op != None:
        status.wpoints = [None]*m.count
        print("Requesting %u waypoints" % m.count)
        mav_master.mav.waypoint_request_send(status.target_system, status.target_component, 0)

    elif mtype == 'WAYPOINT' and status.wp_op != None:
        status.wpoints[m.seq] = m
        if m.seq+1 < len(status.wpoints):
            mav_master.mav.waypoint_request_send(status.target_system, status.target_component, m.seq+1)
        else:
            if status.wp_op == 'list':
                for w in status.wpoints:
                    print("%u %u %.10f %.10f %f" % (w.command, w.frame, w.x, w.y, w.z))
            elif status.wp_op == "save":
                save_waypoints(status.wp_save_filename)
            status.wp_op = None

    elif mtype == "RC_CHANNELS_RAW" and status.show_pwm:
        print(m)

    elif mtype == "WAYPOINT_REQUEST":
        process_waypoint_request(m, master)

    elif mtype == "WAYPOINT_CURRENT":
        if m.seq != status.last_waypoint:
            status.last_waypoint = m.seq
            say("waypoint %u" % m.seq,'message')

    elif mtype == "SYS_STATUS":
        mstring = mode_string(m.mode, m.nav_mode)
        if mstring != status.mode_string:
            status.mode_string = mstring
            rl.set_prompt(mstring + "> ")
            say("Mode " + mstring,'important')

    elif (mtype == "VFR_HUD"
          and 'GPS_RAW' in status.msgs
          and status.msgs['GPS_RAW'].fix_type == 2):
        if status.first_altitude == 0:
            status.first_altitude = m.alt
            status.last_altitude_announce = 0.0
            say("GPS lock at %u meters" % m.alt,'notification')
        else:
            if m.alt < status.first_altitude:
                status.first_altitude = m.alt
                status.last_altitude_announce = m.alt
            if math.fabs(m.alt - status.last_altitude_announce) >= 10.0:
                status.last_altitude_announce = m.alt
                rounded_alt = 10 * ((5+int(m.alt - status.first_altitude)) / 10)
                say("%u meters" % rounded_alt,'notification')

    elif mtype == "RC_CHANNELS_RAW":
        if (m.chan7_raw > 1700 and status.mode_string == "MANUAL"):
            system_check()

    elif mtype == "BAD_DATA":
        if all_printable(m.data):
            sys.stdout.write(m.data)
            sys.stdout.flush()
    elif mtype in [ 'HEARTBEAT', 'GLOBAL_POSITION', 'RC_CHANNELS_SCALED',
                    'ATTITUDE', 'RC_CHANNELS_RAW', 'GPS_STATUS', 'WAYPOINT_CURRENT',
                    'SERVO_OUTPUT_RAW', 'VFR_HUD',
                    'GLOBAL_POSITION_INT', 'RAW_PRESSURE', 'RAW_IMU', 'WAYPOINT_ACK',
                    'NAV_CONTROLLER_OUTPUT', 'GPS_RAW', 'WAYPOINT' ]:
        pass
    else:
        print("Got MAVLink msg: %s" % m)

    # keep the last message of each type around
    status.msgs[m.get_type()] = m
    if not m.get_type() in status.msg_count:
        status.msg_count[m.get_type()] = 0
    status.msg_count[m.get_type()] += 1

    # also send the message on to all the slaves
    if mtype != "BAD_DATA":
        for r in recipients:
            r.write(m.get_msgbuf())

    # and log them
    if master.logfile and mtype != "BAD_DATA":
        master.logfile.write(struct.pack('>Q', get_usec()))
        master.logfile.write(m.get_msgbuf())


def process_master(m):
    '''process packets from the MAVLink master'''
    s = m.recv()
    if m.logfile_raw:
        m.logfile_raw.write(s)

    if status.setup_mode:
        sys.stdout.write(s)
        sys.stdout.flush()
        return

    for c in s:
        msg = m.mav.parse_char(c)
        if msg and msg.get_type() == "BAD_DATA":
            if opts.show_errors:
                print("MAV error: %s" % msg)
            status.mav_error += 1

    

def process_mavlink(slave, master):
    '''process packets from MAVLink slaves, forwarding to the master'''
    try:
        buf = slave.recv()
    except socket.error:
        return
    try:
        m = slave.mav.decode(buf)
    except mavlink.MAVError, msg:
        print("Bad MAVLink slave message from %s: %s" % (slave.address, msg))
        return
    if not status.setup_mode:
        master.write(m.get_msgbuf())
    status.counters['Slave'] += 1

def send_flightgear_controls(fg):
    '''send control values to flightgear'''
    status.counters['FGearOut'] += 1
    if opts.quadcopter:
        buf = struct.pack('>ddddI',
                          status.rc_throttle[0], # right
                          status.rc_throttle[1], # left
                          status.rc_throttle[2], # front
                          status.rc_throttle[3], # back
                          0x4c56414d)
    else:
        buf = struct.pack('>ddddI', status.rc_aileron, status.rc_elevator,
                          status.rc_rudder, status.rc_throttle, 0x4c56414d)
    fg.write(buf)
    


def process_flightgear(m, master):
    '''process flightgear protocol input'''
    buf = m.recv()
    if len(buf) == 0:
        return
    # see MAVLink.xml for the protocol format
    try:
        (latitude, longitude, altitude, heading,
         speedN, speedE,
         xAccel, yAccel, zAccel,
         rollRate, pitchRate, yawRate,
         rollDeg, pitchDeg, yawDeg,
         airspeed, magic) = struct.unpack('>ddddddddddddddddI', buf)
    except struct.error, msg:
        print("Bad flightgear input of length %u: %s" % (len(buf), msg))
        return
    if magic != 0x4c56414d:
        print("Bad flightgear magic 0x%08x should be 0x4c56414d" % magic)
        return
    if altitude < 0:
        # the first packet from flightgear is sometimes rubbish
        return

    status.counters['FGearIn'] += 1

    if yawDeg == 0.0:
        # not all planes give a yaw value
        yawDeg = heading

    if status.setup_mode:
        return

    # send IMU data to the master
    status.counters['MasterOut'] += 1
    master.mav.attitude_send(get_usec(),
                             deg2rad(rollDeg), deg2rad(pitchDeg), deg2rad(yawDeg),
                             deg2rad(rollRate),deg2rad(pitchRate),deg2rad(yawRate))

    groundspeed = ft2m(math.sqrt((speedN * speedN) + (speedE * speedE)))

    if math.isnan(heading):
        heading = 0.0

    # and airspeed
    status.counters['MasterOut'] += 1
    if opts.quadcopter:
        master.mav.vfr_hud_send(kt2mps(airspeed), groundspeed, int(heading),
                                int(status.rc_throttle[0]*100), ft2m(altitude), 0)
    else:
        master.mav.vfr_hud_send(kt2mps(airspeed), groundspeed, int(heading),
                                int(status.rc_throttle*100), ft2m(altitude), 0)

    # remember GPS fix, we send this at opts.gpsrate
    status.gps = mavlink.MAVLink_gps_raw_message(get_usec(),
                                                 3, # we have a 3D fix
                                                 latitude, longitude,
                                                 ft2m(altitude),
                                                 0, # no uncertainty
                                                 0, # no uncertainty
                                                 ft2m(math.sqrt((speedN * speedN) +
                                                                (speedE * speedE))),
                                                 heading);

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
                             

def open_logs(mav_master):
    '''open log files'''
    if opts.append_log:
        mode = 'a'
    else:
        mode = 'w'
    logfile = opts.logfile
    if opts.aircraft is not None:
        dirname = "%s/logs/%s" % (opts.aircraft, time.strftime("%Y-%m-%d"))
        mkdir_p(dirname)
        for i in range(1, 10000):
            fdir = os.path.join(dirname, 'flight%u' % i)
            if not os.path.exists(fdir):
                break
        if os.path.exists(fdir):
            print("Flight logs full")
            sys.exit(1)
        mkdir_p(fdir)
        print(fdir)
        logfile = os.path.join(fdir, logfile)
    print("Logging to %s" % logfile)
    mav_master.logfile = open(logfile, mode=mode)
    mav_master.logfile_raw = open(logfile+'.raw', mode=mode)


# master mavlink device
mav_master = None

# mavlink outputs
mav_outputs = []

# flightgear input
fg_input = None

# flightgear output
fg_output = None


def main_loop():
    '''main processing loop'''
    fg_period = periodic_event(opts.fgrate)
    gps_period = periodic_event(opts.gpsrate)
    status_period = periodic_event(1.0)
    msg_period = periodic_event(1.0)
    heartbeat_period = periodic_event(0.5)
    battery_period = periodic_event(0.1)

    while True:
        rin = [0, mav_master.fd]
        for m in mav_outputs:
            rin.append(m.fd)
        if fg_input:
            rin.append(fg_input.fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], 0.001)
        except select.error, (errno, msg):
            continue
        for fd in rin:
            if fd == 0:
                rl.read_char()
            if fd == mav_master.fd:
                process_master(mav_master)
            for m in mav_outputs:
                if fd == m.fd:
                    process_mavlink(m, mav_master)
            if fg_input and fd == fg_input.fd:
                process_flightgear(fg_input, mav_master)

        if (status.setup_mode or
            status.target_system == -1 or
            status.target_component == -1):
            continue
        
        if fg_output and fg_period.trigger():
            send_flightgear_controls(fg_output)

        if status.gps and gps_period.trigger():
            status.counters['MasterOut'] += 1
            mav_master.mav.send(status.gps)

        if status_period.trigger():
            status.write()

        if heartbeat_period.trigger():
            status.counters['MasterOut'] += 1
            MAV_GROUND = 5
            MAV_AUTOPILOT_NONE = 4
            MAVLINK_VERSION = 2
            mav_master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE, MAVLINK_VERSION)

        if msg_period.trigger():
            status.counters['MasterOut'] += 1
            mav_master.mav.request_data_stream_send(status.target_system, status.target_component,
                                                    mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            if len(mav_param) == 0:
                status.counters['MasterOut'] += 1
                mav_master.mav.param_request_list_send(status.target_system, status.target_component)
 
        if battery_period.trigger():
            battery_report()
            
       

if __name__ == '__main__':

    from optparse import OptionParser
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master",dest="master", help="MAVLink master port")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="master port baud rate", default=115200)
    parser.add_option("--in",    dest="input",  help="MAVLink input port",
                      action='append')
    parser.add_option("--out",   dest="output", help="MAVLink output port",
                      action='append', default=[])
    parser.add_option("--fgin",  dest="fgin",   help="flightgear input")
    parser.add_option("--fgout", dest="fgout",  help="flightgear output")
    parser.add_option("--fgrate",dest="fgrate", default=50.0, type='float',
                      help="flightgear update rate")
    parser.add_option("--gpsrate",dest="gpsrate", default=4.0, type='float',
                      help="GPS update rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=-1, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=-1, help='MAVLink target master component')
    parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
                      default='mav.log')
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
    
    
    (opts, args) = parser.parse_args()

    if not opts.master:
        parser.error("You must specify a MAVLink master serial port")

    # container for status information
    status = status()
    status.target_system = opts.TARGET_SYSTEM
    status.target_component = opts.TARGET_COMPONENT

    # open master link
    if opts.master.find(':') != -1:
        mav_master = mavudp(opts.master, input=True)
    else:
        mav_master = mavserial(opts.master, baud=opts.baudrate)
    mav_master.mav.set_callback(master_callback, mav_master, mav_outputs)

    # log all packets from the master, for later replay
    open_logs(mav_master)

    # open any mavlink UDP ports
    for p in opts.output:
        mav_outputs.append(mavudp(p))

    # open any flightgear UDP ports
    if opts.fgin:
        fg_input = mavudp(opts.fgin, input=True)
    if opts.fgout:
        fg_output = mavudp(opts.fgout)
    
    rl = rline(process_stdin, "MAV> ", mav_master)
    if opts.setup:
        rl.set_prompt("")
    try:
        main_loop()
        rl.cleanup()
    except KeyboardInterrupt:
        rl.cleanup()
        sys.exit(1)
    except Exception:
        rl.cleanup()
        raise
    
