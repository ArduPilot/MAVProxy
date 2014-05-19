#!/usr/bin/env python
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

'''

import sys, os, struct, math, time, socket
import fnmatch, errno, threading
import serial, Queue, select
import traceback
import select

# allow running without installing
#sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))


from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import dumpstacks

class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        self.gps	 = None
        self.msgs = {}
        self.msg_count = {}
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.setup_mode = opts.setup
        self.mav_error = 0
        self.target_system = 1
        self.target_component = 1
        self.altitude = 0
        self.last_altitude_announce = 0.0
        self.last_distance_announce = 0.0
        self.last_battery_announce = 0
        self.last_avionics_battery_announce = 0
        self.battery_level = -1
        self.voltage_level = -1
        self.avionics_battery_level = -1
        self.exit = False
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        self.logdir = None
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = False
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.highest_msec = 0
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        self.watch = None
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        self.last_seq = 0
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

def say_text(text, priority='important'):
    '''text output - default function for say()'''
    mpstate.console.writeln(text)

def say(text, priority='important'):
    '''text and/or speech output'''
    mpstate.functions.say(text, priority)

def add_input(cmd, immediate=False):
    '''add some command input to be processed'''
    if immediate:
        process_stdin(cmd)
    else:
        mpstate.input_queue.put(cmd)

class MAVFunctions(object):
    '''core functions available in modules'''
    def __init__(self):
        self.process_stdin = add_input
        self.param_set = param_set
        self.get_mav_param = get_mav_param
        self.say = say_text

class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.map = None
        self.map_functions = {}
        self.vehicle_type = None
        self.vehicle_name = None
        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        self.settings = MPSettings(
            [ MPSetting('link', int, 1, 'Primary Link', tab='Link', range=(0,4), increment=1),
              MPSetting('streamrate', int, 4, 'Stream rate link1', range=(0,20), increment=1),
              MPSetting('streamrate2', int, 4, 'Stream rate link2', range=(0,20), increment=1),
              MPSetting('heartbeat', int, 1, 'Heartbeat rate', range=(0,5), increment=1),
              MPSetting('mavfwd', bool, True, 'Allow forwarded control'),
              MPSetting('mavfwd_rate', bool, False, 'Allow forwarded rate control'),
              MPSetting('shownoise', bool, True, 'Show non-MAVLink data'),
              
              MPSetting('altreadout', int, 10, 'Altitude Readout',
                        range=(0,100), increment=1, tab='Announcements'),
              MPSetting('distreadout', int, 200, 'Distance Readout', range=(0,10000), increment=1),

              MPSetting('moddebug', int, 0, 'Module Debug Level', range=(0,3), increment=1, tab='Debug'),
              MPSetting('numcells', int, 1, range=(0,10), increment=1),
              MPSetting('flushlogs', bool, False, 'Flush logs on every packet'),
              MPSetting('requireexit', bool, False, 'Require exit command'),

              MPSetting('basealt', int, 0, 'Base Altitude', range=(0,30000), increment=1, tab='Altitude'),
              MPSetting('wpalt', int, 100, 'Default WP Altitude', range=(0,10000), increment=1),
              MPSetting('rallyalt', int, 90, 'Default Rally Altitude', range=(0,10000), increment=1)]
            )

        self.completions = {
            "script" : ["(FILENAME)"],
            "set"    : ["(SETTING)"]
            }

        self.status = MPStatus()

        # master mavlink device
        self.mav_master = None

        # mavlink outputs
        self.mav_outputs = []

        # SITL output
        self.sitl_output = None

        self.mav_param = mavparm.MAVParmDict()
        self.modules = []
        self.public_modules = {}
        self.functions = MAVFunctions()
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}

    def module(self, name):
        '''Find a public module (most modules are private)'''
        if name in self.public_modules:
            return self.public_modules[name]
        return None
    
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

def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)

def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)

def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print("usage: script <filename>")
        return

    run_script(args[0])

def cmd_set(args):
    '''control mavproxy options'''
    mpstate.settings.command(args)

def cmd_status(args):
    '''show status'''
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern)

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

def load_module(modname, quiet=False):
    '''load a module'''
    modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if not quiet:
                print("module %s already loaded" % modname)
            return False
    for modpath in modpaths:
        try:
            m = import_package(modpath)
            reload(m)
            module = m.init(mpstate)
            if isinstance(module, mp_module.MPModule):
                mpstate.modules.append((module, m))
                if not quiet:
                    print("Loaded module %s" % (modname,))
                return True
            else:
                ex = "%s.init did not return a MPModule instance" % modname
                break
        except ImportError as msg:
            ex = msg
            if mpstate.settings.moddebug > 1:
                import traceback
                print(traceback.format_exc())
    print("Failed to load module: %s" % ex)
    return False

def unload_module(modname):
    '''unload a module'''
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if hasattr(m, 'unload'):
                m.unload()
            mpstate.modules.remove((m,pm))
            print("Unloaded module %s" % modname)
            return True
    print("Unable to find module %s" % modname)
    return False

def cmd_module(args):
    '''module commands'''
    usage = "usage: module <list|load|reload|unload>"
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        for (m,pm) in mpstate.modules:
            print("%s: %s" % (m.name, m.description))
    elif args[0] == "load":
        if len(args) < 2:
            print("usage: module load <name>")
            return
        load_module(args[1])
    elif args[0] == "reload":
        if len(args) < 2:
            print("usage: module reload <name>")
            return
        modname = args[1]
        pmodule = None
        for (m,pm) in mpstate.modules:
            if m.name == modname:
                pmodule = pm
        if pmodule is None:
            print("Module %s not loaded" % modname)
            return
        if unload_module(modname):
            reload(pmodule)
            if load_module(modname, quiet=True):
                print("Reloaded module %s" % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print("usage: module unload <name>")
            return
        modname = os.path.basename(args[1])
        unload_module(modname)
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
    'script'  : (cmd_script,   'run a script of MAVProxy commands'),
    'setup'   : (cmd_setup,    'go into setup mode'),
    'reset'   : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'status'  : (cmd_status,   'show status'),
    'set'     : (cmd_set,      'mavproxy settings'),
    'link'    : (cmd_link,     'show link status'),
    'watch'   : (cmd_watch,    'watch a MAVLink pattern'),
    'module'  : (cmd_module,   'module commands'),
    'alias'   : (cmd_alias,    'command aliases')
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
    if cmd == 'exit' and mpstate.settings.requireexit:
        reply = raw_input("Are you sure you want to exit? (y/N)")
        if reply.lower() == 'y':
            mpstate.status.exit = True
        return

    if not cmd in command_map:
        for (m,pm) in mpstate.modules:
            if hasattr(m, 'unknown_command'):
                try:
                    if m.unknown_command(args):
                        return
                except Exception as e:
                    print("ERROR in command: %s" % str(e))
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command: %s" % str(e))
        if mpstate.settings.moddebug > 1:
            traceback.print_exc()


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
    mpstate.status.voltage_level = SYS_STATUS.voltage_battery

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
    batt_mon = get_mav_param('BATT_MONITOR',0)

    #report voltage level only 
    if batt_mon == 3:
        mpstate.console.set_status('Battery', 'Batt: %.2fV' % (float(mpstate.status.voltage_level) / 1000.0), row=1)
    elif batt_mon == 4:
        mpstate.console.set_status('Battery', 'Batt: %u%%/%.2fV' % (mpstate.status.battery_level, (float(mpstate.status.voltage_level) / 1000.0)), row=1)

        rbattery_level = int((mpstate.status.battery_level+5)/10)*10

        if rbattery_level != mpstate.status.last_battery_announce:
            say("Flight battery %u percent" % rbattery_level, priority='notification')
            mpstate.status.last_battery_announce = rbattery_level
        if rbattery_level <= 20:
            say("Flight battery warning")

    else:
        #clear battery status
        mpstate.console.set_status('Battery')

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

        if master.flightmode != mpstate.status.flightmode and time.time() > mpstate.status.last_mode_announce + 2:
            mpstate.status.flightmode = master.flightmode
            mpstate.status.last_mode_announce = time.time()
            mpstate.rl.set_prompt(mpstate.status.flightmode + "> ")
            say("Mode " + mpstate.status.flightmode)

        if m.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING]:
            mpstate.vehicle_type = 'plane'
            mpstate.vehicle_name = 'ArduPlane'
        elif m.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                        mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                        mavutil.mavlink.MAV_TYPE_SUBMARINE]:
            mpstate.vehicle_type = 'rover'
            mpstate.vehicle_name = 'APMrover2'
        elif m.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER]:
            mpstate.vehicle_type = 'copter'
            mpstate.vehicle_name = 'ArduCopter'
        elif m.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
            mpstate.vehicle_type = 'antenna'
            mpstate.vehicle_name = 'AntennaTracker'
        
    elif mtype == 'STATUSTEXT':
        if m.text != mpstate.status.last_apm_msg or time.time() > mpstate.status.last_apm_msg_time+2:
            mpstate.console.writeln("APM: %s" % m.text, bg='red')
            mpstate.status.last_apm_msg = m.text
            mpstate.status.last_apm_msg_time = time.time()

    elif mtype == "SYS_STATUS":
        battery_update(m)

    elif mtype == "VFR_HUD":
        have_gps_fix = False
        if 'GPS_RAW' in mpstate.status.msgs and mpstate.status.msgs['GPS_RAW'].fix_type == 2:
            have_gps_fix = True
        if 'GPS_RAW_INT' in mpstate.status.msgs and mpstate.status.msgs['GPS_RAW_INT'].fix_type == 3:
            have_gps_fix = True
        if have_gps_fix and not mpstate.status.have_gps_lock and m.alt != 0:
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
    
    elif mtype == "GLOBAL_POSITION_INT":
        report_altitude(m.relative_alt*0.001)

    elif mtype == "COMPASSMOT_STATUS":
        print(m)

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
        for (mod,pm) in mpstate.modules:
            if not hasattr(mod, 'mavlink_packet'):
                continue
            try:
                mod.mavlink_packet(m)
            except Exception as msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)

def process_master(m):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        time.sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        time.sleep(0.1)
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
        if mpstate.settings.flushlogs:
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
        if opts.mission is not None:
            print(opts.mission)
            dirname = "%s/logs/%s/Mission%s" % (opts.aircraft, time.strftime("%Y-%m-%d"), opts.mission)
        else:
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

    if battery_period.trigger():
        battery_report()

    # call optional module idle tasks. These are called at several hundred Hz
    for (m,pm) in mpstate.modules:
        if hasattr(m, 'idle_task'):
            try:
                m.idle_task()
            except Exception as msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)


def main_loop():
    '''main processing loop'''
    if not mpstate.status.setup_mode and not opts.nowait:
        for master in mpstate.mav_master:
            send_heartbeat(master)
            master.wait_heartbeat()
        set_stream_rates()

    while True:
        if mpstate is None or mpstate.status.exit:
            return
        while not mpstate.input_queue.empty():
            line = mpstate.input_queue.get()
            cmds = line.split(';')
            for c in cmds:
                process_stdin(c)

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
            (rin, win, xin) = select.select(rin, [], [], 0.01)
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
                except Exception as msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)



def input_loop():
    '''wait for user input'''
    while mpstate.status.exit != True:
        try:
            if mpstate.status.exit != True:
                line = raw_input(mpstate.rl.prompt)
        except EOFError:
            mpstate.status.exit = True
            sys.exit(1)
        mpstate.input_queue.put(line)


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

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink master port and optional baud rate",
                      default=[])
    parser.add_option("--out", dest="output", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink output port and optional baud rate",
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=115200)
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
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")
    parser.add_option("--mission", dest="mission", help="mission name", default=None)

    (opts, args) = parser.parse_args()

    # warn people about ModemManager which interferes badly with APM and Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print("WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk")

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    from pymavlink import mavutil, mavparm
    mavutil.set_dialect(opts.dialect)

    # global mavproxy state
    mpstate = MPState()
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode

    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        load_module('speech')

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
        if ',' in mdev and not os.path.exists(mdev):
            port, baud = mdev.split(',')
        else:
            port, baud = mdev, opts.baudrate

        m = mavutil.mavlink_connection(port, autoreconnect=True, baud=int(baud))
        m.mav.set_callback(master_callback, m)
        if hasattr(m.mav, 'set_send_callback'):
            m.mav.set_send_callback(master_send_callback, m)
        if opts.rtscts:
            m.set_rtscts(True)
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

    # open any mavlink UDP ports
    for p in opts.output:
        if ',' in p and not os.path.exists(p):
            port, baud = p.split(',')            
        else:
            port, baud = p, opts.baudrate

        mpstate.mav_outputs.append(mavutil.mavlink_connection(port, baud=int(baud), input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.numcells = opts.num_cells
    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    msg_period = mavutil.periodic_event(1.0/15)
    heartbeat_period = mavutil.periodic_event(1)
    battery_period = mavutil.periodic_event(0.1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.input_queue = Queue.Queue()
    mpstate.rl = rline.rline("MAV> ", mpstate)
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
        standard_modules = ['log', 'wp', 'rally','fence','param','relay',
                            'tuneopt','arm','mode','calibration','rc','auxopt','misc']
        for m in standard_modules:
            load_module(m, quiet=True)

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
    while (mpstate.status.exit != True):
        try:
            input_loop()
        except KeyboardInterrupt:
            if mpstate.settings.requireexit:
                print("Interrupt caught.  Use 'exit' to quit MAVProxy.")

                #Just lost the map and console, get them back:
                for (m,pm) in mpstate.modules:
                    if m.name in ["map", "console"]:
                        if hasattr(m, 'unload'):
                            try:
                                m.unload()
                            except Exception:
                                pass
                        reload(m)
                        m.init(mpstate)   

            else:
                mpstate.status.exit = True
                sys.exit(1)

    #this loop executes after leaving the above loop and is for cleanup on exit
    for (m,pm) in mpstate.modules:
        if hasattr(m, 'unload'):
            print("Unloading module %s" % m.name)
            m.unload()
        
    sys.exit(1)
