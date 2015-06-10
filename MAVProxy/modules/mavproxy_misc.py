#!/usr/bin/env python
'''miscellaneous commands'''

import time, math
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module


from os import kill
from signal import signal
from subprocess import PIPE, Popen

class RepeatCommand(object):
    '''repeated command object'''
    def __init__(self, period, cmd):
        self.period = period
        self.cmd = cmd
        self.event = mavutil.periodic_event(1.0/period)

    def __str__(self):
        return "Every %.1f seconds: %s" % (self.period, self.cmd)
        

def run_command(args, cwd = None, shell = False, timeout = None, env = None):
    '''
    Run a shell command with a timeout.
    See http://stackoverflow.com/questions/1191374/subprocess-with-timeout
    '''
    from subprocess import PIPE, Popen
    from StringIO import StringIO
    import fcntl, os, signal
    p = Popen(args, shell = shell, cwd = cwd, stdout = PIPE, stderr = PIPE, env = env)
    tstart = time.time()
    buf = StringIO()

    # try to make it non-blocking
    try:
        fcntl.fcntl(p.stdout, fcntl.F_SETFL, fcntl.fcntl(p.stdout, fcntl.F_GETFL) | os.O_NONBLOCK)
    except Exception:
        pass

    while True:
        time.sleep(0.1)
        retcode = p.poll()
        try:
            buf.write(p.stdout.read())
        except Exception:
            pass
        if retcode is not None:
            break
        if timeout is not None and time.time() > tstart + timeout:
            print("timeout in process %u" % p.pid)
            try: 
                os.kill(p.pid, signal.SIGKILL)
            except OSError:
                pass
            p.wait()
    return buf.getvalue()

class MiscModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MiscModule, self).__init__(mpstate, "misc", "misc commands")
        self.add_command('alt', self.cmd_alt, "show altitude information")
        self.add_command('up', self.cmd_up, "adjust pitch trim by up to 5 degrees")
        self.add_command('reboot', self.cmd_reboot, "reboot autopilot")
        self.add_command('time', self.cmd_time, "show autopilot time")
        self.add_command('shell', self.cmd_shell, "run shell command")
        self.add_command('changealt', self.cmd_changealt, "change target altitude")
        self.add_command('land', self.cmd_land, "auto land")
        self.add_command('repeat', self.cmd_repeat, "repeat a command at regular intervals",
                         ["<add|remove|clear>"])
        self.add_command('version', self.cmd_version, "show version")
        self.add_command('rcbind', self.cmd_rcbind, "bind RC receiver")
        self.repeats = []

    def altitude_difference(self, pressure1, pressure2, ground_temp):
        '''calculate barometric altitude'''
        scaling = pressure2 / pressure1
        temp = ground_temp + 273.15
        return 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))

    def qnh_estimate(self):
        '''estimate QNH pressure from GPS altitude and scaled pressure'''
        alt_gps = self.master.field('GPS_RAW_INT', 'alt', 0) * 0.001
        pressure2 = self.master.field('SCALED_PRESSURE', 'press_abs', 0)
        ground_temp = self.get_mav_param('GND_TEMP', 21)
        temp = ground_temp + 273.15
        pressure1 = pressure2 / math.exp(math.log(1.0 - (alt_gps / (153.8462 * temp))) / 0.190259)
        return pressure1

    def cmd_alt(self, args):
        '''show altitude'''
        print("Altitude:  %.1f" % self.status.altitude)
        qnh_pressure = self.get_mav_param('AFS_QNH_PRESSURE', None)
        if qnh_pressure is not None and qnh_pressure > 0:
            ground_temp = self.get_mav_param('GND_TEMP', 21)
            pressure = self.master.field('SCALED_PRESSURE', 'press_abs', 0)
            qnh_alt = self.altitude_difference(qnh_pressure, pressure, ground_temp)
            print("QNH Alt: %u meters %u feet for QNH pressure %.1f" % (qnh_alt, qnh_alt*3.2808, qnh_pressure))
        print("QNH Estimate: %.1f millibars" % self.qnh_estimate())

    def cmd_shell(self, args):
        '''shell command'''
        print(run_command(args, shell=False, timeout=3))

    def cmd_up(self, args):
        '''adjust TRIM_PITCH_CD up by 5 degrees'''
        if len(args) == 0:
            adjust = 5.0
        else:
            adjust = float(args[0])
        old_trim = self.get_mav_param('TRIM_PITCH_CD', None)
        if old_trim is None:
            print("Existing trim value unknown!")
            return
        new_trim = int(old_trim + (adjust*100))
        if math.fabs(new_trim - old_trim) > 1000:
            print("Adjustment by %d too large (from %d to %d)" % (adjust*100, old_trim, new_trim))
            return
        print("Adjusting TRIM_PITCH_CD from %d to %d" % (old_trim, new_trim))
        self.param_set('TRIM_PITCH_CD', new_trim)

    def cmd_reboot(self, args):
        '''reboot autopilot'''
        self.master.reboot_autopilot()

    def cmd_time(self, args):
        '''show autopilot time'''
        tusec = self.master.field('SYSTEM_TIME', 'time_unix_usec', 0)
        if tusec == 0:
            print("No SYSTEM_TIME time available")
            return
        print("%s (%s)\n" % (time.ctime(tusec * 1.0e-6), time.ctime()))

    def cmd_changealt(self, args):
        '''change target altitude'''
        if len(args) < 1:
            print("usage: changealt <relaltitude>")
            return
        relalt = float(args[0])
        self.master.mav.mission_item_send(self.settings.target_system,
                                          self.settings.target_component,
                                          0,
                                          3,
                                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                          3, 1, 0, 0, 0, 0,
                                          0, 0, relalt)
        print("Sent change altitude command for %.1f meters" % relalt)

    def cmd_land(self, args):
        '''auto land commands'''
        if len(args) < 1:
            self.master.mav.command_long_send(self.settings.target_system,
                                              0,
                                              mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                              0, 0, 0, 0, 0, 0, 0, 0)
        elif args[0] == 'abort':
            self.master.mav.command_long_send(self.settings.target_system,
                                              0,
                                              mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
                                              0, 0, 0, 0, 0, 0, 0, 0)
        else:
            print("Usage: land [abort]")

    def cmd_version(self, args):
        '''show version'''
        self.master.mav.command_long_send(self.settings.target_system,
                                          self.settings.target_component,
                                          mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                                          0,
                                          1, 0, 0, 0, 0, 0, 0)

    def cmd_rcbind(self, args):
        '''start RC bind'''
        if len(args) < 1:
            print("Usage: rcbind <dsmmode>")
            return
        self.master.mav.command_long_send(self.settings.target_system,
                                          self.settings.target_component,
                                          mavutil.mavlink.MAV_CMD_START_RX_PAIR,
                                          0,
                                          float(args[0]), 0, 0, 0, 0, 0, 0)

    def cmd_repeat(self, args):
        '''repeat a command at regular intervals'''
        if len(args) == 0:
            if len(self.repeats) == 0:
                print("No repeats")
                return
            for i in range(len(self.repeats)):
                print("%u: %s" % (i, self.repeats[i]))
            return
        if args[0] == 'add':
            if len(args) < 3:
                print("Usage: repeat add PERIOD CMD")
                return
            self.repeats.append(RepeatCommand(float(args[1]), " ".join(args[2:])))
        elif args[0] == 'remove':
            if len(args) < 2:
                print("Usage: repeat remove INDEX")
                return
            i = int(args[1])
            if i < 0 or i >= len(self.repeats):
                print("Invalid index %d" % i)
                return
            self.repeats.pop(i)
            return
        elif args[0] == 'clean':
            self.repeats = []
        else:
            print("Usage: repeat <add|remove|clean>")

    def idle_task(self):
        '''called on idle'''
        for r in self.repeats:
            if r.event.trigger():
                self.mpstate.functions.process_stdin(r.cmd, immediate=True)

def init(mpstate):
    '''initialise module'''
    return MiscModule(mpstate)
