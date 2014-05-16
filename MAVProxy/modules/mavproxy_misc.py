#!/usr/bin/env python
'''miscellaneous commands'''

import time, math
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class MiscModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MiscModule, self).__init__(mpstate, "misc", "misc commands")
        self.add_command('alt', self.cmd_alt, "show altitude information")
        self.add_command('bat', self.cmd_bat, "show battery information")
        self.add_command('up', self.cmd_up, "adjust pitch trim by up to 5 degrees")
        self.add_command('reboot', self.cmd_reboot, "reboot autopilot")
        self.add_command('time', self.cmd_time, "show autopilot time")

    def cmd_alt(self, args):
        '''show altitude'''
        print("Altitude:  %.1f" % self.status.altitude)

    def cmd_bat(self, args):
        '''show battery levels'''
        print("Flight battery:   %u%%" % self.status.battery_level)
        print("Avionics battery: %u%%" % self.status.avionics_battery_level)

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



def init(mpstate):
    '''initialise module'''
    return MiscModule(mpstate)
