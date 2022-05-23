#!/usr/bin/env python
'''rc command handling'''

import time, os, struct
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class RCModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RCModule, self).__init__(mpstate, "rc", "rc command handling", public = True)
        self.count = 18
        self.override = [ 0 ] * self.count
        self.last_override = [ 0 ] * self.count
        self.override_counter = 0
        x = "|".join(str(x) for x in range(1, (self.count+1)))
        self.add_command('rc', self.cmd_rc, "RC input control", ['<%s|all>' % x])
        self.add_command('switch', self.cmd_switch, "flight mode switch control", ['<0|1|2|3|4|5|6>'])
        self.rc_settings = mp_settings.MPSettings(
            [('override_hz', float, 10.0)])
        if self.sitl_output:
            self.rc_settings.override_hz = 20.0
        self.add_completion_function('(RCSETTING)',
                                     self.rc_settings.completion)
        self.override_period = mavutil.periodic_event(self.rc_settings.override_hz)

    def idle_task(self):
        self.override_period.frequency = self.rc_settings.override_hz
        if self.override_period.trigger():
            if (self.override != [ 0 ] * self.count or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                self.send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1

    def send_rc_override(self):
        '''send RC override packet'''
        if self.sitl_output:
            chan16 = self.override[:16]
            buf = struct.pack('<HHHHHHHHHHHHHHHH', *chan16)
            self.sitl_output.write(buf)
        else:
            chan18 = self.override[:18]
            self.master.mav.rc_channels_override_send(self.target_system,
                                                      self.target_component,
                                                      *chan18)

    def cmd_switch(self, args):
        '''handle RC switch changes'''
        mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
        if len(args) != 1:
            print("Usage: switch <pwmvalue>")
            return
        value = int(args[0])
        if value < 0 or value > 6:
            print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
            return
        if self.vehicle_type == 'copter':
            default_channel = 5
        else:
            default_channel = 8
        if self.vehicle_type == 'rover':
            flite_mode_ch_parm = int(self.get_mav_param("MODE_CH", default_channel))
        else:
            flite_mode_ch_parm = int(self.get_mav_param("FLTMODE_CH", default_channel))
        self.override[flite_mode_ch_parm - 1] = mapping[value]
        self.override_counter = 10
        self.send_rc_override()
        if value == 0:
            print("Disabled RC switch override")
        else:
            print("Set RC switch override to %u (PWM=%u channel=%u)" % (
                value, mapping[value], flite_mode_ch_parm))

    def set_override(self, newchannels):
        '''this is a public method for use by drone API or other scripting'''
        self.override = newchannels
        self.override_counter = 10
        self.send_rc_override()

    def set_override_chan(self, channel, value):
        '''this is a public method for use by drone API or other scripting'''
        self.override[channel] = value
        self.override_counter = 10
        self.send_rc_override()

    def get_override_chan(self, channel):
        '''this is a public method for use by drone API or other scripting'''
        return self.override[channel]

    def cmd_rc(self, args):
        '''handle RC value override'''
        if len(args) > 0 and args[0] == 'set':
            self.rc_settings.command(args[1:])
            return
        if len(args) == 1 and args[0] == 'clear':
            channels = self.override
            for i in range(self.count):
                channels[i] = 0
            self.set_override(channels)
            return
        if len(args) != 2:
            print("Usage: rc <set|channel|all|clear> <pwmvalue>")
            return
        value = int(args[1])
        if value > 65535 or value < -1:
            raise ValueError("PWM value must be a positive integer between 0 and 65535")
        if value == -1:
            value = 65535
        channels = self.override
        if args[0] == 'all':
            for i in range(self.count):
                channels[i] = value
        else:
            channel = int(args[0])
            if channel < 1 or channel > self.count:
                print("Channel must be between 1 and %u or 'all'" % self.count)
                return
            channels[channel - 1] = value
        self.set_override(channels)

def init(mpstate):
    '''initialise module'''
    return RCModule(mpstate)
