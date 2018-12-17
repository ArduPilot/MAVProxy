"""
monitoring of heliplane
"""

import os, sys, math, time

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class HeliPlaneModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(HeliPlaneModule, self).__init__(mpstate, "heliplane", "HeliPlane", public=False)
        self.last_chan_check = 0

        self.update_channels()

    def get_rc_input(self, msg, chan):
        '''extract RC input value'''
        if chan <= 0:
            return -1
        return getattr(msg, 'chan%u_raw' % chan, -1)

    def get_pwm_output(self, msg, chan):
        '''extract PWM output value'''
        if chan <= 0:
            return -1
        return getattr(msg, 'servo%u_raw' % chan, -1)
    
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()

        master = self.master

        # add some status fields
        if type in [ 'RC_CHANNELS' ]:
            ilock = self.get_rc_input(msg, self.interlock_channel)
            if ilock <= 0:
                self.console.set_status('ILOCK', 'ILOCK:--', fg='grey', row=4)
            elif ilock >= 1800:
                self.console.set_status('ILOCK', 'ILOCK:ON', fg='red', row=4)
            else:
                self.console.set_status('ILOCK', 'ILOCK:OFF', fg='green', row=4)

            override = self.get_rc_input(msg, self.override_channel)
            if override <= 0:
                self.console.set_status('OVR', 'OVR:--', fg='grey', row=4)
            elif override >= 1800:
                self.console.set_status('OVR', 'OVR:ON', fg='red', row=4)
            else:
                self.console.set_status('OVR', 'OVR:OFF', fg='green', row=4)

            zeroi = self.get_rc_input(msg, self.zero_I_channel)
            if zeroi <= 0:
                self.console.set_status('ZEROI', 'ZEROI:--', fg='grey', row=4)
            elif zeroi >= 1800:
                self.console.set_status('ZEROI', 'ZEROI:ON', fg='red', row=4)
            else:
                self.console.set_status('ZEROI', 'ZEROI:OFF', fg='green', row=4)

            novtol = self.get_rc_input(msg, self.no_vtol_channel)
            if novtol <= 0:
                self.console.set_status('NOVTOL', 'NOVTOL:--', fg='grey', row=4)
            elif novtol >= 1800:
                self.console.set_status('NOVTOL', 'NOVTOL:ON', fg='red', row=4)
            else:
                self.console.set_status('NOVTOL', 'NOVTOL:OFF', fg='green', row=4)
                
        if type in [ 'SERVO_OUTPUT_RAW' ]:
            rsc = self.get_pwm_output(msg, self.rsc_out_channel)
            if rsc <= 0:
                self.console.set_status('RSC', 'RSC:--', fg='grey', row=4)
            elif rsc <= 1200:
                self.console.set_status('RSC', 'RSC:%u' % rsc, fg='red', row=4)
            elif rsc <= 1600:
                self.console.set_status('RSC', 'RSC:%u' % rsc, fg='orange', row=4)
            else:
                self.console.set_status('RSC', 'RSC:%u' % rsc, fg='green', row=4)

            thr = self.get_pwm_output(msg, self.fwd_thr_channel)
            if thr <= 0:
                self.console.set_status('FTHR', 'FTHR:--', fg='grey', row=4)
            elif thr <= 1100:
                self.console.set_status('FTHR', 'FTHR:%u' % thr, fg='red', row=4)
            elif thr <= 1500:
                self.console.set_status('FTHR', 'FTHR:%u' % thr, fg='orange', row=4)
            else:
                self.console.set_status('FTHR', 'FTHR:%u' % thr, fg='green', row=4)
                
        if type in [ 'RPM' ]:
            rpm = msg.rpm1
            if rpm < 1000:
                rpm_colour = 'red'
            elif rpm < 2000:
                rpm_colour = 'orange'
            else:
                rpm_colour = 'green'
            self.console.set_status('RPM', 'RPM: %u' % rpm, fg=rpm_colour, row=4)

    def update_channels(self):
        '''update which channels provide input'''
        self.interlock_channel = -1
        self.override_channel = -1
        self.zero_I_channel = -1
        self.no_vtol_channel = -1

        # output channels
        self.rsc_out_channel = 9
        self.fwd_thr_channel = 10

        for ch in range(1,16):
            option = self.get_mav_param("RC%u_OPTION" % ch, 0)
            if option == 32:
                self.interlock_channel = ch;
            elif option == 63:
                self.override_channel = ch;
            elif option == 64:
                self.zero_I_channel = ch;
            elif option == 65:
                self.override_channel = ch;
            elif option == 66:
                self.no_vtol_channel = ch;

            function = self.get_mav_param("SERVO%u_FUNCTION" % ch, 0)
            if function == 32:
                self.rsc_out_channel = ch
            if function == 70:
                self.fwd_thr_channel = ch

    def idle_task(self):
        '''run periodic tasks'''
        now = time.time()
        if now - self.last_chan_check >= 1:
            self.last_chan_check = now
            self.update_channels()

def init(mpstate):
    '''initialise module'''
    return HeliPlaneModule(mpstate)
