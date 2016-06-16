"""
helicopter monitoring and control module gas helicopters
"""

import os, sys, math, time

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class GasHeliModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GasHeliModule, self).__init__(mpstate, "gas_heli", "Gas Heli", public=False)
        self.console.set_status('IGN', 'IGN', row=4)
        self.console.set_status('THR', 'THR', row=4)
        self.console.set_status('RPM', 'RPM: 0', row=4)
        self.add_command('gasheli', self.cmd_gasheli,
                         'gas helicopter control',
                         ['<start|stop>',
                          'set (GASHELISETTINGS)'])
        self.gasheli_settings = mp_settings.MPSettings(
            [ ('ignition_chan', int, 0),
              ('ignition_disable_time', float, 0.5),
              ('ignition_stop_time', float, 3),
              ('starter_chan', int, 0),
              ('starter_time', float, 3.0),
              ('starter_pwm_on', int, 2000),
              ('starter_pwm_off', int, 1000),
              ]
            )
        self.add_completion_function('(GASHELISETTINGS)', self.gasheli_settings.completion)
        self.starting_motor = False
        self.stopping_motor = False
        self.motor_t1 = None
        self.old_override = 0

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()

        master = self.master

        # add some status fields
        if type in [ 'RC_CHANNELS_RAW' ]:
            rc6 = msg.chan6_raw
            if rc6 > 1500:
                ign_colour = 'green'
            else:
                ign_colour = 'red'
            self.console.set_status('IGN', 'IGN', fg=ign_colour, row=4)

        if type in [ 'SERVO_OUTPUT_RAW' ]:
            rc8 = msg.servo8_raw
            if rc8 < 1200:
                thr_colour = 'red'
            elif rc8 < 1300:
                thr_colour = 'orange'
            else:
                thr_colour = 'green'
            self.console.set_status('THR', 'THR', fg=thr_colour, row=4)

        if type in [ 'RPM' ]:
            rpm = msg.rpm1
            if rpm < 3000:
                rpm_colour = 'red'
            elif rpm < 10000:
                rpm_colour = 'orange'
            else:
                rpm_colour = 'green'
            self.console.set_status('RPM', 'RPM: %u' % rpm, fg=rpm_colour, row=4)

    def valid_starter_settings(self):
        '''check starter settings'''
        if self.gasheli_settings.ignition_chan <= 0 or self.gasheli_settings.ignition_chan > 8:
            print("Invalid ignition channel %d" % self.gasheli_settings.ignition_chan)
            return False
        if self.gasheli_settings.starter_chan <= 0 or self.gasheli_settings.starter_chan > 14:
            print("Invalid starter channel %d" % self.gasheli_settings.starter_chan)
            return False
        return True

    def idle_task(self):
        '''run periodic tasks'''
        if self.starting_motor:
            if self.gasheli_settings.ignition_disable_time > 0:
                elapsed = time.time() - self.motor_t1
                if elapsed >= self.gasheli_settings.ignition_disable_time:
                    self.module('rc').set_override_chan(self.gasheli_settings.ignition_chan-1, self.old_override)
                    self.starting_motor = False
        if self.stopping_motor:
            elapsed = time.time() - self.motor_t1
            if elapsed >= self.gasheli_settings.ignition_stop_time:
                # hand back control to RC
                self.module('rc').set_override_chan(self.gasheli_settings.ignition_chan-1, self.old_override)
                self.stopping_motor = False

    def start_motor(self):
        '''start motor'''
        if not self.valid_starter_settings():
            return
        self.motor_t1 = time.time()
        self.stopping_motor = False

        if self.gasheli_settings.ignition_disable_time > 0:
            self.old_override = self.module('rc').get_override_chan(self.gasheli_settings.ignition_chan-1)
            self.module('rc').set_override_chan(self.gasheli_settings.ignition_chan-1, 1000)
            self.starting_motor = True
        else:
            # nothing more to do
            self.starting_motor = False

        # setup starter run
        self.master.mav.command_long_send(self.target_system,
                                          self.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0,
                                          self.gasheli_settings.starter_chan,
                                          self.gasheli_settings.starter_pwm_on,
                                          1,
                                          self.gasheli_settings.starter_time*2,
                                          0, 0, 0)
        print("Starting motor")

    def stop_motor(self):
        '''stop motor'''
        if not self.valid_starter_settings():
            return
        self.motor_t1 = time.time()
        self.starting_motor = False
        self.stopping_motor = True
        self.old_override = self.module('rc').get_override_chan(self.gasheli_settings.ignition_chan-1)
        self.module('rc').set_override_chan(self.gasheli_settings.ignition_chan-1, 1000)
        print("Stopping motor")

    def cmd_gasheli(self, args):
        '''gas help commands'''
        usage = "Usage: gasheli <start|stop|set>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "start":
            self.start_motor()
        elif args[0] == "stop":
            self.stop_motor()
        elif args[0] == "set":
            self.gasheli_settings.command(args[1:])
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return GasHeliModule(mpstate)
