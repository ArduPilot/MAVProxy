#!/usr/bin/env python
'''calibration command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class CalibrationModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CalibrationModule, self).__init__(mpstate, "calibration")
        self.add_command('ground', self.cmd_ground,   'do a ground start')
        self.add_command('level', self.cmd_level,    'set level on a multicopter')
        self.add_command('compassmot', self.cmd_compassmot, 'do compass/motor interference calibration')
        self.add_command('calpress', self.cmd_calpressure,'calibrate pressure sensors')
        self.add_command('accelcal', self.cmd_accelcal, 'do 3D accelerometer calibration')
        self.add_command('gyrocal', self.cmd_gyrocal, 'do gyro calibration')
        self.accelcal_count = -1
        self.accelcal_wait_enter = False
        self.compassmot_running = False
        self.empty_input_count = 0

    def cmd_ground(self, args):
        '''do a ground start mode'''
        self.master.calibrate_imu()

    def cmd_level(self, args):
        '''run a accel level'''
        self.master.calibrate_level()

    def cmd_accelcal(self, args):
        '''do a full 3D accel calibration'''
        mav = self.master
        # ack the APM to begin 3D calibration of accelerometers
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 1, 0, 0)
        self.accelcal_count = 0
        self.accelcal_wait_enter = False

    def cmd_gyrocal(self, args):
        '''do a full gyro calibration'''
        mav = self.master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  1, 0, 0, 0, 0, 0, 0)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if self.accelcal_count != -1:
            if m.get_type() == 'STATUSTEXT':
                # handle accelcal packet
                text = str(m.text)
                if text.startswith('Place '):
                    self.accelcal_wait_enter = True
                    self.empty_input_count = self.mpstate.empty_input_count

    def idle_task(self):
        '''handle mavlink packets'''
        if self.accelcal_count != -1:
            if self.accelcal_wait_enter and self.empty_input_count != self.mpstate.empty_input_count:
                self.accelcal_wait_enter = False
                self.accelcal_count += 1
                # tell the APM that user has done as requested
                self.master.mav.command_ack_send(self.accelcal_count, 1)
                if self.accelcal_count >= 6:
                    self.accelcal_count = -1

        if self.compassmot_running:
            if self.mpstate.empty_input_count != self.empty_input_count:
                # user has hit enter, stop the process
                    self.compassmot_running = False
                    print("sending stop")
                    self.master.mav.command_ack_send(0, 1)

    
    def cmd_compassmot(self, args):
        '''do a compass/motor interference calibration'''
        mav = self.master
        print("compassmot starting")
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 0, 1, 0)
        self.compassmot_running = True
        self.empty_input_count = self.mpstate.empty_input_count

    def cmd_calpressure(self, args):
        '''calibrate pressure sensors'''
        self.master.calibrate_pressure()

def init(mpstate):
    '''initialise module'''
    return CalibrationModule(mpstate)
