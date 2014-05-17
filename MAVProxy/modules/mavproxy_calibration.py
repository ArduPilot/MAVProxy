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
        count = 0
        # we expect 6 messages and acks
        while count < 6:
            m = mav.recv_match(type='STATUSTEXT', blocking=True)
            text = str(m.text)
            if not text.startswith('Place '):
                continue
            # wait for user to hit enter
            self.mpstate.rl.line = None
            while self.mpstate.rl.line is None:
                time.sleep(0.1)
            self.mpstate.rl.line = None
            count += 1
            # tell the APM that we've done as requested
            mav.mav.command_ack_send(count, 1)


    def cmd_compassmot(self, args):
        '''do a compass/motor interference calibration'''
        mav = self.master
        print("compassmot starting")
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 0, 1, 0)
        self.mpstate.rl.line = None
        while True:
            m = mav.recv_match(type=['COMMAND_ACK','COMPASSMOT_STATUS'], blocking=False)
            if m is not None:
                print(m)
                if m.get_type() == 'COMMAND_ACK':
                    break
            if self.mpstate.rl.line is not None:
                # user has hit enter, stop the process
                mav.mav.command_ack_send(0, 1)
                break
            time.sleep(0.01)
        print("compassmot done")

    def cmd_calpressure(self, args):
        '''calibrate pressure sensors'''
        self.master.calibrate_pressure()

def init(mpstate):
    '''initialise module'''
    return CalibrationModule(mpstate)
