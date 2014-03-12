#!/usr/bin/env python
'''calibration command handling'''

import time, os
from pymavlink import mavutil

def name():
    '''return module name'''
    return "calibration"

def description():
    '''return module description'''
    return "calibration handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.command_map['ground']     = (cmd_ground,   'do a ground start')
    mpstate.command_map['level']      = (cmd_level,    'set level on a multicopter')
    mpstate.command_map['compassmot'] = (cmd_compassmot, 'do compass/motor interference calibration')
    mpstate.command_map['calpress']   = (cmd_calpressure,'calibrate pressure sensors')
    mpstate.command_map['accelcal']   = (cmd_accelcal, 'do 3D accelerometer calibration')

def cmd_ground(args):
    '''do a ground start mode'''
    mpstate.master().calibrate_imu()

def cmd_level(args):
    '''run a accel level'''
    mpstate.master().calibrate_level()

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
        if not text.startswith('Place '):
            continue
        # wait for user to hit enter
        mpstate.rl.line = None
        while mpstate.rl.line is None:
            time.sleep(0.1)
        mpstate.rl.line = None
        count += 1
        # tell the APM that we've done as requested
        mav.mav.command_ack_send(count, 1)


def cmd_compassmot(args):
    '''do a compass/motor interference calibration'''
    mav = mpstate.master()
    print("compassmot starting")
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                              0, 0, 0, 0, 0, 1, 0)
    mpstate.rl.line = None
    while True:
        m = mav.recv_match(type=['COMMAND_ACK','COMPASSMOT_STATUS'], blocking=False)
        if m is not None:
            print(m)
            if m.get_type() == 'COMMAND_ACK':
                break
        if mpstate.rl.line is not None:
            # user has hit enter, stop the process
            mav.mav.command_ack_send(0, 1)
            break
        time.sleep(0.01)
    print("compassmot done")

def cmd_calpressure(args):
    '''calibrate pressure sensors'''
    mpstate.master().calibrate_pressure()
