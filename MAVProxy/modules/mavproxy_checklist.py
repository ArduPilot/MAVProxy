#!/usr/bin/env python
'''
Checklist module
Stephen Dade
July 2014
'''

import sys, os, time
from MAVProxy.modules.lib import mp_checklist
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

class ChecklistModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ChecklistModule, self).__init__(mpstate, "checklist", "checklist module")
        self.checklist = mp_checklist.CheckUI()

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if not isinstance(self.checklist, mp_checklist.CheckUI):
            return
        if not self.checklist.is_alive():
            return
        
        type = msg.get_type()
        master = self.master

        if type == 'HEARTBEAT':
            '''beforeEngineList - APM booted'''
            if self.mpstate.status.heartbeat_error == True:
                self.checklist.set_check("Pixhawk Booted", 0)
            else:
                self.checklist.set_check("Pixhawk Booted", 1)

        '''beforeEngineList - Flight mode MANUAL'''
        if self.mpstate.status.flightmode == "MANUAL":
            self.checklist.set_check("Flight mode MANUAL", 1)
        else:
            self.checklist.set_check("Flight mode MANUAL", 0)

        if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
            '''beforeEngineList - GPS lock'''
            if ((msg.fix_type >= 3 and master.mavlink10()) or
                (msg.fix_type == 2 and not master.mavlink10())):
                self.checklist.set_check("GPS lock", 1)
            else:
                self.checklist.set_check("GPS lock", 0)

        '''beforeEngineList - Radio Links > 6db margin TODO: figure out how to read db levels'''
        if type in ['RADIO', 'RADIO_STATUS']:
            if msg.rssi < msg.noise+6 or msg.remrssi < msg.remnoise+6:
                self.checklist.set_check("Radio links > 6db margin", 0)
            else:
                self.checklist.set_check("Radio Links > 6db margin", 0)

        if type == 'HWSTATUS':
            '''beforeEngineList - Avionics Battery'''
            if msg.Vcc >= 4600 and msg.Vcc <= 5300:
                self.checklist.set_check("Avionics Power", 1)
            else:
                self.checklist.set_check("Avionics Power", 0)

        if type == 'POWER_STATUS':
            '''beforeEngineList - Servo Power'''
            if msg.Vservo >= 4900 and msg.Vservo <= 6500:
                self.checklist.set_check("Servo Power", 1)
            else:
                self.checklist.set_check("Servo Power", 0)

        '''beforeEngineList - Waypoints Loaded'''
        if type == 'HEARTBEAT':
            if self.module('wp').wploader.count() == 0:
                self.checklist.set_check("Waypoints Loaded", 0)
            else:
                self.checklist.set_check("Waypoints Loaded", 1)

		'''beforeTakeoffList - Compass active'''
        if type == 'GPS_RAW':
            if math.fabs(msg.hdg - master.field('VFR_HUD', 'heading', '-')) < 10 or math.fabs(msg.hdg - master.field('VFR_HUD', 'heading', '-')) > 355:
                self.checklist.set_check("Compass active", 1)
            else:
                self.checklist.set_check("Compass active", 0)

		'''beforeCruiseList - Airspeed > 10 m/s , Altitude > 30 m'''
        if type == 'VFR_HUD':
            rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
            if rel_alt > 30:
                self.checklist.set_check("Altitude > 30 m", 1)
            else:
                self.checklist.set_check("Altitude > 30 m", 0)
            if msg.airspeed > 10 or msg.groundspeed > 10:
                self.checklist.set_check("Airspeed > 10 m/s", 1)
            else:
                self.checklist.set_check("Airspeed > 10 m/s", 0)

		'''beforeEngineList - IMU'''
        if type in ['SYS_STATUS']:
            sensors = { 'AS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                        'MAG' : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'INS' : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS}
            bits = sensors['INS']
            present = ((msg.onboard_control_sensors_enabled & bits) == bits)
            healthy = ((msg.onboard_control_sensors_health & bits) == bits)
            if not present or not healthy:
                self.checklist.set_check("IMU Check", 1)
            else:
                self.checklist.set_check("IMU Check", 0)


def init(mpstate):
    '''initialise module'''
    return ChecklistModule(mpstate)
