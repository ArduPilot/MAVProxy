#!/usr/bin/env python
"""
  MAVProxy instructor station module
  André Kjellstrup @ NORCE
"""

import math
from MAVProxy.modules.lib import mp_instructor
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
#from MAVProxy.modules.lib import multiproc
import MAVProxy.modules.mavproxy_link



class InstructorModule(mp_module.MPModule):
    def __init__(self, mpstate):
        #self.in_pipe, self.out_pipe = multiproc.Pipe()

        super(InstructorModule, self).__init__(mpstate, "instructor", "instructor module")
        self.instructor = mp_instructor.InstructorUI()

        self.voltage_is_dropping = False
        self.voltage_drop = 0.0  # accumulated voltage drop
        self.voltage_start = 0  # voltage before drop
        self.voltage_drop_rate = 0.1  # v/s drop rate

    def unload(self):
        '''unload module'''
        print('unloading') # self.mpstate.horizonIndicator.close()

    def mavlink_packet(self, msg):
        """handle an incoming mavlink packet"""
        if not isinstance(self.instructor, mp_instructor.InstructorUI):
            return
        if not self.instructor.is_alive():
            return

        type = msg.get_type()
        master = self.master

        if type == 'HEARTBEAT':
            'beforeEngineList - APM booted'
            if self.mpstate.status.heartbeat_error == True:
                self.instructor.set_check("Pixhawk Booted", 0)
            else:
                self.instructor.set_check("Pixhawk Booted", 1)

            if self.voltage_is_dropping:
                self.voltage_drop += self.voltage_drop_rate
                self.param_set('SIM_BATT_VOLTAGE', (self.voltage_start - self.voltage_drop))

        # if mp_instructor.InstructorFrame.out_pipe.poll():

        if self.instructor.pipe_to_gui.poll():
            obj = self.instructor.pipe_to_gui.recv()
            if obj[0] == "dis_gnss":
                # print(str(obj[1]))
                self.param_set('SIM_GPS_DISABLE', int(obj[1]))

            elif obj[0] == "setmode":
                self.master.set_mode("RTL")

            elif obj[0] == "volt_drop_rate":
                self.voltage_drop_rate = obj[1]
                #print(self.voltage_drop_rate)

            elif obj[0] == "volt_drop":
                if obj[1]:
                    self.voltage_is_dropping = True
                    self.voltage_start = self.get_mav_param('SIM_BATT_VOLTAGE')
                    print('voltage dropping')
                else:
                    self.voltage_is_dropping = False
                    self.param_set('SIM_BATT_VOLTAGE', self.voltage_start)
                    print('voltage restored')
                    #print(self.voltage_start)
                    #print(self.voltage_is_dropping)
                    #print(self.voltage_drop_rate)
                    #print(self.voltage_drop)

                #print(self.get_mav_param('SIM_BATT_VOLTAGE'))
                #self.mav_param('SIM_BATT_VOLTAGE', int(obj[1]))

            elif obj[0] == "gcs_comm_loss":
                if obj[1]:
                    print('on')
                    #MAVProxy.modules.mavproxy_link.LinkModule.cmd_link_add("udp:127.0.0.1:9777")
                else:
                    print('off')
                    #MAVProxy.modules.mavproxy_link.LinkModule.cmd_link_remove("127.0.0.1:14550")
            elif obj[0] == "wind_dir":
                self.param_set('SIM_WIND_DIR', obj[1])

            elif obj[0] == "wind_vel":
                self.param_set('SIM_WIND_SPD', obj[1])

            elif obj[0] == "wind_turbulence":
                self.param_set('SIM_WIND_TURB', obj[1])
                self.param_set('SIM_WIND_T', obj[1])

            # self.instructor.set_check("Odroid Booted", 1)
            #print(obj)

            # Plane Actions
            elif obj[0] == "pitot_fail_low":
                self.param_set('SIM_ARSPD_FAIL', obj[1])

            elif obj[0] == "pitot_fail_high":
                self.param_set('SIM_ARSPD_FAILP', obj[1])

            elif obj[0] == "arspd_offset":
                self.param_set('SIM_ARSPD_OFS', obj[1])


            elif obj[0] == "plane_thrust_loss":
                self.param_set('SERVO3_MAX', 2000 - obj[1])

            elif obj[0] == "plane_thrust_loss_curr":
                self.param_set('SERVO3_MAX', 2000 - obj[1])
                # simulate increased current by 0...-2 offset
                self.param_set('BATT_AMP_OFFSET', (-obj[1]/500))


           # Copter Actions

            elif obj[0] == "copter_thrust_loss":
                self.param_set('MOT_PWM_MAX', 2000 - obj[1])
                # simulate increased current
                self.param_set('BATT_AMP_PERVLT', 17 + (obj[1]/40))

            elif obj[0] == "copter_reset":
                print("MOT_PWM_MAX 2000")
                self.param_set('MOT_PWM_MAX', 2000)
                print("BATT_AMP_PERVLT 17")
                self.param_set('BATT_AMP_PERVLT', 17)
                print('Done')

        '''beforeEngineList - Flight mode MANUAL'''
        if self.mpstate.status.flightmode == "MANUAL":
            self.instructor.set_check("Flight mode MANUAL", 1)
        else:
            self.instructor.set_check("Flight mode MANUAL", 0)

        if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
            '''beforeEngineList - GPS lock'''
            if ((msg.fix_type >= 3 and master.mavlink10()) or
                (msg.fix_type == 2 and not master.mavlink10())):
                self.instructor.set_check("GNSS FIX", 1)
            else:
                self.instructor.set_check("GNSS FIX", 0)

        '''beforeEngineList - Radio Links > 6db margin TODO: figure out how to read db levels'''
        if type in ['RADIO', 'RADIO_STATUS']:
            if msg.rssi < msg.noise+6 or msg.remrssi < msg.remnoise+6:
                self.instructor.set_check("Radio links > 6db margin", 0)
            else:
                self.instructor.set_check("Radio Links > 6db margin", 0)

        if type == 'HWSTATUS':
            '''beforeEngineList - Avionics Battery'''
            if msg.Vcc >= 4600 and msg.Vcc <= 5300:
                self.instructor.set_check("Avionics Power", 1)
            else:
                self.instructor.set_check("Avionics Power", 0)

        if type == 'POWER_STATUS':
            '''beforeEngineList - Servo Power'''
            if msg.Vservo >= 4900 and msg.Vservo <= 6500:
                self.instructor.set_check("Servo Power", 1)
            else:
                self.instructor.set_check("Servo Power", 0)

        '''beforeEngineList - Waypoints Loaded'''
        if type == 'HEARTBEAT':
            if self.module('wp').wploader.count() == 0:
                self.instructor.set_check("Waypoints Loaded", 0)
            else:
                self.instructor.set_check("Waypoints Loaded", 1)

        '''beforeTakeoffList - Compass active'''
        if type == 'GPS_RAW':
            if math.fabs(msg.hdg - master.field('VFR_HUD', 'heading', '-')) < 10 or math.fabs(msg.hdg - master.field('VFR_HUD', 'heading', '-')) > 355:
                self.instructor.set_check("Compass active", 1)
            else:
                self.instructor.set_check("Compass active", 0)

        '''beforeCruiseList - Airspeed > 10 m/s , Altitude > 30 m'''
        if type == 'VFR_HUD':
            rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
            if rel_alt > 30:
                self.instructor.set_check("Altitude > 30 m", 1)
            else:
                self.instructor.set_check("Altitude > 30 m", 0)
            if msg.airspeed > 10 or msg.groundspeed > 10:
                self.instructor.set_check("Airspeed > 10 m/s", 1)
            else:
                self.instructor.set_check("Airspeed > 10 m/s", 0)

        '''beforeEngineList - IMU'''
        if type in ['SYS_STATUS']:
            sensors = {'AS': mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                        'MAG': mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'INS': mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'AHRS': mavutil.mavlink.MAV_SYS_STATUS_AHRS}
            bits = sensors['INS']
            present = ((msg.onboard_control_sensors_enabled & bits) == bits)
            healthy = ((msg.onboard_control_sensors_health & bits) == bits)
            if not present or not healthy:
                self.instructor.set_check("IMU Check", 1)
            else:
                self.instructor.set_check("IMU Check", 0)
    def unload(self):
        '''unload module'''
        self.instructor.close()

def init(mpstate):
    """initialise module"""
    return InstructorModule(mpstate)
