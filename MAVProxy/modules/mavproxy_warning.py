#!/usr/bin/env python
'''Warning'''

import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil

class WarningModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(WarningModule, self).__init__(mpstate, "warning", "warning module")
        self.add_command('warning', self.cmd_warning, "warning")
        self.prev_call = time.time()
        self.esc_fwd_err = 0
        self.num_escs = 4
        self.sensor_present = dict()
        self.sensor_enabled = dict()
        self.sensor_healthy = dict()
        self.details = ""
        self.warning_settings = mp_settings.MPSettings(
            [ ('check_freq', int, 5),
              ('details', bool, True),
              ('min_sat', int, 9),
              ('sat_diff', int, 5),
              ('rpm_diff', int, 1000),
              ('airspd_diff', float, 5),
              ('max_wind', float, 10),
              ('max_agl_alt', float, 123),
          ])


    def cmd_warning(self, args):
        '''warning commands'''
        state = self
        if args and args[0] == 'set':
            if len(args) < 3:
                state.warning_settings.show_all()
            else:
                state.warning_settings.set(args[1], args[2])
        else:
            print('usage: warning set')


    def monitor(self): 
        fwd_escs = [ 0, 2 ]
        vtol_escs = [ 1, 3 ]

        gps1 = self.master.messages.get("GPS_RAW_INT", None)
        gps2 = self.master.messages.get("GPS2_RAW", None)
        esc = self.master.messages.get("ESC_TELEMETRY_1_TO_4", None)
        vfr_hud = self.master.messages.get("VFR_HUD", None)
        terr = self.master.messages.get("TERRAIN_REPORT", None)
        nvf_as2 = self.master.messages.get("NAMED_VALUE_FLOAT", None)
        if nvf_as2 != None:
            nvf_as2 = nvf_as2['AS2']

        if gps1 is None or gps2 is None:
            self.details = "No GPS1 and/or GPS2 message"
            self.warn("GPS")
        else:
            #check min visible sat
            if gps1.satellites_visible < self.warning_settings.min_sat:
                self.details = "GPS1 satellites less than " + str(self.warning_settings.min_sat)
                self.warn("GPS") 

            if gps2.satellites_visible < self.warning_settings.min_sat:
                self.details = "GPS2 satellites less than " + str(self.warning_settings.min_sat)
                self.warn("GPS") 

            #check difference in sat counts
            if abs(gps1.satellites_visible - gps2.satellites_visible) > self.warning_settings.sat_diff: 
                self.details = "Satellite diff between GPS1 and GPS2 more than " + str(self.warning_settings.sat_diff)
                self.warn("GPS")

        #check fwd esc rpms
        if esc is None:
            self.details = "No ESC message"
            self.warn("ESC")
        else:
            max_esc = 0
            min_esc = 1e9
            for no in fwd_escs:
                # print("ESC number: " + str(no))
                if esc.rpm[no] > max_esc:
                    max_esc = esc.rpm[no]
                if esc.rpm[no] < min_esc:
                    min_esc = esc.rpm[no]

            if (max_esc - min_esc) > self.warning_settings.rpm_diff:
                self.details = "FWD ESC difference between max and min RPM more than " + str(self.warning_settings.rpm_diff)
                self.warn("ESC")

            #check vtol esc rpms
            max_esc = 0
            min_esc = 1e9
            for no in vtol_escs:
                # print("ESC number: " + str(no))
                if esc.rpm[no] > max_esc:
                    max_esc = esc.rpm[no]
                if esc.rpm[no] < min_esc:
                    min_esc = esc.rpm[no]

            if (max_esc - min_esc) > self.warning_settings.rpm_diff:
                self.details = "VTOL ESC difference between max and min RPM more than " + str(self.warning_settings.rpm_diff)
                self.warn("ESC")

        if not (self.sensor_enabled['AS'] and self.sensor_present['AS'] and self.sensor_healthy['AS']):
            self.details = "Airspeed sensor issue: Enabled: " + str(self.sensor_enabled['AS']) + ", Present: " + str(self.sensor_present['AS']) + ", Healthy: " + str(self.sensor_healthy['AS'])
            self.warn("Airspeed")

        if vfr_hud is None or nvf_as2 is None:
            self.details = "No HUD and/or AS2 message"
            self.warn("Airspeed")
        else:
            if abs(vfr_hud.airspeed - nvf_as2.value) > self.warning_settings.airspd_diff:
                self.details = "Airspeed 1 vs airspeed 2 difference greater than " + str(self.warning_settings.airspd_diff)
                self.warn("Airspeed")

            if abs(vfr_hud.groundspeed - (vfr_hud.airspeed + nvf_as2.value)/2) > self.warning_settings.max_wind:
                self.details = "Airspeed vs groundspeed difference more than " + str(self.warning_settings.max_wind)
                self.warn("Airspeed")
            
        if terr is None:
            self.details = "No terrain message."
            self.warn("Terrain")
        else:
            if terr.pending > 0:
                self.details = "Pending terrain items."
                self.warn("Terrain")

            if terr.current_height > self.warning_settings.max_agl_alt:
                self.details = "AGL Alt more than " + str(self.warning_settings.max_agl_alt)
                self.warn("Terrain")


    def update_status(self):
        sys_status = self.master.messages["SYS_STATUS"]
        sensors = { 'AS'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                    'MAG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                    'INS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                    'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS,
                    'RC'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
                    'TERR' : mavutil.mavlink.MAV_SYS_STATUS_TERRAIN,
                    'RNG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION,
                    'LOG'  : mavutil.mavlink.MAV_SYS_STATUS_LOGGING,
                    'PRX'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY,
                    'PRE'  : mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK,
                    'FLO'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW,
        }
        for s in sensors.keys():
            bits = sensors[s]
            self.sensor_present[s] = ((sys_status.onboard_control_sensors_present & bits) == bits)
            self.sensor_enabled[s] = ((sys_status.onboard_control_sensors_enabled & bits) == bits)
            self.sensor_healthy[s] = ((sys_status.onboard_control_sensors_health & bits) == bits)
            

    def warn(self, err):
        self.say("Warning " + err + " failure")
        if self.warning_settings.details == True and self.details != "":
            self.console.writeln(self.details, fg = 'grey')
            self.details = ""


    def idle_task(self):
        '''called on idle'''
        now = time.time()
        if (now - self.prev_call) > self.warning_settings.check_freq:
            self.prev_call = now
            self.update_status()
            self.monitor()


def init(mpstate):
    '''initialise module'''
    return WarningModule(mpstate)
