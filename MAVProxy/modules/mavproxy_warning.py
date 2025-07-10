#!/usr/bin/env python
'''module to display and announce warnings about system failures'''

import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil

class WarningModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(WarningModule, self).__init__(mpstate, "warning", "warning module")
        self.add_command('warning', self.cmd_warning, "warning", ["details", "set (WARNINGSETTING)"])
        self.check_time = time.time()
        self.warn_time = time.time()
        self.failure = None
        self.details = []
        self.warning_settings = mp_settings.MPSettings(
            [ ('warn_time', int, 5),
              ('details', bool, True),
              ('min_sat', int, 10),
              ('sat_diff', int, 5),
              ('alt_diff', float, 20),
              ('airspd_diff', float, 5),
              ('airspd_age', float, 5),
              ('esc_group1', int, 0),
              ('esc_group2', int, 0),
              ('esc_min_rpm', int, 100),
              ('max_wind', int, 10),
          ])
        self.add_completion_function('(WARNINGSETTING)',
                                     self.warning_settings.completion)
        # variables for monitoring airspeed changes
        self.last_as1 = 0
        self.last_as2 = 0
        self.last_as1_change = time.time()
        self.last_as2_change = time.time()

    def cmd_warning(self, args):
        '''warning commands'''
        state = self
        if len(args) > 0:
            if args[0] == 'set':
                state.warning_settings.command(args[1:])
            if args[0] == 'details':
                print("warning: %s" % '|'.join(self.details))
        else:
            print('usage: warning set|details')

    def get_esc_rpms(self):
        '''get a dictionary of ESC RPMs'''
        ret = {}
        esc_messages = [ ('ESC_TELEMETRY_1_TO_4', 1),
                         ('ESC_TELEMETRY_5_TO_8', 5),
                         ('ESC_TELEMETRY_9_TO_12', 9),
                         ('ESC_TELEMETRY_13_TO_16', 13) ]
        for (mname, base) in esc_messages:
            m = self.master.messages.get(mname, None)
            if m:
                for i in range(4):
                    ret[base+i] = m.rpm[i]
        return ret


    def check_esc_group(self, group_mask, rpms):
        '''check one ESC group for consistency. Either all running or none running'''
        escs = []
        for i in range(16):
            if group_mask & (1<<i):
                escs.append(i+1)
        num_running = 0
        for e in escs:
            rpm = rpms.get(e,0)
            if rpm >= self.warning_settings.esc_min_rpm:
                num_running += 1
        return num_running != 0 and num_running != len(escs)

    def check_escs(self):
        '''check for ESC consistency'''
        rpms = self.get_esc_rpms()
        if self.check_esc_group(self.warning_settings.esc_group1, rpms):
            self.details.append("ESC group1 fail")
            return True
        if self.check_esc_group(self.warning_settings.esc_group2, rpms):
            self.details.append("ESC group2 fail")
            return True
        return False

    def check_gps(self):
        '''check GPS issues'''
        gps1 = self.master.messages.get("GPS_RAW_INT", None)
        gps2 = self.master.messages.get("GPS2_RAW", None)
        if gps1 and gps2:
            if abs(gps1.satellites_visible - gps2.satellites_visible) > self.warning_settings.sat_diff:
                self.details.append("GPS sat diff")
                return True
            if abs(gps1.alt*0.001 - gps2.alt*0.001) > self.warning_settings.alt_diff:
                self.details.append("GPS alt diff")
                return True
        if gps1 and gps1.satellites_visible < self.warning_settings.min_sat:
            self.details.append("GPS1 low sat count")
            return True
        if gps2 and gps2.satellites_visible < self.warning_settings.min_sat:
            self.details.append("GPS2 low sat count")
            return True
        return False

    def check_airspeed(self):
        '''check airspeed sensors'''
        vfr_hud = self.master.messages.get("VFR_HUD", None)
        nvf_as2 = self.master.messages.get("NAMED_VALUE_FLOAT[AS2]", None)
        if vfr_hud and nvf_as2:
            as1 = vfr_hud.airspeed
            as2 = nvf_as2.value
            if abs(as1 - as2) > self.warning_settings.airspd_diff:
                self.details.append("Airspeed difference")
                return True
            now = time.time()
            if as1 != self.last_as1:
                self.last_as1 = as1
                self.last_as1_change = now
            if as2 != self.last_as2:
                self.last_as2 = as2
                self.last_as2_change = now
            if as1 > 0 and now - self.last_as1_change > self.warning_settings.airspd_age:
                self.details.append("Airspeed1 age")
                return True
            if as2 > 0 and now - self.last_as2_change > self.warning_settings.airspd_age:
                self.details.append("Airspeed2 age")
                return True
        return False

    def check_status(self):
        '''check SYS_STATUS health bits'''
        status = self.master.messages.get("SYS_STATUS", None)
        if not status:
            return False
        sensors = { 'AS'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                    'MAG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                    'INS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                    'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS,
                    'TERR' : mavutil.mavlink.MAV_SYS_STATUS_TERRAIN,
                    'LOG'  : mavutil.mavlink.MAV_SYS_STATUS_LOGGING,
                    'FEN'  : mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
        }
        health = status.onboard_control_sensors_health
        for sname in sensors:
            bits = sensors[sname]
            if health & bits != bits:
                self.details.append("status %s" % sname)
                return True
        return False

    def check_wind(self):
        '''check wind level'''
        wind = self.master.messages.get("WIND", None)
        if wind:
            if wind.speed > self.warning_settings.max_wind:
                self.details.append("Wind %.1f" % wind.speed)
                return True
        return False

    def check_all(self):
        failures = []
        if self.check_escs():
            failures.append('ESC')
        if self.check_gps():
            failures.append('GPS')
        if self.check_airspeed():
            failures.append('AIRSPEED')
        if self.check_status():
            failures.append('STATUS')
        if self.check_wind():
            failures.append('WIND')
        if len(failures) > 0:
            self.failure = '|'.join(failures)

    def monitor(self):
        last_fail = self.failure
        self.failure = None
        self.details = []
        self.check_all()
        if self.failure and self.failure != last_fail:
            self.warn(self.failure)
        if not self.failure:
            self.console.set_status('WARN', 'WARN:OK', fg='green', row=2)
        else:
            self.console.set_status('WARN', 'WARN:%s' % self.failure, fg='red', row=2)

    def warn(self, err):
        self.say("Warning " + err)
        self.warn_time = time.time()

    def idle_task(self):
        '''called on idle'''
        now = time.time()

        # at 1Hz update status. If status changes warn immediately
        if (now - self.check_time) >= 1:
            self.check_time = now
            self.monitor()

        # periodically announce the failure if any
        if (self.failure and
            self.warning_settings.warn_time > 0 and
            (now - self.warn_time) >= self.warning_settings.warn_time):
            self.warn(self.failure)
            

def init(mpstate):
    '''initialise module'''
    return WarningModule(mpstate)
