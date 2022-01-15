#!/usr/bin/env python
'''battery commands'''

import time, math
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting

class BatteryModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(BatteryModule, self).__init__(mpstate, "battery", "battery commands")
        self.add_command('bat', self.cmd_bat, "show battery information")
        self.last_battery_announce = 0
        self.last_battery_announce_time = 0
        self.last_battery_cell_announce_time = 0
        self.battery_level = {}
        self.voltage_level = {}
        self.current_battery = {}
        self.per_cell = {}
        self.servo_voltage = -1
        self.high_servo_voltage = -1
        self.last_servo_warn_time = 0
        self.last_vcc_warn_time = 0

        self.settings.append(
            MPSetting('battwarn', int, 1, 'Battery Warning Time', tab='Battery'))
        self.settings.append(
            MPSetting('batwarncell', float, 3.7, 'Battery cell Warning level'))
        self.settings.append(
            MPSetting('servowarn', float, 4.3, 'Servo voltage warning level'))
        self.settings.append(
            MPSetting('vccwarn', float, 4.3, 'Vcc voltage warning level'))
        self.settings.append(MPSetting('numcells', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells2', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells3', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells4', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells5', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells6', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells7', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells8', int, 0, range=(0,50), increment=1))
        self.settings.append(MPSetting('numcells9', int, 0, range=(0,50), increment=1))
        self.battery_period = mavutil.periodic_event(5)

    def idstr(self, id):
        return "" if id == 0 else "%u" % (id+1)

    def numcells(self, id):
        ncells = "numcells" + self.idstr(id)
        return self.settings.get_setting(ncells).value

    def cmd_bat(self, args):
        '''show battery levels'''
        print("Flight battery:   %u%%" % self.battery_level[0])
        for id in range(9):
            if self.numcells(id) != 0 and id in self.voltage_level:
                print("Bat%u %.2f V/cell for %u cells %.1fA %.2f%%" % (id+1,
                                                                        self.per_cell[id],
                                                                        self.numcells(id),
                                                                        self.current_battery[id],
                                                                        self.battery_level[id]))

    def battery_report(self):
        battery_string = ''
        for id in range(9):
            if not id in self.voltage_level:
                continue
            batt_mon = int(self.get_mav_param('BATT%s_MONITOR' % self.idstr(id),0))
            if batt_mon == 0:
                continue

            #report voltage level only
            if batt_mon == 3:
                battery_string += 'Batt%u: %.2fV ' % (id+1, self.voltage_level[id])
            elif batt_mon >= 4:
                battery_string += 'Batt%u: %u%%/%.2fV %.1fA ' % (id+1,
                                                                   self.battery_level[id],
                                                                   self.voltage_level[id],
                                                                   self.current_battery[id])
        self.console.set_status('Battery', battery_string, row=1)

        # only announce first battery
        if not 0 in self.battery_level:
            return

        batt_mon = int(self.get_mav_param('BATT_MONITOR',0))
        if batt_mon == 0:
            return

        rbattery_level = int((self.battery_level[0]+5)/10)*10
        if batt_mon >= 4 and self.settings.battwarn > 0 and time.time() > self.last_battery_announce_time + 60*self.settings.battwarn:
            if rbattery_level != self.last_battery_announce:
                self.say("Flight battery %u percent" % rbattery_level, priority='notification')
            if rbattery_level <= 20:
                self.say("Flight battery warning")
            self.last_battery_announce_time = time.time()

        if (self.numcells(0) != 0 and 0 in self.per_cell and self.per_cell[0] < self.settings.batwarncell and
            self.settings.battwarn > 0 and time.time() > self.last_battery_cell_announce_time + 60*self.settings.battwarn):
            self.say("Cell warning")
            self.last_battery_cell_announce_time = time.time()


    def battery_update(self, BATTERY_STATUS):
        '''update battery level'''
        # main flight battery
        id = BATTERY_STATUS.id
        self.battery_level[id] = BATTERY_STATUS.battery_remaining
        self.voltage_level[id] = BATTERY_STATUS.voltages[0]*0.001
        self.current_battery[id] = BATTERY_STATUS.current_battery*0.01
        if self.numcells(id) > 0:
            self.per_cell[id] = self.voltage_level[id] / self.numcells(id)

    def power_status_update(self, POWER_STATUS):
        '''update POWER_STATUS warnings level'''
        now = time.time()
        Vservo = POWER_STATUS.Vservo * 0.001
        Vcc = POWER_STATUS.Vcc * 0.001
        self.high_servo_voltage = max(self.high_servo_voltage, Vservo)
        if self.high_servo_voltage > 1 and Vservo < self.settings.servowarn:
            if now - self.last_servo_warn_time > 30:
                self.last_servo_warn_time = now
                self.say("Servo volt %.1f" % Vservo)
                if Vservo < 1:
                    # prevent continuous announcements on power down
                    self.high_servo_voltage = Vservo

        if Vcc > 0 and Vcc < self.settings.vccwarn:
            if now - self.last_vcc_warn_time > 30:
                self.last_vcc_warn_time = now
                self.say("Vcc %.1f" % Vcc)


    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "BATTERY_STATUS":
            self.battery_update(m)
        if mtype == "BATTERY2":
            self.battery2_voltage = m.voltage * 0.001
        if mtype == "POWER_STATUS":
            self.power_status_update(m)
        if self.battery_period.trigger():
            self.battery_report()

def init(mpstate):
    '''initialise module'''
    return BatteryModule(mpstate)
