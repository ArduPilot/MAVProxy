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
        self.battery_level = -1
        self.voltage_level = -1
        self.current_battery = -1
        self.per_cell = 0
        self.settings.append(
            MPSetting('battwarn', int, 1, 'Battery Warning Time'))
        self.settings.append(
            MPSetting('batwarncell', float, 3.7, 'Battery cell Warning level'))
        self.settings.append(MPSetting('numcells', int, 0, range=(0,10), increment=1))
        self.battery_period = mavutil.periodic_event(5)

    def cmd_bat(self, args):
        '''show battery levels'''
        print("Flight battery:   %u%%" % self.battery_level)
        if self.settings.numcells != 0:
            print("%.2f V/cell for %u cells - approx %u%%" % (self.per_cell,
                                                              self.settings.numcells,
                                                              self.vcell_to_battery_percent(self.per_cell)))

    def battery_report(self):
        batt_mon = int(self.get_mav_param('BATT_MONITOR',0))

        #report voltage level only 
        if batt_mon == 3:
            self.console.set_status('Battery', 'Batt: %.2fV' % (float(self.voltage_level) / 1000.0), row=1)
        elif batt_mon == 4:
            self.console.set_status('Battery', 'Batt: %u%%/%.2fV %.1fA' % (self.battery_level, (float(self.voltage_level) / 1000.0), self.current_battery / 100.0 ), row=1)
 
        else:
            #clear battery status
            self.console.set_status('Battery', row=1)

        rbattery_level = int((self.battery_level+5)/10)*10
        if self.settings.battwarn > 0 and time.time() > self.last_battery_announce_time + 60*self.settings.battwarn:
            self.last_battery_announce_time = time.time()
            if rbattery_level != self.last_battery_announce:
                self.say("Flight battery %u percent" % rbattery_level, priority='notification')
                self.last_battery_announce = rbattery_level
            #check voltage level to ensure we've actually received data about
            #the battery (prevents false positive warning at startup)
            if self.voltage_level != -1 and rbattery_level <= 20:
                self.say("Flight battery warning")

        if self.settings.numcells != 0 and self.per_cell < self.settings.batwarncell and time.time() > self.last_battery_cell_announce_time + 60*self.settings.battwarn:
            self.say("Cell warning")
            self.last_battery_cell_announce_time = time.time()
            

    def vcell_to_battery_percent(self, vcell):
        '''convert a cell voltage to an approximate
        percentage battery level for a LiPO'''
        if vcell > 4.1:
            # above 4.1 is 100% battery
            return 100.0
        elif vcell > 3.81:
            # 3.81 is 17% remaining, from flight logs
            return 17.0 + 83.0 * (vcell - 3.81) / (4.1 - 3.81)
        elif vcell > 3.2:
            # below 3.2 it degrades fast. It's dead at 3.2
            return 0.0 + 17.0 * (vcell - 3.20) / (3.81 - 3.20)
        # it's dead or disconnected
        return 0.0


    def battery_update(self, SYS_STATUS):
        '''update battery level'''
        # main flight battery
        self.battery_level = SYS_STATUS.battery_remaining
        self.voltage_level = SYS_STATUS.voltage_battery
        self.current_battery = SYS_STATUS.current_battery
        if self.settings.numcells != 0:
            self.per_cell = (self.voltage_level*0.001) / self.settings.numcells

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "SYS_STATUS":
            self.battery_update(m)
        if self.battery_period.trigger():
            self.battery_report()

def init(mpstate):
    '''initialise module'''
    return BatteryModule(mpstate)
