#!/usr/bin/env python
'''RC min/max setup'''

from MAVProxy.modules.lib import mp_module

class RCSetupModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RCSetupModule, self).__init__(mpstate, "rcsetup")
        self.calibrating = False
        self.num_channels = 4
        self.clear_rc_cal()
        self.add_command('rccal', self.cmd_rccal, "RC calibration start/stop")
        self.add_command('rctrim', self.cmd_rctrim, "RC min/max trim")
        print("rcsetup initialised")

    def clear_rc_cal(self):
        self.rc_cal = []
        self.rc_cal.append("") # 0 will be empty
        for i in range(1,self.num_channels+1):
            #min, max, modified
            self.rc_cal.append([1500, 1500, False])

    def apply_rc_cal(self):
        for i in range(1, len(self.rc_cal)):
            #only apply calibration changes to channels that
            #were modified during calibration
            if (self.rc_cal[i][2] == False):
                continue

            self.param_set('RC%u_MIN' % i, self.rc_cal[i][0], 5)
            self.console.writeln("Set: RC%u_MIN=%u" % (i, self.rc_cal[i][0]))
            self.param_set('RC%u_MAX' % i, self.rc_cal[i][1], 5)
            self.console.writeln("Set: RC%u_MAX=%u" % (i, self.rc_cal[i][1]))

    def get_cal_min(self, channel):
        return self.rc_cal[channel][0]

    def get_cal_max(self, channel):
        return self.rc_cal[channel][1]

    def set_cal_min(self, channel, val):
        self.rc_cal[channel][0] = val
        self.rc_cal[channel][2] = True

    def set_cal_max(self, channel, val):
        self.rc_cal[channel][1] = val
        self.rc_cal[channel][2] = True

    def cmd_rccal(self, args):
        '''start/stop RC calibration'''
        if len(args) < 1:
            self.print_cal_usage()
            return

        if (args[0] == "start"):
            if len(args) > 1:
                self.num_channels = int(args[1])
            print("Calibrating %u channels" % self.num_channels)
            print("WARNING: remove propellers from electric planes!!")
            print("Push return when ready to calibrate.")
            raw_input()

            self.clear_rc_cal()
            self.calibrating = True
        elif (args[0] == "done"):
            self.calibrating = False
            self.apply_rc_cal()
        else:
            self.print_cal_usage()

    def cmd_rctrim(self, args):
        '''set RCx_TRIM'''
        if not 'RC_CHANNELS_RAW' in self.status.msgs:
            print("No RC_CHANNELS_RAW to trim with")
            return
        m = self.status.msgs['RC_CHANNELS_RAW']
        for ch in range(1,5):
            self.param_set('RC%u_TRIM' % ch, getattr(m, 'chan%u_raw' % ch))


    def unload(self):
        if 'rcreset' in self.mpstate.command_map:
            self.mpstate.command_map.pop('rcreset')
        if 'rctrim' in self.mpstate.command_map:
            self.mpstate.command_map.pop('rctrim')


    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        #do nothing if not caibrating
        if (self.calibrating == False):
            return

        if m.get_type() == 'RC_CHANNELS_RAW':
            for i in range(1,self.num_channels+1):
                v = getattr(m, 'chan%u_raw' % i)

                if self.get_cal_min(i) > v:
                    self.set_cal_min(i,v)
                    self.console.writeln("Calibrating: RC%u_MIN=%u" % (i, v))
                if self.get_cal_max(i) < v:
                    self.set_cal_max(i,v)
                    self.console.writeln("Calibrating: RC%u_MAX=%u" % (i, v))

    def print_cal_usage(self):
        print("Usage rccal <start|done>")

def init(mpstate):
    '''initialise module'''
    return RCSetupModule(mpstate)
