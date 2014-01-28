#!/usr/bin/env python
'''RC min/max setup'''

class rc_state(object):
    def __init__(self):
        self.calibrating = False
        self.clear_rc_cal()

    def clear_rc_cal(self):
        self.rc_cal = []
        self.rc_cal.append("") # 0 will be empty
        for i in range(1,9):
            #min, max, modified
            self.rc_cal.append([1500, 1500, False])

    def apply_rc_cal(self):
        for i in range(1, len(self.rc_cal)):
            #only apply calibration changes to channels that
            #were modified during calibration
            if (self.rc_cal[i][2] == False):
                continue

            mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MIN' % i, self.rc_cal[i][0], 5)
            mpstate.console.writeln("Set: RC%u_MIN=%u" % (i, self.rc_cal[i][0]))
            mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MAX' % i, self.rc_cal[i][1], 5)
            mpstate.console.writeln("Set: RC%u_MAX=%u" % (i, self.rc_cal[i][1]))

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

def name():
    '''return module name'''
    return "rcsetup"

def description():
    '''return module description'''
    return "RC setup"

def cmd_rccal(args):
    '''start/stop RC calibration'''
    if len(args) < 1:
        print_cal_usage()
        return

    if (args[0] == "start"):
        print "WARNING: remove propellers from electric planes!!"
        print "Push return when ready to calibrate."
        raw_input()

        mpstate.rc_state.clear_rc_cal()
        mpstate.rc_state.calibrating = True
    elif (args[0] == "done"):
        mpstate.rc_state.calibrating = False
        mpstate.rc_state.apply_rc_cal()
    else:
        print_cal_usage()

def cmd_rctrim(args):
    '''set RCx_TRIM'''
    if not 'RC_CHANNELS_RAW' in mpstate.status.msgs:
        print("No RC_CHANNELS_RAW to trim with")
        return
    m = mpstate.status.msgs['RC_CHANNELS_RAW']
    for ch in range(1,5):
        mpstate.mav_param.mavset(mpstate.master(), 'RC%u_TRIM' % ch, getattr(m, 'chan%u_raw' % ch))

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.rc_state = rc_state()
    mpstate.command_map['rccal'] = (cmd_rccal, "RC calibration start/stop")
    mpstate.command_map['rctrim'] = (cmd_rctrim, "RC min/max trim")
    print("rcsetup initialised")

def unload():
    if 'rcreset' in mpstate.command_map:
        mpstate.command_map.pop('rcreset')
    if 'rctrim' in mpstate.command_map:
        mpstate.command_map.pop('rctrim')

    
def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    #do nothing if not caibrating
    if (mpstate.rc_state.calibrating == False):
        return

    if m.get_type() == 'RC_CHANNELS_RAW':
        for i in range(1,9):
            v = getattr(m, 'chan%u_raw' % i)

            if mpstate.rc_state.get_cal_min(i) > v:
                mpstate.rc_state.set_cal_min(i,v)
                mpstate.console.writeln("Calibrating: RC%u_MIN=%u" % (i, v))
            if mpstate.rc_state.get_cal_max(i) < v:
                mpstate.rc_state.set_cal_max(i,v)
                mpstate.console.writeln("Calibrating: RC%u_MAX=%u" % (i, v))

def print_cal_usage():
    print("Usage rccal <start|done>")

