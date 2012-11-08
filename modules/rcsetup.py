#!/usr/bin/env python
'''RC min/max setup'''

def name():
    '''return module name'''
    return "rcsetup"

def description():
    '''return module description'''
    return "RC setup"

def cmd_rcreset(args):
    '''reset RC min/max'''
    for ch in range(1,5):
        mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MIN' % ch, 1500)
        mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MAX' % ch, 1500)
    print("Reset all channels MIN/MAX to 1500")

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
    mpstate.command_map['rcreset'] = (cmd_rcreset, "RC min/max reset")
    mpstate.command_map['rctrim'] = (cmd_rctrim, "RC min/max trim")
    print("rcsetup initialised")

def unload():
    if 'rcreset' in mpstate.command_map:
        mpstate.command_map.pop('rcreset')
    if 'rctrim' in mpstate.command_map:
        mpstate.command_map.pop('rctrim')

    
def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    if m.get_type() == 'RC_CHANNELS_RAW':
        for i in range(1,9):
            v = getattr(m, 'chan%u_raw' % i)
            rcmin = mpstate.mav_param.get('RC%u_MIN' % i, 0)
            rcmax = mpstate.mav_param.get('RC%u_MAX' % i, 0)
            if rcmin > v:
                if mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MIN' % i, v):
                    mpstate.console.writeln("Set RC%u_MIN=%u" % (i, v))
            if rcmax < v:
                if mpstate.mav_param.mavset(mpstate.master(), 'RC%u_MAX' % i, v):
                    mpstate.console.writeln("Set RC%u_MAX=%u" % (i, v))

