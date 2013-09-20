#!/usr/bin/env python
'''joystick interface module

Contributed by AndrewF:
  http://diydrones.com/profile/AndrewF

'''

import pygame, fnmatch
from time import sleep

mpstate = None

class module_state(object):
    def __init__(self):
        self.js = None

'''
A map of joystick identifiers to channels and scalings.
Each joystick type can control 8 channels, each channel is defined
by its axis number, the multiplier and the additive offset
'''
joymap = {
    'CarolBox USB*':
    # http://www.hobbyking.com/hobbyking/store/__13597__USB_Simulator_Cable_XTR_AeroFly_FMS.html
    # has 6 usable axes. This assumes mode 1
    [(3, 500, 1500),
     (0, 500, 1500),
     (1, 700, 1500),
     (4, 500, 1500),
     (5, 500, 1500),
     None,
     (2, 500, 1500),
     (5, 500, 1500)],
    
    'Sony PLAYSTATION(R)3 Controller':
    # only 4 axes usable. This assumes mode 1
    [(2, 500,  1500),
     (1, -500,  1500),
     (3, -500, 1000),
     (0, -500,  1500)],

    'GREAT PLANES InterLink Elite':
    # 4 axes usable
    [(0, 500,  1500),
     (1, -500,  1500),
     (2, -1000, 1500),
     (4, -500,  1500),
     None,
     None,
     None,
     (3, 500,  1500)],

    'Great Planes GP Controller':
    # 4 axes usable
    [(0, 500,  1500),
     (1, -500,  1500),
     (2, -1000, 1500),
     (4, -500,  1500),
     None,
     None,
     None,
     (3, 500,  1500)]
}

def idle_task():
    '''called in idle time'''
    state = mpstate.joystick_state
    if state.js is None:
        return
    for e in pygame.event.get(): # iterate over event stack
        #the following is somewhat custom for the specific joystick model:
        override = mpstate.status.override[:]
        for i in range(len(state.map)):
            m = state.map[i]
            if m is None:
                continue
            (axis, mul, add) = m
            if axis >= state.num_axes:
                continue
            v = int(state.js.get_axis(axis)*mul + add)
            v = max(min(v, 2000), 1000)
            override[i] = v
        if override != mpstate.status.override:
            mpstate.status.override = override
            mpstate.override_period.force()

def name():
    '''return module name'''
    return "joystick"

def description():
    '''return module description'''
    return "joystick aircraft control"

def mavlink_packet(pkt):
    pass

def unload():
    '''unload module'''
    pass

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    state = module_state()
    mpstate.joystick_state = state
    
    #initialize joystick, if available
    pygame.init()
    pygame.joystick.init() # main joystick device system

    for i in range(pygame.joystick.get_count()):
        print("Trying joystick %u" % i)
        try:
            j = pygame.joystick.Joystick(i)
            j.init() # init instance
            name = j.get_name()
            print 'joystick found: ' + name
            for jtype in joymap:
                if fnmatch.fnmatch(name, jtype):
                    print "Matched type '%s'" % jtype
                    print '%u axes available' % j.get_numaxes()
                    state.js = j
                    state.num_axes = j.get_numaxes()
                    state.map = joymap[jtype]
                    break
        except pygame.error:
            continue    

if __name__ == "__main__":
    class dummy(object):
        def __init__(self):
            pass
    d = dummy()
    init(d)
    
