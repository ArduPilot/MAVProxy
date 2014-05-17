#!/usr/bin/env python
'''joystick interface module

Contributed by AndrewF:
  http://diydrones.com/profile/AndrewF

'''

import pygame, fnmatch
from time import sleep

mpstate = None

from MAVProxy.modules.lib import mp_module


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


class JSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(JSModule, self).__init__(mpstate, "joystick", "joystick aircraft control")
        self.js = None

        #initialize joystick, if available
        pygame.init()
        pygame.joystick.init() # main joystick device system

        for i in range(pygame.joystick.get_count()):
            print("Trying joystick %u" % i)
            try:
                j = pygame.joystick.Joystick(i)
                j.init() # init instance
                name = j.get_name()
                print('joystick found: ' + name)
                for jtype in joymap:
                    if fnmatch.fnmatch(name, jtype):
                        print("Matched type '%s'" % jtype)
                        print '%u axes available' % j.get_numaxes()
                        self.js = j
                        self.num_axes = j.get_numaxes()
                        self.map = joymap[jtype]
                        break
            except pygame.error:
                continue

    def idle_task(self):
        '''called in idle time'''
        if self.js is None:
            return
        for e in pygame.event.get(): # iterate over event stack
            #the following is somewhat custom for the specific joystick model:
            override = self.module('rc').override[:]
            for i in range(len(self.map)):
                m = self.map[i]
                if m is None:
                    continue
                (axis, mul, add) = m
                if axis >= self.num_axes:
                    continue
                v = int(self.js.get_axis(axis)*mul + add)
                v = max(min(v, 2000), 1000)
                override[i] = v
            if override != self.module('rc').override:
                self.module('rc').override = override
                self.module('rc').override_period.force()

def init(mpstate):
    '''initialise module'''
    return JSModule(mpstate)
