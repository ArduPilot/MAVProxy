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
     (3, 500,  1500)],

    # This supports a Spektrum DX7s with the USB Adapter from:
    # http://www.amazon.com/gp/product/B000RO7JAI/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1
    'WAILLY PPM TO USB Adapter':
    [(1, 570, 1500), # Roll
     (2, 614, 1500), # Pitch
     (0, 770, 1500), # Throttle
     (5, 571, 1500), # Yaw
     None,
     None],

    # Support for Optic 6 with the IPACS easyFly2 Interface USB Adapter from:
    # http://www.ikarus.net/deutsch-infos-zubehor/die-ikarus-interfacekabel/?lang=en
    'IPACS easyFly2 Interface':
    [(0, 758, 1242), # Roll
     (1, 758, 1242), # Pitch
     (2, 770, 1230), # Throttle
     (4, 2553, 974), # (custom: flaps?)
     None,
     (3, 758, 1242), # Yaw
     (5, 758, 1242), # (custom: switch)
     None],

    # This supports a ADC cheap ebay USB controller
    # sold as a "FMS Simulator" joystick
    'ADC':
    [(0, -500, 1500), # Roll
     (1, -500, 1500), # Pitch
     (2, -500, 1500), # Throttle
     (4, 500, 1500), # Yaw
     None,
     None]
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
            if e.type in [
                pygame.JOYAXISMOTION,
                pygame.JOYBALLMOTION,
                pygame.JOYBUTTONDOWN,
                pygame.JOYBUTTONUP,
                pygame.JOYHATMOTION
            ] :
                # print '--- %s ' % pygame.event.event_name(e.type)
                # print 'axis %u : %f' % (e.axis, e.value)
                m = self.map[e.axis]
                if m is None:
                    continue
                (axis, mul, add) = m
                if axis >= self.num_axes:
                    continue
                v = int(e.value * mul + add)
                v = max(min(v, 2000), 1000)
                # if e.axis == 0 :
                #     print 'e.axis: %u, axis: %u, v: %u' % (e.axis, axis, v)
                override = self.module('rc').override[:]
                override[e.axis] = v
                if override != self.module('rc').override:
                    self.module('rc').override = override
                    self.module('rc').override_period.force()

def init(mpstate):
    '''initialise module'''
    return JSModule(mpstate)
