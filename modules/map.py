#!/usr/bin/env python
'''
map display module
Andrew Tridgell
June 2012
'''

import sys, os, cv
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import mp_slipmap

mpstate = None

class module_state(object):
    def __init__(self):
        self.lat = None
        self.lon = None
        self.heading = 0
        self.wp_change_time = 0
        self.fence_change_time = 0

def name():
    '''return module name'''
    return "map"

def description():
    '''return module description'''
    return "map display"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.map_state = module_state()
    mpstate.map = mp_slipmap.MPSlipMap(service='GoogleSat')

    # setup a plane icon
    plane = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                         'data', 'planetracker.png')
    icon = cv.LoadImage(plane)
    mpstate.map.add_object(mp_slipmap.SlipIcon('plane', (0,0), icon, layer=3, rotation=0,
                                               follow=True,
                                               trail=mp_slipmap.SlipTrail()))

def unload():
    '''unload module'''
    mpstate.map = None

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.map_state
    if m.get_type() == 'GPS_RAW':
        (state.lat, state.lon) = (m.lat, m.lon)
        state.heading = m.hdg
    elif m.get_type() == 'GPS_RAW_INT':
        (state.lat, state.lon) = (m.lat/1.0e7, m.lon/1.0e7)
        state.heading = m.cog*0.01
    else:
        return

    mpstate.map.set_icon_position('plane', (state.lat, state.lon), rotation=state.heading)

    # if the waypoints have changed, redisplay
    if state.wp_change_time != mpstate.status.wploader.last_change:
        state.wp_change_time = mpstate.status.wploader.last_change
        points = mpstate.status.wploader.polygon()
        if len(points) > 1:
            mpstate.map.add_polygon('mission', points, layer=1, linewidth=2, colour=(255,255,255))

    # if the fence has changed, redisplay
    if state.fence_change_time != mpstate.status.fenceloader.last_change:
        state.fence_change_time = mpstate.status.fenceloader.last_change
        points = mpstate.status.fenceloader.polygon()
        if len(points) > 1:
            mpstate.map.add_polygon('fence', points, layer=1, linewidth=2, colour=(0,255,0))
