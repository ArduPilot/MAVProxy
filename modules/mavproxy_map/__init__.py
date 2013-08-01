#!/usr/bin/env python
'''
map display module
Andrew Tridgell
June 2012
'''

import sys, os, math
import functools
import time
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_util

mpstate = None

class module_state(object):
    def __init__(self):
        self.lat = None
        self.lon = None
        self.heading = 0
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.have_simstate = False
        self.have_blueplane = False
        self.move_wp = -1
        self.moving_wp = 0
        self.brightness = 1
        self.icon_counter = 0
        self.click_position = None

def name():
    '''return module name'''
    return "map"

def description():
    '''return module description'''
    return "map display"

def cmd_map(args):
    '''map commands'''
    state = mpstate.map_state
    if args[0] == "brightness":
        if len(args) < 2:
            print("Brightness %.1f" % state.brightness)
        else:
            state.brightness = float(args[1])
            mpstate.map.add_object(mp_slipmap.SlipBrightness(state.brightness))
    elif args[0] == "icon":
        if len(args) < 3:
            print("Usage: map icon <lat> <lon> <icon>")
        else:
            lat = args[1]
            lon = args[2]
            flag = 'flag.png'
            if len(args) > 3:
                flag = args[3] + '.png'
            icon = mpstate.map.icon(flag)
            mpstate.map.add_object(mp_slipmap.SlipIcon('icon - %s [%u]' % (str(flag),state.icon_counter),
                                                       (float(lat),float(lon)),
                                               icon, layer=3, rotation=0, follow=False))
            state.icon_counter += 1
    elif args[0] == "grid":
        mpstate.map.add_object(mp_slipmap.SlipGrid('grid', layer=3, linewidth=1, colour=(255,255,0)))
    else:
        print("usage: map <brightness|icon|grid>")

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.map_state = module_state()
    service = 'GoogleSat'
    import platform
    if platform.system() == 'Windows':
        # windows has trouble with Google tile URLs
        service = 'MicrosoftSat'
    mpstate.map = mp_slipmap.MPSlipMap(service=service, elevation=True, title='Map')

    # setup a plane icon
    icon = mpstate.map.icon('planetracker.png')
    mpstate.map.add_object(mp_slipmap.SlipIcon('plane', (0,0), icon, layer=3, rotation=0,
                                               follow=True,
                                               trail=mp_slipmap.SlipTrail()))

    mpstate.map.add_callback(functools.partial(map_callback))
    mpstate.command_map['map'] = (cmd_map, "map control")


def display_waypoints():
    '''display the waypoints'''
    polygons = mpstate.status.wploader.polygon_list()
    mpstate.map.add_object(mp_slipmap.SlipClearLayer('Mission'))
    for i in range(len(polygons)):
        p = polygons[i]
        if len(p) > 1:
            mpstate.map.add_object(mp_slipmap.SlipPolygon('mission%u' % i, p,
                                                          layer='Mission', linewidth=2, colour=(255,255,255)))

def closest_waypoint(latlon):
    '''find closest waypoint to a position'''
    (lat, lon) = latlon
    best_distance = -1
    closest = -1
    for i in range(mpstate.status.wploader.count()):
        w = mpstate.status.wploader.wp(i)
        distance = mp_util.gps_distance(lat, lon, w.x, w.y)
        if best_distance == -1 or distance < best_distance:
            best_distance = distance
            closest = i
    if best_distance < 20:
        return closest
    else:
        return -1
        

def map_callback(obj):
    '''called when an event happens on the slipmap'''
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    state = mpstate.map_state
    if not isinstance(obj, mp_slipmap.SlipMouseEvent):
        return
    if obj.event.m_leftDown and state.moving_wp != 0:
        state.moving_wp = 0
        print("cancelled WP move")
    else:
        state.click_position = obj.latlon
    if obj.event.m_rightDown:
        if state.moving_wp == 0:
            wpnum = closest_waypoint(obj.latlon)
            if wpnum != -1:
                state.moving_wp = time.time()
                state.move_wp = wpnum
                wp = mpstate.status.wploader.wp(state.move_wp)
                print("Selected WP %u : %s" % (wpnum, getattr(wp,'comment','')))
        elif time.time() - state.moving_wp >= 1:
            wp = mpstate.status.wploader.wp(state.move_wp)
            (lat, lon) = obj.latlon
            if getattr(mpstate.console, 'ElevationMap', None) is not None:
                alt1 = mpstate.console.ElevationMap.GetElevation(lat, lon) 
                alt2 = mpstate.console.ElevationMap.GetElevation(wp.x, wp.y)
                wp.z += alt1 - alt2
            wp.x = lat
            wp.y = lon
            
            wp.target_system    = mpstate.status.target_system
            wp.target_component = mpstate.status.target_component
            state.moving_wp = 0
            mpstate.status.loading_waypoints = True
            mpstate.status.loading_waypoint_lasttime = time.time()
            mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                                 mpstate.status.target_component,
                                                                 state.move_wp, state.move_wp)
            print("Moved WP %u to %f, %f at %.1fm" % (state.move_wp, lat, lon, wp.z))
            display_waypoints()
            
        

def unload():
    '''unload module'''
    mpstate.map = None

def create_blueplane():
    '''add the blue plane to the map'''
    if mpstate.map_state.have_blueplane:
        return
    mpstate.map_state.have_blueplane = True
    icon = mpstate.map.icon('blueplane.png')
    mpstate.map.add_object(mp_slipmap.SlipIcon('blueplane', (0,0), icon, layer=3, rotation=0,
                                               trail=mp_slipmap.SlipTrail()))

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.map_state

    if m.get_type() == "SIMSTATE":
        if not mpstate.map_state.have_simstate:
            mpstate.map_state.have_simstate  = True
            create_blueplane()
        mpstate.map.set_position('blueplane', (m.lat, m.lng), rotation=math.degrees(m.yaw))

    if m.get_type() == "GPS_RAW_INT" and not mpstate.map_state.have_simstate:
        (lat, lon) = (m.lat*1.0e-7, m.lon*1.0e-7)
        if state.lat is not None and (mpstate.map_state.have_blueplane or
                                      mp_util.gps_distance(lat, lon, state.lat, state.lon) > 10):
            create_blueplane()
            mpstate.map.set_position('blueplane', (lat, lon), rotation=m.cog*0.01)

    if m.get_type() == "NAV_CONTROLLER_OUTPUT":
        if mpstate.master().flightmode in [ "AUTO", "GUIDED", "LOITER", "RTL" ]:
            trajectory = [ (state.lat, state.lon),
                           mp_util.gps_newpos(state.lat, state.lon, m.target_bearing, m.wp_dist) ]
            mpstate.map.add_object(mp_slipmap.SlipPolygon('trajectory', trajectory, layer='Trajectory',
                                                          linewidth=2, colour=(255,0,180)))
        else:
            mpstate.map.add_object(mp_slipmap.SlipClearLayer('Trajectory'))

        
    if m.get_type() == 'GLOBAL_POSITION_INT':
        (state.lat, state.lon, state.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
    else:
        return

    if state.lat != 0 or state.lon != 0:
        mpstate.map.set_position('plane', (state.lat, state.lon), rotation=state.heading)

    # if the waypoints have changed, redisplay
    if state.wp_change_time != mpstate.status.wploader.last_change:
        state.wp_change_time = mpstate.status.wploader.last_change
        display_waypoints()

    # if the fence has changed, redisplay
    if state.fence_change_time != mpstate.status.fenceloader.last_change:
        state.fence_change_time = mpstate.status.fenceloader.last_change
        points = mpstate.status.fenceloader.polygon()
        if len(points) > 1:
            mpstate.map.add_object(mp_slipmap.SlipPolygon('fence', points, layer=1, linewidth=2, colour=(0,255,0)))

    # check for any events from the map
    mpstate.map.check_events()
    
