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
from MAVProxy.modules.lib import mp_settings

mpstate = None

class module_state(object):
    def __init__(self):
        self.lat = None
        self.lon = None
        self.heading = 0
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.rally_change_time = 0
        self.have_simstate = False
        self.have_vehicle = {}
        self.move_wp = -1
        self.moving_wp = 0
        self.icon_counter = 0
        self.click_position = None
        self.draw_line = None
        self.draw_callback = None
        self.vehicle_type = 'plane'
        self.circle_fence = None
        self.last_service = ''
        self.settings = mp_settings.MPSettings(
            [ ('showgpspos', int, 0),
              ('showsimpos', int, 0),
              ('brightness', float, 1),
              ('service', str, 'YahooSat')])

class circle_fence(object):
    def __init__(self, mpstate):
        self.mpstate = mpstate
        self.home = None
        self.enabled = None
        self.type = None
        self.action = None
        self.radius = None
        self.alt = None
        self.margin = None

    def refresh_params(self):
        if mpstate.status.wploader.count() > 0:
            self.home = mpstate.status.wploader.wp(0)
        self.enabled = self.getparam('FENCE_ENABLE')
        self.type = self.getparam('FENCE_TYPE')
        self.action = self.getparam('FENCE_ACTION')
        self.radius = self.getparam('FENCE_RADIUS')
        self.alt = self.getparam('FENCE_ALT_MAX')
        self.margin = self.getparam('FENCE_MARGIN')

    def getparam(self, param, default=None):
        return self.mpstate.mav_param.get(param, default)

    def refresh_display(self, force=False):
        '''Refresh display if neccessary'''
        if self.home is None and mpstate.status.wploader.count() == 0:
            # Home is not set, so neither the fence
            return
        if mpstate.status.wploader.count() > 0:
            wp_home = mpstate.status.wploader.wp(0)
        else:
            wp_home = None
        if (force or self.home != wp_home or
                self.enabled != self.getparam('FENCE_ENABLE') or
                self.radius != self.getparam('FENCE_RADIUS')):
            self.refresh_params()
            #TODO: change color to suit enabled/disabled status
            if self.enabled:
                colour = (255,0,0)
            else:
                colour = (0,255,0)
            if self.type == 2 or  self.type == 3:
                # only display if it's in Circle or Altitude_and_Circle mode
                fence_drawing = mp_slipmap.SlipCircle('fence', (self.home.x, self.home.y), self.radius, layer=1, linewidth=2, colour=colour)
                # print("bounding box: %f %f %f %f" % fence_drawing.bounds())
                self.mpstate.map.add_object(fence_drawing)

def name():
    '''return module name'''
    return "map"

def description():
    '''return module description'''
    return "map display"

def cmd_map(args):
    '''map commands'''
    state = mpstate.map_state
    if args[0] == "icon":
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
    elif args[0] == "set":
        if len(args) < 3:
            state.settings.show_all()
        else:
            state.settings.set(args[1], args[2])
            mpstate.map.add_object(mp_slipmap.SlipBrightness(state.settings.brightness))
    else:
        print("usage: map <brightness|icon|grid|set>")

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.map_state = module_state()
    if 'MAP_SERVICE' in os.environ:
        mpstate.map_state.set('service', os.environ['MAP_SERVICE'])
    import platform
    mpstate.map = mp_slipmap.MPSlipMap(service=mpstate.map_state.settings.service, elevation=True, title='Map')
    mpstate.map_functions = { 'draw_lines' : draw_lines }

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
    elif obj.event.m_leftDown:
        state.click_position = obj.latlon
        drawing_update()
    if obj.event.m_rightDown:
        if state.draw_callback is not None:
            drawing_end()
            return
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
    mpstate.map_functions = {}

def create_vehicle_icon(name, colour, follow=False):
    '''add a vehicle to the map'''
    state = mpstate.map_state
    if name in mpstate.map_state.have_vehicle and mpstate.map_state.have_vehicle[name] == state.vehicle_type:
        return
    mpstate.map_state.have_vehicle[name] = state.vehicle_type
    icon = mpstate.map.icon(colour + state.vehicle_type + '.png')
    mpstate.map.add_object(mp_slipmap.SlipIcon(name, (0,0), icon, layer=3, rotation=0, follow=follow,
                                               trail=mp_slipmap.SlipTrail()))

def drawing_update():
    '''update line drawing'''
    state = mpstate.map_state
    if state.draw_callback is None:
        return
    state.draw_line.append(state.click_position)
    if len(state.draw_line) > 1:
        mpstate.map.add_object(mp_slipmap.SlipPolygon('drawing', state.draw_line,
                                                      layer='Drawing', linewidth=2, colour=(128,128,255)))

def drawing_end():
    '''end line drawing'''
    state = mpstate.map_state
    if state.draw_callback is None:
        return
    state.draw_callback(state.draw_line)
    state.draw_callback = None
    mpstate.map.add_object(mp_slipmap.SlipClearLayer('Drawing'))

def draw_lines(callback):
    '''draw a series of connected lines on the map, calling callback when done'''
    state = mpstate.map_state
    state.draw_callback = callback
    state.draw_line = []
    
def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.map_state

    if state.last_service != state.settings.service:
        state.last_service = state.settings.service
        mpstate.map.add_object(mp_slipmap.SlipService(state.settings.service))

    if m.get_type() == "HEARTBEAT":
        from pymavlink import mavutil
        if m.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING]:
            state.vehicle_type = 'plane'
        elif m.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                        mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                        mavutil.mavlink.MAV_TYPE_SUBMARINE]:
            state.vehicle_type = 'rover'
        elif m.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER]:
            state.vehicle_type = 'copter'

    if m.get_type() == "SIMSTATE" and state.settings.showsimpos:
        create_vehicle_icon('SimVehicle', 'green')
        mpstate.map.set_position('SimVehicle', (m.lat*1.0e-7, m.lng*1.0e-7), rotation=math.degrees(m.yaw))

    if m.get_type() == "GPS_RAW_INT" and state.settings.showgpspos:
        (lat, lon) = (m.lat*1.0e-7, m.lon*1.0e-7)
        if lat != 0 or lon != 0:
            create_vehicle_icon('GPSVehicle', 'blue')
            mpstate.map.set_position('GPSVehicle', (lat, lon), rotation=m.cog*0.01)

    if m.get_type() == 'GLOBAL_POSITION_INT':
        (state.lat, state.lon, state.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
        if state.lat != 0 or state.lon != 0:
            create_vehicle_icon('PosVehicle', 'red', follow=True)
            mpstate.map.set_position('PosVehicle', (state.lat, state.lon), rotation=state.heading)

    if m.get_type() == "NAV_CONTROLLER_OUTPUT":
        if mpstate.master().flightmode in [ "AUTO", "GUIDED", "LOITER", "RTL" ]:
            trajectory = [ (state.lat, state.lon),
                           mp_util.gps_newpos(state.lat, state.lon, m.target_bearing, m.wp_dist) ]
            mpstate.map.add_object(mp_slipmap.SlipPolygon('trajectory', trajectory, layer='Trajectory',
                                                          linewidth=2, colour=(255,0,180)))
        else:
            mpstate.map.add_object(mp_slipmap.SlipClearLayer('Trajectory'))

        
    # if the waypoints have changed, redisplay
    if state.wp_change_time != mpstate.status.wploader.last_change:
        state.wp_change_time = mpstate.status.wploader.last_change
        display_waypoints()

    # if the fence has changed, redisplay
    if state.vehicle_type == 'copter':
        if state.circle_fence is None:
            # Refresh waypoints to get home location
            mpstate.status.wp_op = "fetch_home"
            mpstate.master().waypoint_request_list_send()
            state.circle_fence = circle_fence(mpstate)
            state.circle_fence.refresh_display(force=True)
        else:
            state.circle_fence.refresh_display()
    elif state.fence_change_time != mpstate.status.fenceloader.last_change:
        state.fence_change_time = mpstate.status.fenceloader.last_change
        points = mpstate.status.fenceloader.polygon()
        if len(points) > 1:
            mpstate.map.add_object(mp_slipmap.SlipPolygon('fence', points, layer=1, linewidth=2, colour=(0,255,0)))

    # if the rallypoints have changed, redisplay
    if state.rally_change_time != mpstate.status.rallyloader.last_change:
        state.rally_change_time = mpstate.status.rallyloader.last_change
        icon = mpstate.map.icon('rallypoint.png')
        mpstate.map.add_object(mp_slipmap.SlipClearLayer('RallyPoints'))
        for i in range(mpstate.status.rallyloader.rally_count()):
            rp = mpstate.status.rallyloader.rally_point(i)
            mpstate.map.add_object(mp_slipmap.SlipIcon('Rally-%u' % i, (rp.lat*1.0e-7, rp.lng*1.0e-7), icon,
                                                       layer='RallyPoints', rotation=0, follow=False))

    # check for any events from the map
    mpstate.map.check_events()
    
