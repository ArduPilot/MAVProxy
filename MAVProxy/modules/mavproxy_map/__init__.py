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
from MAVProxy.modules.lib import mp_module

class MapModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MapModule, self).__init__(mpstate, "map", "map display", public = True)
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
        self.click_time = 0
        self.draw_line = None
        self.draw_callback = None
        self.vehicle_type_name = 'plane'
        self.map_settings = mp_settings.MPSettings(
            [ ('showgpspos', int, 0),
              ('showgps2pos', int, 1),
              ('showsimpos', int, 0),
              ('showahrs2pos', int, 0),
              ('brightness', float, 1)])
        service='YahooSat'
        if 'MAP_SERVICE' in os.environ:
            service = os.environ['MAP_SERVICE']
        import platform
        mpstate.map = mp_slipmap.MPSlipMap(service=service, elevation=True, title='Map')
        mpstate.map_functions = { 'draw_lines' : self.draw_lines }
    
        mpstate.map.add_callback(functools.partial(self.map_callback))
        self.add_command('map', self.cmd_map, "map control", ['icon',
                                      'set (MAPSETTING)'])
        self.add_completion_function('(MAPSETTING)', self.map_settings.completion)
        
    def cmd_map(self, args):
        '''map commands'''
        if args[0] == "icon":
            if len(args) < 3:
                print("Usage: map icon <lat> <lon> <icon>")
            else:
                lat = args[1]
                lon = args[2]
                flag = 'flag.png'
                if len(args) > 3:
                    flag = args[3] + '.png'
                icon = self.mpstate.map.icon(flag)
                self.mpstate.map.add_object(mp_slipmap.SlipIcon('icon - %s [%u]' % (str(flag),self.icon_counter),
                                                           (float(lat),float(lon)),
                                                   icon, layer=3, rotation=0, follow=False))
                self.icon_counter += 1
        elif args[0] == "set":
            self.map_settings.command(args[1:])
            self.mpstate.map.add_object(mp_slipmap.SlipBrightness(self.map_settings.brightness))
        else:
            print("usage: map <icon|set>")
    
    def display_waypoints(self):
        '''display the waypoints'''
        polygons = self.module('wp').wploader.polygon_list()
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Mission'))
        for i in range(len(polygons)):
            p = polygons[i]
            if len(p) > 1:
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('mission%u' % i, p,
                                                              layer='Mission', linewidth=2, colour=(255,255,255)))
    
    def closest_waypoint(self, latlon):
        '''find closest waypoint to a position'''
        (lat, lon) = latlon
        best_distance = -1
        closest = -1
        for i in range(self.module('wp').wploader.count()):
            w = self.module('wp').wploader.wp(i)
            distance = mp_util.gps_distance(lat, lon, w.x, w.y)
            if best_distance == -1 or distance < best_distance:
                best_distance = distance
                closest = i
        if best_distance < 20:
            return closest
        else:
            return -1
            
    
    def map_callback(self, obj):
        '''called when an event happens on the slipmap'''
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        if not isinstance(obj, mp_slipmap.SlipMouseEvent):
            return
        if obj.event.m_leftDown and self.moving_wp != 0:
            self.moving_wp = 0
            print("cancelled WP move")
        elif obj.event.m_leftDown:
            if time.time() - self.click_time > 0.1:
                self.click_position = obj.latlon
                self.click_time = time.time()
                self.drawing_update()
        if obj.event.m_rightDown:
            if self.draw_callback is not None:
                self.drawing_end()
                return
            if self.moving_wp == 0:
                wpnum = self.closest_waypoint(obj.latlon)
                if wpnum != -1:
                    self.moving_wp = time.time()
                    self.move_wp = wpnum
                    wp = self.module('wp').wploader.wp(self.move_wp)
                    print("Selected WP %u : %s" % (wpnum, getattr(wp,'comment','')))
            elif time.time() - self.moving_wp >= 1:
                wp = self.module('wp').wploader.wp(self.move_wp)
                (lat, lon) = obj.latlon
                if getattr(self.console, 'ElevationMap', None) is not None:
                    alt1 = self.console.ElevationMap.GetElevation(lat, lon) 
                    alt2 = self.console.ElevationMap.GetElevation(wp.x, wp.y)
                    wp.z += alt1 - alt2
                wp.x = lat
                wp.y = lon
                
                wp.target_system    = self.target_system
                wp.target_component = self.target_component
                self.moving_wp = 0
                self.module('wp').loading_waypoints = True
                self.module('wp').loading_waypoint_lasttime = time.time()
                self.master.mav.mission_write_partial_list_send(self.target_system,
                                                                     self.target_component,
                                                                     self.move_wp, self.move_wp)
                print("Moved WP %u to %f, %f at %.1fm" % (self.move_wp, lat, lon, wp.z))
                self.display_waypoints()
                
            
    
    def unload(self):
        '''unload module'''
        self.mpstate.map.close()
        self.mpstate.map = None
        self.mpstate.map_functions = {}
    
    def create_vehicle_icon(self, name, colour, follow=False, vehicle_type=None):
        '''add a vehicle to the map'''
        if vehicle_type is None:
            vehicle_type = self.vehicle_type_name
        if name in self.have_vehicle and self.have_vehicle[name] == vehicle_type:
            return
        self.have_vehicle[name] = vehicle_type
        icon = self.mpstate.map.icon(colour + vehicle_type + '.png')
        self.mpstate.map.add_object(mp_slipmap.SlipIcon(name, (0,0), icon, layer=3, rotation=0, follow=follow,
                                                   trail=mp_slipmap.SlipTrail()))
    
    def drawing_update(self):
        '''update line drawing'''
        if self.draw_callback is None:
            return
        self.draw_line.append(self.click_position)
        if len(self.draw_line) > 1:
            self.mpstate.map.add_object(mp_slipmap.SlipPolygon('drawing', self.draw_line,
                                                          layer='Drawing', linewidth=2, colour=(128,128,255)))
    
    def drawing_end(self):
        '''end line drawing'''
        if self.draw_callback is None:
            return
        self.draw_callback(self.draw_line)
        self.draw_callback = None
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Drawing'))
    
    def draw_lines(self, callback):
        '''draw a series of connected lines on the map, calling callback when done'''
        self.draw_callback = callback
        self.draw_line = []
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "HEARTBEAT":
            from pymavlink import mavutil
            if m.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING]:
                self.vehicle_type_name = 'plane'
            elif m.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                            mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                            mavutil.mavlink.MAV_TYPE_SUBMARINE]:
                self.vehicle_type_name = 'rover'
            elif m.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER]:
                self.vehicle_type_name = 'copter'
            elif m.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
                self.vehicle_type_name = 'antenna'     
    
        # this is the beginnings of allowing support for multiple vehicles
        # in the air at the same time
        vehicle = 'Vehicle%u' % m.get_srcSystem()
    
        if m.get_type() == "SIMSTATE" and self.map_settings.showsimpos:
            self.create_vehicle_icon('Sim' + vehicle, 'green')
            self.mpstate.map.set_position('Sim' + vehicle, (m.lat*1.0e-7, m.lng*1.0e-7), rotation=math.degrees(m.yaw))
    
        if m.get_type() == "AHRS2" and self.map_settings.showahrs2pos:
            self.create_vehicle_icon('AHRS2' + vehicle, 'blue')
            self.mpstate.map.set_position('AHRS2' + vehicle, (m.lat*1.0e-7, m.lng*1.0e-7), rotation=math.degrees(m.yaw))
    
        if m.get_type() == "GPS_RAW_INT" and self.map_settings.showgpspos:
            (lat, lon) = (m.lat*1.0e-7, m.lon*1.0e-7)
            if lat != 0 or lon != 0:
                self.create_vehicle_icon('GPS' + vehicle, 'blue')
                self.mpstate.map.set_position('GPS' + vehicle, (lat, lon), rotation=m.cog*0.01)
    
        if m.get_type() == "GPS2_RAW" and self.map_settings.showgps2pos:
            (lat, lon) = (m.lat*1.0e-7, m.lon*1.0e-7)
            if lat != 0 or lon != 0:
                self.create_vehicle_icon('GPS2' + vehicle, 'green')
                self.mpstate.map.set_position('GPS2' + vehicle, (lat, lon), rotation=m.cog*0.01)
    
        if m.get_type() == 'GLOBAL_POSITION_INT':
            (self.lat, self.lon, self.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
            if self.lat != 0 or self.lon != 0:
                self.create_vehicle_icon('Pos' + vehicle, 'red', follow=True)
                self.mpstate.map.set_position('Pos' + vehicle, (self.lat, self.lon), rotation=self.heading)
    
        if m.get_type() == "NAV_CONTROLLER_OUTPUT":
            if self.master.flightmode in [ "AUTO", "GUIDED", "LOITER", "RTL" ]:
                trajectory = [ (self.lat, self.lon),
                               mp_util.gps_newpos(self.lat, self.lon, m.target_bearing, m.wp_dist) ]
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('trajectory', trajectory, layer='Trajectory',
                                                              linewidth=2, colour=(255,0,180)))
            else:
                self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Trajectory'))
    
            
        # if the waypoints have changed, redisplay
        if self.wp_change_time != self.module('wp').wploader.last_change:
            self.wp_change_time = self.module('wp').wploader.last_change
            self.display_waypoints()
    
        # if the fence has changed, redisplay
        if self.fence_change_time != self.module('fence').fenceloader.last_change:
            self.fence_change_time = self.module('fence').fenceloader.last_change
            points = self.module('fence').fenceloader.polygon()
            if len(points) > 1:
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('fence', points, layer=1, linewidth=2, colour=(0,255,0)))
    
        # if the rallypoints have changed, redisplay
        if self.rally_change_time != self.module('rally').rallyloader.last_change:
            self.rally_change_time = self.module('rally').rallyloader.last_change
            icon = self.mpstate.map.icon('rallypoint.png')
            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('RallyPoints'))
            for i in range(self.module('rally').rallyloader.rally_count()):
                rp = self.module('rally').rallyloader.rally_point(i)
                self.mpstate.map.add_object(mp_slipmap.SlipIcon('Rally-%u' % i, (rp.lat*1.0e-7, rp.lng*1.0e-7), icon,
                                                           layer='RallyPoints', rotation=0, follow=False))
    
        # check for any events from the map
        self.mpstate.map.check_events()
    
def init(mpstate):
    '''initialise module'''
    return MapModule(mpstate)
