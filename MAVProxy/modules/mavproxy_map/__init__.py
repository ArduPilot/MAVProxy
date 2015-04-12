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
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_menu import *
from pymavlink import mavutil

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
        self.moving_wp = None
        self.moving_fencepoint = None
        self.moving_rally = None
        self.mission_list = None
        self.icon_counter = 0
        self.click_position = None
        self.click_time = 0
        self.draw_line = None
        self.draw_callback = None
        self.have_global_position = False
        self.vehicle_type_name = 'plane'
        self.drag_mission = False
        self.drag_mission_start = None
        self.drag_mission_draw_time = time.time()
        self.drag_mission_last_drag = time.time()
        self.drag_mission_start_idx = 1
        self.drag_mission_end_idx = 0
        self.drag_lat = 0.0
        self.darg_lon = 0.0
        self.ElevationMap = mp_elevation.ElevationModel()
        self.map_settings = mp_settings.MPSettings(
            [ ('showgpspos', int, 0),
              ('showgps2pos', int, 1),
              ('showsimpos', int, 0),
              ('showahrs2pos', int, 0),
              ('brightness', float, 1),
              ('rallycircle', bool, False),
              ('loitercircle',bool, False)])
        service='OviHybrid'
        if 'MAP_SERVICE' in os.environ:
            service = os.environ['MAP_SERVICE']
        import platform
        mpstate.map = mp_slipmap.MPSlipMap(service=service, elevation=True, title='Map')
        mpstate.map_functions = { 'draw_lines' : self.draw_lines }
    
        mpstate.map.add_callback(functools.partial(self.map_callback))
        self.add_command('map', self.cmd_map, "map control", ['icon',
                                      'set (MAPSETTING)'])
        self.add_completion_function('(MAPSETTING)', self.map_settings.completion)

        self.default_popup = MPMenuSubMenu('Popup', items=[])
        self.add_menu(MPMenuItem('Fly To', 'Fly To', '# guided ',
                                 handler=MPMenuCallTextDialog(title='Altitude (m)', default=100)))
        self.add_menu(MPMenuItem('Set Home', 'Set Home', '# map sethome '))
        self.add_menu(MPMenuItem('Terrain Check', 'Terrain Check', '# terrain check'))
        self.add_menu(MPMenuItem('Show Position', 'Show Position', 'showPosition'))

    def add_menu(self, menu):
        '''add to the default popup menu'''
        self.default_popup.add(menu)
        self.mpstate.map.add_object(mp_slipmap.SlipDefaultPopup(self.default_popup, combine=True))

    def show_position(self):
        '''show map position click information'''
        pos = self.click_position
        dms = (mp_util.degrees_to_dms(pos[0]), mp_util.degrees_to_dms(pos[1]))
        msg =  "Coordinates in WGS84\n"
        msg += "Decimal: %.6f %.6f\n" % (pos[0], pos[1])
        msg += "DMS:     %s %s\n" % (dms[0], dms[1])
        msg += "Grid:    %s\n" % mp_util.latlon_to_grid(pos)
        if self.logdir:
            logf = open(os.path.join(self.logdir, "positions.txt"), "a")
            logf.write("Position: %.6f %.6f at %s\n" % (pos[0], pos[1], time.ctime()))
            logf.close()
        MPMenuChildMessageDialog('Position', msg, font_size=32)

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
        elif args[0] == "sethome":
            self.cmd_set_home(args)
        else:
            print("usage: map <icon|set>")
    
    def display_waypoints(self):
        '''display the waypoints'''
        self.mission_list = self.module('wp').wploader.view_list()
        polygons = self.module('wp').wploader.polygon_list()
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Mission'))

        for i in range(len(polygons)):
            p = polygons[i]
            if len(p) > 1:
                popup = MPMenuSubMenu('Popup',
                                      items=[MPMenuItem('Set', returnkey='popupMissionSet'),
                                             MPMenuItem('WP Remove', returnkey='popupMissionRemove'),
                                             MPMenuItem('WP Move', returnkey='popupMissionMove')])
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('mission %u' % i, p,
                                                                   layer='Mission', linewidth=2, colour=(255,255,255),
                                                                   popup_menu=popup))
        loiter_rad = self.get_mav_param('WP_LOITER_RAD')
        labeled_wps = {}
        for i in range(len(self.mission_list)):
            next_list = self.mission_list[i]
            for j in range(len(next_list)):
                #label already printed for this wp?
                if (next_list[j] not in labeled_wps):
                    self.mpstate.map.add_object(mp_slipmap.SlipLabel(
                        'Label %u' % next_list[j], polygons[i][j], str(next_list[j]), 'Mission', colour=(0,255,255)))

                    if (self.map_settings.loitercircle and 
                        self.module('wp').wploader.wp_is_loiter(next_list[j])):
                        self.mpstate.map.add_object(mp_slipmap.SlipCircle('Loiter Circle %u' % next_list[j], 'LoiterCircles', polygons[i][j], abs(loiter_rad), (255, 255, 255), 2))

                    labeled_wps[next_list[j]] = (i,j)

    def display_drag_mission(self):
        '''display the visualization for dragging a mission'''
        if (self.drag_mission == False):
            #not currently dragging the mission; nothing to draw
            return

        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('DragMission'))
        self.drag_mission_draw_time = self.drag_mission_last_drag
        loit_rad = self.get_mav_param('WP_LOITER_RAD')
        prev_wp = None

        start_idx = self.drag_mission_start_idx
        end_idx = self.module('wp').wploader.count() - 1

        if self.drag_mission_end_idx <= end_idx:
            end_idx = self.drag_mission_end_idx

        for wp in self.module('wp').wploader.wpoints:
           #skip home point, skip mission commands that don't have positions
            if (wp.seq == 0 or 
               (wp.x == 0.0 and wp.y == 0.0 and 
                wp.command is not mavutil.mavlink.MAV_CMD_DO_JUMP)):
                continue

            #skip waypoints the user doesn't want to drag
            if wp.seq < start_idx:
                continue
            if wp.seq > end_idx:
                #done with all waypoints user wants to drag
                break

            offset_lat = wp.x + self.drag_lat
            offset_lon = wp.y + self.drag_lon

            self.mpstate.map.add_object(mp_slipmap.SlipLabel(
                'DragLabel %u' % wp.seq, (offset_lat, offset_lon), 
                str(wp.seq), 'DragMission', colour=(255,0,0)))

            if (self.map_settings.loitercircle and
                self.module('wp').wploader.wp_is_loiter(wp.seq)):
                self.mpstate.map.add_object(mp_slipmap.SlipCircle(
                    'DragPoint %u' % wp.seq, 'DragMission',
                    (offset_lat, offset_lon), abs(loit_rad), (255, 0, 0), 2))
           
            #Cant' just use: 
            #polygons = self.module('wp').wploader.polygon_list()
            #because we might only be moving a subset of the mission
            #and it is not possible to split the list of vertices
            #returned from wploader.polygon_list()
            #based on waypoint index number.  Hence the following
            #to draw lines between waypoints:
            if (prev_wp != None and
                wp.command is not mavutil.mavlink.MAV_CMD_DO_JUMP):
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon(
                    'DragLine %u' % wp.seq, 
                    [(offset_lat, offset_lon),
                    (prev_wp.x + self.drag_lat, prev_wp.y + self.drag_lon)],
                    layer='DragMission', linewidth=2, colour=(255, 0, 0)))
            
            if wp.command is mavutil.mavlink.MAV_CMD_DO_JUMP:
                jump_wp = self.module('wp').wploader.wp(int(wp.param1))
                if jump_wp is not None:
                    if jump_wp.x == 0.0 and jump_wp.y == 0.0:
                        wp = None
                    else:
                        self.mpstate.map.add_object(mp_slipmap.SlipPolygon(
                         'DragLine %u' % wp.seq, 
                         [(prev_wp.x + self.drag_lat,
                         prev_wp.y + self.drag_lon),
                         (jump_wp.x + self.drag_lat,
                         jump_wp.y + self.drag_lon)],
                         layer='DragMission', linewidth=2, colour=(255, 0, 0)))
                        
            prev_wp = wp
            if prev_wp.command is mavutil.mavlink.MAV_CMD_DO_JUMP:
                 prev_wp = None

    def start_drag_mission(self, start_idx, end_idx):
         self.drag_mission_start = None
         self.drag_mission = True
         self.drag_lat = 0.0
         self.drag_lon = 0.0
         self.drag_mission_start_idx = start_idx
         self.drag_mission_end_idx = end_idx
         self.drag_mission_last_drag = time.time()

    def stop_drag_mission(self):
        self.drag_mission = False
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('DragMission'))

    def display_fence(self):
        '''display the fence'''
        self.fence_change_time = self.module('fence').fenceloader.last_change
        points = self.module('fence').fenceloader.polygon()
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Fence'))
        if len(points) > 1:
            popup = MPMenuSubMenu('Popup',
                                  items=[MPMenuItem('FencePoint Remove', returnkey='popupFenceRemove'),
                                         MPMenuItem('FencePoint Move', returnkey='popupFenceMove')])
            self.mpstate.map.add_object(mp_slipmap.SlipPolygon('Fence', points, layer=1,
                                                               linewidth=2, colour=(0,255,0), popup_menu=popup))

            
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

    def remove_rally(self, key):
        '''remove a rally point'''
        a = key.split(' ')
        if a[0] != 'Rally' or len(a) != 2:
            print("Bad rally object %s" % key)
            return
        i = int(a[1])
        self.mpstate.functions.process_stdin('rally remove %u' % i)

    def move_rally(self, key):
        '''move a rally point'''
        a = key.split(' ')
        if a[0] != 'Rally' or len(a) != 2:
            print("Bad rally object %s" % key)
            return
        i = int(a[1])
        self.moving_rally = i

    def selection_index_to_idx(self, key, selection_index):
        '''return a mission idx from a selection_index'''
        a = key.split(' ')
        if a[0] != 'mission' or len(a) != 2:
            print("Bad mission object %s" % key)
            return None
        midx = int(a[1])
        if midx < 0 or midx >= len(self.mission_list):
            print("Bad mission index %s" % key)
            return None
        mlist = self.mission_list[midx]
        if selection_index < 0 or selection_index >= len(mlist):
            print("Bad mission polygon %s" % selection_index)
            return None
        idx = mlist[selection_index]
        return idx

    def move_mission(self, key, selection_index):
        '''move a mission point'''
        idx = self.selection_index_to_idx(key, selection_index)
        self.moving_wp = idx
        print("Moving wp %u" % idx)        

    def remove_mission(self, key, selection_index):
        '''remove a mission point'''
        idx = self.selection_index_to_idx(key, selection_index)
        self.mpstate.functions.process_stdin('wp remove %u' % idx) 

    def remove_fencepoint(self, key, selection_index):
        '''remove a fence point'''
        self.mpstate.functions.process_stdin('fence remove %u' % (selection_index+1)) 

    def move_fencepoint(self, key, selection_index):
        '''move a fence point'''
        self.moving_fencepoint = selection_index
        print("Moving fence point %u" % selection_index) 

    def set_mission(self, key, selection_index):
        '''set a mission point'''
        idx = self.selection_index_to_idx(key, selection_index)
        self.mpstate.functions.process_stdin('wp set %u' % idx) 

    def handle_menu_event(self, obj):
        '''handle a popup menu event from the map'''
        menuitem = obj.menuitem
        if menuitem.returnkey.startswith('# '):
            cmd = menuitem.returnkey[2:]
            if menuitem.handler is not None:
                if menuitem.handler_result is None:
                    return
                cmd += menuitem.handler_result
            self.mpstate.functions.process_stdin(cmd)
        elif menuitem.returnkey == 'popupRallyRemove':
            self.remove_rally(obj.selected[0].objkey)
        elif menuitem.returnkey == 'popupRallyMove':
            self.move_rally(obj.selected[0].objkey)            
        elif menuitem.returnkey == 'popupMissionSet':
            self.set_mission(obj.selected[0].objkey, obj.selected[0].extra_info)
        elif menuitem.returnkey == 'popupMissionRemove':
            self.remove_mission(obj.selected[0].objkey, obj.selected[0].extra_info)
        elif menuitem.returnkey == 'popupMissionMove':
            self.move_mission(obj.selected[0].objkey, obj.selected[0].extra_info)
        elif menuitem.returnkey == 'popupFenceRemove':
            self.remove_fencepoint(obj.selected[0].objkey, obj.selected[0].extra_info)
        elif menuitem.returnkey == 'popupFenceMove':
            self.move_fencepoint(obj.selected[0].objkey, obj.selected[0].extra_info)
        elif menuitem.returnkey == 'showPosition':
            self.show_position()

    def map_callback(self, obj):
        '''called when an event happens on the slipmap'''
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        if isinstance(obj, mp_slipmap.SlipMenuEvent):
            self.handle_menu_event(obj)
            return
        if not isinstance(obj, mp_slipmap.SlipMouseEvent):
            return
        if obj.event.m_leftDown and self.moving_rally is not None:
            self.click_position = obj.latlon
            self.click_time = time.time()
            self.mpstate.functions.process_stdin("rally move %u" % self.moving_rally)
            self.moving_rally = None
            return
        if obj.event.m_rightDown and self.moving_rally is not None:
            print("Cancelled rally move")
            self.moving_rally = None
            return
        if obj.event.m_leftDown and self.moving_wp is not None:
            self.click_position = obj.latlon
            self.click_time = time.time()
            self.mpstate.functions.process_stdin("wp move %u" % self.moving_wp)
            self.moving_wp = None
            return
        if obj.event.m_leftDown and self.moving_fencepoint is not None:
            self.click_position = obj.latlon
            self.click_time = time.time()
            self.mpstate.functions.process_stdin("fence move %u" % (self.moving_fencepoint+1))
            self.moving_fencepoint = None
            return
        if obj.event.m_rightDown and self.moving_wp is not None:
            print("Cancelled wp move")
            self.moving_wp = None
            return
        if obj.event.m_rightDown and self.moving_fencepoint is not None:
            print("Cancelled fence move")
            self.moving_fencepoint = None
            return
        elif obj.event.m_leftDown:
            if time.time() - self.click_time > 0.1:
                self.click_position = obj.latlon
                self.click_time = time.time()
                self.drawing_update()

            if self.module('misseditor') is not None:
                self.module('misseditor').update_map_click_position(self.click_position)

        if obj.event.m_rightDown:
            if self.draw_callback is not None:
                self.drawing_end()
                return                
            if time.time() - self.click_time > 0.1:
                self.click_position = obj.latlon
                self.click_time = time.time()
           
        if obj.event.m_middleDown and self.drag_mission:
            if (self.drag_mission_start == None):
                self.drag_mission_start = obj.latlon

            self.drag_lat = obj.latlon[0] - self.drag_mission_start[0]
            self.drag_lon = obj.latlon[1] - self.drag_mission_start[1]

            #redraw
            self.drag_mission_last_drag = time.time()

        if obj.event.m_leftDown and self.drag_mission:
            #if no change to apply then don't bother sending up waypoints:
            if self.drag_lat == 0.0 and self.drag_lon == 0.0:
                self.stop_drag_mission()
                return

            #apply mouse drag change to mission
            for wp in self.module('wp').wploader.wpoints:
                #skip home point and waypoints with no position information
                if wp.seq == 0 or wp.x == 0.0 or wp.y == 0.0:
                    continue

                #skip waypoints the user doesn't want to drag
                if wp.seq < self.drag_mission_start_idx:
                    continue
                if wp.seq > self.drag_mission_end_idx:
                    #done with all waypoints user wants to drag
                    break

                self.module('wp').cmd_wp_move([str(wp.seq)], wp.x + self.drag_lat, 
                                                             wp.y + self.drag_lon) 
            
            self.module('wp').loading_waypoints = True
            self.module('wp').loading_waypoint_lasttime = time.time()
            self.master.mav.mission_write_partial_list_send(
                    self.module('wp').target_system, self.module('wp').target_component,
                    1,self.module('wp').wploader.count())

            self.stop_drag_mission()

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
        self.mpstate.map.add_object(mp_slipmap.SlipDefaultPopup(self.default_popup, combine=True))
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Drawing'))
    
    def draw_lines(self, callback):
        '''draw a series of connected lines on the map, calling callback when done'''
        self.draw_callback = callback
        self.draw_line = []
        self.mpstate.map.add_object(mp_slipmap.SlipDefaultPopup(None))

    def cmd_set_home(self, args):
        '''called when user selects "Set Home" on map'''
        (lat, lon) = (self.click_position[0], self.click_position[1])
        alt = self.ElevationMap.GetElevation(lat, lon)
        print("Setting home to: ", lat, lon, alt)
        self.master.mav.command_long_send(
            self.settings.target_system, self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, # set position
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            lat, # lat
            lon, # lon
            alt) # param7
        
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "HEARTBEAT":
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
                            mavutil.mavlink.MAV_TYPE_TRICOPTER]:
                self.vehicle_type_name = 'copter'
            elif m.type in [mavutil.mavlink.MAV_TYPE_HELICOPTER]:
                self.vehicle_type_name = 'heli'
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
            if abs(self.lat) > 1.0e-3 or abs(self.lon) > 1.0e-3:
                self.have_global_position = True
                self.create_vehicle_icon('Pos' + vehicle, 'red', follow=True)
                self.mpstate.map.set_position('Pos' + vehicle, (self.lat, self.lon), rotation=self.heading)

        if m.get_type() == 'LOCAL_POSITION_NED' and not self.have_global_position:
            (self.lat, self.lon) = mp_util.gps_offset(0, 0, m.x, m.y)
            self.heading = math.degrees(math.atan2(m.vy, m.vx))
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
        last_wp_change = self.module('wp').wploader.last_change
        if self.wp_change_time != last_wp_change and abs(time.time() - last_wp_change) > 1:
            self.wp_change_time = last_wp_change
            self.display_waypoints()

            #this may have affected the landing lines from the rally points:
            self.rally_change_time = time.time()
    
        # if the fence has changed, redisplay
        if self.fence_change_time != self.module('fence').fenceloader.last_change:
            self.display_fence()

        #if the mission drag visualization needs update, redraw it:
        if self.drag_mission_draw_time != self.drag_mission_last_drag:
            self.display_drag_mission()
    
        # if the rallypoints have changed, redisplay
        if self.rally_change_time != self.module('rally').rallyloader.last_change:
            self.rally_change_time = self.module('rally').rallyloader.last_change
            icon = self.mpstate.map.icon('rallypoint.png')
            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('RallyPoints'))
            for i in range(self.module('rally').rallyloader.rally_count()):
                rp = self.module('rally').rallyloader.rally_point(i)
                popup = MPMenuSubMenu('Popup',
                                      items=[MPMenuItem('Rally Remove', returnkey='popupRallyRemove'),
                                             MPMenuItem('Rally Move', returnkey='popupRallyMove')])
                self.mpstate.map.add_object(mp_slipmap.SlipIcon('Rally %u' % (i+1), (rp.lat*1.0e-7, rp.lng*1.0e-7), icon,
                                                                layer='RallyPoints', rotation=0, follow=False,
                                                                popup_menu=popup))

                loiter_rad = self.get_mav_param('WP_LOITER_RAD')

                if self.map_settings.rallycircle:
                    self.mpstate.map.add_object(mp_slipmap.SlipCircle('Rally Circ %u' % (i+1), 'RallyPoints', (rp.lat*1.0e-7, rp.lng*1.0e-7), abs(loiter_rad), (255,255,0), 2))

                #draw a line between rally point and nearest landing point
                nearest_land_wp = None
                nearest_distance = 10000000.0
                for j in range(self.module('wp').wploader.count()):
                    w = self.module('wp').wploader.wp(j)
                    if (w.command == 21): #if landing waypoint
                        #get distance between rally point and this waypoint
                        dis = mp_util.gps_distance(w.x, w.y, rp.lat*1.0e-7, rp.lng*1.0e-7)
                        if (dis < nearest_distance):
                            nearest_land_wp = w
                            nearest_distance = dis

                if nearest_land_wp != None:
                    points = []
                    #tangential approach?
                    if self.get_mav_param('LAND_BREAK_PATH') == 0:
                        theta = math.degrees(math.atan(loiter_rad / nearest_distance))
                        tan_dis = math.sqrt(nearest_distance * nearest_distance - (loiter_rad * loiter_rad))

                        ral_bearing = mp_util.gps_bearing(nearest_land_wp.x, nearest_land_wp.y,rp.lat*1.0e-7, rp.lng*1.0e-7)

                        points.append(mp_util.gps_newpos(nearest_land_wp.x,nearest_land_wp.y, ral_bearing + theta, tan_dis))

                    else: #not tangential approach
                        points.append((rp.lat*1.0e-7, rp.lng*1.0e-7))

                    points.append((nearest_land_wp.x, nearest_land_wp.y))
                    self.mpstate.map.add_object(mp_slipmap.SlipPolygon('Rally Land %u' % (i+1), points, 'RallyPoints', (255,255,0), 2))

        # check for any events from the map
        self.mpstate.map.check_events()
    
def init(mpstate):
    '''initialise module'''
    return MapModule(mpstate)
