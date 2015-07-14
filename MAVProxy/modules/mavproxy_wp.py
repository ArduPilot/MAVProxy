#!/usr/bin/env python
'''waypoint command handling'''

import time, os, fnmatch, copy, platform
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class WPModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(WPModule, self).__init__(mpstate, "wp", "waypoint handling", public = True)
        self.wp_op = None
        self.wp_save_filename = None
        self.wploader = mavwp.MAVWPLoader()
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.last_waypoint = 0
        self.wp_period = mavutil.periodic_event(0.5)
        self.undo_wp = None
        self.undo_type = None
        self.undo_wp_idx = -1
        self.add_command('wp', self.cmd_wp,       'waypoint management',
                         ["<list|clear|move|remove|loop|set|undo|movemulti|param>",
                          "<load|update|save|show> (FILENAME)"])

        if self.continue_mode and self.logdir != None:
            waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded waypoints from %s" % waytxt)

        self.menu_added_console = False
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Mission',
                                  items=[MPMenuItem('Clear', 'Clear', '# wp clear'),
                                         MPMenuItem('List', 'List', '# wp list'),
                                         MPMenuItem('Load', 'Load', '# wp load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Mission Load',
                                                                                 wildcard='*.txt')),
                                         MPMenuItem('Save', 'Save', '# wp save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Mission Save',
                                                                                 wildcard='*.txt')),
                                         MPMenuItem('Draw', 'Draw', '# wp draw ',
                                                    handler=MPMenuCallTextDialog(title='Mission Altitude (m)',
                                                                                 default=100)),
                                         MPMenuItem('Undo', 'Undo', '# wp undo'),
                                         MPMenuItem('Loop', 'Loop', '# wp loop')])


    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
            if self.wp_op is None:
                self.console.error("No waypoint load started")
            else:
                self.wploader.clear()
                self.wploader.expected_count = m.count
                self.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                                 time.asctime(time.localtime(m._timestamp)),
                                                                                 time.asctime()))
                self.master.waypoint_request_send(0)

        elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and self.wp_op != None:
            if m.seq > self.wploader.count():
                self.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, self.wploader.count()))
            elif m.seq < self.wploader.count():
                # a duplicate
                pass
            else:
                self.wploader.add(m)
            if m.seq+1 < self.wploader.expected_count:
                self.master.waypoint_request_send(m.seq+1)
            else:
                if self.wp_op == 'list':
                    for i in range(self.wploader.count()):
                        w = self.wploader.wp(i)
                        print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                            w.command, w.frame, w.x, w.y, w.z,
                            w.param1, w.param2, w.param3, w.param4,
                            w.current, w.autocontinue))
                    if self.logdir != None:
                        waytxt = os.path.join(self.logdir, 'way.txt')
                        self.save_waypoints(waytxt)
                        print("Saved waypoints to %s" % waytxt)
                elif self.wp_op == "save":
                    self.save_waypoints(self.wp_save_filename)
                self.wp_op = None

        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
            self.process_waypoint_request(m, self.master)

        elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
            if m.seq != self.last_waypoint:
                self.last_waypoint = m.seq
                if self.settings.wpupdates:
                    self.say("waypoint %u" % m.seq,priority='message')


    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master is not None and self.master.time_since('MISSION_ITEM') >= 2 and self.wploader.count() < getattr(self.wploader,'expected_count',0):
                seq = self.wploader.count()
                print("re-requesting WP %u" % seq)
                self.master.waypoint_request_send(seq)
        if self.module('console') is not None and not self.menu_added_console:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)
        if self.module('map') is not None and not self.menu_added_map:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)

    def process_waypoint_request(self, m, master):
        '''process a waypoint request from the master'''
        if (not self.loading_waypoints or
            time.time() > self.loading_waypoint_lasttime + 10.0):
            self.loading_waypoints = False
            self.console.error("not loading waypoints")
            return
        if m.seq >= self.wploader.count():
            self.console.error("Request for bad waypoint %u (max %u)" % (m.seq, self.wploader.count()))
            return
        wp = self.wploader.wp(m.seq)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        self.master.mav.send(self.wploader.wp(m.seq))
        self.loading_waypoint_lasttime = time.time()
        self.console.writeln("Sent waypoint %u : %s" % (m.seq, self.wploader.wp(m.seq)))
        if m.seq == self.wploader.count() - 1:
            self.loading_waypoints = False
            self.console.writeln("Sent all %u waypoints" % self.wploader.count())

    def send_all_waypoints(self):
        '''send all waypoints to vehicle'''
        self.master.waypoint_clear_all_send()
        if self.wploader.count() == 0:
            return
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(self.wploader.count())

    def load_waypoints(self, filename):
        '''load waypoints from a file'''
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            self.wploader.load(filename)
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u waypoints from %s" % (self.wploader.count(), filename))
        self.send_all_waypoints()

    def update_waypoints(self, filename, wpnum):
        '''update waypoints from a file'''
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            self.wploader.load(filename)
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        if self.wploader.count() == 0:
            print("No waypoints found in %s" % filename)
            return
        if wpnum == -1:
            print("Loaded %u updated waypoints from %s" % (self.wploader.count(), filename))
        elif wpnum >= self.wploader.count():
            print("Invalid waypoint number %u" % wpnum)
            return
        else:
            print("Loaded updated waypoint %u from %s" % (wpnum, filename))

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        if wpnum == -1:
            start = 0
            end = self.wploader.count()-1
        else:
            start = wpnum
            end = wpnum
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                             self.target_component,
                                                             start, end)

    def save_waypoints(self, filename):
        '''save waypoints to a file'''
        try:
            self.wploader.save(filename)
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u waypoints to %s" % (self.wploader.count(), filename))

    def get_default_frame(self):
        '''default frame for waypoints'''
        if self.settings.terrainalt == 'Auto':
            if self.get_mav_param('TERRAIN_FOLLOW',0) == 1:
                return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
            return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        if self.settings.terrainalt == 'True':
            return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
        return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def wp_draw_callback(self, points):
        '''callback from drawing waypoints'''
        if len(points) < 3:
            return
        from MAVProxy.modules.lib import mp_util
        home = self.wploader.wp(0)
        self.wploader.clear()
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        self.wploader.add(home)
        if self.get_default_frame() == mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            use_terrain = True
        else:
            use_terrain = False
        for p in points:
            self.wploader.add_latlonalt(p[0], p[1], self.settings.wpalt, terrain_alt=use_terrain)
        self.send_all_waypoints()

    def wp_loop(self):
        '''close the loop on a mission'''
        loader = self.wploader
        if loader.count() < 2:
            print("Not enough waypoints (%u)" % loader.count())
            return
        wp = loader.wp(loader.count()-2)
        if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
            print("Mission is already looped")
            return
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_DO_JUMP,
                                                          0, 1, 1, -1, 0, 0, 0, 0, 0)
        loader.add(wp)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(self.wploader.count())
        print("Closed loop on mission")

    def set_home_location(self):
        '''set home location from last map click'''
        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        lat = float(latlon[0])
        lon = float(latlon[1])
        if self.wploader.count() == 0:
            self.wploader.add_latlonalt(lat, lon, 0)
        w = self.wploader.wp(0)
        w.x = lat
        w.y = lon
        self.wploader.set(w, 0)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                             self.target_component,
                                                             0, 0)


    def cmd_wp_move(self, args):
        '''handle wp move'''
        if len(args) != 1:
            print("usage: wp move WPNUM")
            return
        idx = int(args[0])
        if idx < 1 or idx > self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return
        wp = self.wploader.wp(idx)

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "move"

        (lat, lon) = latlon
        if getattr(self.console, 'ElevationMap', None) is not None and wp.frame != mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            alt1 = self.console.ElevationMap.GetElevation(lat, lon)
            alt2 = self.console.ElevationMap.GetElevation(wp.x, wp.y)
            if alt1 is not None and alt2 is not None:
                wp.z += alt1 - alt2
        wp.x = lat
        wp.y = lon

        wp.target_system    = self.target_system
        wp.target_component = self.target_component
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        idx, idx)
        self.wploader.set(wp, idx)
        print("Moved WP %u to %f, %f at %.1fm" % (idx, lat, lon, wp.z))


    def cmd_wp_movemulti(self, args):
        '''handle wp move of multiple waypoints'''
        if len(args) < 3:
            print("usage: wp movemulti WPNUM WPSTART WPEND <rotation>")
            return
        idx = int(args[0])
        if idx < 1 or idx > self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        wpstart = int(args[1])
        if wpstart < 1 or wpstart > self.wploader.count():
            print("Invalid wp number %u" % wpstart)
            return
        wpend = int(args[2])
        if wpend < 1 or wpend > self.wploader.count():
            print("Invalid wp number %u" % wpend)
            return
        if idx < wpstart or idx > wpend:
            print("WPNUM must be between WPSTART and WPEND")
            return

        # optional rotation about center point
        if len(args) > 3:
            rotation = float(args[3])
        else:
            rotation = 0

        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return
        wp = self.wploader.wp(idx)
        if not self.wploader.is_location_command(wp.command):
            print("WP must be a location command")
            return

        (lat, lon) = latlon
        distance = mp_util.gps_distance(wp.x, wp.y, lat, lon)
        bearing  = mp_util.gps_bearing(wp.x, wp.y, lat, lon)

        for wpnum in range(wpstart, wpend+1):
            wp = self.wploader.wp(wpnum)
            if not self.wploader.is_location_command(wp.command):
                continue
            (newlat, newlon) = mp_util.gps_newpos(wp.x, wp.y, bearing, distance)
            if wpnum != idx and rotation != 0:
                # add in rotation
                d2 = mp_util.gps_distance(lat, lon, newlat, newlon)
                b2 = mp_util.gps_bearing(lat, lon, newlat, newlon)
                (newlat, newlon) = mp_util.gps_newpos(lat, lon, b2+rotation, d2)
                
            if getattr(self.console, 'ElevationMap', None) is not None and wp.frame != mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
                alt1 = self.console.ElevationMap.GetElevation(newlat, newlon)
                alt2 = self.console.ElevationMap.GetElevation(wp.x, wp.y)
                if alt1 is not None and alt2 is not None:
                    wp.z += alt1 - alt2
            wp.x = newlat
            wp.y = newlon
            wp.target_system    = self.target_system
            wp.target_component = self.target_component
            self.wploader.set(wp, wpnum)
            
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        wpstart, wpend+1)
        print("Moved WPs %u:%u to %f, %f rotation=%.1f" % (wpstart, wpend, lat, lon, rotation))

    def cmd_wp_remove(self, args):
        '''handle wp remove'''
        if len(args) != 1:
            print("usage: wp remove WPNUM")
            return
        idx = int(args[0])
        if idx < 0 or idx >= self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        wp = self.wploader.wp(idx)

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "remove"

        self.wploader.remove(wp)
        self.send_all_waypoints()
        print("Removed WP %u" % idx)

    def cmd_wp_undo(self):
        '''handle wp undo'''
        if self.undo_wp_idx == -1 or self.undo_wp is None:
            print("No undo information")
            return
        wp = self.undo_wp
        if self.undo_type == 'move':
            wp.target_system    = self.target_system
            wp.target_component = self.target_component
            self.loading_waypoints = True
            self.loading_waypoint_lasttime = time.time()
            self.master.mav.mission_write_partial_list_send(self.target_system,
                                                            self.target_component,
                                                            self.undo_wp_idx, self.undo_wp_idx)
            self.wploader.set(wp, self.undo_wp_idx)
            print("Undid WP move")
        elif self.undo_type == 'remove':
            self.wploader.insert(self.undo_wp_idx, wp)
            self.send_all_waypoints()
            print("Undid WP remove")
        else:
            print("bad undo type")
        self.undo_wp = None
        self.undo_wp_idx = -1

    def cmd_wp_param(self, args):
        '''handle wp parameter change'''
        if len(args) < 2:
            print("usage: wp param WPNUM PNUM <VALUE>")
            return
        idx = int(args[0])
        if idx < 1 or idx > self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        wp = self.wploader.wp(idx)
        param = [wp.param1, wp.param2, wp.param3, wp.param4]
        pnum = int(args[1])
        if pnum < 1 or pnum > 4:
            print("Invalid param number %u" % pnum)
            return

        if len(args) == 2:
            print("Param %u: %f" % (pnum, param[pnum-1]))
            return

        param[pnum-1] = float(args[2])
        wp.param1 = param[0]
        wp.param2 = param[1]
        wp.param3 = param[2]
        wp.param4 = param[3]
        
        wp.target_system    = self.target_system
        wp.target_component = self.target_component
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        idx, idx)
        self.wploader.set(wp, idx)
        print("Set param %u for %u to %f" % (pnum, idx, param[pnum-1]))

    def cmd_wp(self, args):
        '''waypoint commands'''
        usage = "usage: wp <list|load|update|save|set|clear|loop|remove|move>"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == "load":
            if len(args) != 2:
                print("usage: wp load <filename>")
                return
            self.load_waypoints(args[1])
        elif args[0] == "update":
            if len(args) < 2:
                print("usage: wp update <filename> <wpnum>")
                return
            if len(args) == 3:
                wpnum = int(args[2])
            else:
                wpnum = -1
            self.update_waypoints(args[1], wpnum)
        elif args[0] == "list":
            self.wp_op = "list"
            self.master.waypoint_request_list_send()
        elif args[0] == "save":
            if len(args) != 2:
                print("usage: wp save <filename>")
                return
            self.wp_save_filename = args[1]
            self.wp_op = "save"
            self.master.waypoint_request_list_send()
        elif args[0] == "savelocal":
            if len(args) != 2:
                print("usage: wp savelocal <filename>")
                return
            self.wploader.save(args[1])
        elif args[0] == "show":
            if len(args) != 2:
                print("usage: wp show <filename>")
                return
            self.wploader.load(args[1])
        elif args[0] == "move":
            self.cmd_wp_move(args[1:])
        elif args[0] == "movemulti":
            self.cmd_wp_movemulti(args[1:])
        elif args[0] == "param":
            self.cmd_wp_param(args[1:])
        elif args[0] == "remove":
            self.cmd_wp_remove(args[1:])
        elif args[0] == "undo":
            self.cmd_wp_undo()
        elif args[0] == "set":
            if len(args) != 2:
                print("usage: wp set <wpindex>")
                return
            self.master.waypoint_set_current_send(int(args[1]))
        elif args[0] == "clear":
            self.master.waypoint_clear_all_send()
            self.wploader.clear()
        elif args[0] == "draw":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return
            if self.wploader.count() == 0:
                print("Need home location - refresh waypoints")
                return
            if len(args) > 1:
                self.settings.wpalt = int(args[1])
            self.mpstate.map_functions['draw_lines'](self.wp_draw_callback)
            print("Drawing waypoints on map at altitude %d" % self.settings.wpalt)
        elif args[0] == "sethome":
            self.set_home_location()
        elif args[0] == "loop":
            self.wp_loop()
        else:
            print(usage)

    def fetch(self):
        """Download wpts from vehicle (this operation is public to support other modules)"""
        if self.wp_op is None:  # If we were already doing a list or save, just restart the fetch without changing the operation
            self.wp_op = "fetch"
        self.master.waypoint_request_list_send()

def init(mpstate):
    '''initialise module'''
    return WPModule(mpstate)
