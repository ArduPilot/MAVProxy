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
        self.wp_requested = {}
        self.wp_received = {}
        self.wp_save_filename = None
        self.wploader_by_sysid = {}
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.last_waypoint = 0
        self.wp_period = mavutil.periodic_event(0.5)
        self.undo_wp = None
        self.undo_type = None
        self.undo_wp_idx = -1
        self.wploader.expected_count = 0
        self.add_command('wp', self.cmd_wp,       'waypoint management',
                         ["<list|clear|move|remove|loop|set|undo|movemulti|changealt|param|status|slope>",
                          "<load|update|save|savecsv|show> (FILENAME)"])

        if self.continue_mode and self.logdir is not None:
            waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded waypoints from %s" % waytxt)

        self.menu_added_console = False
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Mission',
                                  items=[MPMenuItem('Editor', 'Editor', '# wp editor'),
                                         MPMenuItem('Clear', 'Clear', '# wp clear'),
                                         MPMenuItem('List', 'List', '# wp list'),
                                         MPMenuItem('Load', 'Load', '# wp load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Mission Load',
                                                                                 wildcard='MissionFiles(*.txt.*.wp,*.waypoints)|*.txt;*.wp;*.waypoints')),
                                         MPMenuItem('Save', 'Save', '# wp save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Mission Save',
                                                                                 wildcard='MissionFiles(*.txt.*.wp,*.waypoints)|*.txt;*.wp;*.waypoints')),
                                         MPMenuItem('Draw', 'Draw', '# wp draw ',
                                                    handler=MPMenuCallTextDialog(title='Mission Altitude (m)',
                                                                                 default=100)),
                                         MPMenuItem('Undo', 'Undo', '# wp undo'),
                                         MPMenuItem('Loop', 'Loop', '# wp loop'),
                                         MPMenuItem('Add NoFly', 'Loop', '# wp noflyadd')])

    @property
    def wploader(self):
        '''per-sysid wploader'''
        if self.target_system not in self.wploader_by_sysid:
            self.wploader_by_sysid[self.target_system] = mavwp.MAVWPLoader()
        return self.wploader_by_sysid[self.target_system]

    def missing_wps_to_request(self):
        ret = []
        tnow = time.time()
        next_seq = self.wploader.count()
        for i in range(5):
            seq = next_seq+i
            if seq+1 > self.wploader.expected_count:
                continue
            if seq in self.wp_requested and tnow - self.wp_requested[seq] < 2:
                continue
            ret.append(seq)
        return ret

    def send_wp_requests(self, wps=None):
        '''send some more WP requests'''
        if wps is None:
            wps = self.missing_wps_to_request()
        tnow = time.time()
        for seq in wps:
            self.wp_requested[seq] = tnow
            if self.settings.wp_use_mission_int:
                self.master.mav.mission_request_int_send(self.master.target_system, self.master.target_component, seq)
            else:
                self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, seq)

    def wp_status(self):
        '''show status of wp download'''
        try:
            print("Have %u of %u waypoints" % (self.wploader.count()+len(self.wp_received), self.wploader.expected_count))
        except Exception:
            print("Have %u waypoints" % (self.wploader.count()+len(self.wp_received)))


    def wp_slope(self, args):
        '''show slope of waypoints'''
        if len(args) == 2:
            # specific waypoints
            wp1 = int(args[0])
            wp2 = int(args[1])
            w1 = self.wploader.wp(wp1)
            w2 = self.wploader.wp(wp2)
            delta_alt = w1.z - w2.z
            if delta_alt == 0:
                slope = "Level"
            else:
                delta_xy = mp_util.gps_distance(w1.x, w1.y, w2.x, w2.y)
                slope = "%.1f" % (delta_xy / delta_alt)
            print("wp%u -> wp%u %s" % (wp1, wp2, slope))
            return
        if len(args) != 0:
            print("Usage: wp slope WP1 WP2")
            return
        last_w = None
        for i in range(1, self.wploader.count()):
            w = self.wploader.wp(i)
            if w.command not in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, mavutil.mavlink.MAV_CMD_NAV_LAND]:
                continue
            if last_w is not None:
                if last_w.frame != w.frame:
                    print("WARNING: frame change %u -> %u at %u" % (last_w.frame, w.frame, i))
                delta_alt = last_w.z - w.z
                if delta_alt == 0:
                    slope = "Level"
                else:
                    delta_xy = mp_util.gps_distance(w.x, w.y, last_w.x, last_w.y)
                    slope = "%.1f" % (delta_xy / delta_alt)
                print("WP%u: slope %s" % (i, slope))
            last_w = w

            
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
            self.wploader.expected_count = m.count
            if self.wp_op is None:
                #self.console.error("No waypoint load started")
                pass
            else:
                self.wploader.clear()
                self.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                                 time.asctime(time.localtime(m._timestamp)),
                                                                                 time.asctime()))
                self.send_wp_requests()

        elif mtype in ['WAYPOINT', 'MISSION_ITEM', 'MISSION_ITEM_INT'] and self.wp_op is not None:
            if m.get_type() == 'MISSION_ITEM_INT':
                if getattr(m, 'mission_type', 0) != 0:
                    # this is not a mission item, likely fence
                    return
                # our internal structure assumes MISSION_ITEM'''
                m = self.wp_from_mission_item_int(m)
            if m.seq < self.wploader.count():
                #print("DUPLICATE %u" % m.seq)
                return
            if m.seq+1 > self.wploader.expected_count:
                self.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, self.wploader.count()))
            self.wp_received[m.seq] = m
            next_seq = self.wploader.count()
            while next_seq in self.wp_received:
                m = self.wp_received.pop(next_seq)
                self.wploader.add(m)
                next_seq += 1
            if self.wploader.count() != self.wploader.expected_count:
                #print("m.seq=%u expected_count=%u" % (m.seq, self.wploader.expected_count))
                self.send_wp_requests()
                return
            if self.wp_op == 'list':
                for i in range(self.wploader.count()):
                    w = self.wploader.wp(i)
                    print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                        w.command, w.frame, w.x, w.y, w.z,
                        w.param1, w.param2, w.param3, w.param4,
                        w.current, w.autocontinue))
                if self.logdir is not None:
                    fname = 'way.txt'
                    if m.get_srcSystem() != 1:
                        fname = 'way_%u.txt' % m.get_srcSystem()
                    waytxt = os.path.join(self.logdir, fname)
                    self.save_waypoints(waytxt)
                    print("Saved waypoints to %s" % waytxt)
                self.loading_waypoints = False
            elif self.wp_op == "save":
                self.save_waypoints(self.wp_save_filename)
            self.wp_op = None
            self.wp_requested = {}
            self.wp_received = {}

        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
            self.process_waypoint_request(m, self.master)

        elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
            if m.seq != self.last_waypoint:
                self.last_waypoint = m.seq
                if self.settings.wpupdates:
                    self.say("waypoint %u" % m.seq,priority='message')

        elif mtype == "MISSION_ITEM_REACHED":
            wp = self.wploader.wp(m.seq)
            if wp is None:
                # should we spit out a warning?!
                # self.say("No waypoints")
                pass
            else:
                if wp.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    alt_offset = self.get_mav_param('ALT_OFFSET', 0)
                    if alt_offset > 0.005:
                        self.say("ALT OFFSET IS NOT ZERO passing DO_LAND_START")

    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master is not None and self.master.time_since('MISSION_ITEM') >= 2 and self.wploader.count() < getattr(self.wploader,'expected_count',0):
                wps = self.missing_wps_to_request();
                print("re-requesting WPs %s" % str(wps))
                self.send_wp_requests(wps)
        if self.module('console') is not None and not self.menu_added_console:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)
        if self.module('map') is not None and not self.menu_added_map:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)

    def wp_to_mission_item_int(self, wp):
        '''convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as MISSION_ITEM_INT
           to give cm level accuracy'''
        if wp.get_type() == 'MISSION_ITEM_INT':
            return wp
        wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(wp.target_system,
                                                                  wp.target_component,
                                                                  wp.seq,
                                                                  wp.frame,
                                                                  wp.command,
                                                                  wp.current,
                                                                  wp.autocontinue,
                                                                  wp.param1,
                                                                  wp.param2,
                                                                  wp.param3,
                                                                  wp.param4,
                                                                  int(wp.x*1.0e7),
                                                                  int(wp.y*1.0e7),
                                                                  wp.z)
        return wp_int

    def wp_from_mission_item_int(self, wp):
        '''convert a MISSION_ITEM_INT to a MISSION_ITEM'''
        wp2 = mavutil.mavlink.MAVLink_mission_item_message(wp.target_system,
                                                           wp.target_component,
                                                           wp.seq,
                                                           wp.frame,
                                                           wp.command,
                                                           wp.current,
                                                           wp.autocontinue,
                                                           wp.param1,
                                                           wp.param2,
                                                           wp.param3,
                                                           wp.param4,
                                                           wp.x*1.0e-7,
                                                           wp.y*1.0e-7,
                                                           wp.z)
        # preserve srcSystem as that is used for naming waypoint file
        wp2._header.srcSystem = wp.get_srcSystem()
        wp2._header.srcComponent = wp.get_srcComponent()
        return wp2

    def process_waypoint_request(self, m, master):
        '''process a waypoint request from the master'''
        if (m.target_system != self.settings.source_system or
            m.target_component != self.settings.source_component):
            # self.console.error("Mission request is not for me")
            return
        if (not self.loading_waypoints or
            time.time() > self.loading_waypoint_lasttime + 10.0):
            self.loading_waypoints = False
            #self.console.error("not loading waypoints")
            return
        if m.seq >= self.wploader.count():
            self.console.error("Request for bad waypoint %u (max %u)" % (m.seq, self.wploader.count()))
            return
        wp = self.wploader.wp(m.seq)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        if self.settings.wp_use_mission_int:
            wp_send = self.wp_to_mission_item_int(wp)
        else:
            wp_send = wp
        self.master.mav.send(wp_send)
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
            #need to remove the leading and trailing quotes in filename
            self.wploader.load(filename.strip('"'))
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
            #need to remove the leading and trailing quotes in filename
            self.wploader.save(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u waypoints to %s" % (self.wploader.count(), filename))

    def save_waypoints_csv(self, filename):
        '''save waypoints to a file in a human readable CSV file'''
        try:
            #need to remove the leading and trailing quotes in filename
            self.wploader.savecsv(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u waypoints to CSV %s" % (self.wploader.count(), filename))

    def get_default_frame(self):
        '''default frame for waypoints'''
        if self.settings.terrainalt == 'Auto':
            if self.get_mav_param('TERRAIN_FOLLOW',0) == 1:
                return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
            return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        if self.settings.terrainalt == 'True':
            return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
        return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def get_home(self):
        '''get home location'''
        if 'HOME_POSITION' in self.master.messages:
            h = self.master.messages['HOME_POSITION']
            return mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                                self.target_component,
                                                                0,
                                                                0,
                                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                0, 0, 0, 0, 0, 0,
                                                                h.latitude*1.0e-7, h.longitude*1.0e-7, h.altitude*1.0e-3)
        if self.wploader.count() > 0:
            return self.wploader.wp(0)
        return None
        

    def wp_draw_callback(self, points):
        '''callback from drawing waypoints'''
        if len(points) < 3:
            return
        from MAVProxy.modules.lib import mp_util
        home = self.get_home()
        if home is None:
            print("Need home location for draw - please run gethome")
            return
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

    def nofly_add(self):
        '''add a square flight exclusion zone'''
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No position chosen")
            return
        loader = self.wploader
        (center_lat, center_lon) = latlon
        points = []
        points.append(mp_util.gps_offset(center_lat, center_lon, -25,  25))
        points.append(mp_util.gps_offset(center_lat, center_lon,  25,  25))
        points.append(mp_util.gps_offset(center_lat, center_lon,  25, -25))
        points.append(mp_util.gps_offset(center_lat, center_lon, -25, -25))
        start_idx = loader.count()
        for p in points:
            wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                                                              0, 1, 4, 0, 0, 0, p[0], p[1], 0)
            loader.add(wp)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        start_idx, start_idx+4)
        print("Added nofly zone")
        
    def set_home_location(self):
        '''set home location from last map click'''
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No position available")
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
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return
        wp = self.wploader.wp(idx)

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "move"

        (lat, lon) = latlon
        if (getattr(self.console, 'ElevationMap', None) is not None and
            wp.frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT and
            self.settings.wpterrainadjust):
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


    def cmd_wp_movemulti(self, args, latlon=None):
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

        if latlon is None:
            latlon = self.mpstate.click_location
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

            if (getattr(self.console, 'ElevationMap', None) is not None and
                wp.frame != mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT and
                self.settings.wpterrainadjust):
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


    def cmd_wp_changealt(self, args):
        '''handle wp change target alt of multiple waypoints'''
        if len(args) < 2:
            print("usage: wp changealt WPNUM NEWALT <NUMWP>")
            return
        idx = int(args[0])
        if idx < 1 or idx > self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        newalt = float(args[1])
        if len(args) >= 3:
            count = int(args[2])
        else:
            count = 1

        for wpnum in range(idx, idx+count):
            wp = self.wploader.wp(wpnum)
            if not self.wploader.is_location_command(wp.command):
                continue
            wp.z = newalt
            wp.target_system    = self.target_system
            wp.target_component = self.target_component
            self.wploader.set(wp, wpnum)

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        idx, idx+count)
        print("Changed alt for WPs %u:%u to %f" % (idx, idx+(count-1), newalt))

    def fix_jumps(self, idx, delta):
        '''fix up jumps when we add/remove rows'''
        numrows = self.wploader.count()
        for row in range(numrows):
            wp = self.wploader.wp(row)
            jump_cmds = [mavutil.mavlink.MAV_CMD_DO_JUMP]
            if hasattr(mavutil.mavlink, "MAV_CMD_DO_CONDITION_JUMP"):
                jump_cmds.append(mavutil.mavlink.MAV_CMD_DO_CONDITION_JUMP)
            if wp.command in jump_cmds:
                p1 = int(wp.param1)
                if p1 > idx and p1+delta>0:
                    wp.param1 = float(p1+delta)
                    self.wploader.set(wp, row)

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
        self.fix_jumps(idx, -1)
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
            self.fix_jumps(self.undo_wp_idx, 1)
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

    def get_loc(self, m):
        '''return a mavutil.location for item m'''
        t = m.get_type()
        if t == "MISSION_ITEM":
            lat = m.x * 1e7
            lng = m.y * 1e7
            alt = m.z * 1e2
        elif t == "MISSION_ITEM_INT":
            lat = m.x
            lng = m.y
            alt = m.z
        else:
            return None
        return mavutil.location(lat, lng, alt)

    def cmd_split(self, args):
        '''splits the segment ended by the supplied waypoint into two'''
        try:
            num = int(args[0])
        except IOError as e:
            return "Bad wp num (%s)" % args[0]

        if num < 1 or num > self.wploader.count():
            print("Bad item %s" % str(num))
            return
        wp = self.wploader.wp(num)
        if wp is None:
            print("Could not get wp %u" % num)
            return
        loc = self.get_loc(wp)
        if loc is None:
            print("wp is not a location command")
            return

        prev = num - 1
        if prev < 1 or prev > self.wploader.count():
            print("Bad item %u" % num)
            return
        prev_wp = self.wploader.wp(prev)
        if prev_wp is None:
            print("Could not get previous wp %u" % prev)
            return
        prev_loc = self.get_loc(prev_wp)
        if prev_loc is None:
            print("previous wp is not a location command")
            return

        if wp.frame != prev_wp.frame:
            print("waypoints differ in frame (%u vs %u)" %
                  (wp.frame, prev_wp.frame))
            return

        if wp.frame != prev_wp.frame:
            print("waypoints differ in frame")
            return

        lat_avg = (loc.lat + prev_loc.lat)/2
        lng_avg = (loc.lng + prev_loc.lng)/2
        alt_avg = (loc.alt + prev_loc.alt)/2
        new_wp = mavutil.mavlink.MAVLink_mission_item_message(
            self.target_system,
            self.target_component,
            wp.seq,    # seq
            wp.frame,    # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    # command
            0,    # current
            0,    # autocontinue
            0.0,  # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            lat_avg * 1e-7,  # x (latitude)
            lng_avg * 1e-7,  # y (longitude)
            alt_avg * 1e-2,  # z (altitude)
        )
        self.wploader.insert(wp.seq, new_wp)
        self.fix_jumps(wp.seq, 1)
        self.send_all_waypoints()

    def cmd_clear(self, args):
        '''clear waypoints'''
        clear_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        if len(args) > 0 and args[0] == "all":
            clear_type = mavutil.mavlink.MAV_MISSION_TYPE_ALL
        self.master.mav.mission_clear_all_send(self.target_system, self.target_component, clear_type)
        self.wploader.clear()
        
    def cmd_wp(self, args):
        '''waypoint commands'''
        usage = "usage: wp <editor|list|load|update|save|set|clear|loop|remove|move|movemulti|changealt>"
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
        elif args[0] == "savecsv":
            if len(args) != 2:
                print("usage: wp savecsv <filename.csv>")
                return
            self.savecsv(args[1])
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
            self.cmd_wp_movemulti(args[1:], None)
        elif args[0] == "changealt":
            self.cmd_wp_changealt(args[1:])
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
        elif args[0] == "split":
            self.cmd_split(args[1:])
        elif args[0] == "clear":
            self.cmd_clear(args[1:])
        elif args[0] == "editor":
            if self.module('misseditor'):
                self.mpstate.functions.process_stdin("module reload misseditor", immediate=True)
            else:
                self.mpstate.functions.process_stdin("module load misseditor", immediate=True)
        elif args[0] == "draw":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return
            if self.get_home() is None:
                print("Need home location - please run gethome")
                return
            if len(args) > 1:
                self.settings.wpalt = int(args[1])
            self.mpstate.map_functions['draw_lines'](self.wp_draw_callback)
            print("Drawing waypoints on map at altitude %d" % self.settings.wpalt)
        elif args[0] == "sethome":
            self.set_home_location()
        elif args[0] == "loop":
            self.wp_loop()
        elif args[0] == "noflyadd":
            self.nofly_add()
        elif args[0] == "status":
            self.wp_status()
        elif args[0] == "slope":
            self.wp_slope(args[1:])
        else:
            print(usage)

    def pretty_enum_value(self, enum_name, enum_value):
        if enum_name == "MAV_FRAME":
            if enum_value == 0:
                return "Abs"
            elif enum_value == 1:
                return "Local"
            elif enum_value == 2:
                return "Mission"
            elif enum_value == 3:
                return "Rel"
            elif enum_value == 4:
                return "Local ENU"
            elif enum_value == 5:
                return "Global (INT)"
            elif enum_value == 10:
                return "AGL"
        ret = mavutil.mavlink.enums[enum_name][enum_value].name
        ret = ret[len(enum_name)+1:]
        return ret

    def csv_line(self, line):
        '''turn a list of values into a CSV line'''
        self.csv_sep = ","
        return self.csv_sep.join(['"' + str(x) + '"' for x in line])

    def pretty_parameter_value(self, value):
        '''pretty parameter value'''
        return value

    def savecsv(self, filename):
        '''save waypoints to a file in human-readable CSV file'''
        f = open(filename, mode='w')
        headers = ["Seq", "Frame", "Cmd", "P1", "P2", "P3", "P4", "X", "Y", "Z"]
        print(self.csv_line(headers))
        f.write(self.csv_line(headers) + "\n")
        for w in self.wploader.wpoints:
            if getattr(w, 'comment', None):
#                f.write("# %s\n" % w.comment)
                pass
            out_list = [ w.seq,
                         self.pretty_enum_value('MAV_FRAME', w.frame),
                         self.pretty_enum_value('MAV_CMD', w.command),
                         self.pretty_parameter_value(w.param1),
                         self.pretty_parameter_value(w.param2),
                         self.pretty_parameter_value(w.param3),
                         self.pretty_parameter_value(w.param4),
                         self.pretty_parameter_value(w.x),
                         self.pretty_parameter_value(w.y),
                         self.pretty_parameter_value(w.z),
                         ]
            print(self.csv_line(out_list))
            f.write(self.csv_line(out_list) + "\n")
        f.close()

    def fetch(self):
        """Download wpts from vehicle (this operation is public to support other modules)"""
        if self.wp_op is None:  # If we were already doing a list or save, just restart the fetch without changing the operation
            self.wp_op = "fetch"
        self.master.waypoint_request_list_send()

def init(mpstate):
    '''initialise module'''
    return WPModule(mpstate)
