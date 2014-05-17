#!/usr/bin/env python
'''waypoint command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module
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
        self.add_command('wp', self.cmd_wp,       'waypoint management',
                         ["<list|clear|move|remove|loop|set>",
                          "<load|update|save> (FILENAME)"])

        if self.continue_mode and self.logdir != None:
            waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded waypoints from %s" % waytxt)

        self.menu_added_console = False
        self.menu_added_map = False
        self.menu = MPMenuSubMenu('Mission',
                                  items=[MPMenuItem('Clear', 'Clear', '# wp clear'),
                                         MPMenuItem('List', 'List', '# wp list'),
                                         MPMenuItem('Load', 'Load', '# wp load ',
                                                    handler=MPMenuCallFileDialog(flags=wx.FD_OPEN,
                                                                                 title='Mission Load',
                                                                                 wildcard='*.txt')),
                                         MPMenuItem('Save', 'Save', '# wp save ',
                                                    handler=MPMenuCallFileDialog(flags=wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT,
                                                                                 title='Mission Save',
                                                                                 wildcard='*.txt')),
                                         MPMenuItem('Draw', 'Draw', '# wp draw ',
                                                    handler=MPMenuCallTextDialog(title='Mission Altitude (m)',
                                                                                 default=100)),
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
                self.say("waypoint %u" % m.seq,priority='message')



    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master.time_since('MISSION_ITEM') >= 2 and self.wploader.count() < getattr(self.wploader,'expected_count',0):
                seq = self.wploader.count()
                print("re-requesting WP %u" % seq)
                self.master.waypoint_request_send(seq)
        if not self.menu_added_console and self.module('console') is not None:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)
        if not self.menu_added_map and self.module('map') is not None:
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
        for p in points:
            self.wploader.add_latlonalt(p[0], p[1], self.settings.wpalt)
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
        loader.add(loader.wp(1))
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
        (lat, lon) = latlon
        if getattr(self.console, 'ElevationMap', None) is not None:
            alt1 = self.console.ElevationMap.GetElevation(lat, lon)
            alt2 = self.console.ElevationMap.GetElevation(wp.x, wp.y)
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
        self.wploader.remove(wp)
        self.send_all_waypoints()
        print("Removed WP %u" % idx)

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
        elif args[0] == "remove":
            self.cmd_wp_remove(args[1:])
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
