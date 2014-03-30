#!/usr/bin/env python
'''waypoint command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module

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
        self.add_command('wp', self.cmd_wp,       'waypoint management', ["<list|clear>",
                                     "<load|update|save> (FILENAME)"])
        
        if self.continue_mode and self.logdir != None:
            waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded waypoints from %s" % waytxt)
    
    
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
        state = self
        if state.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master.time_since('MISSION_ITEM') >= 2 and state.wploader.count() < getattr(state.wploader,'expected_count',0):
                seq = state.wploader.count()
                print("re-requesting WP %u" % seq)
                self.master.waypoint_request_send(seq)
    
    def process_waypoint_request(self, m, master):
        '''process a waypoint request from the master'''
        state = self
        if (not state.loading_waypoints or
            time.time() > state.loading_waypoint_lasttime + 10.0):
            state.loading_waypoints = False
            self.console.error("not loading waypoints")
            return
        if m.seq >= state.wploader.count():
            self.console.error("Request for bad waypoint %u (max %u)" % (m.seq, state.wploader.count()))
            return
        wp = state.wploader.wp(m.seq)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        self.master.mav.send(state.wploader.wp(m.seq))
        state.loading_waypoint_lasttime = time.time()
        self.console.writeln("Sent waypoint %u : %s" % (m.seq, state.wploader.wp(m.seq)))
        if m.seq == state.wploader.count() - 1:
            state.loading_waypoints = False
            self.console.writeln("Sent all %u waypoints" % state.wploader.count())
    
    def load_waypoints(self, filename):
        '''load waypoints from a file'''
        state = self
        state.wploader.target_system = self.target_system
        state.wploader.target_component = self.target_component
        try:
            state.wploader.load(filename)
        except Exception, msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u waypoints from %s" % (state.wploader.count(), filename))
    
        self.master.waypoint_clear_all_send()
        if state.wploader.count() == 0:
            return
    
        state.loading_waypoints = True
        state.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(state.wploader.count())
    
    def update_waypoints(self, filename, wpnum):
        '''update waypoints from a file'''
        state = self
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            state.wploader.load(filename)
        except Exception, msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        if state.wploader.count() == 0:
            print("No waypoints found in %s" % filename)
            return
        if wpnum == -1:
            print("Loaded %u updated waypoints from %s" % (state.wploader.count(), filename))
        elif wpnum >= state.wploader.count():
            print("Invalid waypoint number %u" % wpnum)
            return
        else:
            print("Loaded updated waypoint %u from %s" % (wpnum, filename))
    
        state.loading_waypoints = True
        state.loading_waypoint_lasttime = time.time()
        if wpnum == -1:
            start = 0
            end = state.wploader.count()-1
        else:
            start = wpnum
            end = wpnum
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                             self.target_component,
                                                             start, end)
    
    def save_waypoints(self, filename):
        '''save waypoints to a file'''
        state = self
        try:
            state.wploader.save(filename)
        except Exception, msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u waypoints to %s" % (state.wploader.count(), filename))
    
    def wp_draw_callback(self, points):
        '''callback from drawing waypoints'''
        state = self
        if len(points) < 3:
            return
        from MAVProxy.modules.lib import mp_util
        home = state.wploader.wp(0)
        state.wploader.clear()
        state.wploader.target_system = self.target_system
        state.wploader.target_component = self.target_component
        state.wploader.add(home)
        for p in points:
            state.wploader.add_latlonalt(p[0], p[1], self.settings.wpalt)
        self.master.waypoint_clear_all_send()
        if state.wploader.count() == 0:
            return
        state.loading_waypoints = True
        state.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(state.wploader.count())
    
    def wp_loop(self):
        '''close the loop on a mission'''
        state = self
        loader = state.wploader
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
        state.loading_waypoints = True
        state.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(state.wploader.count())
        print("Closed loop on mission")
    
    def set_home_location(self):
        '''set home location from last map click'''
        state = self
        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        lat = float(latlon[0])
        lon = float(latlon[1])
        if state.wploader.count() == 0:
            state.wploader.add_latlonalt(lat, lon, 0)
        w = state.wploader.wp(0)
        w.x = lat
        w.y = lon
        state.wploader.set(w, 0)
        state.loading_waypoints = True
        state.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                             self.target_component,
                                                             0, 0)
        
    
    def cmd_wp(self, args):
        '''waypoint commands'''
        state = self
        if len(args) < 1:
            print("usage: wp <list|load|update|save|set|clear|loop>")
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
            state.wp_op = "list"
            self.master.waypoint_request_list_send()
        elif args[0] == "save":
            if len(args) != 2:
                print("usage: wp save <filename>")
                return
            state.wp_save_filename = args[1]
            state.wp_op = "save"
            self.master.waypoint_request_list_send()
        elif args[0] == "savelocal":
            if len(args) != 2:
                print("usage: wp savelocal <filename>")
                return
            state.wploader.save(args[1])
        elif args[0] == "show":
            if len(args) != 2:
                print("usage: wp show <filename>")
                return
            state.wploader.load(args[1])
        elif args[0] == "set":
            if len(args) != 2:
                print("usage: wp set <wpindex>")
                return
            self.master.waypoint_set_current_send(int(args[1]))
        elif args[0] == "clear":
            self.master.waypoint_clear_all_send()
        elif args[0] == "draw":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return        
            if state.wploader.count() == 0:
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
            print("Usage: wp <list|load|save|set|show|clear|draw|loop>")

    def fetch(self):
        """Download wpts from vehicle (this operation is public to support other modules)"""
        if self.wp_op is None:  # If we were already doing a list or save, just restart the fetch without changing the operation
            self.wp_op = "fetch"
        self.master.waypoint_request_list_send()

def init(mpstate):
    '''initialise module'''
    return WPModule(mpstate)
