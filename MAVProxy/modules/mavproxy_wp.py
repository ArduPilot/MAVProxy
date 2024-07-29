#!/usr/bin/env python
'''waypoint command handling'''

from MAVProxy.modules.lib import mission_item_protocol
from MAVProxy.modules.lib import mp_util

from pymavlink import mavutil
from pymavlink import mavwp

import time

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem


class WPModule(mission_item_protocol.MissionItemProtocolModule):
    def __init__(self, mpstate):
        super(WPModule, self).__init__(mpstate, "wp", "waypoint handling", public=True)
        # support for setting mission waypoint via command
        self.accepts_DO_SET_MISSION_CURRENT = {}  # keyed by (sysid/compid)

    def gui_menu_items(self):
        ret = super(WPModule, self).gui_menu_items()
        ret.extend([
            MPMenuItem('Editor', 'Editor', '# wp editor'),
            MPMenuItem(
                'Draw', 'Draw', '# wp draw ',
                handler=MPMenuCallTextDialog(
                    title='Mission Altitude (m)',
                    default=100)),
            MPMenuItem('Loop', 'Loop', '# wp loop'),
            MPMenuItem(
                'Add Takeoff', 'Add Takeoff', '# wp add_takeoff ',
                handler=MPMenuCallTextDialog(
                    title='Takeoff Altitude (m)',
                    default=20)),
            MPMenuItem('Add Landing', 'Add Landing', '# wp add_landing'),
            MPMenuItem('Add RTL', 'Add RTL', '# wp add_rtl'),
            MPMenuItem('Add DO_LAND_START', 'Add DO_LAND_START', '# wp add_dls'),
            MPMenuItem('Reset', 'Reset', '# wp set 0'),
        ])
        return ret

    def mission_ftp_name(self):
        return "@MISSION/mission.dat"

    def loader_class(self):
        return mavwp.MAVWPLoader

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_MISSION

    def save_filename_base(self):
        return 'way'

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'waypoints'

    def itemtype(self):
        '''returns description of item'''
        return 'waypoint'

    def index_from_0(self):
        # other similar user-visible interfaces start indexing
        # user-modifiable items from 1.  waypoints make index 0
        # visible to the user.
        return True

    def command_name(self):
        return "wp"

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()

        if mtype in ["MISSION_CURRENT"]:
            if m.seq != self.last_waypoint:
                self.last_waypoint = m.seq
                if self.settings.wpupdates:
                    self.say("waypoint %u" % m.seq, priority='message')

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

        elif mtype == "COMMAND_ACK":
            # check to see if the vehicle has bounced our attempts to
            # set the current mission item via mavlink command (as
            # opposed to the old message):
            if m.command == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
                key = (m.get_srcSystem(), m.get_srcComponent())
                if m.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    # stop sending the commands:
                    self.accepts_DO_SET_MISSION_CURRENT[key] = False
                elif m.result in [mavutil.mavlink.MAV_RESULT_ACCEPTED]:
                    self.accepts_DO_SET_MISSION_CURRENT[key] = True

        super(WPModule, self).mavlink_packet(m)

    def idle_task(self):
        if (self.master is not None and
                'HOME_POSITION' not in self.master.messages and
                time.time() - self.last_get_home > 2):
            self.master.mav.command_long_send(
                self.settings.target_system,
                0,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
            self.last_get_home = time.time()

        super(WPModule, self).idle_task()

    def cmd_moverelhome(self, args, latlon=None):
        '''handle wp move to a point relative to home by dist/bearing'''
        if len(args) < 3:
            print("usage: wp moverelhome WPNUM dist bearing")
            return
        idx = int(args[0])
        if idx < 1 or idx > self.wploader.count():
            print("Invalid wp number %u" % idx)
            return
        dist = float(args[1])
        bearing = float(args[2])

        home = self.get_WP0(home_only=True)
        if home is None:
            print("Need home")
            return

        wp = self.wploader.wp(idx)
        if not self.is_location_wp(wp):
            print("Not a nav command")
            return
        (newlat, newlon) = mp_util.gps_newpos(home.x, home.y, bearing, dist)
        wp.x = newlat
        wp.y = newlon
        wp.target_system    = self.target_system
        wp.target_component = self.target_component
        self.wploader.set(wp, idx)

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        idx, idx+1)
        print("Moved WP %u %.1fm bearing %.1f from home" % (idx, dist, bearing))

    def commands(self):
        ret = super(WPModule, self).commands()

        ret.update({
            'add': self.cmd_add,
            "changealt": self.cmd_changealt,
            "changeframe": self.cmd_changeframe,
            'draw': self.cmd_draw,
            'editor': self.cmd_editor,
            'loop': self.cmd_loop,
            "param": self.cmd_param,
            "movemulti": self.cmd_movemulti,
            "moverelhome": self.cmd_moverelhome,
            'set': self.cmd_set,
            'sethome': self.cmd_sethome,
            'slope': self.cmd_slope,
            'split': self.cmd_split,
            "move": self.cmd_move,  # handled in parent class
            "add_takeoff": self.wp_add_takeoff,
            "add_landing": self.wp_add_landing,
            "add_rtl": self.wp_add_RTL,
            "add_dls": self.wp_add_dls,
            "update": (self.cmd_update, ["(FILENAME)"]),
            "undo": self.cmd_undo,
        })

        return ret

    def mission_type_string(self):
        return 'Mission'

    # waypoint-specific methods:
    def cmd_slope(self, args):
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

    def get_default_frame(self):
        '''default frame for waypoints'''
        if self.settings.terrainalt == 'Auto':
            if self.get_mav_param('TERRAIN_FOLLOW', 0) == 1:
                return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
            return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        if self.settings.terrainalt == 'True':
            return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
        return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT


    def get_WP0(self, home_only=False):
        '''get a location for WP0 when building a mission
        this ideally should be home, but if home is not available then use a click position
        '''
        (lat,lon,alt) = (None,None,None)
        if 'HOME_POSITION' in self.master.messages:
            h = self.master.messages['HOME_POSITION']
            (lat,lon,alt) = (h.latitude*1.0e-7, h.longitude*1.0e-7, h.altitude*1.0e-3)
        elif home_only:
            return None
        elif self.wploader.count() > 0:
            return self.wploader.wp(0)
        else:
            latlon = self.mpstate.click_location
            if latlon is None:
                return None
            (lat,lon,alt) = (latlon[0],latlon[1],0)
        if lat is None or lon is None:
            return None
        w = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         0,
                                                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                         0, 0, 0, 0, 0, 0,
                                                         lat,lon,alt)
        return w

    def get_home(self):
        '''get a location for home'''
        return self.get_WP0(home_only=True)
    
    def wp_draw_callback(self, points):
        '''callback from drawing waypoints'''
        if len(points) < 2:
            return
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        if self.wploader.count() < 2:
            home = self.get_WP0()
            if home is None:
                print("Need home location for draw - please run gethome")
                return
            self.wploader.clear()
            self.wploader.add(home)
        if self.get_default_frame() == mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            use_terrain = True
        else:
            use_terrain = False
        for p in points:
            self.wploader.add_latlonalt(p[0], p[1], self.settings.wpalt, terrain_alt=use_terrain)
        self.send_all_waypoints()

    def cmd_draw(self, args):
        if 'draw_lines' not in self.mpstate.map_functions:
            print("No map drawing available")
            return
        if self.get_WP0() is None:
            print("Need home location - please run gethome")
            return
        if len(args) > 1:
            self.settings.wpalt = int(args[1])
        self.mpstate.map_functions['draw_lines'](self.wp_draw_callback)
        print("Drawing %s on map at altitude %d" %
              (self.itemstype(), self.settings.wpalt))

    def cmd_editor(self, args):
        if self.module('misseditor'):
            self.mpstate.functions.process_stdin("module reload misseditor", immediate=True)
        else:
            self.mpstate.functions.process_stdin("module load misseditor", immediate=True)

    def cmd_set(self, args):
        if len(args) != 1:
            print("usage: wp set <wpindex>")
            return

        wp_num = int(args[0])

        # At time of writing MAVProxy sends to (1, 0) by default,
        # but ArduPilot will respond from (1,1) by default -and
        # that means COMMAND_ACK handling will fill
        # self.accepts_DO_SET_MISSION_CURRENT for (1, 1) and we
        # will not get that value here:
        key = (self.target_system, self.target_component)
        supports = self.accepts_DO_SET_MISSION_CURRENT.get(key, None)
        # if we don't know, send both.  If we do know, send only one.
        # we "know" because we hook receipt of COMMAND_ACK.

        if self.settings.wp_use_waypoint_set_current or supports is False:
            self.master.waypoint_set_current_send(wp_num)
        else:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0,
                wp_num, 0, 0, 0, 0, 0, 0
            )

    def cmd_add(self, args):
        '''add a NAV waypoint at the last map click position'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        if len(args) < 1:
            alt = self.settings.wpalt
        else:
            alt = float(args[0])

        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,    # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    # command
            0,    # current
            0,    # autocontinue
            0.0,  # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            alt,                   # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def cmd_loop(self, args):
        '''close the loop on a mission'''
        loader = self.wploader
        if loader.count() < 2:
            print("Not enough waypoints (%u)" % loader.count())
            return
        wp = loader.wp(loader.count()-2)
        if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
            print("Mission is already looped")
            return
        if (loader.count() > 1 and
                loader.wp(1).command in [mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF]):
            target = 2
        else:
            target = 1
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_DO_JUMP,
                                                          0, 1, target, -1, 0, 0, 0, 0, 0)
        loader.add(wp)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(self.wploader.count())
        print("Closed loop on mission")

    def is_quadplane(self):
        Q_ENABLE = int(self.get_mav_param("Q_ENABLE", 0))
        return Q_ENABLE > 0

    def wp_add_takeoff(self, args):
        '''add a takeoff as first mission item'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No position chosen")
            return
        takeoff_alt = 20
        if len(args) > 0:
            takeoff_alt = float(args[0])
        if self.is_quadplane():
            wptype = mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF
        else:
            wptype = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0,
                                                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                          wptype,
                                                          0, 1, 0, 0, 0, 0, latlon[0], latlon[1], takeoff_alt)
        if self.wploader.count() < 2:
            home = self.get_WP0()
            if home is None:
                print("Need home location - please run gethome")
                return
            self.wploader.clear()
            self.wploader.add(home)
        # assume first waypoint
        self.wploader.insert(1, wp)
        self.send_all_waypoints()

    def wp_add_landing(self, args):
        '''add a landing as last mission item'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No position chosen")
            return
        if self.is_quadplane():
            wptype = mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND
        else:
            wptype = mavutil.mavlink.MAV_CMD_NAV_LAND
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0,
                                                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                          wptype,
                                                          0, 1, 0, 0, 0, 0, latlon[0], latlon[1], 0)
        # assume last waypoint
        self.wploader.add(wp)
        self.send_all_waypoints()

    def wp_add_RTL(self, args):
        '''add a RTL as last mission item'''
        if not self.check_have_list():
            return
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0,
                                                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                          mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                                          0, 1, 0, 0, 0, 0, 0, 0, 0)
        # assume last waypoint
        self.wploader.add(wp)
        self.send_all_waypoints()

    def wp_add_dls(self, args):
        '''add a DO_LAND_START as last mission item'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No position chosen")
            return
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0,
                                                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                          mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                                          0, 1, 0, 0, 0, 0, latlon[0], latlon[1], 0)
        # assume last waypoint
        self.wploader.add(wp)
        self.send_all_waypoints()

    def cmd_sethome(self, args):
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
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            0, 0
        )

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
                if p1 > idx and p1 + delta > 0:
                    wp.param1 = float(p1+delta)
                    self.wploader.set(wp, row)

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
        except IOError:
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
        self.wploader.expected_count += 1
        self.fix_jumps(wp.seq, 1)
        self.send_all_waypoints()


def init(mpstate):
    '''initialise module'''
    return WPModule(mpstate)
