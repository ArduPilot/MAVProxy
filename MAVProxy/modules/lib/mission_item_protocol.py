#!/usr/bin/env python

'''
base class for modules generally transfering items using the MISSION_ITEM protocol

AP_FLAKE8_CLEAN
'''

import copy
import os
import re
import struct
import time

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallFileDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu

try:
    # py2
    from StringIO import StringIO as SIO
except ImportError:
    # py3
    from io import BytesIO as SIO


class MissionItemProtocolModule(mp_module.MPModule):
    def __init__(self, mpstate, name, description, **args):
        super(MissionItemProtocolModule, self).__init__(mpstate, name, description, **args)
        self.add_command(self.command_name(),
                         self.cmd_wp,
                         '%s management' % self.itemtype(),
                         self.completions())
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
        self.upload_start = None
        self.last_get_home = time.time()
        self.ftp_count = None

        if self.continue_mode and self.logdir is not None:
            waytxt = os.path.join(mpstate.status.logdir, self.save_filename())
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded %s from %s" % (self.itemstype(), waytxt))

        self.init_gui_menus()

    def gui_menu_items(self):
        return [
            MPMenuItem('FTP', 'FTP', '# %s ftp' % self.command_name()),
            MPMenuItem('Clear', 'Clear', '# %s clear' % self.command_name()),
            MPMenuItem('List', 'List', '# %s list' % self.command_name()),
            MPMenuItem(
                'Load', 'Load', '# %s load ' % self.command_name(),
                handler=MPMenuCallFileDialog(
                    flags=('open',),
                    title='%s Load' % self.mission_type_string(),
                    wildcard='MissionFiles(*.txt.*.wp,*.waypoints)|*.txt;*.wp;*.waypoints')),
            MPMenuItem(
                'Save', 'Save', '# %s save ' % self.command_name(),
                handler=MPMenuCallFileDialog(
                    flags=('save', 'overwrite_prompt'),
                    title='%s Save' % self.mission_type_string(),
                    wildcard='MissionFiles(*.txt.*.wp,*.waypoints)|*.txt;*.wp;*.waypoints')),
            MPMenuItem('Undo', 'Undo', '# %s undo' % self.command_name()),
        ]

    def mission_type_string(self):
        loader_class_string = str(self.loader_class())
        m = re.search("'([^']*)'", loader_class_string)
        if m is None:
            raise ValueError("Failed to match %s" % loader_class_string)
        cname = m.group(1)
        items = cname.split("_")
        return items[-1]

    def init_gui_menus(self):
        '''initialise menus for console and map'''
        self.menu_added_console = False
        self.menu_added_map = False
        self.menu = None

        if not mp_util.has_wxpython:
            return

        self.menu = MPMenuSubMenu(
            self.mission_type_string(),
            items=self.gui_menu_items()
        )

    def completions(self):
        '''form up MAVProxy-style completion strings used for tab completi
on'''
        cs = self.commands()
        no_arguments = []
        command_argument_buckets = {}
        for c in cs:
            value = cs[c]
            if isinstance(value, tuple):
                (function, arguments) = value
                args_string = " ".join(arguments)
                if args_string not in command_argument_buckets:
                    command_argument_buckets[args_string] = []
                command_argument_buckets[args_string].append(c)
            else:
                no_arguments.append(c)

        ret = []
        if len(no_arguments):
            ret.append("<" + "|".join(sorted(no_arguments)) + ">")
        for k in command_argument_buckets:
            ret.append("<" + "|".join(sorted(command_argument_buckets[k])) + "> " + k)
        return ret

    def unload(self):
        self.remove_command(self.command_name())
        self.unload_remove_menu_items()

    def unload_remove_menu_items(self):
        '''remove out menu items from other modules'''

        if self.menu is None:
            '''can get here if wxpython is not present'''
            return

        if self.module('console') is not None and self.menu_added_console:
            self.menu_added_console = False
            self.module('console').remove_menu(self.menu)
        if self.module('map') is not None and self.menu_added_map:
            self.menu_added_map = False
            self.module('map').remove_menu(self.menu)
        super(MissionItemProtocolModule, self).unload()

    def create_loader(self):
        c = self.loader_class()
        return c()

    def last_change(self):
        return self.wploader.last_change

    def check_have_list(self):
        if self.last_change() == 0:
            print("Please list %s items first" % self.command_name())
            return False
        return True

    def index_from_0(self):
        '''e.g. rally points etc are indexed from 1 from the user interface
        perspective'''
        return False

    def save_filename_base(self):
        return self.itemstype().replace(" ", "-")

    def save_filename(self):
        return self.save_filename_base() + ".txt"

    @property
    def wploader(self):
        '''per-sysid wploader'''
        if self.target_system not in self.wploader_by_sysid:
            self.wploader_by_sysid[self.target_system] = self.create_loader()
            self.wploader_by_sysid[self.target_system].expected_count = 0
        return self.wploader_by_sysid[self.target_system]

    def good_item_num_to_manipulate(self, idx):
        if idx > self.wploader.count():
            return False
        if idx < 1:
            return False
        return True

    def item_num_to_offset(self, item_num):
        if self.index_from_0():
            return item_num
        return item_num - 1

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

    def append(self, item):
        '''append an item to the held item list'''
        if not self.check_have_list():
            return
        if isinstance(item, list):
            for i in item:
                self.wploader.add(i)
                self.wploader.expected_count += 1
        else:
            self.wploader.add(item)
            self.wploader.expected_count += 1
        self.wploader.last_change = time.time()
        self.wploader.reindex()

    def send_wp_requests(self, wps=None):
        '''send some more WP requests'''
        if wps is None:
            wps = self.missing_wps_to_request()
        tnow = time.time()
        for seq in wps:
            self.wp_requested[seq] = tnow
            if self.settings.wp_use_mission_int:
                method = self.master.mav.mission_request_int_send
            else:
                method = self.master.mav.mission_request_send
            method(self.target_system,
                   self.target_component,
                   seq,
                   mission_type=self.mav_mission_type())

    def cmd_status(self, args):
        '''show status of wp download'''
        if not self.check_have_list():
            return
        try:
            print("Have %u of %u %s" % (
                self.wploader.count()+len(self.wp_received),
                self.wploader.expected_count,
                self.itemstype()))
        except Exception:
            print("Have %u %s" % (self.wploader.count()+len(self.wp_received), self.itemstype()))

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ['MISSION_COUNT']:
            if getattr(m, 'mission_type', 0) != self.mav_mission_type():
                return
            if self.wp_op is None:
                if self.wploader.expected_count != m.count:
                    self.console.writeln("Mission is stale")
            else:
                self.wploader.clear()
                self.console.writeln("Requesting %u %s t=%s now=%s" % (
                    m.count,
                    self.itemstype(),
                    time.asctime(time.localtime(m._timestamp)),
                    time.asctime()))
                self.wploader.expected_count = m.count
                self.send_wp_requests()

        elif mtype in ['MISSION_ITEM', 'MISSION_ITEM_INT'] and self.wp_op is not None:
            if m.get_type() == 'MISSION_ITEM_INT':
                if getattr(m, 'mission_type', 0) != self.mav_mission_type():
                    # this is not a mission item, likely fence
                    return
                # our internal structure assumes MISSION_ITEM'''
                m = self.wp_from_mission_item_int(m)
            if m.seq < self.wploader.count():
                # print("DUPLICATE %u" % m.seq)
                return
            if m.seq+1 > self.wploader.expected_count:
                self.console.writeln("Unexpected %s number %u - expected %u" % (self.itemtype(), m.seq, self.wploader.count()))
            self.wp_received[m.seq] = m
            next_seq = self.wploader.count()
            while next_seq in self.wp_received:
                m = self.wp_received.pop(next_seq)
                self.wploader.add(m)
                next_seq += 1
            if self.wploader.count() != self.wploader.expected_count:
                # print("m.seq=%u expected_count=%u" % (m.seq, self.wploader.expected_count))
                self.send_wp_requests()
                return
            if self.wp_op == 'list':
                self.show_and_save(m.get_srcSystem())
                self.loading_waypoints = False
            elif self.wp_op == "save":
                self.save_waypoints(self.wp_save_filename)
            self.wp_op = None
            self.wp_requested = {}
            self.wp_received = {}

        elif mtype in ["MISSION_REQUEST"]:
            self.process_waypoint_request(m, self.master)

    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_period.trigger():
            # cope with packet loss fetching mission
            if (self.master is not None and
                    self.master.time_since('MISSION_ITEM') >= 2 and
                    self.wploader.count() < getattr(self.wploader, 'expected_count', 0)):
                wps = self.missing_wps_to_request()
                print("re-requesting %s %s" % (self.itemstype(), str(wps)))
                self.send_wp_requests(wps)

        self.idle_task_add_menu_items()

    def idle_task_add_menu_items(self):
        '''check for load of other modules, add our items as required'''

        if self.menu is None:
            '''can get here if wxpython is not present'''
            return

        if self.module('console') is not None:
            if not self.menu_added_console:
                self.menu_added_console = True
                self.module('console').add_menu(self.menu)
        else:
            self.menu_added_console = False

        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

    def has_location(self, cmd_id):
        '''return True if a WP command has a location'''
        if cmd_id in mavutil.mavlink.enums['MAV_CMD'].keys():
            cmd_enum = mavutil.mavlink.enums['MAV_CMD'][cmd_id]
            # default to having location for older installs of pymavlink
            # which don't have the attribute
            return getattr(cmd_enum, 'has_location', True)
        return False

    def wp_to_mission_item_int(self, wp):
        '''convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as
           MISSION_ITEM_INT to give cm level accuracy
        '''
        if wp.get_type() == 'MISSION_ITEM_INT':
            return wp
        if self.has_location(wp.command):
            p5 = int(wp.x*1.0e7)
            p6 = int(wp.y*1.0e7)
        else:
            p5 = int(wp.x)
            p6 = int(wp.y)
        wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(
            wp.target_system,
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
            p5,
            p6,
            wp.z,
            wp.mission_type
        )
        return wp_int

    def wp_from_mission_item_int(self, wp):
        '''convert a MISSION_ITEM_INT to a MISSION_ITEM'''
        if self.has_location(wp.command):
            p5 = wp.x*1.0e-7
            p6 = wp.y*1.0e-7
        else:
            p5 = wp.x
            p6 = wp.y
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
                                                           p5,
                                                           p6,
                                                           wp.z,
                                                           wp.mission_type)
        # preserve srcSystem as that is used for naming waypoint file
        wp2._header.srcSystem = wp.get_srcSystem()
        wp2._header.srcComponent = wp.get_srcComponent()
        return wp2

    def process_waypoint_request(self, m, master):
        '''process a waypoint request from the master'''
        if m.mission_type != self.mav_mission_type():
            return
        # print("Processing %s request: (%s)" % (self.itemtype(), str(m)))
        if (m.target_system != self.settings.source_system or
                m.target_component != self.settings.source_component):
            # self.console.error("Mission request is not for me")
            return
        if (not self.loading_waypoints or
                time.time() > self.loading_waypoint_lasttime + 10.0):
            self.loading_waypoints = False
            # self.console.error("not loading waypoints")
            return
        if m.seq >= self.wploader.count():
            self.console.error("Request for bad %s %u (max %u)" %
                               (self.itemtype, m.seq, self.wploader.count()))
            return
        wp = self.wploader.wp(m.seq)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        if self.settings.wp_use_mission_int:
            wp_send = self.wp_to_mission_item_int(wp)
        else:
            wp_send = wp

        if wp.mission_type != self.mav_mission_type():
            print("Wrong mission type in (%s)" % str(wp))

        self.master.mav.send(wp_send)

        self.loading_waypoint_lasttime = time.time()

        # update the user on our progress:
        self.mpstate.console.set_status(self.itemtype(), '%s %u/%u' % (self.itemtype(), m.seq, self.wploader.count()-1))

        # see if the transfer is complete:
        if m.seq == self.wploader.count() - 1:
            self.loading_waypoints = False
            print("Loaded %u %s in %.2fs" % (
                self.wploader.count(),
                self.itemstype(),
                time.time() - self.upload_start))
            self.console.writeln(
                "Sent all %u %s" %
                (self.wploader.count(), self.itemstype()))

    def send_all_waypoints(self):
        return self.send_all_items()

    def send_all_items(self):
        '''send all waypoints to vehicle'''
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.upload_start = time.time()
        self.master.mav.mission_count_send(
            self.target_system,
            self.target_component,
            self.wploader.count(),
            mission_type=self.mav_mission_type())

    def load_waypoints(self, filename):
        '''load waypoints from a file'''
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.load(filename.strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u %s from %s" % (self.wploader.count(), self.itemstype(), filename))
        self.wploader.expected_count = self.wploader.count()
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
            print("No %s found in %s" % (self.itemstype(), filename))
            return
        if wpnum == -1:
            print("Loaded %u updated %s from %s" % (self.wploader.count(), self.itemstype(), filename))
        elif wpnum >= self.wploader.count():
            print("Invalid %s number %u" % (self.itemtype(), wpnum))
            return
        else:
            print("Loaded updated %s %u from %s" % (self.itemtype(), wpnum, filename))

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        if wpnum == -1:
            start = 0
            end = self.wploader.count()-1
        else:
            start = wpnum
            end = wpnum
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            start,
            end,
            self.mav_mission_type())

    def save_waypoints(self, filename):
        '''save waypoints to a file'''
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.save(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u %s to %s" % (self.wploader.count(), self.itemstype(), filename))

    def save_waypoints_csv(self, filename):
        '''save waypoints to a file in a human readable CSV file'''
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.savecsv(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u %s to CSV %s" % (self.wploader.count(), self.itemstype(), filename))

    def cmd_move(self, args):
        '''handle wp move'''
        if len(args) != 1:
            print("usage: wp move WPNUM")
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        offset = self.item_num_to_offset(idx)
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return
        wp = self.wploader.wp(offset)

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "move"

        (lat, lon) = latlon
        if (len(self.module_matching('terrain')) > 0 and
                wp.frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT and
                self.settings.wpterrainadjust):
            alt1 = self.module('terrain').ElevationModel.GetElevation(lat, lon)
            alt2 = self.module('terrain').ElevationModel.GetElevation(wp.x, wp.y)
            if alt1 is not None and alt2 is not None:
                wp.z += alt1 - alt2
        wp.x = lat
        wp.y = lon

        wp.target_system    = self.target_system
        wp.target_component = self.target_component
        self.wploader.set(wp, offset)
        self.wploader.last_change = time.time()

        print("Moving %s %u to %f, %f at %.1fm" % (self.itemtype(), idx, lat, lon, wp.z))

        self.send_single_waypoint(offset)

    def send_single_waypoint(self, idx):
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.upload_start = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            idx,
            idx,
            self.mav_mission_type()
        )

    def is_location_command(self, cmd):
        '''see if cmd is a MAV_CMD with a latitude/longitude'''
        mav_cmd = mavutil.mavlink.enums['MAV_CMD']
        if cmd not in mav_cmd:
            return False
        return getattr(mav_cmd[cmd], 'has_location', True)

    def is_location_wp(self, w):
        '''see if w.command is a MAV_CMD with a latitude/longitude'''
        if w.x == 0 and w.y == 0:
            return False
        return self.is_location_command(w.command)

    def cmd_movemulti(self, args, latlon=None):
        '''handle wp move of multiple waypoints'''
        if len(args) < 3:
            print("usage: wp movemulti WPNUM WPSTART WPEND <rotation>")
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid move %s number %u" % (self.itemtype(), idx))
            return
        wpstart = int(args[1])
        if not self.good_item_num_to_manipulate(wpstart):
            print("Invalid start %s number %u" % (self.itemtype(), wpstart))
            return
        wpend = int(args[2])
        if not self.good_item_num_to_manipulate(wpend):
            print("Invalid end %s number %u" % (self.itemtype(), wpend))
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
        idx_offset = self.item_num_to_offset(idx)
        wp = self.wploader.wp(idx_offset)
        if not self.is_location_wp(wp):
            print("WP must be a location command")
            return

        (lat, lon) = latlon
        distance = mp_util.gps_distance(wp.x, wp.y, lat, lon)
        bearing  = mp_util.gps_bearing(wp.x, wp.y, lat, lon)

        wpstart_offset = self.item_num_to_offset(wpstart)
        wpend_offset = self.item_num_to_offset(wpend)
        for wpnum in range(wpstart_offset, wpend_offset+1):
            wp = self.wploader.wp(wpnum)
            if wp is None or not self.is_location_wp(wp):
                continue
            (newlat, newlon) = mp_util.gps_newpos(wp.x, wp.y, bearing, distance)
            if wpnum != idx and rotation != 0:
                # add in rotation
                d2 = mp_util.gps_distance(lat, lon, newlat, newlon)
                b2 = mp_util.gps_bearing(lat, lon, newlat, newlon)
                (newlat, newlon) = mp_util.gps_newpos(lat, lon, b2+rotation, d2)

            if (len(self.module_matching('terrain')) > 0 and
                    wp.frame != mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT and
                    self.settings.wpterrainadjust):
                alt1 = self.module('terrain').ElevationModel.GetElevation(newlat, newlon)
                alt2 = self.module('terrain').ElevationModel.GetElevation(wp.x, wp.y)
                if alt1 is not None and alt2 is not None:
                    wp.z += alt1 - alt2
            wp.x = newlat
            wp.y = newlon
            wp.target_system    = self.target_system
            wp.target_component = self.target_component
            self.wploader.set(wp, wpnum)

        self.wploader.last_change = time.time()
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            wpstart_offset,
            wpend_offset)
        print("Moved %s %u:%u to %f, %f rotation=%.1f" % (self.itemstype(), wpstart, wpend, lat, lon, rotation))

    def change_mission_item_range(self, args, desc, changer, newvalstr):
        if not self.check_have_list():
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        if len(args) >= 2:
            count = int(args[1])
        else:
            count = 1
        if not self.good_item_num_to_manipulate(idx+count-1):
            print("Invalid %s number %u" % (self.itemtype(), idx+count-1))
            return

        for wpnum in range(idx, idx+count):
            offset = self.item_num_to_offset(wpnum)
            wp = self.wploader.wp(offset)
            if wp is None:
                continue
            if not self.wploader.is_location_command(wp.command):
                continue
            changer(wp)
            wp.target_system = self.target_system
            wp.target_component = self.target_component
            self.wploader.set(wp, offset)

        self.wploader.last_change = time.time()
        self.upload_start = time.time()
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            self.item_num_to_offset(idx),
            self.item_num_to_offset(idx+count),
            mission_type=self.mav_mission_type())
        print("Changed %s for WPs %u:%u to %s" % (desc, idx, idx+(count-1), newvalstr))

    def cmd_changealt(self, args):
        '''handle wp change target alt of multiple waypoints'''
        if len(args) < 2:
            print("usage: %s changealt WPNUM NEWALT <NUMWP>" % self.command_name())
            return
        value = float(args[1])
        del args[1]

        def changer(wp):
            wp.z = value
        self.change_mission_item_range(args, "alt", changer, str(value))

    def cmd_changeframe(self, args):
        '''handle wp change frame of multiple waypoints'''
        if len(args) < 2:
            print("usage: %s changeframe WPNUM NEWFRAME <NUMWP>" % self.command_name())
            return
        value = int(args[1])
        del args[1]

        def changer(wp):
            wp.frame = value
        self.change_mission_item_range(args, "frame", changer, str(value))

    def fix_jumps(self, idx, delta):
        # nothing by default as only waypoints need worry
        pass

    def cmd_remove(self, args):
        '''handle wp remove'''
        if len(args) != 1:
            print("usage: %s remove WPNUM" % self.command_name())
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        offset = self.item_num_to_offset(idx)
        wp = self.wploader.wp(offset)

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "remove"

        self.wploader.remove(wp)
        self.wploader.expected_count -= 1
        self.wploader.last_change = time.time()
        self.fix_jumps(offset, -1)
        self.send_all_waypoints()
        print("Removed %s %u" % (self.itemtype(), idx))

    def cmd_undo(self, args):
        '''handle wp undo'''
        if self.undo_wp_idx == -1 or self.undo_wp is None:
            print("No undo information")
            return
        wp = self.undo_wp
        if self.undo_type == 'move':
            wp.target_system    = self.target_system
            wp.target_component = self.target_component
            offset = self.item_num_to_offset(self.undo_wp_idx)
            self.wploader.set(wp, offset)
            self.wploader.last_change = time.time()
            self.send_single_waypoint(offset)
            print("Undid %s move" % self.itemtype())
        elif self.undo_type == 'remove':
            offset = self.item_num_to_offset(self.undo_wp_idx)
            self.wploader.insert(offset, wp)
            self.wploader.expected_count += 1
            self.wploader.last_change = time.time()
            self.fix_jumps(self.undo_wp_idx, 1)
            self.send_all_waypoints()
            print("Undid %s remove" % self.itemtype())
        else:
            print("bad undo type")
        self.undo_wp = None
        self.undo_wp_idx = -1

    def cmd_param(self, args):
        '''handle wp parameter change'''
        if len(args) < 2:
            print("usage: wp param WPNUM PNUM <VALUE>")
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        offset = self.item_num_to_offset(idx)
        wp = self.wploader.wp(offset)
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
        self.wploader.set(wp, idx)
        self.wploader.last_change = time.time()
        self.send_single_waypoint(idx)

    def cmd_clear(self, args):
        self.master.mav.mission_clear_all_send(
            self.target_system,
            self.target_component,
            mission_type=self.mav_mission_type())
        self.wploader.clear()
        if getattr(self.wploader, 'expected_count', None) is not None:
            self.wploader.expected_count = 0
        self.wploader.expected_count = 0
        self.loading_waypoint_lasttime = time.time()

    def cmd_list(self, args):
        self.wp_op = "list"
        self.request_list_send()

    def cmd_load(self, args):
        if len(args) != 1:
            print("usage: %s load FILENAME" % self.command_name())
            return
        self.load_waypoints(args[0])

    def cmd_save(self, args):
        if len(args) != 1:
            print("usage: %s save <filename>" % self.command_name())
            return
        self.wp_save_filename = args[0]
        self.wp_op = "save"
        self.request_list_send()

    def cmd_savecsv(self, args):
        if len(args) != 1:
            print("usage: wp savecsv <filename.csv>")
            return
        self.savecsv(args[0])

    def cmd_savelocal(self, args):
        if len(args) != 1:
            print("usage: wp savelocal <filename>")
            return
        self.wploader.save(args[0])

    def cmd_show(self, args):
        if len(args) != 1:
            print("usage: wp show <filename>")
            return
        self.wploader.load(args[0])

    def cmd_update(self, args):
        if not self.check_have_list():
            return
        if len(args) < 1:
            print("usage: %s update <filename> <wpnum>" % self.command_name())
            return
        if len(args) == 2:
            wpnum = int(args[1])
        else:
            wpnum = -1
        self.update_waypoints(args[0], wpnum)

    def commands(self):
        if self.master and not self.master.mavlink20():
            print("%s module not available; use old compat modules" % str(self.itemtype()))
            return
        return {
            "ftp": self.wp_ftp_download,
            "ftpload": self.wp_ftp_upload,
            "clear": self.cmd_clear,
            "list": self.cmd_list,
            "load": (self.cmd_load, ["(FILENAME)"]),
            "remove": self.cmd_remove,
            "save": (self.cmd_save, ["(FILENAME)"]),
            "savecsv": (self.cmd_savecsv, ["(FILENAME)"]),
            "savelocal": self.cmd_savelocal,
            "show": (self.cmd_show, ["(FILENAME)"]),
            "status": self.cmd_status,
        }

    def usage(self):
        subcommands = "|".join(sorted(self.commands().keys()))
        return "usage: %s <%s>" % (self.command_name(), subcommands)

    def cmd_wp(self, args):
        '''waypoint commands'''
        if len(args) < 1:
            print(self.usage())
            return

        commands = self.commands()
        if args[0] not in commands:
            print(self.usage())
            return

        function = commands[args[0]]
        if isinstance(function, tuple):
            (function, function_arguments) = function
            # TODO: do some argument validation here, remove same from
            # cmd_*

        function(args[1:])

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
        # headers = ["Seq", "Frame", "Cmd", "P1", "P2", "P3", "P4", "X", "Y", "Z"]
        for w in self.wploader.wpoints:
            if getattr(w, 'comment', None):
                #                f.write("# %s\n" % w.comment)
                pass
            out_list = [
                w.seq,
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
            f.write(self.csv_line(out_list) + "\n")
        f.close()

    def fetch(self):
        """Download wpts from vehicle (this operation is public to support other modules)"""
        if self.wp_op is None:  # If we were already doing a list or save, just restart the fetch without changing the operation  # noqa
            self.wp_op = "fetch"
        self.request_list_send()

    def request_list_send(self):
        self.master.mav.mission_request_list_send(
            self.target_system,
            self.target_component,
            mission_type=self.mav_mission_type())

    def wp_ftp_download(self, args):
        '''Download items from vehicle with ftp'''
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            print("Need ftp module")
            return
        self.ftp_count = None
        ftp.cmd_get([self.mission_ftp_name()], callback=self.ftp_callback, callback_progress=self.ftp_callback_progress)

    def ftp_callback_progress(self, fh, total_size):
        '''progress callback from ftp fetch of mission items'''
        if self.ftp_count is None and total_size >= 10:
            ofs = fh.tell()
            fh.seek(0)
            buf = fh.read(10)
            fh.seek(ofs)
            magic2, dtype, options, start, num_items = struct.unpack("<HHHHH", buf)
            if magic2 == 0x763d:
                self.ftp_count = num_items
        if self.ftp_count is not None:
            mavmsg = mavutil.mavlink.MAVLink_mission_item_int_message
            item_size = mavmsg.unpacker.size
            done = (total_size - 10) // item_size
            self.mpstate.console.set_status('Mission', 'Mission %u/%u' % (done, self.ftp_count))

    def ftp_callback(self, fh):
        '''callback from ftp fetch of mission items'''
        if fh is None:
            print("mission: failed ftp download")
            return
        magic = 0x763d
        data = fh.read()
        magic2, dtype, options, start, num_items = struct.unpack("<HHHHH", data[0:10])
        if magic != magic2:
            print("%s: bad magic 0x%x expected 0x%x" % (self.itemtype(), magic2, magic))
            return
        if dtype != self.mav_mission_type():
            print("%s: bad data type %u" % (self.itemtype(), dtype))
            return

        self.wploader.clear()

        data = data[10:]
        mavmsg = mavutil.mavlink.MAVLink_mission_item_int_message
        item_size = mavmsg.unpacker.size
        while len(data) >= item_size:
            mdata = data[:item_size]
            data = data[item_size:]
            msg = mavmsg.unpacker.unpack(mdata)
            tlist = list(msg)
            t = tlist[:]
            for i in range(0, len(tlist)):
                tlist[i] = t[mavmsg.orders[i]]
            t = tuple(tlist)
            w = mavmsg(*t)
            w = self.wp_from_mission_item_int(w)
            self.wploader.add(w)
        self.show_and_save(self.target_system)

    def show_and_save(self, source_system):
        '''display waypoints and save'''
        for i in range(self.wploader.count()):
            w = self.wploader.wp(i)
            print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                w.command, w.frame, w.x, w.y, w.z,
                w.param1, w.param2, w.param3, w.param4,
                w.current, w.autocontinue))
        if self.logdir is not None:
            fname = self.save_filename()
            if source_system != 1:
                fname = '%s_%u.txt' % (self.save_filename_base(), source_system)
            waytxt = os.path.join(self.logdir, fname)
            self.save_waypoints(waytxt)
            print("Saved %s to %s" % (self.itemstype(), waytxt))

    def wp_ftp_upload(self, args):
        '''upload waypoints to vehicle with ftp'''
        filename = args[0]
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            print("Need ftp module")
            return
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.load(filename.strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u %s from %s" % (self.wploader.count(), self.itemstype(), filename))
        print("Sending %s with ftp" % self.itemstype())

        fh = SIO()
        fh.write(struct.pack("<HHHHH", 0x763d, self.mav_mission_type(), 0, 0, self.wploader.count()))
        mavmsg = mavutil.mavlink.MAVLink_mission_item_int_message
        for i in range(self.wploader.count()):
            w = self.wploader.wp(i)
            w = self.wp_to_mission_item_int(w)
            tlist = []
            for field in mavmsg.ordered_fieldnames:
                tlist.append(getattr(w, field))
            tlist = tuple(tlist)
            buf = mavmsg.unpacker.pack(*tlist)
            fh.write(buf)
        fh.seek(0)

        self.upload_start = time.time()

        ftp.cmd_put([self.mission_ftp_name(), self.mission_ftp_name()],
                    fh=fh, callback=self.ftp_upload_callback, progress_callback=self.ftp_upload_progress)

    def ftp_upload_progress(self, proportion):
        '''callback from ftp put of items'''
        if proportion is None:
            self.mpstate.console.set_status('Mission', 'Mission ERR')
        else:
            count = self.wploader.count()
            self.mpstate.console.set_status('Mission', 'Mission %u/%u' % (int(proportion*count), count))

    def ftp_upload_callback(self, dlen):
        '''callback from ftp put of items'''
        if dlen is None:
            print("Failed to send %s" % self.itemstype())
        else:
            mavmsg = mavutil.mavlink.MAVLink_mission_item_int_message
            item_size = mavmsg.unpacker.size
            print("Sent %s of length %u in %.2fs" %
                  (self.itemtype(), (dlen - 10) // item_size, time.time() - self.upload_start))
