#!/usr/bin/env python
'''
CMAC mission control
Peter Barker

based on

CUAV mission control
Andrew Tridgell
'''

import math
import os
import time

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util

import pkg_resources

CMAC_LOCATION = mavutil.location(-35.363261, 149.165230, 584, 353)

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu


class CMACModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CMACModule, self).__init__(mpstate,
                                         "CMAC",
                                         "CMAC Checks",
                                         public=True)
        self.rate_period = mavutil.periodic_event(1.0/15)
        self.is_armed = False
        self.done_map_menu = False

        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        self.cmac_settings = MPSettings(
            [
                MPSetting('fence_maxdist', float, 1000,
                          'Max FencePoint Distance from south-CMAC'),
                MPSetting('wp_maxdist', float, 500,
                          'Max WayPoint Distance from south-CMAC'),
                MPSetting('rally_maxdist', float, 200,
                          'Max Rally Distance from south-CMAC'),
            ])
        self.add_completion_function('(CMACCHECKSETTING)',
                                     self.cmac_settings.completion)
        self.add_command('cmaccheck',
                         self.cmd_cmaccheck,
                         'cmac check control',
                         ['check',
                          'set (CMACCHECKSETTING)'])

        self.last_fence_fetch = 0
        self.last_mission_fetch = 0
        self.last_rally_fetch = 0
        self.done_heartbeat_check = 0

        self.check()

    # swiped from ArduPilot's common.py:
    def get_distance(self, loc1, loc2):
        """Get ground distance between two locations."""
        dlat = loc2.lat - loc1.lat
        try:
            dlong = loc2.lng - loc1.lng
        except AttributeError:
            dlong = loc2.lon - loc1.lon

        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def check_map_menu(self):
        # make the initial map menu
        if not mp_util.has_wxpython:
            return
        if self.done_map_menu:
            if not self.module('map'):
                self.done_map_menu = False
            return

        if self.module('map'):
            self.menu = MPMenuSubMenu('CMAC', items=[
                MPMenuItem('Load foamy mission CW',
                           'Load foamy mission CW',
                           '# cmaccheck loadFoamyMissionCW'),
                MPMenuItem('Load foamy mission CCW',
                           'Load foamy mission CCW',
                           '# cmaccheck loadFoamyMissionCCW'),
                MPMenuItem('Load rally points',
                           'Load rally points',
                           '# cmaccheck loadRally'),
                MPMenuItem('Load foamy fence',
                           'Load foamy fence',
                           '# cmaccheck loadFoamyFence'),
            ])
            self.module('map').add_menu(self.menu)
            self.done_map_menu = True

    def loadRally(self):
        filename = "cmac-foamy-rally.txt"
        filepath = pkg_resources.resource_filename(__name__, filename)
        if os.path.exists(filepath):
            rallymod = self.module('rally')
            rallymod.cmd_rally(["load", filepath])

    def loadFoamyFence(self):
        filename = "cmac-foamy-fence.txt"
        filepath = pkg_resources.resource_filename(__name__, filename)
        if os.path.exists(filepath):
            fencemod = self.module('fence')
            fencemod.cmd_fence(["load", filepath])

    def loadFoamyMission(self, filename):
        filepath = pkg_resources.resource_filename(__name__, filename)
        wpmod = self.module('wp')
        wpmod.cmd_wp(["load", filepath])

    def loadFoamyMissionCW(self):
        self.loadFoamyMission("cmac-foamy-mission-cw.txt")

    def loadFoamyMissionCCW(self):
        self.loadFoamyMission("cmac-foamy-mission-ccw.txt")

    def cmd_cmaccheck(self, args):
        '''handle cmaccheck commands'''
        usage = 'Usage: cmaccheck <set>'
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.cmac_settings.command(args[1:])
        elif args[0] == "loadFoamyMissionCW":
            self.loadFoamyMissionCW()
        elif args[0] == "loadFoamyMissionCCW":
            self.loadFoamyMissionCCW()
        elif args[0] == "loadFoamyFence":
            self.loadFoamyFence()
        elif args[0] == "loadRally":
            self.loadRally()
        elif args[0] == "check":
            self.check()
        else:
            print(usage)
            return

    def whinge(self, message):
        self.console.writeln("CMAC: %s" % (message,))

    def check_parameters(self):
        '''check key parameters'''
        want_values = {
            "FENCE_ACTION": 4,
            "FENCE_MAXALT": 80,
            "THR_FAILSAFE": 1,
            "FS_SHORT_ACTN": 0,
            "FS_LONG_ACTN": 1,
        }

        for key in want_values.keys():
            want = want_values[key]
            got = self.mav_param.get(key, None)
            if got is None:
                self.whinge("No param %s" % key)
                return False
            if got != want:
                self.whinge('%s should be %f (not %s)' % (key, want, got))
                return False

        return True

    def idle_task(self):
        '''run periodic tasks'''
        self.check_map_menu()

    def check_fence_location(self):
        fencemod = self.module('fence')
        if fencemod is None:
            self.whinge("Fence module not loaded")
            return False
        if not fencemod.have_list:
            self.whinge("No fence list")
            if self.is_armed:
                return False
            now = time.time()
            if now - self.last_fence_fetch > 10:
                self.last_fence_fetch = now
                self.whinge("Running 'fence list'")
                fencemod.list_fence(None)
            return False

        count = fencemod.fenceloader.count()
        if count < 6:
            self.whinge("Too few fence points")
            return False
        ret = True
        for i in range(fencemod.fenceloader.count()):
            p = fencemod.fenceloader.point(i)
            loc = mavutil.location(p.lat, p.lng)
            dist = self.get_distance(CMAC_LOCATION, loc)
            if dist > self.cmac_settings.fence_maxdist:
                self.whinge("Fencepoint %i too far away (%fm)" % (i, dist))
                ret = False
        return ret

    def check_rally(self):
        rallymod = self.module('rally')
        if rallymod is None:
            self.whinge("No rally module")
            return False
        if not rallymod.have_list:
            self.whinge("No rally list")
            if self.is_armed:
                return False
            now = time.time()
            if now - self.last_rally_fetch > 10:
                self.last_rally_fetch = now
                self.whinge("Running 'rally list'")
                rallymod.cmd_rally(["list"])
            return False

        count = rallymod.rallyloader.rally_count()
        if count < 1:
            self.whinge("Too few rally points")
            return False

        ret = True
        for i in range(count):
            r = rallymod.rallyloader.rally_point(i)
            loc = mavutil.location(r.lat/10000000.0, r.lng/10000000.0)
            dist = self.get_distance(CMAC_LOCATION, loc)
            if dist > self.cmac_settings.rally_maxdist:
                self.whinge("Rally Point %i too far away (%fm)" % (i, dist))
                ret = False

                # ensure we won't loiter over the runway when doing
                # rally loitering:
            v = self.mav_param.get("RTL_RADIUS", None)
            if v is None or v == 0:
                v = self.mav_param.get("WP_LOITER_RAD")
            if v is None:
                self.whinge("No RTL loiter radius available")
                ret = False
#            print("dist=%f v=%f", dist, v)
            if dist < v+30:  # add a few metres of slop
                self.whinge("Rally Point %i too close (%fm)" % (i, dist))
                ret = False

        return ret

    def check_fence_health(self):
        try:
            sys_status = self.master.messages['SYS_STATUS']
        except Exception:
            return False

        bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        present = ((sys_status.onboard_control_sensors_present & bits) == bits)
        enabled = ((sys_status.onboard_control_sensors_enabled & bits) == bits)
        healthy = ((sys_status.onboard_control_sensors_health & bits) == bits)
        if not present or not enabled:
            self.console.writeln('Fence should be enabled', fg='blue')
            return False
        if not healthy:
            self.console.writeln('Fence unhealthy', fg='blue')
            return False

        return True

    def check_fence(self):
        ret = True
        if not self.check_fence_health():
            ret = False
        if not self.check_fence_location():
            ret = False
        return ret

    def check_mission(self):
        wpmod = self.module('wp')
        if wpmod is None:
            self.whinge("No waypoint module")
            return False
        count = wpmod.wploader.count()
        if count == 0:
            self.whinge("No waypoints")
            if self.is_armed:
                return False
            now = time.time()
            if now - self.last_mission_fetch > 10:
                self.whinge("Requesting waypoints")
                self.last_mission_fetch = now
                wpmod.wp_op = "list"
                wpmod.master.waypoint_request_list_send()
            return False
        if count < 2:
            self.whinge("Too few waypoints")
            return False

        ret = True
        for i in range(count):
            if i == 0:
                # skip home
                continue
            w = wpmod.wploader.wp(i)
            if w.command not in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND]:
                continue
            loc = mavutil.location(w.x, w.y)
            dist = self.get_distance(CMAC_LOCATION, loc)
            if dist > self.cmac_settings.wp_maxdist:
                self.whinge("Waypoint %i too far away (%fm)" % (i, dist))
                ret = False
        return ret

    def check_status(self):
        try:
            hb = self.master.messages['HEARTBEAT']
            mc = self.master.messages['MISSION_CURRENT']
        except Exception:
            return False
        self.is_armed = (hb.base_mode &
                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if not self.is_armed and hb.custom_mode == 0:
            # disarmed in MANUAL we should be at WP 0
            if mc.seq > 1:
                self.whinge('Incorrect WP %u' % mc.seq)
                return False
        return True

    def check(self):
        success = True
        self.check_map_menu()

        if self.master.messages.get('HEARTBEAT') is None:
            self.whinge("Waiting for heartbeat")
            success = False
            return
        if not self.check_status():
            success = False
        if not self.check_parameters():
            success = False
        if not self.check_fence():
            success = False
        if not self.check_mission():
            success = False
        if not self.check_rally():
            success = False
        if not success:
            self.whinge("CHECKS BAD")
            return
        if not self.is_armed:
            self.whinge("CHECKS GOOD")

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if not self.done_heartbeat_check:
            if self.master.messages.get('HEARTBEAT') is not None:
                self.check()
                self.done_heartbeat_check = True

        if self.rate_period.trigger():
            self.check()


def init(mpstate):
    '''initialise module'''
    return CMACModule(mpstate)
