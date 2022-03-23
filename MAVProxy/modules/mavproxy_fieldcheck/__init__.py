#!/usr/bin/env python
'''
CMAC mission control
Peter Barker

based on

CUAV mission control
Andrew Tridgell

TODO:
 - ensure parameters are correct
 - add check that battery is within tolerances (chemistry issues...)
 - autodetect numcells being incorrect
'''

import math
import os
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSettings
from MAVProxy.modules.lib.mp_settings import MPSetting

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util

import pkg_resources

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu

class FieldCheck(object):
    def __init__(self):
        self.is_armed = False

        self.last_fence_fetch = 0
        self.last_mission_fetch = 0
        self.last_rally_fetch = 0
        self.done_heartbeat_check = 0

        # an altitude should always be within a few metres of when disarmed:
        self.disarmed_alt = 584
        self.rate_period = mavutil.periodic_event(1.0/15)

        self.done_map_menu = False

    def close_to(self, loc1):
        ret = self.get_distance(loc1, self.location)
        print("Distance to %s: %um" % (self.lc_name, ret))
        return ret < 100

    # swiped from ArduPilot's common.py:
    def get_distance(self, loc1, loc2):
        """Get ground distance between two locations."""
        dlat = loc2.lat - loc1.lat
        try:
            dlong = loc2.lng - loc1.lng
        except AttributeError:
            dlong = loc2.lon - loc1.lon

        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def flightdata_filepath(self, filename):
        if os.path.exists(filename):
            return filename
        return pkg_resources.resource_filename(__name__, filename)

    def loadRally(self):
        filepath = self.flightdata_filepath(self.fc_settings.rally_filename)
        rallymod = self.module('rally')
        rallymod.cmd_rally(["load", filepath])

    def loadFoamyFence(self):
        filepath = self.flightdata_filepath(self.fc_settings.fence_filename)
        fencemod = self.module('fence')
        fencemod.cmd_fence(["load", filepath])

    def loadFoamyMission(self, filename):
        filepath = self.flightdata_filepath(filename)
        wpmod = self.module('wp')
        wpmod.cmd_wp(["load", filepath])

    def loadFoamyMissionCW(self):
        self.loadFoamyMission(self.fc_settings.mission_filename_cw)

    def loadFoamyMissionCCW(self):
        self.loadFoamyMission(self.fc_settings.mission_filename_ccw)

    def fixMissionRallyFence(self):
        self.loadFoamyMissionCW()
        self.loadFoamyFence()
        self.loadRally()

    def fixEVERYTHING(self):
        self.loadFoamyMissionCW()
        self.loadFoamyFence()
        self.loadRally()
        self.check_parameters(fix=True)

    def whinge(self, message):
        self.console.writeln("FC:%s %s" % (self.lc_name, message,))

    def check_parameters(self, fix=False):
        '''check key parameters'''
        want_values = {
            "FENCE_ACTION": 4,
            "FENCE_MAXALT": self.fc_settings.param_fence_maxalt,
            "THR_FAILSAFE": 1,
            "FS_SHORT_ACTN": 0,
            "FS_LONG_ACTN": 1,
        }

        ret = True
        for key in want_values.keys():
            want = want_values[key]
            got = self.mav_param.get(key, None)
            if got is None:
                self.whinge("No param %s" % key)
                ret = False
            if got != want:
                self.whinge('%s should be %f (not %s)' % (key, want, got))
                ret = False
                if fix:
                    self.whinge('Setting %s to %f' % (key, want))
                    self.mav_param.mavset(self.master, key, want, retries=3)

        return ret

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
            dist = self.get_distance(self.location, loc)
            if dist > self.fc_settings.fence_maxdist:
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

        count = rallymod.rally_count()
        if count < 1:
            self.whinge("Too few rally points")
            return False

        rtl_loiter_radius = self.mav_param.get("RTL_RADIUS", None)
        if rtl_loiter_radius is None or rtl_loiter_radius == 0:
            rtl_loiter_radius = self.mav_param.get("WP_LOITER_RAD")
        if rtl_loiter_radius is None:
            self.whinge("No RTL loiter radius available")
            return False

        ret = True
        for i in range(count):
            r = rallymod.rally_point(i)
            loc = mavutil.location(r.lat/10000000.0, r.lng/10000000.0)
            dist = self.get_distance(self.location, loc)
            if dist > self.fc_settings.rally_maxdist:
                self.whinge("Rally Point %i too far away (%fm)" % (i, dist))
                ret = False

            # ensure we won't loiter over the runway when doing
            # rally loitering:
#            print("dist=%f rtl_loiter_radius=%f", dist, rtl_loiter_radius)
            if dist < rtl_loiter_radius+30:  # add a few metres of slop
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

    def check_altitude(self):
        if self.is_armed:
            return True
        try:
            gpi = self.master.messages['GLOBAL_POSITION_INT']
        except Exception:
            return False
        max_delta = 10
        current_alt = gpi.alt / 1000
        if abs(current_alt - self.disarmed_alt) > max_delta:
            self.whinge("Altitude (%f) not within %fm of %fm" %
                        (current_alt, max_delta, self.disarmed_alt))
            return False
        return True

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
            dist = self.get_distance(self.location, loc)
            if dist > self.fc_settings.wp_maxdist:
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
        if not self.check_altitude():
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

    def check_map_menu(self):
        # make the initial map menu
        if not mp_util.has_wxpython:
            return
        if self.done_map_menu:
            if not self.module('map'):
                self.done_map_menu = False
            return

        if self.module('map'):
            self.menu = MPMenuSubMenu('FieldCheck', items=[
                MPMenuItem('Load foamy mission CW',
                           'Load foamy mission CW',
                           '# fieldcheck loadFoamyMissionCW'),
                MPMenuItem('Load foamy mission CCW',
                           'Load foamy mission CCW',
                           '# fieldcheck loadFoamyMissionCCW'),
                MPMenuItem('Load rally points',
                           'Load rally points',
                           '# fieldcheck loadRally'),
                MPMenuItem('Load foamy fence',
                           'Load foamy fence',
                           '# fieldcheck loadFoamyFence'),
                MPMenuItem('Fix Mission+Rally+Fence',
                           'Fix Mission+Rally+Fence',
                           '# fieldcheck fixMissionRallyFence'),
                MPMenuItem('Fix EVERYTHING',
                           'Fix EVERYTHING',
                           '# fieldcheck fixEVERYTHING'),
            ])
            self.module('map').add_menu(self.menu)
            self.done_map_menu = True

    def idle_task(self):
        self.check_map_menu()

    def FC_MPSetting(self, name, atype, default, description):
        xname = "fc_%s_%s" % (self.lc_name, name)
        return MPSetting(name, atype, default, description)

    def select(self):
        self.fc_settings = MPSettings(
            [
                self.FC_MPSetting('fence_maxdist',
                                  float,
                                  1000,
                                  'Max FencePoint Distance from location'),
                self.FC_MPSetting('wp_maxdist',
                                  float,
                                  500,
                                  'Max WayPoint Distance from location'),
                self.FC_MPSetting('rally_maxdist',
                                  float,
                                  200,
                                  'Max Rally Distance from location'),
                self.FC_MPSetting('param_fence_maxalt',
                                  float,
                                  120,
                                  'Value parameter FENCE_MAXALT should have'),
                self.FC_MPSetting('rally_filename',
                                  str,
                                  "%s-foamy-rally.txt" % self.lc_name,
                                  "%s Rally Point File" % self.lc_name),
                self.FC_MPSetting('fence_filename',
                                  str,
                                  "%s-foamy-fence.txt" % self.lc_name,
                                  "%s Fence File" % self.lc_name),
                self.FC_MPSetting('mission_filename_cw',
                                  str,
                                  "%s-foamy-mission-cw.txt" % self.lc_name,
                                  "%s Mission (CW) File" % self.lc_name),
                self.FC_MPSetting('mission_filename_ccw',
                                  str,
                                  "%s-foamy-mission-ccw.txt" % self.lc_name,
                                  "%s Mission (CCW) File" % self.lc_name),
            ])
        self.x.add_completion_function('(FIELDCHECKCHECKSETTING)',
                                       self.fc_settings.completion)
        self.x.add_command('fieldcheck',
                           self.cmd_fieldcheck,
                           'field check control',
                           ['check',
                            'set (FIELDCHECKSETTING)'])

    def cmd_fieldcheck(self, args):
        '''handle fieldcheck commands'''
        usage = 'Usage: fieldcheck <set>'
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.fc_settings.command(args[1:])
        elif args[0] == "loadFoamyMissionCW":
            self.loadFoamyMissionCW()
        elif args[0] == "loadFoamyMissionCCW":
            self.loadFoamyMissionCCW()
        elif args[0] == "loadFoamyFence":
            self.loadFoamyFence()
        elif args[0] == "loadRally":
            self.loadRally()
        elif args[0] == "fixMissionRallyFence":
            self.fixMissionRallyFence()
        elif args[0] == "fixEVERYTHING":
            self.fixEVERYTHING()
        elif args[0] == "check":
            self.check()
        else:
            print(usage)
            return

class FieldCMAC(FieldCheck):
    lc_name = "cmac"
    location = mavutil.location(-35.363261, 149.165230, 584, 353)

class FieldSpringValley(FieldCheck):
    location = mavutil.location(-35.281315, 149.005329, 581, 280)
    lc_name = "springvalley"

class FieldSpringValleyBottom(FieldCheck):
    location = mavutil.location(-35.2824450, 149.0053668, 593, 0)
    lc_name = "springvalleybottom"

class FieldCheckModule(mp_module.MPModule):
    def __init__(self, mpstate):

        super(FieldCheckModule, self).__init__(mpstate,
                                               "FieldCheck",
                                               "FieldCheck Checks",
                                               public=True)

        self.fields = [
            FieldCMAC(),
            FieldSpringValley(),
            FieldSpringValleyBottom(),
        ]

        self.field = None

    def select_field(self, field):
        self.field = field
        self.field.master = self.master
        self.field.mav_param = self.mav_param
        self.field.console = self.console
        self.field.module = self.module
        self.field.x = self
        self.field.select()

    def whinge(self, message):
        self.console.writeln("FC: %s" % (message,))

    def try_select_field(self, loc):
        for field in self.fields:
            if field.close_to(loc):
                self.whinge("Selecting field (%s)" % field.lc_name)
                self.select_field(field)


    def idle_task(self):
        '''run periodic tasks'''
        if self.field is not None:
            self.field.idle_task()

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if self.field is None:
            # attempt to select field automatically based on location
            mtype = m.get_type()
            if mtype == "GPS_RAW_INT":
                if m.fix_type >= 3:
                    lat = m.lat
                    lon = m.lon
                    here = mavutil.location(lat*1e-7, lon*1e-7, 0, 0)
                    self.try_select_field(here)
        if self.field is None:
            return
        self.field.mavlink_packet(m)

def init(mpstate):
    '''initialise module'''
    return FieldCheckModule(mpstate)
