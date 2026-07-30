#!/usr/bin/env python3
'''
BVLOS planning module.

Assists with planning complex BVLOS missions. Not loaded by default, load it
with "module load bvlos_plan".

Adds a BVLOS submenu to the map's right click menu when the map is loaded.
The first check is Return Path Check, which verifies that a
DO_RETURN_PATH_START is safe: that wherever an RTL is started along the
mission, the return ArduPilot picks stays within a turn radius of the mission
path. See return_path.py.
'''

# AP_FLAKE8_CLEAN

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.mavproxy_bvlos_plan import add_return_paths
from MAVProxy.modules.mavproxy_bvlos_plan import mission_model
from MAVProxy.modules.mavproxy_bvlos_plan import return_path

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu

# our own map layer, so it can be cleared without touching the mission
MAP_LAYER = 'BVLOSReturnPath'

# colour of the highlighted parts of the mission
FAIL_COLOUR = (255, 0, 0)
FAIL_LINEWIDTH = 4

# RTL_AUTOLAND value that makes ArduPlane use a DO_RETURN_PATH_START, see
# RtlAutoland in ArduPlane/defines.h
RTL_AUTOLAND_RETURN_PATH = 4


class BvlosPlanModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(BvlosPlanModule, self).__init__(mpstate, "bvlos_plan",
                                              "BVLOS planning")
        self.menu_added_map = False
        self.menu = None
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu(
                'BVLOS',
                items=[
                    MPMenuItem('Return Path Check', 'Return Path Check',
                               '# bvlos_plan returncheck'),
                    MPMenuItem('Add Return Paths', 'Add Return Paths',
                               '# bvlos_plan addreturnpaths'),
                    MPMenuItem('Clear Highlight', 'Clear Highlight',
                               '# bvlos_plan clear'),
                ])
        self.bvlos_settings = mp_settings.MPSettings([
            ('granularity', float, 50.0),
            # 0 means take it from the vehicle parameters
            ('cruise_airspeed', float, 0.0),
            ('roll_limit', float, 0.0),
            # if set, the distance either side of the mission path that the
            # return may use, in metres, instead of the turn radius
            ('return_path_width', float, 0.0),
            # if set, how far to one side an added return path is put, in
            # metres, instead of the turn radius
            ('return_path_sep', float, 0.0),
        ])
        self.add_command('bvlos_plan', self.cmd_bvlos_plan,
                         "BVLOS planning", ['returncheck', 'addreturnpaths',
                                            'clear',
                                            'set (BVLOSPLANSETTING)'])
        self.add_completion_function('(BVLOSPLANSETTING)',
                                     self.bvlos_settings.completion)

    def usage(self):
        return "Usage: bvlos_plan <returncheck|addreturnpaths|clear|set>"

    def cmd_bvlos_plan(self, args):
        if len(args) == 0:
            print(self.usage())
            return
        if args[0] == "returncheck":
            self.cmd_returncheck()
        elif args[0] == "addreturnpaths":
            self.cmd_addreturnpaths()
        elif args[0] == "clear":
            self.clear_highlight()
        elif args[0] == "set":
            self.bvlos_settings.command(args[1:])
        else:
            print(self.usage())

    def terrain_function(self):
        '''terrain lookup, or None if the terrain module is not loaded.

           The terrain module owns the elevation model and rebuilds it when
           the source is changed, so go through the module each time rather
           than holding onto the model.
        '''
        terrain = self.module('terrain')
        if terrain is None:
            return None

        def lookup(lat, lon):
            return terrain.ElevationModel.GetElevation(lat, lon)

        return lookup

    def cruise_airspeed(self):
        '''cruise airspeed as EAS in m/s, or None'''
        if self.bvlos_settings.cruise_airspeed > 0:
            return self.bvlos_settings.cruise_airspeed
        value = self.get_mav_param('AIRSPEED_CRUISE', None)
        if value is not None and value > 0:
            return float(value)
        # renamed in Plane 4.5, older vehicles hold it in cm/s
        value = self.get_mav_param('TRIM_ARSPD_CM', None)
        if value is not None and value > 0:
            return float(value) * 0.01
        return None

    def roll_limit(self):
        '''bank angle limit in degrees, or None'''
        if self.bvlos_settings.roll_limit > 0:
            return self.bvlos_settings.roll_limit
        value = self.get_mav_param('ROLL_LIMIT_DEG', None)
        if value is not None and value > 0:
            return float(value)
        # renamed in Plane 4.5, older vehicles hold it in centidegrees
        value = self.get_mav_param('LIM_ROLL_CD', None)
        if value is not None and value > 0:
            return float(value) * 0.01
        return None

    def mission_items(self):
        '''the loaded mission, or None with a reason printed'''
        wp = self.module('wp')
        if wp is None:
            print("bvlos_plan: the wp module is not loaded")
            return None
        loader = wp.wploader
        count = loader.count()
        if count == 0:
            print("bvlos_plan: no mission loaded, try 'wp list'")
            return None
        # only refuse if the mission we hold is actually incomplete. Note that
        # a "wp load" leaves loading_waypoints set while it uploads, which is
        # no reason not to check the mission we already have
        expected = getattr(loader, 'expected_count', 0)
        if expected and count < expected:
            print("bvlos_plan: only have %u of %u mission items, still loading"
                  % (count, expected))
            return None
        return [loader.wp(i) for i in range(count)]

    def run_check(self, items):
        '''run the return path check, returning (mission, result, cruise,
           roll) or None with a reason printed'''
        width = self.bvlos_settings.return_path_width
        cruise = self.cruise_airspeed()
        roll = self.roll_limit()
        if width <= 0 and (cruise is None or roll is None):
            print("bvlos_plan: need cruise airspeed and bank limit for the "
                  "turn radius. Connect to a vehicle, set them with "
                  "'bvlos_plan set cruise_airspeed' and 'bvlos_plan set "
                  "roll_limit', or give a fixed distance with 'bvlos_plan set "
                  "return_path_width'")
            return None
        terrain_fn = self.terrain_function()
        mission = mission_model.build_mission(items, terrain_fn=terrain_fn)
        result = return_path.check_return_path(
            mission, cruise, roll,
            granularity=self.bvlos_settings.granularity,
            terrain_fn=terrain_fn, width=width)
        return (mission, result, cruise, roll)

    def cmd_returncheck(self):
        '''check that a DO_RETURN_PATH_START is safe'''
        items = self.mission_items()
        if items is None:
            return
        run = self.run_check(items)
        if run is None:
            return
        (mission, result, cruise, roll) = run
        self.report(result, mission, cruise, roll)
        self.highlight(result, mission)

    def cmd_addreturnpaths(self):
        '''add return paths covering the parts of the mission that fail'''
        items = self.mission_items()
        if items is None:
            return
        # the new legs are offset by the turn radius, never by the return
        # path width, which is only a check tolerance. return_path_sep
        # overrides that offset
        separation = self.bvlos_settings.return_path_sep
        cruise = self.cruise_airspeed()
        roll = self.roll_limit()
        if separation <= 0 and (cruise is None or roll is None):
            print("bvlos_plan: adding return paths needs the cruise airspeed "
                  "and bank limit, as the new legs are offset by the turn "
                  "radius. Connect to a vehicle, set them with 'bvlos_plan "
                  "set cruise_airspeed' and 'bvlos_plan set roll_limit', or "
                  "give the offset directly with 'bvlos_plan set "
                  "return_path_sep'")
            return
        run = self.run_check(items)
        if run is None:
            return
        (mission, result, _, _) = run
        if len(result.errors) > 0:
            for err in result.errors:
                print("bvlos_plan: %s" % err)
            return
        if result.failed == 0:
            print("bvlos_plan: the mission already passes, nothing to add")
            return
        if mission.terrain_missing:
            print("bvlos_plan: not changing the mission while %u items need "
                  "terrain that is not available, as the new legs would be "
                  "placed from guessed altitudes. Try 'terrain set source "
                  "SRTM1'" % mission.terrain_missing)
            return
        if not mission.ends_in_landing():
            # new navigation items after a mission that does not end at a
            # landing would be flown as part of the mission
            print("bvlos_plan: the mission does not end at a landing, so "
                  "adding items to the end would change the mission as "
                  "flown. Not changing it")
            return

        added = add_return_paths.build(mission, items, result, cruise, roll,
                                       separation=separation)
        if len(added) == 0:
            print("bvlos_plan: could not work out a return path to add")
            return

        # check what we are about to do before doing it, so a fix that does
        # not help cannot be left behind in the mission
        new_items = []
        for new_path in added:
            new_items.extend(new_path.items)
        trial = self.run_check(items + new_items)
        if trial is None:
            return
        (trial_mission, trial_result, _, _) = trial
        if len(trial_result.errors) > 0 or trial_result.failed >= result.failed:
            print("bvlos_plan: the return paths this would add do not improve "
                  "the mission (%u failing points before, %u after), so it has "
                  "not been changed" % (result.failed, trial_result.failed))
            for err in trial_result.errors:
                print("  would give: %s" % err)
            if trial_result.failed >= result.failed and separation > 0:
                print("  a smaller 'bvlos_plan set return_path_sep' may help")
            return

        loader = self.module('wp').wploader
        count = 0
        for new_path in added:
            for item in new_path.items:
                loader.add(item)
                count += 1
        # keep the loader self consistent for anything watching it
        loader.expected_count = loader.count()
        print("Added %u return path(s), %u mission items:" % (len(added), count))
        for new_path in added:
            source = ("the return_path_sep setting" if new_path.from_setting
                      else "the turn radius")
            print("  mission %u..%u now has a return path offset %.0fm to one "
                  "side, %s, rejoining the existing return path at waypoint %u"
                  % (new_path.from_seq, new_path.to_seq, new_path.separation,
                     source, new_path.rejoin_seq))
        print("  the mission is changed here only, use 'wp save' to keep it "
              "or 'wp list' to go back to the vehicle's copy")

        # show what the mission looks like now
        items = [loader.wp(i) for i in range(loader.count())]
        run = self.run_check(items)
        if run is None:
            return
        (mission, result, cruise, roll) = run
        self.report(result, mission, cruise, roll)
        self.highlight(result, mission)

    def report(self, result, mission, cruise, roll):
        '''print the outcome of a check'''
        if result.fixed_width is not None:
            print("Return path check: allowing %.0fm either side of the "
                  "mission path, sampled every %.0fm"
                  % (result.fixed_width, self.bvlos_settings.granularity))
        else:
            print("Return path check: turn radius from cruise %.1fm/s and "
                  "bank limit %.0fdeg, sampled every %.0fm"
                  % (cruise, roll, self.bvlos_settings.granularity))
        if mission.terrain_missing:
            print("  WARNING: %u mission items need terrain that is not "
                  "available, so their altitudes are approximate. Try "
                  "'terrain set source SRTM1'" % mission.terrain_missing)
        autoland = self.get_mav_param('RTL_AUTOLAND', None)
        if autoland is not None and int(autoland) != RTL_AUTOLAND_RETURN_PATH:
            print("  WARNING: RTL_AUTOLAND is %u, so an RTL will not use the "
                  "return path at all (needs %u)"
                  % (int(autoland), RTL_AUTOLAND_RETURN_PATH))
        for err in result.errors:
            print("  ERROR: %s" % err)
        if len(result.errors) > 0:
            return

        print("  DO_RETURN_PATH_START at %s" %
              ', '.join(str(s) for s in result.return_path_starts))
        if result.worst_radius is not None:
            allowed = "%.0fm allowed" % result.worst_radius
            if result.fixed_width is None:
                allowed = "%.0fm turn radius" % result.worst_radius
            print("  worst case: %.0fm from the mission path against %s, on "
                  "the leg %u->%u, rejoining at waypoint %u"
                  % (result.worst_deviation, allowed,
                     result.worst_leg[0], result.worst_leg[1],
                     result.worst_rejoin))
        if result.failed == 0:
            print("  PASS: all %u points along the mission return within %s "
                  "of the mission path" % (result.checked, self.metric_name(result)))
            return
        print("  FAIL: %u of %u points along the mission would return outside "
              "%s of the mission path"
              % (result.failed, result.checked, self.metric_name(result)))
        for span in result.spans:
            legs = sorted(span.legs)
            print("    legs %s: worst %.0fm"
                  % (', '.join("%u->%u" % leg for leg in legs), span.worst))

    def metric_name(self, result):
        '''how the allowed distance was arrived at, for the report'''
        if result.fixed_width is not None:
            return "the %.0fm return path width" % result.fixed_width
        return "a turn radius"

    def maps(self):
        '''every loaded map instance'''
        return [m for m in self.module_matching('map*')]

    def clear_highlight(self):
        '''remove our own map layer, leaving everything else alone'''
        if not mp_util.has_wxpython:
            return
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        for m in self.maps():
            m.map.add_object(mp_slipmap.SlipClearLayer(MAP_LAYER))

    def highlight(self, result, mission):
        '''draw the failing parts of the mission path on the map'''
        if not mp_util.has_wxpython:
            return
        maps = self.maps()
        if len(maps) == 0:
            if result.failed:
                print("  (load the map module to see the failing parts "
                      "highlighted)")
            return
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        self.clear_highlight()
        for (i, span) in enumerate(result.spans):
            points = [mission.projector.unproject(x, y) for (x, y) in span.points]
            for m in maps:
                if len(points) < 2:
                    # one failing sample has no line to draw, so mark it
                    radius = max(self.bvlos_settings.granularity * 0.5, 25.0)
                    m.map.add_object(mp_slipmap.SlipCircle(
                        'bvlos_return_fail_%u' % i, MAP_LAYER, points[0],
                        radius, FAIL_COLOUR, linewidth=FAIL_LINEWIDTH))
                    continue
                m.map.add_object(mp_slipmap.SlipPolygon(
                    'bvlos_return_fail_%u' % i, points,
                    layer=MAP_LAYER, linewidth=FAIL_LINEWIDTH,
                    colour=FAIL_COLOUR, showcircles=False))

    def idle_task(self):
        '''add our menu to the map, and notice the map going away'''
        if self.menu is None:
            return
        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

    def unload(self):
        '''unload module'''
        self.clear_highlight()
        if self.menu is not None and self.module('map') is not None:
            self.module('map').remove_menu(self.menu)
        self.menu_added_map = False
        self.remove_command('bvlos_plan')
        super(BvlosPlanModule, self).unload()


def init(mpstate):
    '''initialise module'''
    return BvlosPlanModule(mpstate)
