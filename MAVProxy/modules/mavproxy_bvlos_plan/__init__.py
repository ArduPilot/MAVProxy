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

import threading

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

# MIS_OPTIONS bits, AP_Mission::Option
MIS_OPTION_CONTINUE_AFTER_LAND = (1 << 2)
MIS_OPTION_DONT_ZERO_COUNTER = (1 << 3)

# why we would not touch a mission. Adding items to one we do not fully
# understand could change what it flies
REFUSALS = {
    'passes': "bvlos_plan: the mission already passes, nothing to add",
    'nothing': "bvlos_plan: could not work out a return path to add",
    'terrain': "bvlos_plan: not changing the mission while %(n)u points need "
               "terrain that is not available, as the new legs would be "
               "placed from guessed altitudes. Try 'terrain set source SRTM1'",
    'unmodelled': "bvlos_plan: not changing a mission whose flown path is not "
                  "all known, see above. The new legs are placed against that "
                  "path, so they would be in the wrong place",
    'noland': "bvlos_plan: the mission does not end at a landing, so adding "
              "items to the end would change the mission as flown. Not "
              "changing it",
    'continues': "bvlos_plan: a takeoff follows the landing, so with "
                 "MIS_OPTIONS CONTINUE_AFTER_LAND the mission carries on past "
                 "it and appended items would be flown as part of it. Not "
                 "changing it",
}


class MissionItem(object):
    """a snapshot of one mission item.

       The worker must not read the live wploader: a mission download or
       another command can change it while a check runs, and the answer would
       then be about a mission that no longer exists.
    """

    __slots__ = ('seq', 'command', 'frame', 'x', 'y', 'z',
                 'param1', 'param2', 'param3',
                 'target_system', 'target_component')

    def __init__(self, item):
        for name in self.__slots__:
            setattr(self, name, getattr(item, name, 0))


class Job(object):
    '''a check running away from the main loop.

       A check on a long mission takes seconds, and MAVProxy's main loop is
       what services the MAVLink links, so running one inline stops talking to
       the aircraft for that whole time. The work is pure computation on a
       list of mission items, so it runs on a thread and the answer is picked
       up from idle_task.
    '''

    def __init__(self, name, func):
        self.name = name
        self.stamp = None
        self.done = False
        self.value = None
        self.error = None
        self.thread = threading.Thread(target=self.run, args=(func,))
        self.thread.daemon = True
        self.thread.start()

    def run(self, func):
        try:
            self.value = func()
        except Exception as ex:
            self.error = ex
        finally:
            self.done = True


class BvlosPlanModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(BvlosPlanModule, self).__init__(mpstate, "bvlos_plan",
                                              "BVLOS planning")
        self.menu_added_map = False
        self.menu = None
        self.job = None
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
            ('loiter_radius', float, 0.0),
            # if set, the distance either side of the mission path that the
            # return may use, in metres, instead of the turn diameter
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
        '''terrain lookup for a worker, or None if terrain is not available.

           The elevation model is ours alone rather than the terrain module's.
           Sharing that one would mean two threads in the same unlocked tile
           cache, and a "terrain set" could swap it half way through a check.
        '''
        terrain = self.module('terrain')
        if terrain is None:
            return None
        try:
            from MAVProxy.modules.mavproxy_map import mp_elevation
            settings = terrain.terrain_settings
            model = mp_elevation.ElevationModel(database=settings.source,
                                                offline=settings.offline)
        except Exception as ex:
            print("bvlos_plan: no terrain available (%s)" % ex)
            return None

        def lookup(lat, lon):
            return model.GetElevation(lat, lon)

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

    def loiter_radius(self):
        '''the radius a loiter without one of its own will fly, or 0'''
        if self.bvlos_settings.loiter_radius > 0:
            return self.bvlos_settings.loiter_radius
        value = self.get_mav_param('WP_LOITER_RAD', None)
        if value is not None and value != 0:
            return abs(float(value))
        return 0.0

    def mission_options(self):
        '''(continue_after_land, dont_zero_counter) from MIS_OPTIONS'''
        value = self.get_mav_param('MIS_OPTIONS', None)
        if value is None:
            return (False, False)
        bits = int(value)
        return (bits & MIS_OPTION_CONTINUE_AFTER_LAND != 0,
                bits & MIS_OPTION_DONT_ZERO_COUNTER != 0)

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
        return [MissionItem(loader.wp(i)) for i in range(count)]

    def mission_stamp(self):
        '''enough to tell whether the mission changed under a running job'''
        wp = self.module('wp')
        if wp is None:
            return None
        loader = wp.wploader
        return (loader.count(), getattr(loader, 'last_change', None))

    def check_inputs(self):
        '''everything a check needs from the vehicle, gathered on the main
           loop so the worker touches no MAVProxy state'''
        (continue_after_land, dont_zero_counter) = self.mission_options()
        return {
            'cruise': self.cruise_airspeed(),
            'roll': self.roll_limit(),
            'loiter_radius': self.loiter_radius(),
            'width': self.bvlos_settings.return_path_width,
            'granularity': self.bvlos_settings.granularity,
            'terrain_fn': self.terrain_function(),
            'continue_after_land': continue_after_land,
            'dont_zero_counter': dont_zero_counter,
        }

    def run_check(self, items, inputs):
        '''run the return path check on a list of mission items'''
        mission = mission_model.build_mission(
            items, terrain_fn=inputs['terrain_fn'])
        result = return_path.check_return_path(
            mission, inputs['cruise'], inputs['roll'],
            granularity=inputs['granularity'],
            terrain_fn=inputs['terrain_fn'],
            width=inputs['width'],
            loiter_radius=inputs['loiter_radius'],
            dont_zero_counter=inputs['dont_zero_counter'],
            continue_after_land=inputs['continue_after_land'])
        return (mission, result)

    def usable(self, inputs):
        '''True if we know enough to work out a turn radius'''
        if inputs['cruise'] and inputs['roll']:
            return True
        print("bvlos_plan: need the cruise airspeed and bank limit to work "
              "out the turn the aircraft has to fly onto the return. Connect "
              "to a vehicle, or set them with 'bvlos_plan set cruise_airspeed' "
              "and 'bvlos_plan set roll_limit'")
        return False

    def busy(self):
        if self.job is None:
            return False
        print("bvlos_plan: %s is still running" % self.job.name)
        return True

    def cmd_returncheck(self):
        '''check that a DO_RETURN_PATH_START is safe'''
        if self.busy():
            return
        items = self.mission_items()
        if items is None:
            return
        inputs = self.check_inputs()
        if not self.usable(inputs):
            return
        print("Return path check: working through the mission...")
        self.job = Job('the return path check',
                       lambda: ('check',) + self.run_check(items, inputs))
        self.job.stamp = self.mission_stamp()

    def cmd_addreturnpaths(self):
        '''add return paths covering the parts of the mission that fail'''
        if self.busy():
            return
        items = self.mission_items()
        if items is None:
            return
        inputs = self.check_inputs()
        if not self.usable(inputs):
            return
        separation = self.bvlos_settings.return_path_sep
        print("Add Return Paths: working out what to add...")
        self.job = Job('adding return paths',
                       lambda: self.build_paths(items, inputs, separation))
        self.job.stamp = self.mission_stamp()

    def build_paths(self, items, inputs, separation):
        '''work out what to add, and check the mission with it before
           offering it. Runs on the worker thread'''
        (mission, result) = self.run_check(items, inputs)
        if len(result.errors) > 0:
            return ('add', mission, result, None, None, None)
        if result.failed == 0 and result.conclusive():
            return ('add', mission, result, None, None, 'passes')
        if result.terrain_missing:
            return ('add', mission, result, None, None, 'terrain')
        if len(result.unmodelled) > 0:
            return ('add', mission, result, None, None, 'unmodelled')
        if not mission.ends_in_landing():
            return ('add', mission, result, None, None, 'noland')
        if mission.takeoff_after_landing():
            return ('add', mission, result, None, None, 'continues')

        # try a spread of separations and keep the best that breaks
        # nothing, rather than making the operator guess one. Offsetting
        # further moves the new legs away from the ground the mission covers,
        # so more is not better and the useful range is narrow
        if separation > 0:
            candidates = [separation]
        else:
            turn = mission_model.turn_radius(inputs['cruise'], inputs['roll'],
                                             mission.home_amsl)
            candidates = [turn * f for f in (1.0, 1.5, 2.0, 3.0)]
        best = None
        for candidate in candidates:
            added = add_return_paths.build(mission, items, result,
                                           inputs['cruise'], inputs['roll'],
                                           separation=candidate)
            if len(added) == 0:
                continue
            new_items = []
            for new_path in added:
                new_items.extend(new_path.items)
            (_, trial) = self.run_check(items + new_items, inputs)
            broken = len(trial.failed_at - result.failed_at)
            score = (broken > 0, trial.failed, len(trial.errors))
            if best is None or score < best[0]:
                best = (score, added, trial, broken)
        if best is None:
            return ('add', mission, result, None, None, 'nothing')
        (_, added, trial, _) = best
        return ('add', mission, result, added, trial, None)

    def idle_task(self):
        '''add our menu to the map, notice the map going away, and pick up a
           finished check'''
        if self.job is not None and self.job.done:
            job = self.job
            self.job = None
            if job.error is not None:
                print("bvlos_plan: %s failed: %s" % (job.name, job.error))
            else:
                self.finished(job.value, job.stamp)
        if self.menu is None:
            return
        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

    def finished(self, value, stamp):
        '''handle a completed job, back on the main loop'''
        if stamp != self.mission_stamp():
            print("bvlos_plan: the mission changed while that was running, so "
                  "the answer is about the mission as it was. Run it again")
            return
        if value[0] == 'check':
            (_, mission, result) = value
            self.report(result, mission)
            self.highlight(result, mission)
            return
        (_, mission, result, added, trial, refusal) = value
        self.report(result, mission)
        self.highlight(result, mission)
        if refusal is not None:
            print(REFUSALS[refusal] % {'n': result.terrain_missing})
            return
        if added is None:
            return
        if not self.improves(result, trial):
            return
        self.apply(added, trial)

    def improves(self, result, trial):
        '''only take a fix that leaves nothing newly failing.

           Counting failures alone would let a fix trade many small ones for a
           smaller number of much worse ones, which is not a fix.
        '''
        if len(trial.errors) > 0:
            print("bvlos_plan: the return paths this would add give: %s. Not "
                  "changing the mission" % '; '.join(trial.errors))
            return False
        if trial.terrain_missing > result.terrain_missing:
            print("bvlos_plan: the return paths this would add reach ground "
                  "whose terrain is not available, so they cannot be placed. "
                  "Not changing the mission")
            return False
        broken = trial.failed_at - result.failed_at
        if len(broken) > 0:
            print("bvlos_plan: the return paths this would add break %u points "
                  "along the mission that pass now, worst %.0fm. Not changing "
                  "the mission" % (len(broken), trial.worst_deviation))
            self.suggest_sep()
            return False
        if trial.failed >= result.failed:
            print("bvlos_plan: the return paths this would add do not help "
                  "(%u failing points before, %u after). Not changing the "
                  "mission" % (result.failed, trial.failed))
            self.suggest_sep()
            return False
        if trial.failed > 0:
            print("bvlos_plan: note, %u points still fail, worst %.0fm against "
                  "%.0fm allowed" % (trial.failed, trial.worst_deviation,
                                     trial.worst_radius or 0))
            self.suggest_sep()
        return True

    def suggest_sep(self):
        if self.bvlos_settings.return_path_sep > 0:
            print("  a different 'bvlos_plan set return_path_sep' may help")
        else:
            print("  'bvlos_plan set return_path_sep' chooses how far to one "
                  "side the new legs go, which is worth trying")

    def apply(self, added, trial):
        '''put the new items into the mission, on the main loop'''
        wp = self.module('wp')
        if wp is None:
            print("bvlos_plan: the wp module went away, not changing anything")
            return
        loader = wp.wploader
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
        print("  worst case is now %.0fm against %.0fm allowed"
              % (trial.worst_deviation, trial.worst_radius or 0))
        print("  the mission is changed here only, use 'wp save' to keep it "
              "or 'wp list' to go back to the vehicle's copy")

    def report(self, result, mission):
        '''print the outcome of a check'''
        if result.fixed_width is not None:
            print("Return path check: allowing %.0fm either side of the "
                  "mission path, sampled every %.0fm"
                  % (result.fixed_width, self.bvlos_settings.granularity))
        else:
            print("Return path check: allowing the turn diameter, sampled "
                  "every %.0fm" % self.bvlos_settings.granularity)
        if result.terrain_missing:
            print("  INCONCLUSIVE: %u points need terrain that is not "
                  "available, so their altitudes are guesses. Try 'terrain "
                  "set source SRTM1'" % result.terrain_missing)
        for warning in result.warnings:
            print("  note: %s" % warning)
        for reason in result.unmodelled:
            print("  INCONCLUSIVE: %s" % reason)
        autoland = self.get_mav_param('RTL_AUTOLAND', None)
        if autoland is not None and int(autoland) != RTL_AUTOLAND_RETURN_PATH:
            print("  WARNING: RTL_AUTOLAND is %u, so an RTL will not use the "
                  "return path at all (needs %u)"
                  % (int(autoland), RTL_AUTOLAND_RETURN_PATH))
        if result.jump_states > 1:
            print("  note: checked against %u DO_JUMP counter states, as which "
                  "return an RTL finds depends on how many times each loop has "
                  "already run" % result.jump_states)
        for err in result.errors:
            print("  ERROR: %s" % err)
        if len(result.errors) > 0:
            return

        print("  DO_RETURN_PATH_START at %s" %
              ', '.join(str(s) for s in result.return_path_starts))
        if result.worst_radius is not None:
            print("  worst case: %.0fm from the mission path against %.0fm "
                  "allowed, on the leg %u->%u, rejoining at waypoint %u"
                  % (result.worst_deviation, result.worst_radius,
                     result.worst_leg[0], result.worst_leg[1],
                     result.worst_rejoin))
        if result.failed == 0:
            if result.conclusive():
                print("  PASS: all %u points along the mission return within "
                      "%s of the mission path"
                      % (result.checked, self.metric_name(result)))
                print("  (checked every %.0fm along the mission. Between one "
                      "point and the next an RTL can pick a different leg, so "
                      "a finer granularity checks more)"
                      % self.bvlos_settings.granularity)
            else:
                print("  INCONCLUSIVE: none of the %u points checked failed, "
                      "but the mission is not all modelled, see above"
                      % result.checked)
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
        return "the turn diameter"

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
