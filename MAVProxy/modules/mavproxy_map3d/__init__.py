'''
3D map module: draped satellite imagery over ArduPilot quantized-mesh terrain,
rendered natively with VTK. Shows the same elements as the 2D map (flight path,
mission, fence, rally, vehicle) for both live telemetry and log review.

Andrew Tridgell / CanberraUAV

AP_FLAKE8_CLEAN
'''

import math
import queue
import threading
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map3d.map3d import (
    Map3D, MissionItem, MISSION_LABEL_SIZE, MISSION_LABEL_SIZES,
    MISSION_STYLES, missing_packages, missing_packages_message)

# fence colours as the 2D map's PolyFence layer uses them (OpenCV BGR)
FENCE_INCLUSION_BGR = (0, 255, 0)
FENCE_HOME_INCLUSION_BGR = (0, 255, 96)
FENCE_EXCLUSION_BGR = (255, 0, 0)
FENCE_RETURN_BGR = (255, 127, 127)
FENCE_RETURN_RADIUS = 10.0
# a plane's mission is flown again when the way it points on the ground turns
# this far, but not more often than this
TAKEOFF_HEADING_CHANGE = 5.0
TAKEOFF_HEADING_REDRAW = 2.0
# nor for home moving less than this, in metres, which it does by the GPS
# wandering while ArduPlane keeps setting it before arming
HOME_MOVE_CHANGE = 10.0
# a VTOL landing approach is flown into the wind, whose direction the
# vehicle's estimate wanders about: a mission landing so is flown again for
# a change of this many degrees
APPROACH_CHANGE = 10.0


def bgr_to_rgb(bgr, default=(1.0, 0.0, 1.0)):
    '''SlipMap/OpenCV colours are BGR; VTK wants RGB floats'''
    try:
        return tuple(max(0, min(255, int(bgr[i]))) / 255.0 for i in (2, 1, 0))
    except (IndexError, TypeError, ValueError):
        return default


def make_settings():
    '''the module's settings.  The view's own controls set some of them:
    showlabels, missionpath, follow and the render settings'''
    return mp_settings.MPSettings([
        ('service', str, 'MicrosoftSat'),
        ('zexag', float, 1.0),
        ('debug', bool, False),
        mp_settings.MPSetting('fpvfov', float, 90.0, range=(20.0, 150.0)),
        mp_settings.MPSetting('terrainbrightness', float, 1.25,
                              range=(0.25, 2.0)),
        ('terrainshading', bool, True),
        ('terrainwireframe', bool, False),
        ('showdirection', bool, True),
        ('showlabels', bool, False),
        mp_settings.MPSetting('labelsize', int, MISSION_LABEL_SIZE,
                              range=MISSION_LABEL_SIZES),
        # flown, geometry or plain: see Map3D.set_mission_style()
        mp_settings.MPSetting('missionpath', str, MISSION_STYLES[0],
                              choice=MISSION_STYLES),
    ])


class Map3DModule(mp_module.MPModule):
    # the EKF origin's altitude, which a rally point may be measured from,
    # until the vehicle says where it is
    origin_amsl = None

    def __init__(self, mpstate):
        # Do not register as a public "map*" module. ADS-B, AIS, KML and other
        # modules use that wildcard for the 2D SlipMap API (add_object,
        # remove_object, set_position), which Map3D intentionally does not
        # implement.
        super(Map3DModule, self).__init__(mpstate, "map3d", "3D map display")
        self.map3d_settings = make_settings()
        self.add_command('map3d', self.cmd_map3d,
                         "3D map control", ['<start|stop|follow|nofollow|center>',
                                            'set (MAP3DSETTING)'])
        self.add_completion_function('(MAP3DSETTING)',
                                     self.map3d_settings.completion)
        self.map = None
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.rally_change_time = 0
        self.last_vehicle_send = 0
        self.last_global_position = 0
        self.last_attitude = (0.0, 0.0, 0.0)
        self.home_amsl = None
        self.home_position = None
        # whether the vehicle is armed, and the way it last pointed while it
        # was not.  A fixed-wing takeoff holds the ground course the vehicle
        # has once it gets moving, which is not known until it does: this
        # is the best there is before then, and wrong wherever a crosswind,
        # a turn on the ground or a compass error has it moving some other
        # way
        self.armed = False
        self.ground_heading = None
        self.ground_heading_changed = False
        self.ground_heading_redrawn = 0
        self.reset_flown_track()
        self.icon_type = None
        self.follow = True
        self.kml_change_state = None
        self.terrain_lock = threading.Lock()
        self.terrain_lookups = queue.Queue()
        self.terrain_requested = set()
        self.terrain_running = False
        self.terrain_resolved = False
        self.start_map()

    # ------------------------------------------------------------------ command
    def cmd_map3d(self, args):
        if len(args) == 0:
            print("usage: map3d <start|stop|follow|nofollow|center|set>")
            return
        cmd = args[0]
        if cmd == "start":
            self.start_map()
        elif cmd == "stop":
            self.stop_map()
        elif cmd == "follow":
            self.follow = True
            if self.map:
                self.map.set_follow(True)
        elif cmd == "nofollow":
            self.follow = False
            if self.map:
                self.map.set_follow(False)
        elif cmd == "center":
            if self.map:
                self.map.center_on_vehicle()
        elif cmd == "set":
            self.map3d_settings.command(args[1:])
            if (len(args) > 1 and args[1] == 'missionpath' and
                    self.map3d_settings.missionpath == 'flown'):
                self.flown_asked = True
            if self.map is not None and self.map.is_alive():
                self.map.set_mission_arrows(self.map3d_settings.showdirection)
                self.map.set_mission_labels(self.map3d_settings.showlabels)
                self.map.set_mission_label_size(self.map3d_settings.labelsize)
                self.map.set_mission_style(self.map3d_settings.missionpath)
                # the path flown is only worked out while it is drawn
                self.send_mission()
                self.send_render_settings()
        else:
            print("unknown map3d command: %s" % cmd)

    def start_map(self):
        if self.map is not None and self.map.is_alive():
            print("map3d already running")
            return
        missing = missing_packages()
        if missing:
            print(missing_packages_message(missing))
            return
        self.map = Map3D(title="MAVProxy 3D Map",
                         service=self.map3d_settings.service,
                         zexag=self.map3d_settings.zexag,
                         debug=self.map3d_settings.debug,
                         fpvfov=self.map3d_settings.fpvfov,
                         terrain_brightness=self.map3d_settings.terrainbrightness,
                         terrain_shading=self.map3d_settings.terrainshading,
                         terrain_wireframe=self.map3d_settings.terrainwireframe,
                         follow=self.follow)
        # push whatever we already know
        self.map.set_mission_arrows(self.map3d_settings.showdirection)
        self.map.set_mission_labels(self.map3d_settings.showlabels)
        self.map.set_mission_label_size(self.map3d_settings.labelsize)
        self.map.set_mission_style(self.map3d_settings.missionpath)
        self.send_mission()
        self.send_fence()
        self.send_rally()
        self.send_cached_state()
        self.send_kml()

    def send_render_settings(self):
        self.map.set_fpv_fov(self.map3d_settings.fpvfov)
        self.map.set_render_settings(
            self.map3d_settings.terrainbrightness,
            self.map3d_settings.terrainshading,
            self.map3d_settings.terrainwireframe)

    def stop_map(self):
        if self.map is not None:
            self.map.close()
            self.map = None

    # --------------------------------------------------------------- data feeds
    def send_cached_state(self):
        '''Seed an auto-started map from MAVProxy's latest known state.'''
        try:
            messages = self.master.messages
        except Exception:
            return

        # a restarted viewer child knows nothing, so push the type again
        heartbeat = messages.get('HEARTBEAT')
        if heartbeat is not None:
            name = mp_util.vehicle_type_name(heartbeat.type)
            if name is not None:
                self.icon_type = name
        if self.icon_type is not None:
            self.map.set_vehicle_type(self.icon_type)

        attitude = messages.get('ATTITUDE')
        if attitude is not None:
            self.last_attitude = (attitude.roll, attitude.pitch, attitude.yaw)
        else:
            # DataFlash ATT angles are in degrees.
            attitude = messages.get('ATT')
            if attitude is not None:
                self.last_attitude = tuple(math.radians(v) for v in
                                           (attitude.Roll, attitude.Pitch,
                                            attitude.Yaw))

        origin = messages.get('GPS_GLOBAL_ORIGIN') or messages.get('ORGN')
        if origin is not None:
            self.mavlink_packet(origin, force=True)

        home = messages.get('HOME_POSITION')
        if home is not None:
            self.home_amsl = home.altitude * 1.0e-3
            self.set_home_position(home.latitude * 1.0e-7,
                                   home.longitude * 1.0e-7)

        # Prefer the estimator position, then DataFlash POS, then raw GPS.
        position = (messages.get('GLOBAL_POSITION_INT') or
                    messages.get('POS') or
                    messages.get('GPS_RAW_INT'))
        if position is not None:
            self.mavlink_packet(position, force=True)

    def send_vehicle_position(self, lat, lon, alt, home_amsl=None,
                              attitude=None, force=False):
        if lat == 0 and lon == 0:
            return
        now = time.time()
        if not force and now - self.last_vehicle_send < 0.1:
            return
        self.last_vehicle_send = now

        if not self.map.origin_set():
            self.map.set_origin(lat, lon, alt)
            if home_amsl is None:
                home_amsl = self.home_amsl
            if home_amsl is None:
                home_amsl = alt
            self.home_amsl = home_amsl
            self.map.set_home(home_amsl)
            self.send_mission()
            self.send_fence()
            self.send_rally()

        if attitude is None:
            attitude = self.last_attitude
        self.map.set_vehicle(lat, lon, alt, *attitude)

    def terrain_alt(self, lat, lon):
        '''terrain elevation (m AMSL) at lat/lon. Prefer the quantized mesh (same
        source rendered in 3D); fall back to the terrain module's SRTM model.

        This runs on the main loop, so it never fetches: an uncached mesh tile is
        handed to a background thread and the mission is re-sent once it lands.
        The SRTM fallback is already non-blocking (timeout=0 returns None until
        the terrain module has downloaded the tile).
        '''
        try:
            from MAVProxy.modules.mavproxy_map3d.terrain import sample_terrain
            alt = sample_terrain(lat, lon, cache_only=True)
            if alt is not None:
                return alt
            self.request_terrain_lookup(lat, lon)
        except Exception:
            pass
        tm = self.module('terrain')
        if tm is not None and getattr(tm, 'ElevationModel', None) is not None:
            return tm.ElevationModel.GetElevation(lat, lon)
        return None

    def request_terrain_lookup(self, lat, lon):
        '''queue an uncached terrain sample for the background resolver'''
        with self.terrain_lock:
            if (lat, lon) in self.terrain_requested:
                return
            self.terrain_requested.add((lat, lon))
            self.terrain_lookups.put((lat, lon))
            if self.terrain_running:
                return
            self.terrain_running = True
        threading.Thread(target=self.terrain_resolver, daemon=True).start()

    def terrain_resolver(self):
        '''background: fetch/decode mesh tiles so the main loop never blocks'''
        from MAVProxy.modules.mavproxy_map3d.terrain import sample_terrain
        while True:
            with self.terrain_lock:
                try:
                    (lat, lon) = self.terrain_lookups.get_nowait()
                except queue.Empty:
                    self.terrain_running = False
                    return
            try:
                resolved = sample_terrain(lat, lon) is not None
            except Exception:
                resolved = False
            # drop it either way: a success is cached from here on, and a
            # failure must stay retryable rather than blocking the point forever
            with self.terrain_lock:
                self.terrain_requested.discard((lat, lon))
            if resolved:
                self.terrain_resolved = True

    def mission_home(self, wploader):
        '''lat/lon a positionless takeoff climbs from, or None'''
        if self.home_position is not None:
            return self.home_position
        try:
            home = wploader.wp(0)
        except Exception:
            return None
        if home is None or (home.x == 0 and home.y == 0):
            return None
        return (home.x, home.y)

    def item_amsl(self, alt, frame):
        '''the AMSL altitude of a mission item, or None if we cannot tell.

        The viewer resolves the frames itself; this is only so that the climb
        between two items can be measured with both of them in the one frame
        '''
        if frame in (0, 5):          # already AMSL
            return alt
        if frame in (10, 11):
            # still above terrain: send_mission() turns these into AMSL once
            # it has the terrain height, and until then nobody knows
            return None
        if self.home_amsl is None:
            # relative to a home we have not been told about yet
            return None
        return self.home_amsl + alt

    def send_mission(self):
        if self.map is None:
            return
        try:
            wploader = self.module('wp').wploader
        except Exception:
            return
        items = []
        # every item after home, for flying the mission through, and which
        # of their altitudes are absolute rather than moving with home: True
        # for one of its own, False for one above home, and None for an item
        # whose altitude neither moves nor stays, having none to fly at
        flown = []
        fixed_alt = []
        default_radius = self.default_circle_radius()
        previous = None
        for w in wploader.wpoints:
            frame = getattr(w, 'frame', 0)
            (lat, lon) = (w.x, w.y)
            if w.seq != 0 and (lat == 0 and lon == 0 and
                               w.command not in mp_util.TAKEOFF_COMMANDS):
                from MAVProxy.modules.lib import plane_track
                amsl = plane_track.positionless_amsl(w.z, frame,
                                                     self.home_amsl)
                flown.append((w.command, 0.0, 0.0, amsl,
                              (w.param1, w.param2, w.param3, w.param4)))
                if (amsl is None or
                        not plane_track.is_navigation_command(w.command)):
                    # flown at the altitude the aircraft is at, or not
                    # flown at all
                    fixed_alt.append(None)
                else:
                    fixed_alt.append(frame in (0, 5))
            if lat == 0 and lon == 0 and w.command in mp_util.TAKEOFF_COMMANDS:
                # draw the climb from home, otherwise the takeoff altitude is
                # dropped and the mission appears to start at the first waypoint
                home = self.mission_home(wploader)
                if home is None:
                    continue
                (lat, lon) = home
            if lat == 0 and lon == 0:
                continue
            z = w.z
            if frame in (10, 11):    # terrain-relative -> resolve to AMSL
                terr = self.terrain_alt(lat, lon)
                if terr is not None:
                    z = terr + w.z
                    frame = 0
            # the items are drawn in whatever frames they carry, so resolve
            # the altitude before measuring anything against the item before
            amsl = self.item_amsl(z, frame)
            params = (w.param1, w.param2, w.param3, w.param4)
            circle_radius = mp_util.mission_circle_radius(
                w.command, params, default_radius, self.vehicle_type)
            # items which say how many turns they fly say so themselves;
            # None is an item which circles until something else stops it
            circle_turns = mp_util.mission_circle_turns(w.command, params)
            if w.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
                # this one circles until it reaches its altitude, so what is
                # left to climb on arrival decides how many turns to draw
                approach = None
                alt_change = None
                if previous is not None:
                    approach = mp_util.gps_distance(previous[0], previous[1],
                                                    lat, lon)
                    if amsl is not None and previous[2] is not None:
                        alt_change = amsl - previous[2]
                circle_turns = mp_util.loiter_to_alt_turns(
                    circle_radius, alt_change, self.mav_param, approach)
            exit_converge = None
            if (circle_radius is not None and
                    mp_util.mission_crosstracks_from_centre(w.command, params)):
                # the next leg is flown against a track from the loiter
                # centre, so the vehicle pulls back onto it after leaving
                exit_converge = mp_util.vehicle_track_convergence(self.mav_param)
            items.append(MissionItem(lat, lon, z, frame, w.command, w.seq,
                                     w.param1, circle_radius, circle_turns,
                                     exit_converge))
            if w.seq != 0:
                flown.append((w.command, lat, lon, amsl, params))
                # a terrain item resolved above has frame 0 by now: its
                # altitude is where it is, whatever home does
                fixed_alt.append(frame in (0, 5))
            previous = (lat, lon, amsl)
        self.mission_sent = items
        self.map.set_mission(
            items, self.flown_track(wploader, flown, fixed_alt))

    def reset_flown_track(self):
        '''forget any plane mission flown.  Flying one can take a second or
        more, so it is done on a thread of its own rather than the main one:
        the mission is drawn at once, and again with the path flown when the
        thread has worked it out'''
        self.track_lock = threading.Lock()
        # the last path worked out, as (key, track, home), so an unchanged
        # mission is not flown again
        self.plane_track = (None, None, None)
        # for the thread: the latest mission to fly, as (key, home, items,
        # params, heading, rally), and the thread while it is running
        self.track_request = None
        self.track_thread = None
        # from the thread, for the idle task to draw: (key, track, home)
        self.track_result = None
        # (key, flown_from, home, items, alt_moves) of the mission on screen
        # while it waits for its path: the home it is being flown from, the
        # home and items it has now, and whether any of its altitudes move
        # with home
        self.track_wanted = None
        self.mission_sent = None
        # the mission last found to have no path flown, so it is said once
        self.unflown_key = None
        # whether the path flown has been asked for by name since it was
        # last said it could not be drawn for this vehicle
        self.flown_asked = False
        # the course of a VTOL landing approach, into the wind the vehicle
        # last said it estimates, to APPROACH_CHANGE; and whether the path
        # on screen was flown with it
        self.approach = None
        self.approach_used = False

    @staticmethod
    def same_home(home, other):
        '''whether two homes are near enough to fly a mission from alike'''
        return (other is not None and
                mp_util.gps_distance(home[0], home[1],
                                     other[0], other[1]) < HOME_MOVE_CHANGE and
                abs(home[2] - other[2]) < HOME_MOVE_CHANGE)

    def flown_track(self, wploader, items, fixed_alt):
        '''the path a plane flies the mission along, where it has already
        been worked out; otherwise None, and the thread is asked to work it
        out.  None too for any other vehicle, where the mission cannot be
        flown through, or where the map is not drawing the path flown'''
        if self.map3d_settings.missionpath != 'flown':
            self.track_wanted = None
            return None
        if self.vehicle_type != 'plane':
            if self.flown_asked and self.vehicle_type is not None:
                # asked for by name, rather than left as it starts
                print("map3d: only a plane's mission can be drawn as the "
                      "path flown; drawing its geometry")
                self.flown_asked = False
            self.track_wanted = None
            return None
        home = self.mission_home(wploader)
        if home is None or self.home_amsl is None:
            self.track_wanted = None
            return None
        from MAVProxy.modules.lib import plane_track
        home = (home[0], home[1], self.home_amsl)
        # copied here, since the thread must not read them as they change
        params = dict((name, mp_util.param_value(self.mav_param, name))
                      for (names, _) in plane_track.PARAMETERS.values()
                      for (name, _) in names)

        # the mission as it stands relative to home, so the same mission from
        # a home which has only wandered a little is still the same mission.
        # An altitude of its own stands as it is, since home does not move it
        def keyed_alt(amsl, fixed):
            if amsl is None or fixed is None:
                return None
            return round(amsl if fixed else amsl - home[2], 2)
        relative = tuple(
            (command,
             'home' if (lat, lon) == home[:2] else (lat, lon),
             keyed_alt(amsl, fixed),
             tuple(params_of_item))
            for ((command, lat, lon, amsl, params_of_item), fixed)
            in zip(items, fixed_alt))
        points = self.rally_points(home)
        rally = [point[:3] for point in points]
        moves = [not fixed for fixed in fixed_alt if fixed is not None]
        moves += [not fixed for (_, _, _, fixed) in points]
        # where some of it moves with home and the rest does not, the path
        # cannot be moved to fit a home which has: it is moved up or down
        # with home all the same while home is near enough to be the same
        # one, which puts what does not move out by no more than that, and
        # flown again beyond
        key = (relative, tuple(sorted(params.items())), self.ground_heading,
               tuple((lat, lon, keyed_alt(amsl, fixed))
                     for (lat, lon, amsl, fixed) in points))
        # only a mission which lands on a VTOL approach is flown again as
        # the wind changes
        approach = None
        self.approach_used = plane_track.uses_vtol_approach(items, params)
        if self.approach_used:
            approach = self.approach
            key += (approach,)
        (cached_key, cached_track, cached_home) = self.plane_track
        if cached_key == key and self.same_home(home, cached_home):
            self.track_wanted = None
            return self.moved_home(cached_track, cached_home, home, items,
                                   any(moves))
        wanted = self.track_wanted
        if wanted is not None and wanted[0] == key and self.same_home(home, wanted[1]):
            # already being flown, from a home near enough: drawn, when it
            # has been, from where home is now
            self.track_wanted = (key, wanted[1], home, items, any(moves))
            return None
        self.track_wanted = (key, home, home, items, any(moves))
        with self.track_lock:
            self.track_request = (key, home, list(items), params,
                                  self.ground_heading, rally, approach)
            if self.track_thread is None:
                self.track_thread = threading.Thread(target=self.fly_tracks,
                                                     daemon=True)
                self.track_thread.start()
        return None

    def rally_points(self, home):
        '''the vehicle's rally points, as (lat, lon, amsl, fixed_alt), for a
        return to launch to go to.  fixed_alt is a point whose altitude is
        its own rather than one which moves with home'''
        from MAVProxy.modules.lib import plane_track
        try:
            loader = self.module('rally').rallyloader
            points = [loader.rally_point(i) for i in range(loader.rally_count())]
        except Exception:
            return []
        out = []
        for r in points:
            flags = getattr(r, 'flags', 0)
            frame = plane_track.rally_alt_frame(flags)
            (lat, lon) = (r.lat * 1.0e-7, r.lng * 1.0e-7)
            terrain = None
            if frame == plane_track.RALLY_ALT_ABOVE_TERRAIN:
                terrain = self.terrain_alt(lat, lon)
            # the EKF origin is where the vehicle first had a position, which
            # is usually home and near enough to it until it says otherwise
            origin = self.origin_amsl
            fixed = plane_track.rally_alt_is_fixed(flags)
            if frame == plane_track.RALLY_ALT_ABOVE_ORIGIN and origin is None:
                (origin, fixed) = (home[2], False)
            out.append((lat, lon,
                        plane_track.rally_amsl(r.alt, flags, home[2],
                                               origin, terrain),
                        fixed))
        return out

    @staticmethod
    def moved_home(track, flown_home, home, items, alt_moves=True):
        '''a path flown from flown_home, moved to start from home, which is
        near enough not to fly it again.  Its altitudes move with home where
        the mission's do, and its start moves with home too, less and less
        along the way to the first item with a position of its own, which
        does not move'''
        if track is None or flown_home == home:
            return track
        dlat = home[0] - flown_home[0]
        dlon = (home[1] - flown_home[1] + 180.0) % 360.0 - 180.0
        dalt = home[2] - flown_home[2] if alt_moves else 0.0
        reach = 0.0
        for (command, lat, lon, amsl, params) in items:
            if (lat, lon) != (0.0, 0.0) and (lat, lon) != home[:2]:
                reach = mp_util.gps_distance(flown_home[0], flown_home[1],
                                             lat, lon)
                break
        moved = []
        travelled = 0.0
        previous = None
        for (lat, lon, amsl) in track:
            if previous is not None:
                travelled += mp_util.gps_distance(previous[0], previous[1],
                                                  lat, lon)
            previous = (lat, lon)
            weight = max(0.0, 1.0 - travelled / reach) if reach > 0 else 0.0
            moved.append((lat + dlat * weight, lon + dlon * weight,
                          amsl + dalt))
        return moved

    def fly_tracks(self):
        '''the thread flying plane missions: the latest asked for, until there
        is none left to fly'''
        from MAVProxy.modules.lib import plane_track
        while True:
            with self.track_lock:
                request = self.track_request
                self.track_request = None
                if request is None:
                    self.track_thread = None
                    return
            (key, home, items, params, heading, rally, approach) = request
            try:
                track = plane_track.mission_track(home, items, params,
                                                  heading, rally=rally,
                                                  approach=approach)
            except Exception as ex:
                # drawn from its items instead; the thread carries on
                print("map3d: could not fly the mission: %s" % ex)
                track = None
            with self.track_lock:
                self.track_result = (key, track, home)

    def draw_flown_track(self):
        '''draw the path the thread has flown, if it is for the mission on
        screen.  Called from the idle task, on the main thread'''
        with self.track_lock:
            result = self.track_result
            self.track_result = None
        if result is None:
            return
        self.plane_track = result
        (key, track, home) = result
        wanted = self.track_wanted
        if wanted is None or wanted[0] != key or home != wanted[1]:
            # another mission since, which the thread has yet to fly
            return
        self.track_wanted = None
        if track is None and key != self.unflown_key:
            self.unflown_key = key
            print("map3d: could not work out the path this mission is flown "
                  "along -- it may be too long, never finish an item, or use "
                  "a command which cannot be flown here; drawing its geometry")
        if self.map is not None and self.mission_sent is not None:
            self.map.set_mission(self.mission_sent,
                                 self.moved_home(track, home, wanted[2],
                                                 wanted[3], wanted[4]))

    def set_icon_type(self, name):
        '''vehicle type changed: the viewer picks its icon from it. Not named
        vehicle_type: MPModule has a read-only property of that name, and it is
        a coarser mapping than the icons need (no heli or boat of its own)'''
        if name is None or name == self.icon_type:
            return
        self.icon_type = name
        self.map.set_vehicle_type(name)

    def set_home_position(self, lat, lon):
        '''home moved: around-home fence circles are centred on it, and a
        positionless takeoff is drawn there'''
        if (lat, lon) == self.home_position:
            return
        self.home_position = (lat, lon)
        self.send_fence()
        self.send_mission()

    @staticmethod
    def _fence_latlon(item):
        '''fence items are MISSION_ITEM_INT (1e7 scaled); MISSION_ITEM is degrees'''
        (lat, lon) = (item.x, item.y)
        if item.get_type() == 'MISSION_ITEM_INT':
            lat *= 1.0e-7
            lon *= 1.0e-7
        return (lat, lon)

    def send_fence(self):
        '''Mirror the 2D map's PolyFence layer: each inclusion/exclusion polygon
        and circle is its own shape, not one flattened ring.'''
        if self.map is None:
            return
        fence_mod = self.module('fence')
        if fence_mod is None:
            return
        shapes = []
        try:
            for (polygons, bgr) in ((fence_mod.inclusion_polygons(),
                                     FENCE_INCLUSION_BGR),
                                    (fence_mod.exclusion_polygons(),
                                     FENCE_EXCLUSION_BGR)):
                for polygon in polygons:
                    points = [self._fence_latlon(p) for p in polygon]
                    if len(points) >= 2:
                        shapes.append(('polygon', points, bgr_to_rgb(bgr)))

            for (circles, bgr) in ((fence_mod.inclusion_circles(),
                                    FENCE_INCLUSION_BGR),
                                   (fence_mod.exclusion_circles(),
                                    FENCE_EXCLUSION_BGR)):
                for circle in circles:
                    shapes.append(('circle', self._fence_latlon(circle),
                                   circle.param1, bgr_to_rgb(bgr)))

            # home circles are centred on home, not on their own lat/lon
            home_circles = fence_mod.home_inclusion_circles()
            if home_circles and self.home_position is not None:
                for circle in home_circles:
                    shapes.append(('circle', self.home_position, circle.param1,
                                   bgr_to_rgb(FENCE_HOME_INCLUSION_BGR)))

            returnpoint = fence_mod.returnpoint()
            if returnpoint is not None:
                shapes.append(('circle', self._fence_latlon(returnpoint),
                               FENCE_RETURN_RADIUS,
                               bgr_to_rgb(FENCE_RETURN_BGR)))
        except Exception:
            return
        self.map.set_fence(shapes)

    def send_rally(self):
        '''draw the rally points at the altitude a return to launch goes to
        each at, where that can be worked out, and above home otherwise'''
        if self.map is None:
            return
        try:
            loader = self.module('rally').rallyloader
            raw = [loader.rally_point(i) for i in range(loader.rally_count())]
        except Exception:
            return
        resolved = self.rally_points((0.0, 0.0, self.home_amsl))
        self.map.set_rally([(lat, lon, amsl, r.alt) for
                            ((lat, lon, amsl, _), r) in zip(resolved, raw)])

    @staticmethod
    def _kml_state(kml_mod):
        if kml_mod is None:
            return None
        return (id(kml_mod), getattr(kml_mod, 'last_change', 0))

    def send_kml(self, kml_mod=None):
        '''Mirror the visible KML polylines, using their 2D map colours.'''
        if self.map is None:
            return
        if kml_mod is None:
            kml_mod = self.module('kmlread')
        features = []
        if kml_mod is not None:
            for layer, objects in kml_mod.map_objects.items():
                for key, obj in objects.items():
                    points = getattr(obj, 'points', None)
                    if points is None or getattr(obj, 'hidden', False):
                        continue
                    try:
                        points = [(float(p[0]), float(p[1]))
                                  for p in points]
                    except (IndexError, TypeError, ValueError):
                        continue
                    if len(points) < 2:
                        continue
                    rgb = bgr_to_rgb(getattr(obj, 'colour', (255, 0, 255)))
                    width = max(1.0, float(getattr(obj, 'linewidth', 2.0)))
                    features.append(("%s:%s" % (layer, key), points,
                                     rgb, width))
        features.sort(key=lambda feature: feature[0])
        self.map.set_kml(features)
        self.kml_change_state = self._kml_state(kml_mod)

    def idle_task(self):
        if self.map is None:
            return
        # drain events before dropping a dead child, so we don't lose the
        # reason it failed to start
        alive = self.map.is_alive()
        for event in self.map.check_events():
            if event[0] == 'startup_error':
                print("map3d: the 3D view failed to start:\n%s" % event[1])
            # what the view's own controls were set to.  Each is sent back
            # as the setting now stands: a command given here while the
            # event was on its way has already been sent to the view, and
            # would otherwise be overwritten there by this older event
            # while the setting kept it
            elif event[0] == 'render_settings':
                (_, brightness, shading, wireframe, fpvfov) = event
                self.map3d_settings.terrainbrightness = brightness
                self.map3d_settings.terrainshading = shading
                self.map3d_settings.terrainwireframe = wireframe
                self.map3d_settings.fpvfov = fpvfov
                self.send_render_settings()
            elif event[0] == 'follow':
                self.follow = bool(event[1])
                self.map.set_follow(self.follow)
            elif event[0] == 'mission_labels':
                self.map3d_settings.showlabels = bool(event[1])
                self.map.set_mission_labels(self.map3d_settings.showlabels)
            elif event[0] == 'mission_style':
                self.map3d_settings.missionpath = event[1]
                self.flown_asked = event[1] == 'flown'
                self.map.set_mission_style(self.map3d_settings.missionpath)
                # the path flown is only worked out while it is drawn
                self.send_mission()
        if not alive:
            self.map = None
            return
        kml_mod = self.module('kmlread')
        if self._kml_state(kml_mod) != self.kml_change_state:
            self.send_kml(kml_mod)
        # poll change times like the 2D map / cesium modules
        try:
            wp_change = self.module('wp').wploader.last_change
            if wp_change != self.wp_change_time:
                self.wp_change_time = wp_change
                self.send_mission()
        except Exception:
            pass
        try:
            fence_change = self.module('fence').wploader.last_change
            if fence_change != self.fence_change_time:
                self.fence_change_time = fence_change
                self.send_fence()
        except Exception:
            pass
        try:
            rally_change = self.module('rally').rallyloader.last_change
            if rally_change != self.rally_change_time:
                self.rally_change_time = rally_change
                self.send_rally()
                # a return to launch may go to one
                self.send_mission()
        except Exception:
            pass
        if self.terrain_resolved:
            # a deferred terrain lookup landed: redo the terrain-frame items
            self.terrain_resolved = False
            self.send_mission()
            self.send_rally()
        self.redraw_for_ground_heading()
        self.draw_flown_track()

    def redraw_for_ground_heading(self, now=None):
        '''fly the mission again from the way the vehicle points now, if that
        has changed, and not too often'''
        if not self.ground_heading_changed:
            return
        if now is None:
            now = time.time()
        if now - self.ground_heading_redrawn < TAKEOFF_HEADING_REDRAW:
            return
        self.ground_heading_changed = False
        self.ground_heading_redrawn = now
        if self.vehicle_type == 'plane':
            self.send_mission()

    def note_ground_heading(self, yaw):
        '''the vehicle points yaw radians while disarmed.  A change of less
        than TAKEOFF_HEADING_CHANGE is not worth flying the mission again'''
        heading = round(math.degrees(yaw)) % 360
        if self.ground_heading is not None:
            change = abs((heading - self.ground_heading + 180) % 360 - 180)
            if change < TAKEOFF_HEADING_CHANGE:
                return
        self.ground_heading = heading
        self.ground_heading_changed = True

    def mavlink_packet(self, m, force=False):
        if self.map is None or not self.map.is_alive():
            return
        mtype = m.get_type()
        if mtype in ('HEARTBEAT', 'HIGH_LATENCY2'):
            self.set_icon_type(mp_util.vehicle_type_name(m.type))
            if (mtype == 'HEARTBEAT' and
                    m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                self.armed = bool(m.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif mtype == 'GPS_GLOBAL_ORIGIN':
            # the EKF origin, which a rally point's altitude may be above
            origin = m.altitude * 1.0e-3        # AMSL (mm -> m)
            if origin != self.origin_amsl:
                self.origin_amsl = origin
                self.send_rally()
                self.send_mission()
        elif mtype == 'ORGN':
            if m.Type == 0 and m.Alt != self.origin_amsl:
                self.origin_amsl = m.Alt
                self.send_rally()
                self.send_mission()
        elif mtype == 'WIND':
            # where the wind comes from, which is the course ArduPlane
            # works out for a VTOL landing approach, by the same sum
            approach = (round(m.direction / APPROACH_CHANGE) *
                        APPROACH_CHANGE)
            approach = (approach + 180.0) % 360.0 - 180.0
            if approach != self.approach:
                self.approach = approach
                if self.approach_used:
                    self.send_mission()
        elif mtype == 'HOME_POSITION':
            home_amsl = m.altitude * 1.0e-3     # AMSL (mm -> m)
            if home_amsl != self.home_amsl:
                self.home_amsl = home_amsl
                # rally points above home, or the origin in its place
                self.send_rally()
            self.map.set_home(self.home_amsl)
            self.set_home_position(m.latitude * 1.0e-7, m.longitude * 1.0e-7)
        elif mtype == 'ATTITUDE':
            self.last_attitude = (m.roll, m.pitch, m.yaw)
            if not self.armed:
                self.note_ground_heading(m.yaw)
        elif mtype == 'ATT':
            # ArduPilot DataFlash attitude is recorded in degrees.
            self.last_attitude = tuple(math.radians(v) for v in
                                       (m.Roll, m.Pitch, m.Yaw))
        elif mtype == 'GLOBAL_POSITION_INT':
            self.last_global_position = time.time()
            lat = m.lat * 1.0e-7
            lon = m.lon * 1.0e-7
            alt = m.alt * 1.0e-3            # AMSL (mm -> m)
            rel = m.relative_alt * 1.0e-3   # above home
            self.send_vehicle_position(lat, lon, alt, alt - rel, force=force)
        elif mtype == 'POS':
            # DataFlash log playback does not contain GLOBAL_POSITION_INT.
            if not force and time.time() - self.last_global_position < 1.0:
                return
            rel = getattr(m, 'RelHomeAlt', 0.0)
            self.send_vehicle_position(m.Lat, m.Lng, m.Alt, m.Alt - rel,
                                       force=force)
        elif mtype == 'GPS_RAW_INT':
            # Raw GPS is a live fallback until estimator position is available.
            if not force and time.time() - self.last_global_position < 1.0:
                return
            if getattr(m, 'fix_type', 0) < 2:
                return
            lat = m.lat * 1.0e-7
            lon = m.lon * 1.0e-7
            alt = m.alt * 1.0e-3
            yaw = math.radians(m.cog * 0.01)
            attitude = (self.last_attitude[0], self.last_attitude[1], yaw)
            self.send_vehicle_position(lat, lon, alt, attitude=attitude,
                                       force=force)

    def unload(self):
        self.stop_map()


def init(mpstate):
    return Map3DModule(mpstate)
