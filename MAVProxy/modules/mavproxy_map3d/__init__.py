'''
3D map module: draped satellite imagery over ArduPilot quantized-mesh terrain,
rendered natively with VTK. Shows the same elements as the 2D map (flight path,
mission, fence, rally, vehicle) for both live telemetry and log review.

Andrew Tridgell / CanberraUAV
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
    Map3D, MissionItem, missing_packages, missing_packages_message)

# fence colours as the 2D map's PolyFence layer uses them (OpenCV BGR)
FENCE_INCLUSION_BGR = (0, 255, 0)
FENCE_HOME_INCLUSION_BGR = (0, 255, 96)
FENCE_EXCLUSION_BGR = (255, 0, 0)
FENCE_RETURN_BGR = (255, 127, 127)
FENCE_RETURN_RADIUS = 10.0

# takeoff items normally carry an altitude and no position
TAKEOFF_COMMANDS = (22, 84)      # NAV_TAKEOFF, NAV_VTOL_TAKEOFF


def bgr_to_rgb(bgr, default=(1.0, 0.0, 1.0)):
    '''SlipMap/OpenCV colours are BGR; VTK wants RGB floats'''
    try:
        return tuple(max(0, min(255, int(bgr[i]))) / 255.0 for i in (2, 1, 0))
    except (IndexError, TypeError, ValueError):
        return default


class Map3DModule(mp_module.MPModule):
    def __init__(self, mpstate):
        # Do not register as a public "map*" module. ADS-B, AIS, KML and other
        # modules use that wildcard for the 2D SlipMap API (add_object,
        # remove_object, set_position), which Map3D intentionally does not
        # implement.
        super(Map3DModule, self).__init__(mpstate, "map3d", "3D map display")
        self.map3d_settings = mp_settings.MPSettings([
            ('service', str, 'MicrosoftSat'),
            ('zexag', float, 1.0),
            ('debug', bool, False),
            mp_settings.MPSetting('fpvfov', float, 90.0, range=(20.0, 150.0)),
            mp_settings.MPSetting('terrainbrightness', float, 1.25,
                                  range=(0.25, 2.0)),
            ('terrainshading', bool, True),
            ('terrainwireframe', bool, False),
            ('showdirection', bool, False),
        ])
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
            if self.map is not None and self.map.is_alive():
                self.map.set_fpv_fov(self.map3d_settings.fpvfov)
                self.map.set_mission_arrows(self.map3d_settings.showdirection)
                self.map.set_render_settings(
                    self.map3d_settings.terrainbrightness,
                    self.map3d_settings.terrainshading,
                    self.map3d_settings.terrainwireframe)
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
        self.send_mission()
        self.send_fence()
        self.send_rally()
        self.send_cached_state()
        self.send_kml()

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

    def send_mission(self):
        if self.map is None:
            return
        try:
            wploader = self.module('wp').wploader
        except Exception:
            return
        items = []
        default_radius = self.default_circle_radius()
        previous = None
        for w in wploader.wpoints:
            frame = getattr(w, 'frame', 0)
            (lat, lon) = (w.x, w.y)
            if lat == 0 and lon == 0 and w.command in TAKEOFF_COMMANDS:
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
            circle_radius = mp_util.mission_circle_radius(
                w.command,
                (w.param1, w.param2, w.param3, w.param4),
                default_radius,
                self.vehicle_type)
            circle_turns = None
            if w.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
                # this one circles until it reaches its altitude, so what is
                # left to climb on arrival decides how many turns to draw
                approach = None
                if previous is not None:
                    approach = mp_util.gps_distance(previous[0], previous[1],
                                                    lat, lon)
                circle_turns = mp_util.loiter_to_alt_turns(
                    circle_radius,
                    z - previous[2] if previous is not None else None,
                    self.mav_param, approach)
            exit_converge = None
            if circle_radius is not None and not (w.param4 > 0):
                # param4 == 0 asks for the next leg to be crosstracked from
                # the loiter centre rather than from where it was left
                exit_converge = mp_util.vehicle_track_convergence(self.mav_param)
            items.append(MissionItem(lat, lon, z, frame, w.command, w.seq,
                                     w.param1, circle_radius, circle_turns,
                                     exit_converge))
            previous = (lat, lon, z)
        self.map.set_mission(items)

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
        try:
            rally_mod = self.module('rally')
            pts = []
            for i in range(rally_mod.rallyloader.rally_count()):
                r = rally_mod.rallyloader.rally_point(i)
                pts.append((r.lat * 1.0e-7, r.lng * 1.0e-7, r.alt))
        except Exception:
            return
        self.map.set_rally(pts)

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
            elif event[0] == 'render_settings':
                (_, brightness, shading, wireframe, fpvfov) = event
                self.map3d_settings.terrainbrightness = brightness
                self.map3d_settings.terrainshading = shading
                self.map3d_settings.terrainwireframe = wireframe
                self.map3d_settings.fpvfov = fpvfov
            elif event[0] == 'follow':
                self.follow = bool(event[1])
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
        except Exception:
            pass
        if self.terrain_resolved:
            # a deferred terrain lookup landed: redo the terrain-frame items
            self.terrain_resolved = False
            self.send_mission()

    def mavlink_packet(self, m, force=False):
        if self.map is None or not self.map.is_alive():
            return
        mtype = m.get_type()
        if mtype in ('HEARTBEAT', 'HIGH_LATENCY2'):
            self.set_icon_type(mp_util.vehicle_type_name(m.type))
        elif mtype == 'HOME_POSITION':
            self.home_amsl = m.altitude * 1.0e-3     # AMSL (mm -> m)
            self.map.set_home(self.home_amsl)
            self.set_home_position(m.latitude * 1.0e-7, m.longitude * 1.0e-7)
        elif mtype == 'ATTITUDE':
            self.last_attitude = (m.roll, m.pitch, m.yaw)
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
