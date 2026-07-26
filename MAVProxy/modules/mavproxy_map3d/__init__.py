'''
3D map module: draped satellite imagery over ArduPilot quantized-mesh terrain,
rendered natively with VTK. Shows the same elements as the 2D map (flight path,
mission, fence, rally, vehicle) for both live telemetry and log review.

Andrew Tridgell / CanberraUAV
'''

import math
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map3d.map3d import Map3D


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
        self.follow = True
        self.kml_change_state = None
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
        source rendered in 3D); fall back to the terrain module's SRTM model.'''
        try:
            from MAVProxy.modules.mavproxy_map3d.terrain import sample_terrain
            alt = sample_terrain(lat, lon)
            if alt is not None:
                return alt
        except Exception:
            pass
        tm = self.module('terrain')
        if tm is not None and getattr(tm, 'ElevationModel', None) is not None:
            return tm.ElevationModel.GetElevation(lat, lon)
        return None

    def send_mission(self):
        try:
            wploader = self.module('wp').wploader
        except Exception:
            return
        items = []
        for w in wploader.wpoints:
            frame = getattr(w, 'frame', 0)
            if w.x == 0 and w.y == 0 and w.command not in (16, 22, 82):
                continue
            z = w.z
            if frame in (10, 11):    # terrain-relative -> resolve to AMSL
                terr = self.terrain_alt(w.x, w.y)
                if terr is not None:
                    z = terr + w.z
                    frame = 0
            items.append((w.x, w.y, z, frame, w.command, w.seq))
        self.map.set_mission(items)

    def send_fence(self):
        try:
            fence_mod = self.module('fence')
            pts = [(p.x, p.y) for p in fence_mod.wploader.wpoints]
        except Exception:
            return
        self.map.set_fence(pts)

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
                    # SlipMap/OpenCV colours are BGR; VTK expects RGB.
                    bgr = getattr(obj, 'colour', (255, 0, 255))
                    try:
                        rgb = tuple(max(0, min(255, int(bgr[i]))) / 255.0
                                    for i in (2, 1, 0))
                    except (IndexError, TypeError, ValueError):
                        rgb = (1.0, 0.0, 1.0)
                    width = max(1.0, float(getattr(obj, 'linewidth', 2.0)))
                    features.append(("%s:%s" % (layer, key), points,
                                     rgb, width))
        features.sort(key=lambda feature: feature[0])
        self.map.set_kml(features)
        self.kml_change_state = self._kml_state(kml_mod)

    def idle_task(self):
        if self.map is None:
            return
        if not self.map.is_alive():
            self.map = None
            return
        for event in self.map.check_events():
            if event[0] == 'render_settings':
                (_, brightness, shading, wireframe, fpvfov) = event
                self.map3d_settings.terrainbrightness = brightness
                self.map3d_settings.terrainshading = shading
                self.map3d_settings.terrainwireframe = wireframe
                self.map3d_settings.fpvfov = fpvfov
            elif event[0] == 'follow':
                self.follow = bool(event[1])
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

    def mavlink_packet(self, m, force=False):
        if self.map is None or not self.map.is_alive():
            return
        mtype = m.get_type()
        if mtype == 'HOME_POSITION':
            self.home_amsl = m.altitude * 1.0e-3     # AMSL (mm -> m)
            self.map.set_home(self.home_amsl)
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
