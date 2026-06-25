'''
3D map module: draped satellite imagery over ArduPilot quantized-mesh terrain,
rendered natively with VTK. Shows the same elements as the 2D map (flight path,
mission, fence, rally, vehicle) for both live telemetry and log review.

Andrew Tridgell / CanberraUAV
'''

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map3d.map3d import Map3D


class Map3DModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(Map3DModule, self).__init__(mpstate, "map3d", "3D map display", public=True)
        self.map3d_settings = mp_settings.MPSettings([
            ('service', str, 'MicrosoftSat'),
            ('zexag', float, 1.0),
            ('debug', bool, False),
        ])
        self.add_command('map3d', self.cmd_map3d,
                         "3D map control", ['<start|stop|follow|nofollow|center>',
                                            'set (MAP3DSETTING)'])
        self.map = None
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.rally_change_time = 0
        self.last_vehicle_send = 0
        self.last_attitude = (0.0, 0.0, 0.0)

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
            if self.map:
                self.map.set_follow(True)
        elif cmd == "nofollow":
            if self.map:
                self.map.set_follow(False)
        elif cmd == "center":
            if self.map:
                self.map.center_on_vehicle()
        elif cmd == "set":
            self.map3d_settings.command(args[1:])
        else:
            print("unknown map3d command: %s" % cmd)

    def start_map(self):
        if self.map is not None and self.map.is_alive():
            print("map3d already running")
            return
        self.map = Map3D(title="MAVProxy 3D Map",
                         service=self.map3d_settings.service,
                         zexag=self.map3d_settings.zexag,
                         debug=self.map3d_settings.debug)
        # push whatever we already know
        self.send_mission()
        self.send_fence()
        self.send_rally()

    def stop_map(self):
        if self.map is not None:
            self.map.close()
            self.map = None

    # --------------------------------------------------------------- data feeds
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

    def idle_task(self):
        if self.map is None:
            return
        if not self.map.is_alive():
            self.map = None
            return
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

    def mavlink_packet(self, m):
        if self.map is None or not self.map.is_alive():
            return
        mtype = m.get_type()
        if mtype == 'HOME_POSITION':
            self.map.set_home(m.altitude * 1.0e-3)   # AMSL (mm -> m)
        elif mtype == 'ATTITUDE':
            self.last_attitude = (m.roll, m.pitch, m.yaw)
        elif mtype == 'GLOBAL_POSITION_INT':
            now = time.time()
            if now - self.last_vehicle_send < 0.1:
                return
            self.last_vehicle_send = now
            lat = m.lat * 1.0e-7
            lon = m.lon * 1.0e-7
            alt = m.alt * 1.0e-3            # AMSL (mm -> m)
            rel = m.relative_alt * 1.0e-3   # above home
            if not self.map.origin_set():
                self.map.set_origin(lat, lon, alt)
                self.map.set_home(alt - rel)   # home AMSL = current AMSL - relative
                self.send_mission()
                self.send_fence()
                self.send_rally()
            (roll, pitch, yaw) = self.last_attitude
            self.map.set_vehicle(lat, lon, alt, roll, pitch, yaw)

    def unload(self):
        self.stop_map()


def init(mpstate):
    return Map3DModule(mpstate)
