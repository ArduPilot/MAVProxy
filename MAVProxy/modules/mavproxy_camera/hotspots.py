"""Project CAMERA_THERMAL_RANGE maxima onto the camera's ground map.

AP_FLAKE8_CLEAN
"""
import math
import time
from collections import OrderedDict, deque

from MAVProxy.modules.lib import mp_util

# drawn after (above) the 'Survey coverage' layer: layers draw in name order
LAYER = 'Thermal hotspots'


class HotspotGroup:
    """nearby detections shown as one marker at their centroid"""
    def __init__(self, owner):
        self.owner = owner
        # newest detections; count covers all of them
        self.points = deque(maxlen=10000)
        self.count = 0
        self.latlon = None
        self.temperature = -math.inf
        self.drawn = 0

    def add(self, lat, lon, temperature):
        self.points.append((lat, lon, temperature))
        self.count += 1
        count = self.count
        if count == 1:
            self.latlon = (lat, lon)
        else:
            self.latlon = (self.latlon[0] + (lat - self.latlon[0]) / count,
                           mp_util.wrap_180(self.latlon[1] + mp_util.wrap_180(lon - self.latlon[1]) / count))
        self.temperature = max(self.temperature, temperature)

    def spread(self):
        """distance in metres from the centroid to the furthest point"""
        lat0, lon0 = self.latlon
        scale = math.cos(math.radians(lat0))
        furthest = max((lat - lat0) ** 2 + (mp_util.wrap_180(lon - lon0) * scale) ** 2
                       for lat, lon, _temperature in self.points)
        return math.sqrt(furthest) * 111319.5


class CameraHotspots:
    def __init__(self, module):
        self.module = module
        self.markers = OrderedDict()
        self.limit = 4000
        self.radius = 50
        self.last_frames = {}
        self.newest = {}
        self.seen = {}
        self.icon = None
        self.dirty = set()
        self.number = 0
        self.displays = ()

    def command(self, args):
        if args == ['clear']:
            camera = self.module.command_camera
            self.clear(None if camera is None else (camera.system_id, camera.component_id))
            return
        if len(args) > 1:
            raise ValueError('usage: camera hotspot [CELSIUS|clear]')
        threshold = float(args[0]) if args else None
        if threshold is not None and not math.isfinite(threshold):
            raise ValueError('Hotspot temperature must be finite')
        camera = self.module._selected_camera()
        if camera is None:
            return
        if threshold is not None:
            camera.hotspot_threshold = threshold
            if threshold != 0:
                self.module._request_thermal_state(camera, force=True)
        value = camera.hotspot_threshold
        print('Camera %u:%u hotspots: %s' % (
            camera.system_id, camera.component_id,
            'disabled' if value == 0 else 'above %g degC' % value))

    def packet(self, camera, message):
        if camera.hotspot_threshold == 0 or not mp_util.has_wxpython:
            return
        # A non-MAVLink camera attached to an autopilot is a different camera
        # instance; its stream and mount cannot be inferred from this component.
        if getattr(message, 'camera_device_id', 0) != 0:
            return
        owner = (camera.system_id, camera.component_id)
        source = owner + (getattr(message, 'stream_id', 0),)
        stamp = getattr(message, 'time_boot_ms', None)
        values = (getattr(message, f, None) for f in ('max', 'max_point_x', 'max_point_y'))
        frame = (stamp,) + tuple(None if isinstance(v, float) and math.isnan(v) else v for v in values)
        # links can interleave repeats of recent frames
        recent = self.last_frames.setdefault(source, deque(maxlen=256))
        if stamp and frame in recent:
            return
        # a late frame can't be projected with the current pose; a much
        # older stamp is a camera restart, unless it repeats a known frame
        newest = self.newest.get(source, 0)
        seen = self.seen.setdefault(source, deque(maxlen=256))
        repeat = bool(stamp) and frame in seen
        # Only retry an unplotted repeat while its timestamp is current.
        late = bool(stamp and newest) and (0 < (newest - stamp) % (1 << 32) <= 10000 or
                                           (repeat and stamp != newest))
        if stamp and not repeat:
            seen.append(frame)
            if not late:
                self.newest[source] = stamp
        try:
            temperature, x, y = message.max, message.max_point_x, message.max_point_y
            if (not all(math.isfinite(v) for v in (temperature, x, y)) or
                    temperature <= camera.hotspot_threshold or
                    not (0 <= x <= 1 and 0 <= y <= 1)):
                return
            stream = camera.streams.get(message.stream_id)
            if stream is None or getattr(stream, 'camera_device_id', 0) != 0:
                return
            width, height = stream.resolution_h, stream.resolution_v
            if not all(math.isfinite(v) and v > 0 for v in (width, height)):
                return
        except (AttributeError, TypeError, ValueError):
            return
        if late:
            return
        location = self.module.project_pixel(camera, stream, x * width, y * height, width, height)
        if location is None or not all(math.isfinite(v) for v in location):
            return
        if stamp:
            recent.append(frame)
        self.add_point(owner, location[0], location[1], temperature)
        self.flush()

    def add_point(self, owner, lat, lon, temperature):
        """add a detection to the nearest group within self.radius metres"""
        best, best_distance = None, self.radius
        margin = self.radius / 111000.0
        for name, group in self.markers.items():
            if group.owner != owner or abs(group.latlon[0] - lat) > margin:
                continue
            distance = mp_util.gps_distance(lat, lon, *group.latlon)
            if distance <= best_distance:
                best, best_distance = name, distance
        if best is None:
            self.number += 1
            best = 'CameraHotspot_%u_%u_%u' % (*owner, self.number)
            self.markers[best] = HotspotGroup(owner)
        self.markers[best].add(lat, lon, temperature)
        self.markers.move_to_end(best)
        self.dirty.add(best)
        while len(self.markers) > self.limit:
            oldest, _ = self.markers.popitem(last=False)
            self.remove(oldest, self.module.coverage.maps())

    def flush(self, displays=None, force=False):
        """draw changed groups, at most once a second each"""
        if not mp_util.has_wxpython:
            return
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        if displays is None:
            displays = self.module.coverage.maps()
        now = time.time()
        for name in list(self.markers if force else self.dirty):
            group = self.markers[name]
            if not force:
                if now - group.drawn < 1:
                    continue
                self.dirty.discard(name)
                group.drawn = now
            count = group.count
            # the map font is ASCII only
            label = '%.0f C' % group.temperature
            if count > 1:
                label += ' (%u)' % count
            area = self.area_name(name)
            radius = group.spread()
            for display in displays:
                if self.icon is None:
                    self.icon = display.icon('flame.png')
                if count > 1:
                    # yellow circle shows the scatter of the group
                    display.add_object(mp_slipmap.SlipCircle(
                        area, LAYER, group.latlon, max(radius, 1), (255, 255, 0), 1))
                display.add_object(mp_slipmap.SlipIcon(
                    name, group.latlon, self.icon, layer=LAYER,
                    rotation=0, follow=False, label=label))

    @staticmethod
    def area_name(name):
        # sorts before the icon, so the flame is drawn over the circle
        return name.replace('CameraHotspot_', 'CameraHotspotArea_')

    def remove(self, name, displays):
        self.dirty.discard(name)
        for display in displays:
            display.remove_object(name)
            display.remove_object(self.area_name(name))

    def idle(self):
        displays = self.module.coverage.maps()
        added = tuple(d for d in displays if d not in self.displays)
        self.displays = displays
        if added:
            self.flush(added, force=True)
        self.flush(displays)

    def clear(self, owner=None):
        displays = self.module.coverage.maps()
        for name, group in list(self.markers.items()):
            if owner is not None and owner != group.owner:
                continue
            self.remove(name, displays)
            del self.markers[name]
