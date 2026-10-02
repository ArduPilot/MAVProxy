"""Captured-image coverage; geometry is paired by exposure timestamp, never live pose."""
import math
import os
from collections import Counter, OrderedDict
from MAVProxy.modules.lib import mp_util


def footprint(fov):
    q = fov.q
    if not all(math.isfinite(v) for v in (*q, fov.hfov, fov.vfov)):
        return None
    if not (0 < fov.hfov < 150 and 0 < fov.vfov < 150):
        return None
    norm = math.sqrt(sum(v*v for v in q))
    if norm < .5 or norm > 1.5:
        return None
    w, x, y, z = (v / norm for v in q)
    # Rotation from camera forward/right/down axes to NED.
    rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    height = (fov.alt_camera - fov.alt_image) * .001
    if not 0 < height < 20000:
        return None
    h, v = math.tan(math.radians(fov.hfov/2)), math.tan(math.radians(fov.vfov/2))
    points = []
    for right, down in ((-h, -v), (h, -v), (h, v), (-h, v)):
        n, e, d = (r[0] + r[1]*right + r[2]*down for r in rotation)
        if d < .02 or math.hypot(n, e) * height / d > 20000:
            return None
        points.append(mp_util.gps_offset(fov.lat_camera*1e-7, fov.lon_camera*1e-7,
                                         e*height/d, n*height/d))
    return points + points[:1]


class SurveyCoverage:
    def __init__(self, module):
        self.module = module
        self.fovs = OrderedDict()
        self.pending = OrderedDict()
        self.images = OrderedDict()
        self.counts = Counter()
        self.hidden = set()
        self.limit = 4000
        self.alpha = .18
        self.displays = ()

    def packet(self, message):
        changed = False
        kind = message.get_type()
        if kind not in ('CAMERA_FOV_STATUS', 'CAMERA_IMAGE_CAPTURED'):
            return
        camera = (message.get_srcSystem(), message.get_srcComponent())
        stamp = camera + (message.time_boot_ms,)
        if kind == 'CAMERA_FOV_STATUS':
            self.fovs[stamp] = message
        elif message.capture_result == 1:
            self.pending[stamp] = message
        if stamp in self.fovs and stamp in self.pending:
            capture = self.pending.pop(stamp)
            fov = self.fovs.pop(stamp)
            # UTC plus image index separates camera restarts and retried events.
            key = camera + (capture.time_utc, capture.time_boot_ms, capture.image_index)
            points = footprint(fov)
            if points and key not in self.images:
                self.images[key] = points
                self.counts[camera] += 1
                changed = True
                self.draw(key, points)
        for cache in (self.fovs, self.pending):
            while len(cache) > 256:
                cache.popitem(last=False)
        while len(self.images) > self.limit:
            key, _ = self.images.popitem(last=False)
            self.remove(key)
        return changed

    @staticmethod
    def name(key):
        return 'survey_%s' % '_'.join(str(v) for v in key)

    def maps(self):
        displays = []
        matching = getattr(self.module, 'module_matching', None)
        if matching is not None:
            displays = [m.map for m in matching('map*') if getattr(m, 'map', None) is not None]
        primary = getattr(self.module.mpstate, 'map', None)
        if primary is not None and primary not in displays:
            displays.append(primary)
        return tuple(displays)

    def idle(self):
        # A map may be opened/reloaded after captures have already arrived.
        displays = self.maps()
        if displays != self.displays:
            self.displays = displays
            for key, points in self.images.items():
                self.draw(key, points)

    def remove(self, key):
        for display in self.maps():
            display.remove_object(self.name(key))

    def draw(self, key, points):
        if key[:2] in self.hidden or not mp_util.has_wxpython:
            return
        from MAVProxy.modules.mavproxy_map.mp_slipmap_util import SlipPolygon
        for display in self.maps():
            # Cyan contrasts with vegetation; a thin border also makes small
            # footprints visible at flight-planning zoom levels.
            display.add_object(SlipPolygon(self.name(key), points, layer='Survey coverage',
                                           colour=(0, 255, 255), linewidth=1, showlines=True,
                                           showcircles=False, fill_alpha=self.alpha))

    def command(self, args):
        if len(args) >= 2 and args[0] == 'load':
            from pymavlink import mavutil
            path = ' '.join(args[1:])
            if not os.path.isfile(path):
                raise ValueError('Telemetry log not found: ' + path)
            log = mavutil.mavlink_connection(path)
            try:
                while True:
                    message = log.recv_match(type=['CAMERA_FOV_STATUS', 'CAMERA_IMAGE_CAPTURED'])
                    if message is None:
                        break
                    self.packet(message)
            finally:
                log.close()
            print('Survey coverage: %u footprints retained after loading %s' % (len(self.images), path))
            return
        if len(args) == 2 and args[0] == 'alpha':
            alpha = float(args[1])
            if not math.isfinite(alpha) or not 0 <= alpha <= 1:
                raise ValueError('Coverage alpha must be 0..1')
            self.alpha = alpha
            for key, points in self.images.items():
                self.draw(key, points)
            print('Survey coverage opacity: %.0f%%' % (100*self.alpha))
            return
        if len(args) != 1 or args[0] not in ('show', 'hide', 'clear', 'status'):
            raise ValueError('usage: camera coverage <show|hide|clear|status|alpha 0..1|load LOG.tlog>')
        selected = self.module._selected_camera()
        if selected is None:
            return
        camera = (selected.system_id, selected.component_id)
        action = args[0]
        if action == 'hide':
            self.hidden.add(camera)
        elif action == 'show':
            self.hidden.discard(camera)
        for key in list(self.images):
            if key[:2] != camera:
                continue
            if action == 'show':
                self.draw(key, self.images[key])
            elif action in ('hide', 'clear'):
                self.remove(key)
                if action == 'clear':
                    del self.images[key]
        if action == 'clear':
            self.counts[camera] = 0
            for cache in (self.fovs, self.pending):
                for key in list(cache):
                    if key[:2] == camera:
                        del cache[key]
        if action == 'status':
            print('Survey coverage: %u captured footprints (limit %u); %u awaiting exposure geometry' %
                  (sum(k[:2] == camera for k in self.images), self.limit,
                   sum(k[:2] == camera for k in self.pending)))
            print('Display: %s; %u map(s); opacity %.0f%% with cyan outlines' %
                  ('hidden' if camera in self.hidden else 'visible', len(self.maps()), 100*self.alpha))
