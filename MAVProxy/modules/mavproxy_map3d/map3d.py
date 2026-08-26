'''
Parent-side handle for the 3D map. Spawns the VTK/wx viewer in a child process
(mirrors mp_slipmap) and pushes element/camera updates over a queue.
'''

import collections
import importlib.util
import time
import queue
import traceback

from MAVProxy.modules.lib import multiproc

# one mission item as the viewer needs it.  circle_radius is the signed radius
# of the circle the item flies about its own location (positive is clockwise),
# or None for the items which do not fly one.  circle_turns is how many turns
# to draw that circle over, which shows as a spiral for an item that changes
# altitude while it circles
MissionItem = collections.namedtuple(
    'MissionItem',
    'lat lon alt frame command seq param1 circle_radius circle_turns',
    defaults=(0.0, None, None))

PACKAGES = ('vtk', 'quantized_mesh_tile')


def missing_packages():
    '''optional 3D map packages that are not installed. The viewer imports them
    in a child process, where an ImportError would go unseen, so callers check
    before starting a viewer. find_spec avoids importing VTK into the parent.'''
    missing = []
    for name in PACKAGES:
        try:
            if importlib.util.find_spec(name) is None:
                missing.append(name)
        except Exception:
            missing.append(name)
    return missing


def missing_packages_message(missing):
    # install via the extra, not 'pip install vtk quantized-mesh-tile': the
    # latter pulls numpy>=2 without the opencv/matplotlib builds to match
    return ("map3d needs extra packages: pip install 'MAVProxy[map3d]' "
            "(missing %s)" % ', '.join(missing))


class Map3D:
    def __init__(self, title="3D Map", service="MicrosoftSat", zexag=1.0,
                 width=1100, height=800, debug=False, fpvfov=90.0,
                 terrain_brightness=1.25, terrain_shading=True,
                 terrain_wireframe=False, follow=True):
        self.title = title
        self.service = service
        self.zexag = zexag
        self.width = width
        self.height = height
        self.debug = debug
        self.fpvfov = fpvfov
        self.terrain_brightness = terrain_brightness
        self.terrain_shading = terrain_shading
        self.terrain_wireframe = terrain_wireframe
        self.follow = bool(follow)

        self.object_queue = multiproc.Queue()
        self.event_queue = multiproc.Queue()
        self.app_ready = multiproc.Event()
        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()
        self._origin_set = False

    def child_task(self):
        from MAVProxy.modules.lib import mp_util
        mp_util.child_close_fds()
        try:
            from MAVProxy.modules.lib import wx_processguard  # noqa: F401
            from MAVProxy.modules.lib.wx_loader import wx
            from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame

            app = wx.App(False)
            app.SetExitOnFrameDelete(True)
            frame = Map3DFrame(self)
            frame.Show()
        except (Exception, SystemExit):
            # our stderr goes nowhere when MAVProxy is started from a GUI, so
            # hand the failure to the parent rather than dying unexplained.
            # wx exits rather than raising when it cannot open the display,
            # hence SystemExit
            self.event_queue.put(('startup_error', traceback.format_exc()))
            return
        self.app_ready.set()
        app.MainLoop()

    def is_alive(self):
        return self.child.is_alive()

    def _put(self, msg):
        if self.child.is_alive():
            self.object_queue.put(msg)

    # ------------------------------------------------------------- push helpers
    def set_origin(self, lat, lon, amsl_ref):
        self._origin_set = True
        self._put(('origin', lat, lon, amsl_ref))

    def origin_set(self):
        return self._origin_set

    def set_home(self, amsl):
        self._put(('home', amsl))

    def set_vehicle(self, lat, lon, amsl, roll=0.0, pitch=0.0, yaw=0.0):
        self._put(('vehicle', lat, lon, amsl, roll, pitch, yaw))

    def set_vehicle_type(self, vehicle_type):
        '''vehicle_type is a name from mp_util.vehicle_type_name()'''
        self._put(('vehicletype', vehicle_type))

    def set_path(self, path):
        self._put(('path', list(path)))

    def set_time_range(self, trange):
        '''Limit timestamped path points to a Matplotlib date-number range.'''
        self._put(('time_range', None if trange is None else tuple(trange)))

    def set_mission(self, items):
        self._put(('mission', list(items)))

    def set_fence(self, shapes):
        '''shapes: ('polygon', [(lat,lon), ...], rgb) / ('circle', (lat,lon), radius_m, rgb)'''
        self._put(('fence', list(shapes)))

    def set_kml(self, features):
        self._put(('kml', list(features)))

    def set_rally(self, pts):
        self._put(('rally', list(pts)))

    def set_follow(self, enable):
        self._put(('follow', bool(enable)))

    def set_fpv_fov(self, fov):
        self._put(('fpvfov', float(fov)))

    def set_render_settings(self, brightness, shading, wireframe):
        self._put(('render_settings', float(brightness), bool(shading),
                   bool(wireframe)))

    def check_events(self):
        events = []
        while True:
            try:
                events.append(self.event_queue.get_nowait())
            except queue.Empty:
                return events

    def center_on_vehicle(self):
        self._put(('center', None))

    def look_at(self, lat, lon, amsl, dist=None):
        self._put(('lookat', lat, lon, amsl, dist))

    def close(self):
        self.close_window.release()
        count = 0
        while self.child.is_alive() and count < 20:
            time.sleep(0.1)
            count += 1
        if self.child.is_alive():
            self.child.terminate()
        self.child.join()
