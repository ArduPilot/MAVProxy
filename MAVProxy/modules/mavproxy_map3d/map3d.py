'''
Parent-side handle for the 3D map. Spawns the VTK/wx viewer in a child process
(mirrors mp_slipmap) and pushes element/camera updates over a queue.
'''

import time

from MAVProxy.modules.lib import multiproc


class Map3D:
    def __init__(self, title="3D Map", service="MicrosoftSat", zexag=1.0,
                 width=1100, height=800, debug=False):
        self.title = title
        self.service = service
        self.zexag = zexag
        self.width = width
        self.height = height
        self.debug = debug

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
        from MAVProxy.modules.lib import wx_processguard  # noqa: F401
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame

        app = wx.App(False)
        app.SetExitOnFrameDelete(True)
        frame = Map3DFrame(self)
        frame.Show()
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

    def set_path(self, path):
        self._put(('path', list(path)))

    def set_mission(self, items):
        self._put(('mission', list(items)))

    def set_fence(self, pts):
        self._put(('fence', list(pts)))

    def set_rally(self, pts):
        self._put(('rally', list(pts)))

    def set_follow(self, enable):
        self._put(('follow', bool(enable)))

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
