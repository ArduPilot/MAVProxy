'''
Child-process GUI for the 3D map: a wx frame hosting a VTK render window. A timer
drains the parent's object queue (element/camera updates) and the terrain
manager's async results, then renders.
'''

import math
import queue

from MAVProxy.modules.lib.wx_loader import wx
from vtk.wx.wxVTKRenderWindowInteractor import wxVTKRenderWindowInteractor

from MAVProxy.modules.mavproxy_map import mp_tile
from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera, TerrainStyle
from MAVProxy.modules.mavproxy_map3d.terrain import TerrainManager, R
from MAVProxy.modules.mavproxy_map3d.elements import ElementManager

import vtk


class Map3DFrame(wx.Frame):
    def __init__(self, state):
        wx.Frame.__init__(self, None, -1, state.title,
                          size=(state.width, state.height))
        self.state = state
        self.terrain = None
        self.elements = None
        self.tc = None
        self.mt = None
        self.follow = False
        self.last_vehicle = None

        self.widget = wxVTKRenderWindowInteractor(self, -1)
        self.widget.Enable(1)
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(0.45, 0.62, 0.85)
        self.widget.GetRenderWindow().AddRenderer(self.ren)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.widget, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_CLOSE, self.on_close)

    # ----------------------------------------------------------------- scene
    def init_scene(self, lat0, lon0, amsl_ref):
        self.mt = mp_tile.MPTile(service=self.state.service, tile_delay=0.02)
        self.terrain = TerrainManager(self.ren, self.mt, lat0, lon0,
                                      zexag=self.state.zexag,
                                      screen_h=self.GetClientSize()[1] or self.state.height)
        self.elements = ElementManager(self.ren, lat0, lon0, self.state.zexag)
        self.elements.set_home(amsl_ref)
        span = 4000.0
        self.tc = TerrainCamera(self.ren.GetActiveCamera(),
                                focal=(0.0, 0.0, amsl_ref * self.state.zexag),
                                dist=1.6 * span, yaw=0.0, pitch=30.0)
        style = TerrainStyle(self.tc, on_change=self.on_camera_change)
        self.widget.SetInteractorStyle(style)
        self.terrain.update(self.tc)

    def on_camera_change(self):
        if self.terrain:
            self.terrain.update(self.tc)
        self.render()

    def render(self):
        self.widget.GetRenderWindow().Render()

    def look_at_latlon(self, lat, lon, amsl, dist=None):
        e = math.radians(lon - self.terrain.lon0) * R * math.cos(math.radians(self.terrain.lat0))
        n = math.radians(lat - self.terrain.lat0) * R
        u = amsl * self.state.zexag
        self.tc.look_at((e, n, u), dist=dist)
        self.on_camera_change()

    # --------------------------------------------------------------- messages
    def handle(self, msg):
        kind = msg[0]
        if kind == 'origin':
            if self.terrain is None:
                self.init_scene(msg[1], msg[2], msg[3])
        if self.terrain is None:
            return
        if kind == 'home':
            self.elements.set_home(msg[1])
        elif kind == 'vehicle':
            (_, lat, lon, amsl, roll, pitch, yaw) = msg
            enu = self.elements.set_vehicle(lat, lon, amsl, roll, pitch, yaw)
            self.elements.add_trail_point(lat, lon, amsl)
            self.last_vehicle = (lat, lon, amsl)
            if self.follow:
                self.tc.look_at((enu[0], enu[1], enu[2]))
                self.terrain.update(self.tc)
        elif kind == 'path':
            self.elements.set_path(msg[1])
        elif kind == 'mission':
            self.elements.set_mission(msg[1])
        elif kind == 'fence':
            self.elements.set_fence(msg[1])
        elif kind == 'rally':
            self.elements.set_rally(msg[1])
        elif kind == 'lookat':
            (_, lat, lon, amsl, dist) = msg
            self.look_at_latlon(lat, lon, amsl, dist)
        elif kind == 'follow':
            self.follow = msg[1]
        elif kind == 'center':
            if self.last_vehicle:
                self.look_at_latlon(*self.last_vehicle)

    def on_timer(self, event):
        # drain parent messages
        drained = False
        for _ in range(200):
            try:
                msg = self.state.object_queue.get_nowait()
            except queue.Empty:
                break
            self.handle(msg)
            drained = True
        # drain terrain worker results
        if self.terrain is not None:
            if self.terrain.process(self.tc):
                drained = True
        if drained:
            self.render()
        # close request from parent
        if self.state.close_window.acquire(False):
            self.timer.Stop()
            self.Close()

    def on_close(self, event):
        if self.terrain is not None:
            self.terrain.shutdown()
        self.Destroy()
