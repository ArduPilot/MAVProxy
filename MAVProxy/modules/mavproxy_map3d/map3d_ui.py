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


class RenderSettingsDialog(wx.Dialog):
    def __init__(self, parent, values, on_preview):
        wx.Dialog.__init__(self, parent, title="3D Render Settings")
        self.on_preview = on_preview
        brightness, shading, wireframe, fpvfov = values

        self.brightness = wx.Slider(
            self, -1, int(round(brightness * 100.0)), 25, 200,
            style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.shading = wx.CheckBox(self, -1, "Enable terrain lighting")
        self.shading.SetValue(shading)
        self.wireframe = wx.CheckBox(self, -1, "Render terrain as wireframe")
        self.wireframe.SetValue(wireframe)
        self.fpvfov = wx.SpinCtrlDouble(self, -1, min=20.0, max=150.0,
                                        inc=5.0)
        self.fpvfov.SetDigits(1)
        self.fpvfov.SetValue(fpvfov)

        grid = wx.FlexGridSizer(4, 2, 8, 12)
        grid.Add(wx.StaticText(self, -1, "Terrain brightness"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.brightness, 1, wx.EXPAND)
        grid.Add(wx.StaticText(self, -1, "Terrain shading"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.shading, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(wx.StaticText(self, -1, "Terrain geometry"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.wireframe, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(wx.StaticText(self, -1, "FPV field of view (degrees)"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.fpvfov, 0, wx.EXPAND)
        grid.AddGrowableCol(1, 1)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(grid, 1, wx.ALL | wx.EXPAND, 12)
        sizer.Add(self.CreateButtonSizer(wx.OK | wx.CANCEL), 0,
                  wx.ALL | wx.ALIGN_RIGHT, 8)
        self.SetSizerAndFit(sizer)
        self.SetMinSize((480, self.GetSize()[1]))
        self.CentreOnParent()

        self.Bind(wx.EVT_SLIDER, self.on_control, self.brightness)
        self.Bind(wx.EVT_CHECKBOX, self.on_control, self.shading)
        self.Bind(wx.EVT_CHECKBOX, self.on_control, self.wireframe)
        self.Bind(wx.EVT_SPINCTRLDOUBLE, self.on_control, self.fpvfov)

    def values(self):
        return (self.brightness.GetValue() / 100.0,
                self.shading.GetValue(),
                self.wireframe.GetValue(),
                self.fpvfov.GetValue())

    def on_control(self, event):
        self.on_preview(self.values())
        event.Skip()


class Map3DFrame(wx.Frame):
    def __init__(self, state):
        wx.Frame.__init__(self, None, -1, state.title,
                          size=(state.width, state.height))
        self.state = state
        self.terrain = None
        self.elements = None
        self.tc = None
        self.mt = None
        self.follow = bool(state.follow)
        self.last_vehicle = None
        self.last_vehicle_pose = None
        self.fpv_enabled = False
        self.fpv_fov = min(150.0, max(20.0, float(state.fpvfov)))
        self.terrain_brightness = min(
            2.0, max(0.25, float(state.terrain_brightness)))
        self.terrain_shading = bool(state.terrain_shading)
        self.terrain_wireframe = bool(state.terrain_wireframe)
        self.saved_map_view = None

        self.widget = wxVTKRenderWindowInteractor(self, -1)
        self.widget.Enable(1)
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(0.45, 0.62, 0.85)
        self.widget.GetRenderWindow().AddRenderer(self.ren)
        self.status_actor = vtk.vtkTextActor()
        self.status_actor.SetInput("Waiting for vehicle position...")
        self.status_actor.SetPosition(20, 20)
        self.status_actor.GetTextProperty().SetFontSize(18)
        self.status_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)
        self.ren.AddActor2D(self.status_actor)
        self.initial_render = True

        sizer = wx.BoxSizer(wx.VERTICAL)
        controls = wx.BoxSizer(wx.HORIZONTAL)
        self.follow_button = wx.ToggleButton(self, -1, "Follow")
        self.follow_button.SetValue(self.follow)
        self.follow_button.SetToolTip("Keep the normal 3D view on the aircraft")
        self.Bind(wx.EVT_TOGGLEBUTTON, self.on_follow_toggle,
                  self.follow_button)
        controls.Add(self.follow_button, 0, wx.ALL, 4)
        self.fpv_button = wx.ToggleButton(self, -1, "FPV View")
        self.fpv_button.SetToolTip("Toggle level-horizon first-person view")
        self.Bind(wx.EVT_TOGGLEBUTTON, self.on_fpv_toggle, self.fpv_button)
        controls.Add(self.fpv_button, 0, wx.ALL, 4)
        gear = wx.ArtProvider.GetBitmap(wx.ART_HELP_SETTINGS, wx.ART_BUTTON,
                                        (16, 16))
        self.settings_button = wx.BitmapButton(self, -1, gear)
        self.settings_button.SetToolTip("3D render settings")
        self.Bind(wx.EVT_BUTTON, self.on_settings, self.settings_button)
        controls.Add(self.settings_button, 0, wx.ALL, 4)
        sizer.Add(controls, 0, wx.EXPAND)
        sizer.Add(self.widget, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_CLOSE, self.on_close)

    # ----------------------------------------------------------------- scene
    def init_scene(self, lat0, lon0, amsl_ref):
        self.status_actor.SetInput("Loading terrain...")
        self.mt = mp_tile.MPTile(service=self.state.service, tile_delay=0.02)
        self.terrain = TerrainManager(self.ren, self.mt, lat0, lon0,
                                      zexag=self.state.zexag,
                                      screen_h=self.GetClientSize()[1] or self.state.height,
                                      brightness=self.terrain_brightness,
                                      shading=self.terrain_shading,
                                      wireframe=self.terrain_wireframe)
        self.elements = ElementManager(self.ren, lat0, lon0, self.state.zexag)
        self.elements.set_terrain_height(self.terrain.height_at)
        self.elements.set_home(amsl_ref)
        span = 4000.0
        self.tc = TerrainCamera(self.ren.GetActiveCamera(),
                                focal=(0.0, 0.0, amsl_ref * self.state.zexag),
                                dist=1.6 * span, yaw=0.0, pitch=30.0)
        self.style = TerrainStyle(self.tc, on_change=self.on_camera_change)
        self.widget.SetInteractorStyle(self.style)
        if self.fpv_enabled:
            camera = self.ren.GetActiveCamera()
            self.saved_map_view = (list(self.tc.focal), self.tc.dist,
                                   self.tc.yaw, self.tc.pitch,
                                   camera.GetViewAngle())
            self.style.interaction_enabled = False
            self.elements.set_vehicle_visible(False)
            camera.SetViewAngle(self.fpv_fov)
        self.terrain.update(self.tc)

    def on_fpv_toggle(self, event):
        self.set_fpv_enabled(self.fpv_button.GetValue())

    def on_follow_toggle(self, event):
        self.set_follow_enabled(self.follow_button.GetValue(), notify=True)

    def render_settings(self):
        return (self.terrain_brightness, self.terrain_shading,
                self.terrain_wireframe, self.fpv_fov)

    def apply_render_settings(self, values, notify=False):
        brightness, shading, wireframe, fpvfov = values
        self.terrain_brightness = min(2.0, max(0.25, float(brightness)))
        self.terrain_shading = bool(shading)
        self.terrain_wireframe = bool(wireframe)
        self.set_fpv_fov(fpvfov)
        if self.terrain is not None:
            self.terrain.set_render_settings(
                self.terrain_brightness, self.terrain_shading,
                self.terrain_wireframe)
            self.render()
        if notify:
            self.state.event_queue.put(
                ('render_settings', self.terrain_brightness,
                 self.terrain_shading, self.terrain_wireframe, self.fpv_fov))

    def on_settings(self, event):
        original = self.render_settings()
        dialog = RenderSettingsDialog(
            self, original,
            lambda values: self.apply_render_settings(values, notify=False))
        result = dialog.ShowModal()
        if result == wx.ID_OK:
            self.apply_render_settings(dialog.values(), notify=True)
        else:
            self.apply_render_settings(original, notify=False)
        dialog.Destroy()

    def set_fpv_enabled(self, enable):
        enable = bool(enable)
        if enable == self.fpv_enabled:
            return
        self.fpv_enabled = enable
        self.fpv_button.SetValue(enable)
        self.follow_button.Enable(not enable)
        if self.elements is not None:
            self.elements.set_vehicle_visible(not enable)
        if self.tc is None:
            return

        camera = self.ren.GetActiveCamera()
        if enable:
            self.saved_map_view = (list(self.tc.focal), self.tc.dist,
                                   self.tc.yaw, self.tc.pitch,
                                   camera.GetViewAngle())
            self.style.interaction_enabled = False
            camera.SetViewAngle(self.fpv_fov)
            self.update_fpv_camera()
        else:
            self.style.interaction_enabled = True
            if self.saved_map_view is not None:
                focal, dist, yaw, pitch, fov = self.saved_map_view
                self.tc.look_at(focal, dist=dist, yaw=yaw, pitch=pitch)
                camera.SetViewAngle(fov)
                self.saved_map_view = None
            if self.follow and self.last_vehicle is not None:
                self.look_at_latlon(*self.last_vehicle)
            else:
                self.on_camera_change()

    def set_follow_enabled(self, enable, notify=False):
        self.follow = bool(enable)
        self.follow_button.SetValue(self.follow)
        if self.follow and not self.fpv_enabled and self.last_vehicle is not None:
            self.look_at_latlon(*self.last_vehicle)
        if notify:
            self.state.event_queue.put(('follow', self.follow))

    def set_fpv_fov(self, fov):
        self.fpv_fov = min(150.0, max(20.0, float(fov)))
        if self.fpv_enabled and self.tc is not None:
            self.ren.GetActiveCamera().SetViewAngle(self.fpv_fov)
            self.render()

    def update_fpv_camera(self, vehicle_enu=None):
        if not self.fpv_enabled or self.tc is None or self.last_vehicle_pose is None:
            return
        if vehicle_enu is None:
            vehicle_enu = self.elements.refresh_vehicle()
        if vehicle_enu is None:
            return
        (_, _, _, _roll, pitch, yaw) = self.last_vehicle_pose
        self.tc.set_fpv(vehicle_enu, yaw, pitch)
        self.ren.GetActiveCamera().SetViewAngle(self.fpv_fov)
        self.terrain.update(self.tc)
        self.render()

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
        if kind == 'fpvfov':
            self.set_fpv_fov(msg[1])
            return
        if kind == 'follow':
            self.set_follow_enabled(msg[1])
            return
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
            self.last_vehicle_pose = (lat, lon, amsl, roll, pitch, yaw)
            if self.fpv_enabled:
                self.update_fpv_camera(enu)
            elif self.follow:
                self.tc.look_at((enu[0], enu[1], enu[2]))
                self.terrain.update(self.tc)
        elif kind == 'path':
            self.elements.set_path(msg[1])
        elif kind == 'mission':
            self.elements.set_mission(msg[1])
        elif kind == 'fence':
            self.elements.set_fence(msg[1], self.terrain.height_at)
        elif kind == 'rally':
            self.elements.set_rally(msg[1])
        elif kind == 'lookat':
            (_, lat, lon, amsl, dist) = msg
            if not self.fpv_enabled:
                self.look_at_latlon(lat, lon, amsl, dist)
        elif kind == 'center':
            if self.last_vehicle:
                self.look_at_latlon(*self.last_vehicle)

    def on_timer(self, event):
        # drain parent messages
        drained = self.initial_render
        self.initial_render = False
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
                self.elements.refresh_fence(self.terrain.height_at)
                vehicle_enu = self.elements.refresh_vehicle()
                if self.fpv_enabled:
                    self.update_fpv_camera(vehicle_enu)
                if self.status_actor is not None and self.terrain.tiles:
                    self.ren.RemoveActor2D(self.status_actor)
                    self.status_actor = None
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
