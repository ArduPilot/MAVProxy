'''
Cesium-like camera and interactor for the 3D map.

The camera keeps the horizon level at all times (no roll/flip): only yaw (about
world up) and pitch (look-down angle, clamped). Default mouse drag pans; Ctrl+drag
rotates (left-right = yaw, forward-back = pitch); wheel zooms.
'''

import math

import vtk


class TerrainCamera:
    def __init__(self, cam, focal, dist, yaw=0.0, pitch=45.0):
        self.cam = cam
        self.focal = list(focal)
        self.dist = dist
        self.yaw = yaw
        self.pitch = pitch
        self.pos = (0.0, 0.0, 0.0)
        self.apply()

    def apply(self):
        y = math.radians(self.yaw)
        p = math.radians(self.pitch)
        vd = (math.sin(y) * math.cos(p), math.cos(y) * math.cos(p), -math.sin(p))
        pos = (self.focal[0] - self.dist * vd[0],
               self.focal[1] - self.dist * vd[1],
               self.focal[2] - self.dist * vd[2])
        self.pos = pos
        self.cam.SetFocalPoint(*self.focal)
        self.cam.SetPosition(*pos)
        self.cam.SetViewUp(0, 0, 1)
        self.cam.SetClippingRange(max(1.0, self.dist * 0.005), self.dist * 60.0)

    def pan(self, dx, dy):
        y = math.radians(self.yaw)
        right = (math.cos(y), -math.sin(y), 0.0)
        fwd = (math.sin(y), math.cos(y), 0.0)
        s = self.dist * 0.0015
        self.focal[0] -= (right[0] * dx + fwd[0] * dy) * s
        self.focal[1] -= (right[1] * dx + fwd[1] * dy) * s
        self.apply()

    def rotate(self, dx, dy):
        self.yaw += dx * 0.3
        self.pitch = min(89.0, max(5.0, self.pitch - dy * 0.3))
        self.apply()

    def zoom(self, factor):
        self.dist = max(20.0, self.dist * factor)
        self.apply()

    def look_at(self, focal, dist=None, yaw=None, pitch=None):
        self.focal = list(focal)
        if dist is not None:
            self.dist = dist
        if yaw is not None:
            self.yaw = yaw
        if pitch is not None:
            self.pitch = pitch
        self.apply()


class TerrainStyle(vtk.vtkInteractorStyleUser):
    '''default drag = pan, Ctrl+drag = yaw/pitch, wheel = zoom'''
    def __init__(self, tc, on_change=None):
        self.tc = tc
        self.on_change = on_change
        self.last = None
        self.moved = False
        self.AddObserver("LeftButtonPressEvent", self._down)
        self.AddObserver("LeftButtonReleaseEvent", self._up)
        self.AddObserver("MouseMoveEvent", self._move)
        self.AddObserver("MouseWheelForwardEvent", self._wf)
        self.AddObserver("MouseWheelBackwardEvent", self._wb)

    def _render(self):
        self.GetInteractor().GetRenderWindow().Render()

    def _down(self, o, e):
        self.last = self.GetInteractor().GetEventPosition()
        self.moved = False

    def _up(self, o, e):
        self.last = None
        if self.moved and self.on_change:
            self.on_change()

    def _move(self, o, e):
        if self.last is None:
            return
        it = self.GetInteractor()
        x, y = it.GetEventPosition()
        dx, dy = x - self.last[0], y - self.last[1]
        self.last = (x, y)
        self.moved = True
        if it.GetControlKey():
            self.tc.rotate(dx, dy)
        else:
            self.tc.pan(dx, dy)
        self._render()

    def _wf(self, o, e):
        self.tc.zoom(1.0 / 1.2)
        if self.on_change:
            self.on_change()
        self._render()

    def _wb(self, o, e):
        self.tc.zoom(1.2)
        if self.on_change:
            self.on_change()
        self._render()
