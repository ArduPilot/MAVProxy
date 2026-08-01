'''
Cesium-like camera and interactor for the 3D map.

The camera keeps the horizon level at all times (no roll/flip): only yaw (about
world up) and pitch (look-down angle, clamped). Default mouse drag pans; Ctrl+drag
or a right drag rotates (left-right = yaw, forward-back = pitch); wheel zooms.
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

    def set_fpv(self, position, yaw, pitch, lookahead=5000.0):
        '''Place the camera at the vehicle and look along its yaw/pitch.

        yaw and pitch are radians from MAVLink. Roll is deliberately omitted;
        world-up remains camera-up so the FPV horizon is always level.
        '''
        pitch = min(math.radians(85.0), max(math.radians(-85.0), pitch))
        cp = math.cos(pitch)
        direction = (math.sin(yaw) * cp,
                     math.cos(yaw) * cp,
                     math.sin(pitch))
        self.pos = tuple(position)
        self.focal = [position[i] + lookahead * direction[i] for i in range(3)]
        self.dist = lookahead
        self.yaw = math.degrees(yaw)
        self.pitch = math.degrees(pitch)
        self.cam.SetPosition(*self.pos)
        self.cam.SetFocalPoint(*self.focal)
        self.cam.SetViewUp(0, 0, 1)
        self.cam.SetClippingRange(0.5, lookahead * 100.0)


class TerrainStyle(vtk.vtkInteractorStyleUser):
    '''default drag = pan, Ctrl+drag or right drag = yaw/pitch, wheel = zoom'''
    def __init__(self, tc, on_change=None):
        self.tc = tc
        self.on_change = on_change
        self.last = None
        self.moved = False
        self.rotating = False
        self.active_button = None
        # Do not call this "enabled": vtkInteractorStyle exposes an enabled
        # property that attempts to enable the style immediately.
        self.interaction_enabled = True
        self.AddObserver("LeftButtonPressEvent", self._down_pan)
        self.AddObserver("LeftButtonReleaseEvent", self._up_left)
        # macOS turns Ctrl+left into a right click, so the Ctrl+drag above
        # never reaches us there; rotate on a right drag as well
        self.AddObserver("RightButtonPressEvent", self._down_rotate)
        self.AddObserver("RightButtonReleaseEvent", self._up_right)
        self.AddObserver("MouseMoveEvent", self._move)
        self.AddObserver("MouseWheelForwardEvent", self._wf)
        self.AddObserver("MouseWheelBackwardEvent", self._wb)

    def _render(self):
        self.GetInteractor().GetRenderWindow().Render()

    def cancel_drag(self):
        '''forget any drag in progress. Without this a drag left unfinished
        (interaction disabled part way through, or a release we never saw)
        would keep moving the camera on plain mouse motion'''
        self.last = None
        self.moved = False
        self.rotating = False
        self.active_button = None

    def _begin(self, button, rotating):
        # one button owns the drag: a second press must not switch pan/rotate
        # under the first, and its release must not end the first drag
        if not self.interaction_enabled or self.active_button is not None:
            return
        self.active_button = button
        self.rotating = rotating
        self.last = self.GetInteractor().GetEventPosition()
        self.moved = False

    def _end(self, button):
        if self.active_button != button:
            return
        moved = self.moved
        enabled = self.interaction_enabled
        self.cancel_drag()
        if enabled and moved and self.on_change:
            self.on_change()

    def _down_pan(self, o, e):
        self._begin('left', False)

    def _down_rotate(self, o, e):
        self._begin('right', True)

    def _up_left(self, o, e):
        self._end('left')

    def _up_right(self, o, e):
        self._end('right')

    def _move(self, o, e):
        if not self.interaction_enabled or self.last is None:
            return
        it = self.GetInteractor()
        x, y = it.GetEventPosition()
        dx, dy = x - self.last[0], y - self.last[1]
        self.last = (x, y)
        self.moved = True
        if self.rotating or it.GetControlKey():
            self.tc.rotate(dx, dy)
        else:
            self.tc.pan(dx, dy)
        self._render()

    def _wf(self, o, e):
        if not self.interaction_enabled:
            return
        self.tc.zoom(1.0 / 1.2)
        if self.on_change:
            self.on_change()
        self._render()

    def _wb(self, o, e):
        if not self.interaction_enabled:
            return
        self.tc.zoom(1.2)
        if self.on_change:
            self.on_change()
        self._render()
