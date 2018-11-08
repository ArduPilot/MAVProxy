# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
from __future__ import division

import math
import os.path
import time

from OpenGL.GL import *
from pymavlink import quaternion
from pymavlink.rotmat import Vector3

from MAVProxy.modules import mavproxy_magical as magical
from MAVProxy.modules.lib import geodesic_grid
from MAVProxy.modules.lib import opengl
from MAVProxy.modules.lib import wavefront as wv
from MAVProxy.modules.lib.mp_util import quaternion_to_axis_angle
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.mavproxy_magical import glrenderer

class Renderer(glrenderer.Renderer):
    def __init__(self, background):
        super(Renderer, self).__init__(background)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.visible = [False for _ in range(len(geodesic_grid.sections))]

        self.common_model_transform = opengl.Transform()

        self.last_render_quaternion = quaternion.Quaternion(
                self.common_model_transform.quaternion)

        vertices = [(p.x, p.y, p.z) for t in geodesic_grid.sections for p in t]
        self.sphere = opengl.Object(vertices, enable_alpha=True)
        self.sphere.local.scale(0.46)
        self.sphere.model = self.common_model_transform

        self.base_color = Vector3(0.08, 0.68, 0.706667)

        self.vehicle = None

        path = os.path.join(magical.datapath, 'arrow.obj')
        obj = wv.ObjParser(filename=path).parse()
        self.mag = opengl.WavefrontObject(obj)
        self.mag.local.scale(.88)

    def get_load_progress(self):
        return self.vehicle_loader.get_progress()

    def rotate(self, vector, angle, rotate_mag=False):
        self.common_model_transform.rotate(vector, angle)
        if rotate_mag and self.mag:
            self.mag.model.rotate(vector, angle)

    def set_attitude(self, roll, pitch, yaw):
        self.common_model_transform.set_euler(roll, pitch, yaw)

    def set_mag(self, mag):
        if not mag.length():
            return
        m = self.common_model_transform.apply(mag).normalized()
        axis = Vector3(0, 0, 1) % m
        angle = math.acos(Vector3(0, 0, 1) * m)
        self.mag.model.set_rotation(axis, angle)

    def render(self):
        super(Renderer, self).render()

        if self.vehicle:
            self.vehicle.draw(self.program)

        self.sphere.material.set_color(1.2 * self.base_color)
        self.sphere.material.specular_exponent = 4
        self.sphere.material.alpha = 1.0
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        self.sphere.draw(self.program)

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        self.mag.draw(self.program)

        self.sphere.material.set_color(self.base_color)
        self.sphere.material.alpha = .4
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        faces = [i for i in range(80) if self.visible[i]]
        self.sphere.draw(self.program, faces=faces, camera=self.camera)

        self.last_render_quaternion = quaternion.Quaternion(
                self.common_model_transform.quaternion)

    def set_section_visible(self, section, visible=True):
        self.visible[section] = visible

    def clear_sections(self):
        for i in range(len(self.visible)):
            self.visible[i] = False

    def set_vehicle_wavefront(self, vehicle):
        self.vehicle = opengl.WavefrontObject(vehicle)
        # FIXME: this class shouldn't need to be aware of the proper scaling
        self.vehicle.local.scale(3.5)
        self.vehicle.model = self.common_model_transform

class GeodesicGrid(glrenderer.GLCanvas):
    def __init__(self, *k, **kw):
        super(GeodesicGrid, self).__init__(*k, **kw)

        self.vehicle_wavefront = None
        self.dragging = False

        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)
        self.Bind(wx.EVT_MOTION, self.OnMotion)

        self.attitude_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnAttitudeTimer, self.attitude_timer)
        self.attitude_timer.Start(40)

        self.gyro = Vector3(0, 0, 0)
        self.mag = Vector3(0, 0, 0)
        self.filtered_mag = Vector3(0, 0, 0)
        self.attitude_timer_last = 0
        self.attitude_timestamp = 0

    def CreateRenderer(self):
        r, g, b = self.GetParent().GetBackgroundColour()
        self.renderer = Renderer(
            background=(r / 255.0, g / 255.0, b / 255.0, 1),
        )
        if self.vehicle_wavefront:
            self.renderer.set_vehicle_wavefront(self.vehicle_wavefront)
            self.vehicle_wavefront = None

    def SetVehicleWavefront(self, vehicle):
        if self.renderer:
            self.SetCurrent(self.context)
            self.renderer.set_vehicle_wavefront(vehicle)
            self.Refresh()
            return
        self.vehicle_wavefront = vehicle

    def SetMag(self, x, y, z):
        self.mag = Vector3(x, y, z)

    def SetAttitude(self, roll, pitch, yaw, timestamp):
        if not self.renderer:
            return

        dt = 0xFFFFFFFF & (timestamp - self.attitude_timestamp)
        dt *= 0.001
        self.attitude_timestamp = timestamp

        desired_quaternion = quaternion.Quaternion((roll, pitch, yaw))
        desired_quaternion.normalize()
        error = desired_quaternion / self.renderer.common_model_transform.quaternion
        error.normalize()
        self.gyro = quaternion_to_axis_angle(error) * (1.0 / dt)

    def OnAttitudeTimer(self, evt):
        if not self.renderer:
            return

        if self.dragging:
            return

        t = time.time()
        dt = t - self.attitude_timer_last
        self.attitude_timer_last = t

        angle = self.gyro.length()
        angle *= dt
        self.renderer.rotate(self.gyro, angle)

        alpha = 0.8
        self.filtered_mag = alpha * self.filtered_mag + (1 - alpha) * self.mag
        self.renderer.set_mag(self.filtered_mag)

        q = self.renderer.common_model_transform.quaternion
        diff = q / self.renderer.last_render_quaternion
        angle = quaternion_to_axis_angle(diff).length()
        if angle < math.radians(.5):
            # Save some CPU time
            return

        self.Refresh()

    def CalcRotationVector(self, dx, dy):
        angle = math.degrees(math.atan2(dy, dx))
        # Make angle discrete on multiples of 45 degrees
        angle = angle + 22.5 - (angle + 22.5) % 45

        if angle % 90 == 45:
            x, y = 1, 1
        elif (angle / 90) % 2 == 0:
            x, y = 1, 0
        else:
            x, y = 0, 1

        if abs(angle) > 90:
            x *= -1
        if angle < 0:
            y *= -1

        # NED coordinates from the camera point of view
        dx, dy, dz = 0, x, y
        # Get rotation axis by rotating the moving vector -90 degrees on x axis
        self.rotation_vector = Vector3(dx, dz, -dy)

    def GetDeltaAngle(self):
        radius = 60.0
        pos = wx.GetMousePosition()
        d = pos - self.motion_reference
        self.motion_reference = pos

        self.CalcRotationVector(d.x, d.y)

        arc = math.sqrt(d.x**2 + d.y**2)
        return arc / radius

    def OnLeftDown(self, evt):
        self.motion_reference = wx.GetMousePosition()
        self.dragging = True

    def OnLeftUp(self, evt):
        self.dragging = False

    def OnMotion(self, evt):
        if hasattr(evt, 'ButtonIsDown'):
            left_button_down = evt.ButtonIsDown(wx.MOUSE_BTN_LEFT)
        else:
            left_button_down = evt.leftIsDown
        if not evt.Dragging() or not left_button_down:
            return

        angle = self.GetDeltaAngle()
        self.renderer.rotate(self.rotation_vector, angle, rotate_mag=True)

        self.Refresh()

    def UpdateVisibleSections(self, visible):
        for i, v in enumerate(visible):
            self.renderer.set_section_visible(i, v)
        self.Refresh()
