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
import time

from OpenGL.GL import *
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3

from MAVProxy.modules.lib import opengl
from MAVProxy.modules.lib.mp_util import quaternion_to_axis_angle
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.mavproxy_magical import glrenderer

from wx import glcanvas

class Renderer(glrenderer.Renderer):
    def __init__(self, background):
        super(Renderer, self).__init__(background)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)

        self.vehicle = None

    def render(self):
        super(Renderer, self).render()

        if self.vehicle:
            self.vehicle.draw(self.program)

    def set_vehicle_wavefront(self, vehicle):
        self.vehicle = opengl.WavefrontObject(vehicle)
        # FIXME: this class shouldn't need to be aware of the proper scaling
        self.vehicle.local.scale(4.4)

class Vehicle(glrenderer.GLCanvas):
    script_available_methods = (
        'SetAngvel',
        'SetEuler',
    )
    def __init__(self, *k, **kw):
        kw['attribList'] = (glcanvas.WX_GL_SAMPLES, 4)
        super(Vehicle, self).__init__(*k, **kw)

        self.context = glcanvas.GLContext(self)
        self.vehicle_wavefront = None

        self.script = None
        self.script_command = 0
        self.script_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnScriptTimer, self.script_timer)

        self.angvel = None
        self.desired_quaternion = None
        self.attitude_callback = None

        self.actuation_state = None
        self.actuation_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnActuationTimer, self.actuation_timer)

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

    def SetEuler(self, roll, pitch, yaw, callback=None):
        self.desired_quaternion = Quaternion((roll, pitch, yaw))
        self.attitude_callback = callback
        self.actuation_state = 'attitude'
        self.angvel = Vector3(1, 0, 0), 0
        self.StartActuation()

    def SetAngvel(self, axis, speed):
        if not isinstance(axis, Vector3):
            axis = Vector3(*axis)
        axis = self.renderer.vehicle.model.apply(axis)
        self.angvel = axis, speed
        self.actuation_state = 'angvel'
        self.StartActuation()

    def StartActuation(self):
        if not self.actuation_timer.IsRunning():
            self.actuation_timer.Start(40)

    def StopActuation(self):
        self.actuation_timer.Stop()

    def OnActuationTimer(self, evt):
        if not self.renderer.vehicle:
            return

        dt = evt.GetInterval() * .001
        axis, speed = self.angvel
        angle = speed * dt
        self.renderer.vehicle.model.rotate(axis, angle)

        if self.actuation_state == 'attitude':
            diff = self.desired_quaternion / self.renderer.vehicle.model.quaternion
            diff = quaternion_to_axis_angle(diff)
            angle = diff.length()
            if abs(angle) <= 0.001:
                self.renderer.vehicle.model.quaternion = self.desired_quaternion
                self.StopActuation()
                if self.attitude_callback:
                    self.attitude_callback()
                    self.attitude_callback = None
            else:
                speed = angle / dt
                self.angvel = Vector3(diff.x, diff.y, diff.z), speed * .3

        self.Refresh()

    def RunScript(self, script):
        '''
        Actuate on the vehicle through a script. The script is a sequence of
        commands. Each command is a sequence with the first element as the
        command name and the remaining values as parameters.

        The script is executed asynchronously by a timer with a period of 0.1
        seconds. At each timer tick, the next command is executed if it's ready
        for execution.

        If a command name starts with '.', then the remaining characters are a
        method name and the arguments are passed to the method call. For
        example, the command ('.SetEuler', 0, math.pi / 2, 0) sets the vehicle
        nose up. The methods available for that type of command are in the
        class property script_available_methods.

        The other possible commands are:
         - wait(sec): wait for sec seconds. Because of the timer period, some
           inaccuracy is expected depending on the value of sec.
         - restart: go back to the first command.

        Example::

            vehicle.RunScript((
                ('.SetEuler', 0, 0, 0), # Set vehicle level
                ('wait', 2), # Wait for 2 seconds
                # Rotate continuously on the x-axis at a rate of 90 degrees per
                # second
                ('.SetAngvel', (1, 0, 0), math.pi / 2),
                ('wait', 5), # Let the vehicle rotating for 5 seconds
                ('restart',), # Restart the script
            ))
        '''
        self.script = script
        self.script_command = 0
        self.script_command_start_time = 0
        self.script_command_state = 'ready'
        self.script_timer.Start(100)
        self.script_wait_time = 0

    def OnScriptTimer(self, evt):
        if not self.script or self.script_command >= len(self.script):
            self.StopScript()
            return

        cmd = self.script[self.script_command]
        name = cmd[0]
        args = cmd[1:]
        if self.script_command_state == 'ready':
            if name.startswith('.'):
                name = name[1:]
                if name not in self.script_available_methods:
                    raise Exception("Vehicle: script: invalid method name \"%s\"" % name)

                if name == 'SetEuler':
                    self.script_command_state = 'running'
                    def callback():
                        self.script_command += 1
                        self.script_command_state = 'ready'
                    args = tuple(args[:3]) + (callback,)
                else:
                    self.script_command += 1

                getattr(self, name)(*args)
            elif name == 'wait':
                self.script_wait_time = float(args[0]) if args else 1
                self.script_command_state = 'running'
            elif name == 'restart':
                self.script_command = 0
            else:
                raise Exception("Vehicle: script: invalid command \"%s\"" % name)
            self.script_command_start_time = time.time()

        elif self.script_command_state == 'running':
            if name == 'wait':
                t = time.time() - self.script_command_start_time
                if t >= self.script_wait_time:
                    self.script_command += 1
                    self.script_command_state = 'ready'


    def StopScript(self):
        self.script_timer.Stop()
        self.script = None
