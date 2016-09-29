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
from OpenGL.GL import *
from pymavlink.rotmat import Vector3

from MAVProxy.modules.lib import opengl
from MAVProxy.modules.lib.wx_loader import wx

from wx import glcanvas

class Renderer(object):
    '''
    Base class for rendering 3D opengl objects in Magical UI. It preforms
    common routines for the renderers used. The method render() should perform
    the rendering routine. The default implementation only does basic setup and
    can be called from the subclass.
    '''
    def __init__(self, background):
        glClearColor(*background)

        self.program = opengl.Program()
        self.program.compile_and_link()

        self.program.use_light(opengl.Light(
            position=Vector3(-2.0, 0.0, -1.0),
            ambient=Vector3(0.8, 0.8, 0.8),
            diffuse=Vector3(0.5, 0.5, 0.5),
            specular=Vector3(0.25, 0.25, 0.25),
            att_linear=0.000,
            att_quad=0.000,
        ))

        self.camera = opengl.Camera()
        # Adjust camera to show NED coordinates properly
        self.camera.base = (
            Vector3( 0, 1, 0),
            Vector3( 0, 0,-1),
            Vector3(-1, 0, 0),
        )
        self.camera.position = Vector3(-100.0, 0, 0)

        near = -self.camera.position.x - 1.0
        far = near + 2.0
        self.projection = opengl.Orthographic(near=near, far=far)
        self.program.use_projection(self.projection)

    def set_viewport(self, viewport):
        glViewport(*viewport)

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.program.use_camera(self.camera)

class GLCanvas(glcanvas.GLCanvas):
    def __init__(self, *k, **kw):
        kw['attribList'] = (glcanvas.WX_GL_SAMPLES, 4)
        super(GLCanvas, self).__init__(*k, **kw)
        self.context = glcanvas.GLContext(self)
        self.renderer = None

        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def CreateRenderer(self):
        raise NotImplementedError()

    def OnPaint(self, evt):
        w, h = self.GetClientSize()
        e = min(w, h)
        if not e:
            return

        self.SetCurrent(self.context)

        if not self.renderer:
            self.CreateRenderer()

        x, y = int((w - e) / 2), int((h - e) / 2)
        self.renderer.set_viewport((x, y, e, e))
        self.renderer.render()

        self.SwapBuffers()
