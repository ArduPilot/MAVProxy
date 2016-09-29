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
'''
This module takes
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Math/AP_GeodesicGrid.h
as reference for defining the geodesic sections.
'''
import math

from pymavlink.rotmat import Vector3
from scipy.constants import golden as g

_first_half = (
    (Vector3(-g, 1, 0), Vector3(-1, 0,-g), Vector3(-g,-1, 0)),
    (Vector3(-1, 0,-g), Vector3(-g,-1, 0), Vector3( 0,-g,-1)),
    (Vector3(-g,-1, 0), Vector3( 0,-g,-1), Vector3( 0,-g, 1)),
    (Vector3(-1, 0,-g), Vector3( 0,-g,-1), Vector3( 1, 0,-g)),
    (Vector3( 0,-g,-1), Vector3( 0,-g, 1), Vector3( g,-1, 0)),
    (Vector3( 0,-g,-1), Vector3( 1, 0,-g), Vector3( g,-1, 0)),
    (Vector3( g,-1, 0), Vector3( 1, 0,-g), Vector3( g, 1, 0)),
    (Vector3( 1, 0,-g), Vector3( g, 1, 0), Vector3( 0, g,-1)),
    (Vector3( 1, 0,-g), Vector3( 0, g,-1), Vector3(-1, 0,-g)),
    (Vector3( 0, g,-1), Vector3(-g, 1, 0), Vector3(-1, 0,-g)),
)
_second_half = tuple((-a, -b, -c) for a, b, c in _first_half)

triangles = _first_half + _second_half

radius = math.sqrt(1 + g**2)

# radius / (length of two vertices of an icosahedron triangle)
_midpoint_projection_scale = radius / (2 * g)

sections = ()
for a, b, c in triangles:
    ma = _midpoint_projection_scale * (a + b)
    mb = _midpoint_projection_scale * (b + c)
    mc = _midpoint_projection_scale * (c + a)

    sections += (
        (ma, mb, mc),
        ( a, ma, mc),
        (ma,  b, mb),
        (mc, mb,  c),
    )

del _first_half, _second_half
