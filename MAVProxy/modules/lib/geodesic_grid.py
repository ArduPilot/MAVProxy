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
and
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Math/AP_GeodesicGrid.cpp
as reference for defining the geodesic sections and implementing almost all
functions. Those files should be consulted for implementation details.
'''
import math

from pymavlink.rotmat import Matrix3, Vector3
# The golden number below was obtained from scipy.constants.golden. Let's use
# the literal value here as this is the only place that would require scipy
# module.
g = 1.618033988749895

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

class _NeighborUmbrella:
    index_to_attr = ('x', 'y', 'z')
    def __init__(self, components, v0_c0, v1_c1, v2_c1, v4_c4, v0_c4):
        self.components = components
        self.v0_c0 = self.index_to_attr[v0_c0]
        self.v1_c1 = self.index_to_attr[v1_c1]
        self.v2_c1 = self.index_to_attr[v2_c1]
        self.v4_c4 = self.index_to_attr[v4_c4]
        self.v0_c4 = self.index_to_attr[v0_c4]

_neighbor_umbrellas = (
    _NeighborUmbrella(( 9,  8,  7, 12, 14), 1, 2, 0, 0, 2),
    _NeighborUmbrella(( 1,  2,  4,  5,  3), 0, 0, 2, 2, 0),
    _NeighborUmbrella((16, 15, 13, 18, 17), 2, 2, 0, 2, 1),
)

_inverses = (
    Matrix3(
        Vector3(-0.309017,  0.500000,  0.190983),
        Vector3( 0.000000,  0.000000, -0.618034),
        Vector3(-0.309017, -0.500000,  0.190983)),
    Matrix3(
        Vector3(-0.190983,  0.309017, -0.500000),
        Vector3(-0.500000, -0.190983,  0.309017),
        Vector3( 0.309017, -0.500000, -0.190983)
    ),
    Matrix3(
        Vector3(-0.618034,  0.000000,  0.000000),
        Vector3( 0.190983, -0.309017, -0.500000),
        Vector3( 0.190983, -0.309017,  0.500000)
    ),
    Matrix3(
        Vector3(-0.500000,  0.190983, -0.309017),
        Vector3( 0.000000, -0.618034,  0.000000),
        Vector3( 0.500000,  0.190983, -0.309017)
    ),
    Matrix3(
        Vector3(-0.190983, -0.309017, -0.500000),
        Vector3(-0.190983, -0.309017,  0.500000),
        Vector3( 0.618034,  0.000000,  0.000000)
    ),
    Matrix3(
        Vector3(-0.309017, -0.500000, -0.190983),
        Vector3( 0.190983,  0.309017, -0.500000),
        Vector3( 0.500000, -0.190983,  0.309017)
    ),
    Matrix3(
        Vector3( 0.309017, -0.500000,  0.190983),
        Vector3( 0.000000,  0.000000, -0.618034),
        Vector3( 0.309017,  0.500000,  0.190983)
    ),
    Matrix3(
        Vector3( 0.190983, -0.309017, -0.500000),
        Vector3( 0.500000,  0.190983,  0.309017),
        Vector3(-0.309017,  0.500000, -0.190983)
    ),
    Matrix3(
        Vector3( 0.500000, -0.190983, -0.309017),
        Vector3( 0.000000,  0.618034,  0.000000),
        Vector3(-0.500000, -0.190983, -0.309017)
    ),
    Matrix3(
        Vector3( 0.309017,  0.500000, -0.190983),
        Vector3(-0.500000,  0.190983,  0.309017),
        Vector3(-0.190983, -0.309017, -0.500000)
    ),
)

_mid_inverses = (
    Matrix3(
        Vector3(-0.000000,  1.000000, -0.618034),
        Vector3( 0.000000, -1.000000, -0.618034),
        Vector3(-0.618034,  0.000000,  1.000000)
    ),
    Matrix3(
        Vector3(-1.000000,  0.618034, -0.000000),
        Vector3(-0.000000, -1.000000,  0.618034),
        Vector3( 0.618034, -0.000000, -1.000000)
    ),
    Matrix3(
        Vector3(-0.618034, -0.000000, -1.000000),
        Vector3( 1.000000, -0.618034, -0.000000),
        Vector3(-0.618034,  0.000000,  1.000000)
    ),
    Matrix3(
        Vector3(-1.000000, -0.618034, -0.000000),
        Vector3( 1.000000, -0.618034,  0.000000),
        Vector3(-0.000000,  1.000000, -0.618034)
    ),
    Matrix3(
        Vector3(-1.000000, -0.618034,  0.000000),
        Vector3( 0.618034,  0.000000,  1.000000),
        Vector3( 0.618034,  0.000000, -1.000000)
    ),
    Matrix3(
        Vector3(-0.618034, -0.000000, -1.000000),
        Vector3( 1.000000,  0.618034, -0.000000),
        Vector3( 0.000000, -1.000000,  0.618034)
    ),
    Matrix3(
        Vector3( 0.000000, -1.000000, -0.618034),
        Vector3( 0.000000,  1.000000, -0.618034),
        Vector3( 0.618034, -0.000000,  1.000000)
    ),
    Matrix3(
        Vector3( 1.000000, -0.618034, -0.000000),
        Vector3( 0.000000,  1.000000,  0.618034),
        Vector3(-0.618034,  0.000000, -1.000000)
    ),
    Matrix3(
        Vector3( 1.000000,  0.618034, -0.000000),
        Vector3(-1.000000,  0.618034,  0.000000),
        Vector3( 0.000000, -1.000000, -0.618034)
    ),
    Matrix3(
        Vector3(-0.000000,  1.000000,  0.618034),
        Vector3(-1.000000, -0.618034, -0.000000),
        Vector3( 0.618034,  0.000000, -1.000000)
    ),
)

def get_section_hit(v):
    i = _triangle_index(v)
    if i < 0:
        return -1

    j =  _subtriangle_index(i, v)
    if j < 0:
        return -1

    return 4 * i + j

def _neighbor_umbrella_component(idx, comp_idx):
    if idx < 3:
        return _neighbor_umbrellas[idx].components[comp_idx]
    return (_neighbor_umbrellas[idx % 3].components[comp_idx] + 10) % 20

def _from_neighbor_umbrella(idx, v, u):
    # TODO: fix equality tests for floating point values (here and in the rest
    # of this file)
    if u.x == u.y:
        comp = _neighbor_umbrella_component(idx, 0)
        w = _inverses[comp % 10] * v
        if comp > 9:
            w = -w
        x0 = getattr(w, _neighbor_umbrellas[idx % 3].v0_c0)
        if x0 <= 0:
            return -1
        return comp

    if u.y > u.x:
        comp = _neighbor_umbrella_component(idx, 1)
        w = _inverses[comp % 10] * v
        if comp > 9:
            w = -w

        x1 = getattr(w, _neighbor_umbrellas[idx % 3].v1_c1)
        x2 = getattr(w, _neighbor_umbrellas[idx % 3].v2_c1)

        if x1 == 0:
            return -1
        elif x1 < 0:
            return _neighbor_umbrella_component(idx, 2)

        if x2 == 0:
            return -1
        elif x2 < 0:
            return _neighbor_umbrella_component(idx, 0)

        return comp
    else:
        comp = _neighbor_umbrella_component(idx, 4)
        w = _inverses[comp % 10] * v
        if comp > 9:
            w = -w
        x4 = getattr(w, _neighbor_umbrellas[idx % 3].v4_c4)
        x0 = getattr(w, _neighbor_umbrellas[idx % 3].v0_c4)

        if x4 == 0:
            return -1
        elif x4 < 0:
            return _neighbor_umbrella_component(idx, 0)

        if x0 == 0:
            return -1
        elif x0 < 0:
            return _neighbor_umbrella_component(idx, 3)

        return comp

def _triangle_index(v):
    w = _inverses[0] * v
    zero_count = 0
    balance = 0
    umbrella = -1

    if w.x == 0:
        zero_count += 1
    elif w.x > 0:
        balance += 1
    else:
        balance -= 1

    if w.y == 0:
        zero_count += 1
    elif w.y > 0:
        balance += 1
    else:
        balance -= 1

    if w.z == 0:
        zero_count += 1
    elif w.z > 0:
        balance += 1
    else:
        balance -= 1

    if balance == 3:
        return 0
    elif balance == -3:
        return 10
    elif balance == 2:
        return -1
    elif balance == -2:
        return -1
    elif balance == 1:
        if zero_count == 2:
            return -1

        if w.x < 0:
            umbrella = 1
        elif w.y < 0:
            umbrella = 2
        else:
            umbrella = 0
    elif balance == -1:
        if zero_count == 2:
            return -1

        if w.x > 0:
            umbrella = 4
        elif w.y > 0:
            umbrella = 5
        else:
            umbrella = 3

        w = -w
    elif balance == 0:
        if zero_count == 3:
            return -1

        if w.x < 0:
            umbrella = 1
        elif w.y < 0:
            umbrella = 2
        else:
            umbrella = 0

    m = umbrella % 3

    if m == 0:
        w.z = -w.z
    elif m == 1:
        w.x, w.y, w.z = w.y, w.z, -w.x
    elif m == 2:
        w.x, w.y, w.z = w.z, w.x, -w.y

    return _from_neighbor_umbrella(umbrella, v, w, inclusive)

def _subtriangle_index(triangle_index, v):
    w = _mid_inverses[triangle_index % 10] * v
    if triangle_index > 9:
        w = -w

    if 0 in (w.x, w.y, w.z):
        return -1

    if w.x < 0:
        return 3
    if w.y < 0:
        return 1
    if w.z < 0:
        return 2

    return 0
