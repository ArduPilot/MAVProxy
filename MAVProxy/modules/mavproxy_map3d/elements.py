'''
VTK actors for the map elements drawn over the 3D terrain: flight path / trail,
mission, fence, KML, rally points and the vehicle. All positions are converted
to the same local ENU frame used by the terrain (origin lat0,lon0, up = AMSL *
zexag).
'''

import math

import vtk

from MAVProxy.modules.mavproxy_map3d.terrain import enu, R

# MAV_FRAME altitude conventions
FRAME_GLOBAL = (0, 5)            # AMSL
FRAME_RELATIVE = (3, 6)          # relative to home
FRAME_TERRAIN = (10, 11)         # above terrain
FENCE_CLEARANCE = 3.0
FENCE_SAMPLE_SPACING = 20.0
FENCE_MAX_SAMPLES_PER_EDGE = 1000
FENCE_CIRCLE_SEGMENTS = 64


def circle_latlon(centre, radius, segments=FENCE_CIRCLE_SEGMENTS):
    '''lat/lon ring approximating a circle of radius metres about centre.

    Great-circle destination points, so the ring stays a circle at high latitude
    instead of blowing up as a flat-earth 1/cos(lat) would. Longitudes are left
    unwrapped (centre longitude plus an offset) so the ring stays continuous for
    the ENU projection rather than jumping at the 180th meridian.
    '''
    (lat, lon) = centre
    delta = max(0.0, float(radius)) / R
    lat1 = math.radians(lat)
    (sin_lat1, cos_lat1) = (math.sin(lat1), math.cos(lat1))
    (sin_d, cos_d) = (math.sin(delta), math.cos(delta))
    if abs(cos_lat1) < 1.0e-12:
        # exactly on a pole the bearing terms cancel, so sweep longitude instead
        polar_lat = math.degrees(math.copysign(math.pi / 2 - delta, lat1))
        return [(polar_lat, lon - 180.0 + 360.0 * i / segments)
                for i in range(segments)]
    points = []
    for i in range(segments):
        bearing = 2.0 * math.pi * i / segments
        sin_lat2 = min(1.0, max(-1.0, sin_lat1 * cos_d +
                                cos_lat1 * sin_d * math.cos(bearing)))
        lat2 = math.asin(sin_lat2)
        dlon = math.atan2(math.sin(bearing) * sin_d * cos_lat1,
                          cos_d - sin_lat1 * sin_lat2)
        points.append((math.degrees(lat2), lon + math.degrees(dlon)))
    return points


def _polyline(points_enu, colour, width, dashed=False):
    vpts = vtk.vtkPoints()
    line = vtk.vtkPolyLine()
    line.GetPointIds().SetNumberOfIds(len(points_enu))
    for i, p in enumerate(points_enu):
        vpts.InsertNextPoint(*p)
        line.GetPointIds().SetId(i, i)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(line)
    poly = vtk.vtkPolyData()
    poly.SetPoints(vpts)
    poly.SetLines(cells)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    mapper.SetResolveCoincidentTopologyToPolygonOffset()
    mapper.SetRelativeCoincidentTopologyLineOffsetParameters(-1.0, -1.0)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*colour)
    actor.GetProperty().SetLineWidth(width)
    actor.GetProperty().SetLighting(False)
    if dashed:
        actor.GetProperty().SetLineStipplePattern(0xF0F0)
    return actor


def _circle(cx, cy, radius, segments=20):
    '''polygon approximating a circle'''
    return [(cx + radius * math.sin(2 * math.pi * i / segments),
             cy + radius * math.cos(2 * math.pi * i / segments))
            for i in range(segments)]


def _ellipse(cx, cy, rx, ry, segments=24):
    return [(cx + rx * math.sin(2 * math.pi * i / segments),
             cy + ry * math.cos(2 * math.pi * i / segments))
            for i in range(segments)]


def _bar(p0, p1, width):
    '''rectangle of the given width running along p0 -> p1'''
    ((x0, y0), (x1, y1)) = (p0, p1)
    (dx, dy) = (x1 - x0, y1 - y0)
    length = math.hypot(dx, dy)
    (ox, oy) = (-dy / length * width * 0.5, dx / length * width * 0.5)
    return [(x0 + ox, y0 + oy), (x1 + ox, y1 + oy),
            (x1 - ox, y1 - oy), (x0 - ox, y0 - oy)]


def _plane_shape():
    return [[
        (0.00, 0.50), (-0.07, 0.24), (-0.48, 0.02), (-0.48, -0.07),
        (-0.08, 0.00), (-0.07, -0.30), (-0.22, -0.43), (-0.22, -0.50),
        (0.00, -0.42), (0.22, -0.50), (0.22, -0.43), (0.07, -0.30),
        (0.08, 0.00), (0.48, -0.07), (0.48, 0.02), (0.07, 0.24),
    ]]


def _copter_shape():
    '''motors on X arms, with a forward pointing body so heading reads'''
    shapes = []
    arm = 0.32
    for (x, y) in ((-arm, arm), (arm, arm), (-arm, -arm), (arm, -arm)):
        shapes.append(_bar((0.0, 0.0), (x, y), 0.09))
        shapes.append(_circle(x, y, 0.16))
    shapes.append([(0.00, 0.36), (0.15, -0.06), (0.00, -0.18), (-0.15, -0.06)])
    return shapes


def _singlecopter_shape():
    '''ducted rotor with control vanes below it'''
    return [_circle(0.0, 0.06, 0.32),
            [(0.00, 0.50), (0.15, 0.22), (-0.15, 0.22)],
            _bar((-0.34, -0.30), (0.34, -0.30), 0.12),
            _bar((0.0, -0.44), (0.0, -0.16), 0.12)]


def _heli_shape():
    '''crossed rotor blades, cabin, and a tail boom reaching past the disc'''
    return [_bar((-0.34, 0.46), (0.34, -0.22), 0.05),
            _bar((-0.34, -0.22), (0.34, 0.46), 0.05),
            _ellipse(0.0, 0.16, 0.14, 0.26),
            _bar((0.0, 0.16), (0.0, -0.48), 0.07),
            _bar((-0.13, -0.44), (0.13, -0.44), 0.20)]


def _rover_shape():
    '''car body with the wheels clear of it in silhouette'''
    return [[(0.00, 0.46), (0.16, 0.30), (0.16, -0.44), (-0.16, -0.44),
             (-0.16, 0.30)],
            _bar((-0.30, 0.30), (-0.30, 0.04), 0.16),
            _bar((0.30, 0.30), (0.30, 0.04), 0.16),
            _bar((-0.30, -0.14), (-0.30, -0.40), 0.16),
            _bar((0.30, -0.14), (0.30, -0.40), 0.16)]


def _boat_shape():
    '''pointed bow and square transom, with a beam across midships'''
    return [[(0.00, 0.50), (0.16, 0.10), (0.16, -0.40), (-0.16, -0.40),
             (-0.16, 0.10)],
            _bar((-0.34, 0.02), (0.34, 0.02), 0.10)]


def _sub_shape():
    '''slim hull with bow planes and tail fins'''
    return [_ellipse(0.0, 0.02, 0.12, 0.44),
            _bar((-0.20, 0.14), (0.20, 0.14), 0.14),
            _bar((-0.26, -0.36), (0.26, -0.36), 0.09)]


def _antenna_shape():
    '''a dish on a mast, pointing the way the tracker is aimed'''
    return [[(0.00, 0.08), (0.34, 0.44), (0.20, 0.50), (-0.20, 0.50),
             (-0.34, 0.44)],
            _bar((0.0, 0.16), (0.0, -0.30), 0.09),
            _bar((-0.26, -0.42), (0.26, -0.42), 0.16)]


def _blimp_shape():
    '''fat envelope with large tail fins'''
    return [_ellipse(0.0, 0.06, 0.22, 0.42),
            _bar((-0.34, -0.30), (0.34, -0.30), 0.12)]


# keyed by the names mp_util.vehicle_type_name() returns
VEHICLE_SHAPES = {
    'plane': _plane_shape,
    'copter': _copter_shape,
    'singlecopter': _singlecopter_shape,
    'heli': _heli_shape,
    'rover': _rover_shape,
    'boat': _boat_shape,
    'sub': _sub_shape,
    'antenna': _antenna_shape,
    'blimp': _blimp_shape,
}
DEFAULT_VEHICLE_TYPE = 'plane'


def _vehicle_icon(vehicle_type=DEFAULT_VEHICLE_TYPE):
    '''Return an opaque vehicle glyph in the local horizontal plane.

    Using native geometry avoids platform-dependent PNG alpha/texture issues.
    The outlines describe a unit vehicle pointing towards +Y. The glyph is a
    single colour, so overlapping shapes just merge into one silhouette.
    '''
    shape = VEHICLE_SHAPES.get(vehicle_type, VEHICLE_SHAPES[DEFAULT_VEHICLE_TYPE])
    points = vtk.vtkPoints()
    cells = vtk.vtkCellArray()
    for outline in shape():
        polygon = vtk.vtkPolygon()
        polygon.GetPointIds().SetNumberOfIds(len(outline))
        for i, (x, y) in enumerate(outline):
            polygon.GetPointIds().SetId(i, points.InsertNextPoint(x, y, 0.0))
        cells.InsertNextCell(polygon)
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputData(poly)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(triangles.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetLighting(False)
    actor.GetProperty().SetAmbient(1.0)
    actor.GetProperty().SetDiffuse(0.0)
    actor.GetProperty().SetColor(1.0, 0.05, 0.0)
    actor.ForceOpaqueOn()

    # Keep the pipeline alive for the lifetime of the actor.
    actor._map3d_triangles = triangles
    return actor


def _points(points_enu, colour, size):
    vpts = vtk.vtkPoints()
    verts = vtk.vtkCellArray()
    for i, p in enumerate(points_enu):
        vpts.InsertNextPoint(*p)
        verts.InsertNextCell(1, [i])
    poly = vtk.vtkPolyData()
    poly.SetPoints(vpts)
    poly.SetVerts(verts)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*colour)
    actor.GetProperty().SetPointSize(size)
    actor.GetProperty().SetLighting(False)
    return actor


class ElementManager:
    def __init__(self, renderer, lat0, lon0, zexag):
        self.ren = renderer
        self.lat0 = lat0
        self.lon0 = lon0
        self.zexag = zexag
        self.home_amsl = 0.0
        self.actors = {}            # key -> list of actors
        self.trail = []             # accumulated live positions (enu)
        self.vehicle = None
        self.vehicle_pose = None
        self.vehicle_visible = True
        self.vehicle_type = DEFAULT_VEHICLE_TYPE
        self.terrain_height = None
        self.fence = []
        self.fence_geometry = None
        self.kml_features = []
        self.kml_geometry = None
        self.kml_height_cache = {}

    def _enu(self, lat, lon, amsl):
        e, n, u = enu(lat, lon, amsl, self.lat0, self.lon0)
        return (e, n, u * self.zexag)

    def _resolve_amsl(self, alt, frame):
        if frame in FRAME_GLOBAL:
            return alt
        # relative-to-home and terrain both referenced to home AMSL in v1
        return self.home_amsl + alt

    def _replace(self, key, actors):
        for a in self.actors.get(key, []):
            self.ren.RemoveActor(a)
        for a in actors:
            self.ren.AddActor(a)
        self.actors[key] = actors

    def set_home(self, amsl):
        self.home_amsl = amsl
        if self.fence:
            self.refresh_fence()
        if self.kml_features:
            self.refresh_kml()

    def set_terrain_height(self, terrain_height):
        self.terrain_height = terrain_height

    def set_vehicle_visible(self, visible):
        self.vehicle_visible = bool(visible)
        if self.vehicle is not None:
            self.vehicle.SetVisibility(self.vehicle_visible)

    def set_vehicle_type(self, vehicle_type):
        '''vehicle_type is a name from mp_util.vehicle_type_name()'''
        if vehicle_type not in VEHICLE_SHAPES or vehicle_type == self.vehicle_type:
            return
        self.vehicle_type = vehicle_type
        if self.vehicle is not None:
            self.ren.RemoveActor(self.vehicle)
            self.vehicle = None
            self.refresh_vehicle()

    def set_path(self, path):
        '''path: list of (lat,lon,amsl)'''
        if not path:
            return
        pts = [self._enu(lat, lon, a + 2.0) for (lat, lon, a) in path]
        self._replace('path', [_polyline(pts, (1.0, 0.0, 0.7), 2.5)])

    def add_trail_point(self, lat, lon, amsl):
        self.trail.append(self._enu(lat, lon, amsl))
        if len(self.trail) > 5000:
            self.trail = self.trail[-5000:]
        if len(self.trail) >= 2:
            self._replace('trail', [_polyline(self.trail, (1.0, 1.0, 0.0), 2.0)])

    def set_mission(self, items):
        '''items: list of (lat,lon,z,frame,command,seq)'''
        line = []
        markers = []
        for (lat, lon, z, frame, command, seq) in items:
            if lat == 0 and lon == 0:
                continue
            amsl = self._resolve_amsl(z, frame)
            p = self._enu(lat, lon, amsl)
            line.append(p)
            markers.append(p)
        actors = []
        if len(line) >= 2:
            actors.append(_polyline(line, (1.0, 1.0, 1.0), 2.0, dashed=True))
        if markers:
            actors.append(_points(markers, (1.0, 1.0, 1.0), 9))
        self._replace('mission', actors)

    def _terrain_samples(self, points, closed):
        '''Densify a lat/lon polyline so it follows terrain between vertices.'''
        points = list(points)
        if closed and len(points) > 2 and points[0] == points[-1]:
            points = points[:-1]
        segment_count = len(points) if closed else len(points) - 1
        for i in range(max(0, segment_count)):
            lat1, lon1 = points[i]
            lat2, lon2 = points[(i + 1) % len(points)]
            e1, n1, _ = enu(lat1, lon1, 0.0, self.lat0, self.lon0)
            e2, n2, _ = enu(lat2, lon2, 0.0, self.lat0, self.lon0)
            distance = math.hypot(e2 - e1, n2 - n1)
            count = max(1, int(math.ceil(distance / FENCE_SAMPLE_SPACING)))
            count = min(count, FENCE_MAX_SAMPLES_PER_EDGE)
            for j in range(count):
                t = float(j) / count
                yield (lat1 + (lat2 - lat1) * t,
                       lon1 + (lon2 - lon1) * t)
        if points:
            yield points[0] if closed else points[-1]

    def _terrain_draped_line(self, points, closed, height_cache=None):
        fallback = self.home_amsl * self.zexag
        clearance = FENCE_CLEARANCE * self.zexag
        line = []
        for lat, lon in self._terrain_samples(points, closed):
            e, n, _ = enu(lat, lon, 0.0, self.lat0, self.lon0)
            cache_key = (lat, lon)
            ground = (height_cache.get(cache_key)
                      if height_cache is not None else None)
            if ground is None and self.terrain_height is not None:
                ground = self.terrain_height(lat, lon)
                if ground is not None and height_cache is not None:
                    height_cache[cache_key] = ground
            if ground is None:
                ground = fallback
            line.append((e, n, ground + clearance))
        return line

    def refresh_fence(self, terrain_height=None):
        '''Rebuild the fence shapes just above the currently loaded terrain.

        terrain_height returns rendered world Z for a lat/lon, or None where
        terrain has not arrived yet. In that case home AMSL is a safe temporary
        fallback; the UI calls this again whenever terrain tiles arrive.
        '''
        if terrain_height is not None:
            self.set_terrain_height(terrain_height)
        geometry = []
        for shape in self.fence:
            if shape[0] == 'circle':
                (_, centre, radius, colour) = shape
                points = circle_latlon(centre, radius)
            else:
                (_, points, colour) = shape
            if len(points) < 2:
                continue
            ring = self._terrain_draped_line(points, closed=True)
            if len(ring) >= 2:
                geometry.append((ring, colour))

        if geometry == self.fence_geometry:
            return
        self.fence_geometry = geometry
        self._replace('fence', [_polyline(ring, colour, 2.0)
                                for (ring, colour) in geometry])

    def set_fence(self, shapes, terrain_height=None):
        '''shapes: list of ('polygon', [(lat,lon), ...], rgb) or
        ('circle', (lat,lon), radius_m, rgb), each clamped above loaded terrain'''
        self.fence = list(shapes)
        if terrain_height is not None:
            self.set_terrain_height(terrain_height)
        self.refresh_fence()

    def refresh_kml(self, terrain_height=None):
        '''Rebuild visible KML lines just above the loaded terrain.'''
        if terrain_height is not None:
            self.set_terrain_height(terrain_height)
        geometry = []
        for key, points, colour, width in self.kml_features:
            closed = len(points) > 2 and points[0] == points[-1]
            line = self._terrain_draped_line(
                points, closed, self.kml_height_cache)
            if len(line) >= 2:
                geometry.append((key, line, colour, width))
        if geometry == self.kml_geometry:
            return
        self.kml_geometry = geometry
        actors = [_polyline(line, colour, width)
                  for _key, line, colour, width in geometry]
        self._replace('kml', actors)

    def set_kml(self, features, terrain_height=None, refresh=True):
        '''features: visible (key, [(lat,lon)], RGB colour, line width).'''
        self.kml_features = list(features)
        if terrain_height is not None:
            self.set_terrain_height(terrain_height)
        self.kml_geometry = None
        if not self.kml_features:
            self.kml_height_cache = {}
            self.kml_geometry = []
            self._replace('kml', [])
        elif refresh:
            self.refresh_kml()

    def set_rally(self, pts):
        '''pts: list of (lat,lon,alt_rel)'''
        if not pts:
            self._replace('rally', [])
            return
        markers = [self._enu(lat, lon, self.home_amsl + alt) for (lat, lon, alt) in pts]
        self._replace('rally', [_points(markers, (0.4, 0.8, 1.0), 12)])

    def refresh_vehicle(self):
        if self.vehicle_pose is None:
            return None
        lat, lon, amsl, roll, pitch, yaw = self.vehicle_pose
        if self.vehicle is None:
            self.vehicle = _vehicle_icon(self.vehicle_type)
            self.vehicle.SetVisibility(self.vehicle_visible)
            self.ren.AddActor(self.vehicle)
        e, n, u = self._enu(lat, lon, amsl)
        if self.terrain_height is not None:
            ground = self.terrain_height(lat, lon)
            if ground is not None:
                u = max(u, ground + 5.0 * self.zexag)
        self.vehicle.SetPosition(e, n, u)
        camera = self.ren.GetActiveCamera()
        cx, cy, cz = camera.GetPosition()
        distance = math.sqrt((cx - e) ** 2 + (cy - n) ** 2 + (cz - u) ** 2)
        # Scale with camera distance for a stable screen size. These values are
        # half the original marker size.
        display_size = max(22.5, distance * 0.03)
        self.vehicle.SetScale(display_size)
        # The glyph starts level, pointing north (+Y). Apply the aircraft
        # attitude in the local east/north/up frame so a level aircraft remains
        # parallel to the terrain rather than facing the camera.
        self.vehicle.SetOrientation(0.0, 0.0, 0.0)
        self.vehicle.RotateZ(-math.degrees(yaw))
        self.vehicle.RotateX(math.degrees(pitch))
        self.vehicle.RotateY(math.degrees(roll))
        return (e, n, u)

    def set_vehicle(self, lat, lon, amsl, roll, pitch, yaw):
        self.vehicle_pose = (lat, lon, amsl, roll, pitch, yaw)
        return self.refresh_vehicle()
