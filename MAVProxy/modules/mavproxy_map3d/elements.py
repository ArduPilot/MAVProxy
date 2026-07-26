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
    '''lat/lon ring approximating a circle of radius metres about centre'''
    (lat, lon) = centre
    dlat = math.degrees(max(0.0, float(radius)) / R)
    coslat = math.cos(math.radians(lat))
    dlon = dlat / coslat if abs(coslat) > 1.0e-9 else 0.0
    points = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        points.append((lat + dlat * math.cos(angle),
                       lon + dlon * math.sin(angle)))
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


def _vehicle_icon():
    '''Return an opaque aircraft glyph in the local horizontal plane.

    Using native geometry avoids platform-dependent PNG alpha/texture issues.
    The outline coordinates describe a unit aircraft pointing towards +Y.
    '''
    outline = [
        (0.00, 0.50), (-0.07, 0.24), (-0.48, 0.02), (-0.48, -0.07),
        (-0.08, 0.00), (-0.07, -0.30), (-0.22, -0.43), (-0.22, -0.50),
        (0.00, -0.42), (0.22, -0.50), (0.22, -0.43), (0.07, -0.30),
        (0.08, 0.00), (0.48, -0.07), (0.48, 0.02), (0.07, 0.24),
    ]
    points = vtk.vtkPoints()
    polygon = vtk.vtkPolygon()
    polygon.GetPointIds().SetNumberOfIds(len(outline))
    for i, (x, y) in enumerate(outline):
        points.InsertNextPoint(x, y, 0.0)
        polygon.GetPointIds().SetId(i, i)
    cells = vtk.vtkCellArray()
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
            self.vehicle = _vehicle_icon()
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
