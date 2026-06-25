'''
VTK actors for the map elements drawn over the 3D terrain: flight path / trail,
mission, fence, rally points and the vehicle. All positions are converted to the
same local ENU frame used by the terrain (origin lat0,lon0, up = AMSL * zexag).
'''

import math

import vtk

from MAVProxy.modules.mavproxy_map3d.terrain import enu

# MAV_FRAME altitude conventions
FRAME_GLOBAL = (0, 5)            # AMSL
FRAME_RELATIVE = (3, 6)          # relative to home
FRAME_TERRAIN = (10, 11)         # above terrain


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
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*colour)
    actor.GetProperty().SetLineWidth(width)
    actor.GetProperty().SetLighting(False)
    if dashed:
        actor.GetProperty().SetLineStipplePattern(0xF0F0)
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

    def set_fence(self, pts):
        '''pts: list of (lat,lon) polygon (drawn near ground)'''
        if len(pts) < 2:
            self._replace('fence', [])
            return
        ring = [self._enu(lat, lon, self.home_amsl + 1.0) for (lat, lon) in pts]
        ring.append(ring[0])
        self._replace('fence', [_polyline(ring, (0.0, 1.0, 0.0), 2.0)])

    def set_rally(self, pts):
        '''pts: list of (lat,lon,alt_rel)'''
        if not pts:
            self._replace('rally', [])
            return
        markers = [self._enu(lat, lon, self.home_amsl + alt) for (lat, lon, alt) in pts]
        self._replace('rally', [_points(markers, (0.4, 0.8, 1.0), 12)])

    def set_vehicle(self, lat, lon, amsl, roll, pitch, yaw):
        if self.vehicle is None:
            cone = vtk.vtkConeSource()
            cone.SetHeight(40.0)
            cone.SetRadius(14.0)
            cone.SetResolution(16)
            cone.SetDirection(0, 1, 0)         # base orientation: north
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cone.GetOutputPort())
            self.vehicle = vtk.vtkActor()
            self.vehicle.SetMapper(mapper)
            self.vehicle.GetProperty().SetColor(1.0, 0.2, 0.0)
            self.ren.AddActor(self.vehicle)
        e, n, u = self._enu(lat, lon, amsl)
        self.vehicle.SetPosition(e, n, u)
        # yaw about Z (heading), then pitch/roll best-effort
        self.vehicle.SetOrientation(0, 0, 0)
        self.vehicle.RotateZ(-math.degrees(yaw))
        self.vehicle.RotateX(math.degrees(pitch))
        self.vehicle.RotateY(math.degrees(roll))
        return (e, n, u)
