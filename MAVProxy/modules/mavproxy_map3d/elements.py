'''
VTK actors for the map elements drawn over the 3D terrain: flight path / trail,
mission, fence, KML, rally points and the vehicle. All positions are converted
to the same local ENU frame used by the terrain (origin lat0,lon0, up = AMSL *
zexag).
'''

import math

import vtk

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
from MAVProxy.modules.mavproxy_map3d.terrain import enu, R

# MAV_FRAME altitude conventions
FRAME_GLOBAL = (0, 5)            # AMSL
FRAME_RELATIVE = (3, 6)          # relative to home
FRAME_TERRAIN = (10, 11)         # above terrain
FENCE_CLEARANCE = 3.0
FENCE_SAMPLE_SPACING = 20.0
FENCE_MAX_SAMPLES_PER_EDGE = 1000
FENCE_CIRCLE_SEGMENTS = 64
MISSION_CIRCLE_SEGMENTS = 64
# roughly how many direction-of-travel arrows to spread along the mission,
# and how many pixels each should take up on the screen
MISSION_ARROW_COUNT = 60
MISSION_ARROW_PIXELS = 22.0


def ring_points(centre, radius, bearings):
    '''lat/lon points radius metres from centre at the given bearings (radians).

    Great-circle destination points, so a ring stays a circle at high latitude
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
        return [(polar_lat, lon - 180.0 + math.degrees(bearing))
                for bearing in bearings]
    points = []
    for bearing in bearings:
        sin_lat2 = min(1.0, max(-1.0, sin_lat1 * cos_d +
                                cos_lat1 * sin_d * math.cos(bearing)))
        lat2 = math.asin(sin_lat2)
        dlon = math.atan2(math.sin(bearing) * sin_d * cos_lat1,
                          cos_d - sin_lat1 * sin_lat2)
        points.append((math.degrees(lat2), lon + math.degrees(dlon)))
    return points


def circle_latlon(centre, radius, segments=FENCE_CIRCLE_SEGMENTS):
    '''lat/lon ring approximating a circle of radius metres about centre'''
    return ring_points(centre, radius,
                       [2.0 * math.pi * i / segments for i in range(segments)])


def leg_tangent(centre, radius, clockwise, to_centre, to_radius, to_clockwise):
    '''the bearings at which the leg between two circles leaves the first and
    joins the second, each measured from its own centre.

    A vehicle joins and leaves a loiter along a tangent rather than flying at
    the middle of the circle, so the leg is the line that touches both
    circles without crossing either.  Two circles turned the same way are
    joined by an outer tangent, with the radii to the touch points parallel;
    turned opposite ways the leg crosses between them and the radii are
    opposed.  A plain waypoint is a circle of no radius, which reduces to the
    tangent from a point.

    Returns (leave, join).  Circles too close together to have a tangent fall
    back to the bearing straight between them
    '''
    distance = mp_util.gps_distance(centre[0], centre[1],
                                    to_centre[0], to_centre[1])
    bearing = mp_util.gps_bearing(centre[0], centre[1],
                                  to_centre[0], to_centre[1])
    if distance <= 0:
        return (bearing, bearing + 180.0)
    if clockwise == to_clockwise:
        reach = (radius - to_radius) / distance
        opposed = False
    else:
        reach = (radius + to_radius) / distance
        opposed = True
    if reach < -1.0 or reach > 1.0:
        return (bearing, bearing + 180.0)
    offset = math.degrees(math.acos(reach))
    leave = bearing - offset if clockwise else bearing + offset
    return (leave, leave + 180.0 if opposed else leave)


def _circle_of(item, clockwise):
    '''(centre, radius, clockwise) for a mission item, treating one that does
    not circle as a circle of no radius'''
    radius = item.circle_radius if item is not None else None
    if not radius:
        return ((item.lat, item.lon) if item is not None else (0.0, 0.0),
                0.0, clockwise)
    return ((item.lat, item.lon), abs(radius), radius > 0)


def spiral_latlon(centre, radius, turns, segments=MISSION_CIRCLE_SEGMENTS,
                  start_bearing=0.0):
    '''lat/lon points along turns turns about centre, from start_bearing.

    A positive radius winds clockwise, negative counter-clockwise, as the
    loiter and orbit commands define it.  Both ends are included, so the
    caller can walk an altitude along the points
    '''
    count = max(2, int(round(segments * abs(turns))))
    sweep = 2.0 * math.pi * abs(turns) * (1.0 if radius >= 0 else -1.0)
    return ring_points(centre, abs(radius),
                       [start_bearing + sweep * i / count
                        for i in range(count + 1)])


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


def _arrows(points_enu, colour, renderer=None, count=MISSION_ARROW_COUNT):
    """cones spread along a polyline pointing the way it is travelled.

    They are spaced by distance rather than by vertex, so a long leg gets as
    many as a densely sampled spiral does.  Given a renderer they are drawn a
    fixed size on the screen however far away they are, the way the 2D map
    draws its arrows; without one they fall back to a size in metres taken
    from the spacing, which only looks right at one zoom level
    """
    lengths = [math.dist(points_enu[i-1], points_enu[i])
               for i in range(1, len(points_enu))]
    total = 0.0
    for length in lengths:
        total += length
    if total <= 0 or count < 1:
        return None
    spacing = total / count

    positions = vtk.vtkPoints()
    directions = vtk.vtkDoubleArray()
    directions.SetNumberOfComponents(3)
    directions.SetName('direction')
    walked = spacing * 0.5
    for (i, length) in enumerate(lengths):
        if length <= 0:
            continue
        (a, b) = (points_enu[i], points_enu[i+1])
        heading = [(b[j] - a[j]) / length for j in range(3)]
        while walked < length:
            fraction = walked / length
            positions.InsertNextPoint(*[a[j] + (b[j] - a[j]) * fraction
                                        for j in range(3)])
            directions.InsertNextTuple3(*heading)
            walked += spacing
        walked -= length
    if positions.GetNumberOfPoints() == 0:
        return None

    poly = vtk.vtkPolyData()
    poly.SetPoints(positions)
    poly.GetPointData().SetVectors(directions)
    cone = vtk.vtkConeSource()
    cone.SetResolution(10)
    cone.SetHeight(1.0)
    cone.SetRadius(0.3)
    glyph = vtk.vtkGlyph3D()
    glyph.SetSourceConnection(cone.GetOutputPort())
    glyph.SetVectorModeToUseVector()
    glyph.OrientOn()
    if renderer is not None:
        # scale each cone by its distance from the camera, so they hold the
        # same size on the screen at any zoom
        to_camera = vtk.vtkDistanceToCamera()
        to_camera.SetInputData(poly)
        to_camera.SetScreenSize(MISSION_ARROW_PIXELS)
        to_camera.SetRenderer(renderer)
        glyph.SetInputConnection(to_camera.GetOutputPort())
        glyph.SetScaleModeToScaleByScalar()
        glyph.SetScaleFactor(1.0)
        # the filter's scale is not the active scalar array, so name it
        glyph.SetInputArrayToProcess(
            0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS,
            'DistanceToCamera')
    else:
        glyph.SetInputData(poly)
        glyph.SetScaleModeToDataScalingOff()
        cone.SetHeight(min(max(spacing * 0.25, 1.0), 500.0))
        cone.SetRadius(cone.GetHeight() * 0.4)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(glyph.GetOutputPort())
    # the scale carried through the glyph is not something to colour by
    mapper.ScalarVisibilityOff()
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*colour)
    actor.GetProperty().SetLighting(False)
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
        self.path = []              # (ENU point, optional timestamp)
        self.path_time_range = None
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
        self.mission_line = []
        self.mission_markers = []
        self.mission_arrows = False

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
        '''path: list of (lat,lon,amsl[,matplotlib_date_number])'''
        self.path = [(self._enu(p[0], p[1], p[2] + 2.0),
                      p[3] if len(p) > 3 else None) for p in path]
        self.refresh_path()

    def set_time_range(self, trange):
        '''Set the graph time range used to display a timestamped path.'''
        self.path_time_range = trange
        self.refresh_path()

    def refresh_path(self):
        '''Rebuild the visible path, preserving the 2D map's segment rules.

        The 2D map includes a segment when its first point is in the selected
        range. Build separate actors for non-contiguous runs so filtering can
        never join two unrelated portions of a track.
        '''
        if len(self.path) < 2:
            self._replace('path', [])
            return
        if (self.path_time_range is None or
                any(timestamp is None for _, timestamp in self.path)):
            segments = [[point for point, _ in self.path]]
        else:
            low, high = self.path_time_range
            segments = []
            current = None
            for i in range(len(self.path) - 1):
                point, timestamp = self.path[i]
                if low <= timestamp <= high:
                    if current is None:
                        current = [point]
                        segments.append(current)
                    current.append(self.path[i + 1][0])
                else:
                    current = None
        actors = [_polyline(segment, (1.0, 0.0, 0.7), 2.5)
                  for segment in segments if len(segment) >= 2]
        self._replace('path', actors)

    def add_trail_point(self, lat, lon, amsl):
        self.trail.append(self._enu(lat, lon, amsl))
        if len(self.trail) > 5000:
            self.trail = self.trail[-5000:]
        if len(self.trail) >= 2:
            self._replace('trail', [_polyline(self.trail, (1.0, 1.0, 0.0), 2.0)])

    def set_mission(self, items):
        '''items: list of MissionItem (plain tuples are accepted too).

        The line drawn is the path the vehicle is expected to fly, so it is
        continuous throughout: an arc waypoint curves, and an item which
        circles about its location is entered from the near side of its
        circle and left again where it comes off, rather than the line
        running to the middle of the circle where the vehicle never goes.
        The markers stay on the mission item locations
        '''
        line = []
        markers = []
        previous = None
        rejoin = None
        flown = [MissionItem(*item) for item in items]
        flown = [i for i in flown if not (i.lat == 0 and i.lon == 0)]
        for (index, item) in enumerate(flown):
            amsl = self._resolve_amsl(item.alt, item.frame)
            if (item.command == mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT and
                    previous is not None):
                # the leg into an arc waypoint is a circular arc rather
                # than a straight line; climb linearly along it
                ((prev_lat, prev_lon), prev_amsl) = previous
                arc = mp_util.arc_points((prev_lat, prev_lon),
                                         (item.lat, item.lon), item.param1)
                for i in range(1, len(arc) - 1):
                    fraction = float(i) / (len(arc) - 1)
                    line.append(self._enu(arc[i][0], arc[i][1],
                                          prev_amsl + (amsl - prev_amsl) * fraction))
            markers.append(self._enu(item.lat, item.lon, amsl))
            if item.circle_radius:
                previous = self._append_circle(
                    line, item, amsl, previous,
                    flown[index-1] if index > 0 else None,
                    flown[index+1] if index + 1 < len(flown) else None)
                # the leg out of a loiter is flown against a track from the
                # loiter's own centre unless the item says otherwise, so the
                # vehicle has to pull back onto it after leaving the circle
                rejoin = (((item.lat, item.lon), previous[0], amsl,
                           item.exit_converge)
                          if item.exit_converge else None)
                continue
            if rejoin is not None:
                self._append_rejoin(line, rejoin, (item.lat, item.lon), amsl)
            rejoin = None
            line.append(self._enu(item.lat, item.lon, amsl))
            previous = ((item.lat, item.lon), amsl)
        self.mission_line = line
        self.mission_markers = markers
        self.refresh_mission()

    def set_mission_arrows(self, enable):
        '''show or hide the direction of travel along the mission'''
        enable = bool(enable)
        if enable == self.mission_arrows:
            return
        self.mission_arrows = enable
        self.refresh_mission()

    def refresh_mission(self):
        '''rebuild the mission actors from the last mission drawn'''
        line = self.mission_line
        actors = []
        if len(line) >= 2:
            actors.append(_polyline(line, (1.0, 1.0, 1.0), 2.0, dashed=True))
            if self.mission_arrows:
                arrows = _arrows(line, (1.0, 1.0, 1.0), self.ren)
                if arrows is not None:
                    actors.append(arrows)
        if self.mission_markers:
            actors.append(_points(self.mission_markers, (1.0, 1.0, 1.0), 9))
        self._replace('mission', actors)

    def _append_rejoin(self, line, rejoin, target, target_amsl):
        """walk the vehicle from where it left a loiter back onto the track it
        is flying against.

        ArduPlane crosstracks the leg after a loiter from the loiter's centre
        rather than from the tangent it left on, so the vehicle comes off the
        circle a radius or so to one side of that track and pulls back onto
        it.  The pull-back is the L1 controller's, which closes the error over
        its own distance, so it is drawn decaying over that
        """
        ((clat, clon), (elat, elon), exit_amsl, converge) = rejoin
        track = mp_util.gps_bearing(clat, clon, target[0], target[1])
        length = mp_util.gps_distance(clat, clon, target[0], target[1])
        exit_bearing = mp_util.gps_bearing(clat, clon, elat, elon)
        radius = mp_util.gps_distance(clat, clon, elat, elon)
        offset_angle = math.radians(exit_bearing - track)
        # where the exit sits measured along the track, and to one side of it
        along = radius * math.cos(offset_angle)
        across = radius * math.sin(offset_angle)
        run = length - along
        if run <= 0 or abs(across) < 1.0:
            return
        # the vehicle leaves on the tangent, which already points at the next
        # waypoint and so is closing on the track at this rate.  The curve has
        # to start along it, or the drawn path kinks at the exit and cuts back
        # inside the circle it has just left
        slope = -across / run
        # closing the rest of the way as the L1 controller does, over its own
        # distance, while keeping that starting direction
        rate = slope + across / converge
        if across * rate <= 0:
            # the tangent alone closes at least as fast as the controller
            # would; there is nothing to pull back onto
            return
        # whatever error is left when the waypoint arrives has to be gone by
        # then, since that is where the vehicle ends up; take it out with a
        # term that starts flat, so the tangent still sets the way it leaves
        residual = (across + rate * run) * math.exp(-run / converge)
        step = max(converge * 0.125, 1.0)
        distance = along + step
        while distance < length:
            travelled = distance - along
            decay = math.exp(-travelled / converge)
            remaining = (across + rate * travelled) * decay
            remaining -= residual * (travelled / run) ** 2
            if abs(remaining) < 0.5:
                break
            fraction = travelled / run
            amsl = exit_amsl + (target_amsl - exit_amsl) * fraction
            point = mp_util.gps_newpos(clat, clon, track, distance)
            point = mp_util.gps_newpos(point[0], point[1],
                                       track + 90.0, remaining)
            line.append(self._enu(point[0], point[1], amsl))
            distance += step

    def _append_circle(self, line, item, amsl, previous, before, after):
        '''add the turns an item flies about its own location to the line.

        The circle is joined along a tangent from the item before it and left
        along a tangent towards the item after it, and the altitude runs at a
        steady rate over the whole of that -- the leg included, since the
        vehicle is already climbing or descending on its way there.  Returns
        where the vehicle leaves the circle
        '''
        centre = (item.lat, item.lon)
        radius = abs(item.circle_radius)
        clockwise = item.circle_radius > 0
        entry_bearing = 0.0
        prev_amsl = amsl
        if previous is not None:
            (_, prev_amsl) = previous
            (_, entry_bearing) = leg_tangent(
                *_circle_of(before, clockwise) + (centre, radius, clockwise))
        # an item that changes altitude while it circles is a spiral; one that
        # does not is a single turn at its own altitude
        turns = item.circle_turns if prev_amsl != amsl else None
        if not turns:
            turns = 1.0
        if after is not None:
            # come off the circle pointing at whatever is next, going round
            # as many whole extra times as the turns asked for
            (exit_bearing, _) = leg_tangent(
                centre, radius, clockwise, *_circle_of(after, clockwise))
            swept = exit_bearing - entry_bearing
            if not clockwise:
                swept = -swept
            swept %= 360.0
            laps = max(0, int(round((turns * 360.0 - swept) / 360.0)))
            turns = (swept + 360.0 * laps) / 360.0
        if turns <= 0:
            turns = 1.0
        ring = spiral_latlon(centre, item.circle_radius, turns,
                             MISSION_CIRCLE_SEGMENTS,
                             math.radians(entry_bearing))
        # spread the altitude change over the leg and the turns together, so
        # the approach shows its share of the climb or descent
        leg_length = 0.0
        if previous is not None:
            ((prev_lat, prev_lon), _) = previous
            leg_length = mp_util.gps_distance(prev_lat, prev_lon,
                                              ring[0][0], ring[0][1])
        total = leg_length + turns * 2.0 * math.pi * radius
        entry_amsl = prev_amsl
        if total > 0:
            entry_amsl += (amsl - prev_amsl) * leg_length / total
        last = len(ring) - 1
        for (i, (la, lo)) in enumerate(ring):
            line.append(self._enu(la, lo,
                                  entry_amsl + (amsl - entry_amsl) * i / last))
        return (ring[-1], amsl)

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
