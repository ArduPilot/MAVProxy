#!/usr/bin/env python3
'''
Mission geometry for the BVLOS planning checks.

Deliberately free of any MAVProxy or vehicle state, so a mission can be
checked offline from a file. Mission items are anything with the mavlink
mission item interface (seq, command, frame, x, y, z), which covers both the
wp module's wploader and mavwp.MAVWPLoader reading a mission file.
'''

# AP_FLAKE8_CLEAN

import math

from pymavlink import mavutil

mavlink = mavutil.mavlink

# ArduPilot constants, see libraries/AP_Math/definitions.h
GRAVITY_MSS = 9.80665
SSL_AIR_DENSITY = 1.225
ISA_LAPSE_RATE = 0.0065
SSL_TEMPERATURE = 288.15
# the 1976 standard atmosphere gas constant used by AP_Baro_atmosphere.cpp
R_SPECIFIC = 287.053072

# metres per degree of latitude, matching LOCATION_SCALING_FACTOR in
# libraries/AP_Common/Location.h
METRES_PER_DEG = 0.011131884502145034 * 1.0e7

# commands carrying a location, mirroring AP_Mission::stored_in_location()
LOCATION_COMMANDS = frozenset([
    mavlink.MAV_CMD_NAV_WAYPOINT,
    mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavlink.MAV_CMD_NAV_LOITER_TIME,
    mavlink.MAV_CMD_NAV_LAND,
    mavlink.MAV_CMD_NAV_TAKEOFF,
    mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
    mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
    mavlink.MAV_CMD_DO_SET_HOME,
    mavlink.MAV_CMD_DO_RETURN_PATH_START,
    mavlink.MAV_CMD_DO_LAND_START,
    mavlink.MAV_CMD_DO_GO_AROUND,
    mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
    mavlink.MAV_CMD_DO_SET_ROI,
    mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
    mavlink.MAV_CMD_NAV_VTOL_LAND,
    mavlink.MAV_CMD_NAV_PAYLOAD_PLACE,
    # these live in their own storage so a mission download never contains
    # them, but stored_in_location() lists them and the walk would use them
    mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
    mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
    mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
    mavlink.MAV_CMD_NAV_RALLY_POINT,
] + ([mavlink.MAV_CMD_NAV_ARC_WAYPOINT]
     if hasattr(mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT') else []))

# navigation commands whose flown path is not the straight line between the
# stored coordinates, so a corridor built from stored coordinates would be
# wrong. Loiters are handled separately, as their radius is known
CURVED_COMMANDS = frozenset(
    [mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT] +
    ([mavlink.MAV_CMD_NAV_ARC_WAYPOINT]
     if hasattr(mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT') else []))

# navigation commands that fly a circle about the stored location
LOITER_COMMANDS = frozenset([
    mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavlink.MAV_CMD_NAV_LOITER_TIME,
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
])

# commands that end a return path, AP_Mission::is_landing_type_cmd()
LANDING_COMMANDS = frozenset([
    mavlink.MAV_CMD_NAV_LAND,
    mavlink.MAV_CMD_NAV_VTOL_LAND,
    mavlink.MAV_CMD_DO_PARACHUTE,
])

# AP_Mission::is_nav_cmd(): everything up to NAV_LAST, plus these
# smallest sample spacing we will use, so a mistyped setting cannot divide by
# zero or ask for an unbounded amount of work
MIN_SPACING = 1.0

# most samples we will take along a mission
MAX_SAMPLES = 20000

# and the most distance queries the whole check may ask for. Each sample walks
# its own return, so halving the granularity is four times the work, and a
# granularity far too small for the mission is a typo rather than a request
MAX_QUERIES = 4.0e7


def path_length(path):
    '''length of a path in metres'''
    total = 0.0
    for i in range(1, len(path)):
        total += math.hypot(path[i].x - path[i - 1].x,
                            path[i].y - path[i - 1].y)
    return total


def usable_spacing(path, spacing, states=1):
    '''the given spacing, held to something that can actually be worked
       through on the mission in hand.

       The work is one sample every spacing along the mission, each walking a
       return that can be as long as the mission, for each counter state, so
       it grows as the square of how fine the sampling is.
    '''
    spacing = max(float(spacing), MIN_SPACING)
    length = path_length(path)
    if length <= 0:
        return spacing
    floor = max(length / MAX_SAMPLES,
                length * math.sqrt(max(1, states) / MAX_QUERIES))
    return max(spacing, floor)


NAV_LAST = mavlink.MAV_CMD_NAV_LAST
EXTRA_NAV_COMMANDS = frozenset([
    mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
    getattr(mavlink, 'MAV_CMD_NAV_SCRIPT_TIME', 42702),
    getattr(mavlink, 'MAV_CMD_NAV_ATTITUDE_TIME', 42703),
])

RELATIVE_FRAMES = frozenset([
    mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
])
TERRAIN_FRAMES = frozenset([
    mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
])


def is_nav_command(command):
    '''AP_Mission::is_nav_cmd()'''
    return command <= NAV_LAST or command in EXTRA_NAV_COMMANDS


def command_name(command):
    '''readable name for a MAV_CMD'''
    enums = mavlink.enums['MAV_CMD']
    if command in enums:
        return enums[command].name.replace('MAV_CMD_', '')
    return str(command)


def wrap_180(angle):
    '''wrap a longitude difference to -180..180'''
    return (angle + 180.0) % 360.0 - 180.0


def eas2tas(alt_amsl):
    '''equivalent to true airspeed ratio at an altitude, matching the
       gradient layer of AP_Baro::get_air_density_for_alt_amsl()'''
    temp = SSL_TEMPERATURE - ISA_LAPSE_RATE * alt_amsl
    if temp <= 0:
        # far above anywhere an aircraft flies
        return 1.0
    exponent = GRAVITY_MSS / (ISA_LAPSE_RATE * R_SPECIFIC) - 1.0
    density = SSL_AIR_DENSITY * (temp / SSL_TEMPERATURE) ** exponent
    if density <= 0:
        return 1.0
    return math.sqrt(SSL_AIR_DENSITY / density)


def turn_radius(cruise_eas, roll_limit_deg, alt_amsl):
    '''fixed wing turn radius in metres, from the coordinated turn relation
       used by fixedwing_turn_rate() in libraries/AP_Math/AP_Math.cpp'''
    bank = max(1.0, min(float(roll_limit_deg), 80.0))
    tas = cruise_eas * eas2tas(alt_amsl)
    return (tas * tas) / (GRAVITY_MSS * math.tan(math.radians(bank)))


class Projector(object):
    '''project lat/lon onto a local metric frame.

       Uses ArduPilot's own equirectangular scaling so distances match
       Location::get_distance_NED_alt_frame(), except that the longitude
       scale is taken at one reference latitude rather than at the mean
       latitude of each pair. Over a mission spanning a degree of latitude
       that is a few tenths of a percent, immaterial against a turn radius
       tolerance of tens of metres.
    '''

    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self.lon_scale = max(math.cos(math.radians(lat0)), 0.01)

    def project(self, lat, lon):
        '''return (east, north) metres from the reference'''
        return (wrap_180(lon - self.lon0) * METRES_PER_DEG * self.lon_scale,
                (lat - self.lat0) * METRES_PER_DEG)

    def unproject(self, x, y):
        '''return (lat, lon) for a point in metres'''
        lat = self.lat0 + y / METRES_PER_DEG
        lon = self.lon0 + x / (METRES_PER_DEG * self.lon_scale)
        return (lat, wrap_180(lon))


def segment_distance(px, py, ax, ay, bx, by):
    '''distance from a point to a closed 2D segment'''
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    d2 = vx * vx + vy * vy
    if d2 <= 0.0:
        return math.hypot(wx, wy)
    t = (vx * wx + vy * wy) / d2
    t = max(0.0, min(1.0, t))
    return math.hypot(wx - t * vx, wy - t * vy)


def segment_distance_3d(px, py, pz, ax, ay, az, bx, by, bz):
    '''distance from a point to a closed 3D segment, which is what
       AP_Mission::distance_to_mission_leg() measures'''
    vx = bx - ax
    vy = by - ay
    vz = bz - az
    wx = px - ax
    wy = py - ay
    wz = pz - az
    d2 = vx * vx + vy * vy + vz * vz
    if d2 <= 0.0:
        return math.sqrt(wx * wx + wy * wy + wz * wz)
    t = (vx * wx + vy * wy + vz * wz) / d2
    t = max(0.0, min(1.0, t))
    dx = wx - t * vx
    dy = wy - t * vy
    dz = wz - t * vz
    return math.sqrt(dx * dx + dy * dy + dz * dz)


# segments per grid cell to aim for, trading build cost against query cost
GRID_PER_CELL = 2.0
# above this many cells per axis the grid costs more to build than it saves
GRID_MAX_CELLS = 256


class PathIndex(object):
    '''the segments of a path with a uniform grid over them, so the distance
       from a point to the whole path does not have to look at every segment.

       A check makes millions of these queries, once per sample along every
       cut across the mission, so this is the hot loop of the whole module.
    '''

    def __init__(self, points, extra_segments=None):
        self.segments = []
        for i in range(1, len(points)):
            a = points[i - 1]
            b = points[i]
            self._add(a.x, a.y, b.x, b.y)
        for seg in (extra_segments or []):
            self._add(seg[0], seg[1], seg[2], seg[3])
        self.fallback = (points[0].x, points[0].y) if len(points) else None
        self._build_grid()

    def _add(self, ax, ay, bx, by):
        vx = bx - ax
        vy = by - ay
        d2 = vx * vx + vy * vy
        if d2 <= 0.0:
            return
        self.segments.append((ax, ay, vx, vy, d2))

    def _build_grid(self):
        '''walk each segment marking the cells it passes through.

           Marking the cells of its bounding box instead would put a long
           diagonal leg into every cell of a huge square, which is most of the
           grid for a mission whose legs are kilometres long.
        '''
        self.grid = None
        count = len(self.segments)
        if count == 0:
            return
        xs = []
        ys = []
        total = 0.0
        for (ax, ay, vx, vy, d2) in self.segments:
            xs.extend((ax, ax + vx))
            ys.extend((ay, ay + vy))
            total += math.sqrt(d2)
        self.minx = min(xs)
        self.miny = min(ys)
        span = max(max(xs) - self.minx, max(ys) - self.miny)
        if span <= 0:
            return
        # size cells by the average leg, which puts a query near the path in
        # the first ring or two while keeping the marked cell count near the
        # path length rather than its area
        cell = max(total / count * 0.5, span / GRID_MAX_CELLS)
        if cell <= 0:
            return
        self.cell = cell
        self.across = max(1, int(span / cell) + 1)
        grid = {}
        for (index, seg) in enumerate(self.segments):
            self._mark(index, seg, grid)
        self.grid = grid
        self.seen = [-1] * count
        self.query = 0
        self.cell_lo = (min(key[0] for key in grid),
                        min(key[1] for key in grid))
        self.cell_hi = (max(key[0] for key in grid),
                        max(key[1] for key in grid))

    def _mark(self, index, seg, grid):
        '''mark every cell the segment passes through, walking it cell by
           cell. Exact, so a query can stop as soon as its best beats the
           nearest unexamined ring'''
        (ax, ay, vx, vy, _) = seg
        cell = self.cell
        cx = self._cell_of(ax, self.minx)
        cy = self._cell_of(ay, self.miny)
        ex = self._cell_of(ax + vx, self.minx)
        ey = self._cell_of(ay + vy, self.miny)
        stepx = 1 if vx > 0 else (-1 if vx < 0 else 0)
        stepy = 1 if vy > 0 else (-1 if vy < 0 else 0)
        big = float('inf')
        if stepx:
            edge = self.minx + (cx + (1 if stepx > 0 else 0)) * cell
            tx = (edge - ax) / vx
            tdx = abs(cell / vx)
        else:
            tx = tdx = big
        if stepy:
            edge = self.miny + (cy + (1 if stepy > 0 else 0)) * cell
            ty = (edge - ay) / vy
            tdy = abs(cell / vy)
        else:
            ty = tdy = big
        # bounded in case a boundary lands exactly on a coordinate
        guard = abs(ex - cx) + abs(ey - cy) + 4
        while guard > 0:
            guard -= 1
            grid.setdefault((cx, cy), []).append(index)
            if cx == ex and cy == ey:
                return
            if tx < ty:
                tx += tdx
                cx += stepx
            else:
                ty += tdy
                cy += stepy

    def _cell_of(self, value, origin):
        return int(math.floor((value - origin) / self.cell))

    def _segment_d2(self, index, px, py):
        (ax, ay, vx, vy, d2) = self.segments[index]
        wx = px - ax
        wy = py - ay
        t = (vx * wx + vy * wy) / d2
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        dx = wx - t * vx
        dy = wy - t * vy
        return dx * dx + dy * dy

    def distance(self, px, py):
        '''smallest distance from a point to the path'''
        if len(self.segments) == 0:
            if self.fallback is None:
                return 0.0
            return math.hypot(px - self.fallback[0], py - self.fallback[1])
        if self.grid is None:
            best = min(self._segment_d2(i, px, py)
                       for i in range(len(self.segments)))
            return math.sqrt(best)

        cell = self.cell
        cx = self._cell_of(px, self.minx)
        cy = self._cell_of(py, self.miny)
        self.query += 1
        stamp = self.query
        seen = self.seen
        grid = self.grid
        best = None
        ring = 0
        # far enough to have reached every marked cell from wherever the query
        # landed, which for a query outside the grid is further than the grid
        # is wide
        limit = (max(abs(cx - self.cell_lo[0]), abs(cx - self.cell_hi[0])) +
                 max(abs(cy - self.cell_lo[1]), abs(cy - self.cell_hi[1])))
        while True:
            for key in self._ring(cx, cy, ring):
                for index in grid.get(key, ()):
                    if seen[index] == stamp:
                        continue
                    seen[index] = stamp
                    d2 = self._segment_d2(index, px, py)
                    if best is None or d2 < best:
                        best = d2
            # the query sits inside its own cell, so anything not looked at
            # yet lies in a cell at least this far off. Once the best beats
            # that there is nothing better left to find
            if best is not None and best <= (ring * cell) ** 2:
                break
            ring += 1
            if ring > limit:
                # past every marked cell. Falling back to all of them keeps
                # the answer exact whatever the ring bookkeeping did
                best = min(self._segment_d2(i, px, py)
                           for i in range(len(self.segments)))
                break
        return math.sqrt(best)

    def _ring(self, cx, cy, ring):
        '''the cells at Chebyshev distance ring from (cx, cy)'''
        if ring == 0:
            yield (cx, cy)
            return
        for dx in range(-ring, ring + 1):
            yield (cx + dx, cy - ring)
            yield (cx + dx, cy + ring)
        for dy in range(-ring + 1, ring):
            yield (cx - ring, cy + dy)
            yield (cx + ring, cy + dy)


class MissionPoint(object):
    '''a mission item reduced to what the checks need'''

    def __init__(self, seq, command, frame, lat, lon, alt,
                 param1=0, param2=0, param3=0):
        self.seq = seq
        self.command = command
        self.frame = frame
        self.lat = lat
        self.lon = lon
        # altitude as stored, in its own frame
        self.alt = alt
        # DO_JUMP uses param1 as the target and param2 as the repeat count;
        # a loiter carries its radius in param3, or param2 for LOITER_TO_ALT
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        # True for a point we invented, such as one on a loiter orbit, which
        # must never be written back into a mission
        self.synthetic = False
        # resolved by build_mission()
        self.amsl = None
        self.ground = None
        self.x = None
        self.y = None

    def initialised(self):
        '''Location::initialised(). A non zero altitude alone makes a
           location count as valid, which ArduPilot relies on'''
        return self.lat != 0 or self.lon != 0 or self.alt != 0

    def has_location(self):
        return self.command in LOCATION_COMMANDS and self.initialised()

    def is_landing(self):
        return self.command in LANDING_COMMANDS

    def is_nav(self):
        return is_nav_command(self.command)

    def is_terrain_frame(self):
        return self.frame in TERRAIN_FRAMES

    def is_loiter(self):
        return self.command in LOITER_COMMANDS

    def is_curved(self):
        '''flown as a curve, so a straight leg to it is not where it goes'''
        return self.command in CURVED_COMMANDS

    def position_known(self):
        '''False for an item with no stored position.

           ArduPilot's Location::sanitize() fills a zero latitude and
           longitude in from wherever the aircraft is when the command runs,
           so where it goes cannot be known while planning.
        '''
        return self.lat != 0 or self.lon != 0

    def loiter_radius(self, default_radius):
        '''radius of the circle a loiter flies, or None if unknown.

           Which field carries it differs by command, see
           AP_Mission::mavlink_to_mission_cmd(). NAV_LOITER_TIME has none:
           its seconds use all sixteen bits of p1 and param3 is only the
           direction, so verify_loiter_time() circles at WP_LOITER_RAD.
        '''
        if self.command == mavlink.MAV_CMD_NAV_LOITER_TIME:
            radius = 0.0
        elif self.command == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            radius = self.param2
        else:
            radius = self.param3
        radius = abs(float(radius))
        if radius > 0:
            return radius
        if default_radius and default_radius > 0:
            return float(default_radius)
        return None

    def loiter_ccw(self):
        '''True if the circle is flown anticlockwise. The sign of the field
           carrying the radius gives the direction'''
        if self.command == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            return float(self.param2) < 0
        return float(self.param3) < 0

    def loiter_forever(self):
        '''True for a loiter the mission never leaves'''
        return self.command == mavlink.MAV_CMD_NAV_LOITER_UNLIM

    def __str__(self):
        return "%u:%s" % (self.seq, command_name(self.command))


# max_loops in AP_Mission::get_next_cmd()
MAX_JUMP_LOOPS = 64

# AP_MISSION_JUMP_REPEAT_FOREVER
JUMP_REPEAT_FOREVER = -1

# how many times a mission item may be flown before we call it a loop. Two
# passes is enough to have every leg of the loop, including the one that
# closes it
MAX_REPEATS = 2

# points used to approximate a loiter circle
LOITER_POINTS = 24


def next_command(mission, index, jump_counts, dont_zero_counter=False):
    '''AP_Mission::get_next_cmd(): the next non jump command at or after
       index, following DO_JUMP. Returns (point, index), with a None point at
       the end of the mission or on a bad jump'''
    loops = MAX_JUMP_LOOPS
    total = mission.count()
    while 0 <= index < total:
        point = mission.point(index)
        command = point.command
        if command == mavlink.MAV_CMD_DO_JUMP:
            target = int(point.param1)
        elif command == getattr(mavlink, 'MAV_CMD_DO_JUMP_TAG', -1):
            target = jump_tag_index(mission, int(point.param1))
        else:
            return (point, index)
        if loops == 0 or target is None:
            return (None, index)
        loops -= 1
        # an invalid target aborts the search, as in ArduPilot
        if target >= total or target == 0:
            return (None, index)
        num_times = int(point.param2)
        run = jump_counts.get(index, 0)
        if num_times == JUMP_REPEAT_FOREVER:
            # ArduPilot counts these too, but never looks at the count. Not
            # counting them keeps the walk state finite, which is what lets a
            # loop be recognised as one rather than run until a step limit
            index = target
        elif run < num_times:
            jump_counts[index] = run + 1
            index = target
        elif dont_zero_counter:
            index += 1
        else:
            # having finished a jump loop ArduPilot zeroes the counter, so
            # coming back to it later runs the loop again. MIS_OPTIONS bit 3
            # turns that off
            jump_counts[index] = 0
            index += 1
    return (None, index)


def jump_tag_index(mission, tag):
    '''index of the JUMP_TAG item carrying this tag, as
       AP_Mission::get_index_of_jump_tag() finds it'''
    tag_cmd = getattr(mavlink, 'MAV_CMD_JUMP_TAG', None)
    if tag_cmd is None:
        return None
    for point in mission.points:
        if point.command == tag_cmd and int(point.param1) == tag:
            return point.seq
    return None


class Mission(object):
    '''a mission with altitudes resolved to AMSL and positions projected'''

    def __init__(self, points, home_amsl, projector, terrain_missing=0):
        self.points = points
        self.home_amsl = home_amsl
        self.projector = projector
        # how many items needed terrain we did not have
        self.terrain_missing = terrain_missing

    def count(self):
        return len(self.points)

    def point(self, index):
        if 0 <= index < len(self.points):
            return self.points[index]
        return None

    def return_path_starts(self):
        return [p.seq for p in self.points
                if p.command == mavlink.MAV_CMD_DO_RETURN_PATH_START]

    def takeoff_next(self, index, jump_counts=None, dont_zero_counter=False):
        '''AP_Mission::is_takeoff_next(): whether the next navigation command
           from here is a takeoff, which is what lets a mission carry on past
           a landing'''
        takeoffs = frozenset([mavlink.MAV_CMD_NAV_TAKEOFF,
                              mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
                              getattr(mavlink, 'MAV_CMD_NAV_TAKEOFF_LOCAL',
                                      24)])
        skippable = frozenset([mavlink.MAV_CMD_DO_AUX_FUNCTION,
                               mavlink.MAV_CMD_NAV_DELAY])
        counts = dict(jump_counts or {})
        # ArduPilot looks at a maximum of 16 items
        for _ in range(16):
            (point, index) = next_command(self, index, counts,
                                          dont_zero_counter)
            if point is None:
                return False
            index = point.seq + 1
            if not point.is_nav():
                continue
            if point.command in takeoffs:
                return True
            if point.command in skippable:
                continue
            return False
        return False

    def flown_path(self, jump_counts=None, dont_zero_counter=False,
                   continue_after_land=False, loiter_radius=0.0):
        '''where the aircraft actually goes: the navigation items with a
           location that AUTO would fly, after home, up to and including the
           first landing.

           The mission is walked the way it runs rather than in storage
           order, so a DO_JUMP forward does not leave the items it skips in
           the corridor, and one backward puts the leg it flies back along
           into it. Without that the corridor is not what the aircraft covers,
           and a cut across ground the mission never flies could pass.

           Only navigation commands count. DO_SET_HOME, DO_SET_ROI,
           DO_LAND_START and DO_RETURN_PATH_START carry a location but are
           never flown to, and treating them as corridor would invent
           corridor that does not exist.

           Anything past the landing is not flown either. Both reference
           BVLOS missions carry LOITER_TURNS and DO_JUMP pairs after the
           landing, which the operator selects rather than flies, and which
           ArduPilot's own return path walk never reaches because it stops at
           the landing.
        '''
        path = []
        counts = dict(jump_counts or {})
        seen = set()
        index = 1
        while True:
            # the walk is decided entirely by where we are and what the jump
            # counters hold, so meeting the same pair twice means a loop that
            # never ends. A loop with a repeat count is not one: its counters
            # move on every time round, so it runs to its exit and out the
            # other side
            state = (index, tuple(sorted(counts.items())))
            if state in seen:
                break
            seen.add(state)
            (point, index) = next_command(self, index, counts,
                                          dont_zero_counter)
            if point is None:
                break
            index = point.seq + 1
            if point.is_nav() and point.has_location():
                path.append(point)
            if point.is_nav() and point.has_location() and \
                    point.loiter_forever():
                # NAV_LOITER_UNLIM never completes, so nothing after it is
                # ever flown
                break
            if point.is_landing():
                if not (continue_after_land and
                        self.takeoff_next(index, counts, dont_zero_counter)):
                    break
        return self.expand_loiters(path, loiter_radius)

    def ends_in_landing(self):
        '''True if the flown path finishes at a landing'''
        path = self.flown_path()
        return len(path) > 0 and path[-1].is_landing()

    def takeoff_after_landing(self):
        '''whether a takeoff follows the landing, which is what
           continue_after_land_check_for_takeoff() asks. With MIS_OPTIONS
           CONTINUE_AFTER_LAND set that makes the mission carry on past the
           landing rather than finishing there'''
        path = self.flown_path()
        if len(path) == 0 or not path[-1].is_landing():
            return False
        return self.takeoff_next(path[-1].seq + 1)

    def loiters_without_radius(self, default_radius=0.0, path=None):
        """sequence numbers of loiters whose circle we cannot work out.

           One with a radius has already been expanded into its orbit by the
           time the path gets here, so anything still a bare loiter point is
           one we could not place.
        """
        if path is None:
            path = self.flown_path(loiter_radius=default_radius)
        seqs = []
        for point in path:
            if not point.is_loiter() or point.synthetic:
                continue
            if point.loiter_radius(default_radius) is None and \
                    point.seq not in seqs:
                seqs.append(point.seq)
        return seqs

    def unmodelled(self, default_radius=0.0, path=None):
        '''reasons the corridor is not where the aircraft actually goes.

           These make a result inconclusive rather than a pass: the check can
           only compare against the ground it believes the mission covers.
        '''
        reasons = []
        if path is None:
            path = self.flown_path(loiter_radius=default_radius)
        for point in path:
            if not point.position_known():
                reasons.append(
                    "item %u (%s) has no stored position, so where it is "
                    "flown is only known in the air"
                    % (point.seq, command_name(point.command)))
            if point.is_curved():
                reasons.append(
                    "item %u is a %s, which is flown as a curve rather than "
                    "the straight leg used here"
                    % (point.seq, command_name(point.command)))
        for seq in self.loiters_without_radius(default_radius, path):
            reasons.append(
                "item %u is a loiter with no radius of its own, so its circle "
                "is not known without the vehicle's WP_LOITER_RAD" % seq)
        return reasons

    def corridor_index(self, default_radius=0.0, path=None):
        '''a PathIndex over the ground the mission covers.

           The loiter orbits are already part of the path, so there is nothing
           to add: the corridor is exactly what gets flown.
        '''
        if path is None:
            path = self.flown_path(loiter_radius=default_radius)
        return PathIndex(path)

    def expand_loiters(self, path, default_radius):
        '''replace each loiter with the orbit it actually flies.

           A loiter stored as one point made the corridor run straight through
           the middle of the circle, which is ground the aircraft never covers,
           and left every sample on that line rather than round the orbit, so
           an RTL was never started from where the aircraft would really be.
        '''
        out = []
        for (i, point) in enumerate(path):
            if not point.is_loiter():
                out.append(point)
                continue
            radius = point.loiter_radius(default_radius)
            if radius is None:
                # unknown radius, left as it was and reported by unmodelled()
                out.append(point)
                continue
            before = out[-1] if len(out) > 0 else None
            after = path[i + 1] if i + 1 < len(path) else None
            out.extend(self.orbit_points(point, radius, before, after))
        return out

    def orbit_points(self, loiter, radius, before, after):
        '''the circle a loiter flies, as points on it.

           Joined where the aircraft arrives, which is on its way to the
           waypoint, and left where its heading round the circle lines up with
           where it goes next, which is what ModeLoiter::isHeadingLinedUp()
           waits for. Leaving radially instead would put a right angle in the
           path that the aircraft never flies.
        '''
        # in an east/north frame the angle grows anticlockwise
        turn = 1.0 if loiter.loiter_ccw() else -1.0

        entry = 0.0
        if before is not None:
            (dx, dy) = (before.x - loiter.x, before.y - loiter.y)
            if dx != 0.0 or dy != 0.0:
                entry = math.atan2(dy, dx)

        # where carrying on round points the aircraft at the next waypoint.
        # Of the two tangent points it is the one this direction reaches
        # heading towards it rather than away
        exit_angle = entry + math.pi
        if after is not None:
            (dx, dy) = (after.x - loiter.x, after.y - loiter.y)
            span = math.hypot(dx, dy)
            if span > radius:
                offset = math.acos(max(-1.0, min(1.0, radius / span)))
                exit_angle = math.atan2(dy, dx) - turn * offset

        angles = []
        steps = LOITER_POINTS
        for step in range(steps + 1):
            angles.append(entry + turn * 2.0 * math.pi * step / steps)
        # and on round to where it leaves
        sweep = (turn * (exit_angle - entry)) % (2.0 * math.pi)
        extra = max(1, int(math.ceil(sweep / (2.0 * math.pi / steps))))
        for step in range(1, extra + 1):
            angles.append(entry + turn * sweep * step / extra)
        return [self.orbit_point(loiter, radius, angle) for angle in angles]

    def orbit_point(self, loiter, radius, angle):
        '''one point on a loiter orbit, as a mission point the checks can use
           like any other'''
        x = loiter.x + radius * math.cos(angle)
        y = loiter.y + radius * math.sin(angle)
        (lat, lon) = self.projector.unproject(x, y)
        point = MissionPoint(loiter.seq, loiter.command, loiter.frame,
                             lat, lon, loiter.alt,
                             loiter.param1, loiter.param2, loiter.param3)
        (point.x, point.y) = (x, y)
        point.amsl = loiter.amsl
        point.ground = loiter.ground
        # not a stored mission item, so nothing may turn it back into one
        point.synthetic = True
        return point


def build_mission(items, terrain_fn=None):
    '''build a Mission from mavlink mission items. items[0] is home, whose
       altitude is the reference for relative frames'''
    points = [MissionPoint(it.seq, it.command, it.frame, it.x, it.y, it.z,
                           getattr(it, 'param1', 0), getattr(it, 'param2', 0),
                           getattr(it, 'param3', 0))
              for it in items]
    if len(points) == 0:
        return Mission([], 0.0, Projector(0.0, 0.0))

    home_amsl = points[0].alt
    located = [p for p in points if p.has_location()]
    if len(located) == 0:
        return Mission(points, home_amsl,
                       Projector(points[0].lat, points[0].lon))

    # reference the projection at the middle of the mission so the longitude
    # scale error is spread rather than piling up at one end. Unwrap the
    # longitudes about the first point first, or a mission either side of the
    # antimeridian would be referenced half way round the world
    base_lon = located[0].lon
    lons = [base_lon + wrap_180(p.lon - base_lon) for p in located]
    lat0 = 0.5 * (min(p.lat for p in located) + max(p.lat for p in located))
    lon0 = wrap_180(0.5 * (min(lons) + max(lons)))
    projector = Projector(lat0, lon0)

    terrain_missing = 0
    for p in points:
        (p.x, p.y) = projector.project(p.lat, p.lon)
        if p.is_terrain_frame():
            p.ground = terrain_fn(p.lat, p.lon) if terrain_fn is not None else None
            if p.ground is None:
                terrain_missing += 1
                # no terrain, so treat the height above ground as being above
                # home. Wrong, but far closer than taking it as AMSL, which
                # would put a 90m AGL waypoint underground. The caller reports
                # how many items this happened to
                p.amsl = home_amsl + p.alt
            else:
                p.amsl = p.ground + p.alt
        elif p.frame in RELATIVE_FRAMES:
            p.amsl = home_amsl + p.alt
        else:
            # anything else is treated as absolute, the likeliest meaning of
            # an unexpected frame
            p.amsl = p.alt

    return Mission(points, home_amsl, projector, terrain_missing)


class PathSample(object):
    '''a point along the mission path'''

    def __init__(self, x, y, amsl, leg_from, leg_to, distance,
                 ux=0.0, uy=0.0):
        self.x = x
        self.y = y
        self.amsl = amsl
        # sequence numbers of the items at either end of the leg
        self.leg_from = leg_from
        self.leg_to = leg_to
        # distance along the whole path
        self.distance = distance
        # unit vector of the direction of travel, which is the heading the
        # aircraft has to turn from when an RTL starts here
        self.ux = ux
        self.uy = uy


class SampleStats(object):
    '''what happened while sampling, so the caller can report it'''

    def __init__(self):
        self.terrain_missing = 0


def sample_path(path, spacing, projector=None, terrain_fn=None, stats=None):
    '''sample a list of MissionPoints at the given spacing in metres.

       A leg into a terrain frame waypoint follows the terrain, as ArduPilot
       interpolates height above ground rather than AMSL for those legs, so
       the sample altitude is the ground beneath it plus the interpolated
       height above ground.
    '''
    samples = []
    spacing = max(float(spacing), MIN_SPACING)
    if len(path) == 0:
        return samples
    first = PathSample(path[0].x, path[0].y, path[0].amsl,
                       path[0].seq, path[0].seq, 0.0)
    samples.append(first)
    travelled = 0.0
    for i in range(1, len(path)):
        a = path[i - 1]
        b = path[i]
        leg = math.hypot(b.x - a.x, b.y - a.y)
        if leg <= 0:
            continue
        (ux, uy) = ((b.x - a.x) / leg, (b.y - a.y) / leg)
        if len(samples) == 1:
            # the first sample is at the start of the first real leg
            (first.ux, first.uy) = (ux, uy)
        # follow the terrain only if we can look it up and know where the
        # leg started in height above ground terms
        follow_terrain = (b.is_terrain_frame() and terrain_fn is not None and
                          projector is not None and b.ground is not None)
        start_agl = None
        if follow_terrain:
            if a.is_terrain_frame() and a.ground is not None:
                start_agl = a.alt
            elif a.ground is not None:
                start_agl = a.amsl - a.ground
            else:
                ground_a = terrain_fn(a.lat, a.lon)
                if ground_a is not None:
                    start_agl = a.amsl - ground_a
            if start_agl is None:
                follow_terrain = False
        steps = max(1, int(math.ceil(leg / spacing)))
        for step in range(1, steps + 1):
            frac = float(step) / steps
            x = a.x + (b.x - a.x) * frac
            y = a.y + (b.y - a.y) * frac
            amsl = None
            if follow_terrain:
                (slat, slon) = projector.unproject(x, y)
                ground = terrain_fn(slat, slon)
                if ground is not None:
                    amsl = ground + start_agl + (b.alt - start_agl) * frac
                elif stats is not None:
                    stats.terrain_missing += 1
            if amsl is None:
                amsl = a.amsl + (b.amsl - a.amsl) * frac
            samples.append(PathSample(x, y, amsl, a.seq, b.seq,
                                      travelled + leg * frac, ux, uy))
        travelled += leg
    return samples
