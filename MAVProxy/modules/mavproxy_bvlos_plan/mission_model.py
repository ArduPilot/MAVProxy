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
] + ([mavlink.MAV_CMD_NAV_ARC_WAYPOINT]
     if hasattr(mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT') else []))

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


class PathIndex(object):
    '''pre-chewed segments of a path, so the distance from a point to the
       whole path is a tight loop over plain floats'''

    def __init__(self, points):
        self.segments = []
        for i in range(1, len(points)):
            a = points[i - 1]
            b = points[i]
            vx = b.x - a.x
            vy = b.y - a.y
            d2 = vx * vx + vy * vy
            if d2 <= 0.0:
                continue
            self.segments.append((a.x, a.y, vx, vy, d2))
        self.fallback = (points[0].x, points[0].y) if len(points) else None

    def distance(self, px, py):
        '''smallest distance from a point to the path'''
        best = None
        for (ax, ay, vx, vy, d2) in self.segments:
            wx = px - ax
            wy = py - ay
            t = (vx * wx + vy * wy) / d2
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
            dx = wx - t * vx
            dy = wy - t * vy
            d = dx * dx + dy * dy
            if best is None or d < best:
                best = d
        if best is None:
            if self.fallback is None:
                return 0.0
            return math.hypot(px - self.fallback[0], py - self.fallback[1])
        return math.sqrt(best)


class MissionPoint(object):
    '''a mission item reduced to what the checks need'''

    def __init__(self, seq, command, frame, lat, lon, alt, param1=0, param2=0):
        self.seq = seq
        self.command = command
        self.frame = frame
        self.lat = lat
        self.lon = lon
        # altitude as stored, in its own frame
        self.alt = alt
        # DO_JUMP uses param1 as the target and param2 as the repeat count
        self.param1 = param1
        self.param2 = param2
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

    def __str__(self):
        return "%u:%s" % (self.seq, command_name(self.command))


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

    def flown_path(self):
        '''where the aircraft actually goes: navigation items with a location,
           after home, up to and including the first landing.

           Only navigation commands count. DO_SET_HOME, DO_SET_ROI,
           DO_LAND_START and DO_RETURN_PATH_START carry a location but are
           never flown to, and treating them as corridor would invent
           corridor that does not exist and could hide an unsafe cut.

           Anything past the landing is not flown either. Both reference
           BVLOS missions carry LOITER_TURNS and DO_JUMP pairs after the
           landing, which the operator selects rather than flies, and which
           ArduPilot's own return path walk never reaches because it stops at
           the landing.
        '''
        path = []
        for p in self.points[1:]:
            if p.is_nav() and p.has_location():
                path.append(p)
            if p.is_landing():
                break
        return path

    def ends_in_landing(self):
        '''True if the flown path finishes at a landing'''
        path = self.flown_path()
        return len(path) > 0 and path[-1].is_landing()


def build_mission(items, terrain_fn=None):
    '''build a Mission from mavlink mission items. items[0] is home, whose
       altitude is the reference for relative frames'''
    points = [MissionPoint(it.seq, it.command, it.frame, it.x, it.y, it.z,
                           getattr(it, 'param1', 0), getattr(it, 'param2', 0))
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

    def __init__(self, x, y, amsl, leg_from, leg_to, distance):
        self.x = x
        self.y = y
        self.amsl = amsl
        # sequence numbers of the items at either end of the leg
        self.leg_from = leg_from
        self.leg_to = leg_to
        # distance along the whole path
        self.distance = distance


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
    samples.append(PathSample(path[0].x, path[0].y, path[0].amsl,
                              path[0].seq, path[0].seq, 0.0))
    travelled = 0.0
    for i in range(1, len(path)):
        a = path[i - 1]
        b = path[i]
        leg = math.hypot(b.x - a.x, b.y - a.y)
        if leg <= 0:
            continue
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
                                      travelled + leg * frac))
        travelled += leg
    return samples
