'''
the path an ArduPlane flies along a mission

ArduPlane does not fly the lines drawn between its mission items.  Its L1
controller tracks each leg from wherever set_next_WP() says the leg starts,
which is the item before, the centre of a loiter just left, the point the
loiter was left at, or wherever the vehicle happened to be; it flies at a
loiter's centre until it is close enough to capture the circle, and it
leaves only once it is lined up on whatever comes next.  How the aircraft
then turns onto each of those tracks is the controller's own dynamics, so
rather than approximate them with geometry this ports the controller and
the mission logic around it, and flies a simple aircraft through them: a
fixed airspeed, turning with a coordinated bank that lags the controller's
demand, and climbing and descending within the rates TECS allows.

The ports follow AP_L1_Control and ArduPlane's commands_logic.cpp,
navigation.cpp, altitude.cpp and mode_loiter.cpp.

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util

GRAVITY = 9.80665
# ArduPilot's Location maths: metres per 1e-7 degree of latitude
LOCATION_SCALING_FACTOR = 0.011131884502145034

# ArduPlane's own defaults, for parameters a vehicle or log does not carry.
# Older firmware names some of them differently, in other units
PARAMETERS = {
    'NAVL1_PERIOD': ([('NAVL1_PERIOD', 1.0)], 17.0),
    'NAVL1_DAMPING': ([('NAVL1_DAMPING', 1.0)], 0.75),
    'NAVL1_XTRACK_I': ([('NAVL1_XTRACK_I', 1.0)], 0.02),
    'NAVL1_LIM_BANK': ([('NAVL1_LIM_BANK', 1.0)], 0.0),
    'ROLL_LIMIT': ([('ROLL_LIMIT_DEG', 1.0), ('LIM_ROLL_CD', 0.01)], 45.0),
    'RLL2SRV_TCONST': ([('RLL2SRV_TCONST', 1.0)], 0.5),
    'WP_RADIUS': ([('WP_RADIUS', 1.0)], 90.0),
    'WP_MAX_RADIUS': ([('WP_MAX_RADIUS', 1.0)], 0.0),
    'WP_LOITER_RAD': ([('WP_LOITER_RAD', 1.0)], 60.0),
    'AIRSPEED_CRUISE': ([('AIRSPEED_CRUISE', 1.0), ('TRIM_ARSPD_CM', 0.01)],
                        12.0),
    'AIRSPEED_MIN': ([('AIRSPEED_MIN', 1.0), ('ARSPD_FBW_MIN', 1.0)], 9.0),
    'AIRSPEED_MAX': ([('AIRSPEED_MAX', 1.0), ('ARSPD_FBW_MAX', 1.0)], 22.0),
    'TECS_CLMB_MAX': ([('TECS_CLMB_MAX', 1.0)], 5.0),
    'TECS_SINK_MAX': ([('TECS_SINK_MAX', 1.0)], 5.0),
    'TECS_TIME_CONST': ([('TECS_TIME_CONST', 1.0)], 5.0),
    'ALT_SLOPE_MIN': ([('ALT_SLOPE_MIN', 1.0)], 15.0),
    'CLIMB_SLOPE_HGT': ([('CLIMB_SLOPE_HGT', 1.0)], 25.0),
    'RTL_ALTITUDE': ([('RTL_ALTITUDE', 1.0), ('ALT_HOLD_RTL', 0.01)], 100.0),
    'RTL_RADIUS': ([('RTL_RADIUS', 1.0)], 0.0),
    'RALLY_LIMIT_KM': ([('RALLY_LIMIT_KM', 1.0)], 5.0),
    'RALLY_INCL_HOME': ([('RALLY_INCL_HOME', 1.0)], 0.0),
    'Q_ENABLE': ([('Q_ENABLE', 1.0)], 0.0),
    'Q_OPTIONS': ([('Q_OPTIONS', 1.0)], 0.0),
    'Q_RTL_MODE': ([('Q_RTL_MODE', 1.0)], 0.0),
    'Q_RTL_ALT': ([('Q_RTL_ALT', 1.0)], 15.0),
}
# QuadPlane's Q_OPTIONS bit for flying NAV_TAKEOFF as a fixed-wing takeoff
Q_OPTION_ALLOW_FW_TAKEOFF = 1 << 1
# QuadPlane::RTL_MODE, what Q_RTL_MODE makes of a return to launch
Q_RTL_DISABLED = 0
Q_RTL_SWITCH_QRTL = 1
Q_RTL_VTOL_APPROACH = 2
Q_RTL_QRTL_ALWAYS = 3

# how often ArduPlane navigates, and how finely the aircraft is flown
NAV_PERIOD = 0.1
# the drawn path gets a point at least this often, and whenever the aircraft
# has turned this far since the last one
POINT_SPACING = 20.0
POINT_TURN = math.radians(4.0)
# a mission is not flown for longer than this, and one item not for longer
# than it takes to fly its leg this many times over, plus the time its
# loiter asks for -- the turns or the time, or the climb or descent -- and
# MAX_LAPS laps more: past that the aircraft is taken to be stuck, circling
# a waypoint it cannot turn tightly enough to reach.  Flying a mission takes
# a second or so of work for every hour of it, so one which plainly takes
# longer than this is not attempted
MAX_FLIGHT_TIME = 4 * 3600.0
LEG_ALLOWANCE = 4.0
MAX_LAPS = 20
LOITER_COMMANDS = (
    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
)
WAYPOINT_COMMANDS = (
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
)
TAKEOFF_COMMANDS = (
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
)
# navigation commands numbered past MAV_CMD_NAV_LAST
LATE_NAV_COMMANDS = (42702, 42703)     # NAV_SCRIPT_TIME, NAV_ATTITUDE_TIME
MAV_CMD_JUMP_TAG = 600
MAV_CMD_DO_JUMP_TAG = 601
# ArduPlane rejects these, and moves straight on to the item after
SKIPPED_COMMANDS = (
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    mp_util.MAV_CMD_NAV_ARC_WAYPOINT,
)


def parameter(params, name):
    '''a parameter the flight depends on, from params or ArduPlane's
    default'''
    (names, default) = PARAMETERS[name]
    for (param, scale) in names:
        value = mp_util.param_value(params, param)
        if value is not None and not math.isnan(value):
            return float(value) * scale
    return default


def eas2tas(amsl):
    '''the true airspeed of each metre per second of equivalent airspeed at
    an altitude, in the standard atmosphere'''
    temperature = max(288.15 - 0.0065 * amsl, 150.0)
    return 1.0 / math.sqrt((temperature / 288.15) ** 4.2559)


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def wrap_longitude(dlon):
    '''a difference in longitude, the short way round: Location's
    diff_longitude()'''
    return (dlon + 180.0) % 360.0 - 180.0


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def minus(a, b):
    return (a[0] - b[0], a[1] - b[1])


def norm(a):
    return math.hypot(a[0], a[1])


def unit(a):
    length = norm(a)
    return (a[0] / length, a[1] / length)


def path_proportion(point, start, end):
    '''Location::line_path_proportion: how far along start->end point is,
    more than 1 once past end'''
    leg = minus(end, start)
    squared = dot(leg, leg)
    if squared < 0.001:
        return 1.0
    return dot(leg, minus(point, start)) / squared


class L1Control(object):
    '''AP_L1_Control, working in metres north and east'''

    def __init__(self, period, damping, xtrack_i_gain, bank_limit):
        self.period = period
        self.damping = damping
        self.xtrack_i_gain = xtrack_i_gain
        self.bank_limit = bank_limit
        self.xtrack_i = 0.0
        self.last_nu = 0.0
        self.lateral_acceleration = 0.0
        self.circling = False
        self.distance = 0.0
        self.target_bearing = 0.0

    def loiter_radius(self, radius, amsl, cruise):
        '''the radius actually flown for a loiter radius at an altitude'''
        scale = eas2tas(amsl) ** 2
        bank = min(max(self.bank_limit, 0.0), 89.0)
        if bank <= 0 or cruise <= 0:
            return radius * scale
        sea_level = cruise ** 2 / (math.tan(math.radians(bank)) * GRAVITY)
        if sea_level > radius:
            return radius * scale
        return max(sea_level * scale, radius)

    def turn_distance(self, wp_radius, amsl, turn_angle):
        distance = min(wp_radius * eas2tas(amsl) ** 2, self.distance)
        turn_angle = abs(turn_angle)
        if turn_angle >= 90:
            return distance
        return distance * turn_angle / 90.0

    def _prevent_indecision(self, nu, yaw):
        limit = 0.9 * math.pi
        if (abs(nu) > limit and abs(self.last_nu) > limit and
                abs(wrap_pi(self.target_bearing - yaw)) > math.radians(120) and
                nu * self.last_nu < 0):
            return self.last_nu
        return nu

    def update_waypoint(self, position, velocity, yaw, start, end, dt):
        k_l1 = 4.0 * self.damping ** 2
        speed = max(norm(velocity), 0.1)
        self.target_bearing = math.atan2(end[1] - position[1],
                                         end[0] - position[0])
        self.distance = 0.3183099 * self.damping * self.period * speed
        track = minus(end, start)
        track_length = norm(track)
        if track_length < 1.0e-6:
            track = minus(end, position)
            if norm(track) < 1.0e-6:
                track = (math.cos(yaw), math.sin(yaw))
        track = unit(track)
        from_start = minus(position, start)
        crosstrack = cross(from_start, track)
        start_distance = norm(from_start)
        along = dot(from_start, track)
        if (start_distance > self.distance and
                along / max(start_distance, 1.0) < -0.7071):
            # behind the start of the leg: fly to its start
            towards = unit(from_start)
            towards = (-towards[0], -towards[1])
            nu = math.atan2(cross(velocity, towards), dot(velocity, towards))
        elif along > track_length + speed * 3:
            # three seconds past the end of it: fly back to the end
            towards = unit(minus(end, position))
            nu = math.atan2(cross(velocity, towards), dot(velocity, towards))
        else:
            nu2 = math.atan2(cross(velocity, track), dot(velocity, track))
            sine_nu1 = crosstrack / max(self.distance, 0.1)
            nu1 = math.asin(min(max(sine_nu1, -0.7071), 0.7071))
            if self.xtrack_i_gain <= 0:
                self.xtrack_i = 0.0
            elif abs(nu1) < math.radians(5):
                self.xtrack_i += nu1 * self.xtrack_i_gain * dt
                self.xtrack_i = min(max(self.xtrack_i, -0.1), 0.1)
            nu = nu1 + self.xtrack_i + nu2
        nu = self._prevent_indecision(nu, yaw)
        self.last_nu = nu
        nu = min(max(nu, -1.5708), 1.5708)
        self.lateral_acceleration = k_l1 * speed * speed / self.distance * math.sin(nu)
        self.circling = False

    def update_loiter(self, position, velocity, yaw, centre, radius, direction):
        omega = 2 * math.pi / self.period
        kx = omega * omega
        kv = 2.0 * self.damping * omega
        k_l1 = 4.0 * self.damping ** 2
        speed = max(norm(velocity), 1.0)
        self.target_bearing = math.atan2(centre[1] - position[1],
                                         centre[0] - position[0])
        self.distance = 0.3183099 * self.damping * self.period * speed
        from_centre = minus(position, centre)
        if norm(from_centre) > 0.1:
            outwards = unit(from_centre)
        else:
            outwards = unit(velocity)
        # capturing the centre, as a waypoint
        xtrack_velocity = cross(outwards, velocity)
        ltrack_velocity = -dot(velocity, outwards)
        nu = self._prevent_indecision(
            math.atan2(xtrack_velocity, ltrack_velocity), yaw)
        self.last_nu = nu
        nu = min(max(nu, -math.pi / 2), math.pi / 2)
        capture = k_l1 * speed * speed / self.distance * math.sin(nu)
        # tracking the circle
        radial_velocity = -ltrack_velocity
        radial_error = norm(from_centre) - radius
        pd = radial_error * kx + radial_velocity * kv
        tangential = xtrack_velocity * direction
        centripetal = tangential ** 2 / max(0.5 * radius, radius + radial_error)
        if ltrack_velocity < 0:
            if tangential < 0:
                pd = max(pd, 0.0)
            elif radial_error < 0:
                pd = max(pd, -centripetal)
        circle = direction * (pd + centripetal)
        # the switch from capture to circling happens where the two demands
        # cross, and only outside the circle
        if radial_error > 0 and direction * capture < direction * circle:
            self.lateral_acceleration = capture
            self.circling = False
        else:
            self.lateral_acceleration = circle
            self.circling = True


class MissionFlight(object):
    '''ArduPlane flying a mission in AUTO, from home'''

    def __init__(self, origin, home, items, params, heading=None, start=None,
                 rally=None):
        self.origin = origin
        self.lon_scale = math.cos(math.radians(origin[0]))
        self.params = params
        self.l1 = L1Control(parameter(params, 'NAVL1_PERIOD'),
                            parameter(params, 'NAVL1_DAMPING'),
                            parameter(params, 'NAVL1_XTRACK_I'),
                            parameter(params, 'NAVL1_LIM_BANK'))
        self.cruise = parameter(params, 'AIRSPEED_CRUISE')
        self.airspeed = self.cruise
        self.airspeed_min = parameter(params, 'AIRSPEED_MIN')
        self.airspeed_max = parameter(params, 'AIRSPEED_MAX')
        self.roll_limit = math.radians(parameter(params, 'ROLL_LIMIT'))
        self.roll_tconst = max(parameter(params, 'RLL2SRV_TCONST'), 0.05)
        self.wp_radius = parameter(params, 'WP_RADIUS')
        self.wp_max_radius = parameter(params, 'WP_MAX_RADIUS')
        self.loiter_radius = parameter(params, 'WP_LOITER_RAD')
        self.climb = parameter(params, 'TECS_CLMB_MAX')
        self.sink = parameter(params, 'TECS_SINK_MAX')
        self.alt_tconst = max(parameter(params, 'TECS_TIME_CONST'), 0.5)
        self.alt_slope_min = parameter(params, 'ALT_SLOPE_MIN')
        self.climb_slope_height = parameter(params, 'CLIMB_SLOPE_HGT')
        self.rtl_altitude = parameter(params, 'RTL_ALTITUDE')
        self.rtl_radius = parameter(params, 'RTL_RADIUS')
        self.rally_limit = parameter(params, 'RALLY_LIMIT_KM') * 1000.0
        self.rally_incl_home = parameter(params, 'RALLY_INCL_HOME') > 0
        self.quadplane = parameter(params, 'Q_ENABLE') > 0
        self.q_options = int(parameter(params, 'Q_OPTIONS'))
        self.q_rtl_mode = int(parameter(params, 'Q_RTL_MODE'))
        self.q_rtl_alt = parameter(params, 'Q_RTL_ALT')
        # the jumps followed so far, by index
        self.jumps_taken = set()
        self.home = self.local(home[0], home[1])
        self.home_amsl = home[2]
        self.rally = [(self.local(lat, lon), amsl)
                      for (lat, lon, amsl) in (rally or [])]
        self.items = items
        # the course a fixed-wing takeoff holds, where there is something to
        # go on
        self.takeoff_heading = heading
        # where the aircraft is when the mission starts: home, unless it
        # started somewhere else
        self.position = self.home
        self.amsl = home[2]
        if start is not None:
            self.position = self.local(start[0], start[1])
            if start[2] is not None:
                self.amsl = start[2]
        self.yaw = 0.0
        self.roll = 0.0
        self.time = 0.0
        self.points = []
        self.last_point = None
        # what set_next_WP() keeps
        self.prev_wp = (self.position, self.amsl)
        self.next_wp = (self.position, self.amsl)
        self.crosstrack = False
        self.next_wp_crosstrack = False
        self.offset_altitude = 0.0
        self.target_amsl = self.amsl

    def local(self, lat, lon):
        return ((lat - self.origin[0]) * 1.0e7 * LOCATION_SCALING_FACTOR,
                wrap_longitude(lon - self.origin[1]) * 1.0e7 *
                LOCATION_SCALING_FACTOR * self.lon_scale)

    def latlon(self, point):
        '''back from local().  The longitude is left unwrapped, so a path
        across the antimeridian stays continuous, as the 3D map's rings do'''
        return (self.origin[0] + point[0] / (1.0e7 * LOCATION_SCALING_FACTOR),
                self.origin[1] + point[1] / (1.0e7 * LOCATION_SCALING_FACTOR *
                                             self.lon_scale))

    def location(self, item):
        '''an item's location and altitude.  An item with no position of its
        own is flown where the aircraft is, and one with no altitude at the
        one it has'''
        (command, lat, lon, amsl, params) = item
        if lat == 0 and lon == 0:
            point = self.position
        else:
            point = self.local(lat, lon)
        if amsl is None:
            amsl = self.amsl
        return (point, amsl)

    def jump_target(self, index):
        '''the index a DO_JUMP or DO_JUMP_TAG at index goes to, or None if it
        goes nowhere there is'''
        (command, lat, lon, amsl, params) = self.items[index]
        target = int(params[0] or 0)
        if command == MAV_CMD_DO_JUMP_TAG:
            for (i, item) in enumerate(self.items):
                if item[0] == MAV_CMD_JUMP_TAG and int(item[4][0] or 0) == target:
                    return i
            return None
        # items are numbered from home, which is not among them
        if target < 1 or target > len(self.items):
            return None
        return target - 1

    def jumps(self, index):
        '''whether the jump at index is followed.  AP_Mission follows one
        its repeat count, or for ever with a count of -1; drawn, each is
        followed once, so the path goes round a loop the once'''
        repeats = int(self.items[index][4][1] or 0)
        return repeats != 0 and index not in self.jumps_taken

    def loops_forever(self, index):
        '''whether the jump at index has already been followed and is
        followed for ever, so that the mission never gets past it'''
        repeats = int(self.items[index][4][1] or 0)
        return repeats < 0 and index in self.jumps_taken

    def next_nav_index(self, index, take=False, passed=None):
        '''AP_Mission::get_next_nav_cmd: the index of the next item after
        index which ArduPlane navigates to, following jumps, or None.  take
        counts the jumps as followed, and passed collects the indexes of the
        other commands on the way'''
        i = index + 1
        followed = set()
        while i < len(self.items):
            command = self.items[i][0]
            if command in (mavutil.mavlink.MAV_CMD_DO_JUMP, MAV_CMD_DO_JUMP_TAG):
                if i in followed:
                    # round in a circle without navigating anywhere
                    return None
                if self.loops_forever(i):
                    # the aircraft goes round this loop for ever, and has
                    # been drawn going round it once
                    return None
                if self.jumps(i):
                    target = self.jump_target(i)
                    if target is None:
                        return None
                    followed.add(i)
                    if take:
                        self.jumps_taken.add(i)
                    i = target
                    continue
                i += 1
                continue
            if command in SKIPPED_COMMANDS:
                i += 1
                continue
            if (command in LOITER_COMMANDS or command in WAYPOINT_COMMANDS or
                    command in TAKEOFF_COMMANDS or
                    command < mavutil.mavlink.MAV_CMD_NAV_LAST or
                    command in LATE_NAV_COMMANDS):
                # including a navigation command this does not know how to fly
                return i
            if passed is not None:
                passed.append(i)
            i += 1
        return None

    def add_point(self, force=False):
        if self.last_point is not None and not force:
            (point, yaw) = self.last_point
            if (norm(minus(self.position, point)) < POINT_SPACING and
                    abs(wrap_pi(self.yaw - yaw)) < POINT_TURN):
                return
        self.last_point = (self.position, self.yaw)
        (lat, lon) = self.latlon(self.position)
        self.points.append((lat, lon, self.amsl))

    def set_next_wp(self, location):
        '''Plane::set_next_WP'''
        if self.next_wp_crosstrack:
            self.prev_wp = self.next_wp
            self.crosstrack = True
        else:
            self.prev_wp = (self.position, self.amsl)
            self.next_wp_crosstrack = True
            self.crosstrack = False
        self.next_wp = location
        if path_proportion(self.position, self.prev_wp[0], self.next_wp[0]) >= 1:
            self.prev_wp = (self.position, self.amsl)
        # Plane::set_offset_altitude_location
        self.offset_altitude = self.next_wp[1] - self.prev_wp[1]
        if (self.alt_slope_min <= 0 or
                abs(self.offset_altitude) < self.alt_slope_min):
            self.offset_altitude = 0.0

    def update_target_altitude(self):
        '''Mode::update_target_altitude, for AUTO away from landing'''
        (end, end_amsl) = self.next_wp
        (start, start_amsl) = self.prev_wp
        if self.l1.circling:
            self.target_amsl = end_amsl
        elif (self.offset_altitude != 0 and
                path_proportion(self.position, start, end) < 1):
            if (self.offset_altitude > 0 and
                    self.amsl - self.home_amsl < self.climb_slope_height):
                # a full rate climb up to CLIMB_SLOPE_HGT
                self.target_amsl = end_amsl
                return
            proportion = path_proportion(self.position, start, end)
            proportion = min(max(1.0 - proportion, 0.0), 1.0)
            target = end_amsl - self.offset_altitude * proportion
            self.target_amsl = min(max(target, min(start_amsl, end_amsl)),
                                   max(start_amsl, end_amsl))
        else:
            self.target_amsl = end_amsl

    def fly(self, lateral_acceleration, dt, climb=None):
        '''move the aircraft on by dt, banking towards the controller's
        demand, and climbing or descending towards the target altitude, or
        at the climb rate given'''
        demand = math.atan(lateral_acceleration / GRAVITY)
        demand = min(max(demand, -self.roll_limit), self.roll_limit)
        self.roll += (demand - self.roll) * (1.0 - math.exp(-dt / self.roll_tconst))
        speed = self.speed()
        self.yaw = wrap_pi(self.yaw + GRAVITY * math.tan(self.roll) / speed * dt)
        self.position = (self.position[0] + speed * math.cos(self.yaw) * dt,
                         self.position[1] + speed * math.sin(self.yaw) * dt)
        if climb is None:
            rate = (self.target_amsl - self.amsl) / self.alt_tconst
            climb = min(max(rate, -self.sink), self.climb)
        self.amsl += climb * dt
        self.time += dt
        self.add_point()

    def speed(self):
        return self.airspeed * eas2tas(self.amsl)

    def velocity(self):
        speed = self.speed()
        return (speed * math.cos(self.yaw), speed * math.sin(self.yaw))

    def turn_angle(self, index):
        '''Plane::setup_turn_angle: the turn at the end of this leg'''
        following = self.next_nav_index(index)
        if following is None:
            return 90.0
        (end, _) = self.next_wp
        (after, _) = self.location(self.items[following])
        if norm(minus(after, end)) < 1.0e-6:
            return 90.0
        leg = math.atan2(end[1] - self.prev_wp[0][1], end[0] - self.prev_wp[0][0])
        course = math.atan2(after[1] - end[1], after[0] - end[0])
        return math.degrees(wrap_pi(course - leg))

    def time_limit(self, extra=0.0):
        (end, _) = self.next_wp
        leg = norm(minus(end, self.position))
        limit = self.time + 120.0 + LEG_ALLOWANCE * leg / self.speed() + extra
        return min(limit, MAX_FLIGHT_TIME)

    def rtl_location(self, home_amsl=None):
        '''AP_Rally::calc_best_rally_or_home_location: home, at home_amsl
        where that is given, and otherwise RTL_ALTITUDE above it, or at the
        altitude the aircraft is at if that is negative; or the rally point
        nearest the aircraft, at its own altitude, where it is within
        RALLY_LIMIT_KM and, with RALLY_INCL_HOME set, nearer than home.  A
        rally point whose altitude is not known raises, since nobody can say
        how high the aircraft flies to it'''
        if home_amsl is not None:
            best = (self.home, home_amsl)
        elif self.rtl_altitude < 0:
            best = (self.home, self.amsl)
        else:
            best = (self.home, self.home_amsl + self.rtl_altitude)
        nearest = None
        for (point, amsl) in self.rally:
            distance = norm(minus(point, self.position))
            if nearest is None or distance < nearest[0]:
                nearest = (distance, point, amsl)
        if nearest is None:
            return best
        (distance, point, amsl) = nearest
        if self.rally_limit > 0 and distance > self.rally_limit:
            return best
        if (self.rally_incl_home and
                distance >= norm(minus(self.home, self.position))):
            return best
        if amsl is None:
            # the caller could not work out how high the point is -- terrain
            # it does not have, most likely -- so nobody can say how high
            # the aircraft flies its return, and it is not drawn flown
            raise ValueError('rally point altitude unknown')
        return (point, amsl)

    def fly_waypoint(self, index):
        '''Plane::verify_nav_wp.  False if the aircraft never gets there'''
        (command, lat, lon, amsl, params) = self.items[index]
        acceptance = 0.0
        passby = 0.0
        if command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            acceptance = min(max(params[1] or 0.0, 0.0), 255.0)
            passby = min(max(params[2] or 0.0, 0.0), 255.0)
        final = command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        turn_angle = self.turn_angle(index)
        limit = self.time_limit()
        while self.time < limit:
            (start, _) = self.prev_wp
            (end, _) = self.next_wp
            if passby > 0 and norm(minus(end, start)) > 0:
                along = unit(minus(end, start))
                end = (end[0] + along[0] * passby, end[1] + along[1] * passby)
            track_start = start if self.crosstrack else self.position
            self.l1.update_waypoint(self.position, self.velocity(), self.yaw,
                                    track_start, end, NAV_PERIOD)
            self.update_target_altitude()
            if (not final and self.wp_max_radius > 0 and
                    norm(minus(self.next_wp[0], self.position)) >
                    self.wp_max_radius):
                # WP_MAX_RADIUS: not reached until this close, however far
                # past it the aircraft has flown.  Having flown past, it
                # comes back round for it along a track from where it is
                if (passby == 0 and
                        path_proportion(self.position, start, end) >= 1):
                    self.prev_wp = (self.position, self.amsl)
                self.fly(self.l1.lateral_acceleration, NAV_PERIOD)
                continue
            if final:
                accept = 0.0
            elif acceptance > 0:
                accept = acceptance
            elif passby == 0:
                accept = self.l1.turn_distance(self.wp_radius, self.amsl,
                                               turn_angle)
            else:
                accept = 0.0
            if (norm(minus(end, self.position)) <= max(accept, 1.0) or
                    path_proportion(self.position, start, end) >= 1):
                return True
            self.fly(self.l1.lateral_acceleration, NAV_PERIOD)
        return False

    def fly_loiter(self, index):
        '''Plane::verify_loiter_*, flying update_loiter() until the item is
        done.  False if it never is'''
        mavlink = mavutil.mavlink
        (command, lat, lon, amsl, params) = self.items[index]
        if command == mavlink.MAV_CMD_NAV_LOITER_TIME:
            requested = 0.0
            ccw = (params[2] or 0.0) < 0
        elif command == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            requested = params[1] or 0.0
            ccw = requested < 0
        else:
            requested = params[2] or 0.0
            ccw = requested < 0
        radius = abs(requested)
        direction = -1 if ccw else 1
        if radius <= 1:
            # Plane::update_loiter: WP_LOITER_RAD, turned the way its sign says
            # unless the item asks for counter-clockwise
            radius = abs(self.loiter_radius) if abs(self.loiter_radius) > 1 else 60.0
            if not ccw:
                direction = -1 if self.loiter_radius < 0 else 1
        if command == mavlink.MAV_CMD_NAV_LOITER_TURNS and radius > 255:
            radius = min(255.0, radius * 0.1) * 10.0
        exit_from_here = not mp_util.mission_crosstracks_from_centre(command,
                                                                     params)
        # AP_Mission keeps whole turns in a byte, and a turn less than one
        # in 256ths of one
        turns = min(max(params[0] or 0.0, 0.0), 255.0)
        if turns >= 1:
            turns = float(int(turns))
        else:
            turns = int(turns * 256) / 256.0
        loiter_time = max(params[0] or 0.0, 0.0)
        started = False
        start_time = None
        swept = 0.0
        old_bearing = None
        reached_alt = False
        unable = False
        lap_check = 3 * 360.0
        lap_alt = self.amsl
        lining_up = False
        following = self.next_nav_index(index)
        seconds_per_lap = 2 * math.pi * radius * 1.5 / max(self.speed(), 1.0)
        extra = MAX_LAPS * seconds_per_lap
        if command == mavlink.MAV_CMD_NAV_LOITER_TIME:
            extra += loiter_time
        elif command == mavlink.MAV_CMD_NAV_LOITER_TURNS:
            extra += turns * seconds_per_lap
        elif command == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            (_, centre_amsl) = self.next_wp
            rate = self.climb if centre_amsl > self.amsl else self.sink
            extra += 1.5 * abs(centre_amsl - self.amsl) / max(rate, 0.1)
        limit = self.time_limit(extra)
        while self.time < limit:
            (centre, centre_amsl) = self.next_wp
            (start, _) = self.prev_wp
            flown = self.l1.loiter_radius(radius, self.amsl, self.airspeed)
            if (not started and self.crosstrack and
                    norm(minus(centre, self.position)) > 3 * flown):
                self.l1.update_waypoint(self.position, self.velocity(), self.yaw,
                                        start, centre, NAV_PERIOD)
            else:
                self.l1.update_loiter(self.position, self.velocity(), self.yaw,
                                      centre, flown, direction)
            if not started and (self.l1.circling or
                                path_proportion(self.position, start, centre) > 1):
                started = True
                start_time = self.time
            # Plane::loiter_angle_update
            reached = self.l1.circling
            if swept == 0 and not reached:
                delta = 0.0
            elif swept == 0:
                delta = 0.01
                lap_alt = self.amsl
                lap_check = 3 * 360.0
            else:
                delta = math.degrees(wrap_pi(self.l1.target_bearing - old_bearing))
            old_bearing = self.l1.target_bearing
            swept += delta * direction
            if reached and abs(self.amsl - centre_amsl) < 5.0:
                reached_alt = True
            elif not reached_alt and abs(swept) >= lap_check:
                unable = abs(self.amsl - lap_alt) < 5.0
                lap_alt = self.amsl
                lap_check += 3 * 360.0
            self.update_target_altitude()
            if not lining_up:
                if command == mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                    if reached and swept >= 360.0:
                        return False
                elif command == mavlink.MAV_CMD_NAV_LOITER_TURNS:
                    lining_up = reached and swept > turns * 360.0 and swept > 0.01
                elif command == mavlink.MAV_CMD_NAV_LOITER_TIME:
                    lining_up = (start_time is not None and swept > 0.01 and
                                 self.time - start_time > loiter_time)
                else:
                    lining_up = abs(swept) > 0.01 and (reached_alt or unable)
                if lining_up:
                    swept = 0.0
                    old_bearing = self.l1.target_bearing
            if lining_up and (reached or
                              command != mavlink.MAV_CMD_NAV_LOITER_TURNS):
                if following is None:
                    return True
                if self.lined_up(centre, flown, direction, swept,
                                 self.location(self.items[following])[0]):
                    if exit_from_here:
                        self.next_wp = (self.position, centre_amsl)
                    return True
            self.fly(self.l1.lateral_acceleration, NAV_PERIOD)
        return False

    def lined_up(self, centre, radius, direction, swept, target):
        '''ModeLoiter::isHeadingLinedUp'''
        if radius <= 0:
            return True
        projected = minus(self.position, centre)
        if norm(projected) < 1.0e-6:
            projected = (math.cos(self.yaw), math.sin(self.yaw))
        projected = unit(projected)
        projected = (projected[0] * radius, projected[1] * radius)
        to_target = minus(target, centre)
        target_distance = norm(to_target)
        if target_distance <= 0:
            return True
        if target_distance >= radius:
            bearing = math.atan2(to_target[1] - projected[1],
                                 to_target[0] - projected[0])
        else:
            chord = radius - target_distance
            segment = 2.0 * math.asin(chord / (2.0 * radius))
            bearing = wrap_pi(math.atan2(to_target[1], to_target[0]) +
                              (math.pi / 2 - segment) * direction)
        heading = wrap_pi(math.atan2(projected[1], projected[0]) +
                          math.pi / 2 * direction)
        acceptance = 10.0 + 10.0 * int(abs(swept) / 360.0)
        return abs(math.degrees(wrap_pi(bearing - heading))) <= acceptance

    def vtol_takeoff(self, command):
        '''QuadPlane::is_vtol_takeoff'''
        if command == mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF:
            return True
        return (command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and
                self.quadplane and
                not self.q_options & Q_OPTION_ALLOW_FW_TAKEOFF)

    def face(self, index):
        '''turn to face the item at index, if it is somewhere else'''
        if index is None:
            return
        (point, _) = self.location(self.items[index])
        if norm(minus(point, self.position)) > 1.0:
            self.yaw = math.atan2(point[1] - self.position[1],
                                  point[0] - self.position[0])

    def fly_takeoff(self, index):
        '''climb out to the takeoff altitude.  A VTOL takeoff climbs straight
        up where the aircraft is, and sets off towards where the mission goes
        next.  A fixed-wing one climbs away at full rate until it is past the
        takeoff altitude, holding the ground course it had once it got
        moving (Plane::verify_takeoff), which nothing in the mission says:
        the heading the caller has for it, which can only be an estimate
        until the aircraft has flown, and otherwise, as a guess, towards
        where the mission goes next'''
        (command, lat, lon, amsl, params) = self.items[index]
        (point, target) = self.location(self.items[index])
        if self.vtol_takeoff(command):
            self.amsl = max(self.amsl, target)
            self.face(self.next_nav_index(index))
            self.add_point(force=True)
        else:
            if self.takeoff_heading is not None:
                self.yaw = wrap_pi(math.radians(self.takeoff_heading))
            else:
                self.face(self.next_nav_index(index))
            limit = min(self.time + 600.0, MAX_FLIGHT_TIME)
            while self.amsl < target and self.time < limit:
                self.target_amsl = target
                self.fly(0.0, NAV_PERIOD, climb=self.climb)
        # Plane::verify_takeoff: no crosstracking the leg after a takeoff
        self.next_wp = self.prev_wp = (self.position, self.amsl)
        self.next_wp_crosstrack = False
        self.target_amsl = self.amsl
        return True

    def change_speed(self, params):
        '''Plane::do_change_speed, for the airspeed it flies at.  Only an
        airspeed change, of -2 for the cruise airspeed or one within
        AIRSPEED_MIN and AIRSPEED_MAX, changes it: a groundspeed change sets
        a minimum ground speed, which there is no wind here to call on'''
        (speed_type, speed) = (params[0] or 0.0, params[1] or 0.0)
        if int(speed_type) != 0:
            return
        if speed == -2:
            self.airspeed = self.cruise
        elif self.airspeed_min <= speed <= self.airspeed_max:
            self.airspeed = speed

    def run(self):
        '''fly the mission.  Returns the path flown, or None where it cannot
        be flown through to its end'''
        mavlink = mavutil.mavlink
        passed = []
        index = self.next_nav_index(-1, take=True, passed=passed)
        if index is None:
            return None
        self.face(index)
        self.add_point(force=True)
        while index is not None:
            if self.time > MAX_FLIGHT_TIME:
                return None
            for i in passed:
                if self.items[i][0] == mavlink.MAV_CMD_DO_CHANGE_SPEED:
                    self.change_speed(self.items[i][4])
            passed = []
            command = self.items[index][0]
            if command in TAKEOFF_COMMANDS:
                self.fly_takeoff(index)
                index = self.next_nav_index(index, take=True, passed=passed)
                continue
            if command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                return self.fly_rtl(index)
            if command in WAYPOINT_COMMANDS or command in LOITER_COMMANDS:
                location = self.location(self.items[index])
            else:
                return None
            self.set_next_wp(location)
            if command in LOITER_COMMANDS:
                finished = self.fly_loiter(index)
                if not finished and command == mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                    break
            else:
                finished = self.fly_waypoint(index)
            if not finished:
                return None
            if command in (mavlink.MAV_CMD_NAV_LAND,
                           mavlink.MAV_CMD_NAV_VTOL_LAND):
                break
            index = self.next_nav_index(index, take=True, passed=passed)
        self.add_point(force=True)
        return self.points

    def fly_rtl(self, index):
        '''a return to launch, which ArduPlane flies in RTL mode, or QRTL
        where Q_RTL_MODE has a QuadPlane do so.  It ends the flight: the
        path returned is the one flown'''
        mavlink = mavutil.mavlink
        # Plane::do_RTL: flown from wherever the aircraft is
        self.next_wp_crosstrack = False
        qrtl = self.quadplane and self.q_rtl_mode in (Q_RTL_SWITCH_QRTL,
                                                      Q_RTL_QRTL_ALWAYS)
        if self.quadplane and self.q_rtl_mode == Q_RTL_QRTL_ALWAYS:
            # ModeRTL::_enter goes straight to QRTL, which returns at
            # Q_RTL_ALT
            location = self.rtl_location(self.home_amsl + self.q_rtl_alt)
        else:
            location = self.rtl_location()
        self.set_next_wp(location)
        if qrtl:
            # the aircraft flies at the point along a track from where it
            # turned for it (Plane::update_loiter_update_nav), and QRTL
            # takes it the rest of the way, to land there
            self.crosstrack = True
            if not self.fly_waypoint(index):
                return None
        else:
            # ModeRTL::update: the aircraft circles where it returns to, at
            # RTL_RADIUS -- WP_LOITER_RAD where that is zero -- the way its
            # sign says.  It is flown by update_loiter() from the start,
            # with no track to follow there since do_RTL() leaves none, so
            # it joins the circle rather than flying over its middle
            self.items.append((mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                               location[0][0], location[0][1], location[1],
                               (0.0, 0.0, self.rtl_radius, 0.0)))
            self.fly_loiter(len(self.items) - 1)
        self.add_point(force=True)
        return self.points


def is_navigation_command(command):
    '''whether ArduPlane flies command, rather than just doing it'''
    return (command < mavutil.mavlink.MAV_CMD_NAV_LAST or
            command in LATE_NAV_COMMANDS)


def positionless_amsl(alt, frame, home_amsl):
    '''the AMSL altitude an item with no position of its own is flown at,
    or None for the altitude the aircraft is at.  Location::sanitize() puts
    such an item where the aircraft is, but keeps its altitude unless that
    is a relative 0; a terrain-relative one needs the terrain under wherever
    that is, which is not known here'''
    if alt is None:
        return None
    if frame in (0, 5):
        return alt
    if alt == 0 or frame not in (3, 6) or home_amsl is None:
        return None
    return home_amsl + alt


# Location::AltFrame, as a rally point's flags carry it
RALLY_ALT_ABSOLUTE = 0
RALLY_ALT_ABOVE_HOME = 1
RALLY_ALT_ABOVE_ORIGIN = 2
RALLY_ALT_ABOVE_TERRAIN = 3


def rally_alt_frame(flags):
    '''the frame a rally point's altitude is in.
    AP_Rally::rally_location_to_location(): above home, unless the flags say
    the frame they carry is a valid one'''
    if int(flags or 0) & (1 << 2):
        return (int(flags) >> 3) & 3
    return RALLY_ALT_ABOVE_HOME


def rally_alt_is_fixed(flags):
    '''whether a rally point is at an altitude of its own rather than one
    which moves with home'''
    return rally_alt_frame(flags) != RALLY_ALT_ABOVE_HOME


def rally_amsl(alt, flags, home_amsl, origin_amsl=None, terrain_amsl=None):
    '''the AMSL altitude of a rally point, or None where it cannot be told.

    Location::get_alt_cm() measures the frame the flags carry from its own
    datum: home, the EKF origin, or the terrain below the point, which the
    caller looks up where it can'''
    if alt is None:
        return None
    frame = rally_alt_frame(flags)
    if frame == RALLY_ALT_ABSOLUTE:
        return float(alt)
    datum = {RALLY_ALT_ABOVE_HOME: home_amsl,
             RALLY_ALT_ABOVE_ORIGIN: origin_amsl,
             RALLY_ALT_ABOVE_TERRAIN: terrain_amsl}[frame]
    if datum is None:
        return None
    return datum + alt


def least_flight_time(home, items, params, start=None):
    '''the least time flying the mission could take: straight from each
    item it navigates to on to the next, in the order the flight takes them
    -- following jumps as it does -- at the fastest airspeed it may fly, and
    loitering for the times asked.  start is where the aircraft is when the
    mission starts, where that is not home'''
    mavlink = mavutil.mavlink
    speed = max(parameter(params, 'AIRSPEED_CRUISE'),
                parameter(params, 'AIRSPEED_MAX'), 1.0)
    walk = MissionFlight((home[0], home[1]), home, list(items), {})
    seconds = 0.0
    (lat, lon) = (start[0], start[1]) if start else (home[0], home[1])
    index = walk.next_nav_index(-1, take=True)
    while index is not None:
        (command, item_lat, item_lon, amsl, item_params) = walk.items[index]
        if command == mavlink.MAV_CMD_NAV_LOITER_TIME:
            seconds += max(item_params[0] or 0.0, 0.0)
        if not (item_lat == 0 and item_lon == 0):
            seconds += mp_util.gps_distance(lat, lon, item_lat, item_lon) / speed
            (lat, lon) = (item_lat, item_lon)
        if command in (mavlink.MAV_CMD_NAV_LAND, mavlink.MAV_CMD_NAV_VTOL_LAND,
                       mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                       mavlink.MAV_CMD_NAV_LOITER_UNLIM):
            break
        index = walk.next_nav_index(index, take=True)
    return seconds


def first_navigation_item(items):
    '''the number of the first item in items -- numbered from 1, as
    mission_track() takes them -- which ArduPlane navigates to when the
    mission starts, or None'''
    flight = MissionFlight((0.0, 0.0), (0.0, 0.0, 0.0), list(items), {})
    index = flight.next_nav_index(-1)
    return None if index is None else index + 1


def mission_track(home, items, params, heading=None, start=None, rally=None):
    '''the path an ArduPlane flies a mission along, as (lat, lon, amsl)
    points, or None where it cannot be worked out.

    home is (lat, lon, amsl).  items are the mission items after home, in
    order and numbered from 1, as (command, lat, lon, amsl, (param1, param2,
    param3, param4)): every item, since the ones with no position of their
    own change how the others are flown.  amsl is None where it is not
    known.  params are the vehicle's parameters; ArduPlane's defaults stand
    in for any not there.  heading is the course in degrees a fixed-wing
    takeoff holds, where there is something to go on; ArduPlane holds the
    ground course it has once it gets moving.  start is (lat, lon, amsl)
    where the aircraft is when the mission starts, where that is not home.
    rally is the vehicle's rally points, as (lat, lon, amsl), where it has
    any, which a return to launch may go to instead of home.

    The aircraft sets off from home, or start, towards the first item.
    Each DO_JUMP is followed the once, so a loop is drawn once however many
    times it is flown, and one repeated for ever ends the flight the second
    time it is reached.  A navigation command this does not know how to fly
    -- NAV_DELAY, CONTINUE_AND_CHANGE_ALT and the like -- ends the attempt,
    as does an item the aircraft never finishes, and a flight longer than
    MAX_FLIGHT_TIME
    '''
    if home is None or (home[0] == 0 and home[1] == 0) or home[2] is None:
        return None
    try:
        if least_flight_time(home, items, params, start) > MAX_FLIGHT_TIME:
            return None
        flight = MissionFlight((home[0], home[1]), home, list(items), params,
                               heading, start, rally)
        return flight.run()
    except (ValueError, ZeroDivisionError, OverflowError):
        return None
