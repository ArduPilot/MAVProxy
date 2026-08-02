#!/usr/bin/env python3
'''
DO_RETURN_PATH_START safety check for BVLOS missions.

With RTL_AUTOLAND=4 an ArduPlane RTL calls
AP_Mission::jump_to_closest_mission_leg(), which finds the closest leg of the
mission after a DO_RETURN_PATH_START and makes that leg's end waypoint the
current command. The aircraft then flies direct to that waypoint, with no
crosstrack on that first leg, so the return can cut across ground the mission
never covers. ArduPilot puts no distance limit on this and does not check it
against a fence.

This checks that for every point along the mission, that cut-across stays
within one turn radius of the mission path, so an RTL from anywhere keeps the
aircraft inside the corridor the mission already surveys. The turn radius is
what the vehicle can achieve at its cruise airspeed and bank limit, at the
altitude of that point of the mission.

The model of the ArduPilot behaviour follows
libraries/AP_Mission/AP_Mission.cpp jump_to_closest_mission_leg() and
distance_to_mission_leg().
'''

# AP_FLAKE8_CLEAN

import math

from pymavlink import mavutil

from MAVProxy.modules.mavproxy_bvlos_plan import mission_model

mavlink = mavutil.mavlink

# AP_Mission::jump_to_closest_mission_leg() budget, shared across all of the
# DO_RETURN_PATH_START candidates
SEARCH_BUDGET = 1000

# max_loops in AP_Mission::get_next_cmd()
MAX_JUMP_LOOPS = 64

# AP_MISSION_JUMP_REPEAT_FOREVER
JUMP_REPEAT_FOREVER = -1


class ReturnPath(object):
    '''the located points reached from one DO_RETURN_PATH_START, in order'''

    def __init__(self, start_seq, points):
        self.start_seq = start_seq
        self.points = points


def next_command(mission, index, jump_counts):
    '''AP_Mission::get_next_cmd(): the next non jump command at or after
       index, following DO_JUMP. Returns None at the end of the mission or on
       a bad jump'''
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
        if num_times == JUMP_REPEAT_FOREVER or run < num_times:
            jump_counts[index] = run + 1
            index = target
        else:
            # having finished a jump loop ArduPilot zeroes the counter, so
            # coming back to it later runs the loop again
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


def build_return_paths(mission):
    '''the return path from each DO_RETURN_PATH_START, walked the way
       AP_Mission::distance_to_mission_leg() walks it: following DO_JUMP, and
       stopping at a landing or a DO_LAND_START inclusive'''
    paths = []
    budget = SEARCH_BUDGET
    exhausted = False
    for start in mission.return_path_starts():
        points = []
        jump_counts = {}
        index = start
        finished = False
        while budget > 0:
            budget -= 1
            (point, index) = next_command(mission, index, jump_counts)
            if point is None:
                # ran off the end of the mission, which ArduPilot still
                # accepts as a path
                finished = True
                break
            index = point.seq + 1
            if point.has_location():
                points.append(point)
            if point.is_landing() or point.command == mavlink.MAV_CMD_DO_LAND_START:
                finished = True
                break
        if not finished:
            # the search budget ran out part way through, which ArduPilot
            # treats as no path rather than as a truncated one
            exhausted = True
            break
        if len(points) > 0:
            paths.append(ReturnPath(start, points))
    if exhausted and len(paths) == 0:
        return []
    return paths


def closest_leg(paths, x, y, amsl):
    '''AP_Mission::jump_to_closest_mission_leg(): the item the vehicle would
       make current, or None.

       Within one path the first located point is measured as a point and
       ties keep the earliest leg; across paths a tie keeps the last
       DO_RETURN_PATH_START, matching the <= in ArduPilot.
    '''
    best_point = None
    best_distance = None
    for path in paths:
        (point, distance) = closest_in_path(path, x, y, amsl)
        if point is None:
            continue
        if best_distance is None or distance <= best_distance:
            best_distance = distance
            best_point = point
    return (best_point, best_distance)


def closest_in_path(path, x, y, amsl):
    '''closest point of one return path, as distance_to_mission_leg() does'''
    points = path.points
    if len(points) == 0:
        return (None, None)
    first = points[0]
    # the first point of a return path is measured as a point, not a leg
    dx = x - first.x
    dy = y - first.y
    dz = amsl - first.amsl
    best_distance = (dx * dx + dy * dy + dz * dz) ** 0.5
    best_point = first
    prev = first
    for point in points[1:]:
        if point.x == prev.x and point.y == prev.y and point.amsl == prev.amsl:
            # a zero length leg is skipped and does not advance prev_loc
            continue
        distance = mission_model.segment_distance_3d(
            x, y, amsl,
            prev.x, prev.y, prev.amsl,
            point.x, point.y, point.amsl)
        # strict, so a tie keeps the earlier leg
        if distance < best_distance:
            best_distance = distance
            best_point = point
        prev = point
    return (best_point, best_distance)


def rejoin_target(mission, point):
    '''where the vehicle actually flies to.

       set_current_cmd() runs advance_current_nav_cmd(), which walks forward
       executing do-commands until it reaches a nav command, so if the closest
       item is not a nav command the aircraft heads for the next nav command
       after it. A DO_RETURN_PATH_START carrying a location can be the closest
       item, which is why this matters.
    '''
    if point is None:
        return None
    if point.is_nav() and point.has_location():
        return point
    jump_counts = {}
    index = point.seq + 1
    for _ in range(mission.count() + 1):
        (candidate, index) = next_command(mission, index, jump_counts)
        if candidate is None:
            return None
        index = candidate.seq + 1
        if candidate.is_nav() and candidate.has_location():
            return candidate
    return None


class FailSpan(object):
    '''a run of consecutive failing samples along the mission'''

    def __init__(self):
        self.points = []
        self.legs = set()
        self.worst = 0.0

    def add(self, sample, deviation):
        self.points.append((sample.x, sample.y))
        self.legs.add((sample.leg_from, sample.leg_to))
        self.worst = max(self.worst, deviation)


class CheckResult(object):
    '''outcome of a return path check'''

    def __init__(self):
        self.checked = 0
        self.failed = 0
        self.worst_deviation = 0.0
        self.worst_radius = None
        self.worst_leg = None
        self.worst_rejoin = None
        self.spans = []
        self.errors = []
        self.warnings = []
        self.return_path_starts = []
        self.terrain_missing = 0
        # set when a fixed width was used instead of the turn radius
        self.fixed_width = None

    def ok(self):
        return len(self.errors) == 0 and self.failed == 0


def check_return_path(mission, cruise_eas, roll_limit_deg, granularity=50.0,
                      terrain_fn=None, width=0.0):
    '''check that an RTL from any point on the mission returns within the
       allowed distance of the mission path.

       That distance is the turn radius the vehicle can achieve at the
       altitude of each point, unless width is greater than zero, in which
       case it is used instead and the airspeed and bank limit are not
       needed.
    '''
    result = CheckResult()
    if width > 0:
        result.fixed_width = width
    result.terrain_missing = mission.terrain_missing
    result.return_path_starts = mission.return_path_starts()

    path = mission.flown_path()
    if len(path) < 2:
        result.errors.append("mission has no flyable path")
        return result
    if len(result.return_path_starts) == 0:
        result.errors.append("mission has no DO_RETURN_PATH_START")
        return result

    paths = build_return_paths(mission)
    if len(paths) == 0:
        result.errors.append(
            "no return path found after DO_RETURN_PATH_START")
        return result

    granularity = max(float(granularity), mission_model.MIN_SPACING)
    index = mission_model.PathIndex(path)
    stats = mission_model.SampleStats()
    samples = mission_model.sample_path(path, granularity,
                                        projector=mission.projector,
                                        terrain_fn=terrain_fn, stats=stats)
    result.terrain_missing += stats.terrain_missing

    unresolved = 0
    span = None
    for sample in samples:
        if width > 0:
            radius = width
        else:
            radius = mission_model.turn_radius(cruise_eas, roll_limit_deg,
                                               sample.amsl)
        (closest, _) = closest_leg(paths, sample.x, sample.y, sample.amsl)
        target = rejoin_target(mission, closest)
        if target is None:
            # ArduPilot's set_current_cmd() would fail here, so an RTL from
            # this point does not get a return path at all
            unresolved += 1
            continue
        deviation = cut_across_deviation(index, sample, target, granularity)
        result.checked += 1
        if deviation > result.worst_deviation:
            result.worst_deviation = deviation
            result.worst_radius = radius
            result.worst_leg = (sample.leg_from, sample.leg_to)
            result.worst_rejoin = target.seq
        if deviation > radius:
            result.failed += 1
            if span is None:
                span = FailSpan()
                result.spans.append(span)
            span.add(sample, deviation)
        else:
            span = None

    if unresolved > 0:
        result.errors.append(
            "%u of %u points along the mission have a return path that does "
            "not reach a navigation command, so an RTL there would not follow "
            "it" % (unresolved, unresolved + result.checked))
    if result.checked == 0 and len(result.errors) == 0:
        result.errors.append("no point along the mission could be checked")

    return result


def cut_across_deviation(index, sample, target, granularity):
    '''how far the direct line from a point to the rejoin waypoint gets from
       the mission path, in metres'''
    dx = target.x - sample.x
    dy = target.y - sample.y
    length = (dx * dx + dy * dy) ** 0.5
    if length <= 0:
        return index.distance(sample.x, sample.y)
    steps = max(1, int(math.ceil(length / granularity)))
    worst = 0.0
    for step in range(steps + 1):
        frac = float(step) / steps
        d = index.distance(sample.x + dx * frac, sample.y + dy * frac)
        if d > worst:
            worst = d
    return worst
