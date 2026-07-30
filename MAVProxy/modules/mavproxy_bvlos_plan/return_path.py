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

Two things bound how much this can claim. Along one return, how far it gets
from the mission path varies no faster than distance along it, so sampling
every so far can miss at most half of that, and anywhere near the limit is
measured again closely until what is left is small. Between one starting point
along the mission and the next there is no such bound: a step of a few metres
can put the aircraft nearer a different leg and send it somewhere else
entirely. A finer granularity is the only answer to that, so a result is only
as good as the granularity it was run at.
'''

# AP_FLAKE8_CLEAN

import math

from pymavlink import mavutil

from MAVProxy.modules.mavproxy_bvlos_plan import mission_model

mavlink = mavutil.mavlink

# AP_Mission::jump_to_closest_mission_leg() budget, shared across all of the
# DO_RETURN_PATH_START candidates
SEARCH_BUDGET = 1000

# how much finer a stretch that could cross the limit is re-measured at
REFINE_RATIO = 0.02

# how hard we look for the point between two samples where an RTL starts
# heading somewhere else
TRANSITION_STEPS = 6

# a sweep this close to a whole circle is really no turn at all
ANGLE_EPS = 1e-9

# turns whose paths are within this of each other in length are both treated
# as possible. Which way an aircraft actually goes near a reversal is down to
# its controller, not to which Dubins path is a hair shorter, so anything this
# close has to be allowed for
TURN_TIE = 0.05

# how many DO_JUMP counter states we are prepared to check before giving up
# and calling the result inconclusive
MAX_JUMP_STATES = 8

# re-exported, as the walk itself lives with the mission geometry
MAX_JUMP_LOOPS = mission_model.MAX_JUMP_LOOPS
JUMP_REPEAT_FOREVER = mission_model.JUMP_REPEAT_FOREVER
next_command = mission_model.next_command
jump_tag_index = mission_model.jump_tag_index


class ReturnPath(object):
    '''the located points reached from one DO_RETURN_PATH_START, in order'''

    def __init__(self, start_seq, points):
        self.start_seq = start_seq
        self.points = points


def finite_jumps(mission):
    '''sequence numbers of DO_JUMP items with a repeat count.

       Their counters carry whatever the running mission has already used up:
       distance_to_mission_leg() backs up and restores _jump_tracking rather
       than clearing it, so the return path depends on how far through the
       mission the RTL happens.
    '''
    jumps = []
    tag_jump = getattr(mavlink, 'MAV_CMD_DO_JUMP_TAG', None)
    for point in mission.points:
        # a DO_JUMP_TAG becomes an ordinary jump before ArduPilot runs it, and
        # shares the same counters, so it has the same states
        if point.command != mavlink.MAV_CMD_DO_JUMP and \
                (tag_jump is None or point.command != tag_jump):
            continue
        num_times = int(point.param2)
        if num_times != JUMP_REPEAT_FOREVER and num_times > 0:
            jumps.append((point.seq, num_times))
    return jumps


def jump_states(mission):
    '''the DO_JUMP counter states an RTL could find, or None if there are too
       many to check.

       A jump that has been used up sends the walk past it, one that has not
       sends it round the loop, so the two expose different return paths.
    '''
    jumps = finite_jumps(mission)
    if len(jumps) == 0:
        return [{}]
    if 2 ** len(jumps) > MAX_JUMP_STATES:
        return None
    states = [{}]
    for (seq, num_times) in jumps:
        grown = []
        for state in states:
            fresh = dict(state)
            used = dict(state)
            used[seq] = num_times
            grown.append(fresh)
            grown.append(used)
        states = grown
    return states


def build_return_paths(mission, jump_counts=None, dont_zero_counter=False):
    '''the return path from each DO_RETURN_PATH_START, walked the way
       AP_Mission::distance_to_mission_leg() walks it: following DO_JUMP, and
       stopping at a landing or a DO_LAND_START inclusive'''
    paths = []
    budget = SEARCH_BUDGET
    exhausted = False
    for start in mission.return_path_starts():
        points = []
        # ArduPilot backs the counters up and restores them around each
        # candidate, so every candidate starts from the same live state
        counts = dict(jump_counts or {})
        index = start
        finished = False
        while budget > 0:
            (point, index) = next_command(mission, index, counts,
                                          dont_zero_counter)
            if point is None:
                # ran off the end of the mission, which ArduPilot still
                # accepts as a path
                finished = True
                break
            index = point.seq + 1
            if point.has_location():
                points.append(point)
            if point.is_landing() or point.command == mavlink.MAV_CMD_DO_LAND_START:
                # ArduPilot leaves the loop here without spending the
                # iteration, so a landing costs nothing from the budget
                finished = True
                break
            budget -= 1
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


def capture_deviation(index, sample, target, granularity, radius,
                      fine=None, limit=None):
    '''how far the aircraft actually gets from the mission path flying an RTL
       from this point, in metres.

       An RTL does not teleport onto the line to the rejoin waypoint. The
       aircraft is flying along the mission, and has to bank round onto that
       line at the radius its airspeed and bank limit allow before it can
       track it. A reversal is the case that matters: turning back the way it
       came takes it a full diameter off the mission track, not none of it, so
       measuring the straight line alone would call a manoeuvre safe that puts
       the aircraft twice as far out as the corridor allows.

       The path measured is therefore the arc from where the aircraft is,
       tangent to its present heading, followed by the straight run to the
       rejoin waypoint.
    '''
    worst = index.distance(sample.x, sample.y)
    (ux, uy) = (sample.ux, sample.uy)
    if radius <= 0 or (ux == 0.0 and uy == 0.0):
        # no heading to turn from, so the straight line is all there is
        return max(worst, straight_deviation(index, sample.x, sample.y,
                                             target.x, target.y, granularity,
                                             fine, limit))

    # the aircraft turns whichever way gets it there sooner, which is the
    # shorter path and not the smaller turn: a wider turn can roll out onto a
    # much shorter run in
    turns = [turn_onto(sample.x, sample.y, ux, uy, radius, direction,
                       target.x, target.y)
             for direction in (1.0, -1.0)]
    shortest = min(turn[7] for turn in turns)
    allowed = shortest * (1.0 + TURN_TIE) + radius * TURN_TIE
    for turn in turns:
        # a reversal makes the two sides the same length, and they can cover
        # very different ground, so anything as short as the best is flown as
        # far as we know and has to be allowed for
        if turn[7] > allowed:
            continue
        got = turn_deviation(index, turn, radius, target, granularity,
                             fine, limit)
        if got > worst:
            worst = got
    return worst


def turn_deviation(index, turn, radius, target, granularity,
                   fine=None, limit=None):
    '''how far one candidate turn and the run in after it get from the path'''
    (sweep, cx, cy, start, qx, qy, direction, _) = turn
    arc = radius * sweep
    steps = max(1, int(math.ceil(arc / granularity)))

    def at(frac):
        angle = start + direction * sweep * frac
        return index.distance(cx + radius * math.cos(angle),
                              cy + radius * math.sin(angle))

    worst = sampled_max(at, steps, arc / steps if steps else 0.0, fine, limit)
    return max(worst, straight_deviation(index, qx, qy, target.x, target.y,
                                         granularity, fine, limit))


def turn_onto(px, py, ux, uy, radius, direction, tx, ty):
    '''the bank limited turn from (px, py) heading (ux, uy) onto the line to
       (tx, ty), going round the circle in the given direction.

       Returns how far round it has to go, where the circle is, where it
       starts on it and where it rolls out.
    '''
    if direction > 0:
        (nx, ny) = (-uy, ux)
    else:
        (nx, ny) = (uy, -ux)
    (cx, cy) = (px + radius * nx, py + radius * ny)
    start = math.atan2(py - cy, px - cx)
    span = math.hypot(tx - cx, ty - cy)
    if span <= radius:
        # the waypoint is inside the turning circle, so the aircraft cannot
        # roll out onto it at all. What it really does is widen out and spiral
        # in, which is not worth modelling: taking the whole circle and then
        # the run at the target covers the ground either way, and covering the
        # ground is the question
        length = 2.0 * math.pi * radius + math.hypot(tx - px, ty - py)
        return (2.0 * math.pi, cx, cy, start, px, py, direction, length)
    to_target = math.atan2(ty - cy, tx - cx)
    offset = math.acos(max(-1.0, min(1.0, radius / span)))
    # of the two tangent points, the one where continuing round this way
    # heads at the target rather than away from it
    angle = to_target - direction * offset
    sweep = (direction * (angle - start)) % (2.0 * math.pi)
    if sweep > 2.0 * math.pi - ANGLE_EPS:
        # already pointing at it: the subtraction can land a hair below zero
        # and wrap to a whole circle, which would invent a 2R excursion out of
        # a manoeuvre that needs no turn at all
        sweep = 0.0
    run = math.sqrt(max(0.0, span * span - radius * radius))
    return (sweep, cx, cy, start,
            cx + radius * math.cos(angle), cy + radius * math.sin(angle),
            direction, radius * sweep + run)


def nearest_on_circle(cx, cy, radius, px, py):
    '''the point of a circle closest to a point'''
    (dx, dy) = (px - cx, py - cy)
    length = math.hypot(dx, dy)
    if length <= 0:
        return (cx + radius, cy)
    return (cx + radius * dx / length, cy + radius * dy / length)


def straight_deviation(index, ax, ay, bx, by, granularity,
                       fine=None, limit=None):
    '''how far a straight run between two points gets from the mission path'''
    (dx, dy) = (bx - ax, by - ay)
    length = math.hypot(dx, dy)
    if length <= 0:
        return index.distance(ax, ay)
    steps = max(1, int(math.ceil(length / granularity)))

    def at(frac):
        return index.distance(ax + dx * frac, ay + dy * frac)

    return sampled_max(at, steps, length / steps, fine, limit)


def sampled_max(at, steps, spacing, fine, limit):
    '''the largest value of a 1-Lipschitz function sampled along a path.

       Sampling every so far can miss up to half of that, so where a stretch
       could be hiding something over the limit it is measured again closely.
       Everywhere else the samples already prove it cannot be, so there is
       nothing to gain by looking harder.
    '''
    seen = [at(float(step) / steps) for step in range(steps + 1)]
    worst = max(seen)
    if fine is None or limit is None or worst > limit:
        # already over the limit, and looking closer can only make it worse,
        # so there is nothing a finer measurement would change
        return worst
    for step in range(steps):
        if max(seen[step], seen[step + 1]) + spacing * 0.5 <= limit:
            continue
        inner = max(1, int(math.ceil(spacing / fine)))
        for k in range(1, inner):
            got = at((step + float(k) / inner) / steps)
            if got > worst:
                worst = got
    return worst


def transition(before, before_rejoin, after, after_rejoin, assess):
    """close in on where the item an RTL heads for changes, and assess it.

       Between two points that head for different items there is a boundary,
       and either side of it is where a return is at its worst. Sampling
       cannot find it, so it is hunted down by halving the gap.
    """
    found = []
    (lo, hi) = (before, after)
    for _ in range(TRANSITION_STEPS):
        mid = midpoint(lo, hi)
        (deviation, rejoin) = assess(mid)
        if deviation is None:
            break
        found.append((mid, deviation, rejoin))
        if rejoin == before_rejoin:
            lo = mid
        else:
            hi = mid
    return found


def midpoint(a, b):
    """a sampling point half way between two, on the same leg"""
    return mission_model.PathSample(
        0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.amsl + b.amsl),
        a.leg_from, a.leg_to, 0.5 * (a.distance + b.distance), a.ux, a.uy)


def cut_across_deviation(index, sample, target, granularity):
    '''how far the straight line from a point to the rejoin waypoint gets from
       the mission path, without the turn onto it'''
    return straight_deviation(index, sample.x, sample.y,
                              target.x, target.y, granularity)


def rejoin_target(mission, point, jump_counts=None, dont_zero_counter=False):
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
    counts = dict(jump_counts or {})
    index = point.seq + 1
    for _ in range(mission.count() + 1):
        (candidate, index) = next_command(mission, index, counts,
                                          dont_zero_counter)
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
        # parts of the mission whose flown path we cannot work out, which make
        # a result inconclusive rather than a pass
        self.unmodelled = []
        # which samples failed, by index along the mission, so a proposed fix
        # can be checked for making anything worse rather than just for
        # lowering the total
        self.failed_at = set()
        # how many DO_JUMP counter states the check had to consider
        self.jump_states = 1
        self.continue_after_land = False

    def conclusive(self):
        '''False if something about the mission means a pass cannot be
           claimed, whatever the samples said'''
        return len(self.unmodelled) == 0 and self.terrain_missing == 0

    def ok(self):
        return (len(self.errors) == 0 and self.failed == 0 and
                self.conclusive())


def check_return_path(mission, cruise_eas, roll_limit_deg, granularity=50.0,
                      terrain_fn=None, width=0.0, loiter_radius=0.0,
                      dont_zero_counter=False, continue_after_land=False):
    '''check that an RTL from any point on the mission returns within the
       allowed distance of the mission path.

       That distance is the turn radius the vehicle can achieve at the
       altitude of each point, unless width is greater than zero, in which
       case it is used instead. The turn radius is still needed to work out
       the arc the aircraft flies onto the return, so the airspeed and bank
       limit are used either way when they are known.
    '''
    result = CheckResult()
    if width > 0:
        result.fixed_width = width
    result.terrain_missing = mission.terrain_missing
    result.return_path_starts = mission.return_path_starts()
    result.continue_after_land = continue_after_land

    path = mission.flown_path(dont_zero_counter=dont_zero_counter,
                              continue_after_land=continue_after_land,
                              loiter_radius=loiter_radius)
    # against the path actually being checked, not the default one. A mission
    # that carries on past its landing flies items the default walk stops
    # before, and they were going unreported
    result.unmodelled = mission.unmodelled(loiter_radius, path)
    if len(path) < 2:
        result.errors.append("mission has no flyable path")
        return result
    if len(result.return_path_starts) == 0:
        result.errors.append("mission has no DO_RETURN_PATH_START")
        return result

    states = jump_states(mission)
    if states is None:
        result.errors.append(
            "the mission has too many DO_JUMP items with a repeat count to "
            "work out every return path an RTL could take, as which one it "
            "picks depends on how many times each loop has already run")
        return result

    path_sets = []
    barren = 0
    for state in states:
        found = build_return_paths(mission, state, dont_zero_counter)
        if len(found) > 0:
            path_sets.append((state, found))
        else:
            # an RTL with the counters in this state finds no return path at
            # all. Quietly leaving the state out would check only the states
            # that happen to work and call the mission safe
            barren += 1
    if len(path_sets) == 0:
        result.errors.append(
            "no return path found after DO_RETURN_PATH_START")
        return result
    if barren > 0:
        result.errors.append(
            "%u of the %u DO_JUMP counter states an RTL could find have no "
            "return path at all, so an RTL with a loop part run would not "
            "follow one" % (barren, len(states)))
        return result
    result.jump_states = len(path_sets)

    asked = max(float(granularity), mission_model.MIN_SPACING)
    granularity = mission_model.usable_spacing(path, asked, len(path_sets))
    if granularity > asked:
        result.warnings.append(
            "sampled every %.0fm rather than the %.0fm asked for, which would "
            "have taken more than %u samples on a mission this long"
            % (granularity, asked, mission_model.MAX_SAMPLES))
    # what a near miss is re-measured at, whose own error is small enough to
    # carry as an allowance rather than hide
    fine = max(granularity * REFINE_RATIO, mission_model.MIN_SPACING)
    index = mission.corridor_index(loiter_radius, path)
    stats = mission_model.SampleStats()
    samples = mission_model.sample_path(path, granularity,
                                        projector=mission.projector,
                                        terrain_fn=terrain_fn, stats=stats)
    result.terrain_missing += stats.terrain_missing

    unresolved = 0
    span = None
    previous = None

    def assess(sample):
        '''the worst an RTL from here does, over every counter state, and
           which item it would head for'''
        worst = None
        rejoin = None
        for (state, paths) in path_sets:
            (closest, _) = closest_leg(paths, sample.x, sample.y, sample.amsl)
            target = rejoin_target(mission, closest, state, dont_zero_counter)
            if target is None:
                # ArduPilot's set_current_cmd() would fail here, so an RTL
                # from this point does not get a return path at all
                return (None, None)
            radius = radius_at(sample)
            got = capture_deviation(index, sample, target, granularity,
                                    turn_at(sample), fine, radius)
            got += fine * 0.5
            if worst is None or got > worst:
                worst = got
                rejoin = target.seq
        return (worst, rejoin)

    def radius_at(sample):
        if width > 0:
            return width
        return 2.0 * turn_at(sample)

    def turn_at(sample):
        if not cruise_eas:
            return 0.0
        return mission_model.turn_radius(cruise_eas, roll_limit_deg,
                                         sample.amsl)

    def record(sample, deviation, rejoin):
        '''fold one assessed point into the result'''
        radius = radius_at(sample)
        result.checked += 1
        if deviation > result.worst_deviation:
            result.worst_deviation = deviation
            result.worst_radius = radius
            result.worst_leg = (sample.leg_from, sample.leg_to)
            result.worst_rejoin = rejoin
        return deviation > radius

    def note(sample, deviation, rejoin, position):
        """fold one assessed point into the result, and say if it failed"""
        nonlocal span
        radius = radius_at(sample)
        result.checked += 1
        if deviation > result.worst_deviation:
            result.worst_deviation = deviation
            result.worst_radius = radius
            result.worst_leg = (sample.leg_from, sample.leg_to)
            result.worst_rejoin = rejoin
        if deviation <= radius:
            return False
        result.failed += 1
        # keyed by the sampling point it belongs to, so two runs over the same
        # mission can be compared even though where a return changes moves
        result.failed_at.add(position)
        if span is None:
            span = FailSpan()
            result.spans.append(span)
        span.add(sample, deviation)
        return True

    for (position, sample) in enumerate(samples):
        (deviation, rejoin) = assess(sample)
        if deviation is None:
            unresolved += 1
            previous = None
            span = None
            continue

        failed = note(sample, deviation, rejoin, position)
        # where the item an RTL heads for changes from one point to the next,
        # the worst case is in between: a step of a few metres can put the
        # aircraft nearer a different leg and send it somewhere else entirely,
        # and sampling alone steps straight over it
        if previous is not None and previous[1] != rejoin and \
                previous[0].leg_to == sample.leg_to:
            for (edge, edge_dev, edge_rejoin) in transition(
                    previous[0], previous[1], sample, rejoin, assess):
                if note(edge, edge_dev, edge_rejoin, position):
                    failed = True
        previous = (sample, rejoin)
        if not failed:
            span = None

    if unresolved > 0:
        result.errors.append(
            "%u of %u points along the mission have a return path that does "
            "not reach a navigation command, so an RTL there would not follow "
            "it" % (unresolved, unresolved + result.checked))
    if result.checked == 0 and len(result.errors) == 0:
        result.errors.append("no point along the mission could be checked")

    return result
