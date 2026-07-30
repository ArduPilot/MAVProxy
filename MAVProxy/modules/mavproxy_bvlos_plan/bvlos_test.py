'''
Standalone test harness for the BVLOS return path check.

Builds its missions in memory, so it needs no vehicle, no terrain and no
mission files, and checks the geometry and the model of ArduPilot's behaviour
that the check rests on.

  PYTHONPATH=$PWD python3 -m MAVProxy.modules.mavproxy_bvlos_plan.bvlos_test
'''

# AP_FLAKE8_CLEAN

import math
import random
import sys

from pymavlink import mavutil

from MAVProxy.modules.mavproxy_bvlos_plan import add_return_paths
from MAVProxy.modules.mavproxy_bvlos_plan import mission_model
from MAVProxy.modules.mavproxy_bvlos_plan import return_path

mavlink = mavutil.mavlink

FAILURES = []


def check(name, condition, detail=''):
    if condition:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        FAILURES.append(name)


def item(seq, command, lat=0.0, lon=0.0, alt=100.0,
         param1=0.0, param2=0.0, param3=0.0,
         frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT):
    return mavlink.MAVLink_mission_item_message(
        1, 1, seq, frame, command, 0, 1,
        param1, param2, param3, 0.0, lat, lon, alt)


def home(lat=-35.0, lon=149.0, alt=600.0):
    return item(0, mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt,
                frame=mavlink.MAV_FRAME_GLOBAL)


def waypoints(pairs, start=1, command=mavlink.MAV_CMD_NAV_WAYPOINT):
    '''waypoints from (lat, lon) pairs'''
    return [item(start + i, command, lat, lon)
            for (i, (lat, lon)) in enumerate(pairs)]


def build(items):
    return mission_model.build_mission(items)


class Sample(object):
    def __init__(self, x, y, ux, uy):
        (self.x, self.y, self.ux, self.uy) = (x, y, ux, uy)


class Point(object):
    def __init__(self, x, y):
        (self.x, self.y) = (x, y)


def test_units():
    print("units and geometry")
    # a coordinated turn at 20m/s and 40 degrees of bank
    radius = mission_model.turn_radius(20.0, 40.0, 0.0)
    check("turn radius at sea level is 49m", abs(radius - 48.6) < 1.0,
          "%.1f" % radius)
    # thinner air up high means a higher true airspeed and a wider turn
    high = mission_model.turn_radius(20.0, 40.0, 1767.0)
    check("turn radius grows with altitude", high > radius * 1.15,
          "%.1f vs %.1f" % (high, radius))
    check("eas2tas is 1 at sea level",
          abs(mission_model.eas2tas(0.0) - 1.0) < 1e-9)

    # the projection has to survive a mission either side of the antimeridian,
    # which used to be measured the long way round the planet
    across = build([home(0.0, 179.9),
                    item(1, mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 179.9),
                    item(2, mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, -179.9)])
    span = math.hypot(across.point(2).x - across.point(1).x,
                      across.point(2).y - across.point(1).y)
    check("the antimeridian is crossed the short way", span < 30000.0,
          "%.0fm" % span)

    projector = mission_model.Projector(-35.0, 149.0)
    (lat, lon) = projector.unproject(*projector.project(-35.5, 149.5))
    check("projection round trips", abs(lat + 35.5) < 1e-6 and
          abs(lon - 149.5) < 1e-6)


def test_path_index():
    print("path index")
    random.seed(7)
    worst = 0.0
    for _ in range(200):
        count = random.randint(2, 40)
        scale = random.choice([10.0, 1000.0, 100000.0])
        points = [Point(random.uniform(-scale, scale),
                        random.uniform(-scale, scale))
                  for _ in range(count)]
        index = mission_model.PathIndex(points)
        if len(index.segments) == 0:
            continue
        for _ in range(20):
            # including far outside the grid, where the ring search has to
            # walk past every marked cell before it can stop
            far = random.choice([1.0, 2.0, 10.0, 50.0])
            (px, py) = (random.uniform(-far * scale, far * scale),
                        random.uniform(-far * scale, far * scale))
            got = index.distance(px, py)
            want = math.sqrt(min(index._segment_d2(i, px, py)
                                 for i in range(len(index.segments))))
            worst = max(worst, abs(got - want))
    check("the grid agrees with looking at every segment", worst == 0.0,
          "%.3e" % worst)


def test_capture_turn():
    print("the turn onto the return")
    radius = 200.0
    # the mission path here is the y axis
    index = mission_model.PathIndex([Point(0.0, -5000.0), Point(0.0, 5000.0)])

    ahead = return_path.capture_deviation(
        index, Sample(0.0, 0.0, 0.0, 1.0), Point(0.0, 3000.0), 2.0, radius)
    check("no turn is needed for a waypoint dead ahead", ahead < 1.0,
          "%.1f" % ahead)

    # the case the whole thing exists for: turning back the way it came takes
    # the aircraft a full diameter off its track, not none of it
    back = return_path.capture_deviation(
        index, Sample(0.0, 0.0, 0.0, 1.0), Point(0.0, -3000.0), 2.0, radius)
    check("a reversal costs a full diameter", abs(back - 2 * radius) < 5.0,
          "%.1f" % back)
    chord = return_path.cut_across_deviation(
        index, Sample(0.0, 0.0, 0.0, 1.0), Point(0.0, -3000.0), 2.0)
    check("the straight line alone would have called that free", chord < 1.0,
          "%.1f" % chord)

    inside = return_path.capture_deviation(
        index, Sample(0.0, 0.0, 0.0, 1.0), Point(150.0, 50.0), 2.0, radius)
    check("a waypoint inside the turning circle costs a whole circle",
          abs(inside - 2 * radius) < 5.0, "%.1f" % inside)

    # the arc and the run in have to join, or the aircraft is being credited
    # with a jump it cannot make
    for (name, target) in (("outside", Point(3000.0, -1000.0)),
                           ("inside", Point(150.0, 50.0))):
        for direction in (1.0, -1.0):
            turn = return_path.turn_onto(0.0, 0.0, 0.0, 1.0, radius,
                                         direction, target.x, target.y)
            (sweep, cx, cy, start, qx, qy, _, length) = turn
            end = (cx + radius * math.cos(start + direction * sweep),
                   cy + radius * math.sin(start + direction * sweep))
            gap = math.hypot(end[0] - qx, end[1] - qy)
            check("the turn to a target %s joins its run in (%+.0f)"
                  % (name, direction), gap < 1e-6, "%.3e" % gap)
            check("its length is the arc plus the run in (%s %+.0f)"
                  % (name, direction),
                  abs(length - (radius * sweep +
                                math.hypot(target.x - qx,
                                           target.y - qy))) < 1e-6)

    # whatever the geometry, flying the turn cannot be closer to the path than
    # pretending the aircraft snaps onto the straight line
    random.seed(3)
    violations = 0
    for _ in range(2000):
        angle = random.uniform(0, 2 * math.pi)
        sample = Sample(0.0, 0.0, math.cos(angle), math.sin(angle))
        target = Point(random.uniform(-4000, 4000),
                       random.uniform(-4000, 4000))
        flown = return_path.capture_deviation(index, sample, target, 5.0,
                                              radius)
        straight = return_path.cut_across_deviation(index, sample, target, 5.0)
        if flown < straight - 1e-6:
            violations += 1
    check("the flown path is never nearer than the straight line",
          violations == 0, "%u" % violations)


def test_sampling_bound():
    print("sampling")
    # the deviation along a cut is 1-Lipschitz, so sampling every g metres
    # cannot miss more than half of g. Without that there is no way to say
    # what a reported margin means
    random.seed(9)
    worst = 0.0
    for _ in range(1500):
        points = [Point(random.uniform(-500, 500), random.uniform(-500, 500))
                  for _ in range(random.randint(2, 5))]
        index = mission_model.PathIndex(points)
        sample = Sample(random.uniform(-500, 500), random.uniform(-500, 500),
                        0.0, 0.0)
        target = Point(random.uniform(-500, 500), random.uniform(-500, 500))
        granularity = random.choice([10.0, 25.0, 50.0, 100.0])
        coarse = return_path.cut_across_deviation(index, sample, target,
                                                  granularity)
        fine = return_path.cut_across_deviation(index, sample, target, 0.02)
        worst = max(worst, fine - coarse - granularity / 2.0)
    check("sampling never misses more than half the granularity",
          worst <= 1e-6, "%.4f over" % worst)

    # a granularity of zero must not divide by zero, and must not turn into a
    # request for an unbounded amount of work either
    result = return_path.check_return_path(tiny_mission(), 20.0, 30.0,
                                           granularity=0.0)
    check("a granularity of zero is survivable", result.checked > 0)

    long_way = simple_mission()
    spacing = mission_model.usable_spacing(long_way.flown_path(), 0.001)
    samples = mission_model.path_length(long_way.flown_path()) / spacing
    check("an absurd granularity is held to a workable number of samples",
          samples <= mission_model.MAX_SAMPLES + 1, "%.0f" % samples)

    # the verdict has to use the bound, not just know about it. A path that
    # bends sharply between two samples is where a coarse pass reads low
    # the path meets the cut at each sample and swings away between them, so
    # the samples all read zero while the truth in between is not
    index = mission_model.PathIndex([Point(0.0, -35.0), Point(200.0, -17.5),
                                     Point(0.0, 0.0), Point(200.0, 17.5),
                                     Point(0.0, 35.0)])
    sample = Sample(0.0, -35.0, 0.0, 1.0)
    target = Point(0.0, 35.0)
    dense = return_path.capture_deviation(index, sample, target, 0.05, 0.0)
    coarse = return_path.capture_deviation(index, sample, target, 50.0, 0.0)
    check("a coarse pass really can read low on a path that bends",
          dense > coarse + 1.0, "%.2f vs %.2f" % (dense, coarse))
    refined = return_path.capture_deviation(index, sample, target, 50.0, 0.0,
                                            fine=1.0, limit=dense - 1.0)
    check("but a limit inside that gap makes it look closer and find it",
          refined >= dense - 1.0, "%.2f vs %.2f" % (refined, dense))
    # and it must not pay for that everywhere: a limit far above is left alone
    cheap = return_path.capture_deviation(index, sample, target, 50.0, 0.0,
                                          fine=1.0, limit=dense + 1000.0)
    check("a limit nowhere near is not paid for", cheap == coarse,
          "%.2f vs %.2f" % (cheap, coarse))


def simple_mission(extra=None):
    '''out along a line, back down it, and land'''
    items = [home()]
    items += waypoints([(-35.00, 149.00), (-35.02, 149.00), (-35.04, 149.00)])
    items.append(item(4, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      -35.04, 149.00))
    items += waypoints([(-35.02, 149.00), (-35.00, 149.00)], start=5)
    items.append(item(7, mavlink.MAV_CMD_NAV_LAND, -35.00, 149.00, 0.0))
    if extra:
        items += extra
    return build(items)


def tiny_mission():
    '''the same shape as simple_mission but a few hundred metres across, for
       tests that sample it very finely'''
    items = [home()]
    items += waypoints([(-35.000, 149.0), (-35.001, 149.0), (-35.002, 149.0)])
    items.append(item(4, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      -35.002, 149.0))
    items += waypoints([(-35.001, 149.0), (-35.000, 149.0)], start=5)
    items.append(item(7, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    return build(items)


def detour_items():
    """out a long way and back by a different route, so an RTL from the far
       end cuts across country the mission never covers"""
    items = [home(-35.0, 149.0)]
    items += waypoints([(-35.0, 149.0), (-35.10, 149.0), (-35.20, 149.0)])
    items.append(item(4, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      -35.20, 149.20))
    items += waypoints([(-35.10, 149.20), (-35.0, 149.0)], start=5)
    items.append(item(7, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    return items


def detour_mission():
    return build(detour_items())


def test_arduplane_walk():
    print("the model of what ArduPilot does")
    mission = simple_mission()
    paths = return_path.build_return_paths(mission)
    check("a return path is found", len(paths) == 1)
    check("the walk stops at the landing",
          paths[0].points[-1].command == mavlink.MAV_CMD_NAV_LAND,
          str(paths[0].points[-1]))

    # a DO_JUMP is followed while walking a return path
    jumped = build([home()] +
                   waypoints([(-35.00, 149.0), (-35.02, 149.0)]) +
                   [item(3, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                         -35.02, 149.0),
                    item(4, mavlink.MAV_CMD_DO_JUMP, param1=6, param2=-1),
                    item(5, mavlink.MAV_CMD_NAV_WAYPOINT, -35.9, 149.0),
                    item(6, mavlink.MAV_CMD_NAV_WAYPOINT, -35.01, 149.0),
                    item(7, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)])
    reached = [p.seq for p in return_path.build_return_paths(jumped)[0].points]
    check("a DO_JUMP is followed, skipping what it jumps over",
          5 not in reached and 6 in reached, str(reached))

    # a command that is not a nav command cannot be flown to, so the target
    # resolves forward the way advance_current_nav_cmd() would
    start = mission.point(4)
    target = return_path.rejoin_target(mission, start)
    check("a DO_RETURN_PATH_START resolves to the next nav command",
          target is not None and target.seq == 5,
          str(target))

    # a return path that reaches no nav command at all is an error, not a pass
    dead = build([home()] +
                 waypoints([(-35.0, 149.0), (-35.02, 149.0)]) +
                 [item(3, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                       -35.02, 149.0)])
    result = return_path.check_return_path(dead, 20.0, 30.0)
    check("a return path reaching no nav command is an error",
          not result.ok() and len(result.errors) > 0,
          str(result.errors))

    # the budget is shared and a candidate that exhausts it is not used
    check("the search budget is a thousand commands",
          return_path.SEARCH_BUDGET == 1000)


def test_jump_states():
    print("DO_JUMP counter states")
    # a jump with a repeat count behaves differently depending on how much of
    # its loop the mission has already run, and ArduPilot does not reset it
    counted = build([home()] +
                    waypoints([(-35.0, 149.0), (-35.02, 149.0)]) +
                    [item(3, mavlink.MAV_CMD_DO_JUMP, param1=1, param2=2),
                     item(4, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)])
    states = return_path.jump_states(counted)
    check("both counter states are considered", len(states) == 2, str(states))

    fresh = counted.flown_path(jump_counts={})
    used = counted.flown_path(jump_counts={3: 2})
    check("a fresh counter goes round the loop, a used one does not",
          len(fresh) > len(used), "%u vs %u" % (len(fresh), len(used)))

    # too many of them to enumerate is inconclusive rather than a guess
    many = [home()] + waypoints([(-35.0, 149.0)])
    for i in range(6):
        many.append(item(2 + i, mavlink.MAV_CMD_DO_JUMP, param1=1, param2=2))
    check("too many counter states to check is refused",
          return_path.jump_states(build(many)) is None)


def test_flown_path():
    print("the corridor")
    # a location on a command that is never flown to is not corridor
    with_roi = build([home()] +
                     waypoints([(-35.0, 149.0), (-35.02, 149.0)]) +
                     [item(3, mavlink.MAV_CMD_DO_SET_ROI, -36.5, 150.5),
                      item(4, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)])
    seqs = [p.seq for p in with_roi.flown_path()]
    check("a DO_SET_ROI is not part of the corridor", 3 not in seqs,
          str(seqs))

    # a forward jump means the items it skips are never flown, so they are not
    # corridor either
    skipping = build([home()] +
                     waypoints([(-35.0, 149.0)]) +
                     [item(2, mavlink.MAV_CMD_DO_JUMP, param1=4, param2=-1),
                      item(3, mavlink.MAV_CMD_NAV_WAYPOINT, -36.0, 150.0),
                      item(4, mavlink.MAV_CMD_NAV_WAYPOINT, -35.01, 149.0),
                      item(5, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)])
    seqs = [p.seq for p in skipping.flown_path()]
    check("what a forward DO_JUMP skips is not corridor", 3 not in seqs,
          str(seqs))

    # walking a loop for ever must still terminate
    looping = build([home()] +
                    waypoints([(-35.0, 149.0), (-35.02, 149.0)]) +
                    [item(3, mavlink.MAV_CMD_DO_JUMP, param1=1, param2=-1)])
    check("an endless loop still terminates",
          len(looping.flown_path()) <= 3 * looping.count())


def test_unmodelled():
    print("what we admit we cannot model")
    spline = simple_mission()
    spline.point(2).command = mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
    check("a spline waypoint is inconclusive",
          len(spline.unmodelled()) > 0, str(spline.unmodelled()))

    # a waypoint with no stored position is filled in from wherever the
    # aircraft is by Location::sanitize(), so it cannot be planned against
    nowhere = simple_mission()
    nowhere.point(2).lat = 0.0
    nowhere.point(2).lon = 0.0
    check("a waypoint with no position is inconclusive",
          len(nowhere.unmodelled()) > 0, str(nowhere.unmodelled()))

    # off to one side, so nothing else in the mission runs through where its
    # circle goes
    loiter_items = [home()]
    loiter_items += waypoints([(-35.00, 149.00)])
    loiter_items.append(item(2, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                             -35.02, 149.05))
    loiter_items += waypoints([(-35.04, 149.00)], start=3)
    loiter_items.append(item(4, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                             -35.04, 149.00))
    loiter_items += waypoints([(-35.00, 149.00)], start=5)
    loiter_items.append(item(6, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    loiter = build(loiter_items)
    check("a loiter with no radius anywhere is inconclusive",
          len(loiter.unmodelled(0.0)) > 0)
    check("a loiter is fine once a radius is known",
          len(loiter.unmodelled(120.0)) == 0, str(loiter.unmodelled(120.0)))
    # the orbit has to be the corridor, not the point at its centre: a
    # straight line through the middle of a circle is ground never flown
    orbit = loiter.flown_path(loiter_radius=120.0)
    centre = loiter.point(2)
    offsets = [math.hypot(p.x - centre.x, p.y - centre.y)
               for p in orbit if p.seq == centre.seq]
    check("the loiter is flown as its orbit",
          len(offsets) > 8 and all(abs(d - 120.0) < 1.0 for d in offsets),
          "%u points, %.1f..%.1f" % (len(offsets), min(offsets),
                                     max(offsets)) if offsets else "none")
    check("nothing is left sitting at the centre of the circle",
          all(d > 1.0 for d in offsets))
    index = loiter.corridor_index(120.0)
    check("the middle of the circle is not corridor",
          index.distance(centre.x, centre.y) > 100.0,
          "%.1fm" % index.distance(centre.x, centre.y))

    result = return_path.check_return_path(loiter, 20.0, 30.0,
                                           loiter_radius=0.0)
    check("an inconclusive mission is not a pass", not result.ok())


def test_terrain_reporting():
    print("terrain")
    items = [home()] + waypoints([(-35.0, 149.0), (-35.02, 149.0)]) + [
        item(3, mavlink.MAV_CMD_DO_RETURN_PATH_START, -35.02, 149.0),
        item(4, mavlink.MAV_CMD_NAV_WAYPOINT, -35.0, 149.0),
        item(5, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)]
    for it in items[1:]:
        it.frame = mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT

    calls = {'n': 0}

    def terrain(lat, lon):
        # the waypoints resolve but the ground between them does not, which
        # used to be counted and then never mentioned
        calls['n'] += 1
        for it in items:
            if abs(it.x - lat) < 1e-9 and abs(it.y - lon) < 1e-9:
                return 700.0
        return None

    mission = mission_model.build_mission(items, terrain_fn=terrain)
    result = return_path.check_return_path(mission, 20.0, 30.0,
                                           granularity=50.0,
                                           terrain_fn=terrain)
    check("terrain missing part way along a leg is counted",
          result.terrain_missing > 0, "%u" % result.terrain_missing)
    check("and that alone stops it being a pass", not result.ok())


def test_fix_generator():
    print("adding return paths")
    items = detour_items()
    mission = build(items)
    result = return_path.check_return_path(mission, 20.0, 30.0)
    check("the detour is caught", result.failed > 0,
          "worst %.0fm" % result.worst_deviation)

    added = add_return_paths.build(mission, items, result, 20.0, 30.0,
                                   separation=200.0)
    check("something is proposed", len(added) > 0)
    if len(added) == 0:
        return
    new_items = []
    for path in added:
        new_items.extend(path.items)
    check("the block starts with a DO_RETURN_PATH_START",
          new_items[0].command == mavlink.MAV_CMD_DO_RETURN_PATH_START)
    check("and ends jumping back onto the existing return",
          new_items[-1].command == mavlink.MAV_CMD_DO_JUMP)
    check("the new items go after the landing",
          new_items[0].seq >= mission.count())

    trial = return_path.check_return_path(build(items + new_items), 20.0, 30.0)
    check("failing samples are recorded, so a proposal can be judged on what "
          "it breaks rather than on a total",
          isinstance(trial.failed_at, set) and isinstance(result.failed_at,
                                                          set))

    # a proposal is only worth taking if nothing that passes now stops
    # passing. An extra DO_RETURN_PATH_START changes which return the rest of
    # the mission gets sent to, so a fix in one place can undo another
    # what a proposal is judged on: which samples it breaks, not how many
    # fail in total. Both have to be answerable for the gate to mean anything
    scores = {}
    for separation in (100.0, 200.0, 400.0, 800.0):
        proposal = add_return_paths.build(mission, items, result, 20.0, 30.0,
                                          separation=separation)
        if len(proposal) == 0:
            continue
        candidate = []
        for path in proposal:
            candidate.extend(path.items)
        outcome = return_path.check_return_path(build(items + candidate),
                                                20.0, 30.0)
        scores[separation] = (len(outcome.failed_at - result.failed_at),
                              outcome.failed)
    check("every separation gives an answer that can be judged",
          len(scores) == 4, str(sorted(scores)))
    check("the separations do not all behave the same, so the search has "
          "something to choose between",
          len(set(scores.values())) > 1, str(scores))

    # the gate itself: a proposal that breaks a sample which passes now must
    # be refused however much it improves the total
    breaks = [sep for (sep, (broken, _)) in scores.items() if broken > 0]
    clean = [sep for (sep, (broken, _)) in scores.items() if broken == 0]
    for sep in breaks:
        check("separation %.0f breaks %u passing points and must be refused"
              % (sep, scores[sep][0]), scores[sep][0] > 0)
    for sep in clean:
        check("separation %.0f breaks nothing, so its total is what counts"
              % sep, scores[sep][1] <= result.failed,
              "%u vs %u" % (scores[sep][1], result.failed))


def test_between_samples():
    print("between one sampling point and the next")
    # a return that is only the closest one over a short stretch of a leg.
    # Sampling alone steps over it, and the answer then depends on the
    # granularity, which is no answer at all
    items = [home(-35.0, 149.0)]
    items += waypoints([(-35.000, 149.0), (-35.001, 149.0)])
    items.append(item(3, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      -35.0005, 149.0))
    items.append(item(4, mavlink.MAV_CMD_NAV_WAYPOINT, -35.0005, 149.010))
    items.append(item(5, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    mission = build(items)
    verdicts = []
    for granularity in (50.0, 10.0, 1.0):
        result = return_path.check_return_path(mission, 20.0, 30.0,
                                               granularity=granularity)
        verdicts.append((result.ok(), round(result.worst_deviation)))
    check("a coarse run agrees with a fine one about the verdict",
          len(set(v[0] for v in verdicts)) == 1, str(verdicts))
    check("and about how bad it is",
          len(set(v[1] for v in verdicts)) == 1, str(verdicts))
    check("which is that it fails", not verdicts[0][0])


def test_work_bound():
    print("how much work a check may ask for")
    mission = simple_mission()
    path = mission.flown_path()
    length = mission_model.path_length(path)
    spacing = mission_model.usable_spacing(path, 0.001)
    samples = length / spacing
    check("the number of sampling points is bounded",
          samples <= mission_model.MAX_SAMPLES + 1, "%.0f" % samples)
    # each sample walks its own return, so the work goes as the square
    check("and so is the work, which grows as the square of that",
          samples * samples <= mission_model.MAX_QUERIES * 1.01,
          "%.3g" % (samples * samples))
    check("more counter states to check means coarser sampling",
          mission_model.usable_spacing(path, 0.001, 8) >
          mission_model.usable_spacing(path, 0.001, 1))


def test_loiter_fidelity():
    print("loiters as ArduPilot flies them")
    # which field carries the radius differs by command. NAV_LOITER_TIME has
    # no room for one at all, so a sign there is a direction and nothing else
    cases = [(mavlink.MAV_CMD_NAV_LOITER_TIME, 0.0, -1.0, 120.0, True),
             (mavlink.MAV_CMD_NAV_LOITER_TIME, 0.0, 1.0, 120.0, False),
             (mavlink.MAV_CMD_NAV_LOITER_TURNS, 0.0, 200.0, 200.0, False),
             (mavlink.MAV_CMD_NAV_LOITER_TURNS, 0.0, -200.0, 200.0, True),
             (mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0.0, -150.0, 150.0, True),
             (mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 300.0, 0.0, 300.0, False)]
    for (command, param2, param3, radius, ccw) in cases:
        mission = build([home(), item(1, command, -35.02, 149.05,
                                      param2=param2, param3=param3)])
        point = mission.point(1)
        check("%s with param2 %.0f param3 %.0f circles at %.0fm"
              % (mission_model.command_name(command), param2, param3, radius),
              point.loiter_radius(120.0) == radius,
              str(point.loiter_radius(120.0)))
        check("  and goes %s" % ("anticlockwise" if ccw else "clockwise"),
              point.loiter_ccw() == ccw)

    # the direction has to reach the orbit, not just the item
    for ccw in (False, True):
        sign = -1.0 if ccw else 1.0
        mission = build([home()] + waypoints([(-35.00, 149.00)]) +
                        [item(2, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                              -35.02, 149.05, param3=sign * 200.0)] +
                        waypoints([(-35.04, 149.00)], start=3))
        orbit = [p for p in mission.flown_path(loiter_radius=200.0)
                 if p.synthetic]
        centre = mission.point(2)
        turned = 0.0
        for i in range(1, len(orbit)):
            before = math.atan2(orbit[i - 1].y - centre.y,
                                orbit[i - 1].x - centre.x)
            after = math.atan2(orbit[i].y - centre.y, orbit[i].x - centre.x)
            step = (after - before + math.pi) % (2.0 * math.pi) - math.pi
            turned += step
        # in an east/north frame anticlockwise means the angle grows
        check("an %s loiter is flown that way round"
              % ("anticlockwise" if ccw else "clockwise"),
              (turned > 0) == ccw, "%.2f rad" % turned)

    # leaving radially would put a right angle in the path the aircraft never
    # flies, so it leaves where its heading round the circle points at the
    # next waypoint
    mission = build([home()] + waypoints([(-35.00, 149.00)]) +
                    [item(2, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                          -35.02, 149.05, param3=200.0)] +
                    waypoints([(-35.04, 149.00)], start=3))
    path = mission.flown_path(loiter_radius=200.0)
    orbit = [p for p in path if p.synthetic]
    following = [p for p in path if not p.synthetic and p.seq == 3][0]
    last = orbit[-1]
    before = orbit[-2]
    heading = math.atan2(last.y - before.y, last.x - before.x)
    onward = math.atan2(following.y - last.y, following.x - last.x)
    turn = abs((onward - heading + math.pi) % (2.0 * math.pi) - math.pi)
    check("it leaves the circle heading where it is going next",
          turn < math.radians(20.0), "%.0f degrees out" % math.degrees(turn))

    # an unlimited loiter is never left
    forever = build([home()] + waypoints([(-35.00, 149.00)]) +
                    [item(2, mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                          -35.02, 149.05, param3=200.0)] +
                    waypoints([(-35.04, 149.00)], start=3) +
                    [item(4, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0)])
    seqs = set(p.seq for p in forever.flown_path(loiter_radius=200.0))
    check("nothing after a NAV_LOITER_UNLIM is ever flown",
          3 not in seqs and 4 not in seqs, str(sorted(seqs)))


def test_synthetic_containment():
    print("what may be written back into a mission")
    items = [home(-35.0, 149.0)]
    items += waypoints([(-35.0, 149.0), (-35.10, 149.0)])
    items.append(item(3, mavlink.MAV_CMD_NAV_LOITER_TURNS, -35.15, 149.0,
                      param3=200.0))
    items += waypoints([(-35.20, 149.0)], start=4)
    items.append(item(5, mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      -35.20, 149.20))
    items += waypoints([(-35.10, 149.20), (-35.0, 149.0)], start=6)
    items.append(item(8, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    mission = build(items)
    result = return_path.check_return_path(mission, 20.0, 30.0,
                                           loiter_radius=200.0)
    path = mission.flown_path(loiter_radius=200.0)
    check("the orbit really is in the path",
          sum(1 for p in path if p.synthetic) > 10)
    if len(result.spans) == 0:
        check("a failing span to test with", False)
        return
    leaked = 0
    for span in result.spans:
        for vertex in add_return_paths.span_vertices(path, span):
            if vertex.synthetic:
                leaked += 1
    check("no invented point can become a mission item", leaked == 0,
          "%u leaked" % leaked)

    added = add_return_paths.build(mission, items, result, 20.0, 30.0,
                                   separation=200.0)
    stored = set(p.seq for p in path if not p.synthetic)
    for new_path in added:
        check("what gets added covers stored waypoints only",
              new_path.from_seq in stored and new_path.to_seq in stored,
              "%u..%u" % (new_path.from_seq, new_path.to_seq))


def test_barren_jump_state():
    print("counter states with no way home")
    # fresh, the jump goes back and the walk finds waypoints; once used up it
    # falls off the end of the mission having found none
    items = [home()]
    items += waypoints([(-35.00, 149.0), (-35.01, 149.0)])
    items.append(item(3, mavlink.MAV_CMD_DO_RETURN_PATH_START, 0.0, 0.0, 0.0,
                      frame=mavlink.MAV_FRAME_GLOBAL))
    items.append(item(4, mavlink.MAV_CMD_DO_JUMP, param1=1, param2=1))
    mission = build(items)
    states = return_path.jump_states(mission)
    found = [len(return_path.build_return_paths(mission, state))
             for state in states]
    check("one counter state has a return path and the other has none",
          sorted(found) == [0, 1], str(found))
    result = return_path.check_return_path(mission, 20.0, 30.0)
    check("a state with no way home is an error, not one state left out",
          not result.ok() and len(result.errors) > 0, str(result.errors))


def test_jump_tag_states():
    print("DO_JUMP_TAG counter states")
    tag_jump = getattr(mavlink, 'MAV_CMD_DO_JUMP_TAG', None)
    if tag_jump is None:
        check("this pymavlink knows about DO_JUMP_TAG", False)
        return
    items = [home()]
    items += waypoints([(-35.00, 149.0), (-35.01, 149.0)])
    items.append(item(3, tag_jump, param1=7, param2=2))
    items.append(item(4, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0))
    mission = build(items)
    check("a tag jump with a repeat count has counter states too",
          len(return_path.finite_jumps(mission)) == 1,
          str(return_path.finite_jumps(mission)))
    check("so both of them get checked",
          len(return_path.jump_states(mission)) == 2)


def test_landing_guard():
    print("guards on changing a mission")
    check("a mission ending at a landing may be appended to",
          simple_mission().ends_in_landing())

    open_ended = build([home()] +
                       waypoints([(-35.0, 149.0), (-35.02, 149.0)]))
    check("one that does not end at a landing may not",
          not open_ended.ends_in_landing())

    # with a takeoff after the landing the mission can carry on past it, so
    # anything appended would be flown as part of the mission
    again = build([home()] +
                  waypoints([(-35.0, 149.0)]) +
                  [item(2, mavlink.MAV_CMD_NAV_LAND, -35.0, 149.0, 0.0),
                   item(3, mavlink.MAV_CMD_NAV_TAKEOFF, -35.0, 149.0),
                   item(4, mavlink.MAV_CMD_NAV_WAYPOINT, -35.02, 149.0)])
    check("a takeoff after the landing is noticed",
          again.takeoff_after_landing())
    check("and without one it is not",
          not simple_mission().takeoff_after_landing())


class FakeSettings(object):
    state_basedir = None


class FakeMPState(object):
    """just enough of MAVProxy for the module to be built and driven"""

    def __init__(self):
        self.command_map = {}
        self.completions = {}
        self.completion_functions = {}
        self.public_modules = {}
        self.modules = []
        self.settings = FakeSettings()
        self.mav_param = {}
        self.functions = None
        self.mods = {}

    def module(self, name):
        return self.mods.get(name)


class FakeLoader(object):
    def __init__(self, items):
        self.items = list(items)
        self.last_change = 1.0
        self.expected_count = 0

    def count(self):
        return len(self.items)

    def wp(self, i):
        return self.items[i]

    def add(self, wp):
        self.items.append(wp)
        self.last_change += 1.0


class FakeWP(object):
    def __init__(self, items):
        self.wploader = FakeLoader(items)


def make_module(items=None):
    from MAVProxy.modules import mavproxy_bvlos_plan
    state = FakeMPState()
    if items is not None:
        state.mods['wp'] = FakeWP(items)
    state.public_modules = dict(state.mods)
    return (state, mavproxy_bvlos_plan.init(state))


def drive(module, limit=60.0):
    """run idle_task until the worker has been picked up"""
    import time
    started = time.time()
    while module.job is not None and time.time() - started < limit:
        module.idle_task()
        time.sleep(0.01)
    module.idle_task()
    return module.job is None


def test_module():
    print("the module")
    from MAVProxy.modules import mavproxy_bvlos_plan as bvlos

    job = bvlos.Job('a job', lambda: 'the answer')
    check("a job runs and hands its answer back", drive_job(job) == 'the answer')
    failing = bvlos.Job('a job', lambda: 1 / 0)
    drive_job(failing)
    check("a job that throws keeps the error rather than the traceback",
          failing.error is not None and failing.value is None)

    items = detour_items()
    (state, module) = make_module(items)
    snapshot = module.mission_items()
    check("the worker gets a snapshot, not the live mission",
          all(not isinstance(s, type(items[0])) for s in snapshot))
    original = items[0].x
    items[0].x = 12.0
    check("changing the mission does not change what the worker holds",
          snapshot[0].x == original)
    items[0].x = original

    # a mission that changes while a check runs must not have an answer about
    # the old one applied to it
    stamp = module.mission_stamp()
    state.mods['wp'].wploader.add(items[-1])
    check("a mission changing under a running job is noticed",
          module.mission_stamp() != stamp)

    # the gate
    before = return_path.CheckResult()
    before.failed = 10
    before.failed_at = set(range(10))
    better = return_path.CheckResult()
    better.failed = 4
    better.failed_at = set(range(4))
    check("a proposal that only fixes things is taken",
          module.improves(before, better))
    worse = return_path.CheckResult()
    worse.failed = 2
    worse.failed_at = set([0, 99])
    check("a proposal that breaks something new is refused however much it "
          "fixes", not module.improves(before, worse))
    same = return_path.CheckResult()
    same.failed = 10
    same.failed_at = set(range(10))
    check("a proposal that changes nothing is refused",
          not module.improves(before, same))
    module.unload()


def drive_job(job):
    import time
    started = time.time()
    while not job.done and time.time() - started < 30.0:
        time.sleep(0.01)
    return job.value


def main():
    for test in (test_units, test_path_index, test_capture_turn,
                 test_sampling_bound, test_arduplane_walk, test_jump_states,
                 test_flown_path, test_unmodelled, test_terrain_reporting,
                 test_fix_generator, test_between_samples, test_work_bound,
                 test_loiter_fidelity, test_synthetic_containment,
                 test_barren_jump_state, test_jump_tag_states,
                 test_landing_guard, test_module):
        test()
    print("")
    if FAILURES:
        print("FAILED: %u" % len(FAILURES))
        for name in FAILURES:
            print("  %s" % name)
        return 1
    print("all passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
