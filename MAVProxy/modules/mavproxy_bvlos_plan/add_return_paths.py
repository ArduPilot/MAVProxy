#!/usr/bin/env python3
'''
Build alternative return paths to fix the parts of a mission that fail the
return path check.

Where a stretch of the mission has no return path leg near it, an RTL from
there cuts across ground the mission never covers. The fix is to give that
stretch a return path of its own: a DO_RETURN_PATH_START and a run of
waypoints back along the mission the way the aircraft came, offset to one
side by the turn radius so the reversal is a turn the aircraft can fly rather
than a course reversal on the spot. A DO_JUMP at the end brings it back onto
the existing return path.

The new items go at the end of the mission, after the landing, so they change
nothing about the mission as flown. ArduPilot considers every
DO_RETURN_PATH_START in the mission and follows DO_JUMP while it walks a
return path, so the appended block is found and leads home.
'''

# AP_FLAKE8_CLEAN

import math

from pymavlink import mavutil

from MAVProxy.modules.mavproxy_bvlos_plan import mission_model
from MAVProxy.modules.mavproxy_bvlos_plan import return_path

mavlink = mavutil.mavlink

# DO_JUMP repeat count meaning "always", AP_MISSION_JUMP_REPEAT_FOREVER
JUMP_FOREVER = -1


class NewPath(object):
    '''one alternative return path that was added'''

    def __init__(self, from_seq, to_seq, separation, from_setting,
                 rejoin_seq, items):
        # the stretch of mission it covers, in mission order
        self.from_seq = from_seq
        self.to_seq = to_seq
        # how far to one side the new legs were put, in metres
        self.separation = separation
        # True if that came from the setting rather than the turn radius
        self.from_setting = from_setting
        self.rejoin_seq = rejoin_seq
        self.items = items


def span_vertices(path, span):
    '''the mission points a failing span covers, in mission order'''
    legs = sorted(span.legs)
    first = legs[0][0]
    last = legs[-1][1]
    return [p for p in path if first <= p.seq <= last]


def offset_side(points, distance):
    '''offset each point to the right of the direction of travel.

       points are in the order they will be flown, so this puts the new path
       to one side of the mission leg it shadows and the aircraft turns onto
       it rather than reversing on the spot.
    '''
    out = []
    for i in range(len(points)):
        if i + 1 < len(points):
            (ax, ay) = (points[i].x, points[i].y)
            (bx, by) = (points[i + 1].x, points[i + 1].y)
        else:
            (ax, ay) = (points[i - 1].x, points[i - 1].y)
            (bx, by) = (points[i].x, points[i].y)
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length <= 0:
            out.append((points[i].x, points[i].y))
            continue
        # right of travel in an east/north frame
        out.append((points[i].x + distance * dy / length,
                    points[i].y - distance * dx / length))
    return out


def nearest_return_item(paths, point):
    '''the item of an existing return path closest to a mission point, which
       is where the new path rejoins'''
    best = None
    best_distance = None
    for path in paths:
        for candidate in path.points:
            d = math.hypot(candidate.x - point.x, candidate.y - point.y)
            if best_distance is None or d < best_distance:
                best_distance = d
                best = candidate
    return best


def make_item(target_system, target_component, seq, command, frame,
              lat, lon, alt, param1=0.0, param2=0.0):
    '''a new mission item addressed to the same vehicle as the mission'''
    return mavlink.MAVLink_mission_item_message(
        target_system, target_component, seq, frame, command, 0, 1,
        param1, param2, 0.0, 0.0, lat, lon, alt)


def build_for_span(mission, span, existing_paths, cruise_eas, roll_limit_deg,
                   target_system, target_component, next_seq, separation=0.0):
    '''the items for one alternative return path, or None if there is nothing
       useful to add'''
    path = mission.flown_path()
    vertices = span_vertices(path, span)
    if len(vertices) < 2:
        return None

    # fly it back the way we came
    vertices = list(reversed(vertices))

    # how far to one side to put the new legs. By default the turn radius
    # over this stretch, taking the largest so it is enough everywhere along
    # it, as that is what the aircraft can actually fly. Never the return
    # path width, which is only a check tolerance
    from_setting = separation > 0
    if from_setting:
        offset = separation
    else:
        offset = max(mission_model.turn_radius(cruise_eas, roll_limit_deg,
                                               v.amsl)
                     for v in vertices)

    rejoin = nearest_return_item(existing_paths, vertices[-1])
    if rejoin is None:
        return None

    offsets = offset_side(vertices, offset)
    items = []
    seq = next_seq
    for (i, vertex) in enumerate(vertices):
        (lat, lon) = mission.projector.unproject(offsets[i][0], offsets[i][1])
        command = (mavlink.MAV_CMD_DO_RETURN_PATH_START if i == 0
                   else mavlink.MAV_CMD_NAV_WAYPOINT)
        items.append(make_item(target_system, target_component, seq, command,
                               vertex.frame, lat, lon, vertex.alt))
        seq += 1
    # back onto the existing return path
    items.append(make_item(target_system, target_component, seq,
                           mavlink.MAV_CMD_DO_JUMP, 0, 0.0, 0.0, 0.0,
                           param1=float(rejoin.seq),
                           param2=float(JUMP_FOREVER)))

    return NewPath(vertices[-1].seq, vertices[0].seq, offset,
                   from_setting, rejoin.seq, items)


def build(mission, items, result, cruise_eas, roll_limit_deg,
          separation=0.0):
    '''alternative return paths covering every failing span of a check.

       Returns a list of NewPath, whose items are to be appended to the
       mission in order. items is the mission as mavlink items, used to
       address the new ones to the same vehicle. separation, if greater than
       zero, is used instead of the turn radius to offset the new legs.
    '''
    if len(result.spans) == 0 or len(items) == 0:
        return []
    target_system = getattr(items[0], 'target_system', 0)
    target_component = getattr(items[0], 'target_component', 0)

    existing_paths = return_path.build_return_paths(mission)
    if len(existing_paths) == 0:
        return []

    added = []
    next_seq = mission.count()
    for span in result.spans:
        new_path = build_for_span(mission, span, existing_paths, cruise_eas,
                                  roll_limit_deg, target_system,
                                  target_component, next_seq, separation)
        if new_path is None:
            continue
        added.append(new_path)
        next_seq += len(new_path.items)
    return added
