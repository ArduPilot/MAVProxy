#!/usr/bin/env python3
'''
record a SITL flight of a mission for the tests which check the path drawn
for it against the path flown

    python3 tests/missions/record_flight.py LOG OUT.json --source "..."

LOG is the flight's dataflash log.  OUT gets what MAVExplorer hands
plane_track.mission_track() for it -- home, the mission, the vehicle's
parameters, and where there are any, the takeoff course, where the flight
started, the rally points and the approach course -- and the path flown,
once a second, from when the aircraft is flying as a plane -- the end of a
QuadPlane's transition, or the first POS of anything else -- to when it
starts to land: a QuadPlane's landing descent, a plane's flare, or the end
of the log -- or to when anything but the mission takes over, as a test
does to end a mission which circles for ever.  --start and --end, in
seconds of the log's TimeUS, override either.

AP_FLAKE8_CLEAN
'''

import argparse
import importlib.util
import json
import os

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import plane_track

HERE = os.path.dirname(os.path.abspath(__file__))
# what the autopilot says as a flight starts and stops being fixed-wing
STARTS = ('Transition done',)
ENDS = ('Land descend started', 'Flare ')
# ArduPlane's modes a mission is flown in: AUTO, and the RTL and QRTL a
# return to launch switches to.  Anything else, or AUTO again after those,
# is the flight being taken over
AUTO = 10
RETURNING = (11, 21)


def explorer():
    path = os.path.join(HERE, '..', '..', 'MAVProxy', 'tools', 'MAVExplorer.py')
    spec = importlib.util.spec_from_file_location('mavexplorer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def extras(heading, start, rally, approach):
    '''what the drawing is given beyond the mission, where there is
    anything to say: a course of 0 is north, which is something'''
    out = {}
    for (name, value) in (('heading', heading), ('start', start),
                          ('rally', rally), ('approach', approach)):
        if value is not None and value != []:
            out[name] = value
    return out


def times(mlog):
    '''when the aircraft starts flying the mission as a plane -- the end of
    a QuadPlane's transition, or arming -- and when it starts to land or
    stops flying the mission'''
    transition = None
    armed = None
    first = None
    last = None
    events = []
    mlog.rewind()
    while True:
        m = mlog.recv_match(type=['MSG', 'POS', 'MODE', 'ARM'])
        if m is None:
            break
        t = m.TimeUS * 1.0e-6
        kind = m.get_type()
        if kind == 'POS':
            if first is None:
                first = t
            last = t
        elif kind == 'ARM':
            if armed is None and m.ArmState:
                armed = t
        elif kind == 'MODE':
            events.append((t, m.ModeNum))
        elif transition is None and m.Message.startswith(STARTS):
            transition = t
        elif m.Message.startswith(ENDS):
            events.append((t, None))
    start = transition
    if start is None:
        start = armed if armed is not None else first
    end = last
    returned = False
    for (t, mode) in events:
        if t < start:
            continue
        if mode is None or (mode != AUTO and mode not in RETURNING) or (
                mode == AUTO and returned):
            end = t
            break
        if mode in RETURNING:
            returned = True
    return (start, end)


def record(log, out, source, start=None, end=None):
    mx = explorer()
    mlog = mavutil.mavlink_connection(log)
    (path, mission, cmds, started, rally, origin,
     approach) = mx.mission_from_log(mlog)
    params = mp_util.log_params(mlog)
    ground0 = min(p[2] for p in path)
    mission = mx.resolve_mission_amsl(mission, ground0, params, mlog.mav_type)
    # what the drawing is given, taken as MAVExplorer gives it
    captured = {}
    real = plane_track.mission_track

    def capture(home, items, params, heading=None, start=None, rally=None,
                approach=None):
        captured['home'] = list(home)
        captured['items'] = [[c, la, lo, a, list(p)]
                             for (c, la, lo, a, p) in items]
        captured.update(extras(heading, start, rally, approach))
        return real(home, items, params, heading, start, rally, approach)
    plane_track.mission_track = capture
    try:
        mx.plane_mission_track(cmds, mission,
                               (path[0][0], path[0][1], ground0), params,
                               mlog.mav_type, path, started, rally, origin,
                               approach)
    finally:
        plane_track.mission_track = real
    if 'items' not in captured:
        raise ValueError('the log has no mission a plane flies')
    names = sorted(set(name for (names, _) in plane_track.PARAMETERS.values()
                       for (name, _) in names))
    captured['params'] = dict((name, params[name]) for name in names
                              if name in params)
    (flying, landing) = times(mlog)
    if start is not None:
        flying = start
    if end is not None:
        landing = end
    flown = []
    last = None
    mlog.rewind()
    while True:
        m = mlog.recv_match(type='POS')
        if m is None:
            break
        t = m.TimeUS * 1.0e-6
        if t < flying or t > landing:
            continue
        if last is not None and t - last < 1.0:
            continue
        last = t
        flown.append([round(m.Lat, 7), round(m.Lng, 7), round(m.Alt, 1)])
    captured['flown'] = flown
    captured['source'] = source
    with open(out, 'w') as f:
        json.dump(captured, f, separators=(',', ':'))
    return (captured, flying, landing)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('log')
    parser.add_argument('out')
    parser.add_argument('--source', required=True,
                        help='where the flight came from, for the record')
    parser.add_argument('--start', type=float, default=None)
    parser.add_argument('--end', type=float, default=None)
    args = parser.parse_args()
    (captured, flying, landing) = record(args.log, args.out, args.source,
                                         args.start, args.end)
    print('%s: %u items, %u points flown from %.1fs to %.1fs, %u bytes' % (
        args.out, len(captured['items']), len(captured['flown']), flying,
        landing, os.path.getsize(args.out)))


if __name__ == '__main__':
    main()
