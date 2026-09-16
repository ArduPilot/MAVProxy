#!/usr/bin/env python3
'''
record a SITL flight of a mission for the tests which check the path drawn
for it against the path flown

    python3 tests/missions/record_flight.py LOG OUT.json --source "..."

LOG is the flight's dataflash log.  OUT gets the mission and the vehicle's
parameters, as MAVExplorer hands them to plane_track, and the path flown,
once a second, from when the aircraft is flying as a plane -- the end of a
QuadPlane's transition, or the first POS of anything else -- to when it
starts to land: a QuadPlane's landing descent, a plane's flare, or the end
of the log.  --start and --end, in seconds of the log's TimeUS, override
either.

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


def explorer():
    path = os.path.join(HERE, '..', '..', 'MAVProxy', 'tools', 'MAVExplorer.py')
    spec = importlib.util.spec_from_file_location('mavexplorer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def times(mlog):
    '''when the aircraft starts flying as a plane, and starts to land'''
    start = None
    end = None
    first = None
    last = None
    mlog.rewind()
    while True:
        m = mlog.recv_match(type=['MSG', 'POS'])
        if m is None:
            break
        t = m.TimeUS * 1.0e-6
        if m.get_type() == 'POS':
            if first is None:
                first = t
            last = t
            continue
        if start is None and m.Message.startswith(STARTS):
            start = t
        if start is not None and end is None and m.Message.startswith(ENDS):
            end = t
    return (first if start is None else start, last if end is None else end)


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

    def capture(home, items, params, *args, **kwargs):
        captured['home'] = list(home)
        captured['items'] = [[c, la, lo, a, list(p)]
                             for (c, la, lo, a, p) in items]
        return real(home, items, params, *args, **kwargs)
    plane_track.mission_track = capture
    try:
        mx.plane_mission_track(cmds, mission,
                               (path[0][0], path[0][1], ground0), params,
                               mlog.mav_type, rally=rally, origin=origin,
                               approach=approach)
    finally:
        plane_track.mission_track = real
    if 'items' not in captured:
        raise ValueError('the log has no mission a plane flies')
    names = sorted(set(name for (names, _) in plane_track.PARAMETERS.values()
                       for (name, _) in names))
    captured['params'] = dict((name, params[name]) for name in names
                              if name in params)
    if rally:
        captured['rally'] = [
            [lat, lon, mx.rally_point_amsl(lat, lon, alt, flags,
                                           captured['home'][2], origin)]
            for (lat, lon, alt, flags) in rally]
    if approach is not None:
        captured['approach'] = approach
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
