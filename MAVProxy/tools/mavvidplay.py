#!/usr/bin/env python3
"""Play a telemetry video with attitude OSD, terrain footprint and map picking."""
import argparse
import math
import sys
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', help='H.264/H.265 recording with AP_CameraGimbal SEI or SIYI subtitle telemetry (first video track)')
    parser.add_argument('--tlog', help='optional MAVLink flight log for SIYI vehicle attitude, heading and speed')
    parser.add_argument('--tlog-system', type=int, help='flight-controller system ID (default: first autopilot heartbeat)')
    parser.add_argument('--tlog-offset', type=float, default=0, help='seconds to add to SIYI UTC timestamps when matching the flight log')
    parser.add_argument('--fov', type=float, help='override horizontal FOV in degrees (default: recorded per-frame FOV)')
    parser.add_argument('--zoom-fov', action='store_true', help='FOV is at 1x; scale focal length by recorded zoom')
    parser.add_argument('--ground-alt', type=float, help='use a flat ground plane at this AMSL altitude in metres')
    parser.add_argument('--terrain', choices=['SRTM1', 'SRTM3'], default='SRTM3')
    parser.add_argument('--offline', action='store_true', help='use cached map/terrain tiles only')
    parser.add_argument('--max-range', type=float, default=10000, help='maximum projection slant range in metres')
    parser.add_argument('--map-service', default='MicrosoftSat')
    parser.add_argument('--paused', action='store_true', help='open on the first frame without playing')
    args = parser.parse_args()
    if args.fov is not None and (not math.isfinite(args.fov) or not 0 < args.fov < 180):
        parser.error('--fov must be between 0 and 180 degrees')
    if not math.isfinite(args.max_range) or args.max_range <= 0:
        parser.error('--max-range must be positive and finite')
    if args.ground_alt is not None and not math.isfinite(args.ground_alt):
        parser.error('--ground-alt must be finite')
    if not math.isfinite(args.tlog_offset):
        parser.error('--tlog-offset must be finite')
    if args.tlog_system is not None and not 1 <= args.tlog_system <= 255:
        parser.error('--tlog-system must be 1..255')
    if args.zoom_fov and args.fov is None:
        parser.error('--zoom-fov requires --fov; recorded FOV already includes zoom')
    try:
        from MAVProxy.modules.lib.video_telemetry import VideoIndex, FlatElevation, ViewProjection, position, finite
        print('Indexing video telemetry...', flush=True)
        index = VideoIndex(args.video)
        if args.fov is None and not any(
                s.record and finite(s.record.get('hfov_deg')) and 0 < s.record['hfov_deg'] < 180
                for s in index.samples):
            parser.error('This video has no recorded FOV; supply --fov in degrees')
        print('%u frames, %u with telemetry, %.2f seconds' % (
            len(index.samples), index.telemetry_count, index.duration), flush=True)
        for warning in index.warnings:
            print(warning, file=sys.stderr)
        if args.tlog:
            from MAVProxy.modules.lib.siyi_video import FlightLog
            print('Indexing flight log...', flush=True)
            log = FlightLog(args.tlog, args.tlog_system)
            matched = log.apply(index, args.tlog_offset)
            print('%u video frames matched to vehicle attitude (system %u)' % (matched, log.system_id), flush=True)
        elif index.siyi_count:
            print('SIYI yaw is vehicle-relative. Add --tlog for vehicle attitude, heading and map projection.', flush=True)
        from MAVProxy.modules.lib import mp_elevation
        elevation = (FlatElevation(args.ground_alt) if args.ground_alt is not None else
                     mp_elevation.ElevationModel(args.terrain, offline=int(args.offline)))
        projection = ViewProjection(index.width, index.height, args.fov, elevation,
                                    args.max_range, args.zoom_fov)
        from MAVProxy.modules.lib import multiproc
        multiproc.freeze_support()
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        first = next((position(s.record) for s in index.samples if position(s.record) is not None), None)
        location = {'lat': first[0], 'lon': first[1]} if first else {}
        # Create the map before the parent wx application (safe with fork too).
        map_display = mp_slipmap.MPSlipMap(title='MAV Video Map', service=args.map_service,
                                          download=not args.offline, **location)
        try:
            from MAVProxy.modules.lib.mavvidplay import Player, wx
            app = wx.App(False)
            player = Player(index, projection, map_display, paused=args.paused)
            player.Show()
            app.MainLoop()
        finally:
            if map_display.is_alive():
                map_display.close()
    except (OSError, ValueError, ImportError, subprocess.SubprocessError) as error:
        print('mavvidplay: %s' % error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
