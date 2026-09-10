# Telemetry video player

`mavvidplay.py` opens a recorded video with an attitude OSD and a separate MAVProxy
map. It reads the `apcg.telemetry.v1` JSON carried in H.264/H.265 unregistered SEI
messages by AP_CameraGimbal. No live MAVLink connection is needed. SIYI vendor MP4 subtitle telemetry is also
supported; an optional flight log supplies vehicle attitude for its map projection. MP4/MOV and Matroska recordings with presentation timestamps are supported;
the first video track is used. Audio is not played.

Install MAVProxy normally to install the command. The player also needs `ffprobe`
(from FFmpeg), OpenCV, NumPy, wxPython and the usual MAVProxy map dependencies.
From a source checkout, run with `PYTHONPATH=.`:

```sh
python MAVProxy/tools/mavvidplay.py flight.mp4
```

New AP_CameraGimbal recordings include `hfov_deg`: the effective horizontal field
of view for each frame, including optical zoom and digital crop. The player uses
this automatically, updates the projection as zoom changes, and shows the FOV in
the OSD. It does not apply the recorded zoom a second time. Thermal FOV remains
independent of visible-camera zoom.

`--fov` overrides the recorded horizontal FOV in degrees, and is required for older
recordings or SIYI vendor videos that have no FOV metadata. By default this override
is constant. To scale a supplied 1x FOV by recorded zoom, also add `--zoom-fov`.
This option requires `--fov`. The projection uses an ideal pinhole camera with
square pixels; lens distortion and electronic stabilization can affect map
accuracy. Camera-app's recorded FOV is a nominal lens estimate.

The default terrain source is SRTM3; `--terrain SRTM1` selects finer terrain.
For a known flat ground altitude (metres AMSL), or an offline test:

```sh
mavvidplay.py flight.mp4 --ground-alt 580 --offline --paused
```

`--offline` disables map/terrain downloads and uses cached tiles.
`--map-service` selects a MAVProxy map service (default `MicrosoftSat`).
`--max-range` limits ground projections to a slant range in metres (default 10000).
Relative altitude is not assumed to be height above terrain. Missing terrain or
missing/stale position or attitude removes the footprint and disables picking;
pixels above the horizon also have no ground location.

## Playback and map controls

- **Space / Play / Pause:** pause or resume; Play at the end restarts the video.
- **Left / Right:** rewind or fast-forward 10 seconds; hold Shift for 60 seconds.
- **Comma / Period:** previous/next frame, pausing playback.
- **Home / End:** seek to the first/last frame.
- **Timeline slider:** seek directly to a frame.
- **Speed selector:** 0.25x, 0.5x, 1x, 2x, 4x or 8x.
- **Follow on map:** keep the map centred on the recorded camera position.
- **Mouse over video:** a small yellow map circle tracks the ground intersection
  of the pixel under the cursor, including while paused. Letterbox bars and pixels
  with no ground intersection clear the circle.
- **Left click in video:** add a red numbered marker at the corresponding map
  location. Markers remain through seeks; their timestamp, coordinates and terrain
  altitude are printed to the terminal. They last for the playback session.

The yellow outline is the camera footprint; the white circle is camera position.
Vehicle and gimbal roll/pitch/yaw are shown in degrees. The gimbal OSD shows its
recorded vehicle-relative yaw; projection adds vehicle yaw to obtain earth yaw,
as in `mavproxy_camera`. The OSD heading comes from recorded vehicle heading.
Ground speed (m/s) is estimated from successive GPS positions and their source
times; it is unavailable until two distinct position updates are available.
Missing values are shown as `--`, rather than retaining data from another frame.

Telemetry is indexed at startup without decoding the video. Playback uses container
presentation timestamps, including when packets are stored out of display order;
SEI timestamps may have a different epoch after remuxing. Decoding and seeking run
in a worker so the controls stay responsive. The map, hover location and OSD always
use telemetry from the image currently displayed. Recordings whose telemetry has
been stripped by transcoding, other telemetry schemas, broken SIYI rollover tracks, and raw streams without
presentation timestamps are not supported.

## SIYI vendor recordings

SIYI `mov_text`/`tx3g` subtitle tracks are detected automatically. The player reads
GPS DMS coordinates, absolute altitude, gimbal roll/pitch/yaw, zoom ratio and the
recorded UTC clock. Empty subtitle samples and style boxes are ignored. It matches
both frame counters and timestamps, allowing the thermal track's 1/25-second
rounding. It rejects rollover tracks whose counters/timestamps belong to another
recording instead of shifting their data onto unrelated images.

For the MT11 vendor recordings, the complete files starting at `02:52:58` work.
For example, from the MAVProxy checkout:

```sh
PYTHONPATH=. python MAVProxy/tools/mavvidplay.py \
  /home/tridge/project/UAV/Phoenix/logs/2026-08-28/SIYI/video/2026-08-28_02-52-58_753_A.mp4 \
  --fov 88 \
  --tlog /home/tridge/project/UAV/Phoenix/logs/2026-08-28/flight1/flight.tlog
```

Set `--fov` to the horizontal FOV for the recorded camera mode (the example uses
88 degrees). The subtitle focal length alone is insufficient to determine FOV
without sensor geometry. `--zoom-fov` uses SIYI's recorded `mix_ratio` as zoom.

Without `--tlog`, video playback, gimbal OSD, GPS map position and estimated ground
speed work. Vehicle attitude/heading remain unavailable, and the footprint,
hover circle and map picking are disabled: the vendor gimbal yaw is relative to
the vehicle. GPS course is not substituted for vehicle heading.

With `--tlog`, timestamped `ATTITUDE` and `GLOBAL_POSITION_INT` messages provide
vehicle attitude, heading and ground speed. The first flight controller heartbeat
selects the system; use `--tlog-system` to select another system explicitly.
Messages more than one second from a subtitle timestamp are not used. The
`--tlog-offset` option adds seconds to the subtitle UTC time when matching the
flight log. It does not move telemetry to a different video frame.

The subtitle UTC clock has only whole-second resolution, so flight-log matching
has that timing limitation. Recorded UTC and video elapsed time advance at
different rates in these files. Speed estimates use successive recorded UTC
seconds and their GPS positions, not the video playback clock. GPS is rounded to
0.1 arcsecond; log-derived speed is preferable when available. The vendor's
`rel_alt` duplicates absolute altitude, so it is not used as relative altitude.

## Tests

```sh
PYTHONPATH=. python -m unittest discover -s tests -p test_video_telemetry.py -v
PYTHONPATH=. python -m unittest discover -s tests -p test_siyi_video.py -v
MAVVIDPLAY_GUI_TEST=1 PYTHONPATH=.:tests python -m unittest discover -s tests -p test_mavvidplay_gui.py -v
```

The third command requires a working graphical display and opens/closes video and
map windows. It tests actual mouse picking, hover removal, playback controls,
seeking, missing telemetry, marker persistence and shutdown, with downloads disabled.
