# MAVLink camera module

The `camera` module controls Camera Protocol v2 components and associated
Gimbal Protocol v2 devices. Load it with `module load camera`; `camera` prints
the command summary. Discovery is automatic from camera and gimbal heartbeats.

The module supports discovery and selection of multiple camera components,
camera information, settings, storage and capture status, still sequences,
recording, zoom, focus, photo/video mode, RGB/thermal source selection, RTSP
stream discovery and viewing, stream state, gimbal information and attitude,
and body- or earth-frame angle/rate control through an ArduPilot gimbal
manager. Set `camera set mount_control device` only when MAVProxy connects
directly to a gimbal device rather than through ArduPilot.

`manager_gimbal_id` defaults to zero (the primary mount). Current ArduPilot
maps nonzero gimbal IDs as `instance = id - 1` rather than accepting a MAVLink
gimbal component ID, so setting it to the discovered component ID (for example
154) fails. The setting makes this workaround explicit and can be changed when
the ArduPilot mapping is corrected.

`camera view rgb`, `camera view thermal`, or `camera view all` opens discovered
RTSP streams when wxPython and an RTSP-capable OpenCV backend are installed.
The viewer uses the same GStreamer pipeline as `mavproxy_SIYI` when available,
and otherwise falls back to OpenCV's FFmpeg backend. Wildcard RTSP hosts are replaced by
`camera set rtsp_host ADDRESS`; conforming cameras should advertise an address
that the GCS can reach.

When the map module is loaded, the module automatically draws footprints for
all discovered cameras with available position and attitude telemetry. Each
camera has its own color and map layer (`Camera`, `Camera2`, etc.); thermal
streams use a darker shade of that camera's color. Polygon names include the
camera menu name, MAVLink system/component address and stream ID, for example
`Camera2FOV_1_101_1`. Updating one camera never removes another's polygons. The
projection uses each `VIDEO_STREAM_INFORMATION` resolution and horizontal FOV,
its associated gimbal's `GIMBAL_DEVICE_ATTITUDE_STATUS`, its vehicle's heading and the
terrain module's elevation model.  Without the terrain module it falls back to
a flat surface derived from that vehicle's `GLOBAL_POSITION_INT.relative_alt`.

Choose **Toggle Projection** in a camera's console or map menu to hide or restore
only that camera's footprints. Hiding is immediate; restoring uses cached
position telemetry when available. The command-line equivalents are
`camera for 1:101 projection toggle` and
`camera for 1:101 set show_fov false`. `camera projection` shows and refreshes the
selected camera's projection status. `camera set show_fov false` changes the
default for cameras without their own visibility override. `mount_roll`,
`mount_pitch`, `mount_yaw` and `mount_alt` provide installation offsets, and
`fov_update_interval` controls each camera's map update period (0.2 seconds by
default). Per-camera overrides apply to these settings as well as `fov_max_range`.
Partial-sky views are clipped against the maximum range in the earth frame,
then ordered as a convex ground envelope. This avoids crossed footprints and
keeps narrower streams nested inside wider streams with the same boresight.
`fov_max_range` rejects unreliable terrain intersections beyond 10 km by
default; set it to zero to disable this range limit.

## Multiple cameras

Each discovered camera has its own console and map menu: **Camera**, **Camera2**,
**Camera3**, and so on, ordered by MAVLink system ID and then component ID.
On one vehicle, component 100 precedes 101 regardless of heartbeat arrival order.
The Info item identifies its MAVLink
system/component address. Matching ArduPilot proxy reports are folded into the
real camera's menu. Cameras of the same model still get separate menus.

Menu actions, settings dialogs, video-window buttons and graphs stay bound to
their camera regardless of `camera select` or `camera_component`. All cameras
are polled for updated settings and recording state. The command-line equivalent
is `camera for SYSID:COMPID COMMAND`, for example:

```
camera for 1:101 photo
camera for 1:101 custom
camera for 1:101 graph attitude
camera for 1:101 graph close
```

The last command closes only that camera's graphs. A bare `camera graph close`
closes all camera graphs. Addressed commands do not change the command-line
selection. Map footprints are independent of the command-line camera selection.

Use `camera for SYSID:COMPID set NAME VALUE` to override module settings for
that camera's menus and addressed commands, such as its `rtsp_host`,
`mount_control`, `gimbal_component`, or `manager_gimbal_id`. Other settings
inherit the module defaults. Direct gimbal control and graphs use the camera's
advertised `gimbal_device_id`, or its explicit `gimbal_component` override.

For an ArduPilot manager (identified by its heartbeat), the module follows
`AP_Mount_MAVLink`'s mapping: gimbal component 154 belongs to MNT1, 171 to MNT2,
and 172–175 to MNT3–MNT6. This also selects the correct forwarded attitude report
when direct gimbal status is consumed by the autopilot. Discovery order does not
affect this mapping. An explicit per-camera setting overrides it:

```
camera for 1:100 set manager_gimbal_id 1
camera for 1:101 set manager_gimbal_id 2
```

Other managers still require explicit mapping when multiple cameras have distinct
gimbals and their association is ambiguous. A camera advertising a non-MAVLink
gimbal ID (1–6) already supplies its manager ID. Cameras sharing one advertised
gimbal can use the default manager setting.

Each projection needs fresh attitude telemetry for its own mount. For example,
if ArduPilot has `MNT2_TYPE=0`, discovering a second camera and its streams does
not provide mount-2 attitude. Configure the corresponding MAVLink mount backend
(`MNT2_TYPE=6`) or provide direct gimbal telemetry. `camera for SYSID:COMPID
projection` reports which mount status is missing.

## Map ROI

With more than one discovered camera, right-clicking the map opens **Set ROI →
ROI Camera1 / ROI Camera2 / … / ROI All**. The numbering matches the camera
menus and uses MAVLink address order. Each choice targets that camera's gimbal;
**ROI All** applies the clicked location to every camera. Terrain elevation
supplies the target's AMSL altitude. If terrain data or a required mount mapping
is unavailable, the command reports the problem before sending any ROI commands.
With zero or one camera, the map retains its existing **Set ROI** action.

In manager mode MAVProxy uses fresh vehicle positions to update the selected
mount's earth-frame pitch/yaw target at up to 5 Hz. This avoids ArduPilot's
vehicle-wide ROI command, which only addresses its primary mount. MAVProxy must
remain connected for this tracking; when position telemetry stops, it stops
updating the angles. In direct device mode, AP_CameraGimbal receives
`MAV_CMD_DO_SET_ROI_LOCATION` addressed to its gimbal component and tracks the
location itself, using vehicle telemetry available to the camera.

`camera for 1:101 roi` uses the current map click, and `camera roi all` targets
all cameras. `camera for 1:101 roi clear` stops that camera's tracking. Manual
angle/rate, center, neutral and retract commands also cancel its ROI. Camera
ROI choices do not change the map's shared ROI used by other modules.
Camera components using the same mount endpoint share one tracking target:
setting another ROI replaces the previous target, and clearing or manually
controlling the mount through either camera stops that tracking.

## Live graphs

Choose **Camera → Graphs** in the console or map menu. Graphs use MAVProxy's
usual live graph windows and require wxPython and matplotlib. The available
graphs are gimbal attitude, angular rates (all axes, pitch only, or yaw only),
gimbal flags, failure flags, sample time, camera zoom, focus, capture mode,
vehicle height above terrain, and vehicle battery voltage. `camera graph`
lists the command names; for example, `camera graph attitude` or
`camera graph rates`. `camera graph close` closes all camera graph windows.

Each window stays bound to the camera/gimbal selected by its menu or command,
identified in its title. Open another window to graph another camera. Gimbal
graphs use direct `GIMBAL_DEVICE_ATTITUDE_STATUS` when available, otherwise
ArduPilot's reports for `manager_gimbal_id` (zero selects primary mount 1).
Attitude is in degrees and rates in degrees/second; yaw uses the frame reported
by that source, without the SIYI protocol's sign inversion. The flags graph
helps identify frame/mode changes. Sample time is the sender's boot time in
seconds. Capture mode is the camera photo/video enum, not SIYI's gimbal mode.
Camera zoom/focus/mode update at the camera state polling interval.

Graphs need their corresponding MAVLink messages to arrive; unsupported or
unknown values are not plotted. In particular, angular rates may be unavailable
in an autopilot's forwarded status even when attitude is available. Terrain
height and battery voltage come from the selected vehicle's autopilot. Voltage
is the total pack voltage, without `siyi.scr`'s aircraft-specific six-cell divisor.
SIYI's controller demand/error, encoder, motor-voltage, threshold and temperature
graphs have no equivalent telemetry in AP_CameraGimbal and are not included.

## Custom camera settings

Choose **Camera → Custom Settings** in the console or map menu, or run
`camera custom`. Each menu opens its own camera's dialog. The command uses the
selected camera; use
`camera select SYSID:COMPID` when more than one camera is discovered.
Automatic selection prefers a matching camera component over ArduPilot's
duplicate camera information from component 1. The autopilot proxy does not
serve the camera's extended parameters. Autopilot definitions are loaded only
when explicitly selected or requested, avoiding duplicate downloads and
parameter requests. Explicit selections are preserved.

`CAMERA_INFORMATION.cam_definition_uri` supplies the camera's XML definition.
HTTP, HTTPS and MAVFTP downloads and XZ-compressed definitions are supported.
Camera settings require pymavlink 2.4.38 or later and defusedxml (installed by
MAVProxy). Definitions are limited to 4 MiB before and after XZ decompression;
MAVFTP enforces the transfer limit before buffering data. DTDs and XML entities
are rejected. HTTP redirects must remain HTTP/HTTPS. Camera-advertised URLs
may refer to local or private networks, so only connect trusted MAVLink peers.
Load `module load ftp` for MAVFTP URLs. Downloads run without blocking camera
telemetry. A changed URI or definition version reloads the metadata and values.
Use `camera definition` to retry a download, or `camera definition FILE|URL` to
load an explicit definition, such as a vendor-supplied local XML file.

The dialog preserves XML order across tabs, with checkboxes for booleans,
dropdowns for named options, sliders for bounded stepped values, and numeric
fields for other settings. Checkboxes, dropdowns and released sliders apply
immediately; numeric fields apply on Enter or their Apply button. Values are
validated against the type, bounds, steps and currently allowed options.
Read-only settings cannot be changed, and settings that have not replied to
parameter requests remain disabled. XML defaults are metadata, not assumed
current camera values. Vendor-specific `custom` binary types have no editor.

Changes use binary `PARAM_EXT_SET` values and wait for `PARAM_EXT_ACK`.
Pending writes, rejections and timeouts are shown in the dialog. Exclusion
rules, conditional option ranges and dependent parameter refreshes follow the
camera definition, including self-refreshing actions such as Workswell's NUC.
Incoming camera values update open dialogs, and periodic reads pick up changes
made by another controller. **Refresh** requests the full parameter list with
`PARAM_EXT_REQUEST_LIST`, then retries missing parameters individually. Closing a
dialog does not undo changes already applied to the camera.

`camera params` lists the received values, and `camera param NAME VALUE`
provides the same validated, acknowledged writes from the command line.

## ArduPilot and MT11 configuration

For the MT11, use one `CAM1_TYPE=6` MAVLink Camera v2 backend and one
`MNT1_TYPE=6` MAVLink mount backend on the same `NET_P1` MAVLink connection.
The MT11 is one camera component (100) with visible and thermal streams, not
two camera components. `CAM2_TYPE=6` searches for camera component 101 and does
not help select or view the MT11's second stream. A second camera backend will
be useful only if a future device exposes a second `MAV_TYPE_CAMERA` component.

Camera commands are addressed to the selected camera component and routed by
ArduPilot. Mount commands use `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` addressed to
the autopilot, which then drives its configured mount backend. This preserves
ArduPilot mount arbitration and frame handling.

## Current limitations

- Standard MAVLink has no stable messages for the MT11 LiDAR, thermal palette,
  thermal gain/environment corrections, radiometric raw frames, temperature
  extrema (the current thermal-range message is work in progress), or the
  vendor-specific face-down and gimbal modes. These remain unavailable here.
- The current MT11 camera service supports vehicle-frame gimbal commands. It
  does not yet consume vehicle attitude well enough to advertise direct
  earth-frame control; earth-frame commands must pass through ArduPilot.
- `VIDEO_STREAM_INFORMATION` describes compressed video. It does not transport
  video inside MAVLink, so the GCS still needs IP reachability to each RTSP URL.
- Viewing requires wxPython and an OpenCV backend with appropriate RTSP and
  H.264/H.265 decoder support. MAVProxy's usual OpenCV/FFmpeg packages provide
  this even when OpenCV was not compiled with GStreamer.
- Camera definition parameters (`PARAM_EXT_*`) are not yet implemented by the
  MT11 camera app. Basic controls are discovered from `CAMERA_INFORMATION` and
  stream configuration from `VIDEO_STREAM_INFORMATION`.
- MAVLink signing and encryption are properties of the surrounding link. The
  MT11 currently emits unsigned MAVLink and its RTSP transport is unencrypted.
