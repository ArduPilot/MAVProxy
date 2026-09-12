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

When the map module is loaded, the module automatically draws a cyan visible
footprint and a dark-blue thermal footprint for the selected camera.  The
projection uses each `VIDEO_STREAM_INFORMATION` resolution and horizontal FOV,
the selected gimbal's `GIMBAL_DEVICE_ATTITUDE_STATUS`, vehicle heading and the
terrain module's elevation model.  Without the terrain module it falls back to
a flat surface derived from `GLOBAL_POSITION_INT.relative_alt`.  Use
`camera set show_fov false` to hide the footprints.  `mount_roll`,
`mount_pitch`, `mount_yaw` and `mount_alt` provide installation offsets, and
`fov_update_interval` controls the map update period (0.2 seconds by default).
Partial-sky views are clipped against the maximum range in the earth frame,
then ordered as a convex ground envelope. This avoids crossed footprints and
keeps narrower streams nested inside wider streams with the same boresight.
`fov_max_range` rejects unreliable terrain intersections beyond 10 km by
default; set it to zero to disable this range limit.

## Custom camera settings

Choose **Camera → Custom Settings** in the console or map menu, or run
`camera custom`. The dialog uses the selected camera; use
`camera select SYSID:COMPID` when more than one camera is discovered.
Automatic selection prefers a matching camera component over ArduPilot's
duplicate camera information from component 1. The autopilot proxy does not
serve the camera's extended parameters. Explicit selections are preserved.

`CAMERA_INFORMATION.cam_definition_uri` supplies the camera's XML definition.
HTTP, HTTPS and MAVFTP downloads and XZ-compressed definitions are supported.
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
