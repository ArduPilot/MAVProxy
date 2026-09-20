"""Live graphs for standard camera/gimbal telemetry.

Each window is bound to one MAVLink source when opened. In particular, do not
mix a device's vehicle-frame yaw with an autopilot's earth-frame mount status.
"""

import math

from pymavlink import mavutil


GIMBAL_STATUS = "GIMBAL_DEVICE_ATTITUDE_STATUS"

# command name, menu/title, message type, fields (displayed with units)
PRESETS = (
    ("attitude", "Attitude", GIMBAL_STATUS,
     ("Roll (deg)", "Pitch (deg)", "Yaw, reported frame (deg)")),
    ("rates", "Angular rates", GIMBAL_STATUS,
     ("Roll rate (deg/s)", "Pitch rate (deg/s)", "Yaw rate (deg/s)")),
    ("pitchrate", "Pitch rate", GIMBAL_STATUS, ("Pitch rate (deg/s)",)),
    ("yawrate", "Yaw rate", GIMBAL_STATUS, ("Yaw rate (deg/s)",)),
    ("flags", "Gimbal flags", GIMBAL_STATUS, ("Gimbal flags (bitmask)",)),
    ("failures", "Gimbal failures", GIMBAL_STATUS, ("Failure flags (bitmask)",)),
    ("time", "Gimbal sample time", GIMBAL_STATUS, ("Gimbal boot time (s)",)),
    ("zoom", "Zoom", "CAMERA_SETTINGS", ("Zoom (%)",)),
    ("focus", "Focus", "CAMERA_SETTINGS", ("Focus (%)",)),
    ("mode", "Capture mode", "CAMERA_SETTINGS", ("Camera mode (enum)",)),
    ("tmax", "TMax", "CAMERA_THERMAL_RANGE", ("Maximum temperature (degC)",)),
    ("terrain", "Terrain height", "TERRAIN_REPORT", ("Height above terrain (m)",)),
    ("voltage", "Battery voltage", "SYS_STATUS", ("Vehicle battery voltage (V)",)),
)


class CameraGraphs:
    def __init__(self, module, quaternion_to_euler):
        self.module = module
        self.quaternion_to_euler = quaternion_to_euler
        self.windows = []

    def open(self, args):
        if args == ["close"]:
            camera = self.module.command_camera
            self.close(None if camera is None else (camera.system_id, camera.component_id))
            return
        preset = next((p for p in PRESETS if args == [p[0]]), None)
        if preset is None:
            print("camera graph <%s|close>" % "|".join(p[0] for p in PRESETS))
            return
        from MAVProxy.modules.lib import mp_util
        if not mp_util.has_wxpython:
            print("Camera graphs require wxPython and matplotlib")
            return
        name, title, message_type, fields = preset
        module = self.module
        camera = module._selected_camera(required=False)
        if message_type == GIMBAL_STATUS and module.command_camera is None and camera is not None:
            # a selected camera's manager overrides apply to its mount graphs
            with module._camera_context(camera):
                return self.open(args)
        system = camera.system_id if camera else module.target_system or 1
        component = module.camera_settings.manager_component
        gimbal_id = None
        stream_id = None
        if message_type == GIMBAL_STATUS:
            gimbal = module._selected_gimbal(required=False)
            if gimbal is not None:
                system = gimbal.system_id
            try:
                manager_id = module._manager_id() or 1
            except ValueError:
                if gimbal is None:
                    raise
                # The camera advertises a device but its manager instance is
                # unknown. Keep this graph on the explicitly associated device.
                manager_id = None
            manager_status = module.manager_attitudes.get((system, component, manager_id))
            if gimbal is not None and (gimbal.attitude is not None or
                                       manager_status is None):
                system, component = gimbal.system_id, gimbal.component_id
            else:
                # ArduPilot may consume targeted device reports locally and
                # publish only its own status to the GCS.
                gimbal_id = manager_id
        elif message_type in ("CAMERA_SETTINGS", "CAMERA_THERMAL_RANGE"):
            if camera is None:
                print("No MAVLink camera discovered; use 'camera discover'")
                return
            component = camera.component_id
            if message_type == "CAMERA_THERMAL_RANGE":
                streams = sorted(s.stream_id for s in camera.streams.values()
                                 if s.flags & mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
                if streams:
                    stream_id = streams[0]
                module._request_thermal_state(camera, force=True)
        else:
            if module.command_camera is None:
                system = module.target_system or 1
            component = module.target_component or 1

        source = "%u:%u" % (system, component)
        if gimbal_id is not None:
            source += " mount %u" % gimbal_id
        if stream_id is not None:
            source += " stream %u" % stream_id
        from MAVProxy.modules.lib.live_graph import LiveGraph
        graph = LiveGraph(list(fields), title="Camera: %s (%s)" % (title, source))
        owner = None if camera is None else (camera.system_id, camera.component_id)
        self.windows.append((name, message_type, system, component, gimbal_id, stream_id, owner, graph))
        print("Camera graph: %s (%s)" % (title, source))

    def values(self, name, message):
        if name == "attitude":
            return [math.degrees(v) for v in self.quaternion_to_euler(message.q)]
        if name in ("rates", "pitchrate", "yawrate"):
            axes = {"rates": "xyz", "pitchrate": "y", "yawrate": "z"}[name]
            return [math.degrees(getattr(message, "angular_velocity_" + axis))
                    for axis in axes]
        field, scale = {
            "flags": ("flags", 1),
            "failures": ("failure_flags", 1),
            "time": ("time_boot_ms", 0.001),
            "zoom": ("zoomLevel", 1),
            "focus": ("focusLevel", 1),
            "mode": ("mode_id", 1),
            "tmax": ("max", 1),
            "terrain": ("current_height", 1),
            "voltage": ("voltage_battery", 0.001),
        }[name]
        value = getattr(message, field)
        if name == "voltage" and value == 65535:
            raise ValueError("battery voltage unavailable")
        return [value * scale]

    def packet(self, message):
        for index, window in enumerate(self.windows):
            name, message_type, system, component, gimbal_id, stream_id, owner, graph = window
            if (message.get_type() != message_type or
                    message.get_srcSystem() != system or
                    message.get_srcComponent() != component or
                    (gimbal_id is not None and
                     getattr(message, "gimbal_device_id", 0) != gimbal_id)):
                continue
            if message_type == "CAMERA_THERMAL_RANGE":
                # A standalone camera's temperature must not be mixed with
                # another stream or an autopilot-attached camera instance.
                if getattr(message, "camera_device_id", 0) != 0:
                    continue
                if stream_id is not None and message.stream_id != stream_id:
                    continue
            try:
                values = self.values(name, message)
                # MAVLink NaN means unknown. LiveGraph cannot autoscale NaNs;
                # never invent a zero for an unsupported camera measurement.
                if not all(math.isfinite(v) for v in values):
                    continue
            except (AttributeError, TypeError, ValueError):
                continue
            if message_type == "CAMERA_THERMAL_RANGE" and stream_id is None:
                self.windows[index] = (name, message_type, system, component,
                                       gimbal_id, message.stream_id, owner, graph)
            graph.add_values(values)

    def idle(self):
        for window in self.windows[:]:
            if not window[-1].is_alive():
                window[-1].close()
                self.windows.remove(window)

    def close(self, owner=None):
        for window in self.windows[:]:
            if owner is None or window[-2] == owner:
                window[-1].close()
                self.windows.remove(window)
