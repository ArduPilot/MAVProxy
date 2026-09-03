#!/usr/bin/env python3
"""Generic MAVLink Camera Protocol v2 and Gimbal Protocol v2 control."""

import math
import time
from urllib.parse import urlsplit, urlunsplit

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import camera_projection
from pymavlink import mavutil
from pymavlink.quaternion import Quaternion

if mp_util.has_wxpython:
    from MAVProxy.modules.mavproxy_map import mp_slipmap


def _text(value):
    if isinstance(value, str):
        return value.rstrip("\0")
    try:
        return bytes(value).split(b"\0", 1)[0].decode("utf-8", "replace")
    except (TypeError, ValueError):
        return str(value).rstrip("\0")


def _firmware_version(value):
    return ".".join(str((value >> shift) & 0xff)
                    for shift in (0, 8, 16, 24))


def _enum_name(enum_name, value):
    entry = mavutil.mavlink.enums.get(enum_name, {}).get(int(value))
    if entry is not None:
        return entry.name
    return "%s_%u" % (enum_name, value)


def _quaternion_to_euler(q):
    """Convert MAVLink w,x,y,z quaternion without a +/-90 degree singularity."""
    if q is None or len(q) != 4 or not all(math.isfinite(value) for value in q):
        raise ValueError("invalid quaternion")
    w, x, y, z = q
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    if abs(sin_pitch) > 1.0 - 1.0e-7:
        # Roll and yaw are coupled at gimbal lock.  Choose zero roll and
        # retain heading, avoiding atan2(0, tiny-negative) becoming 180deg.
        roll = 0.0
        yaw = 2.0 * math.atan2(z, w)
    else:
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class _FlatElevationModel:
    """Constant terrain fallback derived from GLOBAL_POSITION_INT."""

    def __init__(self, elevation):
        self.elevation = elevation

    def GetElevation(self, _latitude, _longitude):
        return self.elevation


class CameraDevice:
    def __init__(self, system_id, component_id):
        self.system_id = system_id
        self.component_id = component_id
        self.last_seen = time.time()
        self.last_request = 0.0
        self.information = None
        self.settings = None
        self.capture_status = None
        self.recording = None
        self.recording_verify_at = 0.0
        self.storage = {}
        self.streams = {}
        self.definition = None

    def label(self):
        if self.information is None:
            return "%u:%u" % (self.system_id, self.component_id)
        return "%s %s (%u:%u)" % (
            _text(self.information.vendor_name),
            _text(self.information.model_name),
            self.system_id, self.component_id)


class GimbalDevice:
    def __init__(self, system_id, component_id):
        self.system_id = system_id
        self.component_id = component_id
        self.last_seen = time.time()
        self.last_request = 0.0
        self.information = None
        self.attitude = None

    def label(self):
        if self.information is None:
            return "%u:%u" % (self.system_id, self.component_id)
        return "%s %s (%u:%u)" % (
            _text(self.information.vendor_name),
            _text(self.information.model_name),
            self.system_id, self.component_id)


class CameraModule(mp_module.MPModule):
    """MAVProxy operator interface for standard MAVLink cameras and mounts."""

    def __init__(self, mpstate):
        super(CameraModule, self).__init__(
            mpstate, "camera", "MAVLink camera and gimbal control", public=True)
        self.camera_settings = mp_settings.MPSettings([
            ("camera_component", int, 0),
            ("gimbal_component", int, 0),
            ("manager_component", int, 1),
            ("manager_gimbal_id", int, 0),
            ("mount_control", str, "manager"),
            ("rtsp_host", str, ""),
            ("rtsp_latency", int, 100),
            ("request_interval", float, 2.0),
            ("status_interval", float, 5.0),
            ("show_fov", bool, True),
            ("fov_update_interval", float, 0.2),
            ("fov_max_range", float, 10000.0),
            ("mount_roll", float, 0.0),
            ("mount_pitch", float, 0.0),
            ("mount_yaw", float, 0.0),
            ("mount_alt", float, 0.0),
        ])
        self.add_command(
            "camera", self.cmd_camera, "MAVLink camera control",
            ["<status|discover|select|info|streams|view|projection|photo|stopphotos|record|zoom|focus|mode|source|stream|mount|set>",
             "set (CAMERASETTING)"])
        self.add_completion_function("(CAMERASETTING)",
                                     self.camera_settings.completion)
        self.cameras = {}
        self.gimbals = {}
        self.manager_attitudes = {}
        self.selected_camera = None
        self.selected_gimbal = None
        self.views = {}
        self.last_status_request = 0.0
        self.last_discovery_request = 0.0
        self.last_fov_update = 0.0
        self.fov_objects = set()
        self.fov_points = {}
        self.last_ack = {}
        self.pending_commands = {}
        self.menu = None
        self.menu_modules = set()
        if mp_util.has_wxpython:
            from MAVProxy.modules.lib.mp_menu import MPMenuItem, MPMenuSubMenu
            self.menu = MPMenuSubMenu("Camera", items=[
                MPMenuItem("Status", returnkey="# camera status"),
                MPMenuItem("Discover", returnkey="# camera discover"),
                MPMenuItem("Take photo", returnkey="# camera photo"),
                MPMenuItem("Toggle recording",
                           returnkey="# camera record toggle"),
                MPMenuItem("Autofocus", returnkey="# camera focus auto"),
                MPMenuItem("Center gimbal",
                           returnkey="# camera mount center"),
                MPMenuItem("View RGB", returnkey="# camera view rgb"),
                MPMenuItem("View thermal",
                           returnkey="# camera view thermal"),
            ])

    def unload(self):
        self.remove_command("camera")
        self._clear_fov()
        for view in self.views.values():
            view.close()
        self.views.clear()
        if self.menu is not None:
            for name in self.menu_modules:
                module = self.module(name)
                if module is not None:
                    module.remove_menu(self.menu)
        super(CameraModule, self).unload()

    def usage(self):
        return """Usage:
  camera status                       show discovered cameras and gimbals
  camera discover                     request fresh discovery information
  camera select [SYSID:]COMPID         select a camera
  camera info                          show selected camera information
  camera streams                       show discovered video streams
  camera view <ID|rgb|thermal|all>     open RTSP viewer(s)
  camera projection                    show/refresh map projection status
  camera photo [INTERVAL [COUNT]]      capture one or a sequence of photos
  camera stopphotos                    stop an indefinite/interval capture
  camera record <start|stop|toggle>    control recording
  camera zoom <PERCENT|in|out|stop>    control optical zoom
  camera focus <auto|PERCENT|in|out|stop>
  camera mode <photo|video>            set capture mode
  camera source <rgb|thermal>          select the primary image source
  camera stream <start|stop> [ID]      set logical streaming state
  camera mount info
  camera mount angle PITCH YAW [body|earth]
  camera mount rate PITCH_RATE YAW_RATE [body|earth]
  camera mount <center|neutral|retract>
  camera set [NAME [VALUE]]            show/change module settings"""

    def _camera_key(self, system_id, component_id):
        return (int(system_id), int(component_id))

    @staticmethod
    def _is_gimbal_device_component(component_id):
        return (mavutil.mavlink.MAV_COMP_ID_GIMBAL <= component_id <=
                mavutil.mavlink.MAV_COMP_ID_GIMBAL6)

    def _ensure_camera(self, system_id, component_id):
        key = self._camera_key(system_id, component_id)
        camera = self.cameras.get(key)
        if camera is None:
            camera = CameraDevice(*key)
            self.cameras[key] = camera
        camera.last_seen = time.time()
        if self.selected_camera is None:
            self.selected_camera = key
        return camera

    def _ensure_gimbal(self, system_id, component_id):
        key = self._camera_key(system_id, component_id)
        gimbal = self.gimbals.get(key)
        if gimbal is None:
            gimbal = GimbalDevice(*key)
            self.gimbals[key] = gimbal
        gimbal.last_seen = time.time()
        if self.selected_gimbal is None:
            self.selected_gimbal = key
        return gimbal

    def _selected_camera(self, required=True):
        component = self.camera_settings.camera_component
        if component:
            key = (self.target_system or 1, component)
            camera = self.cameras.get(key)
            if camera is None:
                camera = self._ensure_camera(*key)
            return camera
        if self.selected_camera in self.cameras:
            return self.cameras[self.selected_camera]
        if required:
            print("No MAVLink camera discovered; use 'camera discover'")
        return None

    def _selected_gimbal(self, required=True):
        component = self.camera_settings.gimbal_component
        if component:
            key = (self.target_system or 1, component)
            return self.gimbals.get(key) or self._ensure_gimbal(*key)
        camera = self._selected_camera(required=False)
        if camera is not None and camera.information is not None:
            component = getattr(camera.information, "gimbal_device_id", 0)
            key = (camera.system_id, component)
            if self._is_gimbal_device_component(component) and key in self.gimbals:
                return self.gimbals[key]
        if self.selected_gimbal in self.gimbals:
            return self.gimbals[self.selected_gimbal]
        if required:
            print("No MAVLink gimbal discovered; use 'camera discover'")
        return None

    def _send_command(self, system_id, component_id, command, params=()):
        values = list(params) + [0.0] * (7 - len(params))
        key = (system_id, component_id, command)
        self.pending_commands[key] = self.pending_commands.get(key, 0) + 1
        self.master.mav.command_long_send(
            system_id, component_id, command, 0, *values[:7])

    def _request_message(self, system_id, component_id, message_id, instance=0):
        self._send_command(
            system_id, component_id, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            (message_id, instance))

    def discover(self):
        system_id = self.target_system or 1
        components = set(range(mavutil.mavlink.MAV_COMP_ID_CAMERA,
                               mavutil.mavlink.MAV_COMP_ID_CAMERA6 + 1))
        components.update((mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                           mavutil.mavlink.MAV_COMP_ID_GIMBAL2,
                           mavutil.mavlink.MAV_COMP_ID_GIMBAL3,
                           mavutil.mavlink.MAV_COMP_ID_GIMBAL4,
                           mavutil.mavlink.MAV_COMP_ID_GIMBAL5,
                           mavutil.mavlink.MAV_COMP_ID_GIMBAL6))
        for component in sorted(components):
            message_id = (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION
                          if component <= mavutil.mavlink.MAV_COMP_ID_CAMERA6
                          else mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION)
            self._request_message(system_id, component, message_id)
        self.last_discovery_request = time.time()

    def _request_camera_state(self, camera, full=False):
        requests = [
            (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0),
            (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_SETTINGS, 0),
            (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, 0),
            (mavutil.mavlink.MAVLINK_MSG_ID_STORAGE_INFORMATION, 0),
            (mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, 0),
        ]
        if not full:
            requests = requests[1:]
        for message_id, instance in requests:
            self._request_message(camera.system_id, camera.component_id,
                                  message_id, instance)
        camera.last_request = time.time()

    def _request_gimbal_state(self, gimbal):
        self._request_message(
            gimbal.system_id, gimbal.component_id,
            mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION)
        self._request_message(
            gimbal.system_id, gimbal.component_id,
            mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS)
        gimbal.last_request = time.time()

    def cmd_camera(self, args):
        if not args:
            print(self.usage())
            return
        command = args[0].lower()
        try:
            if command == "status":
                self.show_status()
            elif command == "discover":
                self.discover()
                print("MAVLink camera discovery requested")
            elif command == "select":
                self.cmd_select(args[1:])
            elif command == "info":
                self.show_info()
            elif command == "streams":
                self.show_streams()
            elif command == "view":
                self.cmd_view(args[1:])
            elif command == "projection":
                self.show_projection()
            elif command == "photo":
                self.cmd_photo(args[1:])
            elif command == "stopphotos":
                self.camera_command(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE)
            elif command == "record":
                self.cmd_record(args[1:])
            elif command == "zoom":
                self.cmd_zoom(args[1:])
            elif command == "focus":
                self.cmd_focus(args[1:])
            elif command == "mode":
                self.cmd_mode(args[1:])
            elif command == "source":
                self.cmd_source(args[1:])
            elif command == "stream":
                self.cmd_stream(args[1:])
            elif command == "mount":
                self.cmd_mount(args[1:])
            elif command == "set":
                self.camera_settings.command(args[1:])
            else:
                print(self.usage())
        except (TypeError, ValueError) as error:
            print("Camera command error: %s" % error)

    def cmd_select(self, args):
        if len(args) != 1:
            raise ValueError("usage: camera select [SYSID:]COMPID")
        if ":" in args[0]:
            system_text, component_text = args[0].split(":", 1)
            key = (int(system_text), int(component_text))
        else:
            key = (self.target_system or 1, int(args[0]))
        camera = self.cameras.get(key)
        if camera is None:
            camera = self._ensure_camera(*key)
            self._request_camera_state(camera, full=True)
        self.selected_camera = key
        print("Selected camera %s" % camera.label())

    def camera_command(self, command, params=()):
        camera = self._selected_camera()
        if camera is None:
            return
        self._send_command(camera.system_id, camera.component_id, command, params)

    def cmd_photo(self, args):
        if len(args) > 2:
            raise ValueError("usage: camera photo [INTERVAL [COUNT]]")
        interval = float(args[0]) if args else 0.0
        count = int(args[1]) if len(args) == 2 else 1
        if interval < 0 or count < 0:
            raise ValueError("interval and count must be non-negative")
        self.camera_command(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                            (0, interval, count, 0))

    def cmd_record(self, args):
        if len(args) != 1 or args[0].lower() not in ("start", "stop", "toggle"):
            raise ValueError("usage: camera record <start|stop|toggle>")
        action = args[0].lower()
        camera = self._selected_camera()
        if camera is None:
            return
        if action == "toggle":
            recording = camera.recording
            if recording is None:
                recording = (camera.capture_status is not None and
                             camera.capture_status.video_status != 0)
            action = "stop" if recording else "start"
        command = (mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE
                   if action == "start"
                   else mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE)
        self._send_command(camera.system_id, camera.component_id, command,
                           (0, 1.0 if action == "start" else 0.0))
        # Keep repeated toggles coherent before relayed status catches up,
        # then verify this optimistic state against the camera response.
        camera.recording = action == "start"
        camera.recording_verify_at = time.time() + 0.5
        self._set_console_status()

    def cmd_zoom(self, args):
        if len(args) != 1:
            raise ValueError("usage: camera zoom <PERCENT|in|out|stop>")
        value = args[0].lower()
        if value in ("in", "out", "stop"):
            rate = {"in": 1.0, "out": -1.0, "stop": 0.0}[value]
            params = (mavutil.mavlink.ZOOM_TYPE_CONTINUOUS, rate)
        else:
            percent = float(value)
            if percent < 0 or percent > 100:
                raise ValueError("zoom percentage must be 0 to 100")
            params = (mavutil.mavlink.ZOOM_TYPE_RANGE, percent)
        self.camera_command(mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM, params)

    def cmd_focus(self, args):
        if len(args) != 1:
            raise ValueError("usage: camera focus <auto|PERCENT|in|out|stop>")
        value = args[0].lower()
        if value == "auto":
            params = (mavutil.mavlink.FOCUS_TYPE_AUTO, 0.0)
        elif value in ("in", "out", "stop"):
            rate = {"in": -1.0, "out": 1.0, "stop": 0.0}[value]
            params = (mavutil.mavlink.FOCUS_TYPE_CONTINUOUS, rate)
        else:
            percent = float(value)
            if percent < 0 or percent > 100:
                raise ValueError("focus percentage must be 0 to 100")
            params = (mavutil.mavlink.FOCUS_TYPE_RANGE, percent)
        self.camera_command(mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS, params)

    def cmd_mode(self, args):
        if len(args) != 1 or args[0].lower() not in ("photo", "video"):
            raise ValueError("usage: camera mode <photo|video>")
        mode = (mavutil.mavlink.CAMERA_MODE_IMAGE
                if args[0].lower() == "photo"
                else mavutil.mavlink.CAMERA_MODE_VIDEO)
        self.camera_command(mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE, (0, mode))

    def cmd_source(self, args):
        if len(args) != 1 or args[0].lower() not in ("rgb", "thermal"):
            raise ValueError("usage: camera source <rgb|thermal>")
        source = (mavutil.mavlink.CAMERA_SOURCE_RGB
                  if args[0].lower() == "rgb"
                  else mavutil.mavlink.CAMERA_SOURCE_IR)
        self.camera_command(mavutil.mavlink.MAV_CMD_SET_CAMERA_SOURCE,
                            (0, source, 0))

    def cmd_stream(self, args):
        if not args or len(args) > 2 or args[0].lower() not in ("start", "stop"):
            raise ValueError("usage: camera stream <start|stop> [ID]")
        stream_id = int(args[1]) if len(args) == 2 else 0
        command = (mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING
                   if args[0].lower() == "start"
                   else mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING)
        self.camera_command(command, (stream_id,))

    def _manager_command(self, pitch=math.nan, yaw=math.nan,
                         pitch_rate=math.nan, yaw_rate=math.nan, flags=0):
        # Manager commands address the vehicle's gimbal manager, not the
        # discovered device.  Requiring a device here also made manager
        # control silently depend on discovery of forwarded device messages.
        system_id = self.target_system or 1
        self._send_command(
            system_id, self.camera_settings.manager_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            (pitch, yaw, pitch_rate, yaw_rate, flags, 0,
             self.camera_settings.manager_gimbal_id))

    def _device_command(self, pitch=math.nan, yaw=math.nan,
                        pitch_rate=math.nan, yaw_rate=math.nan, flags=0):
        gimbal = self._selected_gimbal()
        if gimbal is None:
            return
        if math.isfinite(pitch) or math.isfinite(yaw):
            q = Quaternion([0.0, math.radians(pitch if math.isfinite(pitch) else 0),
                            math.radians(yaw if math.isfinite(yaw) else 0)]).q
        else:
            q = [math.nan] * 4
        self.master.mav.gimbal_device_set_attitude_send(
            gimbal.system_id, gimbal.component_id, flags, q,
            math.nan,
            math.radians(pitch_rate) if math.isfinite(pitch_rate) else math.nan,
            math.radians(yaw_rate) if math.isfinite(yaw_rate) else math.nan)

    def _mount_command(self, **kwargs):
        mode = self.camera_settings.mount_control.lower()
        if mode == "manager":
            self._manager_command(**kwargs)
        elif mode == "device":
            self._device_command(**kwargs)
        else:
            raise ValueError("mount_control must be manager or device")

    def cmd_mount(self, args):
        if not args:
            raise ValueError("usage: camera mount <info|angle|rate|center|neutral|retract>")
        action = args[0].lower()
        if action == "info":
            self.show_gimbal_info()
            return
        if action in ("center", "neutral", "retract"):
            flag = (mavutil.mavlink.GIMBAL_MANAGER_FLAGS_RETRACT
                    if action == "retract"
                    else mavutil.mavlink.GIMBAL_MANAGER_FLAGS_NEUTRAL)
            self._mount_command(flags=flag)
            return
        if action not in ("angle", "rate") or len(args) not in (3, 4):
            raise ValueError("usage: camera mount <angle|rate> PITCH YAW [body|earth]")
        frame = args[3].lower() if len(args) == 4 else "body"
        if frame not in ("body", "earth"):
            raise ValueError("mount frame must be body or earth")
        if frame == "earth" and self.camera_settings.mount_control.lower() == "device":
            raise ValueError("earth frame requires mount_control=manager")
        flags = (mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK
                 if frame == "earth" else 0)
        first, second = float(args[1]), float(args[2])
        if action == "angle":
            self._mount_command(pitch=first, yaw=second, flags=flags)
        else:
            self._mount_command(pitch_rate=first, yaw_rate=second, flags=flags)

    def _stream_matches(self, stream, selector):
        if selector == "rgb":
            return not bool(stream.flags &
                            mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
        if selector == "thermal":
            return bool(stream.flags &
                        mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
        return stream.stream_id == int(selector)

    def _resolved_uri(self, camera, stream):
        uri = _text(stream.uri)
        parsed = urlsplit(uri)
        host = parsed.hostname
        replacement = self.camera_settings.rtsp_host.strip()
        if not replacement and host in ("0.0.0.0", "::", ""):
            definition_uri = ""
            if camera.information is not None:
                definition_uri = _text(camera.information.cam_definition_uri)
            replacement = urlsplit(definition_uri).hostname or ""
        if replacement and host in ("0.0.0.0", "::", ""):
            netloc = replacement
            if parsed.port is not None:
                netloc += ":%u" % parsed.port
            uri = urlunsplit((parsed.scheme, netloc, parsed.path,
                              parsed.query, parsed.fragment))
        return uri

    def _clear_fov(self, keep=None):
        keep = set() if keep is None else set(keep)
        map_display = getattr(self.mpstate, "map", None)
        for name in self.fov_objects - keep:
            if map_display is not None:
                map_display.remove_object(name)
            self.fov_points.pop(name, None)
        self.fov_objects.intersection_update(keep)

    def _fov_attitude(self):
        """Return camera roll, pitch and earth-frame yaw in degrees."""
        gimbal = self._selected_gimbal(required=False)
        system_id = self.target_system or 1
        candidates = []
        if gimbal is not None and gimbal.attitude is not None:
            candidates.append(gimbal.attitude)
            system_id = gimbal.system_id
        manager_id = self.camera_settings.manager_gimbal_id or 1
        manager_attitude = self.manager_attitudes.get((system_id, manager_id))
        if manager_attitude is not None:
            candidates.append(manager_attitude)
        if not candidates:
            return None
        attitude = max(candidates,
                       key=lambda value: getattr(value, "_timestamp", 0.0))
        if time.time() - getattr(attitude, "_timestamp", time.time()) > 2.0:
            return None
        try:
            roll, pitch, yaw = _quaternion_to_euler(attitude.q)
        except (AttributeError, TypeError, ValueError):
            return None

        # Extrapolate over the short interval between 5Hz device reports.
        attitude_timestamp = getattr(attitude, "_timestamp", time.time())
        dt = max(0.0, min(0.5, time.time() - attitude_timestamp))
        rates = (getattr(attitude, "angular_velocity_x", math.nan),
                 getattr(attitude, "angular_velocity_y", math.nan),
                 getattr(attitude, "angular_velocity_z", math.nan))
        angles = [roll, pitch, yaw]
        for index, rate in enumerate(rates):
            if math.isfinite(rate):
                angles[index] += rate * dt
        roll, pitch, yaw = angles

        flags = getattr(attitude, "flags", 0)
        explicit_frame = flags & (
            mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME |
            mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME)
        earth_frame = bool(
            flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME or
            (not explicit_frame and
             flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK))
        if not earth_frame:
            vehicle_attitude = self.master.messages.get("ATTITUDE")
            if vehicle_attitude is None:
                return None
            vehicle_dt = max(0.0, min(
                0.5, time.time() -
                getattr(vehicle_attitude, "_timestamp", time.time())))
            yaw += (vehicle_attitude.yaw +
                    getattr(vehicle_attitude, "yawspeed", 0.0) * vehicle_dt)

        return (math.degrees(roll) - self.camera_settings.mount_roll,
                math.degrees(pitch) - self.camera_settings.mount_pitch,
                mp_util.wrap_180(math.degrees(yaw) -
                                 self.camera_settings.mount_yaw))

    def _show_fov(self, position):
        """Project all selected-camera stream footprints onto the map."""
        if not mp_util.has_wxpython:
            return
        map_display = getattr(self.mpstate, "map", None)
        if map_display is None or not self.camera_settings.show_fov:
            self._clear_fov()
            return
        now = time.time()
        interval = max(0.0, self.camera_settings.fov_update_interval)
        if now - self.last_fov_update < interval:
            return
        self.last_fov_update = now

        camera = self._selected_camera(required=False)
        attitude = self._fov_attitude()
        if camera is None or attitude is None or not camera.streams:
            self._clear_fov()
            return
        try:
            latitude = position.lat * 1.0e-7
            longitude = position.lon * 1.0e-7
            altitude = position.alt * 1.0e-3 + self.camera_settings.mount_alt
            ground_altitude = ((position.alt - position.relative_alt) *
                               1.0e-3)
        except AttributeError:
            return

        terrain = self.module("terrain")
        elevation_model = getattr(terrain, "ElevationModel", None)
        if elevation_model is None:
            elevation_model = _FlatElevationModel(ground_altitude)

        active = set()
        for stream_id in sorted(camera.streams):
            stream = camera.streams[stream_id]
            width = int(getattr(stream, "resolution_h", 0))
            height = int(getattr(stream, "resolution_v", 0))
            hfov = float(getattr(stream, "hfov", 0))
            if width <= 0 or height <= 0 or not 0.0 < hfov < 180.0:
                continue
            name = "CameraFOV_%u_%u_%u" % (
                camera.system_id, camera.component_id, stream_id)
            try:
                params = camera_projection.CameraParams(
                    xresolution=width, yresolution=height, FOV=hfov)
                projection = camera_projection.CameraProjection(
                    params, elevation_model=elevation_model)
                points = projection.get_projection(
                    latitude, longitude, altitude, *attitude,
                    max_range=self.camera_settings.fov_max_range)
            except Exception:
                # Terrain tiles may be temporarily unavailable.  Keep MAVLink
                # packet processing alive and retry on the next position.
                points = None
            if points is None:
                if name in self.fov_objects:
                    map_display.remove_object(name)
                    self.fov_objects.discard(name)
                    self.fov_points.pop(name, None)
                continue
            thermal = bool(
                stream.flags &
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
            colour = (0, 0, 128) if thermal else (0, 128, 128)
            map_display.add_object(mp_slipmap.SlipPolygon(
                name, points, layer="Camera", linewidth=2, colour=colour,
                showcircles=False))
            self.fov_points[name] = points
            active.add(name)
        self.fov_objects.update(active)
        self._clear_fov(keep=active)

    def show_projection(self):
        map_display = getattr(self.mpstate, "map", None)
        camera = self._selected_camera(required=False)
        position = self.master.messages.get("GLOBAL_POSITION_INT")
        attitude = self._fov_attitude()
        print("Camera projection: enabled=%s map=%s camera=%s" % (
            self.camera_settings.show_fov,
            "loaded" if map_display is not None else "not loaded",
            camera.label() if camera is not None else "none"))
        if camera is not None:
            print(" streams: %s" % ", ".join(
                "%u=%ux%u/%.1fdeg" %
                (stream.stream_id, stream.resolution_h, stream.resolution_v,
                 stream.hfov)
                for stream in camera.streams.values()))
        if attitude is None:
            print(" attitude: unavailable (device=%s manager=%s)" % (
                self._selected_gimbal(required=False) is not None,
                sorted(self.manager_attitudes)))
        else:
            print(" attitude: roll=%.1f pitch=%.1f earth-yaw=%.1f" % attitude)
        if position is None:
            print(" position: unavailable")
            return
        print(" position: %.7f %.7f AMSL=%.1fm relative=%.1fm" % (
            position.lat * 1.0e-7, position.lon * 1.0e-7,
            position.alt * 1.0e-3, position.relative_alt * 1.0e-3))
        self.last_fov_update = 0.0
        self._show_fov(position)
        if not self.fov_points:
            print(" polygons: none (view does not intersect terrain)")
            return
        for name, points in sorted(self.fov_points.items()):
            print(" %s: lat %.7f..%.7f lon %.7f..%.7f" % (
                name, min(point[0] for point in points),
                max(point[0] for point in points),
                min(point[1] for point in points),
                max(point[1] for point in points)))

    def cmd_view(self, args):
        if len(args) != 1:
            raise ValueError("usage: camera view <ID|rgb|thermal|all>")
        if not mp_util.has_wxpython:
            print("Camera video viewing requires wxPython and an RTSP-capable OpenCV backend")
            return
        camera = self._selected_camera()
        if camera is None:
            return
        selector = args[0].lower()
        streams = [s for s in camera.streams.values()
                   if selector == "all" or self._stream_matches(s, selector)]
        if not streams:
            print("No matching stream discovered; use 'camera streams'")
            return
        from MAVProxy.modules.mavproxy_camera.video_view import VideoView
        for stream in streams:
            key = (camera.system_id, camera.component_id, stream.stream_id)
            if key in self.views and self.views[key].alive():
                continue
            uri = self._resolved_uri(camera, stream)
            if not uri:
                print("Stream %u has no URI" % stream.stream_id)
                continue
            self.views[key] = VideoView(
                self, camera, stream, uri, self.camera_settings.rtsp_latency)

    def show_status(self):
        if not self.cameras and not self.gimbals:
            print("No MAVLink cameras or gimbals discovered")
            return
        print("Cameras:")
        for key in sorted(self.cameras):
            marker = "*" if key == self.selected_camera else " "
            camera = self.cameras[key]
            print(" %s %s streams=%u" %
                  (marker, camera.label(), len(camera.streams)))
        print("Gimbals:")
        for key in sorted(self.gimbals):
            marker = "*" if key == self.selected_gimbal else " "
            print(" %s %s" % (marker, self.gimbals[key].label()))

    def show_info(self):
        camera = self._selected_camera()
        if camera is None:
            return
        info = camera.information
        if info is None:
            print("Camera information pending for %u:%u" %
                  (camera.system_id, camera.component_id))
            return
        print("%s firmware=%s resolution=%ux%u flags=0x%x gimbal=%u" %
              (camera.label(), _firmware_version(info.firmware_version),
               info.resolution_h, info.resolution_v, info.flags,
               getattr(info, "gimbal_device_id", 0)))
        if _text(info.cam_definition_uri):
            print(" definition: %s (version %u)" %
                  (_text(info.cam_definition_uri), info.cam_definition_version))
        if camera.settings is not None:
            print(" mode=%u zoom=%.1f%% focus=%.1f%%" %
                  (camera.settings.mode_id, camera.settings.zoomLevel,
                   camera.settings.focusLevel))
        if camera.capture_status is not None:
            recording = camera.recording
            if recording is None:
                recording = camera.capture_status.video_status != 0
            print(" recording=%s photos=%u available=%.1f MiB" %
                  ("yes" if recording else "no",
                   camera.capture_status.image_count,
                   camera.capture_status.available_capacity))

    def show_streams(self):
        camera = self._selected_camera()
        if camera is None:
            return
        if not camera.streams:
            print("No video streams reported yet")
            self._request_message(
                camera.system_id, camera.component_id,
                mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, 0)
            return
        for stream_id in sorted(camera.streams):
            stream = camera.streams[stream_id]
            thermal = bool(stream.flags &
                           mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
            print("%u: %s %ux%u %.1fHz %s %s%s" %
                  (stream.stream_id, _text(stream.name), stream.resolution_h,
                   stream.resolution_v, stream.framerate,
                   "H.265" if stream.encoding == 2 else "H.264",
                   self._resolved_uri(camera, stream),
                   " [thermal]" if thermal else ""))

    def show_gimbal_info(self):
        gimbal = self._selected_gimbal()
        if gimbal is None:
            return
        if gimbal.information is None:
            print("Gimbal information pending for %u:%u" %
                  (gimbal.system_id, gimbal.component_id))
            self._request_gimbal_state(gimbal)
            return
        info = gimbal.information
        print("%s firmware=%s pitch=%.1f..%.1f yaw=%.1f..%.1f flags=0x%x" %
              (gimbal.label(), _firmware_version(info.firmware_version),
               math.degrees(info.pitch_min), math.degrees(info.pitch_max),
               math.degrees(info.yaw_min), math.degrees(info.yaw_max),
               getattr(info, "cap_flags2", 0) or info.cap_flags))
        if gimbal.attitude is not None:
            roll, pitch, yaw = _quaternion_to_euler(gimbal.attitude.q)
            print(" attitude roll=%.1f pitch=%.1f yaw=%.1f" %
                  tuple(math.degrees(value) for value in (roll, pitch, yaw)))

    def _set_console_status(self):
        camera = self._selected_camera(required=False)
        if camera is None:
            text = "CAMERA --"
        elif camera.information is None:
            text = "CAMERA %u:%u" % (camera.system_id, camera.component_id)
        else:
            recording = camera.recording
            if recording is None:
                recording = (camera.capture_status is not None and
                             camera.capture_status.video_status != 0)
            text = "CAMERA %s %s" % (
                _text(camera.information.model_name),
                "REC" if recording else "READY")
        self.console.set_status("CAMERA", text, row=6)

    def mavlink_packet(self, message):
        message_type = message.get_type()
        system_id = message.get_srcSystem()
        component_id = message.get_srcComponent()
        if message_type == "HEARTBEAT":
            if message.type == mavutil.mavlink.MAV_TYPE_CAMERA:
                camera = self._ensure_camera(system_id, component_id)
                if camera.information is None and time.time() - camera.last_request > 1:
                    self._request_camera_state(camera, full=True)
            elif message.type == mavutil.mavlink.MAV_TYPE_GIMBAL:
                gimbal = self._ensure_gimbal(system_id, component_id)
                if gimbal.information is None and time.time() - gimbal.last_request > 1:
                    self._request_gimbal_state(gimbal)
            return
        if message_type == "CAMERA_INFORMATION":
            camera = self._ensure_camera(system_id, component_id)
            camera.information = message
            gimbal_component = getattr(message, "gimbal_device_id", 0)
            if self._is_gimbal_device_component(gimbal_component):
                self._ensure_gimbal(system_id, gimbal_component)
            self._request_camera_state(camera)
            self._set_console_status()
        elif message_type == "CAMERA_SETTINGS":
            self._ensure_camera(system_id, component_id).settings = message
        elif message_type == "CAMERA_CAPTURE_STATUS":
            camera = self._ensure_camera(system_id, component_id)
            camera.capture_status = message
            reported_recording = message.video_status != 0
            # A status packet already queued before the command may arrive
            # just after it.  Preserve the optimistic command state during
            # the short transition window unless the report confirms it.
            if not (camera.recording_verify_at > time.time() and
                    camera.recording is not None and
                    reported_recording != camera.recording):
                camera.recording = reported_recording
                camera.recording_verify_at = 0.0
            self._set_console_status()
        elif message_type == "STORAGE_INFORMATION":
            camera = self._ensure_camera(system_id, component_id)
            camera.storage[message.storage_id] = message
        elif message_type == "VIDEO_STREAM_INFORMATION":
            camera = self._ensure_camera(system_id, component_id)
            camera.streams[message.stream_id] = message
            for stream_id in range(1, message.count + 1):
                if stream_id not in camera.streams:
                    self._request_message(
                        system_id, component_id,
                        mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
                        stream_id)
        elif message_type == "VIDEO_STREAM_STATUS":
            camera = self._ensure_camera(system_id, component_id)
            stream = camera.streams.get(message.stream_id)
            if stream is not None:
                stream.flags = message.flags
        elif message_type == "CAMERA_IMAGE_CAPTURED":
            result = "captured" if message.capture_result == 1 else "failed"
            print("Camera %u image %u %s%s" %
                  (component_id, message.image_index, result,
                   " at %s" % _text(message.file_url)
                   if _text(message.file_url) else ""))
        elif message_type == "GIMBAL_DEVICE_INFORMATION":
            if self._is_gimbal_device_component(component_id):
                gimbal = self._ensure_gimbal(system_id, component_id)
                gimbal.information = message
        elif message_type == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            # ArduPilot republishes mount status from its own component.  It
            # is a manager view, not a second gimbal device, but is useful for
            # projection when targeted device status is consumed locally and
            # therefore not routed onward to the GCS.
            if self._is_gimbal_device_component(component_id):
                self._ensure_gimbal(system_id, component_id).attitude = message
            elif component_id == self.camera_settings.manager_component:
                gimbal_id = getattr(message, "gimbal_device_id", 0)
                if gimbal_id:
                    self.manager_attitudes[(system_id, gimbal_id)] = message
        elif message_type == "GLOBAL_POSITION_INT":
            self._show_fov(message)
        elif message_type == "COMMAND_ACK":
            key = (system_id, component_id, message.command)
            pending = self.pending_commands.get(key, 0)
            if pending == 0:
                return
            self.last_ack[key] = message
            if message.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                if pending == 1:
                    del self.pending_commands[key]
                else:
                    self.pending_commands[key] = pending - 1
            if message.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                      mavutil.mavlink.MAV_RESULT_IN_PROGRESS) and not (
                    message.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE):
                print("Camera command %s rejected by %u:%u (%s)" %
                      (_enum_name("MAV_CMD", message.command),
                       system_id, component_id,
                       _enum_name("MAV_RESULT", message.result)))

    def idle_task(self):
        now = time.time()
        if self.menu is not None:
            for name in ("console", "map"):
                module = self.module(name)
                if module is not None and name not in self.menu_modules:
                    module.add_menu(self.menu)
                    self.menu_modules.add(name)
        for key, view in list(self.views.items()):
            view.check_events()
            if not view.alive():
                del self.views[key]
        if self.last_discovery_request == 0:
            self.discover()
        elif now - self.last_discovery_request >= self.camera_settings.request_interval:
            for camera in self.cameras.values():
                if camera.information is None and now - camera.last_request >= 1:
                    self._request_camera_state(camera, full=True)
            for gimbal in self.gimbals.values():
                if gimbal.information is None and now - gimbal.last_request >= 1:
                    self._request_gimbal_state(gimbal)
            self.last_discovery_request = now
        for camera in self.cameras.values():
            if (camera.recording_verify_at != 0.0 and
                    now >= camera.recording_verify_at):
                self._request_message(
                    camera.system_id, camera.component_id,
                    mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS)
                camera.recording_verify_at = 0.0
        if now - self.last_status_request >= self.camera_settings.status_interval:
            camera = self._selected_camera(required=False)
            if camera is not None and camera.information is not None:
                self._request_camera_state(camera)
            self.last_status_request = now


def init(mpstate):
    """Initialise the MAVProxy camera module."""
    return CameraModule(mpstate)
