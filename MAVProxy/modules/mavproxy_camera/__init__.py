#!/usr/bin/env python3
"""Generic MAVLink Camera Protocol v2 and Gimbal Protocol v2 control."""

import math
import colorsys
from contextlib import contextmanager
from copy import copy
import time
from urllib.parse import urlsplit, urlunsplit

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import camera_projection
from MAVProxy.modules.mavproxy_camera.parameters import CameraParameters
from MAVProxy.modules.mavproxy_camera.graphs import CameraGraphs, PRESETS
from MAVProxy.modules.mavproxy_camera.roi import CameraROI, gimbal_capabilities
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
        self.parameters = None
        self.control_overrides = {}
        self.fov_objects = set()
        self.last_fov_update = 0.0

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
            mpstate, "camera", "MAVLink camera and gimbal control", public=True, multi_vehicle=True)
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
            ["<status|discover|select|for|info|custom|definition|params|param|streams|view|projection|graph|roi|photo|stopphotos|record|zoom|focus|mode|source|stream|mount|set>",
             "roi <all|clear>",
             "projection <toggle>",
             "for (CAMERAADDRESS)",
             "graph <%s|close>" % "|".join(p[0] for p in PRESETS),
             "set (CAMERASETTING)"])
        self.add_completion_function("(CAMERASETTING)",
                                     self.camera_settings.completion)
        self.add_completion_function("(CAMERAADDRESS)",
                                     lambda text: ["%u:%u" % key for key in self._camera_menu_keys()])
        self.cameras = {}
        self.gimbals = {}
        self.manager_attitudes = {}
        self.selected_camera = None
        self.camera_selection_explicit = False
        self.selected_gimbal = None
        self.command_camera = None
        self.views = {}
        self.graphs = CameraGraphs(self, _quaternion_to_euler)
        self.roi = CameraROI(self)
        self.last_status_request = 0.0
        self.last_discovery_request = 0.0
        self.vehicle_messages = {}
        self.fov_objects = set()
        self.fov_points = {}
        self.last_ack = {}
        self.pending_commands = {}
        self.menu_cameras = []
        self.menus = [self._make_menu("Camera", None)] if mp_util.has_wxpython else []
        self.menu = self.menus[0] if self.menus else None
        self.menu_modules = {}
        self.camera_status_names = set()

    def _make_menu(self, name, key):
        from MAVProxy.modules.lib.mp_menu import MPMenuItem, MPMenuSubMenu
        prefix = "# camera " if key is None else "# camera for %u:%u " % key
        return MPMenuSubMenu(name, items=[
            MPMenuItem("Status" if key is None else "Info (%u:%u)" % key,
                       returnkey=prefix + ("status" if key is None else "info")),
            MPMenuItem("Discover", returnkey=prefix + "discover"),
            MPMenuItem("Custom Settings", returnkey=prefix + "custom"),
            MPMenuItem("Toggle Projection", returnkey=prefix + "projection toggle"),
            MPMenuSubMenu("Graphs", items=[
                MPMenuItem(title, returnkey=prefix + "graph " + graph_name)
                for graph_name, title, _message_type, _fields in PRESETS
            ]),
            MPMenuItem("Take photo", returnkey=prefix + "photo"),
            MPMenuItem("Toggle recording", returnkey=prefix + "record toggle"),
            MPMenuItem("Autofocus", returnkey=prefix + "focus auto"),
            MPMenuItem("Center gimbal", returnkey=prefix + "mount center"),
            MPMenuItem("View RGB", returnkey=prefix + "view rgb"),
            MPMenuItem("View thermal", returnkey=prefix + "view thermal"),
        ])

    def _camera_menu_keys(self):
        # Only collapse autopilot proxies, never two physical cameras with
        # identical vendor/model names. Number by MAVLink address so component
        # 101 cannot become Camera just because its heartbeat arrived first.
        replacements = {}
        for key, camera in self.cameras.items():
            if key[1] != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                continue
            for other_key, other in sorted(self.cameras.items()):
                if (other_key[0] == key[0] and
                        mavutil.mavlink.MAV_COMP_ID_CAMERA <= other_key[1] <=
                        mavutil.mavlink.MAV_COMP_ID_CAMERA6 and
                        camera.information is not None and other.information is not None and
                        all(_text(getattr(camera.information, field)) ==
                            _text(getattr(other.information, field))
                            for field in ('vendor_name', 'model_name', 'cam_definition_uri'))):
                    replacements[key] = other_key
                    break
        return sorted({replacements.get(key, key) for key in self.cameras})

    def _sync_menus(self):
        if not mp_util.has_wxpython:
            return
        keys = self._camera_menu_keys()
        changed = keys != self.menu_cameras
        old_menus = self.menus
        if changed:
            # A late lower-address camera can change the numbered map layers.
            # Remove old names before refreshing their new names and colours.
            self._clear_fov()
            self.menu_cameras = keys
            self.menus = [self._make_menu("Camera" if i == 0 else "Camera%u" % (i + 1), key)
                          for i, key in enumerate(keys or [None])]
            self.menu = self.menus[0]
            self._refresh_fov()
            self._set_console_status()
        for name in ("console", "map"):
            module = self.module(name)
            previous = self.menu_modules.get(name)
            if module is None:
                self.menu_modules.pop(name, None)
                continue
            if changed and module is previous:
                for menu in old_menus:
                    module.remove_menu(menu)
            if changed or module is not previous:
                for menu in self.menus:
                    module.add_menu(menu)
                self.menu_modules[name] = module

    def unload(self):
        self.roi.close()
        self.remove_command("camera")
        self._clear_fov()
        for view in self.views.values():
            view.close()
        self.views.clear()
        self.graphs.close()
        for camera in self.cameras.values():
            camera.parameters.close()
        for name, module in self.menu_modules.items():
            if self.module(name) is module:
                for menu in self.menus:
                    module.remove_menu(menu)
        super(CameraModule, self).unload()

    def usage(self):
        return """Usage:
  camera status                       show discovered cameras and gimbals
  camera discover                     request fresh discovery information
  camera select [SYSID:]COMPID         select a camera
  camera for SYSID:COMPID COMMAND      control one camera without changing selection
  camera info                          show selected camera information
  camera custom                       open live Custom Settings dialog
  camera definition [FILE|URL]         reload or override camera definition
  camera params                       show custom parameter values
  camera param NAME VALUE             set a custom camera parameter
  camera streams                       show discovered video streams
  camera view <ID|rgb|thermal|all>     open RTSP viewer(s)
  camera projection [toggle]           show/refresh or toggle this camera's projection
  camera graph [NAME|close]            list/open graphs or close camera graphs
  camera roi [all|clear]               point camera(s) at map click or stop tracking
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
            camera.parameters = CameraParameters(self, camera)
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
        if self.command_camera is not None:
            return self.command_camera
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

    def _prefer_camera_component(self):
        """Prefer a camera over the autopilot's duplicate camera advertisement.

        ArduPilot publishes CAMERA_INFORMATION from component 1, but does not
        serve the camera's PARAM_EXT parameters there. Keep explicitly chosen
        targets and only replace an automatic choice with a matching camera.
        """
        if self.camera_selection_explicit or self.camera_settings.camera_component:
            return
        selected = self.cameras.get(self.selected_camera)
        if (selected is None or selected.information is None or
                selected.component_id != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
            return
        info = selected.information
        for key, camera in self.cameras.items():
            if (camera.system_id != selected.system_id or
                    not mavutil.mavlink.MAV_COMP_ID_CAMERA <= camera.component_id <=
                    mavutil.mavlink.MAV_COMP_ID_CAMERA6 or camera.information is None):
                continue
            if any(_text(getattr(info, field)) !=
                   _text(getattr(camera.information, field))
                   for field in ('vendor_name', 'model_name', 'cam_definition_uri')):
                continue
            self.selected_camera = key
            print("Camera: selected %s instead of autopilot camera proxy %u:%u" %
                  (camera.label(), selected.system_id, selected.component_id))
            # A dialog requested before camera discovery must follow this
            # correction, rather than remain bound to the unresponsive proxy.
            previous = selected.parameters
            if previous.dialog is not None or previous.open_when_ready:
                if previous.dialog is not None:
                    previous.dialog.close()
                    previous.dialog = None
                previous.open_when_ready = False
                camera.parameters.open_dialog()
            return

    def _selected_gimbal(self, required=True):
        if self.command_camera is not None:
            camera = self.command_camera
            component = (self.camera_settings.gimbal_component or
                         getattr(camera.information, "gimbal_device_id", 0))
            if self._is_gimbal_device_component(component):
                return self._ensure_gimbal(camera.system_id, component)
            # Never fall back to another camera's gimbal. A sole camera may
            # use a sole device on its own system when no association is sent.
            siblings = [key for key in self._camera_menu_keys() if key[0] == camera.system_id]
            devices = [g for g in self.gimbals.values() if g.system_id == camera.system_id]
            if component == 0 and len(siblings) == 1 and len(devices) == 1:
                return devices[0]
            if required:
                print("No associated MAVLink gimbal for camera %u:%u" %
                      (camera.system_id, camera.component_id))
            return None
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

    @contextmanager
    def _camera_context(self, camera):
        """Resolve a synchronous command against one camera, then restore defaults."""
        previous_camera, previous_settings = self.command_camera, self.camera_settings
        settings = mp_settings.MPSettings([
            copy(previous_settings.get_setting(name)) for name in previous_settings.list()])
        for name, value in camera.control_overrides.items():
            settings.set(name, value)
        self.command_camera, self.camera_settings = camera, settings
        try:
            yield
        finally:
            self.command_camera, self.camera_settings = previous_camera, previous_settings

    def cmd_for(self, args):
        if len(args) < 2 or args[1].lower() in ("for", "select"):
            raise ValueError("usage: camera for SYSID:COMPID COMMAND [ARGS]")
        parts = args[0].split(":")
        if len(parts) != 2:
            raise ValueError("camera address must be SYSID:COMPID")
        key = tuple(int(value) for value in parts)
        camera = self.cameras.get(key)
        if camera is None:
            raise ValueError("camera %s has not been discovered" % args[0])
        with self._camera_context(camera):
            self.cmd_camera(args[1:])

    def cmd_camera(self, args):
        if not args:
            print(self.usage())
            return
        command = args[0].lower()
        try:
            if command == "for":
                self.cmd_for(args[1:])
            elif command == "status":
                if self.command_camera is not None:
                    self.show_info()
                else:
                    self.show_status()
            elif command == "discover":
                if self.command_camera is not None:
                    self._request_camera_state(self.command_camera, full=True)
                else:
                    self.discover()
                print("MAVLink camera discovery requested")
            elif command == "select":
                self.cmd_select(args[1:])
            elif command == "info":
                self.show_info()
            elif command in ("custom", "definition", "params", "param"):
                self.cmd_custom(command, args[1:])
            elif command == "streams":
                self.show_streams()
            elif command == "view":
                self.cmd_view(args[1:])
            elif command == "projection":
                self.cmd_projection(args[1:])
            elif command == "graph":
                self.graphs.open(args[1:])
            elif command == "roi":
                self.roi.command(args[1:])
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
                if (self.command_camera is not None and len(args) == 3 and
                        args[1] in self.camera_settings.list()):
                    if self.camera_settings.set(args[1], args[2]):
                        self.command_camera.control_overrides[args[1]] = self.camera_settings.get(args[1])
                else:
                    self.camera_settings.command(args[1:])
                if len(args) >= 3 and args[1] == "show_fov":
                    self._refresh_fov()
            else:
                print(self.usage())
        except (TypeError, ValueError) as error:
            print("Camera command error: %s" % error)

    def cmd_custom(self, command, args):
        camera = self._selected_camera()
        if camera is None:
            return
        parameters = camera.parameters
        if (command != "definition" and parameters.identity is None and
                camera.information is not None):
            parameters.information(camera.information)
        if command == "custom":
            parameters.open_dialog()
            if camera.information is None:
                self._request_camera_state(camera, full=True)
        elif command == "definition":
            if args:
                uri = args[0]
                parameters.load(uri, local="://" not in uri)
            elif parameters.identity:
                uri, version = parameters.identity
                parameters.load(uri, version, local="://" not in uri)
            else:
                self._request_camera_state(camera, full=True)
        elif command == "param":
            if len(args) != 2:
                raise ValueError("usage: camera param NAME VALUE")
            parameters.set_value(*args)
        elif parameters.definition is None:
            print(parameters.status)
        else:
            for name, param in parameters.definition.parameters.items():
                print("%16s %-20s %s%s" % (
                    name, parameters.values.get(name, "(unread)"), param.description,
                    " [pending]" if name in parameters.pending else ""))

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
        self.camera_selection_explicit = True
        if camera.information is not None and camera.parameters.identity is None:
            camera.parameters.information(camera.information)
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

    def _manager_id(self):
        camera = self.command_camera
        if (self.camera_settings.manager_gimbal_id != 0 or
                (camera is not None and "manager_gimbal_id" in camera.control_overrides)):
            return self.camera_settings.manager_gimbal_id
        if camera is not None:
            association = (self.camera_settings.gimbal_component or
                           getattr(camera.information, "gimbal_device_id", 0))
            if 1 <= association <= 6:
                return association
            heartbeat = self._vehicle_message(
                "HEARTBEAT", camera.system_id, self.camera_settings.manager_component)
            if (heartbeat is not None and getattr(heartbeat, "autopilot", None) ==
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
                # AP_Mount_MAVLink::find_gimbal binds MNT1 to component 154,
                # MNT2 to 171, etc. ArduPilot republishes their attitudes with
                # IDs 1, 2, ... even when targeted device status is not routed
                # to the GCS. This association is independent of discovery order.
                components = [mavutil.mavlink.MAV_COMP_ID_GIMBAL] + list(range(
                    mavutil.mavlink.MAV_COMP_ID_GIMBAL2,
                    mavutil.mavlink.MAV_COMP_ID_GIMBAL6 + 1))
                if association in components:
                    return components.index(association) + 1
            siblings = [self.cameras[key] for key in self._camera_menu_keys()
                        if key[0] == camera.system_id]
            associations = {getattr(c.information, "gimbal_device_id", 0) for c in siblings}
            if len(siblings) > 1 and (len(associations) != 1 or 0 in associations):
                # ArduPilot mount instance numbers cannot be inferred from
                # MAVLink device component IDs or camera discovery order.
                raise ValueError("set 'camera for %u:%u set manager_gimbal_id N' "
                                 "to this camera's ArduPilot mount number (1, 2, ...)" %
                                 (camera.system_id, camera.component_id))
        return self.camera_settings.manager_gimbal_id

    def _manager_command(self, pitch=math.nan, yaw=math.nan,
                         pitch_rate=math.nan, yaw_rate=math.nan, flags=0):
        # Manager commands address the vehicle's gimbal manager, not the
        # discovered device.  Requiring a device here also made manager
        # control silently depend on discovery of forwarded device messages.
        system_id = (self.command_camera.system_id if self.command_camera is not None
                     else self.target_system or 1)
        self._send_command(
            system_id, self.camera_settings.manager_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            (pitch, yaw, pitch_rate, yaw_rate, flags, 0,
             self._manager_id()))

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
        if mode not in ("manager", "device"):
            raise ValueError("mount_control must be manager or device")
        if mode == "manager":
            self._manager_id()
        camera = self._selected_camera(required=False)
        if camera is not None:
            self.roi.clear(camera)
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

    def _clear_fov(self, keep=None, camera=None):
        keep = set() if keep is None else set(keep)
        map_display = getattr(self.mpstate, "map", None)
        owned = self.fov_objects if camera is None else camera.fov_objects
        removed = owned - keep
        for name in removed:
            if map_display is not None:
                map_display.remove_object(name)
            self.fov_points.pop(name, None)
        self.fov_objects.difference_update(removed)
        for device in self.cameras.values():
            device.fov_objects.difference_update(removed)

    def _vehicle_message(self, message_type, system_id, component_id):
        message = self.vehicle_messages.get((system_id, component_id, message_type))
        if message is None:
            message = self.master.messages.get(message_type)
        if (message is not None and message.get_srcSystem() == system_id and
                message.get_srcComponent() == component_id):
            return message
        return None

    def _fov_attitude(self):
        """Return camera roll, pitch and earth-frame yaw in degrees."""
        gimbal = self._selected_gimbal(required=False)
        camera = self._selected_camera(required=False)
        system_id = camera.system_id if camera is not None else self.target_system or 1
        candidates = []
        if gimbal is not None and gimbal.attitude is not None:
            candidates.append(gimbal.attitude)
            system_id = gimbal.system_id
        try:
            manager_id = self._manager_id() or 1
        except ValueError:
            # A known device is usable even without an ArduPilot mount mapping.
            manager_id = None
        manager_attitude = self.manager_attitudes.get(
            (system_id, self.camera_settings.manager_component, manager_id))
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
            vehicle_attitude = self._vehicle_message(
                "ATTITUDE", system_id, self.camera_settings.manager_component)
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

    def _projection_style(self, camera):
        key = (camera.system_id, camera.component_id)
        index = self._camera_menu_keys().index(key)
        name = "Camera" if index == 0 else "Camera%u" % (index + 1)
        # Space successive hues around the colour wheel, without repeating a
        # short palette when several vehicles/cameras are connected. OpenCV BGR.
        rgb = colorsys.hsv_to_rgb((0.5 + index * 0.61803398875) % 1.0, 0.9, 1.0)
        colour = tuple(round(channel * 255) for channel in reversed(rgb))
        return name, colour

    def _show_fov(self, position):
        """Update each camera on this vehicle without replacing other footprints."""
        keys = self._camera_menu_keys()
        for key, camera in self.cameras.items():
            if key not in keys:
                self._clear_fov(camera=camera)
        for key in keys:
            camera = self.cameras[key]
            if camera.system_id != position.get_srcSystem():
                continue
            with self._camera_context(camera):
                if position.get_srcComponent() == self.camera_settings.manager_component:
                    self._show_camera_fov(camera, position)

    def _refresh_fov(self):
        cameras = ([self.command_camera] if self.command_camera is not None else
                   [self.cameras[key] for key in self._camera_menu_keys()])
        for camera in cameras:
            with self._camera_context(camera):
                camera.last_fov_update = 0.0
                position = self._vehicle_message(
                    "GLOBAL_POSITION_INT", camera.system_id,
                    self.camera_settings.manager_component)
                self._show_camera_fov(camera, position)

    def _show_camera_fov(self, camera, position):
        """Project this camera's streams using its command context/settings."""
        if not mp_util.has_wxpython:
            return
        map_display = getattr(self.mpstate, "map", None)
        if (map_display is None or not self.camera_settings.show_fov or position is None or
                (camera.system_id, camera.component_id) not in self._camera_menu_keys()):
            self._clear_fov(camera=camera)
            return
        now = time.time()
        interval = max(0.0, self.camera_settings.fov_update_interval)
        if now - camera.last_fov_update < interval:
            return
        camera.last_fov_update = now

        attitude = self._fov_attitude()
        if attitude is None or not camera.streams:
            self._clear_fov(camera=camera)
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
        layer, colour = self._projection_style(camera)
        for stream_id in sorted(camera.streams):
            stream = camera.streams[stream_id]
            width = int(getattr(stream, "resolution_h", 0))
            height = int(getattr(stream, "resolution_v", 0))
            hfov = float(getattr(stream, "hfov", 0))
            if width <= 0 or height <= 0 or not 0.0 < hfov < 180.0:
                continue
            name = "%sFOV_%u_%u_%u" % (
                layer, camera.system_id, camera.component_id, stream_id)
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
                continue
            thermal = bool(
                stream.flags &
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL)
            stream_colour = tuple(round(c * 0.65) for c in colour) if thermal else colour
            map_display.add_object(mp_slipmap.SlipPolygon(
                name, points, layer=layer, linewidth=2, colour=stream_colour,
                showcircles=False))
            self.fov_points[name] = points
            active.add(name)
        self.fov_objects.update(active)
        camera.fov_objects.update(active)
        self._clear_fov(keep=active, camera=camera)

    def cmd_projection(self, args):
        if args not in ([], ["toggle"]):
            raise ValueError("usage: camera projection [toggle]")
        camera = self._selected_camera()
        if camera is None:
            return
        with self._camera_context(camera):
            if args:
                enabled = not self.camera_settings.show_fov
                camera.control_overrides["show_fov"] = enabled
                self.camera_settings.show_fov = enabled
                self._refresh_fov()
                print("Camera %u:%u projection %s" % (
                    camera.system_id, camera.component_id, "enabled" if enabled else "disabled"))
            else:
                self.show_projection()

    def show_projection(self):
        map_display = getattr(self.mpstate, "map", None)
        camera = self._selected_camera(required=False)
        position = (self._vehicle_message("GLOBAL_POSITION_INT", camera.system_id,
                                         self.camera_settings.manager_component)
                    if camera is not None else None)
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
            try:
                manager_id = self._manager_id() or 1
                print(" waiting for fresh gimbal status from the camera's device or "
                      "%u:%u mount %u" % (camera.system_id,
                      self.camera_settings.manager_component, manager_id))
            except ValueError as error:
                print(" %s" % error)
        else:
            print(" attitude: roll=%.1f pitch=%.1f earth-yaw=%.1f" % attitude)
        if position is None:
            print(" position: unavailable")
            return
        print(" position: %.7f %.7f AMSL=%.1fm relative=%.1fm" % (
            position.lat * 1.0e-7, position.lon * 1.0e-7,
            position.alt * 1.0e-3, position.relative_alt * 1.0e-3))
        camera.last_fov_update = 0.0
        self._show_camera_fov(camera, position)
        if not camera.fov_objects:
            reason = ("projection disabled" if not self.camera_settings.show_fov else
                      "gimbal attitude unavailable" if attitude is None else
                      "no valid stream footprint intersects available terrain")
            print(" polygons: none (%s)" % reason)
            return
        for name in sorted(camera.fov_objects):
            points = self.fov_points[name]
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
               gimbal_capabilities(info)))
        if gimbal.attitude is not None:
            roll, pitch, yaw = _quaternion_to_euler(gimbal.attitude.q)
            print(" attitude roll=%.1f pitch=%.1f yaw=%.1f" %
                  tuple(math.degrees(value) for value in (roll, pitch, yaw)))

    def _set_console_status(self):
        keys = self._camera_menu_keys()
        names = {"CAMERA" if index == 0 else "CAMERA%u" % (index + 1)
                 for index in range(len(keys))}
        for name in self.camera_status_names - names:
            self.console.set_status(name, "")
        self.camera_status_names = names
        if not keys:
            self.console.set_status("CAMERA", "CAMERA --", row=6)
        for index, key in enumerate(keys):
            camera = self.cameras[key]
            name = "CAMERA" if index == 0 else "CAMERA%u" % (index + 1)
            if camera.information is None:
                text = "%s %u:%u" % (name, camera.system_id, camera.component_id)
            else:
                recording = camera.recording
                if recording is None:
                    recording = (camera.capture_status is not None and
                                 camera.capture_status.video_status != 0)
                text = "%s %s %s" % (name, _text(camera.information.model_name),
                                      "REC" if recording else "READY")
            self.console.set_status(name, text, row=6 + index)

    def mavlink_packet(self, message):
        self.graphs.packet(message)
        message_type = message.get_type()
        system_id = message.get_srcSystem()
        component_id = message.get_srcComponent()
        if message_type in ("HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT"):
            self.vehicle_messages[(system_id, component_id, message_type)] = message
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
            # An autopilot may repeat the camera's definition URI without
            # serving its extended parameters. Only load this endpoint when
            # the operator explicitly selects or requests its settings.
            if (component_id != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 or
                    (self.camera_settings.camera_component == component_id and
                     system_id == (self.target_system or 1)) or
                    (self.camera_selection_explicit and
                     self.selected_camera == (system_id, component_id))):
                camera.parameters.information(message)
            self._prefer_camera_component()
            gimbal_component = getattr(message, "gimbal_device_id", 0)
            if self._is_gimbal_device_component(gimbal_component):
                self._ensure_gimbal(system_id, gimbal_component)
            self._request_camera_state(camera)
            self._set_console_status()
        elif message_type in ("PARAM_EXT_VALUE", "PARAM_EXT_ACK"):
            camera = self.cameras.get((system_id, component_id))
            if camera is not None:
                camera.parameters.packet(message)
        elif message_type == "CAMERA_SETTINGS":
            camera = self._ensure_camera(system_id, component_id)
            camera.settings = message
            parameters = camera.parameters
            if parameters.definition is not None and "CAM_MODE" in parameters.definition.parameters:
                parameters.request_read("CAM_MODE")
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
            else:
                gimbal_id = getattr(message, "gimbal_device_id", 0)
                if gimbal_id:
                    # Per-camera manager overrides are applied only during
                    # commands. Retain each source independently for later use.
                    self.manager_attitudes[(system_id, component_id, gimbal_id)] = message
        elif message_type == "GLOBAL_POSITION_INT":
            self._show_fov(message)
        elif message_type == "PARAM_VALUE":
            self.roi.parameter(message)
        elif message_type == "COMMAND_ACK":
            key = (system_id, component_id, message.command)
            pending = self.pending_commands.get(key, 0)
            if pending == 0:
                return
            self.roi.ack(message)
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
        self.graphs.idle()
        self.roi.idle()
        self._sync_menus()
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
            camera.parameters.idle()
            if (camera.recording_verify_at != 0.0 and
                    now >= camera.recording_verify_at):
                self._request_message(
                    camera.system_id, camera.component_id,
                    mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS)
                camera.recording_verify_at = 0.0
        if now - self.last_status_request >= self.camera_settings.status_interval:
            for key in self._camera_menu_keys():
                camera = self.cameras[key]
                if camera.information is not None:
                    self._request_camera_state(camera)
            self.last_status_request = now


def init(mpstate):
    """Initialise the MAVProxy camera module."""
    return CameraModule(mpstate)
