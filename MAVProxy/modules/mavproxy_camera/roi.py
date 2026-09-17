"""Camera-specific map ROI control, without changing the vehicle-wide map ROI."""

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil


@dataclass
class ROITarget:
    location: tuple
    mode: str
    system: int
    component: int
    mount: int
    attempts: int = 0
    sent_at: float = 0
    confirmed: bool = False

    @property
    def endpoint(self):
        # ArduPilot uses both zero and one for the primary mount.
        mount = (self.mount or 1) if self.mode == "manager" else 0
        return self.mode, self.system, self.component, mount


@dataclass
class TargetStream:
    system: int
    component: int
    parameter: str
    original: float = None
    phase: str = "read"
    attempts: int = 0
    sent_at: float = 0


class CameraROI:
    def __init__(self, module):
        self.module = module
        self.targets = {}
        self.streams = {}

    def multiple_gimbals(self, system):
        components = {component for sysid, component in self.module.gimbals if sysid == system}
        for camera in self.module.cameras.values():
            if camera.system_id != system:
                continue
            component = camera.control_overrides.get(
                "gimbal_component", getattr(camera.information, "gimbal_device_id", 0))
            if self.module._is_gimbal_device_component(component):
                components.add(component)
        return len(components) > 1

    def control_mode(self, camera):
        mode = self.module.camera_settings.mount_control.lower()
        if mode not in ("manager", "device"):
            raise ValueError("mount_control must be manager or device")
        return "device" if self.multiple_gimbals(camera.system_id) else mode

    def menu_items(self):
        keys = self.module._camera_menu_keys()
        if len(keys) < 2:
            return []
        return [("ROI Camera%u" % (index + 1), "# camera for %u:%u roi" % key)
                for index, key in enumerate(keys)] + [("ROI All", "# camera roi all")]

    def command(self, args):
        module = self.module
        if args == ["clear"]:
            camera = module._selected_camera()
            if camera is not None:
                self.clear(camera)
            return
        if args not in ([], ["all"]):
            raise ValueError("usage: camera roi [all|clear] (uses map click)")
        if args:
            if module.command_camera is not None:
                raise ValueError("use 'camera roi all' outside a camera-specific command")
            cameras = [module.cameras[key] for key in module._camera_menu_keys()]
        else:
            camera = module._selected_camera()
            cameras = [] if camera is None else [camera]
        if not cameras:
            raise ValueError("no cameras discovered")
        location = getattr(module.mpstate, "click_location", None)
        if location is None:
            raise ValueError("click an ROI location on the map first")
        lat, lon = location[:2]
        if not (math.isfinite(lat) and math.isfinite(lon) and
                -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("invalid ROI coordinates")
        terrain = module.module("terrain")
        model = getattr(terrain, "ElevationModel", None)
        try:
            alt = None if model is None else model.GetElevation(lat, lon)
        except Exception as error:
            raise ValueError("ROI terrain lookup failed: %s" % error) from error
        if alt is None or not math.isfinite(alt):
            raise ValueError("ROI terrain height unavailable; load terrain and retry")

        # Resolve every endpoint before sending any ROI All commands. In
        # particular, an unknown second mount must not redirect to mount 1.
        plans = []
        for camera in cameras:
            with module._camera_context(camera):
                mode = self.control_mode(camera)
                stream = None
                if mode == "manager":
                    target = ROITarget((lat, lon, alt), mode, camera.system_id,
                                       module.camera_settings.manager_component,
                                       module._manager_id())
                elif mode == "device":
                    gimbal = module._selected_gimbal(required=False)
                    if gimbal is None:
                        raise ValueError("no associated gimbal for camera %u:%u" %
                                         (camera.system_id, camera.component_id))
                    target = ROITarget((lat, lon, alt), mode, gimbal.system_id,
                                       gimbal.component_id, 0)
                    if self.multiple_gimbals(camera.system_id):
                        info = gimbal.information
                        if info is None:
                            module._request_gimbal_state(gimbal)
                            raise ValueError("waiting for gimbal %u:%u capabilities; retry ROI" %
                                             (gimbal.system_id, gimbal.component_id))
                        flags = getattr(info, "cap_flags2", 0) or info.cap_flags
                        if not flags & mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL:
                            raise ValueError("gimbal %u:%u does not support geographic ROI" %
                                             (gimbal.system_id, gimbal.component_id))
                        heartbeat = module._vehicle_message(
                            "HEARTBEAT", camera.system_id, module.camera_settings.manager_component)
                        if (heartbeat is not None and
                                heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
                            mount = module._manager_id() or 1
                            stream = TargetStream(camera.system_id,
                                                  module.camera_settings.manager_component,
                                                  "MNT%u_TARG_RATE" % mount)
                        if (target.endpoint in self.streams and
                                self.streams[target.endpoint].phase == "restore"):
                            raise ValueError("waiting for mount target stream restoration; retry ROI")
                plans.append((camera, target, stream))
        for camera, target, stream in plans:
            self.clear(camera, target.endpoint, replacing=True)
            self.targets[(camera.system_id, camera.component_id)] = target
            if stream is not None:
                existing = self.streams.get(target.endpoint)
                if existing is None:
                    self.streams[target.endpoint] = stream
                    self.request_stream(stream)
                elif existing.phase == "active":
                    self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            else:
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            print("Camera %u:%u ROI %.7f %.7f %.1fm AMSL" %
                  (camera.system_id, camera.component_id, lat, lon, alt))

    def request_stream(self, stream):
        mav = self.module.master.mav
        name = stream.parameter.encode("ascii")
        if stream.phase == "read":
            mav.param_request_read_send(stream.system, stream.component, name, -1)
        else:
            value = stream.original if stream.phase == "restore" else 0
            mav.param_set_send(stream.system, stream.component, name, value,
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        stream.attempts += 1
        stream.sent_at = time.monotonic()

    def restore_stream(self, endpoint, retry=False):
        stream = self.streams.get(endpoint)
        if stream is None:
            return
        if stream.phase == "restore":
            if retry and not math.isfinite(stream.sent_at):
                stream.attempts = 0
                self.request_stream(stream)
            return
        if stream.original in (None, 0):
            del self.streams[endpoint]
            return
        stream.phase, stream.attempts = "restore", 0
        self.request_stream(stream)

    def parameter(self, message):
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode("ascii", errors="replace")
        name = name.rstrip("\0")
        for endpoint, stream in list(self.streams.items()):
            if (message.get_srcSystem(), message.get_srcComponent(), name) != (
                    stream.system, stream.component, stream.parameter):
                continue
            value = message.param_value
            if not math.isfinite(value) or not 0 <= value <= 50:
                continue
            if stream.phase == "restore":
                if value == stream.original:
                    del self.streams[endpoint]
                continue
            if stream.phase == "read":
                stream.original = value
                if value != 0:
                    stream.phase, stream.attempts = "pause", 0
                    self.request_stream(stream)
                    continue
            if value == 0 and stream.phase != "active":
                stream.phase = "active"
                for target in self.targets.values():
                    if target.endpoint == endpoint:
                        self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            elif value != 0 and stream.phase == "active":
                # Another operator changed the rate. Respect that value and
                # stop direct ROI instead of competing with the new control.
                del self.streams[endpoint]
                self.cancel_endpoint(endpoint)
                print("Camera ROI stopped: %s target stream resumed" % stream.parameter)

    def cancel_endpoint(self, endpoint):
        for key, target in list(self.targets.items()):
            if target.endpoint == endpoint:
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
                del self.targets[key]
        self.restore_stream(endpoint)

    def ack(self, message):
        if message.command != mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION:
            return
        for target in list(self.targets.values()):
            if (target.system, target.component) != (message.get_srcSystem(), message.get_srcComponent()):
                continue
            if not target.attempts:
                continue
            if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                target.confirmed = True
            elif message.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                self.cancel_endpoint(target.endpoint)

    def idle(self):
        for target in list(self.targets.values()):
            if target.confirmed or not target.attempts or time.monotonic() - target.sent_at < 1:
                continue
            if target.attempts < 3:
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            else:
                print("Camera ROI cancelled: no acknowledgement from %u:%u" %
                      (target.system, target.component))
                self.cancel_endpoint(target.endpoint)
        for endpoint, stream in list(self.streams.items()):
            if stream.phase == "active" or time.monotonic() - stream.sent_at < 1:
                continue
            if stream.attempts < 3:
                self.request_stream(stream)
                continue
            if stream.phase == "restore":
                print("Camera: could not restore %s on %u:%u; set it to %g manually" %
                      (stream.parameter, stream.system, stream.component, stream.original))
                # Retain the original rate so a later clear/unload can retry.
                stream.sent_at = float("inf")
            else:
                print("Camera ROI cancelled: no confirmation for %s on %u:%u" %
                      (stream.parameter, stream.system, stream.component))
                self.cancel_endpoint(endpoint)

    def close(self):
        for endpoint in {target.endpoint for target in self.targets.values()}:
            self.cancel_endpoint(endpoint)
        for stream in self.streams.values():
            if stream.phase == "restore":
                if not math.isfinite(stream.sent_at):
                    self.request_stream(stream)
                print("Camera: restoring %s=%g on %u:%u; unloading before confirmation" %
                      (stream.parameter, stream.original, stream.system, stream.component))

    def send_location(self, target, command):
        if command == mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION:
            target.attempts += 1
            target.sent_at = time.monotonic()
        key = (target.system, target.component, command)
        pending = self.module.pending_commands
        pending[key] = pending.get(key, 0) + 1
        lat, lon, alt = target.location
        self.module.master.mav.command_int_send(
            target.system, target.component, mavutil.mavlink.MAV_FRAME_GLOBAL,
            command, 0, 0, target.mount, 0, 0, 0, round(lat * 1e7), round(lon * 1e7), alt)

    def clear(self, camera, endpoint=None, replacing=False):
        key = (camera.system_id, camera.component_id)
        owned = self.targets.get(key)
        if endpoint is None and owned is not None:
            endpoint = owned.endpoint
        if endpoint is None and (self.targets or self.streams):
            # Two camera components may share one mount. Manual control from
            # either camera must stop tracking initiated through the other.
            with self.module._camera_context(camera):
                settings = self.module.camera_settings
                if self.control_mode(camera) == "manager":
                    try:
                        mount = self.module._manager_id() or 1
                    except ValueError:
                        return
                    endpoint = ("manager", camera.system_id, settings.manager_component, mount)
                else:
                    gimbal = self.module._selected_gimbal(required=False)
                    if gimbal is not None:
                        endpoint = ("device", gimbal.system_id, gimbal.component_id, 0)
        for owner, target in list(self.targets.items()):
            if owner == key or target.endpoint == endpoint:
                del self.targets[owner]
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
                if target.mode == "device":
                    if not replacing or target.endpoint != endpoint:
                        self.restore_stream(target.endpoint)
        if endpoint is not None and not replacing:
            self.restore_stream(endpoint, retry=True)
