"""Camera-specific map ROI control, without changing the vehicle-wide map ROI."""

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util


@dataclass
class ROITarget:
    location: tuple
    mode: str
    system: int
    component: int
    mount: int
    last_update: float = 0

    @property
    def endpoint(self):
        # ArduPilot uses both zero and one for the primary mount.
        mount = (self.mount or 1) if self.mode == "manager" else 0
        return self.mode, self.system, self.component, mount


class CameraROI:
    def __init__(self, module):
        self.module = module
        self.targets = {}

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
                mode = module.camera_settings.mount_control.lower()
                if mode == "manager":
                    target = ROITarget((lat, lon, alt), mode, camera.system_id,
                                       module.camera_settings.manager_component,
                                       module._manager_id())
                    position = module._vehicle_message(
                        "GLOBAL_POSITION_INT", target.system, target.component)
                    angles = self.angles(target, position)
                elif mode == "device":
                    gimbal = module._selected_gimbal(required=False)
                    if gimbal is None:
                        raise ValueError("no associated gimbal for camera %u:%u" %
                                         (camera.system_id, camera.component_id))
                    target = ROITarget((lat, lon, alt), mode, gimbal.system_id,
                                       gimbal.component_id, 0)
                    angles = None
                else:
                    raise ValueError("mount_control must be manager or device")
                plans.append((camera, target, angles))
        for camera, target, angles in plans:
            self.clear(camera, target.endpoint)
            if target.mode == "manager":
                self.send_angles(target, angles)
            else:
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            self.targets[(camera.system_id, camera.component_id)] = target
            print("Camera %u:%u ROI %.7f %.7f %.1fm AMSL" %
                  (camera.system_id, camera.component_id, lat, lon, alt))

    def angles(self, target, position):
        if position is None or time.time() - getattr(position, "_timestamp", 0) > 2:
            raise ValueError("fresh vehicle position required for camera ROI")
        lat, lon, alt = target.location
        settings = self.module.camera_settings
        vehicle_lat, vehicle_lon = position.lat * 1e-7, position.lon * 1e-7
        distance = mp_util.gps_distance(vehicle_lat, vehicle_lon, lat, lon)
        height = alt - (position.alt * 0.001 + settings.mount_alt)
        pitch = math.degrees(math.atan2(height, distance)) + settings.mount_pitch
        yaw = mp_util.wrap_180(mp_util.gps_bearing(vehicle_lat, vehicle_lon, lat, lon) +
                              settings.mount_yaw)
        if not all(math.isfinite(value) for value in (pitch, yaw)):
            raise ValueError("invalid vehicle position for camera ROI")
        return pitch, yaw

    def send_angles(self, target, angles):
        # ArduPilot's DO_SET_ROI_LOCATION handles only its primary mount.
        # Explicit mount angle commands keep tracking independent for each ROI.
        self.module._send_command(
            target.system, target.component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            (*angles, math.nan, math.nan, mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK,
             0, target.mount))
        target.last_update = time.time()

    def send_location(self, target, command):
        key = (target.system, target.component, command)
        pending = self.module.pending_commands
        pending[key] = pending.get(key, 0) + 1
        lat, lon, alt = target.location
        self.module.master.mav.command_int_send(
            target.system, target.component, mavutil.mavlink.MAV_FRAME_GLOBAL,
            command, 0, 0, 0, 0, 0, 0, round(lat * 1e7), round(lon * 1e7), alt)

    def packet(self, position):
        for key, target in self.targets.items():
            if (target.mode != "manager" or position.get_srcSystem() != target.system or
                    position.get_srcComponent() != target.component or
                    time.time() - target.last_update < 0.2):
                continue
            with self.module._camera_context(self.module.cameras[key]):
                try:
                    angles = self.angles(target, position)
                except ValueError:
                    continue
                self.send_angles(target, angles)

    def clear(self, camera, endpoint=None):
        key = (camera.system_id, camera.component_id)
        owned = self.targets.get(key)
        if endpoint is None and owned is not None:
            endpoint = owned.endpoint
        if endpoint is None and self.targets:
            # Two camera components may share one mount. Manual control from
            # either camera must stop tracking initiated through the other.
            with self.module._camera_context(camera):
                settings = self.module.camera_settings
                if settings.mount_control.lower() == "manager":
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
                if target.mode == "device":
                    self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
