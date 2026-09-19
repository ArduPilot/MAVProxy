"""Camera-specific map ROI control, without changing the vehicle-wide map ROI."""

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil


LOCATION_GLOBAL = getattr(mavutil.mavlink,
                          "GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL", 1 << 17)


def gimbal_capabilities(information):
    """Read the extension even when an older pymavlink only decodes cap_flags."""
    extended = getattr(information, "cap_flags2", 0)
    if not hasattr(information, "cap_flags2"):
        # GIMBAL_DEVICE_INFORMATION has 144 base bytes, then the one-byte
        # gimbal_device_id and uint32 cap_flags2 MAVLink 2 extensions. Parsers
        # retain the received frame, including extensions they do not know.
        # Old get_payload() implementations assume a MAVLink 1 header, so
        # take the payload from the validated MAVLink 2 frame instead.
        frame = getattr(information, "get_msgbuf", lambda: None)()
        if frame and frame[0] == 0xfd:
            payload = frame[10:10 + frame[1]]
            extended = int.from_bytes(payload[145:149], "little")
    return extended or information.cap_flags


@dataclass
class ROITarget:
    location: tuple
    system: int
    component: int
    mount: int
    attempts: int = 0
    sent_at: float = 0
    confirmed: bool = False
    cancelled: bool = False

    @property
    def endpoint(self):
        # ArduPilot uses both zero and one for the primary mount.
        return self.system, self.component, self.mount or 1


class CameraROI:
    def __init__(self, module):
        self.module = module
        self.targets = {}
        # COMMAND_ACK identifies the command and manager, not the mount.
        # Allow only one location transaction per manager on the wire.
        self.inflight = {}
        self.ready_at = {}

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
                target = ROITarget((lat, lon, alt), camera.system_id,
                                   module.camera_settings.manager_component,
                                   module._manager_id())
                plans.append((camera, target))
        for camera, target in plans:
            self.clear(camera, target.endpoint)
            self.targets[(camera.system_id, camera.component_id)] = target
            print("Camera %u:%u ROI %.7f %.7f %.1fm AMSL" %
                  (camera.system_id, camera.component_id, lat, lon, alt))
        self._start_pending()

    def _start_pending(self):
        now = time.monotonic()
        for target in self.targets.values():
            manager = (target.system, target.component)
            if (target.attempts or target.confirmed or manager in self.inflight or
                    now < self.ready_at.get(manager, 0)):
                continue
            self.inflight[manager] = target
            self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)

    def _retire(self, manager, target):
        self.inflight.pop(manager, None)
        # Retries may each produce a reply. Drain late replies before a new
        # mount can own the same ACK key; reset this quiet interval on arrival.
        if target.attempts > 1 or target.cancelled:
            self.ready_at[manager] = time.monotonic() + 1.0

    def _cancel(self, target):
        target.cancelled = True
        if target.attempts:
            self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
        # Keep an outstanding transaction as a tombstone until its ACK or
        # timeout, so a late reply cannot be assigned to the next mount.

    def cancel_endpoint(self, endpoint):
        for key, target in list(self.targets.items()):
            if target.endpoint == endpoint:
                del self.targets[key]
                self._cancel(target)

    def ack(self, message):
        if message.command != mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION:
            return False
        mav = self.module.master.mav
        for field, source in (("target_system", "srcSystem"),
                              ("target_component", "srcComponent")):
            recipient = getattr(message, field, 0)
            if recipient and recipient != getattr(mav, source, None):
                return False
        manager = (message.get_srcSystem(), message.get_srcComponent())
        target = self.inflight.get(manager)
        if target is None:
            if manager in self.ready_at:
                self.ready_at[manager] = time.monotonic() + 1.0
            return False
        if message.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            target.sent_at = time.monotonic()
            return True
        self._retire(manager, target)
        if not target.cancelled:
            if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                target.confirmed = True
            else:
                self.cancel_endpoint(target.endpoint)
        # Start queued work only in idle(), after this receive batch. Do not
        # let another ACK in the batch match a newly sent transaction.
        return True

    def idle(self):
        now = time.monotonic()
        for manager, target in list(self.inflight.items()):
            if now - target.sent_at < 1:
                continue
            if not target.cancelled and target.attempts < 3:
                self.send_location(target, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
            else:
                if not target.cancelled:
                    print("Camera ROI cancelled: no acknowledgement from %u:%u mount %u" %
                          (target.system, target.component, target.mount))
                    self.cancel_endpoint(target.endpoint)
                self._retire(manager, target)
        self._start_pending()

    def close(self):
        for endpoint in {target.endpoint for target in self.targets.values()}:
            self.cancel_endpoint(endpoint)

    def send_location(self, target, command):
        if command == mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION:
            target.attempts += 1
            target.sent_at = time.monotonic()
        key = (target.system, target.component, command)
        pending = self.module.pending_commands
        if command != mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION:
            pending[key] = pending.get(key, 0) + 1
        lat, lon, alt = target.location
        self.module.master.mav.command_int_send(
            target.system, target.component, mavutil.mavlink.MAV_FRAME_GLOBAL,
            command, 0, 0, target.mount, 0, 0, 0, round(lat * 1e7), round(lon * 1e7), alt)

    def clear(self, camera, endpoint=None):
        key = (camera.system_id, camera.component_id)
        owned = self.targets.get(key)
        if endpoint is None and owned is not None:
            endpoint = owned.endpoint
        if endpoint is None and self.targets:
            # Cameras may share a mount. Manual control through either camera
            # stops that mount's ROI, without changing another mount's target.
            with self.module._camera_context(camera):
                try:
                    mount = self.module._manager_id() or 1
                except ValueError:
                    return
                endpoint = (camera.system_id,
                            self.module.camera_settings.manager_component, mount)
        for owner, target in list(self.targets.items()):
            if owner == key or target.endpoint == endpoint:
                del self.targets[owner]
                self._cancel(target)
