#!/usr/bin/env python3
"""Unit tests for the generic MAVLink camera module."""

import contextlib
import io
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest import mock

os.environ.setdefault("MAVLINK20", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from MAVProxy.modules.mavproxy_camera import CameraModule  # noqa: E402
from MAVProxy.modules.mavproxy_camera.graphs import PRESETS  # noqa: E402
from MAVProxy.modules.mavproxy_link import LinkModule  # noqa: E402
from pymavlink import mavutil  # noqa: E402


class FakeMav:
    def __init__(self):
        self.commands = []
        self.gimbal_attitudes = []
        self.int_commands = []
        self.param_reads = []
        self.param_sets = []

    def command_long_send(self, *args):
        self.commands.append(args)

    def gimbal_device_set_attitude_send(self, *args):
        self.gimbal_attitudes.append(args)

    def command_int_send(self, *args):
        self.int_commands.append(args)

    def param_request_read_send(self, *args):
        self.param_reads.append(args)

    def param_set_send(self, *args):
        self.param_sets.append(args)


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.messages = {}


class FakeConsole:
    def __init__(self):
        self.status = {}

    def set_status(self, name, text, **_kwargs):
        self.status[name] = text


class FakeFunctions:
    def say(self, _message):
        pass

    def get_mav_param(self, _name, default=None):
        return default


class FakeMPState:
    def __init__(self):
        self.command_map = {}
        self.completions = {}
        self.completion_functions = {}
        self.public_modules = {}
        self.multi_instance = {}
        self.instance_count = {}
        self.settings = SimpleNamespace(target_system=1, target_component=1)
        self.console = FakeConsole()
        self.status = SimpleNamespace(logdir=".")
        self.functions = FakeFunctions()
        self._master = FakeMaster()
        self.modules = {}

    def master(self):
        return self._master

    def module(self, name):
        return self.modules.get(name)


class FakeMap:
    def __init__(self):
        self.objects = {}
        self.removed = []

    def add_object(self, obj):
        self.objects[obj.key] = obj

    def remove_object(self, name):
        self.removed.append(name)
        self.objects.pop(name, None)


class Message:
    def __init__(self, message_type, system_id=1, component_id=100, **fields):
        self._message_type = message_type
        self._system_id = system_id
        self._component_id = component_id
        self.__dict__.update(fields)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._system_id

    def get_srcComponent(self):
        return self._component_id


def camera_information(**overrides):
    fields = dict(
        vendor_name=b"ArduPilot\0", model_name=b"MT11\0",
        firmware_version=1, focal_length=math.nan,
        sensor_size_h=math.nan, sensor_size_v=math.nan,
        resolution_h=1920, resolution_v=1080, lens_id=0,
        flags=(mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
               mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
               mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM),
        cam_definition_version=0, cam_definition_uri="",
        gimbal_device_id=154)
    fields.update(overrides)
    return Message("CAMERA_INFORMATION", **fields)


def stream_information(stream_id, thermal=False, uri=None):
    return Message(
        "VIDEO_STREAM_INFORMATION", stream_id=stream_id, count=2,
        type=mavutil.mavlink.VIDEO_STREAM_TYPE_RTSP,
        flags=(mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_RUNNING |
               (mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL
                if thermal else 0)),
        framerate=30.0, resolution_h=1280 if thermal else 1920,
        resolution_v=720 if thermal else 1080, bitrate=4096000,
        rotation=0, hfov=24 if thermal else 88,
        name="Thermal" if thermal else "Visible",
        uri=uri or "rtsp://192.168.144.25:8554/video%u" % stream_id,
        encoding=mavutil.mavlink.VIDEO_STREAM_ENCODING_H264)


class CameraModuleTest(unittest.TestCase):
    def setUp(self):
        self.state = FakeMPState()
        self.module = CameraModule(self.state)

    def commands(self, command=None):
        commands = self.state._master.mav.commands
        if command is None:
            return commands
        return [item for item in commands if item[2] == command]

    def test_multiple_camera_menus_and_proxy_replacement(self):
        console = mock.Mock()
        map_module = mock.Mock()
        self.state.modules.update(console=console, map=map_module)
        with mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True):
            self.module.mavlink_packet(camera_information(component_id=1, gimbal_device_id=1))
            self.module._sync_menus()
            self.assertEqual([m.name for m in self.module.menus], ["Camera"])
            # The real device replaces its proxy in the first menu slot.
            self.module.mavlink_packet(camera_information())
            self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
            self.module.mavlink_packet(camera_information(system_id=42))
            self.module._sync_menus()
            self.assertTrue(self.module.multi_vehicle)
            self.assertEqual(self.module.menu_cameras, [(1, 100), (1, 101), (42, 100)])
            self.assertEqual([m.name for m in self.module.menus], ["Camera", "Camera2", "Camera3"])
            self.module.cmd_camera(["select", "42:100"])
            self.module.camera_settings.camera_component = 100
            for menu, key in zip(self.module.menus, self.module.menu_cameras):
                prefix = "# camera for %u:%u " % key
                graph_menu = next(item for item in menu.items if item.name == "Graphs")
                self.assertTrue(all(item.returnkey.startswith(prefix + "graph ")
                                    for item in graph_menu.items))
                photo = next(item for item in menu.items if item.name == "Take photo")
                self.module.cmd_camera(photo.returnkey.split()[2:])
                self.assertEqual(self.commands()[-1][:2], key)
            self.assertEqual(self.module.selected_camera, (42, 100))
            self.assertEqual(self.module.camera_settings.camera_component, 100)
            # Repeated telemetry does not duplicate or renumber menus.
            console.reset_mock()
            self.module.mavlink_packet(camera_information(component_id=1))
            self.module._sync_menus()
            console.add_menu.assert_not_called()
            # Reloading the console restores all camera menus.
            replacement = mock.Mock()
            self.state.modules["console"] = replacement
            self.module._sync_menus()
            self.assertEqual(replacement.add_menu.call_count, 3)
            self.module.unload()
            self.assertEqual(replacement.remove_menu.call_count, 3)

    def test_per_camera_controls_settings_and_polling(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        second = self.module.cameras[(1, 101)]
        original_settings = self.module.camera_settings
        for command in (["photo"], ["record", "toggle"], ["focus", "auto"],
                        ["zoom", "30"], ["mode", "video"], ["stream", "start"]):
            self.module.cmd_camera(["for", "1:101"] + command)
            self.assertEqual(self.commands()[-1][:2], (1, 101))
        self.assertTrue(second.recording)
        self.assertIsNone(self.module.cameras[(1, 100)].recording)
        with mock.patch.object(second.parameters, "open_dialog") as dialog:
            self.module.cmd_camera(["for", "1:101", "custom"])
            dialog.assert_called_once()
        self.module.cmd_camera(["for", "1:101", "set", "rtsp_host", "192.168.1.2"])
        self.assertEqual(second.control_overrides["rtsp_host"], "192.168.1.2")
        self.assertEqual(self.module.camera_settings.rtsp_host, "")
        # A rejected command must also restore the command context.
        self.module.cmd_camera(["for", "1:101", "photo", "invalid"])
        self.assertIsNone(self.module.command_camera)
        self.assertIs(self.module.camera_settings, original_settings)
        self.assertEqual(self.module.selected_camera, (1, 100))
        self.state._master.mav.commands.clear()
        self.module.last_discovery_request = time.time()
        self.module.idle_task()
        requests = self.commands(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)
        for component in (100, 101):
            self.assertTrue(any(c[:2] == (1, component) and c[4] ==
                                mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_SETTINGS for c in requests))

    def setup_camera_roi(self):
        self.roi_now = 100.0
        clock = mock.patch("MAVProxy.modules.mavproxy_camera.roi.time",
                           SimpleNamespace(monotonic=lambda: self.roi_now))
        clock.start()
        self.addCleanup(clock.stop)
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        self.module.mavlink_packet(camera_information())
        for component in (154, 171):
            self.module.mavlink_packet(Message(
                "GIMBAL_DEVICE_INFORMATION", component_id=component, cap_flags=0,
                cap_flags2=1 << 17))
        self.module.mavlink_packet(Message(
            "HEARTBEAT", component_id=1, type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
        self.state.click_location = (-35.0, 149.0)
        self.state.modules["terrain"] = SimpleNamespace(
            ElevationModel=SimpleNamespace(GetElevation=lambda lat, lon: 600.0))
        position = self.projection_position(lat=-350010000)
        position.lon = 1490000000
        position.alt = 700000
        position._timestamp = time.time()
        self.module.mavlink_packet(position)
        self.state._master.mav.commands.clear()
        return position


    def roi_ack(self, result=0, system=1, component=1, **fields):
        self.module.mavlink_packet(Message(
            "COMMAND_ACK", system_id=system, component_id=component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            result=result, **fields))

    def roi_tick(self, seconds=0):
        self.roi_now += seconds
        self.module.roi.idle()

    def test_roi_menu_updates_and_restores_single_camera_action(self):
        from MAVProxy.modules.mavproxy_map import MapModule
        from MAVProxy.modules.lib.mp_menu import MPMenuItem, MPMenuSubMenu
        map_module = MapModule.__new__(MapModule)
        map_module.mpstate = self.state
        map_module.default_popup = MPMenuSubMenu("Popup", [])
        map_module.map = mock.Mock()
        map_module.camera_roi_items = None
        self.state.modules["camera"] = self.module
        map_module.update_roi_menu()
        self.assertIsInstance(map_module.default_popup.items[0], MPMenuItem)
        self.assertEqual(map_module.default_popup.items[0].returnkey, "# map setroi ")
        for component in (102, 101, 100):
            self.module.mavlink_packet(camera_information(component_id=component))
            map_module.update_roi_menu()
        submenu = map_module.default_popup.items[0]
        self.assertIsInstance(submenu, MPMenuSubMenu)
        self.assertEqual([item.name for item in submenu.items],
                         ["ROI Camera1", "ROI Camera2", "ROI Camera3", "ROI All"])
        self.assertEqual([item.returnkey for item in submenu.items], [
            "# camera for 1:100 roi", "# camera for 1:101 roi",
            "# camera for 1:102 roi", "# camera roi all"])
        map_module.map.reset_mock()
        map_module.update_roi_menu()
        map_module.map.add_object.assert_not_called()
        del self.state.modules["camera"]
        map_module.update_roi_menu()
        self.assertEqual(len(map_module.default_popup.items), 1)
        self.assertIsInstance(map_module.default_popup.items[0], MPMenuItem)

    def test_single_gimbal_roi_sends_location_to_manager(self):
        position = self.setup_camera_roi()
        del self.module.cameras[(1, 101)]
        del self.module.gimbals[(1, 171)]
        self.module.cmd_camera(["for", "1:100", "roi"])
        mav = self.state._master.mav
        command = mav.int_commands[-1]
        self.assertEqual(command[:4], (1, 1, mavutil.mavlink.MAV_FRAME_GLOBAL,
                                      mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION))
        self.assertEqual(command[6], 1)
        self.assertEqual(command[10:], (-350000000, 1490000000, 600.0))
        self.assertFalse(mav.param_reads)
        self.assertEqual(set(self.module.roi.targets), {(1, 100)})
        self.state._master.mav.commands.clear()
        position.lat = -350020000
        self.module.mavlink_packet(position)
        self.module.mavlink_packet(position)
        position._system_id = 42
        self.module.mavlink_packet(position)
        self.assertFalse(self.commands())
        self.assertEqual(len(mav.int_commands), 1)
        position._system_id = 1
        self.module.cmd_camera(["for", "1:100", "mount", "center"])
        self.assertEqual(mav.int_commands[-1][:4],
                         (1, 1, mavutil.mavlink.MAV_FRAME_GLOBAL,
                          mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE))
        self.assertFalse(self.module.roi.targets)
        self.state._master.mav.commands.clear()
        self.module.mavlink_packet(position)
        self.assertFalse(self.commands())

    def test_roi_all_and_validation_before_commands(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        mav = self.state._master.mav
        self.assertFalse(mav.param_reads)
        self.assertFalse(mav.param_sets)
        self.assertEqual([c[:2] for c in mav.int_commands], [(1, 1)])
        self.roi_ack()
        self.roi_tick()
        self.assertEqual([c[:2] for c in mav.int_commands], [(1, 1), (1, 1)])
        self.assertEqual([c[6] for c in mav.int_commands], [1, 2])
        self.assertFalse(self.commands())
        self.assertEqual(set(self.module.roi.targets), {(1, 100), (1, 101)})
        self.module.cmd_camera(["for", "1:101", "mount", "angle", "0", "90"])
        self.assertEqual(mav.int_commands[-1][3], mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
        self.assertEqual(mav.int_commands[-1][6], 2)
        command = self.commands(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[-1]
        self.assertEqual(command[:2], (1, 1))
        self.assertEqual(command[10], 2)
        self.assertFalse(mav.param_reads)
        self.assertFalse(mav.param_sets)
        self.assertEqual(set(self.module.roi.targets), {(1, 100)})
        self.state._master.mav.commands.clear()
        mav.int_commands.clear()
        self.state.modules["terrain"].ElevationModel.GetElevation = lambda lat, lon: None
        self.module.cmd_camera(["roi", "all"])
        self.assertFalse(self.commands())
        self.assertFalse(mav.int_commands)
        self.assertEqual(set(self.module.roi.targets), {(1, 100)})
        self.state.modules["terrain"].ElevationModel.GetElevation = lambda lat, lon: 600
        self.module.cameras[(1, 101)].information.gimbal_device_id = 0
        self.module.cmd_camera(["roi", "all"])
        self.assertFalse(self.commands())
        self.assertFalse(mav.int_commands)

    def test_shared_mount_roi_does_not_issue_competing_targets(self):
        position = self.setup_camera_roi()
        self.module.cameras[(1, 101)].information.gimbal_device_id = 154
        del self.module.gimbals[(1, 171)]
        self.module.cmd_camera(["for", "1:100", "set", "manager_gimbal_id", "0"])
        self.module.cmd_camera(["for", "1:100", "roi"])
        self.state.click_location = (-35.002, 149.001)
        self.module.cmd_camera(["for", "1:101", "roi"])
        self.assertEqual(set(self.module.roi.targets), {(1, 101)})
        self.state._master.mav.commands.clear()
        self.module.mavlink_packet(position)
        self.assertFalse(self.commands())
        # Camera1's mount=0 and Camera2's mount=1 refer to the same gimbal.
        self.module.cmd_camera(["for", "1:100", "mount", "center"])
        self.assertFalse(self.module.roi.targets)


    def test_lone_second_mount_roi_uses_manager(self):
        self.setup_camera_roi()
        del self.module.cameras[(1, 100)]
        del self.module.gimbals[(1, 154)]
        self.module.cmd_camera(["for", "1:101", "roi"])
        mav = self.state._master.mav
        self.assertEqual(mav.int_commands[-1][:2], (1, 1))
        self.assertEqual(mav.int_commands[-1][6], 2)
        self.module.cmd_camera(["for", "1:101", "roi", "clear"])
        self.assertEqual(mav.int_commands[-1][:2], (1, 1))
        self.assertEqual(mav.int_commands[-1][3], mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
        self.assertEqual(mav.int_commands[-1][6], 2)
        self.assertFalse(mav.param_reads)
        self.assertFalse(mav.param_sets)

    def test_roi_capability_extension_survives_older_dialect(self):
        from pymavlink.generator.mavcrc import x25crc
        self.setup_camera_roi()
        message = mavutil.mavlink.MAVLink_gimbal_device_information_message(
            0, b"vendor", b"model", b"", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=171)
        packet = message.pack(encoder)
        # Build the extension on the wire, including MAVLink 2 zero truncation,
        # without requiring the installed dialect to know the new field.
        for flags in (1 << 17, 0):
            payload = (packet[10:-2].ljust(145, b"\0")[:145] +
                       flags.to_bytes(4, "little")).rstrip(b"\0")
            header = bytearray(packet[:10])
            header[1] = len(payload)
            crc = x25crc(bytes(header[1:]) + payload)
            crc.accumulate(bytes([message.crc_extra]))
            wire = bytes(header) + payload + crc.crc.to_bytes(2, "little")
            decoded = mavutil.mavlink.MAVLink(None).parse_char(wire)
            self.module.mavlink_packet(decoded)
            from MAVProxy.modules.mavproxy_camera.roi import gimbal_capabilities
            self.assertEqual(bool(gimbal_capabilities(decoded) & (1 << 17)), bool(flags))


    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view.VideoView")
    def test_menu_video_order_when_camera_two_is_discovered_first(self, view_class):
        with mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True):
            for component in (101, 100):
                self.module.mavlink_packet(camera_information(component_id=component))
                for stream_id in (1, 2):
                    stream = stream_information(
                        stream_id, thermal=stream_id == 2,
                        uri="rtsp://192.168.1.%u/video%u" % (component, stream_id))
                    stream._component_id = component
                    self.module.mavlink_packet(stream)
                # Exercise menu replacement when the first camera arrives late.
                self.module._sync_menus()
            self.assertEqual(self.module.menu_cameras, [(1, 100), (1, 101)])
            for menu, component in zip(self.module.menus, (100, 101)):
                video = next(item for item in menu.items if item.name == "Video")
                for label, stream_id in (("Visible 1920x1080", 1), ("Thermal 1280x720", 2)):
                    item = next(item for item in video.items if item.name == label)
                    self.module.cmd_camera(item.returnkey.split()[2:])
                    self.assertEqual(view_class.call_args.args[1].component_id, component)
                    self.assertEqual(view_class.call_args.args[3],
                                     "rtsp://192.168.1.%u/video%u" % (component, stream_id))
                graphs = next(item for item in menu.items if item.name == "Graphs")
                self.assertTrue(all("for 1:%u graph " % component in item.returnkey
                                    for item in graphs.items))

    def test_video_menu_follows_stream_information(self):
        console = mock.Mock()
        self.state.modules.update(console=console)
        with mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True):
            self.module.mavlink_packet(camera_information())
            self.module._sync_menus()
            video = next(item for item in self.module.menus[0].items if item.name == "Video")
            self.assertEqual([item.name for item in video.items], ["No streams discovered"])
            self.assertEqual(video.items[0].returnkey, "# camera for 1:100 streams")
            # Stream discovery replaces the placeholder with one item per stream
            self.module.mavlink_packet(stream_information(2, thermal=True))
            console.reset_mock()
            self.module._sync_menus()
            self.assertEqual(console.remove_menu.call_count, 1)
            self.assertEqual(console.add_menu.call_count, 1)
            video = next(item for item in self.module.menus[0].items if item.name == "Video")
            self.assertEqual([item.name for item in video.items], ["Thermal 1280x720"])
            self.module.mavlink_packet(stream_information(1))
            self.module._sync_menus()
            video = next(item for item in self.module.menus[0].items if item.name == "Video")
            self.assertEqual([(item.name, item.returnkey) for item in video.items], [
                ("Visible 1920x1080", "# camera for 1:100 view 1"),
                ("Thermal 1280x720", "# camera for 1:100 view 2")])
            # Unchanged stream information does not churn the console menus
            console.reset_mock()
            self.module.mavlink_packet(stream_information(1))
            self.module._sync_menus()
            console.add_menu.assert_not_called()
            # Duplicate labels stay distinct
            self.module.mavlink_packet(stream_information(3))
            self.module._sync_menus()
            video = next(item for item in self.module.menus[0].items if item.name == "Video")
            self.assertEqual([item.name for item in video.items],
                             ["Visible 1920x1080", "Thermal 1280x720", "Visible 1920x1080 (3)"])

    def test_late_camera_renumbers_projection_layers_without_leftovers(self):
        self.state.map = FakeMap()
        self.module.camera_settings.fov_update_interval = 0
        second = self.add_projection_camera(1, 101, 171)
        self.module._sync_menus()
        self.module.mavlink_packet(self.projection_position())
        self.assertIn("CameraFOV_1_101_1", self.state.map.objects)
        first = self.add_projection_camera(1, 100, 154)
        self.module._sync_menus()
        self.assertEqual(set(self.state.map.objects), {
            "CameraFOV_1_100_1", "CameraFOV_1_100_2",
            "Camera2FOV_1_101_1", "Camera2FOV_1_101_2"})
        self.assertEqual(set(self.state.map.objects), first.fov_objects | second.fov_objects)

    def test_proxy_merge_clears_obsolete_console_status(self):
        self.module.mavlink_packet(camera_information(component_id=1))
        self.module.mavlink_packet(Message("CAMERA_SETTINGS", zoomLevel=0))
        self.module._set_console_status()
        self.assertIn("CAMERA2", self.state.console.status)
        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.state.console.status["CAMERA2"], "")

    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view.VideoView")
    def test_primary_video_with_main_and_sub_streams(self, view_class):
        with mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True):
            self.module.mavlink_packet(camera_information())
            self.module.mavlink_packet(camera_information(component_id=101))
            # The sub stream can arrive first. Both streams are RGB on A8.
            for stream_id in (2, 1):
                self.module.mavlink_packet(stream_information(stream_id))
            second_stream = stream_information(1)
            second_stream._component_id = 101
            self.module.mavlink_packet(second_stream)
            self.module.cmd_camera(["for", "1:100", "view", "rgb"])
            view_class.assert_called_once()
            self.assertEqual(view_class.call_args.args[2].stream_id, 1)
            self.assertEqual(set(self.module.views), {(1, 100, 1)})
            self.module.cmd_camera(["for", "1:100", "view", "rgb"])
            view_class.assert_called_once()
            self.module.cmd_camera(["for", "1:100", "view", "2"])
            self.assertEqual(set(self.module.views), {(1, 100, 1), (1, 100, 2)})
            self.module.views.clear()
            view_class.reset_mock()
            self.module.cmd_camera(["for", "1:100", "view", "all"])
            self.assertEqual(view_class.call_count, 2)
            self.assertEqual(set(self.module.views), {(1, 100, 1), (1, 100, 2)})

    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view.VideoView")
    def test_targeted_video_uses_own_stream_and_host(self, view_class):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101))
        second = self.module.cameras[(1, 101)]
        second.streams[1] = stream_information(1, uri="rtsp://0.0.0.0/video1")
        self.module.cmd_camera(["for", "1:101", "set", "rtsp_host", "192.168.1.2"])
        with mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True):
            self.module.cmd_camera(["for", "1:101", "view", "rgb"])
        self.assertIs(view_class.call_args.args[1], second)
        self.assertEqual(view_class.call_args.args[3], "rtsp://192.168.1.2/video1")
        self.assertIn((1, 101, 1), self.module.views)

    def test_per_camera_gimbal_routing(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        self.state._master.mav.commands.clear()
        # Without a mount-instance mapping, do not accidentally move mount 1.
        self.module.cmd_camera(["for", "1:101", "mount", "center"])
        self.assertEqual(self.commands(), [])
        self.module.cmd_camera(["for", "1:101", "set", "manager_gimbal_id", "2"])
        self.module.cmd_camera(["for", "1:101", "mount", "center"])
        self.assertEqual(self.commands()[-1][:2], (1, 1))
        self.assertEqual(self.commands()[-1][10], 2)
        self.assertEqual(self.module.camera_settings.manager_gimbal_id, 0)
        # Explicitly setting the default value must still record an override.
        self.module.cmd_camera(["for", "1:100", "set", "manager_gimbal_id", "0"])
        self.module.cmd_camera(["for", "1:100", "mount", "center"])
        self.assertEqual(self.commands()[-1][10], 0)
        self.module.cmd_camera(["for", "1:101", "set", "mount_control", "device"])
        self.module.cmd_camera(["for", "1:101", "mount", "center"])
        self.assertFalse(self.state._master.mav.gimbal_attitudes)
        self.assertEqual(self.commands()[-1][:2], (1, 1))
        self.assertEqual(self.commands()[-1][10], 2)
        self.module.mavlink_packet(camera_information(system_id=42))
        self.module.cmd_camera(["for", "42:100", "mount", "center"])
        self.assertEqual(self.commands()[-1][:2], (42, 1))

    def test_camera_menus_inherit_global_gimbal_overrides(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(Message(
            "HEARTBEAT", component_id=1, type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
        self.module.camera_settings.manager_gimbal_id = 2
        self.module.camera_settings.gimbal_component = 171
        camera = self.module.cameras[(1, 100)]
        with self.module._camera_context(camera):
            self.assertEqual(self.module._manager_id(), 2)
            self.assertEqual(self.module._selected_gimbal().component_id, 171)
        self.module.cmd_camera(["for", "1:100", "set", "manager_gimbal_id", "0"])
        self.module.cmd_camera(["for", "1:100", "set", "gimbal_component", "154"])
        with self.module._camera_context(camera):
            self.assertEqual(self.module._manager_id(), 0)
            self.assertEqual(self.module._selected_gimbal().component_id, 154)
        self.assertEqual(self.module.camera_settings.manager_gimbal_id, 2)
        self.assertEqual(self.module.camera_settings.gimbal_component, 171)

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_per_camera_manager_telemetry_is_retained_and_isolated(self, graph):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        for camera, manager, q in (
                (100, 42, [math.sqrt(0.5), 0, -math.sqrt(0.5), 0]),
                (101, 43, [1, 0, 0, 0])):
            self.module.cmd_camera(["for", "1:%u" % camera, "set", "manager_component", str(manager)])
            self.module.cmd_camera(["for", "1:%u" % camera, "set", "manager_gimbal_id", "1"])
            self.module.mavlink_packet(Message(
                "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=manager,
                gimbal_device_id=1, q=q,
                flags=mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,
                _timestamp=time.time()))
        self.assertEqual(set(self.module.manager_attitudes), {(1, 42, 1), (1, 43, 1)})
        for camera, manager, pitch in ((100, 42, -90), (101, 43, 0)):
            with self.module._camera_context(self.module.cameras[(1, camera)]):
                self.assertAlmostEqual(self.module._fov_attitude()[1], pitch)
                self.module.cmd_camera(["graph", "attitude"])
                self.assertIn("1:%u mount 1" % manager, graph.call_args.kwargs["title"])

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_multiple_camera_graphs(self, live_graph):
        first_graph, second_graph, mount_graph = mock.Mock(), mock.Mock(), mock.Mock()
        live_graph.side_effect = [first_graph, second_graph, mount_graph]
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        for address in ("1:100", "1:101"):
            self.module.cmd_camera(["for", address, "graph", "zoom"])
        self.module.mavlink_packet(Message("CAMERA_SETTINGS", component_id=101, zoomLevel=25))
        first_graph.add_values.assert_not_called()
        second_graph.add_values.assert_called_once_with([25])
        self.module.cmd_camera(["for", "1:101", "set", "manager_gimbal_id", "2"])
        status = Message("GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=1,
                         gimbal_device_id=2, q=[1, 0, 0, 0])
        self.module.mavlink_packet(status)
        self.module.cmd_camera(["for", "1:101", "graph", "attitude"])
        self.assertIn("1:1 mount 2", live_graph.call_args.kwargs["title"])
        self.module.mavlink_packet(status)
        mount_graph.add_values.assert_called_once_with([0, 0, 0])
        self.module.cmd_camera(["for", "1:101", "graph", "close"])
        second_graph.close.assert_called_once()
        mount_graph.close.assert_called_once()
        first_graph.close.assert_not_called()
        self.assertEqual(len(self.module.graphs.windows), 1)

    def test_video_buttons_remain_bound_to_their_camera(self):
        from MAVProxy.modules.lib.mp_menu import MPMenuItem
        from MAVProxy.modules.mavproxy_camera.video_view import VideoView
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101))
        view = VideoView.__new__(VideoView)
        view.module = self.module
        view.camera = self.module.cameras[(1, 101)]
        view.image = mock.Mock()
        for action in ("Photo", "Record", "Autofocus"):
            view.image.events.return_value = [MPMenuItem(action, returnkey="Camera:" + action)]
            view.check_events()
            self.assertEqual(self.commands()[-1][:2], (1, 101))
        self.assertEqual(self.module.selected_camera, (1, 100))

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_camera_graph_presets(self, live_graph):
        self.module.mavlink_packet(camera_information())
        angle = math.radians(-45)
        gimbal = Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=154,
            q=[math.cos(angle / 2), 0, math.sin(angle / 2), 0],
            angular_velocity_x=math.radians(1),
            angular_velocity_y=math.radians(2),
            angular_velocity_z=math.radians(3), flags=12,
            failure_flags=0, time_boot_ms=12000)
        self.module.mavlink_packet(gimbal)
        settings = Message("CAMERA_SETTINGS", zoomLevel=25, focusLevel=50,
                           mode_id=1)
        terrain = Message("TERRAIN_REPORT", component_id=1, current_height=80)
        voltage = Message("SYS_STATUS", component_id=1, voltage_battery=24000)
        expected = {
            "attitude": (gimbal, [0, -45, 0]),
            "rates": (gimbal, [1, 2, 3]),
            "pitchrate": (gimbal, [2]), "yawrate": (gimbal, [3]),
            "flags": (gimbal, [12]), "failures": (gimbal, [0]),
            "time": (gimbal, [12]), "zoom": (settings, [25]),
            "focus": (settings, [50]), "mode": (settings, [1]),
            "terrain": (terrain, [80]), "voltage": (voltage, [24]),
            "tmax": (Message("CAMERA_THERMAL_RANGE", stream_id=2,
                             camera_device_id=0, max=37.25), [37.25]),
        }
        for name, _title, _message_type, _fields in PRESETS:
            with self.subTest(name=name):
                self.module.cmd_camera(["graph", name])
                graph = live_graph.return_value
                graph.reset_mock()
                message, values = expected[name]
                self.module.mavlink_packet(message)
                actual = graph.add_values.call_args.args[0]
                self.assertEqual(len(actual), len(values))
                for a, b in zip(actual, values):
                    self.assertAlmostEqual(a, b)
                self.module.cmd_camera(["graph", "close"])
                graph.close.assert_called_once()

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_tmax_graph_is_bound_to_camera_and_thermal_stream(self, live_graph):
        first, second = mock.Mock(), mock.Mock()
        live_graph.side_effect = [first, second]
        for component in (100, 101):
            self.module.mavlink_packet(camera_information(component_id=component))
            stream = stream_information(2, thermal=True)
            stream._component_id = component
            self.module.mavlink_packet(stream)
            self.module.cmd_camera(["for", "1:%u" % component, "graph", "tmax"])
        requests = self.commands(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
        self.assertEqual([c[:2] for c in requests], [(1, 100), (1, 101)])
        for c in requests:
            self.assertEqual(c[4:8], (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE,
                                     200000, 0, 0))
        self.assertIn("1:101 stream 2", live_graph.call_args.kwargs["title"])
        for system, component, stream, device, value in [
                (1, 101, 1, 0, 99), (2, 101, 2, 0, 99),
                (1, 101, 2, 1, 99), (1, 101, 2, 0, math.nan),
                (1, 101, 2, 0, math.inf)]:
            self.module.mavlink_packet(Message("CAMERA_THERMAL_RANGE", system_id=system,
                component_id=component, stream_id=stream, camera_device_id=device, max=value))
        first.add_values.assert_not_called()
        second.add_values.assert_not_called()
        self.module.mavlink_packet(Message("CAMERA_THERMAL_RANGE", component_id=101,
                                          stream_id=2, camera_device_id=0, max=-5.25))
        second.add_values.assert_called_once_with([-5.25])
        first.add_values.assert_not_called()
        self.module.mavlink_packet(Message("CAMERA_THERMAL_RANGE", component_id=100,
                                          stream_id=2, camera_device_id=0, max=42.5))
        first.add_values.assert_called_once_with([42.5])

    def test_thermal_telemetry_requested_without_open_graph(self):
        self.module.mavlink_packet(camera_information(
            flags=mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_THERMAL_RANGE))
        intervals = self.commands(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
        self.assertEqual(len(intervals), 1)
        self.assertEqual(intervals[0][4:6],
                         (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, 200000))
        camera = self.module.cameras[(1, 100)]
        self.state._master.mav.commands.clear()
        self.module._request_thermal_state(camera)
        self.assertFalse(self.commands())
        camera.last_thermal_request = 0
        self.module._request_thermal_state(camera)
        self.assertEqual(len(self.commands(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)), 1)
        self.module.mavlink_packet(Message("CAMERA_THERMAL_RANGE", component_id=100,
                                          stream_id=2, camera_device_id=0, max=42))
        camera.last_thermal_request = 0
        self.state._master.mav.commands.clear()
        self.module._request_thermal_state(camera)
        self.assertFalse(self.commands())
        self.module.camera_settings.temperature_rate = 10
        self.module._request_thermal_state(camera)
        self.assertEqual(self.commands(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)[-1][5], 100000)
        self.module.camera_settings.temperature_rate = 0
        self.module._request_thermal_state(camera)
        self.assertEqual(self.commands(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)[-1][5], -1)

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_graph_source_selection_and_manager_mount_filter(self, live_graph):
        self.module.mavlink_packet(camera_information(system_id=17))
        self.module.camera_settings.manager_gimbal_id = 2
        manager = Message("GIMBAL_DEVICE_ATTITUDE_STATUS", system_id=17,
                          component_id=1, gimbal_device_id=2,
                          angular_velocity_z=math.pi)
        self.module.mavlink_packet(manager)
        self.module.cmd_camera(["graph", "yawrate"])
        graph = live_graph.return_value
        self.assertIn("17:1 mount 2", live_graph.call_args.kwargs["title"])
        self.module.mavlink_packet(manager)
        graph.add_values.assert_called_once_with([180])
        graph.reset_mock()
        for system, component, mount in [(1, 1, 2), (17, 154, 2), (17, 1, 1)]:
            self.module.mavlink_packet(Message(
                "GIMBAL_DEVICE_ATTITUDE_STATUS", system_id=system,
                component_id=component, gimbal_device_id=mount,
                angular_velocity_z=0))
        graph.add_values.assert_not_called()

        self.module.cmd_camera(["graph", "close"])
        self.module.cmd_camera(["graph", "zoom"])
        self.assertIn("17:100", live_graph.call_args.kwargs["title"])
        graph.reset_mock()
        # Existing windows retain their source across camera selections.
        self.module.cmd_camera(["select", "1:101"])
        for system, component in [(1, 100), (17, 101), (1, 101), (17, 1)]:
            self.module.mavlink_packet(Message(
                "CAMERA_SETTINGS", system_id=system, component_id=component,
                zoomLevel=80))
        graph.add_values.assert_not_called()
        self.module.mavlink_packet(Message(
            "CAMERA_SETTINGS", system_id=17, component_id=100, zoomLevel=40))
        graph.add_values.assert_called_once_with([40])

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_selected_camera_graph_binds_to_its_mount(self, live_graph):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(component_id=101, gimbal_device_id=171))
        self.module.mavlink_packet(Message("HEARTBEAT", system_id=1, component_id=1,
                                           autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                           type=mavutil.mavlink.MAV_TYPE_QUADROTOR))
        # ArduPilot forwards both mounts' attitudes as its own gimbal manager
        for mount in (1, 2):
            self.module.mavlink_packet(Message(
                "GIMBAL_DEVICE_ATTITUDE_STATUS", system_id=1, component_id=1,
                gimbal_device_id=mount, angular_velocity_z=0))
        self.module.cmd_camera(["select", "1:101"])
        self.module.cmd_camera(["graph", "yawrate"])
        self.assertIn("1:1 mount 2", live_graph.call_args.kwargs["title"])
        self.module.cmd_camera(["select", "1:100"])
        self.module.cmd_camera(["graph", "yawrate"])
        self.assertIn("1:1 mount 1", live_graph.call_args.kwargs["title"])
        # a bare close still closes every camera's graphs
        self.module.cmd_camera(["graph", "close"])
        self.assertEqual(self.module.graphs.windows, [])

    @mock.patch("MAVProxy.modules.lib.mp_util.has_wxpython", True)
    @mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
    def test_graph_unknown_values_and_cleanup(self, live_graph):
        self.module.mavlink_packet(camera_information())
        self.module.cmd_camera(["graph", "attitude"])
        graph = live_graph.return_value
        for q in ([math.nan, 0, 0, 0], [1, 0]):
            self.module.graphs.packet(Message(
                "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=154, q=q))
        graph.add_values.assert_not_called()
        self.module.graphs.packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=154,
            q=[math.sqrt(0.5), 0, -math.sqrt(0.5), 0]))
        for actual, expected in zip(graph.add_values.call_args.args[0], [0, -90, 0]):
            self.assertAlmostEqual(actual, expected)
        graph.is_alive.return_value = False
        self.module.graphs.idle()
        graph.close.assert_called_once()
        self.assertEqual(self.module.graphs.windows, [])

        for name, message in [
                ("zoom", Message("CAMERA_SETTINGS", zoomLevel=math.nan)),
                ("voltage", Message("SYS_STATUS", component_id=1,
                                    voltage_battery=65535)),
                ("rates", Message("GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=154,
                                  angular_velocity_x=0, angular_velocity_y=math.nan,
                                  angular_velocity_z=math.inf))]:
            self.module.cmd_camera(["graph", name])
            graph.reset_mock()
            self.module.graphs.packet(message)
            graph.add_values.assert_not_called()
            self.module.cmd_camera(["graph", "close"])
        self.module.cmd_camera(["graph", "zoom"])
        graph.reset_mock()
        self.module.unload()
        graph.close.assert_called_once()

    def test_heartbeat_discovery_and_state_collection(self):
        self.module.mavlink_packet(Message(
            "HEARTBEAT", component_id=100,
            type=mavutil.mavlink.MAV_TYPE_CAMERA))
        self.assertIn((1, 100), self.module.cameras)
        self.assertEqual(len(self.commands(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)), 5)

        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.module.selected_camera, (1, 100))
        self.assertIn((1, 154), self.module.gimbals)
        self.assertIn("MT11", self.state.console.status["CAMERA"])

        self.module.mavlink_packet(Message(
            "HEARTBEAT", component_id=154,
            type=mavutil.mavlink.MAV_TYPE_GIMBAL))
        self.assertGreaterEqual(
            len(self.commands(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)), 11)

    def test_definition_parameters_route_to_owning_camera(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(camera_information(system_id=17))
        first = self.module.cameras[(1, 100)]
        second = self.module.cameras[(17, 100)]
        first.parameters = mock.Mock()
        second.parameters = mock.Mock()
        packet = Message("PARAM_EXT_VALUE", system_id=17,
                         param_id="GAIN", param_type=9, param_value=b"\0" * 128)
        self.module.mavlink_packet(packet)
        second.parameters.packet.assert_called_once_with(packet)
        first.parameters.packet.assert_not_called()
        self.module.cmd_camera(["custom"])
        first.parameters.open_dialog.assert_called_once()
        self.module.unload()
        first.parameters.close.assert_called_once()
        second.parameters.close.assert_called_once()

    def test_duplicate_proxy_does_not_download_or_request_parameters(self):
        from MAVProxy.modules.mavproxy_camera.definition import CameraDefinition
        xml = b'<mavlinkcamera><definition/><parameters><parameter name="GAIN" type="uint8"/></parameters></mavlinkcamera>'
        ftp = mock.Mock()
        self.state.modules['ftp'] = ftp
        self.state._master.mav.param_ext_request_list_send = mock.Mock()
        self.state._master.mav.param_ext_request_read_send = mock.Mock()
        uri = 'mftp://[;comp=100]/camera.xml'
        # Idle between advertisements too: proxy loading must not race discovery.
        self.module.mavlink_packet(camera_information(component_id=1, cam_definition_uri=uri))
        self.module.idle_task()
        ftp.cmd_get.assert_not_called()
        self.module.mavlink_packet(camera_information(cam_definition_uri=uri))
        self.assertEqual(ftp.cmd_get.call_count, 1)
        parameters = self.module.cameras[(1, 100)].parameters
        # Deliver the worker result on the same queue as a real download.
        parameters.result_queue.put((parameters.generation, CameraDefinition(xml), None))
        self.module.idle_task()
        self.state._master.mav.param_ext_request_list_send.assert_called_once_with(1, 100)
        parameters.reads['GAIN'][0] = 0
        self.module.idle_task()
        self.state._master.mav.param_ext_request_read_send.assert_called_once_with(1, 100, b'GAIN', -1)
        self.assertFalse(self.module.cameras[(1, 1)].parameters.reads)

    def test_explicit_autopilot_selection_can_still_load_its_definition(self):
        ftp = mock.Mock()
        self.state.modules['ftp'] = ftp
        self.module.mavlink_packet(camera_information(component_id=1,
                                  cam_definition_uri='mftp:///camera.xml'))
        ftp.cmd_get.assert_not_called()
        self.module.cmd_select(['1:1'])
        ftp.cmd_get.assert_called_once()
        self.assertEqual(ftp.cmd_get.call_args.kwargs['target_component'], 1)

    def test_component_override_loads_only_the_selected_system_proxy(self):
        ftp = mock.Mock()
        self.state.modules['ftp'] = ftp
        self.module.camera_settings.camera_component = 1
        self.module.mavlink_packet(camera_information(system_id=17, component_id=1,
                                  cam_definition_uri='mftp:///camera.xml'))
        ftp.cmd_get.assert_not_called()
        self.module.mavlink_packet(camera_information(component_id=1,
                                  cam_definition_uri='mftp:///camera.xml'))
        ftp.cmd_get.assert_called_once()
        self.assertEqual(ftp.cmd_get.call_args.kwargs['target_system'], 1)

    def test_opening_proxy_settings_preserves_local_definition_override(self):
        self.module.mavlink_packet(camera_information(component_id=1))
        parameters = self.module.cameras[(1, 1)].parameters
        parameters.identity = ('/tmp/local-camera.xml', 0)
        with mock.patch.object(parameters, 'information') as information, \
                mock.patch.object(parameters, 'open_dialog') as open_dialog:
            self.module.cmd_custom('custom', [])
            self.module.cmd_select(['1:1'])
        information.assert_not_called()
        open_dialog.assert_called_once()

    def test_camera_component_replaces_automatic_autopilot_proxy_selection(self):
        self.module.mavlink_packet(camera_information(component_id=1))
        self.assertEqual(self.module.selected_camera, (1, 1))
        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.module.selected_camera, (1, 100))
        self.module.mavlink_packet(camera_information(component_id=1))
        self.assertEqual(self.module.selected_camera, (1, 100))

    def test_proxy_information_arriving_after_camera_information(self):
        # Other camera messages can create the proxy before its information.
        self.module._ensure_camera(1, 1)
        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.module.selected_camera, (1, 1))
        self.module.mavlink_packet(camera_information(component_id=1))
        self.assertEqual(self.module.selected_camera, (1, 100))

    def test_proxy_replacement_respects_identity_system_and_explicit_selection(self):
        self.module.mavlink_packet(camera_information(component_id=1))
        self.module.mavlink_packet(camera_information(system_id=17))
        self.module.mavlink_packet(camera_information(model_name=b"Other camera"))
        self.assertEqual(self.module.selected_camera, (1, 1))
        self.module.cmd_select(["1:1"])
        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.module.selected_camera, (1, 1))

    def test_camera_component_setting_prevents_automatic_proxy_replacement(self):
        self.module.camera_settings.camera_component = 1
        self.module.mavlink_packet(camera_information(component_id=1))
        self.module.mavlink_packet(camera_information())
        self.assertEqual(self.module.selected_camera, (1, 1))

    def test_open_proxy_dialog_follows_automatic_camera_selection(self):
        self.module.mavlink_packet(camera_information(component_id=1))
        proxy = self.module.cameras[(1, 1)].parameters
        old_dialog = mock.Mock()
        proxy.dialog = old_dialog
        camera = self.module._ensure_camera(1, 100)
        with mock.patch.object(camera.parameters, "open_dialog") as open_dialog:
            self.module.mavlink_packet(camera_information())
            open_dialog.assert_called_once()
        old_dialog.close.assert_called_once()
        self.assertIsNone(proxy.dialog)
        self.assertFalse(proxy.open_when_ready)

    def test_stream_discovery_and_automatic_urls(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(stream_information(1))
        self.module.mavlink_packet(stream_information(2, thermal=True))
        camera = self.module.cameras[(1, 100)]
        self.assertEqual(len(camera.streams), 2)
        self.assertEqual(self.module._resolved_uri(camera, camera.streams[1]),
                         "rtsp://192.168.144.25:8554/video1")
        camera.streams[1].uri = "rtsp://0.0.0.0:8554/video1"
        camera.information.cam_definition_uri = "http://192.168.144.25/camera.xml"
        self.assertEqual(self.module._resolved_uri(camera, camera.streams[1]),
                         "rtsp://192.168.144.25:8554/video1")
        self.module.camera_settings.rtsp_host = "10.0.0.7"
        self.assertEqual(self.module._resolved_uri(camera, camera.streams[1]),
                         "rtsp://10.0.0.7:8554/video1")

    def test_map_projection_for_visible_and_thermal_streams(self):
        self.state.map = FakeMap()
        self.module.camera_settings.fov_update_interval = 0.0
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(stream_information(1))
        self.module.mavlink_packet(stream_information(2, thermal=True))
        self.state._master.messages["ATTITUDE"] = Message(
            "ATTITUDE", component_id=1, roll=0.0, pitch=0.0, yaw=0.0,
            yawspeed=0.0, _timestamp=time.time())
        # ArduPilot consumes device component 154 status addressed to itself
        # and publishes its manager view from component 1 to the GCS.
        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=1,
            q=[math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0],
            angular_velocity_x=0.0, angular_velocity_y=0.0,
            angular_velocity_z=0.0,
            flags=mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            gimbal_device_id=1, _timestamp=time.time()))
        self.assertNotIn((1, 1), self.module.gimbals)
        self.module.mavlink_packet(Message(
            "GLOBAL_POSITION_INT", component_id=1,
            lat=-353632620, lon=1491652370, alt=684000,
            relative_alt=100000))

        visible = self.state.map.objects["CameraFOV_1_100_1"]
        thermal = self.state.map.objects["CameraFOV_1_100_2"]
        self.assertGreaterEqual(len(visible.points), 4)
        self.assertGreaterEqual(len(thermal.points), 4)
        self.assertNotEqual(visible.colour, thermal.colour)
        self.assertTrue(all(t <= v for t, v in zip(thermal.colour, visible.colour)))
        self.assertFalse(visible._showcircles)
        self.assertFalse(thermal._showcircles)
        self.assertGreater(
            max(point[0] for point in visible.points) -
            min(point[0] for point in visible.points),
            max(point[0] for point in thermal.points) -
            min(point[0] for point in thermal.points))

        self.module.camera_settings.show_fov = False
        self.module.mavlink_packet(Message(
            "GLOBAL_POSITION_INT", component_id=1,
            lat=-353632620, lon=1491652370, alt=684000,
            relative_alt=100000))
        self.assertFalse(self.state.map.objects)

    def add_projection_camera(self, system, component, gimbal):
        self.module.mavlink_packet(camera_information(
            system_id=system, component_id=component, gimbal_device_id=gimbal))
        for stream_id in (1, 2):
            stream = stream_information(stream_id, thermal=stream_id == 2)
            stream._system_id, stream._component_id = system, component
            self.module.mavlink_packet(stream)
        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", system_id=system, component_id=gimbal,
            q=[math.cos(math.pi / 6), 0, -math.sin(math.pi / 6), 0],
            angular_velocity_x=0, angular_velocity_y=0, angular_velocity_z=0,
            flags=mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            _timestamp=time.time()))
        self.module.mavlink_packet(Message(
            "ATTITUDE", system_id=system, component_id=1, yaw=0, yawspeed=0,
            _timestamp=time.time()))
        return self.module.cameras[(system, component)]

    def projection_position(self, system=1, component=1, lat=-353632620):
        return Message("GLOBAL_POSITION_INT", system_id=system, component_id=component,
                       lat=lat, lon=1491652370, alt=684000, relative_alt=100000)

    def test_independent_camera_polygons_colours_and_menu_toggles(self):
        self.state.map = FakeMap()
        self.module.camera_settings.fov_update_interval = 0
        first = self.add_projection_camera(1, 100, 154)
        second = self.add_projection_camera(1, 101, 171)
        self.module.mavlink_packet(self.projection_position())
        self.assertEqual(len(self.state.map.objects), 4)
        self.assertEqual(first.fov_objects, {"CameraFOV_1_100_1", "CameraFOV_1_100_2"})
        self.assertEqual(second.fov_objects, {"Camera2FOV_1_101_1", "Camera2FOV_1_101_2"})
        colours = {p.colour for p in self.state.map.objects.values()}
        self.assertEqual(len(colours), 4)
        self.assertEqual(self.state.map.objects["Camera2FOV_1_101_1"].layer, "Camera2")
        self.module._sync_menus()
        first_polygons = {name: self.state.map.objects[name] for name in first.fov_objects}
        toggle = next(item for item in self.module.menus[1].items
                      if item.name == "Toggle Projection")
        # Disabling is immediate even while this camera is rate limited.
        second.control_overrides["fov_update_interval"] = 100
        self.module.cmd_camera(toggle.returnkey.split()[2:])
        self.assertFalse(second.fov_objects)
        self.assertEqual(self.state.map.objects, first_polygons)
        self.assertTrue(self.module.camera_settings.show_fov)
        self.assertEqual(self.module.selected_camera, (1, 100))
        # A telemetry update cannot resurrect the disabled footprint.
        self.module.mavlink_packet(self.projection_position())
        self.assertEqual(set(self.state.map.objects), first.fov_objects)
        self.module.cmd_camera(toggle.returnkey.split()[2:])
        self.assertEqual(len(self.state.map.objects), 4)
        self.assertEqual(colours, {p.colour for p in self.state.map.objects.values()})
        self.module.cmd_camera(["for", "1:100", "set", "show_fov", "false"])
        self.assertEqual(set(self.state.map.objects), second.fov_objects)
        self.module.unload()
        self.assertFalse(self.state.map.objects)
        self.assertFalse(self.module.fov_points)

    def test_camera_projection_cleanup_and_rates_are_independent(self):
        self.state.map = FakeMap()
        first = self.add_projection_camera(1, 100, 154)
        second = self.add_projection_camera(1, 101, 171)
        first.control_overrides["fov_update_interval"] = 100
        second.control_overrides["fov_update_interval"] = 0
        position = self.projection_position()
        self.module.mavlink_packet(position)
        first_polygon = self.state.map.objects["CameraFOV_1_100_1"]
        second.streams[2].hfov = 0
        self.module.mavlink_packet(position)
        self.assertIs(self.state.map.objects["CameraFOV_1_100_1"], first_polygon)
        self.assertEqual(second.fov_objects, {"Camera2FOV_1_101_1"})
        # Invalid attitude only removes this camera's remaining projection.
        self.module.gimbals[(1, 171)].attitude.q = [math.nan] * 4
        self.module.mavlink_packet(position)
        self.assertEqual(set(self.state.map.objects), first.fov_objects)
        self.assertFalse(second.fov_objects)

    def test_ardupilot_forwarded_projection_with_two_cameras(self):
        self.state.map = FakeMap()
        self.module.camera_settings.fov_update_interval = 0
        # Discover the secondary camera first to rule out discovery-order mapping.
        secondary = self.add_projection_camera(1, 101, 171)
        primary = self.add_projection_camera(1, 100, 154)
        for gimbal in self.module.gimbals.values():
            gimbal.attitude = None
        self.module.mavlink_packet(Message(
            "HEARTBEAT", component_id=1, type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
        # Same source/flags/quaternion as the reported flight log: only the
        # autopilot's mount-1 status reaches the GCS, not device component 154.
        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=1, gimbal_device_id=1,
            q=[0.2718274, 0.12801655, -0.03651098, 0.95309424], flags=60,
            angular_velocity_x=math.nan, angular_velocity_y=math.nan,
            angular_velocity_z=math.nan, _timestamp=time.time()))
        self.module.mavlink_packet(self.projection_position())
        self.assertEqual(len(primary.fov_objects), 2)
        self.assertFalse(secondary.fov_objects)
        with self.module._camera_context(primary):
            self.assertEqual(self.module._manager_id(), 1)
        with self.module._camera_context(secondary):
            self.assertEqual(self.module._manager_id(), 2)
        # A second mount report enables only its own camera, without borrowing
        # mount 1's attitude in the meantime.
        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", component_id=1, gimbal_device_id=2,
            q=[math.sqrt(0.5), 0, -math.sqrt(0.5), 0], flags=60,
            angular_velocity_x=math.nan, angular_velocity_y=math.nan,
            angular_velocity_z=math.nan, _timestamp=time.time()))
        self.module.mavlink_packet(self.projection_position())
        self.assertEqual(len(self.state.map.objects), 4)
        self.module.cmd_camera(["for", "1:101", "set", "manager_gimbal_id", "1"])
        with self.module._camera_context(secondary):
            self.assertEqual(self.module._manager_id(), 1)
        # Another vehicle's heartbeat must not opt this manager into the
        # ArduPilot-specific mapping.
        self.module.vehicle_messages[(1, 1, "HEARTBEAT")].autopilot = mavutil.mavlink.MAV_AUTOPILOT_PX4
        with self.module._camera_context(primary):
            with self.assertRaises(ValueError):
                self.module._manager_id()

    @mock.patch("MAVProxy.modules.lib.camera_projection.CameraProjection.get_projection")
    def test_projections_use_each_vehicles_pose_and_camera_offsets(self, projection):
        projection.return_value = [(-35, 149), (-35.001, 149), (-35, 149.001), (-35, 149)]
        self.state.map = FakeMap()
        self.module.camera_settings.fov_update_interval = 0
        self.add_projection_camera(1, 100, 154)
        second = self.add_projection_camera(42, 100, 154)
        second.control_overrides.update(mount_yaw=10, mount_alt=5)
        for system, yaw in ((1, 0.1), (42, 0.9)):
            self.module.mavlink_packet(Message(
                "ATTITUDE", system_id=system, component_id=1, yaw=yaw,
                yawspeed=0, _timestamp=time.time()))
        for system, yaw in ((1, math.degrees(0.1)), (42, math.degrees(0.9) - 10)):
            projection.reset_mock()
            self.module.mavlink_packet(self.projection_position(system, lat=-350000000-system))
            self.assertEqual(projection.call_count, 2)
            args = projection.call_args.args
            self.assertAlmostEqual(args[0], (-350000000-system) * 1e-7)
            self.assertAlmostEqual(args[2], 689 if system == 42 else 684)
            self.assertAlmostEqual(args[5], yaw)
        self.assertEqual(len(self.state.map.objects), 4)
        # A position from another component must not relocate either camera.
        projection.reset_mock()
        self.module.mavlink_packet(self.projection_position(42, component=100))
        projection.assert_not_called()
        self.module.cmd_camera(["for", "42:100", "projection", "toggle"])
        self.module.cmd_camera(["for", "42:100", "projection", "toggle"])
        self.assertAlmostEqual(projection.call_args.args[0], (-350000000-42) * 1e-7)
        self.assertEqual(len(self.state.map.objects), 4)

    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view._opencv_has_gstreamer",
                return_value=False)
    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view.MPImage")
    def test_video_view_falls_back_to_ffmpeg(self, image_class, _has_gstreamer):
        from MAVProxy.modules.mavproxy_camera.video_view import VideoView

        image = image_class.return_value
        image.get_popup_menu.return_value = mock.Mock()
        stream = stream_information(1)
        camera = SimpleNamespace(label=lambda: "Test camera")
        uri = "rtsp://127.0.0.1:8554/video1"
        VideoView(self.module, camera, stream, uri, 100)
        image.set_video.assert_called_once_with(uri, reconnect=True)
        image.set_gstreamer.assert_not_called()

    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view._opencv_has_gstreamer",
                return_value=True)
    @mock.patch("MAVProxy.modules.mavproxy_camera.video_view.MPImage")
    def test_video_view_uses_siyi_gstreamer_path(self, image_class,
                                                _has_gstreamer):
        from MAVProxy.modules.mavproxy_camera.video_view import VideoView

        image = image_class.return_value
        image.get_popup_menu.return_value = mock.Mock()
        stream = stream_information(2, thermal=True)
        stream.encoding = mavutil.mavlink.VIDEO_STREAM_ENCODING_H265
        camera = SimpleNamespace(label=lambda: "Test camera")
        uri = "rtsp://127.0.0.1:8554/video2"
        VideoView(self.module, camera, stream, uri, 75)
        pipeline = image.set_gstreamer.call_args.args[0]
        self.assertTrue(image.set_gstreamer.call_args.kwargs['reconnect'])
        self.assertIn("rtspsrc location=" + uri, pipeline)
        self.assertIn("latency=75 protocols=tcp", pipeline)
        self.assertIn("rtph265depay", pipeline)
        self.assertIn("h265parse ! avdec_h265", pipeline)
        image.set_video.assert_not_called()

    def test_roi_and_mount_never_modify_target_rate_or_bypass_manager(self):
        self.setup_camera_roi()
        mav = self.state._master.mav
        self.state.mav_param_by_sysid = {(1, 1): {"MNT1_TARG_RATE": 0, "MNT2_TARG_RATE": 0}}
        self.module.cmd_camera(["roi", "all"])
        self.roi_ack()
        self.roi_tick()
        self.roi_ack()
        self.assertEqual([c[6] for c in mav.int_commands], [1, 2])
        self.assertTrue(all(c[:2] == (1, 1) for c in mav.int_commands))
        for component, mount in ((100, 1), (101, 2)):
            prefix = ["for", "1:%u" % component]
            # Old device settings cannot re-enable the removed bypass.
            self.module.cmd_camera(prefix + ["set", "mount_control", "device"])
            for args in (["angle", "-20", "35", "earth"],
                         ["rate", "4", "-8", "body"], ["center"], ["neutral"], ["retract"]):
                self.module.cmd_camera(prefix + ["mount"] + args)
                command = self.commands(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[-1]
                self.assertEqual(command[:2], (1, 1))
                self.assertEqual(command[10], mount)
                if args[0] == "angle":
                    self.assertEqual(command[4:6], (-20, 35))
                    self.assertEqual(command[8], mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
                if args[0] == "rate":
                    self.assertEqual(command[6:8], (4, -8))
            self.module.cmd_camera(prefix + ["roi"])
            self.roi_ack()
            self.module.cmd_camera(prefix + ["roi", "clear"])
            self.assertEqual(mav.int_commands[-1][6], mount)
        self.module.cmd_camera(["roi", "all"])
        self.module.unload()
        self.assertFalse(self.module.roi.targets)
        self.assertFalse(mav.param_reads)
        self.assertFalse(mav.param_sets)
        self.assertFalse(mav.gimbal_attitudes)

    def test_roi_manager_retry_ack_and_rejection(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["for", "1:101", "roi"])
        target = self.module.roi.targets[(1, 101)]
        target.sent_at = 0
        self.module.roi.idle()
        self.assertEqual(target.attempts, 2)
        self.module.mavlink_packet(Message(
            "COMMAND_ACK", component_id=1,
            command=mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED))
        self.assertTrue(target.confirmed)
        self.roi_tick(2)
        self.module.cmd_camera(["for", "1:101", "roi"])
        self.module.mavlink_packet(Message(
            "COMMAND_ACK", component_id=1,
            command=mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            result=mavutil.mavlink.MAV_RESULT_UNSUPPORTED))
        self.assertFalse(self.module.roi.targets)
        self.module.cmd_camera(["for", "1:101", "roi"])
        target = self.module.roi.targets[(1, 101)]
        for _ in range(3):
            target.sent_at = 0
            self.module.roi.idle()
        self.assertFalse(self.module.roi.targets)
        mav = self.state._master.mav
        self.assertFalse(mav.param_sets)
        self.assertTrue(all(c[:2] == (1, 1) and c[6] == 2 for c in mav.int_commands))

    def test_roi_all_mixed_ack_results_are_mount_specific(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        first = self.module.roi.targets[(1, 100)]
        second = self.module.roi.targets[(1, 101)]
        self.assertEqual((first.attempts, second.attempts), (1, 0))
        self.roi_ack()
        self.assertTrue(first.confirmed)
        self.assertFalse(second.confirmed)
        # Duplicate reply in the same receive batch cannot affect queued work.
        self.roi_ack(mavutil.mavlink.MAV_RESULT_FAILED)
        self.assertEqual(set(self.module.roi.targets), {(1, 100), (1, 101)})
        self.roi_tick()
        self.assertEqual(second.attempts, 1)
        self.roi_ack(mavutil.mavlink.MAV_RESULT_FAILED)
        self.assertEqual(set(self.module.roi.targets), {(1, 100)})
        self.assertTrue(first.confirmed)
        clears = [c for c in self.state._master.mav.int_commands if c[3] ==
                  mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE]
        self.assertEqual([c[6] for c in clears], [2])
        key = (1, 1, mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
        self.assertNotIn(key, self.module.pending_commands)

    def test_roi_first_mount_rejected_second_can_succeed(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        self.roi_ack(mavutil.mavlink.MAV_RESULT_DENIED)
        self.assertEqual(set(self.module.roi.targets), {(1, 101)})
        self.roi_tick()
        self.roi_ack()
        self.assertTrue(self.module.roi.targets[(1, 101)].confirmed)

    def test_roi_retries_drain_late_ack_before_next_mount(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        first = self.module.roi.targets[(1, 100)]
        second = self.module.roi.targets[(1, 101)]
        self.roi_tick(1)
        self.assertEqual(first.attempts, 2)
        self.roi_ack()
        self.roi_tick(0.5)
        self.assertEqual(second.attempts, 0)
        self.roi_ack(mavutil.mavlink.MAV_RESULT_FAILED)  # late retry response
        self.roi_tick(0.6)
        self.assertEqual(second.attempts, 0)
        self.assertTrue(first.confirmed)
        self.roi_tick(0.5)
        self.assertEqual(second.attempts, 1)
        self.roi_ack()
        self.assertTrue(second.confirmed)

    def test_roi_clear_outstanding_request_does_not_confirm_next_mount(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        second = self.module.roi.targets[(1, 101)]
        self.module.cmd_camera(["for", "1:100", "roi", "clear"])
        self.roi_tick()
        self.assertEqual(second.attempts, 0)
        self.roi_ack()  # response to cancelled mount 1
        self.assertFalse(second.confirmed)
        self.roi_tick(1.1)
        self.assertEqual(second.attempts, 1)
        self.roi_ack()
        self.assertTrue(second.confirmed)
        self.assertNotIn((1, 100), self.module.roi.targets)

    def test_roi_replacing_pending_target_ignores_old_ack(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["for", "1:100", "roi"])
        old = self.module.roi.targets[(1, 100)]
        self.state.click_location = (-35.001, 149.002)
        self.module.cmd_camera(["for", "1:100", "roi"])
        new = self.module.roi.targets[(1, 100)]
        self.assertEqual(new.attempts, 0)
        self.roi_ack()
        self.assertFalse(new.confirmed)
        self.assertTrue(old.cancelled)
        self.roi_tick(1.1)
        self.assertEqual(new.attempts, 1)
        self.roi_ack()
        self.assertTrue(new.confirmed)

    def test_roi_clear_queued_target_never_sends_it(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        self.module.cmd_camera(["for", "1:101", "roi", "clear"])
        self.roi_ack()
        self.roi_tick(2)
        self.assertEqual([c[6] for c in self.state._master.mav.int_commands], [1])
        self.module.unload()
        self.roi_tick(2)
        self.assertFalse(self.module.roi.targets)
        self.assertFalse(self.module.roi.inflight)

    def test_roi_timeout_only_cancels_its_mount_and_waits_for_late_reply(self):
        self.setup_camera_roi()
        self.module.cmd_camera(["roi", "all"])
        second = self.module.roi.targets[(1, 101)]
        for _ in range(3):
            self.roi_tick(1)
        self.assertNotIn((1, 100), self.module.roi.targets)
        self.assertEqual(second.attempts, 0)
        self.roi_ack()  # delayed success after timeout
        self.assertFalse(second.confirmed)
        self.roi_tick(1.1)
        self.assertEqual(second.attempts, 1)
        self.roi_ack()
        self.assertTrue(second.confirmed)

    def test_roi_other_gcs_and_manager_ack_ignored_progress_does_not_advance(self):
        self.setup_camera_roi()
        mav = self.state._master.mav
        mav.srcSystem, mav.srcComponent = 255, 230
        self.module.cmd_camera(["roi", "all"])
        first = self.module.roi.targets[(1, 100)]
        second = self.module.roi.targets[(1, 101)]
        self.roi_ack(system=42)
        self.roi_ack(component=154)
        self.roi_ack(target_system=254, target_component=230)
        self.roi_ack(target_system=255, target_component=231)
        self.assertFalse(first.confirmed)
        self.roi_tick(0.8)
        self.roi_ack(mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                     target_system=255, target_component=230)
        self.roi_tick(0.8)
        self.assertEqual((first.attempts, second.attempts), (1, 0))
        self.roi_ack(target_system=255, target_component=230)
        self.roi_tick()
        self.assertEqual(second.attempts, 1)

    def test_roi_different_managers_can_progress_independently(self):
        self.setup_camera_roi()
        self.module.mavlink_packet(camera_information(system_id=42))
        self.module.cmd_camera(["roi", "all"])
        third = self.module.roi.targets[(42, 100)]
        self.assertEqual(third.attempts, 1)
        self.roi_ack(system=42)
        self.assertTrue(third.confirmed)
        self.assertFalse(self.module.roi.targets[(1, 100)].confirmed)
        self.assertEqual(self.module.roi.targets[(1, 101)].attempts, 0)

    def test_camera_actions_use_verified_fc_slots(self):
        self.setup_camera_roi()
        # Deliberately reverse CAM slots: component order is not slot order.
        self.state.mav_param_by_sysid = {(1, 1): {
            "CAM1_TYPE": 6, "CAM1_COMPID": 101, "CAM2_TYPE": 6, "CAM2_COMPID": 100}}
        cases = [(["photo"], 2000, 0), (["photo", "0.5", "3"], 2000, 0),
                 (["stopphotos"], 2001, 0), (["record", "start"], 2500, 0),
                 (["record", "stop"], 2501, 0), (["zoom", "63"], 531, 2),
                 (["zoom", "in"], 531, 2), (["focus", "auto"], 532, 2),
                 (["focus", "25"], 532, 2)]
        for component, slot in ((100, 2), (101, 1)):
            for args, command, selector in cases:
                with self.subTest(component=component, args=args):
                    self.module.cmd_camera(["for", "1:%u" % component] + args)
                    sent = self.commands()[-1]
                    self.assertEqual(sent[:3], (1, 1, command))
                    self.assertEqual(sent[4 + selector], slot)
            # Commands the FC cannot proxy retain the routed camera address.
            for args in (["source", "thermal"], ["mode", "video"], ["stream", "start", "2"], ["stream", "stop", "2"]):
                self.module.cmd_camera(["for", "1:%u" % component] + args)
                self.assertEqual(self.commands()[-1][:2], (1, component))
        self.assertFalse(self.state._master.mav.param_sets)

    def test_camera_slot_unknown_wrong_vehicle_or_ambiguous_stays_addressed(self):
        self.setup_camera_roi()
        cases = [{}, {"CAM1_TYPE": 6}, {"CAM1_TYPE": 1, "CAM1_COMPID": 101},
                 {"CAM1_TYPE": 6, "CAM1_COMPID": 101, "CAM2_TYPE": 6, "CAM2_COMPID": 101}]
        for params in cases:
            self.state.mav_param_by_sysid = {
                (1, 1): params, (42, 1): {"CAM1_TYPE": 6, "CAM1_COMPID": 101},
                (1, 100): {"CAM1_TYPE": 6, "CAM1_COMPID": 101}}
            self.module.cmd_camera(["for", "1:101", "photo"])
            self.assertEqual(self.commands()[-1][:2], (1, 101))
            self.assertEqual(self.commands()[-1][4], 0)
        self.state.mav_param_by_sysid = {(1, 1): {"CAM2_TYPE": 6, "CAM2_COMPID": 0}}
        self.module.cmd_camera(["for", "1:101", "photo"])
        self.assertEqual(self.commands()[-1][:2], (1, 1))
        self.assertEqual(self.commands()[-1][4], 2)

    def test_camera_controls(self):
        self.module.mavlink_packet(camera_information())
        cases = [
            (["photo"], mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE),
            (["photo", "0.5", "3"], mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE),
            (["stopphotos"], mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE),
            (["record", "start"], mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE),
            (["record", "stop"], mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE),
            (["zoom", "63"], mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM),
            (["zoom", "in"], mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM),
            (["focus", "auto"], mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS),
            (["focus", "25"], mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS),
            (["mode", "video"], mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE),
            (["source", "thermal"], mavutil.mavlink.MAV_CMD_SET_CAMERA_SOURCE),
            (["stream", "stop", "2"], mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING),
        ]
        for args, command in cases:
            before = len(self.commands())
            self.module.cmd_camera(args)
            self.assertEqual(len(self.commands()), before + 1, args)
            sent = self.commands()[-1]
            self.assertEqual(sent[:3], (1, 100, command))
        zoom = self.commands(mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM)[0]
        self.assertEqual(zoom[4], mavutil.mavlink.ZOOM_TYPE_RANGE)
        self.assertEqual(zoom[5], 63.0)

    def test_record_toggle_tracks_state_and_verifies_status(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(Message(
            "CAMERA_CAPTURE_STATUS", image_status=0, video_status=0,
            image_interval=0.0, recording_time_ms=0,
            available_capacity=100.0, image_count=0))

        self.module.cmd_camera(["record", "toggle"])
        self.assertEqual(
            self.commands()[-1][2],
            mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE)
        camera = self.module.cameras[(1, 100)]
        self.assertTrue(camera.recording)

        # Ignore a pre-command status packet which arrived out of order.
        self.module.mavlink_packet(Message(
            "CAMERA_CAPTURE_STATUS", image_status=0, video_status=0,
            image_interval=0.0, recording_time_ms=0,
            available_capacity=100.0, image_count=0))
        self.assertTrue(camera.recording)

        # A second toggle before status is relayed must stop, rather than
        # consulting the previous CAMERA_CAPTURE_STATUS and starting again.
        self.module.cmd_camera(["record", "toggle"])
        self.assertEqual(
            self.commands()[-1][2],
            mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE)
        self.assertFalse(camera.recording)

        camera.recording_verify_at = time.time() - 1.0
        before = len(self.commands(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE))
        self.module.idle_task()
        requests = self.commands(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)
        self.assertGreater(len(requests), before)
        self.assertTrue(any(
            request[4] ==
            mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS
            for request in requests[before:]))
        self.assertEqual(camera.recording_verify_at, 0.0)


    def test_mount_retains_manager_for_running_or_unknown_stream(self):
        self.setup_camera_roi()
        for params in ({}, {"MNT2_TARG_RATE": 0}, {"MNT2_TARG_RATE": 10}):
            self.state.mav_param_by_sysid = {(1, 1): params,
                                           (42, 1): {"MNT2_TARG_RATE": 0},
                                           (1, 100): {"MNT2_TARG_RATE": 0}}
            self.module.cmd_camera(["for", "1:101", "mount", "angle", "0", "90"])
            command = self.commands(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[-1]
            self.assertEqual(command[:2], (1, 1))
            self.assertEqual(command[10], 2)
            self.assertFalse(self.state._master.mav.gimbal_attitudes)


    def test_manager_targets_vehicle_and_ignores_proxy_gimbal(self):
        self.state.settings.target_system = 7
        self.module.mavlink_packet(camera_information(
            system_id=42, component_id=1, gimbal_device_id=1))
        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS", system_id=42, component_id=1,
            q=[1.0, 0.0, 0.0, 0.0]))
        self.assertNotIn((42, 1), self.module.gimbals)

        self.module.cmd_camera(["mount", "angle", "-12", "8", "body"])
        command = self.commands(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[0]
        self.assertEqual(command[:3], (42, 1,
                         mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW))

        self.module.mavlink_packet(Message(
            "GIMBAL_DEVICE_INFORMATION", system_id=42, component_id=154,
            model_name="MT11"))
        self.assertIn((42, 154), self.module.gimbals)
        self.assertEqual(self.module.selected_gimbal, (42, 154))

    def test_status_updates_and_rejected_ack(self):
        self.module.mavlink_packet(camera_information())
        self.module.mavlink_packet(Message(
            "CAMERA_CAPTURE_STATUS", image_status=0, video_status=1,
            image_interval=0.0, recording_time_ms=20,
            available_capacity=100.0, image_count=4))
        self.assertTrue(self.module.cameras[(1, 100)].recording)
        self.assertIn("REC", self.state.console.status["CAMERA"])
        self.module.cmd_camera(["zoom", "50"])
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.mavlink_packet(Message(
                "COMMAND_ACK", command=mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
                result=mavutil.mavlink.MAV_RESULT_UNSUPPORTED))
        self.assertIn("MAV_CMD_SET_CAMERA_ZOOM", output.getvalue())
        self.assertIn("MAV_RESULT_UNSUPPORTED", output.getvalue())

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.mavlink_packet(Message(
                "COMMAND_ACK",
                command=mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                result=mavutil.mavlink.MAV_RESULT_FAILED,
                component_id=1))
        self.assertEqual(output.getvalue(), "")

    def test_periodic_request_message_acks_are_quiet(self):
        link = SimpleNamespace(settings=SimpleNamespace(
            all_vehicle_command_acks=False,
            source_system=255,
            source_component=0,
        ))
        for result in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                       mavutil.mavlink.MAV_RESULT_FAILED):
            ack = SimpleNamespace(
                command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                result=result,
                target_system=255,
                target_component=0,
            )
            self.assertFalse(LinkModule.should_show_command_ack(link, ack))


if __name__ == "__main__":
    unittest.main()
