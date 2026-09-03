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
from MAVProxy.modules.mavproxy_link import LinkModule  # noqa: E402
from pymavlink import mavutil  # noqa: E402


class FakeMav:
    def __init__(self):
        self.commands = []
        self.gimbal_attitudes = []

    def command_long_send(self, *args):
        self.commands.append(args)

    def gimbal_device_set_attitude_send(self, *args):
        self.gimbal_attitudes.append(args)


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
        self.assertEqual(len(visible.points), 5)
        self.assertEqual(len(thermal.points), 5)
        self.assertEqual(visible.colour, (0, 128, 128))
        self.assertEqual(thermal.colour, (0, 0, 128))
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
        image.set_video.assert_called_once_with(uri)
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
        self.assertIn("rtspsrc location=" + uri, pipeline)
        self.assertIn("latency=75 protocols=tcp", pipeline)
        self.assertIn("rtph265depay", pipeline)
        self.assertIn("h265parse ! avdec_h265", pipeline)
        image.set_video.assert_not_called()

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

    def test_mount_manager_and_direct_device_control(self):
        self.module.mavlink_packet(camera_information())
        self.module.cmd_camera(["mount", "angle", "-20", "35", "earth"])
        command = self.commands(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[0]
        self.assertEqual(command[:3], (1, 1,
                         mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW))
        self.assertEqual(command[4:6], (-20.0, 35.0))
        self.assertEqual(command[8], mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
        self.assertEqual(command[10], 0)

        self.module.camera_settings.mount_control = "device"
        self.module.cmd_camera(["mount", "rate", "4", "-8", "body"])
        direct = self.state._master.mav.gimbal_attitudes[-1]
        self.assertEqual(direct[:3], (1, 154, 0))
        self.assertAlmostEqual(direct[5], math.radians(4))
        self.assertAlmostEqual(direct[6], math.radians(-8))

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
        self.assertEqual(command[:3], (7, 1,
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
