#!/usr/bin/env python3
"""SIYI driver integration with the real AP_CameraGimbal host application.

Build AP_CameraGimbal with `make sitl`, then run:
    MT11_SITL_REPO=/path/to/AP_CameraGimbal python3 -m unittest discover \
        -s tests -p test_mavproxy_siyi_sitl.py -v
MT11_SITL_BINARY optionally selects a separately built camera-app executable.
Each test starts its own camera and gimbal with a temporary runtime filesystem.
"""

import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from MAVProxy.modules import mavproxy_SIYI as siyi
from MAVProxy.modules.mavproxy_SIYI import test_siyi as fixtures
from MAVProxy.modules.mavproxy_SIYI.camera_view import ffmpeg_rtsp_command


@unittest.skipUnless(os.environ.get('MT11_SITL_REPO'), 'set MT11_SITL_REPO')
class SIYISITLTest(unittest.TestCase):
    def reserve_port(self, kind):
        with socket.socket(socket.AF_INET, kind) as sock:
            sock.bind(('127.0.0.1', 0))
            return sock.getsockname()[1]

    def start_process(self, command, ready, env=None):
        log = open(self.runtime / (ready.name + '.log'), 'w+')
        self.addCleanup(log.close)
        process = subprocess.Popen(command, env=env, stdout=log,
                                   stderr=subprocess.STDOUT)

        def stop():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        self.addCleanup(stop)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if process.poll() is not None:
                log.seek(0)
                self.fail(log.read())
            if ready.exists():
                return
            time.sleep(0.02)
        log.seek(0)
        self.fail('SITL readiness timeout: ' + log.read())

    def setUp(self):
        repo = Path(os.environ['MT11_SITL_REPO']).expanduser().resolve()
        binary = Path(os.environ.get('MT11_SITL_BINARY',
                                    str(repo / 'build/sitl/camera-app')))
        self.assertTrue(binary.is_file(), 'build camera-app with make sitl')
        temp = tempfile.TemporaryDirectory(prefix='mavproxy-mt11-')
        self.addCleanup(temp.cleanup)
        self.runtime = Path(temp.name)
        subprocess.run([sys.executable, str(repo / 'sitl/prepare_runtime.py'),
                        str(self.runtime), str(repo / 'camera_app/camera.ini')],
                       check=True)
        gimbal_port = self.reserve_port(socket.SOCK_DGRAM)
        self.port = self.reserve_port(socket.SOCK_STREAM)
        self.rtsp_port = self.reserve_port(socket.SOCK_STREAM)
        self.start_process(
            [sys.executable, str(repo / 'sitl/gimbal_sim.py'), '--port',
             str(gimbal_port), '--ready-file', str(self.runtime / 'gimbal.ready')],
            self.runtime / 'gimbal.ready')
        env = os.environ.copy()
        env.update({
            'CAMERA_APP_UART': 'udp://127.0.0.1:%u' % gimbal_port,
            'CAMERA_APP_PORT': str(self.port),
            'CAMERA_APP_RTSP_PORT': str(self.rtsp_port),
            'CAMERA_APP_MAVLINK_TCP_PORT': '0',
            'CAMERA_APP_MAVLINK_UDP_PORT': '0',
            'CAMERA_APP_CONFIG': str(self.runtime / 'app/camera.ini'),
            'CAMERA_APP_READY_PATH': str(self.runtime / 'camera.ready'),
            'CAMERA_APP_RECORD_STATE': str(self.runtime / 'run/recording.state'),
            'CAMERA_APP_RECORD_ROOT': str(self.runtime / 'mnt/DCIM/record'),
            'CAMERA_APP_CAPTURE_ROOT': str(self.runtime / 'mnt/DCIM/capture'),
            'CAMERA_APP_SITL_RGB_VIDEO': str(repo / 'build/sitl/rgb.h264'),
            'CAMERA_APP_SITL_THERMAL_VIDEO': str(repo / 'build/sitl/thermal.h264'),
            'CAMERA_APP_SITL_PHOTO': str(repo / 'build/sitl/photo.jpg'),
        })
        self.start_process([str(binary)], self.runtime / 'camera.ready', env)
        # Reuse the headless driver fixture, restoring its real wire sender.
        self.module = fixtures.TestSIYIProtocolProfiles().make_parser(
            siyi.CAMERA_TYPE_MT11)
        module = self.module
        del module.send_packet_fmt
        module.sock = None
        module.sequence = 0
        module.attitude = None
        module.last_att_t = time.time()
        module.att_dt_lpf = 0.1
        module.thermal_capture_count = 0
        module.getconfig_pending = False
        module.mpstate = SimpleNamespace(
            map=Mock(), console=Mock(),
            master=lambda: SimpleNamespace(motors_armed=lambda: False))
        # There is no aircraft in this camera-only test.
        module.clear_target = Mock()
        module.siyi_settings.ip = '127.0.0.1'
        module.siyi_settings.port = self.port
        self.replies = {}
        parse_packet = module.parse_packet

        def capture(packet):
            parse_packet(packet)
            self.replies[packet[7]] = packet[8:-2]
        module.parse_packet = capture
        self.addCleanup(lambda: module.sock.close() if module.sock else None)

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.module.send_tcp_heartbeat()
            if select.select([self.module.sock], [], [], 0.05)[0]:
                data = self.module.sock.recv(65536)
                self.assertTrue(data, 'camera closed connection')
                self.module.recv_buffer = self.module.parse_data(
                    self.module.recv_buffer + data)
        self.fail('camera response timeout; replies: %r' % self.replies)

    def request(self, opcode):
        self.replies.pop(opcode, None)
        self.module.send_packet_fmt(opcode, None)
        self.wait_for(lambda: opcode in self.replies)
        return self.replies[opcode]

    def exercise_driver(self, transport):
        module = self.module
        module.siyi_settings.transport = transport
        module.cmd_connect()
        self.assertEqual(module.sock_is_tcp, transport == 'tcp')
        self.request(siyi.ACQUIRE_FIRMWARE_VERSION)
        self.wait_for(lambda: module.hardware_id is not None)
        self.assertTrue(module.have_version)
        self.request(siyi.ACQUIRE_GIMBAL_ATTITUDE)
        module.cmd_siyi(['angle', '15', '-25'])
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.request(siyi.ACQUIRE_GIMBAL_ATTITUDE)
            if abs(module.attitude[1] + 25) < 1 and abs(module.attitude[2] - 15) < 1:
                break
            time.sleep(0.05)
        self.assertAlmostEqual(module.attitude[1], -25, delta=1)
        self.assertAlmostEqual(module.attitude[2], 15, delta=1)
        self.request(siyi.ACQUIRE_GIMBAL_CONFIG_INFO)
        self.assertEqual(module.control_mode, 0)

        for mode, slots in (('wide', (1, 2)), ('zoom', (0, 2)),
                            ('thermal', (2, 0))):
            module.cmd_siyi(['imode', mode])
            self.wait_for(lambda: module.image_slots == slots and
                          module.mt11_image_mode_target is None)
            self.assertEqual(module.image_slots, slots)

        module.cmd_siyi(['imode', 'zoom'])
        self.wait_for(lambda: module.image_slots == (0, 2) and
                      module.mt11_image_mode_target is None)
        module.cmd_siyi(['zoom', '2.5'])
        self.assertEqual(self.request(siyi.GET_ZOOM_VALUE), b'\x02\x05')

        module.cmd_siyi(['focus', 'far', '0.05'])
        self.wait_for(lambda: siyi.MANUAL_FOCUS in self.replies)
        self.assertEqual(self.replies[siyi.MANUAL_FOCUS], b'\x01')
        time.sleep(0.06)
        self.replies.pop(siyi.MANUAL_FOCUS)
        module.check_focus_end()
        self.wait_for(lambda: siyi.MANUAL_FOCUS in self.replies)
        self.assertIsNone(module.focus_end)
        module.cmd_siyi(['autofocus'])
        self.wait_for(lambda: siyi.AUTO_FOCUS in self.replies)
        self.assertEqual(self.replies[siyi.AUTO_FOCUS], b'\x01')

        module.cmd_siyi(['photo'])
        self.wait_for(lambda: module.thermal_capture_count == 1)
        self.assertEqual(len(list((self.runtime / 'mnt/DCIM/capture').glob('*.jpg'))), 3)
        self.replies.pop(siyi.FUNCTION_FEEDBACK_INFO, None)
        with patch('builtins.print') as output:
            module.cmd_siyi(['hdr'])
            self.wait_for(lambda: siyi.FUNCTION_FEEDBACK_INFO in self.replies)
        self.assertEqual(self.replies[siyi.FUNCTION_FEEDBACK_INFO], b'\x03')
        output.assert_any_call('Feedback HDR OFF')
        self.assertEqual(module.thermal_capture_count, 1)

        module.siyi_settings.show_lidar_target = True
        module.request_lidar_target(time.time())
        self.wait_for(lambda: module.lidar_target is not None)
        self.assertAlmostEqual(module.lidar_target[0], -35.1234567)
        self.assertAlmostEqual(module.lidar_target[1], 149.1234567)
        self.assertEqual(module.mpstate.map.add_object.call_args.args[0].key,
                         'SIYILidarTarget')
        self.assertEqual(module.bad_crc, 0)
        module.sock.close()
        module.sock = None
        module.idle_task()
        self.assertIsNone(module.lidar_target)

    def test_tcp_driver(self):
        self.exercise_driver('tcp')
        for stream in ('video1', 'video2'):
            recording = str(self.runtime / (stream + '.ts'))
            command = ffmpeg_rtsp_command(
                'rtsp://127.0.0.1:%u/%s' % (self.rtsp_port, stream),
                recording, (320, 180))
            # Bound both outputs of the driver's recording/viewing command.
            index = command.index(recording)
            command[index:index] = ['-t', '0.3']
            command[-1:-1] = ['-frames:v', '1']
            result = subprocess.run(command, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertEqual(len(result.stdout), 320 * 180 * 3)
            self.assertGreater(Path(recording).stat().st_size, 0)

    def test_udp_driver(self):
        self.exercise_driver('udp')


if __name__ == '__main__':
    unittest.main()
