#!/usr/bin/env python3

import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import struct
import numpy as np
from pymavlink import mavutil

from MAVProxy.modules.mavproxy_SIYI import (
    ABSOLUTE_ZOOM,
    ACQUIRE_GIMBAL_CONFIG_INFO,
    ACQUIRE_GIMBAL_ATTITUDE,
    AUTO_FOCUS,
    CAMERA_TYPE_MT11,
    CAMERA_TYPE_ZT30,
    DEFAULT_CAMERA_TYPE,
    FUNCTION_FEEDBACK_INFO,
    GET_IMAGE_TYPE,
    GET_THERMAL_GAIN,
    GET_ZOOM_VALUE,
    GIMBAL_ROTATION,
    GPS_EXTERNAL,
    READ_CONTROL_MODE,
    READ_GIMBAL_MODE,
    READ_THRESHOLDS,
    READ_TEMP_FULL_SCREEN,
    REQUEST_CONTINUOUS_DATA,
    PHOTO,
    SET_ANGLE,
    SET_THERMAL_GAIN,
    SET_IMAGE_TYPE,
    SET_LASER_STATUS,
    SIYIModule,
    SIYI_HEADER1,
    SIYI_HEADER2,
    ACQUIRE_FIRMWARE_VERSION,
    HARDWARE_ID,
    crc16_from_bytes,
    decode_firmware_versions,
    image_mode_payload,
    rate_mapping,
)
from MAVProxy.modules.mavproxy_SIYI.camera_view import (
    ffmpeg_http_command,
    rtsp_gstreamer_pipeline,
)
from MAVProxy.modules.mavproxy_SIYI.raw_thermal import RawThermal


class TestSIYIProtocolProfiles(unittest.TestCase):
    def test_default_camera_is_mt11(self):
        self.assertEqual(DEFAULT_CAMERA_TYPE, CAMERA_TYPE_MT11)

    def test_rawthermal_uses_separate_ip_setting(self):
        siyi = SimpleNamespace(
            siyi_settings=SimpleNamespace(therm_ip='192.168.1.25'),
            logdir='/tmp')
        with patch('MAVProxy.modules.mavproxy_SIYI.raw_thermal.MPImage'), \
                patch('MAVProxy.modules.mavproxy_SIYI.raw_thermal.Thread'):
            view = RawThermal(siyi, (640, 512))
            default_view = RawThermal(None, (640, 512))
        self.assertEqual(view.uri, ('192.168.1.25', 7345))
        self.assertEqual(default_view.uri, ('192.168.144.25', 7345))

    def test_firmware_version_words(self):
        data = bytes.fromhex('0c0001890600018a00000000')
        self.assertEqual(decode_firmware_versions(data),
                         ((1, 0, 12), (1, 0, 6), (0, 0, 0)))
        with self.assertRaises(ValueError):
            decode_firmware_versions(data[:-1])

    def test_image_mode_profiles(self):
        payload, name, slots = image_mode_payload(CAMERA_TYPE_MT11, ['thermal'])
        self.assertEqual((payload, name, slots), (b'\x02\x00', 'thermal', (2, 0)))
        payload, _, slots = image_mode_payload(CAMERA_TYPE_MT11, ['3', '2'])
        self.assertEqual((payload, slots), (b'\x03\x02', (3, 2)))
        with self.assertRaises(ValueError):
            image_mode_payload(CAMERA_TYPE_MT11, ['wide'])
        payload, name, slots = image_mode_payload(CAMERA_TYPE_ZT30, ['wide'])
        self.assertEqual((payload, name, slots), (b'\x05', 'wide', None))

    def make_module(self, camera_type):
        module = SIYIModule.__new__(SIYIModule)
        module.siyi_settings = SimpleNamespace(
            camera_type=camera_type,
            ip='192.168.144.25',
            therm_ip='192.168.144.25',
            transport='auto',
            rtsp_rgb='auto',
            rtsp_thermal='auto',
            rtsp_codec='auto',
            yaw_rate=10.0,
            pitch_rate=10.0,
            temp_hz=5.0,
            telem_rate=4,
            mode_hz=1.0,
            rates_hz=5.0,
            target_rate=10.0,
            target_control='rate',
            att_control=0,
            mount_yaw=0.0,
            mount_pitch=0.0,
            mount_alt=0.0,
            max_rate=30.0,
            therm_cap_rate=1.0,
            mt11_temp_autoswap=True,
            track_ROI=0,
        )
        module.sent = []
        module.last_zoom = 1.0
        module.rgb_lens = "wide"
        module.image_slots = None
        module.mt11_zoom_state = None
        module.mt11_zoom_target = None
        module.mt11_zoom_actual = None
        module.mt11_zoom_started = 0
        module.mt11_zoom_last_action = 0
        module.last_therm_cap = 0
        module.last_mt11_temp_request = 0
        module.last_mode_t = 0
        module.requested_control_mode = None
        module.requested_control_mode_t = 0
        module.thermal_gain = None
        module.requested_thermal_gain = None
        module.requested_thermal_gain_t = 0
        module.last_thermal_gain_request = 0
        module.last_req_send = 0
        module.last_target_send = 0
        module.active_target_control = None
        module.start_time = time.time()
        module.thermal_view = None
        module.rawthermal_view = None
        module.rgb_view = None
        module.logf = SimpleNamespace(write=lambda *args: None)
        module.send_named_float = lambda *args: None

        def capture(command, fmt, *args):
            module.sent.append((command, fmt, args))

        module.send_packet_fmt = capture
        return module

    def packet(self, command, data):
        packet = struct.pack('<BBBHHB', SIYI_HEADER1, SIYI_HEADER2, 1,
                             len(data), 42, command) + data
        return packet + struct.pack('<H', crc16_from_bytes(packet))

    def make_parser(self, camera_type):
        module = self.make_module(camera_type)
        module.logf = SimpleNamespace(write=lambda *args: None)
        module.start_time = time.time()
        module.bad_crc = 0
        module.have_version = False
        module.hardware_id = None
        module.image_slots = None
        module.control_mode = -1
        module.send_named_float = lambda *args: None
        module.update_status = lambda: None
        return module

    def test_mt11_gimbal_modes_are_confirmed_from_config(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        module.mpstate = SimpleNamespace(
            console=SimpleNamespace(set_status=lambda *args, **kwargs: None),
            master=lambda: SimpleNamespace(motors_armed=lambda: False),
        )
        module.getconfig_pending = False
        module.clear_target = lambda: None

        for command_value, mode in ((3, 0), (4, 1), (5, 2)):
            module.sent = []
            module.cmd_gimbal_mode(command_value, mode)
            self.assertEqual(
                module.sent, [(PHOTO, '<B', (command_value,))])
            self.assertEqual(module.requested_control_mode, mode)

            module.parse_packet(self.packet(
                ACQUIRE_GIMBAL_CONFIG_INFO,
                bytes((0, 0, 0, 0, mode, 1, 0, 0))))
            self.assertEqual(module.control_mode, mode)
            self.assertIsNone(module.requested_control_mode)

    def test_mt11_ignores_invalid_reported_gimbal_rates(self):
        raw = struct.pack('<hhhhhh', 123, -456, 7,
                          32640, 8164, 16281)
        module = self.make_parser(CAMERA_TYPE_MT11)
        module.last_att_t = 9.9
        module.att_dt_lpf = 0.1
        named = {}
        module.send_named_float = lambda name, value: named.__setitem__(name,
                                                                       value)
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   return_value=10.0):
            module.parse_packet(self.packet(ACQUIRE_GIMBAL_ATTITUDE, raw))

        for actual, expected in zip(module.attitude[:3],
                                    (0.7, -45.6, -12.3)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(module.attitude[3:], (0.0, 0.0, 0.0))
        self.assertEqual(named['CROLL_RT'], 0.0)
        self.assertEqual(named['CPITCH_RT'], 0.0)
        self.assertEqual(named['CYAW_RT'], 0.0)

        # Also reject stale non-zero rates which predate a camera-type change.
        module.attitude = (1.0, 2.0, 3.0, 1000.0, 2000.0, 3000.0)
        module.siyi_settings.lag = 1.0
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   return_value=20.0):
            self.assertEqual(module.get_direct_attitude(), (1.0, 2.0, 3.0))

    def test_zt30_preserves_reported_gimbal_rates(self):
        raw = struct.pack('<hhhhhh', 123, -456, 7, 30, -20, 10)
        module = self.make_parser(CAMERA_TYPE_ZT30)
        module.last_att_t = 9.9
        module.att_dt_lpf = 0.1
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   return_value=10.0):
            module.parse_packet(self.packet(ACQUIRE_GIMBAL_ATTITUDE, raw))

        for actual, expected in zip(module.attitude[:3],
                                    (0.7, -45.6, -12.3)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(module.attitude[3:], (1.0, -2.0, -3.0))

    def test_mt11_rates_enter_lock_and_use_zero_stop_value(self):
        self.assertEqual(rate_mapping(0), 0)
        module = self.make_module(CAMERA_TYPE_MT11)
        module.clear_target = lambda: None

        module.cmd_rates(['10', '0'])
        self.assertEqual(module.sent, [
            (PHOTO, '<B', (3,)),
            (GIMBAL_ROTATION, '<bb', (10, 0)),
        ])

        module.sent = []
        module.cmd_rates(['0', '0'])
        self.assertEqual(module.sent, [
            (GIMBAL_ROTATION, '<bb', (0, 0)),
        ])

    def test_mt11_thermal_capture_uses_photo_at_requested_rate(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.therm_cap_rate = 2.0
        module.last_therm_cap = 9.0

        with patch('MAVProxy.modules.mavproxy_SIYI.time.time', return_value=10.0):
            module.therm_capture()
        self.assertEqual(module.sent, [(PHOTO, '<B', (0,))])
        self.assertEqual(module.last_therm_cap, 10.0)

        module.sent = []
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time', return_value=10.4):
            module.therm_capture()
        self.assertEqual(module.sent, [])

        module.siyi_settings.therm_cap_rate = 0
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time', return_value=11.0):
            module.therm_capture()
        self.assertEqual(module.sent, [])

    def test_mt11_capture_feedback_updates_tcap_without_printing(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        statuses = []
        module.mpstate = SimpleNamespace(console=SimpleNamespace(
            set_status=lambda *args, **kwargs: statuses.append((args, kwargs))))
        # Aircraft Lua can originate captures while MAVProxy's own scheduler
        # is disabled, so success handling must not depend on therm_cap_rate.
        module.siyi_settings.therm_cap_rate = 0
        module.thermal_capture_count = 7

        with patch('builtins.print') as output:
            module.parse_packet(self.packet(FUNCTION_FEEDBACK_INFO, b'\x00'))
        output.assert_not_called()
        self.assertEqual(module.thermal_capture_count, 8)
        self.assertEqual(statuses, [
            (('TCAP', 'TCAP 8'), {'row': 6}),
        ])

        statuses.clear()
        with patch('builtins.print') as output:
            module.parse_packet(self.packet(FUNCTION_FEEDBACK_INFO, b'\x01'))
        output.assert_called_once_with('Feedback FailPhoto')
        self.assertEqual(module.thermal_capture_count, 8)

    def test_angle_stops_rate_control_first(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.clear_target = lambda: None
        module.cmd_angle(['30', '-20'])
        self.assertEqual(module.sent, [
            (GIMBAL_ROTATION, '<bb', (0, 0)),
            (SET_ANGLE, '<hh', (-300, -200)),
        ])

    def test_target_attitude_uses_siyi_sign_and_mount_offsets(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.mount_yaw = 2.0
        module.siyi_settings.mount_pitch = -1.0
        module.set_target_attitude(10.0, -20.0)
        self.assertEqual(module.sent, [
            (SET_ANGLE, '<hh', (-120, -210)),
        ])

    def test_target_control_legacy_setting_alias(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.setting_changed(SimpleNamespace(name='att_control', value=1))
        self.assertEqual(module.siyi_settings.target_control, 'attitude')
        module.setting_changed(SimpleNamespace(name='target_control',
                                               value='rate'))
        self.assertEqual(module.siyi_settings.att_control, 0)

    def test_target_control_auto_defaults_by_camera_type(self):
        mt11 = self.make_module(CAMERA_TYPE_MT11)
        mt11.siyi_settings.target_control = 'auto'
        mt11.setting_changed(SimpleNamespace(name='target_control',
                                             value='auto'))
        self.assertEqual(mt11.target_control_mode(), 'lua')
        self.assertEqual(mt11.siyi_settings.att_control, 0)

        zt30 = self.make_module(CAMERA_TYPE_ZT30)
        zt30.siyi_settings.target_control = 'auto'
        zt30.setting_changed(SimpleNamespace(name='target_control',
                                             value='auto'))
        self.assertEqual(zt30.target_control_mode(), 'rate')
        self.assertEqual(zt30.siyi_settings.att_control, 0)

    def test_mt11_lua_target_sends_absolute_terrain_roi_once(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.target_control = 'lua'
        sent_roi = []

        class Mav:
            def command_int_send(self, *args):
                sent_roi.append(args)

        class Map:
            def add_object(self, obj):
                self.obj = obj

        terrain = SimpleNamespace(ElevationModel=SimpleNamespace(
            GetElevation=lambda lat, lon: 612.75))
        module.mpstate = SimpleNamespace(
            click_location=(-35.1234567, 149.7654321),
            map=Map(),
            settings=SimpleNamespace(target_system=3, target_component=4),
            master=lambda: SimpleNamespace(mav=Mav(), messages={}))
        module.module = lambda name: terrain if name == 'terrain' else None
        module.icon = np.zeros((2, 2, 3), dtype=np.uint8)
        module.target_pos = None

        module.cmd_settarget([])

        self.assertIsNone(module.target_pos)
        self.assertEqual(len(sent_roi), 1)
        roi = sent_roi[0]
        self.assertEqual(roi[:4], (
            3, 4,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION))
        self.assertEqual(roi[10], -351234567)
        self.assertEqual(roi[11], 1497654321)
        self.assertEqual(roi[12], 612.75)

        module.clear_roi_location()
        self.assertEqual(len(sent_roi), 2)
        self.assertEqual(sent_roi[1][:4], (
            3, 4,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE))

    def test_clear_target_always_clears_aircraft_and_map_roi(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        # Simulate changing away from Lua control while an aircraft-side ROI
        # established by Lua is still active.
        module.siyi_settings.target_control = 'rate'
        sent_roi = []

        class Mav:
            def command_int_send(self, *args):
                sent_roi.append(args)

        class Map:
            current_ROI = (-35.0, 149.0, 600.0)

            def remove_object(self, _name):
                pass

        map_module = Map()
        module.mpstate = SimpleNamespace(
            map=map_module,
            settings=SimpleNamespace(target_system=3, target_component=4),
            master=lambda: SimpleNamespace(mav=Mav(), messages={}))
        module.module = lambda name: map_module if name == 'map' else None
        module.end_tracking = lambda: None
        module.target_pos = (-35.0, 149.0, 600.0)
        module.last_map_ROI = map_module.current_ROI
        module.yaw_rate = 1.0
        module.pitch_rate = 2.0
        module.active_target_control = 'lua'

        module.clear_target()

        self.assertEqual(len(sent_roi), 1)
        self.assertEqual(sent_roi[0][:4], (
            3, 4,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE))
        self.assertIsNone(map_module.current_ROI)
        self.assertIsNone(module.last_map_ROI)
        self.assertIsNone(module.target_pos)
        self.assertIsNone(module.yaw_rate)
        self.assertIsNone(module.pitch_rate)
        self.assertIsNone(module.active_target_control)

    def test_manual_and_mode_commands_clear_target(self):
        cases = (
            ('center', lambda m: m.cmd_siyi(['center'])),
            ('resetattitude', lambda m: m.cmd_siyi(['resetattitude'])),
            ('lock', lambda m: m.cmd_siyi(['lock'])),
            ('follow', lambda m: m.cmd_siyi(['follow'])),
            ('fpv', lambda m: m.cmd_siyi(['fpv'])),
            ('rates', lambda m: m.cmd_rates(['10', '0'])),
            ('rates-stop', lambda m: m.cmd_rates(['0', '0'])),
            ('yaw', lambda m: m.cmd_yaw(['5'])),
            ('pitch', lambda m: m.cmd_pitch(['5'])),
            ('angle', lambda m: m.cmd_angle(['10', '-20'])),
            ('retract', lambda m: m.retract()),
        )
        for name, command in cases:
            with self.subTest(command=name):
                module = self.make_module(CAMERA_TYPE_MT11)
                clears = []
                module.clear_target = lambda: clears.append(True)
                command(module)
                self.assertEqual(clears, [True])

    def test_terrain_altitude_rejects_missing_and_nonfinite_data(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        with patch('builtins.print'):
            module.module = lambda _name: None
            self.assertIsNone(module.terrain_altitude(-35, 149))
            module.module = lambda _name: SimpleNamespace(
                ElevationModel=SimpleNamespace(
                    GetElevation=lambda _lat, _lon: float('nan')))
            self.assertIsNone(module.terrain_altitude(-35, 149))
        module.module = lambda _name: SimpleNamespace(
            ElevationModel=SimpleNamespace(
                GetElevation=lambda _lat, _lon: 123))
        self.assertEqual(module.terrain_altitude(-35, 149), 123.0)

    def test_attitude_target_is_sent_at_configured_rate(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.target_control = 'attitude'
        module.siyi_settings.target_rate = 10.0
        module.siyi_settings.track_ROI = 0
        module.target_pos = (-35.0, 149.0, 500.0)
        module.attitude = (0, 0, 0, 0, 0, 0)
        module.get_target_yaw_pitch = lambda *args: (12.3, -45.6)
        module.module = lambda _name: None
        master = SimpleNamespace(messages={
            'GLOBAL_POSITION_INT': SimpleNamespace(
                lat=-350000000, lon=1490000000, vx=0, vy=0, vz=0,
                _timestamp=100.0),
            'GPS_RAW_INT': SimpleNamespace(alt=600000),
            'ATTITUDE': SimpleNamespace(
                yaw=0.0, yawspeed=0.0, _timestamp=100.0),
        })
        module.mpstate = SimpleNamespace(master=lambda: master)

        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   side_effect=(100.0, 100.05, 100.11)):
            module.update_target()
            module.update_target()
            module.update_target()

        self.assertEqual(module.sent, [
            (GIMBAL_ROTATION, '<bb', (0, 0)),
            (SET_ANGLE, '<hh', (-123, -456)),
            (SET_ANGLE, '<hh', (-123, -456)),
        ])

    def test_autofocus_payloads(self):
        mt11 = self.make_module(CAMERA_TYPE_MT11)
        mt11.cmd_autofocus(['320', '256'])
        self.assertEqual(mt11.sent, [(AUTO_FOCUS, '<BHH', (1, 320, 256))])
        zt30 = self.make_module(CAMERA_TYPE_ZT30)
        zt30.cmd_autofocus([])
        self.assertEqual(zt30.sent, [(AUTO_FOCUS, '<B', (1,))])

    def test_mt11_thermal_gain_is_confirmed_by_readback(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   return_value=10.0), patch('builtins.print'):
            module.cmd_thermal_gain(['0'])
        self.assertEqual(module.sent, [(SET_THERMAL_GAIN, '<B', (0,))])
        self.assertEqual(module.requested_thermal_gain, 0)

        module.sent = []
        module.update_thermal_gain(10.05)
        self.assertEqual(module.sent, [])
        module.update_thermal_gain(10.2)
        self.assertEqual(module.sent, [(GET_THERMAL_GAIN, None, ())])

        with patch('builtins.print') as output:
            module.parse_packet(self.packet(GET_THERMAL_GAIN, b'\x00'))
        output.assert_called_once_with(
            'SIYI: thermal gain confirmed: Low Gain (high-temperature range)')
        self.assertEqual(module.thermal_gain, 0)
        self.assertIsNone(module.requested_thermal_gain)

    def test_mt11_lens_modes_preserve_temperature_slots(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        module.image_slots = (2, 0)
        module.last_zoom = 4.0
        module.rgb_lens = "zoom"

        module.cmd_imode(["wide"])
        self.assertEqual(module.last_zoom, 4.0)
        self.assertEqual(module.rgb_lens, "zoom")
        self.assertEqual(module.sent, [(SET_IMAGE_TYPE, "<BB", (0, 2))])

        module.sent = []
        module.parse_packet(self.packet(SET_IMAGE_TYPE, b'\x00\x02'))
        self.assertEqual(module.mt11_zoom_state, 'zooming')
        self.assertEqual(module.sent, [(ABSOLUTE_ZOOM, "<BB", (1, 0))])

        module.sent = []
        module.parse_packet(self.packet(GET_ZOOM_VALUE, b'\x01\x00'))
        self.assertIsNone(module.mt11_zoom_state)
        self.assertEqual(module.sent, [])
        self.assertEqual(module.image_slots, (0, 2))
        self.assertEqual(module.last_zoom, 1.0)
        self.assertEqual(module.rgb_lens, "wide")

        module.cmd_imode(["zoom"])
        self.assertEqual(module.sent, [(ABSOLUTE_ZOOM, "<BB", (2, 0))])
        module.sent = []
        module.parse_packet(self.packet(GET_ZOOM_VALUE, b'\x02\x00'))
        self.assertIsNone(module.mt11_zoom_state)
        self.assertEqual(module.last_zoom, 2.0)
        self.assertEqual(module.rgb_lens, "zoom")

    def test_mt11_gps_packet_units(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.start_time = time.time()
        master = SimpleNamespace(messages={
            'GLOBAL_POSITION_INT': SimpleNamespace(
                time_boot_ms=1234, lat=-353000000, lon=1490000000,
                alt=612345, vx=123, vy=-45, vz=67),
            'GPS_RAW_INT': SimpleNamespace(alt_ellipsoid=650005),
        })
        module.mpstate = SimpleNamespace(master=lambda: master)
        module.send_gps()
        self.assertEqual(
            module.sent,
            [(GPS_EXTERNAL, '<Iiiiiiii',
              (1234, -353000000, 1490000000, 61234, 65000,
               1230, -450, 670))])

    def test_rtsp_defaults_follow_mt11_slots(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.image_slots = (2, 0)
        self.assertEqual(module.rtsp_url(True),
                         'rtsp://192.168.144.25:8554/video1')
        self.assertEqual(module.rtsp_url(False),
                         'rtsp://192.168.144.25:8554/video2')
        self.assertEqual(module.rtsp_codec(), 'h264')
        module.image_slots = (0, 2)
        self.assertEqual(module.rtsp_url(True),
                         'rtsp://192.168.144.25:8554/video2')
        self.assertEqual(module.rtsp_url(False),
                         'rtsp://192.168.144.25:8554/video1')

    def initialise_telemetry_state(self, module):
        old = time.time() - 20
        module.last_temp_t = old
        module.last_image_mode_request = old
        module.last_att_t = old
        module.last_rf_t = old
        module.last_enc_t = old
        module.last_volt_t = old
        module.last_thresh_t = old
        module.last_mode_t = old
        module.last_therm_mode = old
        module.mt11_image_mode_user_set = False

    def test_mt11_telemetry_uses_mt11_commands(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        self.initialise_telemetry_state(module)
        module.image_slots = (0, 2)
        module.request_telem()
        commands = [entry[0] for entry in module.sent]
        self.assertIn(READ_TEMP_FULL_SCREEN, commands)
        self.assertIn(READ_GIMBAL_MODE, commands)
        self.assertNotIn(READ_CONTROL_MODE, commands)
        self.assertNotIn(READ_THRESHOLDS, commands)
        module.sent = []
        module.request_telem()
        self.assertNotIn(READ_TEMP_FULL_SCREEN,
                         [entry[0] for entry in module.sent])

    def test_mt11_telemetry_never_polls_lidar(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        self.initialise_telemetry_state(module)
        module.image_slots = (0, 2)
        # Simulate a stale pre-Lua setting loaded by older code.
        module.siyi_settings.use_lidar = 1
        module.sent = []

        module.request_telem()

        self.assertNotIn((SET_LASER_STATUS, '<B', (1,)), module.sent)
        self.assertNotIn((REQUEST_CONTINUOUS_DATA, '<BB', (2, 1)),
                         module.sent)

    def test_mt11_telemetry_does_not_steal_lua_attitude_stream(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        self.initialise_telemetry_state(module)
        module.image_slots = (0, 2)
        module.sent = []

        module.request_telem()

        self.assertNotIn((REQUEST_CONTINUOUS_DATA, '<BB',
                          (1, module.siyi_settings.telem_rate)), module.sent)

    def test_mt11_named_attitude_updates_projection_state(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.attitude = (1.0, 2.0, 3.0, 100.0, 200.0, 300.0)
        module.last_att_t = 9.9
        module.att_dt_lpf = 0.1
        module.update_status = lambda: None
        module.armed_checks = lambda: None

        def named(name, value):
            return SimpleNamespace(get_type=lambda: 'NAMED_VALUE_FLOAT',
                                   name=name, value=value)

        with patch('MAVProxy.modules.mavproxy_SIYI.time.time',
                   return_value=10.0):
            module.mavlink_packet(named(b'CROLL\x00\x00\x00\x00\x00', 4.0))
            module.mavlink_packet(named('CPITCH', 5.0))
            module.mavlink_packet(named('CYAW', 6.0))

        self.assertEqual(module.attitude, (4.0, 5.0, 6.0,
                                           0.0, 0.0, 0.0))
        self.assertEqual(module.last_att_t, 10.0)

    def test_mt11_temperature_autoswap(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        self.initialise_telemetry_state(module)
        module.image_slots = (2, 0)
        module.request_telem()
        self.assertIn((SET_IMAGE_TYPE, '<BB', (0, 2)), module.sent)
        self.assertNotIn(READ_TEMP_FULL_SCREEN,
                         [entry[0] for entry in module.sent])
        module.sent = []
        module.image_slots = None
        module.last_image_mode_request = 0
        module.request_telem()
        self.assertIn((GET_IMAGE_TYPE, None, ()), module.sent)

    def test_zt30_telemetry_preserves_private_commands(self):
        module = self.make_module(CAMERA_TYPE_ZT30)
        self.initialise_telemetry_state(module)
        module.image_slots = None
        module.request_telem()
        commands = [entry[0] for entry in module.sent]
        self.assertIn(READ_TEMP_FULL_SCREEN, commands)
        self.assertIn(READ_CONTROL_MODE, commands)
        self.assertIn(READ_THRESHOLDS, commands)
        self.assertNotIn(READ_GIMBAL_MODE, commands)

    def test_codec_specific_gstreamer_pipeline(self):
        h264 = rtsp_gstreamer_pipeline('rtsp://camera/video1', 'out.mts', 'h264')
        self.assertIn('rtph264depay', h264)
        self.assertIn('avdec_h264', h264)
        self.assertNotIn('h265', h264)
        with self.assertRaises(ValueError):
            rtsp_gstreamer_pipeline('url', 'out', 'vp9')

    def test_supportproxy_http_mpegts_command(self):
        url = 'http://firevps.tridgell.net:40005/v3.ts'
        command = ffmpeg_http_command(url, 'out.ts', (1280, 720))
        self.assertEqual(command[0], 'ffmpeg')
        self.assertIn(url, command)
        self.assertIn('out.ts', command)
        self.assertIn('scale=1280:720', command)
        self.assertEqual(command[-1], 'pipe:1')
        self.assertIn('copy', command)
        self.assertIn('rawvideo', command)

    def test_mt11_version_and_slot_responses(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        module.parse_packet(self.packet(ACQUIRE_FIRMWARE_VERSION,
                                        bytes.fromhex('0c0001890600018a00000000')))
        self.assertTrue(module.have_version)
        commands = [entry[0] for entry in module.sent]
        self.assertIn(HARDWARE_ID, commands)
        self.assertIn(GET_IMAGE_TYPE, commands)
        module.parse_packet(self.packet(GET_IMAGE_TYPE, b'\x00\x02'))
        self.assertEqual(module.image_slots, (0, 2))

    def test_tcp_parser_preserves_fragmented_packets(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        packet = self.packet(GET_IMAGE_TYPE, b'\x02\x00')
        remainder = module.parse_data(packet[:5])
        self.assertEqual(remainder, packet[:5])
        remainder = module.parse_data(remainder + packet[5:])
        self.assertEqual(remainder, b'')
        self.assertEqual(module.image_slots, (2, 0))

    def test_projection_helpers_tolerate_missing_gps(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.mount_alt = 0
        master = SimpleNamespace(messages={})
        module.mpstate = SimpleNamespace(master=lambda: master)
        module.module = lambda _name: SimpleNamespace(ElevationModel=None)
        module.get_fov_attitude = lambda: (0, 0, 0)
        with patch(
                'MAVProxy.modules.mavproxy_SIYI.camera_projection.CameraProjection'):
            self.assertIsNone(module.get_slantrange(0, 0, 1, 1))
            self.assertIsNone(module.get_latlonalt(1, 0, 0, 1, 1))
            self.assertIsNone(module.show_fov1(60, 'test', 1, (0, 0, 0)))

        master.messages = {
            'ATTITUDE': SimpleNamespace(),
            'GLOBAL_POSITION_INT': SimpleNamespace(),
        }
        module.update_target()

    def test_mt11_raw_autoflag_uses_threshold_and_correct_xy_slice(self):
        view = RawThermal.__new__(RawThermal)
        view.res = (640, 512)
        view.tmax = 100.0
        view.marker_history = []
        # Baseline 20C with one hot pixel in the top-right slice. The old
        # transposed slice indices did not inspect this region correctly.
        view.last_data = np.full((512, 640), 20.0 + 273.15)
        view.last_data[20, 600] = 100.0 + 273.15
        flags = []
        settings = SimpleNamespace(
            autoflag_enable=True,
            threshold_temp=50.0,
            autoflag_slices=4,
            autoflag_dist=30.0,
            autoflag_history=50)
        view.siyi = SimpleNamespace(
            siyi_settings=settings,
            have_DATA96=False,
            module=lambda name: object() if name == 'map' else None,
            add_thermal_flag=lambda *args: flags.append(args))
        view.xy_to_latlon = lambda x, y: (-35.0 + y * 1.0e-6,
                                          149.0 + x * 1.0e-6,
                                          600.0)

        view.handle_auto_flag()

        self.assertEqual(len(flags), 1)
        latlonalt, temperature, pixel_x, pixel_y = flags[0]
        self.assertEqual((pixel_x, pixel_y), (560, 64))
        self.assertAlmostEqual(temperature, 100.0)
        self.assertEqual(latlonalt, (-34.999936, 149.00056, 600.0))

    def test_mt11_data96_autoflag_uses_threshold_and_camera_altitude(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        module.siyi_settings.autoflag_enable = True
        module.siyi_settings.threshold_temp = 50.0
        module.siyi_settings.thermal_fov = 24.2
        module.siyi_settings.mount_alt = 1.5
        module.siyi_settings.autoflag_radius = 100.0
        module.last_map_ROI = None
        module.module = lambda _name: SimpleNamespace(ElevationModel=object())
        flags = []
        module.add_thermal_flag = lambda *args, **_kwargs: flags.append(args)
        projection_calls = []

        class Projection:
            def __init__(self, _params, elevation_model=None):
                self.elevation_model = elevation_model

            def get_slantrange(self, *args):
                projection_calls.append(('slant', args))
                return 120.0

            def get_latlonalt_for_pixel(self, *args):
                projection_calls.append(('pixel', args))
                return (-35.0001, 149.0002, 610.0)

        def data96(tmax):
            return struct.pack(
                '<iiifffHHffffff',
                1234, -350000000, 1490000000, 600.0, 605.0,
                tmax, 600, 20,
                1.0, -20.0, 3.0, 2.0, -1.0, 10.0)

        with patch(
                'MAVProxy.modules.mavproxy_SIYI.camera_projection.CameraProjection',
                Projection):
            module.process_DATA96(data96(80.0))
            module.process_DATA96(data96(49.9))

        self.assertEqual(flags, [((-35.0001, 149.0002, 610.0),
                                  80.0, 600, 20)])
        self.assertEqual(len(projection_calls), 2)
        self.assertAlmostEqual(projection_calls[0][1][2], 606.5)
        self.assertAlmostEqual(projection_calls[1][1][4], 606.5)

    def test_thermal_flame_marker_has_dataflash_log(self):
        module = self.make_module(CAMERA_TYPE_MT11)
        added = []
        map_mock = SimpleNamespace(
            icon=lambda _name: np.zeros((2, 2, 3), dtype=np.uint8),
            add_object=lambda obj: added.append(obj))
        module.mpstate = SimpleNamespace(map=map_mock)
        module.thermal_flag_count = 0
        logs = []
        module.logf = SimpleNamespace(write=lambda *args: logs.append(args))
        module.micros64 = lambda: 123456

        module.add_thermal_flag((-35.25, 149.5, 601.0), 88.5, 320, 256)

        self.assertEqual(len(added), 1)
        self.assertEqual(logs, [(
            'SIFL', 'QLLffHH', 'TimeUS,Lat,Lng,Alt,TMax,X,Y',
            123456, -352500000, 1495000000, 601.0, 88.5, 320, 256)])

    def test_mt11_zero_version_is_not_a_handshake(self):
        module = self.make_parser(CAMERA_TYPE_MT11)
        module.parse_packet(self.packet(ACQUIRE_FIRMWARE_VERSION, bytes(12)))
        self.assertFalse(module.have_version)
        self.assertEqual(module.sent, [])


if __name__ == '__main__':
    unittest.main()
