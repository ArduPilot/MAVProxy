"""Offline regression tests for recorded telemetry, picking and seek decoding."""
import copy
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from MAVProxy.modules.lib.video_telemetry import (
    UUID, FlatElevation, Sample, VideoIndex, ViewProjection, attitude, camera_pose,
    image_pixel, image_rect, packet_nals, position, sei_records, unhex)


def record():
    return dict(schema='apcg.telemetry.v1', utc_us=1000000, pts90k=123456789,
                position=dict(lat_e7=-353632610, lon_e7=1491652300, alt_amsl_m=650,
                              alt_relative_m=100, age_ms=0),
                vehicle_attitude=dict(roll_rad=.1, pitch_rad=.2, yaw_rad=math.pi / 2, age_ms=0),
                gimbal_attitude=dict(roll_rad=0, pitch_rad=-math.pi / 2, yaw_rad=.1, age_ms=0),
                heading_rad=math.pi / 2, zoom=1)


def sei(value, codec='h264', suffix=False):
    payload = UUID + json.dumps(value).encode()
    size = bytes([255]) * (len(payload) // 255) + bytes([len(payload) % 255])
    raw = b'\x05' + size + payload + b'\x80'
    escaped = bytearray()
    zeros = 0
    for byte in raw:
        if zeros == 2 and byte <= 3:
            escaped.append(3)
            zeros = 0
        escaped.append(byte)
        zeros = zeros + 1 if byte == 0 else 0
    return (b'\x06' if codec == 'h264' else bytes([(40 if suffix else 39) << 1, 1])) + escaped


class TelemetryTests(unittest.TestCase):
    def test_sei_codecs_and_packet_framing(self):
        for codec in ('h264', 'hevc'):
            for suffix in (False, True):
                nal = sei(record(), codec, suffix)
                for length in (1, 2, 3, 4):
                    if len(nal) >= 256 ** length:
                        continue
                    packet = len(nal).to_bytes(length, 'big') + nal
                    self.assertEqual(list(packet_nals(packet, length)), [nal])
                self.assertEqual(list(sei_records(nal, codec)), [record()])
                self.assertEqual(list(packet_nals(b'\0\0\1' + nal + b'\0\0\0\1' + nal, 0)), [nal, nal])

    def test_malformed_and_unrelated_sei(self):
        for nal in (b'', b'\x06\xff', b'\x06\x05\xff\xff', b'\x06\x05\x20short',
                    sei(record()).replace(UUID, b'x' * 16), sei({'schema': 'other'}),
                    sei(record()).replace(b'apcg', b'\xffpcg')):
            self.assertEqual(list(sei_records(nal, 'h264')), [])
        self.assertEqual(list(packet_nals(b'\0\0\0\xffshort', 4)), [])

    def test_hex_dump(self):
        value = '\n00000000: 0001 0203 0405 0607 0809 0a0b 0c0d 0e0f  ................\n00000010: 10                                       .\n'
        self.assertEqual(unhex(value), bytes(range(17)))
        self.assertEqual(unhex(value.replace('\n', ' ')), bytes(range(17)))

    def test_pose_and_missing_sources(self):
        rec = record()
        pose = camera_pose(rec)
        self.assertAlmostEqual(pose[4], -90)
        self.assertAlmostEqual(pose[5], 90 + math.degrees(.1))
        for key in ('position', 'vehicle_attitude', 'gimbal_attitude'):
            bad = copy.deepcopy(rec)
            bad[key] = None
            self.assertIsNone(camera_pose(bad))
            bad[key] = dict(rec[key], age_ms=10001)
            self.assertIsNone(camera_pose(bad))
        self.assertIsNone(position(dict(rec, position=dict(rec['position'], lat_e7=910000000))))
        self.assertIsNone(attitude(dict(rec, gimbal_attitude=dict(rec['gimbal_attitude'], pitch_rad=math.nan)), 'gimbal_attitude'))

    def test_speed_uses_fix_time_and_survives_repeated_snapshots(self):
        idx = VideoIndex.__new__(VideoIndex)
        idx.samples = []
        for utc, age, lat in ((1000000, 0, 0), (1100000, 100, 0),
                              (1200000, 0, 100), (1300000, 100, 100),
                              (1400000, 0, 100), (1500000, 0, 150)):
            rec = record()
            rec['utc_us'] = utc
            rec['position']['age_ms'] = age
            rec['position']['lat_e7'] += lat
            idx.samples.append(Sample(utc * 1e-6, .1, rec))
        idx._speeds()
        self.assertIsNone(idx.samples[0].speed)
        self.assertIsNone(idx.samples[1].speed)
        self.assertAlmostEqual(idx.samples[2].speed, 5.566, delta=.02)
        self.assertEqual(idx.samples[2].speed, idx.samples[3].speed)
        self.assertEqual(idx.samples[4].speed, 0)
        self.assertAlmostEqual(idx.samples[5].speed, 5.566, delta=.02)

    def test_letterbox_and_resize(self):
        rect = image_rect(1000, 800, 1920, 1080)
        self.assertEqual(rect, (0, 119, 1000, 562))
        self.assertIsNone(image_pixel(rect, 1920, 1080, 500, 118))
        self.assertIsNone(image_pixel(rect, 1920, 1080, 1000, 500))
        self.assertEqual(image_pixel(rect, 1920, 1080, 500, 400), (960, 540))
        self.assertEqual(image_pixel(image_rect(400, 800, 1920, 1080), 1920, 1080, 200, 399.5), (960, 540))

    def test_nadir_pixel_and_footprint(self):
        rec = record()
        view = ViewProjection(640, 480, 60, FlatElevation(550))
        center = view.pixel(rec, 320, 240)
        self.assertAlmostEqual(center[0], position(rec)[0], places=6)
        self.assertAlmostEqual(center[1], position(rec)[1], places=6)
        self.assertAlmostEqual(center[2], 550)
        self.assertGreater(len(view.footprint(rec)), 3)
        self.assertIsNone(view.pixel(rec, -1, 200))
        rec['gimbal_attitude']['pitch_rad'] = math.pi / 2
        self.assertIsNone(view.pixel(rec, 320, 240))
        self.assertIsNone(view.footprint(rec))

    def test_recorded_fov_tracks_frames_without_double_zoom(self):
        rec = record()
        rec['zoom'] = 10
        rec['hfov_deg'] = 60
        view = ViewProjection(640, 480, None, FlatElevation(550), zoom_fov=True)
        first = view.pixel(rec, 600, 240)
        self.assertEqual(view.effective_fov(rec), 60)
        rec['hfov_deg'] = 30
        second = view.pixel(rec, 600, 240)
        self.assertEqual(view.effective_fov(rec), 30)
        self.assertNotEqual(first, second)
        override = ViewProjection(640, 480, 80, FlatElevation(550))
        self.assertEqual(override.effective_fov(rec), 80)
        for value in (None, 0, -1, 180, math.nan, math.inf, '60'):
            rec['hfov_deg'] = value
            self.assertIsNone(view.pixel(rec, 320, 240))
            self.assertIsNone(view.footprint(rec))
        del rec['hfov_deg']
        self.assertIsNone(view.effective_fov(rec))

    def test_zoom_and_range(self):
        rec = record()
        view = ViewProjection(640, 480, 60, FlatElevation(550), zoom_fov=True)
        first = view.pixel(rec, 600, 240)
        rec['zoom'] = 2
        second = view.pixel(rec, 600, 240)
        from MAVProxy.modules.lib.mp_util import gps_distance
        pos = position(rec)
        self.assertAlmostEqual(gps_distance(*pos[:2], *first[:2]) / gps_distance(*pos[:2], *second[:2]), 2, places=3)
        self.assertIsNone(ViewProjection(640, 480, 60, FlatElevation(550), max_range=50).pixel(rec, 320, 240))


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'ffmpeg required')
class RecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.video = str(Path(cls.temp.name) / 'bframes.mp4')
        # A real inter-frame-coded video with reordered packets and an arbitrary
        # telemetry pts90k epoch. Inject SEI using FFmpeg's own bitstream filter.
        text = json.dumps(dict(record(), hfov_deg=60), separators=(',', ':')).replace(',', '\\,').replace(':', '\\:')
        bsf = "h264_metadata=sei_user_data='" + UUID.hex() + '+' + text + "'"
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                        '-t', '2', '-c:v', 'libx264', '-bf', '2', '-g', '5', '-bsf:v', bsf,
                        cls.video], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_index_and_decoder_seeking(self):
        import cv2
        from MAVProxy.modules.lib.mavvidplay import FrameReader
        idx = VideoIndex(self.video)
        self.assertEqual(len(idx.samples), 20)
        self.assertGreater(idx.telemetry_count, 0)
        self.assertEqual(idx.frame_at(.75), 7)
        self.assertEqual(idx.frame_at(-10), 0)
        self.assertEqual(idx.frame_at(100), 19)
        cap = cv2.VideoCapture(self.video)
        reference = []
        while True:
            ok, image = cap.read()
            if not ok:
                break
            reference.append(image)
        cap.release()
        reader = FrameReader(self.video)
        try:
            for generation, target in enumerate((0, 15, 3, 19, 1, 8)):
                reader.request(target, generation)
                number, actual_generation, image, error = reader.results.get(timeout=10)
                self.assertIsNone(error)
                self.assertEqual((number, actual_generation), (target, generation))
                self.assertTrue((image == reference[target]).all())
        finally:
            reader.stop()
        self.assertFalse(reader.is_alive())

    def test_hevc_recording(self):
        raw = str(Path(self.temp.name) / 'source.hevc')
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                        '-t', '1', '-c:v', 'libx265', '-x265-params',
                        'bframes=0:keyint=5:pools=1:frame-threads=1:log-level=error',
                        '-f', 'hevc', raw], check=True)
        output = bytearray()
        frame = 0
        for nal in packet_nals(Path(raw).read_bytes(), 0):
            if (nal[0] >> 1) & 63 <= 31 and nal[2] & 0x80:
                rec = record()
                rec['pts90k'] = frame * 9000
                rec['utc_us'] += frame * 100000
                rec['position']['lat_e7'] += frame * 100
                output.extend(b'\0\0\0\1' + sei(rec, 'hevc'))
                frame += 1
            output.extend(b'\0\0\0\1' + nal)
        injected = str(Path(self.temp.name) / 'injected.hevc')
        Path(injected).write_bytes(output)
        target = str(Path(self.temp.name) / 'hevc.mp4')
        subprocess.run(['ffmpeg', '-v', 'error', '-r', '10', '-i', injected,
                        '-c', 'copy', target], check=True)
        idx = VideoIndex(target)
        self.assertEqual(idx.codec, 'hevc')
        self.assertEqual(idx.telemetry_count, 10)
        self.assertEqual(len(idx.samples), 10)
        for i, sample in enumerate(idx.samples):
            self.assertEqual(sample.record['pts90k'], i * 9000)
            self.assertAlmostEqual(sample.time, i * .1)
        self.assertAlmostEqual(idx.samples[1].speed, 11.13, delta=.03)

    def test_remux_matroska(self):
        original = VideoIndex(self.video)
        for extension in ('mkv',):
            target = str(Path(self.temp.name) / ('remux.' + extension))
            subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', self.video, '-c', 'copy', target], check=True)
            idx = VideoIndex(target)
            self.assertEqual(len(idx.samples), len(original.samples))
            self.assertEqual(idx.telemetry_count, original.telemetry_count)
            self.assertEqual(idx.samples[0].time, 0)
            for actual, expected in zip(idx.samples, original.samples):
                self.assertAlmostEqual(actual.time, expected.time, places=5)
                self.assertEqual(actual.record, expected.record)


if __name__ == '__main__':
    unittest.main()
