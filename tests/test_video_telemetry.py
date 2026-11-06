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
    UUID, FlatElevation, Sample, TelemetryAlignment, VideoIndex, ViewProjection, attitude, camera_pose,
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



class AlignmentTests(unittest.TestCase):
    def timeline(self, utc_values=(100, 101, 102), video_times=(0, .1, .2)):
        from types import SimpleNamespace
        samples = []
        for n, (utc, pts) in enumerate(zip(utc_values, video_times)):
            rec = record()
            rec['utc_us'] = round(utc * 1e6) if utc is not None else None
            rec['position']['lat_e7'] += n * 100
            samples.append(Sample(pts, .1, rec, n * 10, 'test'))
        index = SimpleNamespace(samples=samples, times=list(video_times))
        return index, TelemetryAlignment(index)

    def test_offset_uses_utc_seconds_and_interpolates_wrapped_angles(self):
        index, alignment = self.timeline()
        index.samples[0].record['heading_rad'] = math.radians(350)
        index.samples[1].record['heading_rad'] = math.radians(10)
        index.samples[0].record['gimbal_attitude']['yaw_rad'] = math.radians(350)
        index.samples[1].record['gimbal_attitude']['yaw_rad'] = math.radians(10)
        original = copy.deepcopy([s.record for s in index.samples])
        sample, basis = alignment.at(0, .5)
        self.assertEqual(basis, 'UTC')
        self.assertEqual(sample.record['utc_us'], 100500000)
        self.assertEqual(sample.record['position']['lat_e7'], original[0]['position']['lat_e7'] + 50)
        self.assertAlmostEqual(sample.time, .05)  # not +0.5 playback seconds
        self.assertAlmostEqual(math.sin(sample.record['heading_rad']), 0)
        self.assertAlmostEqual(math.cos(sample.record['gimbal_attitude']['yaw_rad']), 1)
        self.assertAlmostEqual(sample.speed, 5)
        self.assertEqual(sample.record['position']['age_ms'], 500)
        self.assertEqual([s.record for s in index.samples], original)
        earlier, basis = alignment.at(2, -1)
        self.assertIs(earlier, index.samples[1])

    def test_zero_keeps_missing_frame_telemetry_and_image_metadata(self):
        index, alignment = self.timeline()
        index.samples[1].record = None
        sample, _ = alignment.at(1, 0)
        self.assertIs(sample, index.samples[1])
        self.assertIsNone(sample.record)
        self.assertIs(alignment.at(0, 0)[0], index.samples[0])

    def test_outside_coverage_and_large_utc_gap_have_no_pose(self):
        index, alignment = self.timeline((100, 120, 121))
        self.assertIsNone(alignment.at(0, -1)[0])
        self.assertIsNone(alignment.at(2, 1)[0])
        self.assertIsNone(alignment.at(0, 10)[0])
        self.assertIs(alignment.at(0, 20)[0], index.samples[1])
        with self.assertRaises(ValueError):
            alignment.at(0, math.nan)

    def test_interpolates_missing_seconds_in_vendor_rgb_utc(self):
        index, alignment = self.timeline((1787886130, 1787886135, 1787886136))
        shifted, basis = alignment.at(0, 3)
        self.assertEqual(basis, 'UTC')
        self.assertEqual(shifted.record['utc_us'], 1787886133000000)
        self.assertAlmostEqual(shifted.record['position']['lat_e7'], index.samples[0].record['position']['lat_e7'] + 60)

    def test_missing_or_regressing_utc_uses_video_time(self):
        for timestamps in ((None, None, None), (100, 99, 101)):
            index, alignment = self.timeline(timestamps, (0, 1, 2))
            sample, basis = alignment.at(1, .5)
            self.assertEqual(basis, 'video time')
            self.assertEqual(sample.record['position']['lat_e7'], index.samples[1].record['position']['lat_e7'] + 50)

    def test_appended_background_samples_make_offset_available(self):
        index, alignment = self.timeline((100,), (0,))
        self.assertIsNone(alignment.at(0, 1)[0])
        rec = record()
        rec['utc_us'] = 101000000
        index.samples.append(Sample(1, .1, rec))
        index.times.append(1)
        self.assertIs(alignment.at(0, 1)[0], index.samples[1])

    def test_late_background_metadata_invalidates_cached_alignment(self):
        index, alignment = self.timeline()
        index.indexing, index.telemetry_count = True, 2
        late = index.samples[1].record
        index.samples[1].record = None
        self.assertIsNot(alignment.at(0, 1)[0], index.samples[1])
        index.samples[1].record = late
        index.telemetry_count += 1
        self.assertIs(alignment.at(0, 1)[0], index.samples[1])
        shifted, basis = alignment.at(0, .5)
        self.assertEqual(basis, 'UTC')
        self.assertAlmostEqual(shifted.speed, 5)

    def test_finished_index_refreshes_interpolated_speed(self):
        index, alignment = self.timeline()
        index.indexing = True
        self.assertAlmostEqual(alignment.at(0, .5)[0].speed, 5)
        index.samples[1].speed = 20
        index.indexing = False
        self.assertAlmostEqual(alignment.at(0, .5)[0].speed, 10)

    def test_longitude_interpolates_across_date_line(self):
        index, alignment = self.timeline()
        index.samples[0].record['position']['lon_e7'] = 1799999900
        index.samples[1].record['position']['lon_e7'] = -1799999900
        shifted, _ = alignment.at(0, .5)
        self.assertAlmostEqual(abs(position(shifted.record)[1]), 180)

    def test_repeated_utc_keeps_current_sample_at_small_offset(self):
        index, alignment = self.timeline((100, 100, 101), (0, .1, .2))
        shifted, _ = alignment.at(1, .1)
        self.assertAlmostEqual(shifted.record['position']['lat_e7'], index.samples[1].record['position']['lat_e7'] + 10)


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


class ThermalMetadataTests(unittest.TestCase):
    """Reading the apcg.telemetry.v1 snapshot from a thermal frame's Matroska
    block addition, as thermal_to_video.py writes it (an 8-byte addition-type
    prefix and the apcg.thermal.v1 JSON)."""
    class Packet:
        def __init__(self, blob):
            self.blob = blob
        def get_sidedata(self, name):
            assert name == 'matroska_block_additional'
            if self.blob is None:
                raise KeyError(name)
            return self.blob

    def blob(self, meta):
        return (0x41504347).to_bytes(8, 'big') + json.dumps(meta).encode()

    def parse(self, meta_or_blob):
        index = VideoIndex.__new__(VideoIndex)  # only _thermal_record is exercised
        blob = meta_or_blob if isinstance(meta_or_blob, (bytes, type(None))) else self.blob(meta_or_blob)
        return index._thermal_record(self.Packet(blob) if not isinstance(meta_or_blob, VideoIndex) else meta_or_blob)

    def test_reads_nested_telemetry_and_fills_thermal_fov(self):
        meta = dict(schema='apcg.thermal.v1', hfov_deg=24.2, telemetry=dict(record(), hfov_deg=None))
        got = self.parse(meta)
        self.assertEqual(got['position']['lat_e7'], record()['position']['lat_e7'])
        self.assertEqual(got['gimbal_attitude']['pitch_rad'], record()['gimbal_attitude']['pitch_rad'])
        self.assertEqual(got['hfov_deg'], 24.2)  # taken from the outer thermal metadata

    def test_keeps_nested_fov_when_present(self):
        meta = dict(schema='apcg.thermal.v1', hfov_deg=24.2, telemetry=dict(record(), hfov_deg=42.0))
        self.assertEqual(self.parse(meta)['hfov_deg'], 42.0)

    def test_preserves_radiometric_scale_without_assuming_calibration(self):
        meta = dict(schema='apcg.thermal.v1', telemetry=record(),
                    temperature_scale_k=1/64, temperature_offset_k=2,
                    minimum_c=-10, maximum_c=30)
        self.assertEqual(self.parse(meta)['thermal'], {key: meta[key] for key in (
            'temperature_scale_k', 'temperature_offset_k', 'minimum_c', 'maximum_c')})
        meta['temperature_scale_k'] = None
        self.assertIsNone(self.parse(meta)['thermal']['temperature_scale_k'])

    def test_rejects_unusable_metadata(self):
        for meta in (dict(schema='other', telemetry=record()),
                     dict(schema='apcg.thermal.v1', telemetry=None),
                     dict(schema='apcg.thermal.v1', telemetry=dict(schema='x'))):
            self.assertIsNone(self.parse(meta))
        wrong_type = (1).to_bytes(8, 'big') + json.dumps(
            dict(schema='apcg.thermal.v1', telemetry=record())).encode()
        self.assertIsNone(self.parse(wrong_type))
        self.assertIsNone(self.parse(b'\x00\x00'))  # too short
        self.assertIsNone(self.parse(None))           # side data absent
        index = VideoIndex.__new__(VideoIndex)
        self.assertIsNone(index._thermal_record(object()))  # packet without get_sidedata


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'ffmpeg required')
class RecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.video = str(Path(cls.temp.name) / 'bframes.mp4')
        # A real inter-frame-coded video with reordered packets and an arbitrary
        # telemetry pts90k epoch. Inject SEI using FFmpeg's own bitstream filter.
        text = json.dumps(dict(record(), hfov_deg=60), separators=(',', ':')).replace(',', '\\,').replace(':', '\\:')
        bsf = "h264_metadata=sei_user_data='" + UUID.hex() + '+' + text + "'"
        try:
            subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                            '-t', '2', '-c:v', 'libx264', '-bf', '2', '-g', '5', '-bsf:v', bsf,
                            cls.video], check=True)
        except subprocess.CalledProcessError as error:
            # ffmpeg before 5.0 splits the bsf list on every comma, so the
            # escaped JSON payload cannot be injected
            raise unittest.SkipTest('ffmpeg cannot inject the SEI bitstream filter (exit %d); '
                                    'ffmpeg 5.0 or later is required' % error.returncode)

    def test_sei_only_mpegts_bframe_seeks_use_decode_timestamps(self):
        try:
            import av
        except ImportError:
            self.skipTest('PyAV required for MPEG-TS playback')
        from MAVProxy.modules.lib.mavvidplay import FrameReader
        source_path = str(Path(self.temp.name)/'source-bframes.ts')
        video = str(Path(self.temp.name)/'sei-bframes.ts')
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                        '-t', '2', '-c:v', 'libx264', '-bf', '2', '-g', '5', source_path], check=True)
        with av.open(source_path) as source, av.open(video, 'w', format='mpegts') as output:
            stream = output.add_stream_from_template(source.streams.video[0])
            first = True
            for packet in source.demux(video=0):
                if not packet.size:
                    continue
                if first:
                    nals = list(packet_nals(bytes(packet), 0))
                    nals.insert(1 if nals[0][0]&31 == 9 else 0, sei(dict(record(), hfov_deg=60)))
                    data = av.Packet(b''.join(b'\0\0\0\1'+nal for nal in nals))
                    data.pts, data.dts, data.duration, data.time_base = packet.pts, packet.dts, packet.duration, packet.time_base
                    packet, first = data, False
                packet.stream = stream
                output.mux(packet)
        index = VideoIndex(video)
        self.assertEqual(len(index.samples), 20)
        self.assertGreater(index.telemetry_count, 0)
        self.assertTrue(any(index.keyframe_dts[pts] != pts for pts in index.keyframe_pts))
        with av.open(video) as source:
            reference = [frame.to_ndarray(format='bgr24') for frame in source.decode(video=0)]
        reader = FrameReader(index)
        try:
            for generation, target in enumerate((0, 12, 3, 19, 1)):
                reader.request(target, generation)
                number, gen, image, error = reader.results.get(timeout=10)
                self.assertIsNone(error)
                self.assertEqual((number, gen), (target, generation))
                self.assertTrue((image == reference[target]).all())
        finally:
            reader.stop()
            index.close()

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
