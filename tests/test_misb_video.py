"""MISB decoding, PTS association and real MPEG-TS seek regressions."""
from contextlib import contextmanager
from fractions import Fraction
import importlib.util
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from MAVProxy.modules.lib.misb import UAS_KEY, records
from MAVProxy.modules.lib.video_telemetry import VideoIndex, camera_pose, position


def local_set(items):
    body = b''.join(bytes([tag, len(value)])+value for tag, value in items)
    body += b'\x01\x02'
    size = len(body)+2
    ber = bytes([size]) if size < 128 else bytes([0x81, size])
    packet = UAS_KEY+ber+body
    checksum = sum(byte << (8 if i % 2 == 0 else 0) for i, byte in enumerate(packet)) & 65535
    return packet+checksum.to_bytes(2, 'big')


def sample(number=0, full=False, **angles):
    def unsigned(value, scale, size):
        return round(value/scale*((1 << (8*size))-1)).to_bytes(size, 'big')
    def signed(value, scale, size):
        maximum = (1 << (8*size-1))-1
        encoded = round(value/scale*maximum) if abs(value) <= scale else -maximum-1
        return encoded.to_bytes(size, 'big', signed=True)
    items = [
        (2, (1700000000000000+number*100000).to_bytes(8, 'big')),
        (5, unsigned(angles.get('yaw', 90), 360, 2)),
        (6, signed(angles.get('pitch', 0), 20, 2)),
        (7, signed(angles.get('roll', 0), 50, 2)),
        (13, signed(-35.363261+number*.00001, 90, 4)),
        (14, signed(149.16523, 180, 4)),
        (15, unsigned(1550, 19900, 2)), # 650 metres MSL
        (16, unsigned(60 if number < 10 else 30, 180, 2)),
        (18, unsigned(angles.get('azimuth', 0), 360, 4)),
        (19, signed(angles.get('elevation', -90), 180, 4)),
        (20, unsigned(angles.get('relative_roll', 0), 360, 4)),
        (65, b'\x08'),
    ]
    if full:
        items += [(90, signed(angles.get('pitch', 0), 90, 4)),
                  (91, signed(angles.get('roll', 0), 90, 4))]
    return local_set(items)


class DecoderTests(unittest.TestCase):
    def test_units_and_nadir_pose(self):
        record = list(records(sample()))[0]
        self.assertEqual(record['schema'], 'misb.st0601')
        self.assertEqual(record['utc_us'], 1700000000000000)
        self.assertAlmostEqual(position(record)[0], -35.363261, places=6)
        self.assertAlmostEqual(position(record)[1], 149.16523, places=6)
        self.assertAlmostEqual(position(record)[2], 650, delta=.16)
        self.assertAlmostEqual(record['hfov_deg'], 60, delta=.002)
        self.assertAlmostEqual(camera_pose(record)[4], -90, delta=.002)

    def test_platform_and_sensor_rotations_are_composed(self):
        from pymavlink.quaternion import Quaternion
        angles = dict(roll=30, pitch=10, yaw=350, azimuth=20, elevation=-55, relative_roll=330)
        record = list(records(sample(**angles)))[0]
        platform = Quaternion(list(map(math.radians, (30, 10, 350))))
        relative = Quaternion(list(map(math.radians, (330, -55, 20))))
        expected = tuple(math.degrees(v) for v in (platform*relative).euler)
        actual = camera_pose(record)[3:]
        for a, b in zip(actual[:2], expected[:2]):
            self.assertAlmostEqual(a, b, delta=.005)
        self.assertAlmostEqual((actual[2]-expected[2]+180) % 360-180, 0, delta=.005)

    def test_full_range_angles_override_legacy_and_retain_precision(self):
        for pitch, roll in ((45.123456, 80.654321), (-80, -85), (90, 90), (-90, -90)):
            record = next(records(sample(full=True, pitch=pitch, roll=roll)))
            got = record['vehicle_attitude']
            self.assertIsNotNone(got)
            self.assertAlmostEqual(math.degrees(got['pitch_rad']), pitch, delta=3e-8)
            self.assertAlmostEqual(math.degrees(got['roll_rad']), roll, delta=3e-8)
            self.assertIsNotNone(camera_pose(record))
        # A full-range error must not silently fall back to a legacy value.
        packet = local_set([(2, (1).to_bytes(8, 'big')), (5, b'\x00\x00'),
                            (6, b'\x00\x00'), (7, b'\x00\x00'),
                            (90, b'\x00\x00\x00\x00'), (91, b'\x80\x00\x00\x00'), (65, b'\x08')])
        self.assertIsNone(next(records(packet))['vehicle_attitude'])
        self.assertIsNone(next(records(sample(full=True, roll=150)))['vehicle_attitude'])

    def test_corrupt_truncated_and_unrelated_packets(self):
        data = sample()
        for invalid in (data[:-1], data[:-1]+bytes([data[-1] ^ 1]), UAS_KEY+b'\x80', UAS_KEY+b'\x85', b'other', b''):
            self.assertEqual(list(records(invalid)), [])
        self.assertEqual(len(list(records(data[:-1]+bytes([data[-1] ^ 1])+data))), 1)

    def test_missing_and_out_of_range_pose_stays_unknown(self):
        data = local_set([(2, (1).to_bytes(8, 'big')), (6, b'\x80\x00'), (65, b'\x08')])
        record = list(records(data))[0]
        self.assertIsNone(record['position'])
        self.assertIsNone(record['vehicle_attitude'])
        self.assertIsNone(record['gimbal_attitude'])
        self.assertIsNone(camera_pose(record))


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe') and importlib.util.find_spec('av'), 'FFmpeg and PyAV required')
class RecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import av
        cls.temp = tempfile.TemporaryDirectory(prefix='misb-playback-')
        cls.addClassCleanup(cls.temp.cleanup)
        root = Path(cls.temp.name)
        cls.source, cls.video = root/'source.mp4', root/'misb.ts'
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                        '-t', '2', '-c:v', 'libx264', '-bf', '0', '-g', '5', str(cls.source)], check=True)
        with av.open(str(cls.source)) as source, av.open(str(cls.video), 'w', format='mpegts') as output:
            video = output.add_stream_from_template(source.streams.video[0])
            metadata = output.add_data_stream('klv')
            metadata.time_base = Fraction(1, 90000)
            for number, packet in enumerate(p for p in source.demux(video=0) if p.size):
                if number != 7:
                    data = sample(number)
                    if number == 13:
                        data = data[:-1]+bytes([data[-1] ^ 1])
                    klv = av.Packet(data)
                    klv.stream = metadata
                    klv.time_base = Fraction(1, 90000)
                    klv.pts = klv.dts = round(packet.pts*packet.time_base*90000)
                    output.mux(klv)
                packet.stream = video
                output.mux(packet)

    def test_index_binds_metadata_to_matching_video_pts_only(self):
        index = VideoIndex(self.video)
        self.assertEqual(len(index.samples), 20)
        self.assertEqual(index.misb_count, 18)
        self.assertEqual(index.telemetry_count, 18)
        self.assertIsNone(index.samples[7].record)
        self.assertIsNone(index.samples[13].record)
        self.assertAlmostEqual(index.duration, 2, places=3)
        self.assertAlmostEqual(index.times[0], 0)
        for number in (0, 1, 6, 8, 12, 14, 19):
            record = index.samples[number].record
            self.assertEqual(record['utc_us'], 1700000000000000+number*100000)
            self.assertAlmostEqual(record['hfov_deg'], 60 if number < 10 else 30, delta=.002)

    def paused_index(self, release, frames=1):
        """Pause the real demuxer after its first video packet, before EOF."""
        import av
        real_open = av.open

        @contextmanager
        def paused_open(*args, **kwargs):
            with real_open(*args, **kwargs) as container:
                def demux(*streams):
                    count = 0
                    for packet in container.demux(*streams):
                        yield packet
                        if packet.stream.type == 'video' and packet.size:
                            count += 1
                        if count >= frames:
                            if not release.wait(10):
                                raise RuntimeError('Test did not release the indexer')
                yield SimpleNamespace(streams=container.streams, demux=demux)

        with mock.patch.object(av, 'open', paused_open):
            return VideoIndex(self.video, background=True)

    def test_background_index_allows_decoding_before_scan_finishes(self):
        import av
        from MAVProxy.modules.lib.mavvidplay import FrameReader
        release = threading.Event()
        index = self.paused_index(release)
        reader = None
        try:
            self.assertTrue(index.indexing)
            self.assertFalse(index.wait_for_index(0))
            self.assertEqual(index.times, [0])
            self.assertEqual(index.samples[0].record['utc_us'], 1700000000000000)
            reader = FrameReader(index)
            reader.request(0, 0)
            number, generation, image, error = reader.results.get(timeout=10)
            self.assertIsNone(error)
            self.assertEqual((number, generation), (0, 0))
            with av.open(str(self.source)) as source:
                reference = next(source.decode(video=0)).to_ndarray(format='bgr24')
            self.assertTrue((image == reference).all())
            self.assertTrue(index.indexing)
            release.set()
            self.assertTrue(index.wait_for_index(10))
            self.assertIsNone(index.index_error)
            self.assertEqual(len(index.samples), 20)
            self.assertEqual(index.misb_count, 18)
        finally:
            release.set()
            index.close()
            if reader:
                reader.stop()

    def test_ground_speed_available_before_background_scan_finishes(self):
        release = threading.Event()
        index = self.paused_index(release, frames=2)
        try:
            # Wait for the worker's second frame without requiring EOF.
            import time
            deadline = time.monotonic()+5
            while (len(index.times)<2 or index.samples[1].speed is None) and time.monotonic()<deadline:
                time.sleep(.001)
            self.assertEqual(len(index.times), 2)
            self.assertTrue(index.indexing)
            self.assertAlmostEqual(index.samples[1].speed, 11.13, delta=.03)
        finally:
            release.set()
            index.close()

    def test_background_scan_can_be_cancelled(self):
        release = threading.Event()
        index = self.paused_index(release)
        index._index_stop.set()
        release.set()
        index.close()
        self.assertFalse(index.indexing)
        self.assertFalse(index._index_thread.is_alive())
        self.assertEqual(len(index.samples), 1)
        self.assertIsNone(index.index_error)

    def test_growing_recording_is_limited_to_startup_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            video = Path(directory)/'growing.ts'
            shutil.copyfile(self.video, video)
            appended = []
            progress = []
            def on_progress(percent, count):
                progress.append(percent)
                if not appended:
                    with video.open('ab') as output:
                        output.write(self.video.read_bytes())
                    appended.append(True)
            index = VideoIndex(video, progress=on_progress)
            self.assertEqual(video.stat().st_size, 2*self.video.stat().st_size)
            self.assertEqual(len(index.samples), 20)
            self.assertEqual(index.misb_count, 18)
            self.assertEqual(progress[-1], 100)
            self.assertEqual(progress, sorted(progress))

    def test_background_scan_reports_invalid_telemetry(self):
        release = threading.Event()
        with mock.patch('MAVProxy.modules.lib.misb.records', return_value=iter(())):
            index = self.paused_index(release)
            try:
                self.assertTrue(index.indexing)
                release.set()
                self.assertTrue(index.wait_for_index(10))
                self.assertIn('No valid MISB', index.index_error)
            finally:
                release.set()
                index.close()

    def test_reordered_and_hevc_recordings_seek_with_matching_telemetry(self):
        import av
        from MAVProxy.modules.lib.mavvidplay import FrameReader
        for encoder, bframes in (('libx264', 2), ('libx265', 0), ('libx265', 2)):
            with self.subTest(encoder=encoder, bframes=bframes):
                source_path = Path(self.temp.name)/('%s-%u.mp4' % (encoder, bframes))
                video_path = source_path.with_suffix('.ts')
                command = ['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i',
                           'testsrc2=size=160x120:rate=10', '-t', '2', '-c:v', encoder,
                           '-bf', str(bframes), '-g', '5']
                if encoder == 'libx265':
                    command += ['-x265-params', 'log-level=error:pools=1:frame-threads=1:open-gop=0']
                subprocess.run(command+[str(source_path)], check=True, capture_output=True)
                with av.open(str(source_path)) as source, av.open(str(video_path), 'w', format='mpegts') as output:
                    video = output.add_stream_from_template(source.streams.video[0])
                    metadata = output.add_data_stream('klv')
                    metadata.time_base = Fraction(1, 90000)
                    for packet in source.demux(video=0):
                        if not packet.size:
                            continue
                        number = round(packet.pts*packet.time_base*10)
                        klv = av.Packet(sample(number))
                        klv.stream = metadata
                        klv.time_base = Fraction(1, 90000)
                        klv.pts = round(packet.pts*packet.time_base*90000)
                        klv.dts = round(packet.dts*packet.time_base*90000)
                        output.mux(klv)
                        packet.stream = video
                        output.mux(packet)
                index = VideoIndex(video_path, background=True)
                reader = None
                try:
                    self.assertTrue(index.wait_for_index(10))
                    self.assertIsNone(index.index_error)
                    self.assertEqual(len(index.samples), 20)
                    for number, entry in enumerate(index.samples):
                        self.assertEqual(entry.record['utc_us'], 1700000000000000+number*100000)
                    with av.open(str(source_path)) as source:
                        reference = [f.to_ndarray(format='bgr24') for f in source.decode(video=0)]
                    reader = FrameReader(index)
                    for generation, number in enumerate((0, 12, 3, 19, 1)):
                        reader.request(number, generation)
                        actual, gen, image, error = reader.results.get(timeout=10)
                        self.assertIsNone(error)
                        self.assertEqual((actual, gen), (number, generation))
                        self.assertTrue((image == reference[number]).all())
                finally:
                    index.close()
                    if reader:
                        reader.stop()

    def test_dual_metadata_preserves_inverted_attitude_and_counts_klv(self):
        import av
        from test_video_telemetry import record, sei
        from MAVProxy.modules.lib.video_telemetry import packet_nals
        path = Path(self.temp.name)/'dual.ts'
        with av.open(str(self.video)) as source, av.open(str(path), 'w', format='mpegts') as output:
            video = output.add_stream_from_template(source.streams.video[0])
            metadata = output.add_data_stream('klv')
            metadata.time_base = Fraction(1, 90000)
            for number, packet in enumerate(p for p in source.demux(video=0) if p.size):
                value = dict(record(), utc_us=1700000000000000+number*100000, hfov_deg=60)
                value['vehicle_attitude']['roll_rad'] = math.radians(150)
                value['vehicle_attitude']['pitch_rad'] = math.radians(45)
                nals = list(packet_nals(bytes(packet), 0))
                at = next(i for i, nal in enumerate(nals) if 1 <= nal[0]&31 <= 5)
                nals.insert(at, sei(value))
                frame = av.Packet(b''.join(b'\0\0\0\1'+nal for nal in nals))
                frame.pts, frame.dts, frame.time_base = packet.pts, packet.dts, packet.time_base
                frame.stream = video
                klv = av.Packet(sample(number, full=True, roll=150, pitch=45))
                klv.stream = metadata
                klv.time_base = Fraction(1, 90000)
                klv.pts = klv.dts = round(packet.pts*packet.time_base*90000)
                for entry in ((klv, frame) if number%2 else (frame, klv)):
                    output.mux(entry)
        index = VideoIndex(path, background=True)
        try:
            self.assertTrue(index.wait_for_index(10))
            self.assertIsNone(index.index_error)
            self.assertEqual((len(index.samples), index.telemetry_count, index.misb_count), (20, 20, 20))
            for entry in index.samples:
                self.assertTrue(entry.has_misb)
                self.assertEqual(entry.record['schema'], 'apcg.telemetry.v1')
                self.assertAlmostEqual(entry.record['vehicle_attitude']['roll_rad'], math.radians(150))
                self.assertIsNotNone(camera_pose(entry.record))
        finally:
            index.close()

    def test_decoder_seeks_backwards_and_across_gops_without_pose_drift(self):
        import av
        from MAVProxy.modules.lib.mavvidplay import FrameReader
        index = VideoIndex(self.video)
        with av.open(str(self.source)) as source:
            reference = [f.to_ndarray(format='bgr24') for f in source.decode(video=0)]
        reader = FrameReader(index)
        try:
            for generation, target in enumerate((0, 12, 3, 19, 1, 2, 18, 7)):
                reader.request(target, generation)
                number, actual_generation, image, error = reader.results.get(timeout=10)
                self.assertIsNone(error)
                self.assertEqual((number, actual_generation), (target, generation))
                self.assertTrue((image == reference[target]).all(), target)
        finally:
            reader.stop()
        self.assertFalse(reader.is_alive())


if __name__ == '__main__':
    unittest.main()
