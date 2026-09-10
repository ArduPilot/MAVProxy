"""Vendor subtitle parsing, frame association and tlog clock regression tests."""
from datetime import datetime, timezone
import math
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

from MAVProxy.modules.lib.siyi_video import coordinate, parse_subtitle, attach_subtitles, FlightLog
from MAVProxy.modules.lib.video_telemetry import VideoIndex, Sample, camera_pose


def vendor_text(frame=0, second=58, latitude='S35°16′38.6″'):
    return (f'2026-08-28_02-52-{second:02d} stream->FrameCnt:{frame}\n'
            f'focal_len:4.500000mm\nmix_ratio:1.000000\nmix_ratio:1.000000\n'
            f'dig_ratio:1.000000\nfnum:2.800000\nlatitude:{latitude}\n'
            'longitude:E148°56′36.3″\nrel_alt:651.650024\nabs_alt:651.650024\n'
            'gb_yaw:14.5, gb_pitch:-80.0, gb_roll:0.0\n')


def tx3g(text):
    data = text.encode()
    return len(data).to_bytes(2, 'big') + data


class SubtitleTests(unittest.TestCase):
    def test_dms_and_record(self):
        self.assertAlmostEqual(coordinate('S35°16′38.6″', True), -(35 + 16 / 60 + 38.6 / 3600))
        self.assertAlmostEqual(coordinate('W148°56′36.3″', False), -(148 + 56 / 60 + 36.3 / 3600))
        rec = parse_subtitle(tx3g(vendor_text()) + b'\0\0\0\x08styl')
        self.assertEqual(rec['frame_counter'], 0)
        self.assertEqual(rec['schema'], 'siyi.subtitle.v1')
        self.assertAlmostEqual(rec['gimbal_attitude']['yaw_rad'], math.radians(14.5))
        self.assertNotIn('alt_relative_m', rec['position'])
        self.assertIsNone(camera_pose(rec))
        self.assertIsNone(rec['heading_rad'])
        self.assertEqual(rec['utc_us'], round(datetime(2026, 8, 28, 2, 52, 58, tzinfo=timezone.utc).timestamp() * 1e6))

    def test_invalid_subtitles(self):
        for data in [b'', b'\0\0', b'\0\x20short', tx3g('ordinary subtitle'),
                     tx3g(vendor_text().replace('651.650024', 'nan')),
                     tx3g(vendor_text(latitude='S35°60′38.6″')),
                     tx3g(vendor_text(latitude='S91°00′00.0″')),
                     tx3g(vendor_text().replace('gb_yaw:14.5', 'gb_yaw:inf'))]:
            self.assertIsNone(parse_subtitle(data))

    def test_counter_disambiguates_thermal_pts(self):
        idx = VideoIndex.__new__(VideoIndex)
        idx.samples = [Sample(i / 30, 1 / 30, None) for i in range(5)]
        idx.times = [s.time for s in idx.samples]
        records = [(round(i / 30 * 25) / 25, parse_subtitle(tx3g(vendor_text(i)))) for i in range(5)]
        self.assertEqual(attach_subtitles(idx, records, 0), (5, 0))
        self.assertEqual([s.record['frame_counter'] for s in idx.samples], list(range(5)))

    def test_rollover_and_corruption_not_silently_rebased(self):
        idx = VideoIndex.__new__(VideoIndex)
        idx.samples = [Sample(i / 30, 1 / 30, None) for i in range(100)]
        idx.times = [s.time for s in idx.samples]
        later = [(2, parse_subtitle(tx3g(vendor_text(60))))]
        self.assertEqual(attach_subtitles(idx, later, 0), (0, 1))
        self.assertTrue(all(s.record is None for s in idx.samples))
        valid = parse_subtitle(tx3g(vendor_text()))
        bad = parse_subtitle(tx3g(vendor_text(99)))
        self.assertEqual(attach_subtitles(idx, [(0, valid), (.033, bad)], 0), (1, 1))


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'ffmpeg required')
class VendorRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.video = self.root / 'vendor.mp4'
        subtitles = []
        for i in range(20):
            text = vendor_text(i, second=50 if i < 10 else 55,
                               latitude='S35°16′38.6″' if i < 10 else 'S35°16′39.0″')
            start, end = i * 100, (i + 1) * 100
            subtitles.append(f'{i+1}\n00:00:{start//1000:02d},{start%1000:03d} --> '
                             f'00:00:{end//1000:02d},{end%1000:03d}\n{text}\n')
        srt = self.root / 'vendor.srt'
        srt.write_text(''.join(subtitles))
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=160x120:rate=10',
                        '-i', str(srt), '-t', '2', '-c:v', 'libx264', '-bf', '2', '-g', '5',
                        '-c:s', 'mov_text', str(self.video)], check=True)

    def tearDown(self):
        self.temp.cleanup()

    def test_vendor_mp4_and_real_clock_speed(self):
        idx = VideoIndex(self.video)
        self.assertEqual(idx.siyi_count, 20)
        self.assertEqual(idx.telemetry_count, 20)
        for i, sample in enumerate(idx.samples):
            self.assertEqual(sample.record['frame_counter'], i)
            self.assertAlmostEqual(sample.time, i / 10)
        # 0.4 arcsecond ~12.37 m in five wall-clock seconds, one video second.
        self.assertAlmostEqual(idx.samples[10].speed, 2.474, delta=.02)
        self.assertEqual(idx.samples[10].speed, idx.samples[19].speed)
        self.assertIsNone(idx.samples[0].speed)
        self.assertIsNone(camera_pose(idx.samples[10].record))

    def test_flight_log_system_selection_clock_and_gaps(self):
        from pymavlink import mavutil
        idx = VideoIndex(self.video)
        base = idx.samples[0].record['utc_us'] * 1e-6
        filename = self.root / 'flight.tlog'
        mav = mavutil.mavlink.MAVLink(None)
        with filename.open('wb') as dest:
            def write(message, when, system=17, component=1):
                mav.srcSystem, mav.srcComponent = system, component
                dest.write(struct.pack('>Q', round(when * 1e6)))
                dest.write(message.pack(mav))
            heartbeat = mavutil.mavlink.MAVLink_heartbeat_message
            write(heartbeat(6, 8, 0, 0, 0, 3), base, system=255, component=190)  # GCS
            write(heartbeat(30, 8, 0, 0, 0, 3), base, system=17, component=100)  # camera
            write(heartbeat(1, 3, 0, 0, 0, 3), base)
            for dt in (.1, 5.1):
                write(mavutil.mavlink.MAVLink_attitude_message(0, .1, .2, .3, 0, 0, 0), base + dt)
                write(mavutil.mavlink.MAVLink_global_position_int_message(
                    0, -350000000, 1490000000, 650000, 100000, 300, 400, 0, 9000), base + dt)
                write(mavutil.mavlink.MAVLink_attitude_message(0, 1, 2, 3, 0, 0, 0), base + dt, system=42)
        log = FlightLog(filename)
        self.assertEqual(log.system_id, 17)
        self.assertEqual(log.apply(idx), 20)
        self.assertAlmostEqual(camera_pose(idx.samples[0].record)[5], 14.5 + math.degrees(.3), places=4)
        self.assertEqual(idx.samples[0].speed, 5)
        self.assertEqual(idx.samples[0].speed_source, 'tlog')
        self.assertAlmostEqual(idx.samples[0].record['heading_rad'], math.pi / 2)
        fresh = VideoIndex(self.video)
        self.assertEqual(log.apply(fresh, offset=100), 0)
        self.assertIsNone(camera_pose(fresh.samples[0].record))
        explicit = FlightLog(filename, system_id=42)
        self.assertEqual(explicit.apply(fresh), 20)
        self.assertAlmostEqual(fresh.samples[0].record['vehicle_attitude']['yaw_rad'], 3)


if __name__ == '__main__':
    unittest.main()
