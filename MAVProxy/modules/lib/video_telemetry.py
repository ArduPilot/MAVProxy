"""Index AP_CameraGimbal SEI, MISB KLV, SIYI subtitle and thermal-Matroska telemetry on the video's presentation clock.

No MAVLink connection is needed. ffprobe demuxes packets without decoding video;
MP4 packet offsets allow us to read compressed samples directly from the file.
Lossless thermal recordings from thermal_to_video.py store their per-frame
apcg.telemetry.v1 snapshot in Matroska block additions; those are read with PyAV.
"""
import bisect
from dataclasses import dataclass
from fractions import Fraction
import json
import math
import pathlib
import re
import statistics
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

UUID = bytes.fromhex('8d646b4e556f4a908b7c35e629510321')
UUID_PATTERN = re.compile(re.escape(UUID))
THERMAL_ADDITION_TYPE = 0x41504347  # APCG block-addition FourCC written by thermal_to_video.py
START = re.compile(b'\x00\x00(?:\x00)?\x01')


def unhex(text):
    """Decode ffprobe's padded hex dump (also after XML newline normalization)."""
    return bytes.fromhex(''.join(re.findall(
        r'(?:^|\s)[0-9a-fA-F]{8,}: ([0-9a-fA-F ]{39})', text)))


def packet_nals(packet, length_size, copy=True):
    packet = memoryview(packet)
    if length_size:
        offset = 0
        while offset + length_size <= len(packet):
            size = int.from_bytes(packet[offset:offset + length_size], 'big')
            offset += length_size
            if size == 0 or offset + size > len(packet):
                return
            nal = packet[offset:offset + size]
            yield bytes(nal) if copy else nal
            offset += size
    else:
        starts = list(START.finditer(packet))
        for i, start in enumerate(starts):
            end = starts[i + 1].start() if i + 1 < len(starts) else len(packet)
            while end > start.end() and packet[end-1] == 0:
                end -= 1
            nal = packet[start.end():end]
            yield bytes(nal) if copy else nal


def sei_records(nal, codec):
    if not nal:
        return
    header = 1 if codec == 'h264' else 2
    kind = nal[0] & 31 if header == 1 else (nal[0] >> 1) & 63
    if kind not in ((6,) if header == 1 else (39, 40)):
        return
    rbsp = re.sub(b'\x00\x00\x03', b'\x00\x00', nal[header:])
    offset = 0
    while offset < len(rbsp) and rbsp[offset:] != b'\x80':
        fields = []
        for _ in range(2):
            value = 0
            while offset < len(rbsp) and rbsp[offset] == 255:
                value += 255
                offset += 1
            if offset == len(rbsp):
                return
            fields.append(value + rbsp[offset])
            offset += 1
        kind, size = fields
        if offset + size > len(rbsp):
            return
        payload = rbsp[offset:offset + size]
        offset += size
        if kind == 5 and payload[:16] == UUID:
            try:
                record = json.loads(payload[16:].rstrip(b'\0'))
            except (ValueError, UnicodeError):
                continue
            if isinstance(record, dict) and record.get('schema') == 'apcg.telemetry.v1':
                yield record


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def source(record, key):
    value = record.get(key) if record else None
    if not isinstance(value, dict):
        return None
    age = value.get('age_ms', 0)
    return value if finite(age) and 0 <= age <= 10000 else None


def position(record):
    pos = source(record, 'position')
    if pos is None or not all(finite(pos.get(k)) for k in ('lat_e7', 'lon_e7', 'alt_amsl_m')):
        return None
    lat, lon = pos['lat_e7'] * 1e-7, pos['lon_e7'] * 1e-7
    if abs(lat) > 90 or abs(lon) > 180:
        return None
    return lat, lon, pos['alt_amsl_m']


def attitude(record, key):
    att = source(record, key)
    keys = ('roll_rad', 'pitch_rad', 'yaw_rad')
    if att is None or not all(finite(att.get(k)) for k in keys):
        return None
    return tuple(math.degrees(att[k]) for k in keys)


def camera_pose(record):
    """Gimbal roll/pitch are level-referenced; its yaw is vehicle-relative."""
    pos = position(record)
    gimbal = attitude(record, 'gimbal_attitude')
    vehicle = attitude(record, 'vehicle_attitude')
    if pos is None or gimbal is None or vehicle is None:
        return None
    return (*pos, gimbal[0], gimbal[1], (gimbal[2] + vehicle[2]) % 360)


@dataclass
class Sample:
    time: float
    duration: float
    record: object
    speed: object = None
    speed_source: str = 'GPS estimate'
    has_misb: bool = False


class _SpeedEstimator:
    def __init__(self):
        from MAVProxy.modules.lib import mp_util
        self.gps_distance = mp_util.gps_distance
        self.previous = self.speed = None

    def update(self, sample):
        pos = position(sample.record)
        raw = source(sample.record, 'position')
        utc = sample.record.get('utc_us') if sample.record else None
        if pos is None or not finite(utc):
            self.previous, self.speed = None, None
            return
        timestamp = utc * 1e-6 - raw.get('age_ms', 0) * .001
        if self.previous is not None:
            dt = timestamp - self.previous[0]
            # Repeated snapshots include the same GPS fix with increasing age.
            # Millisecond age quantization can jitter the reconstructed time.
            if dt < -.01 or dt > 10:
                self.speed = None
                self.previous = None
            elif dt > .02:
                self.speed = self.gps_distance(*self.previous[1][:2], *pos[:2]) / dt
                self.previous = (timestamp, pos)
        if self.previous is None:
            self.previous = (timestamp, pos)
        sample.speed = self.speed


class _RecordingSnapshot:
    """Seekable read-only prefix of a recording which may still be growing."""
    def __init__(self, source, size):
        self.source, self.size = source, size

    def read(self, size=-1):
        remaining = max(0, self.size-self.source.tell())
        return self.source.read(remaining if size < 0 else min(size, remaining))

    def seek(self, offset, whence=0):
        if whence == 2:
            offset, whence = self.size+offset, 0
        return self.source.seek(offset, whence)

    def tell(self):
        return self.source.tell()


class VideoIndex:
    def __init__(self, filename, background=False, progress=None):
        self.filename = str(pathlib.Path(filename).resolve(strict=True))
        self.thermal = False
        self.frame_pts = None
        self.keyframe_pts = None
        self.keyframe_dts = {}
        self.misb_count = 0
        self.indexing = False
        self.index_error = None
        self.index_progress = 100.0
        self._index_thread = None
        self._index_stop = threading.Event()
        self._index_ready = threading.Event()
        self._index_done = threading.Event()
        self._index_done.set()
        info = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-show_entries',
            'stream=index,codec_type,codec_name,width,height,avg_frame_rate,has_b_frames,extradata:format=format_name,duration',
            '-show_data', '-of', 'json', self.filename]))
        streams = [s for s in info.get('streams', []) if s.get('codec_type') == 'video']
        subtitle_ids = {s['index'] for s in info.get('streams', []) if s.get('codec_name') == 'mov_text'}
        klv_ids = {s['index'] for s in info.get('streams', []) if s.get('codec_name') == 'klv'}
        if not streams:
            raise ValueError('An H.264/H.265 or APCG thermal Matroska recording is required')
        if streams[0]['codec_name'] == 'ffv1':
            self._index_thermal(streams[0])
            return
        if streams[0]['codec_name'] not in ('h264', 'hevc'):
            raise ValueError('An H.264/H.265 or APCG thermal Matroska recording is required')
        stream = streams[0]
        self.codec = stream['codec_name']
        self.width, self.height = stream['width'], stream['height']
        try:
            self.fps = float(Fraction(stream['avg_frame_rate']))
        except (ValueError, ZeroDivisionError):
            self.fps = 25.0
        if not self.fps > 0:
            self.fps = 25.0
        extra = unhex(stream.get('extradata', ''))
        length_size = 0
        if extra and extra[0] == 1:
            offset = 4 if self.codec == 'h264' else 21
            if len(extra) <= offset:
                raise ValueError('Truncated video codec configuration')
            length_size = (extra[offset] & 3) + 1
        formats = info.get('format', {}).get('format_name', '').split(',')
        direct = 'mov' in formats
        if 'mpegts' in formats:
            self.frame_pts = [] # MPEG-TS seeks use real presentation times.
            self.keyframe_pts = []
            if klv_ids:
                # Video packet hex dumps expand multi-GB recordings into tens
                # of GB of XML. Demux in native code, copying only KLV/SEI.
                self._start_misb_index(stream, klv_ids, info.get('format', {}),
                                       background and not stream.get('has_b_frames', 0), progress)
                return
        entries = 'packet=stream_index,pts,dts,pts_time,duration_time,pos,size,flags' + ('' if direct else ',data')
        command = ['ffprobe', '-v', 'error', '-show_packets',
                   '-show_entries', entries, '-of', 'xml']
        if not direct:
            command += ['-show_data']
        self.samples = []
        self.warnings = []
        subtitles = {}
        klv_samples = []
        from MAVProxy.modules.lib.misb import records as misb_records
        from MAVProxy.modules.lib.siyi_video import parse_subtitle, attach_subtitles
        # Drain stderr to a file so malformed input cannot deadlock the subprocess.
        with open(self.filename, 'rb') as video, tempfile.TemporaryFile() as errors:
            process = subprocess.Popen(command + [self.filename], stdout=subprocess.PIPE, stderr=errors)
            try:
                context = ET.iterparse(process.stdout, events=('start', 'end'))
                packets_element = None
                for event, element in context:
                    if event == 'start' and element.tag == 'packets':
                        packets_element = element
                    if event != 'end' or element.tag != 'packet':
                        continue
                    attrs = element.attrib
                    packet_stream = int(attrs['stream_index'])
                    if packet_stream != stream['index'] and packet_stream not in subtitle_ids | klv_ids:
                        element.clear()
                        if packets_element is not None:
                            packets_element.clear()
                        continue
                    if direct:
                        video.seek(int(attrs['pos']))
                        packet = video.read(int(attrs['size']))
                    else:
                        packet = unhex(attrs.get('data', ''))
                    if packet_stream in klv_ids:
                        timestamp = float(attrs.get('pts_time', 'nan'))
                        records = list(misb_records(packet))
                        if records and math.isfinite(timestamp):
                            klv_samples.append((timestamp, records[-1]))
                        element.clear()
                        if packets_element is not None:
                            packets_element.clear()
                        continue
                    if packet_stream in subtitle_ids:
                        record = parse_subtitle(packet)
                        pts = float(attrs.get('pts_time', 'nan'))
                        if record is not None and math.isfinite(pts):
                            subtitles.setdefault(packet_stream, []).append((pts, record))
                        element.clear()
                        if packets_element is not None:
                            packets_element.clear()
                        continue
                    records = [r for nal in packet_nals(packet, length_size)
                               for r in sei_records(nal, self.codec)]
                    if 'pts_time' not in attrs:
                        raise ValueError('Video packets have no presentation timestamps; '
                                         'use a timestamped MP4 or Matroska recording')
                    timestamp = float(attrs['pts_time'])
                    duration = float(attrs.get('duration_time', 1 / self.fps))
                    if not math.isfinite(timestamp) or not math.isfinite(duration):
                        raise ValueError('Invalid video timestamp')
                    self.samples.append(Sample(timestamp, duration, records[-1] if records else None))
                    if self.frame_pts is not None:
                        if 'pts' not in attrs:
                            raise ValueError('MPEG-TS video packets have no presentation timestamps')
                        self.frame_pts.append(int(attrs['pts']))
                        if 'K' in attrs.get('flags', ''):
                            self.keyframe_dts[int(attrs['pts'])] = int(attrs.get('dts', attrs['pts']))
                            self.keyframe_pts.append(int(attrs['pts']))
                    element.clear()
                    if packets_element is not None:
                        packets_element.clear()
                if process.wait() != 0:
                    errors.seek(0)
                    raise ValueError(errors.read().decode(errors='replace').strip())
            finally:
                process.stdout.close()
                if process.poll() is None:
                    process.terminate()
                process.wait()
        if not self.samples:
            raise ValueError('No video frames found')
        # Packet decode order may differ from display order (B frames). Never use
        # JSON pts90k for seeking: remuxed RTP recordings may have an arbitrary epoch.
        if self.frame_pts is not None:
            ordered = sorted(zip(self.samples, self.frame_pts), key=lambda row: row[0].time)
            self.samples = [row[0] for row in ordered]
            self.frame_pts = [row[1] for row in ordered]
            self.keyframe_pts.sort()
        else:
            self.samples.sort(key=lambda sample: sample.time)
        if klv_samples:
            klv_samples.sort(key=lambda row: row[0])
            klv_times = [row[0] for row in klv_samples]
            for sample in self.samples:
                i = bisect.bisect_left(klv_times, sample.time-1e-5)
                # One 90 kHz tick, allowing ffprobe's decimal rounding. Do not
                # borrow an adjacent frame's pose when metadata is missing.
                if i < len(klv_times) and abs(klv_times[i]-sample.time) <= 1.2e-5:
                    sample.has_misb = True
                    self.misb_count += 1
                    if sample.record is None:
                        sample.record = klv_samples[i][1]
        origin = self.samples[0].time
        for sample in self.samples:
            sample.time -= origin
        self.times = [sample.time for sample in self.samples]
        last_duration = self.samples[-1].duration
        self.duration = self.times[-1] + (last_duration if last_duration > 0 else 1 / self.fps)
        self.siyi_count = 0
        if subtitles:
            selected = max(subtitles.values(), key=len)
            self.siyi_count, rejected = attach_subtitles(self, selected, origin)
            if rejected:
                self.warnings.append('%u SIYI records rejected: subtitle timing/frame counters do not match this video' % rejected)
        self.telemetry_count = sum(s.record is not None for s in self.samples)
        if not self.telemetry_count:
            detail = '; '.join(self.warnings) or 'No AP_CameraGimbal SEI, MISB KLV or SIYI subtitle telemetry found'
            raise ValueError(detail)
        self._speeds()

    def _start_misb_index(self, stream_info, klv_ids, format_info, background, progress):
        self.samples, self.times, self.warnings = [], [], []
        self.siyi_count = self.telemetry_count = 0
        try:
            duration = float(format_info.get('duration', 0))
        except (TypeError, ValueError):
            duration = 0
        self.duration = duration if math.isfinite(duration) and duration > 0 else 1/self.fps
        # The recorder holds an exclusive lock while appending an access unit.
        # Snapshot a complete prefix, even when the selected file is growing.
        with open(self.filename, 'rb') as source_file:
            try:
                import fcntl
            except ImportError:
                fcntl = None
            if fcntl:
                fcntl.flock(source_file, fcntl.LOCK_SH)
            source_file.seek(0, 2)
            size = source_file.tell()
        self.indexing = True
        self.index_progress = 0
        self._index_done.clear()
        if background:
            self._index_thread = threading.Thread(target=self._scan_misb,
                args=(stream_info['index'], klv_ids, size, True, progress),
                name='telemetry-index', daemon=True)
            self._index_thread.start()
            self._index_ready.wait()
        else:
            self._scan_misb(stream_info['index'], klv_ids, size, False, progress)
        if self.index_error:
            raise ValueError(self.index_error)

    def _scan_misb(self, video_id, klv_ids, size, incremental, progress):
        from MAVProxy.modules.lib.misb import records as misb_records
        samples_by_pts, pending = {}, {}
        origin, last_progress = None, 0.0
        speed_at = 0
        try:
            estimator = _SpeedEstimator() if incremental else None
            import av
            # Bound reads and SEEK_END to the captured length. FFmpeg's
            # subfile protocol mishandles seek offsets on multi-GB inputs.
            with open(self.filename, 'rb') as source, av.open(
                    _RecordingSnapshot(source, size), format='mpegts',
                    buffer_size=1024*1024) as container:
                streams = [s for s in container.streams if s.index == video_id or s.index in klv_ids]
                for packet in container.demux(*streams):
                    if self._index_stop.is_set():
                        return
                    if not packet.size or packet.pts is None:
                        continue
                    timestamp = float(packet.pts * packet.time_base)
                    tick = round(timestamp*90000)
                    if packet.stream.index == video_id:
                        if origin is None:
                            origin = timestamp
                        relative = timestamp-origin
                        if incremental and self.times and relative < self.times[-1]:
                            raise ValueError('Non-monotonic MPEG-TS timestamps; reopen without background indexing')
                        duration = float(packet.duration*packet.time_base) if packet.duration else 1/self.fps
                        rec = None
                        data = memoryview(packet)
                        # Preserve SEI precedence in mixed recordings without
                        # copying compressed video or formatting it as hex.
                        if UUID_PATTERN.search(data):
                            recs = [r for nal in packet_nals(data, 0, copy=False)
                                    for r in sei_records(nal, self.codec)]
                            rec = recs[-1] if recs else None
                        klv = pending.pop(tick, None)
                        if rec is None:
                            rec = klv
                        sample = Sample(relative, duration, rec, has_misb=klv is not None)
                        self.misb_count += sample.has_misb
                        self.frame_pts.append(int(packet.pts))
                        if packet.is_keyframe:
                            self.keyframe_dts[int(packet.pts)] = int(packet.dts if packet.dts is not None else packet.pts)
                            self.keyframe_pts.append(int(packet.pts))
                        self.samples.append(sample)
                        samples_by_pts[tick] = sample
                        self.telemetry_count += rec is not None
                        self.duration = max(self.duration, relative+duration)
                        # Publish times last: an exposed index always has a
                        # matching sample and decoder timestamp already stored.
                        self.times.append(relative)
                        self._index_ready.set()
                    else:
                        recs = list(misb_records(bytes(packet)))
                        if recs:
                            sample = samples_by_pts.get(tick)
                            if sample is not None:
                                if not sample.has_misb:
                                    sample.has_misb = True
                                    self.misb_count += 1
                                if sample.record is None:
                                    sample.record = recs[-1]
                                    self.telemetry_count += 1
                            else:
                                pending[tick] = recs[-1]
                    if incremental:
                        # KLV may follow its video packet. Leave a short tail
                        # for late metadata, then treat missing poses as gaps.
                        while speed_at < len(self.samples):
                            sample = self.samples[speed_at]
                            if sample.record is None and speed_at >= len(self.samples)-2*self.fps:
                                break
                            estimator.update(sample)
                            speed_at += 1
                    if packet.pos is not None and 0 <= packet.pos <= size:
                        self.index_progress = max(self.index_progress, min(99.9, packet.pos*100/max(1, size)))
                    now = time.monotonic()
                    if now-last_progress >= .5:
                        if progress:
                            progress(self.index_progress, len(self.samples))
                        last_progress = now
            if not self.samples:
                raise ValueError('No video frames found')
            if not self.telemetry_count:
                raise ValueError('No valid MISB KLV or AP_CameraGimbal SEI telemetry found')
            if not incremental:
                ordered = sorted(zip(self.samples, self.frame_pts), key=lambda row: row[0].time)
                self.samples[:] = [row[0] for row in ordered]
                self.frame_pts[:] = [row[1] for row in ordered]
                start = self.samples[0].time
                for sample in self.samples:
                    sample.time -= start
                self.times[:] = [s.time for s in self.samples]
                self.keyframe_pts.sort()
            self.duration = self.times[-1] + (self.samples[-1].duration or 1/self.fps)
            self._speeds()
            self.index_progress = 100.0
            if progress:
                progress(100.0, len(self.samples))
        except Exception as error:
            self.index_error = str(error)
        finally:
            self.indexing = False
            self._index_ready.set()
            self._index_done.set()

    def wait_for_index(self, timeout=None):
        return self._index_done.wait(timeout)

    def close(self):
        self._index_stop.set()
        if self._index_thread is not None:
            self._index_thread.join(timeout=5)

    def _thermal_record(self, packet):
        """Return the apcg.telemetry.v1 snapshot from a thermal frame's Matroska
        block addition, or None. The thermal lens HFOV comes from the outer
        apcg.thermal.v1 metadata when the nested telemetry does not carry one."""
        if not hasattr(packet, 'get_sidedata'):
            return None
        try:
            data = bytes(packet.get_sidedata('matroska_block_additional'))
        except Exception:
            return None
        if len(data) <= 8 or int.from_bytes(data[:8], 'big') != THERMAL_ADDITION_TYPE:
            return None
        try:
            meta = json.loads(data[8:])
        except (ValueError, UnicodeError):
            return None
        if not isinstance(meta, dict) or meta.get('schema') != 'apcg.thermal.v1':
            return None
        telemetry = meta.get('telemetry')
        if not isinstance(telemetry, dict) or telemetry.get('schema') != 'apcg.telemetry.v1':
            return None
        record = dict(telemetry)
        record['thermal'] = {key: meta.get(key) for key in (
            'temperature_scale_k', 'temperature_offset_k', 'minimum_c', 'maximum_c')}
        if not finite(record.get('hfov_deg')):
            fov = meta.get('hfov_deg')
            if finite(fov) and 0 < fov < 180:
                record['hfov_deg'] = fov
        return record

    def _index_thermal(self, stream_info):
        """Index a lossless thermal recording: FFV1 in Matroska with per-frame
        telemetry in block additions. Each frame is independently coded, so the
        decoder can seek to any frame by presentation timestamp."""
        try:
            import av
        except ImportError as error:
            raise ValueError('Playing an APCG thermal recording needs PyAV: pip install "av>=18.1"') from error
        self.thermal = True
        self.codec = 'ffv1'
        self.width, self.height = stream_info['width'], stream_info['height']
        entries = []
        with av.open(self.filename) as container:
            video = container.streams.video
            if not video or video[0].codec_context.name != 'ffv1':
                raise ValueError('An H.264/H.265 or APCG thermal Matroska recording is required')
            stream = video[0]
            time_base = stream.time_base or Fraction(1, 1000)
            for packet in container.demux(stream):
                if packet.size == 0 or packet.pts is None:
                    continue
                seconds = float(packet.pts * time_base)
                if not math.isfinite(seconds):
                    continue
                entries.append((int(packet.pts), Sample(seconds, 0.0, self._thermal_record(packet))))
        if not entries:
            raise ValueError('No thermal frames found')
        entries.sort(key=lambda item: item[1].time)
        self.frame_pts = [pts for pts, _ in entries]
        self.samples = [sample for _, sample in entries]
        self.warnings = []
        self.siyi_count = 0
        origin = self.samples[0].time
        for sample in self.samples:
            sample.time -= origin
        self.times = [sample.time for sample in self.samples]
        gaps = [b - a for a, b in zip(self.times, self.times[1:])]
        interval = statistics.median(gaps) if gaps else 0.2
        self.fps = 1 / interval if interval > 0 else 5.0
        for i, sample in enumerate(self.samples):
            sample.duration = gaps[i] if i < len(gaps) else (gaps[-1] if gaps else interval)
        self.duration = self.times[-1] + (self.samples[-1].duration or interval)
        self.telemetry_count = sum(sample.record is not None for sample in self.samples)
        if not self.telemetry_count:
            raise ValueError('This thermal recording has no telemetry; '
                             're-convert with thermal_to_video.py --bin FLIGHT.bin')
        self._speeds()

    def has_fov(self):
        return any(sample.record and finite(sample.record.get('hfov_deg')) and
                   0 < sample.record['hfov_deg'] < 180 for sample in self.samples)

    def frame_at(self, seconds):
        return max(0, min(len(self.samples) - 1, bisect.bisect_right(self.times, seconds) - 1))

    def _speeds(self):
        estimator = _SpeedEstimator()
        for sample in self.samples:
            estimator.update(sample)


class FlatElevation:
    def __init__(self, altitude):
        self.altitude = altitude

    def GetElevation(self, latitude, longitude, timeout=0):
        return self.altitude


class ViewProjection:
    def __init__(self, width, height, fov, elevation, max_range=10000, zoom_fov=False):
        self.width, self.height = width, height
        self.fov, self.elevation = fov, elevation
        self.max_range, self.zoom_fov = max_range, zoom_fov
        self.current_fov = None
        self.projection = None

    def effective_fov(self, record):
        if self.fov is None:
            # Recorded FOV already includes optical and digital zoom.
            value = record.get('hfov_deg') if record else None
            return value if finite(value) and 0 < value < 180 else None
        value = self.fov
        if self.zoom_fov:
            zoom = record.get('zoom', 1) if record else 1
            zoom = zoom if finite(zoom) and zoom > 0 else 1
            value = math.degrees(2 * math.atan(math.tan(math.radians(value) / 2) / zoom))
        return value

    def _camera(self, record):
        from MAVProxy.modules.lib.camera_projection import CameraParams, CameraProjection
        hfov = self.effective_fov(record)
        if hfov is None:
            return None
        if self.current_fov != hfov:
            params = CameraParams(xresolution=self.width, yresolution=self.height, FOV=hfov)
            self.projection = CameraProjection(params, elevation_model=self.elevation)
            self.current_fov = hfov
        return self.projection

    def footprint(self, record):
        pose = camera_pose(record)
        if pose is None:
            return None
        camera = self._camera(record)
        return camera.get_projection(*pose, max_range=self.max_range) if camera is not None else None

    def pixel(self, record, x, y):
        pose = camera_pose(record)
        if pose is None or not 0 <= x < self.width or not 0 <= y < self.height:
            return None
        camera = self._camera(record)
        return camera.get_latlonalt_for_pixel(x, y, *pose, max_range=self.max_range) if camera is not None else None


def image_rect(panel_width, panel_height, width, height):
    scale = min(panel_width / width, panel_height / height)
    w, h = max(1, int(width * scale)), max(1, int(height * scale))
    return (panel_width - w) // 2, (panel_height - h) // 2, w, h


def image_pixel(rect, width, height, x, y):
    left, top, w, h = rect
    if left <= x < left + w and top <= y < top + h:
        return (x - left) * width / w, (y - top) * height / h
    return None
