"""Index AP_CameraGimbal SEI and SIYI subtitle telemetry on the video's presentation clock.

No MAVLink connection is needed. ffprobe demuxes packets without decoding video;
MP4 packet offsets allow us to read compressed samples directly from the file.
"""
import bisect
from dataclasses import dataclass
from fractions import Fraction
import json
import math
import pathlib
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

UUID = bytes.fromhex('8d646b4e556f4a908b7c35e629510321')
START = re.compile(b'\x00\x00(?:\x00)?\x01')


def unhex(text):
    """Decode ffprobe's padded hex dump (also after XML newline normalization)."""
    return bytes.fromhex(''.join(re.findall(
        r'(?:^|\s)[0-9a-fA-F]{8,}: ([0-9a-fA-F ]{39})', text)))


def packet_nals(packet, length_size):
    if length_size:
        offset = 0
        while offset + length_size <= len(packet):
            size = int.from_bytes(packet[offset:offset + length_size], 'big')
            offset += length_size
            if size == 0 or offset + size > len(packet):
                return
            yield packet[offset:offset + size]
            offset += size
    else:
        starts = list(START.finditer(packet))
        for i, start in enumerate(starts):
            end = starts[i + 1].start() if i + 1 < len(starts) else len(packet)
            yield packet[start.end():end].rstrip(b'\0')


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


class VideoIndex:
    def __init__(self, filename):
        self.filename = str(pathlib.Path(filename).resolve(strict=True))
        info = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-show_entries',
            'stream=index,codec_type,codec_name,width,height,avg_frame_rate,extradata:format=format_name',
            '-show_data', '-of', 'json', self.filename]))
        streams = [s for s in info.get('streams', []) if s.get('codec_type') == 'video']
        subtitle_ids = {s['index'] for s in info.get('streams', []) if s.get('codec_name') == 'mov_text'}
        if not streams or streams[0]['codec_name'] not in ('h264', 'hevc'):
            raise ValueError('An H.264 or H.265 video recording is required')
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
        direct = 'mov' in info.get('format', {}).get('format_name', '').split(',')
        entries = 'packet=stream_index,pts_time,duration_time,pos,size' + ('' if direct else ',data')
        command = ['ffprobe', '-v', 'error', '-show_packets',
                   '-show_entries', entries, '-of', 'xml']
        if not direct:
            command += ['-show_data']
        self.samples = []
        self.warnings = []
        subtitles = {}
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
                    if packet_stream != stream['index'] and packet_stream not in subtitle_ids:
                        element.clear()
                        if packets_element is not None:
                            packets_element.clear()
                        continue
                    if direct:
                        video.seek(int(attrs['pos']))
                        packet = video.read(int(attrs['size']))
                    else:
                        packet = unhex(attrs.get('data', ''))
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
        self.samples.sort(key=lambda sample: sample.time)
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
            detail = '; '.join(self.warnings) or 'No AP_CameraGimbal SEI or SIYI subtitle telemetry found'
            raise ValueError(detail)
        self._speeds()

    def frame_at(self, seconds):
        return max(0, min(len(self.samples) - 1, bisect.bisect_right(self.times, seconds) - 1))

    def _speeds(self):
        from MAVProxy.modules.lib import mp_util
        previous = None
        speed = None
        for sample in self.samples:
            pos = position(sample.record)
            raw = source(sample.record, 'position')
            utc = sample.record.get('utc_us') if sample.record else None
            if pos is None or not finite(utc):
                previous, speed = None, None
                continue
            timestamp = utc * 1e-6 - raw.get('age_ms', 0) * .001
            if previous is not None:
                dt = timestamp - previous[0]
                # Repeated snapshots include the same GPS fix with increasing age.
                # Millisecond age quantization can jitter the reconstructed time.
                if dt < -.01 or dt > 10:
                    speed = None
                    previous = None
                elif dt > .02:
                    speed = mp_util.gps_distance(*previous[1][:2], *pos[:2]) / dt
                    previous = (timestamp, pos)
            if previous is None:
                previous = (timestamp, pos)
            sample.speed = speed


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
