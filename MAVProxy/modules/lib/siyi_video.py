"""SIYI vendor subtitle telemetry and optional timestamped MAVLink log data."""
import bisect
from datetime import datetime, timezone
import math
import re


def coordinate(text, latitude):
    """SIYI stores coordinates as hemisphere-prefixed DMS, to 0.1 arcsecond."""
    match = re.fullmatch(r'([NSEW])(\d+)°(\d+)[′\']([\d.]+)[″"]', text.strip())
    if match is None or match[1] not in ('NS' if latitude else 'EW'):
        raise ValueError('Invalid SIYI coordinate')
    degrees, minutes, seconds = map(float, match.groups()[1:])
    value = degrees + minutes / 60 + seconds / 3600
    if not 0 <= minutes < 60 or not 0 <= seconds < 60 or value > (90 if latitude else 180):
        raise ValueError('Invalid SIYI coordinate')
    return -value if match[1] in 'SW' else value


def parse_subtitle(payload):
    """Read tx3g's length-prefixed UTF-8 text; ignore trailing style boxes."""
    from MAVProxy.modules.lib.video_telemetry import finite
    if len(payload) < 2:
        return None
    size = int.from_bytes(payload[:2], 'big')
    if not size or size + 2 > len(payload):
        return None
    try:
        text = payload[2:2 + size].decode('utf-8')
        first = re.match(r'(\d{4}-\d\d-\d\d_\d\d-\d\d-\d\d) stream->FrameCnt:(\d+)\s', text)
        if first is None:
            return None
        utc = datetime.strptime(first[1], '%Y-%m-%d_%H-%M-%S').replace(tzinfo=timezone.utc).timestamp()
        fields = dict(re.findall(r'([a-z_]+):([^\n,]+)', text))
        lat = coordinate(fields['latitude'], True)
        lon = coordinate(fields['longitude'], False)
        altitude = float(fields['abs_alt'])
        angles = [float(fields['gb_' + key]) for key in ('roll', 'pitch', 'yaw')]
        if not all(finite(v) for v in [altitude, *angles]):
            return None
        zoom = float(fields.get('mix_ratio', '1'))
        if not finite(zoom) or zoom <= 0:
            zoom = None
        return {'schema': 'siyi.subtitle.v1', 'frame_counter': int(first[2]),
                'utc_us': round(utc * 1e6), 'clock_resolution_s': 1,
                'position': {'lat_e7': round(lat * 1e7), 'lon_e7': round(lon * 1e7),
                             'alt_amsl_m': altitude, 'age_ms': 0},
                # SIYI duplicates absolute altitude in rel_alt. Do not expose
                # that value as height above home/terrain.
                'gimbal_attitude': dict(zip(('roll_rad', 'pitch_rad', 'yaw_rad'), map(math.radians, angles)), age_ms=0),
                'vehicle_attitude': None, 'heading_rad': None, 'zoom': zoom}
    except (ValueError, KeyError, UnicodeError, OverflowError):
        return None


def attach_subtitles(index, subtitles, origin):
    """Match both PTS and frame counter. Never shift broken rollover tracks."""
    if not subtitles:
        return 0, 0
    first_pts, first = subtitles[0]
    if first['frame_counter'] != 0 or abs(first_pts - origin) > .025:
        return 0, len(subtitles)
    assigned = rejected = 0
    for pts, record in subtitles:
        pts -= origin
        # Subtitle ticks are 1/25 s on thermal recordings, video ticks ~1/30 s.
        # The frame counter disambiguates nearby or duplicate rounded timestamps.
        number = record['frame_counter']
        if not 0 <= number < len(index.samples) or abs(index.times[number] - pts) > .025:
            rejected += 1
            continue
        if index.samples[number].record is None:
            index.samples[number].record = record
            assigned += 1
    return assigned, rejected


class FlightLog:
    """Read a local tlog and select the first autopilot (or an explicit system)."""
    def __init__(self, filename, system_id=None):
        from pathlib import Path
        from pymavlink import mavutil
        path = Path(filename).resolve(strict=True)
        if not path.is_file():
            raise ValueError('--tlog must name a local file')
        self.data = {'ATTITUDE': [], 'GLOBAL_POSITION_INT': []}
        self.system_id = system_id
        # mavmmaplog avoids treating a filename as a serial/network connection.
        log = mavutil.mavmmaplog(str(path))
        try:
            while True:
                message = log.recv_match(type=['HEARTBEAT', *self.data])
                if message is None:
                    break
                kind = message.get_type()
                if self.system_id is None and kind == 'HEARTBEAT':
                    if (message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 and
                            message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and
                            message.type != mavutil.mavlink.MAV_TYPE_GCS):
                        self.system_id = message.get_srcSystem()
                if (kind in self.data and message.get_srcSystem() == self.system_id and
                        message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                    self.data[kind].append((message._timestamp, message))
        finally:
            log.close()
        self.times = {}
        for kind, items in self.data.items():
            items.sort(key=lambda item: item[0])
            self.times[kind] = [item[0] for item in items]
        if not any(self.data.values()):
            raise ValueError('No flight-controller attitude/position messages found in tlog')

    def nearest(self, kind, timestamp):
        items, times = self.data[kind], self.times[kind]
        offset = bisect.bisect_left(times, timestamp)
        choices = items[max(0, offset - 1):offset + 1]
        if not choices:
            return None
        when, message = min(choices, key=lambda item: abs(item[0] - timestamp))
        return message if abs(when - timestamp) <= 1 else None

    def apply(self, index, offset=0):
        from MAVProxy.modules.lib.video_telemetry import finite
        count = 0
        for sample in index.samples:
            record = sample.record
            if record is None or record.get('schema') != 'siyi.subtitle.v1':
                continue
            when = record['utc_us'] * 1e-6 + offset
            att = self.nearest('ATTITUDE', when)
            if att is not None and all(finite(v) for v in (att.roll, att.pitch, att.yaw)):
                record['vehicle_attitude'] = dict(roll_rad=att.roll, pitch_rad=att.pitch, yaw_rad=att.yaw, age_ms=0)
                record['heading_rad'] = att.yaw
                count += 1
            pos = self.nearest('GLOBAL_POSITION_INT', when)
            if pos is not None:
                if pos.hdg != 65535:
                    record['heading_rad'] = math.radians(pos.hdg * .01)
                sample.speed = math.hypot(pos.vx, pos.vy) * .01
                sample.speed_source = 'tlog'
        return count
