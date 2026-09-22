"""Lossless APCG thermal Matroska reader. Pixel values remain uint16 Kelvin*64.

PyAV exposes Matroska BlockAdditional as packet side data; metadata is paired
with the compressed frame before decoding, not joined to live MAVLink state.
"""
import json
import math
from urllib.parse import urlsplit

STREAM_TYPE_HTTP_MATROSKA = 200  # experimental, not allocated by MAVLink upstream
SCHEMA = 'apcg.thermal.v1'


def is_raw_thermal(stream):
    return int(getattr(stream, 'type', -1)) == STREAM_TYPE_HTTP_MATROSKA


def parse_metadata(data):
    if len(data) < 8 or len(data) > 65544 or int.from_bytes(data[:8], 'big') != 0x41504347:
        raise ValueError('invalid thermal BlockAdditional')
    m = json.loads(data[8:])
    if not isinstance(m, dict) or m.get('schema') != SCHEMA:
        raise ValueError('unsupported thermal metadata schema')
    if (m.get('width'), m.get('height'), m.get('pixel_format'), m.get('bits_per_sample')) != (640, 512, 'gray16le', 16):
        raise ValueError('unsupported radiometric pixel format')
    for key in ('frame_id', 'capture_monotonic_us'):
        if type(m.get(key)) is not int or m[key] < 1:
            raise ValueError('invalid thermal frame timestamp or sequence')
    for key in ('temperature_scale_k', 'temperature_offset_k', 'hfov_deg', 'minimum_c', 'maximum_c'):
        value = m.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('invalid thermal calibration')
    if m['temperature_scale_k'] <= 0 or not 0 < m['hfov_deg'] < 180:
        raise ValueError('invalid thermal scale or FOV')
    if m.get('rotation_deg') not in (0, 180):
        raise ValueError('unsupported thermal rotation')
    return m


def projection_pose(metadata):
    """Use only this frame's telemetry; refuse missing/stale state for mapping."""
    from MAVProxy.modules.lib.video_telemetry import camera_pose
    if metadata.get('altitude_datum') != 'AMSL' or metadata.get('gimbal_frame') != 'roll_pitch_level_yaw_vehicle':
        return None
    record = metadata.get('telemetry')
    if not isinstance(record, dict):
        return None
    for key in ('position', 'vehicle_attitude', 'gimbal_attitude'):
        state = record.get(key)
        if not isinstance(state, dict):
            return None
        age = state.get('age_ms')
        if type(age) not in (int, float) or not 0 <= age <= 250:
            return None
    if camera_pose(record) is None:
        return None
    # Source ages are explicit. Extrapolate known yaw rates and position to
    # capture time, with the same short horizon as the camera controller.
    record = {k: (dict(v) if isinstance(v, dict) else v) for k, v in record.items()}
    for key in ('vehicle_attitude', 'gimbal_attitude'):
        rate = record[key].get('yaw_rate_rad_s')
        if type(rate) in (int, float) and math.isfinite(rate):
            record[key]['yaw_rad'] += rate * record[key]['age_ms'] * .001
    pose = camera_pose(record)
    if pose is None:
        return None
    velocity = record.get('velocity')
    velocity_age = velocity.get('age_ms') if isinstance(velocity, dict) else None
    if type(velocity_age) in (int, float) and 0 <= velocity_age <= 250:
        v = [velocity.get(k) for k in ('vn_m_s', 've_m_s', 'vd_m_s')]
        if all(type(x) in (int, float) and math.isfinite(x) for x in v):
            from MAVProxy.modules.lib.mp_util import gps_offset
            lat, lon, alt, roll, pitch, yaw = pose
            dt = record['position']['age_ms'] * .001
            lat, lon = gps_offset(lat, lon, v[1]*dt, v[0]*dt)
            pose = lat, lon, alt-v[2]*dt, roll, pitch, yaw
    return pose


class ThermalReader:
    def __init__(self, uri):
        try:
            import av
        except ImportError as error:
            raise RuntimeError('Raw thermal viewing requires PyAV (python3 -m pip install av)') from error
        parsed = urlsplit(str(uri))
        if parsed.scheme not in ('http', 'https', 'file', ''):
            raise ValueError('thermal stream must be HTTP(S) or a local Matroska file')
        # The camera omits DefaultDuration because the rate changes live, so
        # disable fps probing: it would read up to 64 KB of frames inside the
        # open timeout, and an interrupted probe poisons the demuxer.
        self.container = av.open(str(uri), timeout=(3, 3),
                                 options={'probesize': '65536', 'analyzeduration': '0',
                                          'fpsprobesize': '0'})
        streams = self.container.streams.video
        if len(streams) != 1 or streams[0].codec_context.name != 'ffv1':
            self.close()
            raise ValueError('thermal stream must contain one FFV1 video track')
        self.stream = streams[0]
        if (self.stream.width, self.stream.height) != (640, 512):
            self.close()
            raise ValueError('unexpected thermal dimensions')

    def frames(self):
        for packet in self.container.demux(self.stream):
            if not packet.size:
                continue
            if not hasattr(packet, 'get_sidedata'):
                raise RuntimeError('PyAV with packet side-data support is required (18.1 tested)')
            side = packet.get_sidedata('matroska_block_additional')
            metadata = parse_metadata(bytes(side))
            frames = packet.decode()
            if len(frames) != 1:
                raise ValueError('expected one independently coded thermal frame per packet')
            frame = frames[0]
            if frame.format.name != 'gray16le':
                raise ValueError('thermal decoder did not preserve 16-bit grayscale')
            pixels = frame.to_ndarray()
            yield pixels, metadata

    def close(self):
        if self.container is not None:
            self.container.close()
            self.container = None
