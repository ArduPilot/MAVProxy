"""Decode MISB ST 0601 KLV into the recorded-video player's pose fields.

The container owns timing. No KLV timestamp is used to guess a packet's video
frame, and no state is carried across local sets to manufacture missing fields.
"""
import math

UAS_KEY = bytes.fromhex('060e2b34020b01010e01030101000000')


def _ber(data, offset, oid=False):
    if offset >= len(data):
        raise ValueError('Truncated BER value')
    value = data[offset]
    offset += 1
    if oid:
        result = value & 127
        for _ in range(4):
            if not value & 128:
                return result, offset
            if offset >= len(data):
                break
            value = data[offset]
            offset += 1
            result = (result << 7) | (value & 127)
        raise ValueError('Invalid BER-OID tag')
    if not value & 128:
        return value, offset
    count = value & 127
    if not count or count > 4 or offset + count > len(data):
        raise ValueError('Invalid BER length')
    return int.from_bytes(data[offset:offset+count], 'big'), offset+count


def _rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def _decode(packet, offset):
    # The checksum covers the UL and BER length too, excluding only its value.
    if len(packet) < offset+4 or packet[-4:-2] != b'\x01\x02':
        raise ValueError('Missing ST 0601 checksum')
    checksum = sum(byte << (0 if i % 2 else 8) for i, byte in enumerate(packet[:-2])) & 65535
    if checksum != int.from_bytes(packet[-2:], 'big'):
        raise ValueError('Invalid ST 0601 checksum')
    items = {}
    order = []
    while offset < len(packet):
        tag, offset = _ber(packet, offset, oid=True)
        size, offset = _ber(packet, offset)
        if tag in items or offset+size > len(packet):
            raise ValueError('Invalid ST 0601 local set')
        items[tag] = packet[offset:offset+size]
        order.append(tag)
        offset += size
    if not order or order[0] != 2 or len(items[2]) != 8 or len(items.get(65, b'')) != 1:
        raise ValueError('Missing ST 0601 timestamp/version')

    def unsigned(tag, size, scale, bias=0):
        value = items.get(tag, b'')
        if len(value) != size:
            return None
        return int.from_bytes(value, 'big') * scale / ((1 << (8*size))-1) + bias

    def signed(tag, size, scale):
        value = items.get(tag, b'')
        if len(value) != size:
            return None
        n = int.from_bytes(value, 'big', signed=True)
        maximum = (1 << (8*size-1))-1
        return None if n == -maximum-1 else n * scale / maximum

    record = dict(schema='misb.st0601', utc_us=int.from_bytes(items[2], 'big'),
                  position=None, vehicle_attitude=None, gimbal_attitude=None,
                  heading_rad=None, zoom=None, hfov_deg=unsigned(16, 2, 180))
    lat, lon, alt = signed(13, 4, 90), signed(14, 4, 180), unsigned(15, 2, 19900, -900)
    if None not in (lat, lon, alt):
        record['position'] = dict(lat_e7=round(lat*1e7), lon_e7=round(lon*1e7), alt_amsl_m=alt, age_ms=0)
    yaw = unsigned(5, 2, 360)
    # ST 0601.8-16 requires full-range values to supersede legacy ones,
    # including an explicit unknown/error value. Older recordings still work.
    pitch = signed(90, 4, 90) if 90 in items else signed(6, 2, 20)
    roll = signed(91, 4, 90) if 91 in items else signed(7, 2, 50)
    if None not in (roll, pitch, yaw):
        roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
        record['vehicle_attitude'] = dict(roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw, age_ms=0)
        record['heading_rad'] = yaw
        azimuth, elevation, relative_roll = unsigned(18, 4, 360), signed(19, 4, 180), unsigned(20, 4, 360)
        if None not in (azimuth, elevation, relative_roll):
            platform = _rotation(roll, pitch, yaw)
            relative = _rotation(*map(math.radians, (relative_roll, elevation, azimuth)))
            earth = [[sum(platform[i][k]*relative[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
            horizontal = math.hypot(earth[0][0], earth[1][0])
            camera_pitch = math.atan2(-earth[2][0], horizontal)
            camera_yaw = math.atan2(earth[1][0], earth[0][0]) if horizontal > 1e-8 else math.atan2(-earth[0][1], earth[1][1])
            camera_roll = math.atan2(earth[2][1], earth[2][2]) if horizontal > 1e-8 else 0
            # Shared projection/OSD uses level-referenced roll/pitch and
            # vehicle-relative yaw, unlike MISB's full body-relative rotation.
            record['gimbal_attitude'] = dict(roll_rad=camera_roll, pitch_rad=camera_pitch,
                yaw_rad=(camera_yaw-yaw+math.pi) % (2*math.pi)-math.pi, age_ms=0)
    return record


def records(data):
    """Yield valid UAS local sets from a demuxed KLV packet. Ignore other ULs.

    FFmpeg strips synchronous ST 1402 access-unit cells before delivering the
    packet; asynchronous KLV uses the same local-set decoder.
    """
    offset = 0
    while offset+17 <= len(data):
        start = offset
        key = data[offset:offset+16]
        try:
            size, value = _ber(data, offset+16)
            end = value+size
            if end > len(data):
                return
            offset = end
            if key == UAS_KEY:
                yield _decode(data[start:end], value-start)
        except (ValueError, KeyError, OverflowError):
            if offset == start:
                return
            continue
