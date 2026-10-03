'''Geometry for a rectangular survey with a downward-facing camera.'''

import math
from dataclasses import dataclass

from MAVProxy.modules.lib import mp_util


MAX_WAYPOINTS = 2000
# Dialog names mapped to the mission editor's existing frame labels.
FRAMES = {'AboveHome': 'Rel', 'AGL': 'AGL', 'AMSL': 'Abs'}


@dataclass
class Survey:
    points: list
    spacing: float
    height_agl: float


def camera_height(height, frame, home_amsl=None, terrain_amsl=None):
    '''Convert frame-native mission altitude to camera AGL at the origin.'''
    if not math.isfinite(height):
        raise ValueError('Mission height must be finite')
    if frame not in FRAMES:
        raise ValueError('Choose AboveHome, AGL or AMSL')
    if frame != 'AGL':
        if terrain_amsl is None or not math.isfinite(terrain_amsl):
            raise ValueError('Waiting for terrain at the starting point; or choose AGL')
        if frame == 'AboveHome':
            if home_amsl is None or not math.isfinite(home_amsl):
                raise ValueError('Home altitude is unavailable')
            height += home_amsl
        height -= terrain_amsl
    if height <= 0:
        raise ValueError('Mission height must put the camera above ground')
    return height


def generate_survey(origin, length, breadth, height_agl, fov, rotation, overlap):
    '''Start at the origin, fly length along rotation, stepping breadth right.

    Camera FOV and overlap are cross-track. Edge lanes are included and their
    spacing never exceeds the requested spacing. Each endpoint is projected
    independently from the origin to avoid accumulated rounding errors.
    '''
    values = (*origin, length, breadth, height_agl, fov, rotation, overlap)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('All survey parameters must be finite numbers')
    lat, lon = origin
    if not -90 < lat < 90 or not -180 <= lon <= 180:
        raise ValueError('Invalid starting latitude or longitude')
    if not 0 < length <= 10000 or not 0 < breadth <= 10000:
        raise ValueError('Length and breadth must be between 0 and 10000 m')
    if height_agl <= 0:
        raise ValueError('Camera height above ground must be positive')
    if not 0 < fov < 180:
        raise ValueError('Camera FOV must be between 0 and 180 degrees')
    if not 0 <= overlap < 100:
        raise ValueError('Overlap must be from 0 to less than 100 percent')
    swath = 2 * height_agl * math.tan(math.radians(fov) / 2)
    max_spacing = swath * (1 - overlap / 100)
    if not math.isfinite(max_spacing) or max_spacing <= 0:
        raise ValueError('Camera footprint is too small or too large')
    intervals = breadth / max_spacing
    if not math.isfinite(intervals) or intervals > MAX_WAYPOINTS // 2 - 1:
        raise ValueError('Survey exceeds %u points; reduce breadth/overlap or increase height/FOV'
                         % MAX_WAYPOINTS)
    intervals = max(1, math.ceil(intervals))
    spacing = breadth / intervals
    angle = math.radians(rotation % 360)
    points = []
    for lane in range(intervals + 1):
        across = lane * spacing
        ends = (0, length) if lane % 2 == 0 else (length, 0)
        for along in ends:
            north = along * math.cos(angle) - across * math.sin(angle)
            east = along * math.sin(angle) + across * math.cos(angle)
            # gps_newpos uses rhumb lines; disallow routes crossing a pole.
            if abs(lat) + math.degrees(math.hypot(north, east) / mp_util.radius_of_earth) >= 90:
                raise ValueError('Survey is too close to a pole')
            point_lat, point_lon = mp_util.gps_offset(lat, lon, east, north)
            points.append((point_lat, mp_util.wrap_180(point_lon)))
    return Survey(points, spacing, height_agl)
