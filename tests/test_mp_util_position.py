'''gps_newpos, gps_offset, gps_distance and gps_bearing at the antimeridian and the poles'''

import math
import random

import pytest

from MAVProxy.modules.lib import mp_util


def assert_valid_longitude(lon):
    assert -180.0 <= lon <= 180.0, lon


def test_newpos_going_west_over_the_antimeridian_wraps_the_longitude():
    # this used to give -185.79 (fmod keeps the sign of its first argument)
    lat, lon = mp_util.gps_newpos(-79.35121282998307, -176.48548038106568,
                                  247.92906419752563, 200000)
    assert lon == pytest.approx(174.2093885, abs=1e-6)
    assert lat == pytest.approx(-80.0263083, abs=1e-6)


def test_newpos_going_east_over_the_antimeridian_wraps_the_longitude():
    lat, lon = mp_util.gps_newpos(0.0, 179.9, 90.0, 20000)
    expected = 179.9 + math.degrees(20000 / mp_util.radius_of_earth) - 360.0
    assert lon == pytest.approx(expected, abs=1e-9)
    assert lat == pytest.approx(0.0, abs=1e-9)


def test_offset_going_west_over_the_antimeridian_wraps_the_longitude():
    lat, lon = mp_util.gps_offset(-79.35121282998307, -176.48548038106568,
                                  -185343.87148240767, -75150.8436673385)
    assert lon == pytest.approx(174.2093885, abs=1e-6)


def test_newpos_longitude_is_always_valid():
    rnd = random.Random(1)
    for _ in range(2000):
        lat = rnd.uniform(-89.0, 89.0)
        lon = rnd.uniform(-180.0, 180.0)
        bearing = rnd.uniform(0.0, 360.0)
        distance = rnd.choice([10, 1000, 100000, 3000000])
        new_lat, new_lon = mp_util.gps_newpos(lat, lon, bearing, distance)
        assert_valid_longitude(new_lon)
        assert -90.0 <= new_lat <= 90.0


@pytest.mark.parametrize('lat1,lon1,lat2,lon2', [
    (-90.0, 10.0, -90.0, 10.0),
    (-90.0, 10.0, 0.0, 0.0),
    (90.0, 10.0, 0.0, 0.0),
    (0.0, 0.0, -90.0, 10.0),
    (-90.0, 0.0, 45.0, 170.0),
])
def test_distance_and_bearing_work_at_the_poles(lat1, lon1, lat2, lon2):
    # the rhumb line formula divides by zero at the south pole itself
    assert math.isfinite(mp_util.gps_distance(lat1, lon1, lat2, lon2))
    assert 0.0 <= mp_util.gps_bearing(lat1, lon1, lat2, lon2) <= 360.0


def test_distance_from_the_south_pole_to_the_equator():
    quarter = math.pi / 2 * mp_util.radius_of_earth
    assert mp_util.gps_distance(-90.0, 10.0, 0.0, 0.0) == pytest.approx(quarter, rel=1e-3)
    assert mp_util.gps_distance(90.0, 10.0, 0.0, 0.0) == pytest.approx(quarter, rel=1e-3)
