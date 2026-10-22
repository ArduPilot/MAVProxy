"""NMEA output sentences use fixed-width time and position fields."""
import time
from types import SimpleNamespace

from MAVProxy.modules import mavproxy_nmeaout

module = mavproxy_nmeaout.NMEAOutModule


def make_module():
    nmea = module.__new__(module)
    nmea.output_time = 0.0
    return nmea


def test_time_keeps_two_digit_seconds():
    # 2020-01-01 12:34:05.5 UTC
    assert make_module().format_time(1577882045.5) == "123405.500"


def test_time_rounding_carries_into_next_minute():
    # 12:34:59.9996 rounds to 12:35:00.000, not 12:34:60.000
    assert make_module().format_time(1577882099.9996) == "123500.000"


def test_position_minutes():
    nmea = make_module()
    assert nmea.format_lat(-35.5) == "3530.00000,S"
    assert nmea.format_lon(149.25) == "14915.00000,E"
    assert nmea.format_lon(-8.0) == "00800.00000,W"


def test_position_rounding_carries_into_next_degree():
    nmea = make_module()
    assert nmea.format_lat(35.9999999999) == "3600.00000,N"


def test_sentences_written_as_bytes(tmp_path, monkeypatch):
    out = tmp_path / "nmea.txt"
    nmea = make_module()
    nmea.serial = open(out, "wb")
    monkeypatch.setattr(time, "time", lambda: 1577882045.5)
    msg = SimpleNamespace(fix_type=3, lat=-353632610, lon=1491652300, vel=1000, cog=9000,
                          satellites_visible=10, eph=121, alt=584000,
                          get_type=lambda: "GPS_RAW_INT")
    nmea.mavlink_packet(msg)
    nmea.serial.close()
    lines = out.read_bytes().split(b"\r\n")
    assert lines[0].startswith(b"$GPGGA,123405.500,3521.79566,S,14909.91380,E,")
    assert lines[1].startswith(b"$GPRMC,123405.500,A,")
