"""Regression tests for the ADS-B threat table bound (issue #1789)."""

from types import SimpleNamespace

import pytest

from MAVProxy.modules import mavproxy_adsb


@pytest.fixture
def adsb_module():
    """ADSB module built without __init__: no mpstate, no map, no master."""
    module = mavproxy_adsb.ADSBModule.__new__(mavproxy_adsb.ADSBModule)
    module.threat_vehicles = {}
    module.module_matching = lambda pattern: []
    module.now = [1.0]
    module.get_time = lambda: module.now[0]
    module.ADSB_settings = SimpleNamespace(timeout=5)
    return module


def vehicle_state(icao):
    return {
        "ICAO_address": icao,
        "lat": 0,
        "lon": 0,
        "altitude": 1000,
        "callsign": "TEST",
        "heading": 0,
        "squawk": 0,
        "emitter_type": 1,
    }


def test_repeated_address_keeps_single_entry(adsb_module):
    for _ in range(100):
        adsb_module.add_vehicle(vehicle_state(0x400000))
    assert len(adsb_module.threat_vehicles) == 1


def test_table_is_bounded_at_max_tracked_vehicles(adsb_module):
    cap = mavproxy_adsb.MAX_TRACKED_VEHICLES
    for icao in range(0x400000, 0x400000 + cap):
        adsb_module.add_vehicle(vehicle_state(icao))
    assert len(adsb_module.threat_vehicles) == cap
    adsb_module.add_vehicle(vehicle_state(0x400000 + cap))
    assert len(adsb_module.threat_vehicles) == cap
    assert "ADSB-%d" % (0x400000 + cap) not in adsb_module.threat_vehicles


def test_existing_vehicle_still_updates_when_table_full(adsb_module):
    cap = mavproxy_adsb.MAX_TRACKED_VEHICLES
    for icao in range(0x400000, 0x400000 + cap):
        adsb_module.add_vehicle(vehicle_state(icao))
    updated = vehicle_state(0x400000)
    updated["callsign"] = "NEWCALL"
    adsb_module.add_vehicle(updated)
    assert len(adsb_module.threat_vehicles) == cap
    assert adsb_module.threat_vehicles["ADSB-%d" % 0x400000].state["callsign"] == "NEWCALL"


def test_check_threat_timeout_removes_all_expired(adsb_module):
    for icao in range(0x400000, 0x400000 + 10):
        adsb_module.add_vehicle(vehicle_state(icao))
    for vehicle in adsb_module.threat_vehicles.values():
        vehicle.update_time = 1.0
    adsb_module.now[0] = 10.0
    adsb_module.check_threat_timeout()
    assert len(adsb_module.threat_vehicles) == 0


def test_check_threat_timeout_keeps_fresh_vehicles(adsb_module):
    for icao in range(0x400000, 0x400000 + 10):
        adsb_module.add_vehicle(vehicle_state(icao))
    for vehicle in adsb_module.threat_vehicles.values():
        vehicle.update_time = 1.0
    adsb_module.add_vehicle(vehicle_state(0x500000))
    adsb_module.threat_vehicles["ADSB-%d" % 0x500000].update_time = 9.0
    adsb_module.now[0] = 10.0
    adsb_module.check_threat_timeout()
    assert list(adsb_module.threat_vehicles) == ["ADSB-%d" % 0x500000]
