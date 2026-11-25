from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from MAVProxy.modules.mavproxy_adsb import ADSBModule, ADSBVehicle


@pytest.fixture
def adsb_module():
    # Create the module without initializing the full MAVProxy state
    module = ADSBModule.__new__(ADSBModule)

    module.threat_vehicles = {}
    module.ADSB_settings = SimpleNamespace(timeout=5)

    # Use a fixed time for deterministic tests
    module.get_time = lambda: 10.0

    # Disable real map integration
    module.module_matching = lambda pattern: []

    return module


def add_record(module, icao, updated_at):
    key = f"ADSB-{icao}"

    vehicle = ADSBVehicle(id=key, state={})
    vehicle.update_time = updated_at

    module.threat_vehicles[key] = vehicle


def test_cleanup_removes_all_expired_records(adsb_module):
    # Create five expired vehicles
    for icao in range(5):
        add_record(adsb_module, icao, updated_at=1.0)

    adsb_module.check_threat_timeout()

    assert len(adsb_module.threat_vehicles) == 0


def test_cleanup_preserves_fresh_records(adsb_module):
    # Create five fresh vehicles
    for icao in range(5):
        add_record(adsb_module, icao, updated_at=8.0)

    adsb_module.check_threat_timeout()

    assert len(adsb_module.threat_vehicles) == 5


def test_cleanup_mixed_records(adsb_module):
    # Two expired vehicles
    add_record(adsb_module, 101, updated_at=1.0)
    add_record(adsb_module, 102, updated_at=2.0)

    # Two fresh vehicles
    add_record(adsb_module, 103, updated_at=8.0)
    add_record(adsb_module, 104, updated_at=9.0)

    adsb_module.check_threat_timeout()

    assert set(adsb_module.threat_vehicles) == {
        "ADSB-103",
        "ADSB-104",
    }


def test_cleanup_removes_map_objects(adsb_module):
    fake_map = Mock()
    fake_map_module = SimpleNamespace(map=fake_map)
    adsb_module.module_matching = lambda pattern: [fake_map_module]

    add_record(adsb_module, 101, updated_at=1.0)
    add_record(adsb_module, 102, updated_at=2.0)

    adsb_module.check_threat_timeout()

    assert len(adsb_module.threat_vehicles) == 0
    assert fake_map.remove_object.call_count == 4

    for icao in (101, 102):
        fake_map.remove_object.assert_any_call(f"ADSB-{icao}")
        fake_map.remove_object.assert_any_call(f"ADSB-{icao}:circle")



def test_cleanup_keeps_vehicle_at_timeout_boundary(adsb_module):
    add_record(adsb_module, 101, updated_at=5.0)

    adsb_module.check_threat_timeout()

    assert "ADSB-101" in adsb_module.threat_vehicles


def test_cleanup_initializes_zero_update_time(adsb_module):
    add_record(adsb_module, 101, updated_at=0)

    adsb_module.check_threat_timeout()

    vehicle = adsb_module.threat_vehicles["ADSB-101"]
    assert vehicle.update_time == 10.0
