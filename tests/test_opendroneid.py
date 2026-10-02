"""OpenDroneID timestamps count seconds from 00:00:00 UTC on 2019-01-01."""
from datetime import datetime, timezone
from types import SimpleNamespace

from MAVProxy.modules import mavproxy_OpenDroneID


def test_timestamp_2019_epoch_is_utc(monkeypatch):
    jan_1_2019 = datetime(2019, 1, 1, tzinfo=timezone.utc).timestamp()
    monkeypatch.setattr(mavproxy_OpenDroneID.time, 'time', lambda: jan_1_2019 + 90)
    assert mavproxy_OpenDroneID.OpenDroneIDModule.timestamp_2019(SimpleNamespace()) == 90
