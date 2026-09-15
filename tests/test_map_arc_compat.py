"""Map startup and mission rendering with and without the arc command enum."""
import importlib
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil


@pytest.fixture(params=[True, False], ids=['with-enum', 'without-enum'])
def arc_dialect(request, monkeypatch):
    if request.param:
        monkeypatch.setattr(mavutil.mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT', 36, raising=False)
    else:
        monkeypatch.delattr(mavutil.mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT', raising=False)


def test_map_loads_and_labels_arc_waypoints(arc_dialect):
    from MAVProxy.modules import mavproxy_map
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    importlib.reload(mavproxy_map)
    state = SimpleNamespace(
        multi_instance={}, instance_count={}, public_modules={},
        command_map={}, completions={}, completion_functions={},
        settings=SimpleNamespace(guidedalt=100, flytoframe='AboveHome'),
        module=lambda name: None)
    with mock.patch.object(mp_slipmap, 'MPSlipMap'):
        module = mavproxy_map.init(state)
    assert state.public_modules['map'] is module
    assert module._colour_for_wp_command[36] == (64, 255, 255)
    assert module._label_suffix_for_wp_command[36] == 'AW'


def test_mission_arcs_recognizes_numeric_command(arc_dialect):
    from MAVProxy.modules.mavproxy_map import mp_slipmap_util
    importlib.reload(mp_slipmap_util)
    waypoints = [
        SimpleNamespace(command=16, param1=0),
        SimpleNamespace(command=36, param1=90),
        SimpleNamespace(command=16, param1=45),
        SimpleNamespace(command=36, param1=-60),
    ]
    loader = SimpleNamespace(wp=lambda index: waypoints[index])
    assert mp_slipmap_util.mission_arcs(loader, [0, 1, 2, 3]) == {0: 90, 2: -60}
    assert mp_slipmap_util.mission_arcs(loader, [0, 2]) == {}


def test_map3d_renders_numeric_arc_command(arc_dialect):
    pytest.importorskip('vtk')
    pytest.importorskip('quantized_mesh_tile')
    from MAVProxy.modules.mavproxy_map3d import elements
    from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
    importlib.reload(elements)
    manager = elements.ElementManager(mock.Mock(), -35, 149, 1)
    items = [
        MissionItem(-35, 149, 100, 0, 16, 0),
        MissionItem(-35.001, 149.001, 100, 0, 36, 1, 90),
    ]
    with mock.patch.object(manager, 'refresh_mission'):
        manager.set_mission(items)
    assert len(manager.mission_markers) == 2
    assert len(manager.mission_line) > 2
