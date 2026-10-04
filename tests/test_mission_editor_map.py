'''Local mission drafts must render without changing vehicle transfer state.'''

import queue
import threading
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil, mavwp

from MAVProxy.modules.mavproxy_misseditor import get_mission_for_map, me_event, mission_editor


def mission(count):
    loader = mavwp.MAVWPLoader()
    for seq in range(count):
        loader.add(mavutil.mavlink.MAVLink_mission_item_message(
            1, 1, seq, 0 if seq == 0 else 3, 16, 0, 1,
            0, 0, 0, 0, -35 + seq * 0.001, 149, 100))
    return loader


@pytest.fixture
def backend():
    editor = mission_editor.MissionEditorMain.__new__(mission_editor.MissionEditorMain)
    editor.time_to_quit = False
    editor.map_mission = None
    editor.reading_mission = False
    editor.num_wps_expected = 0
    editor.wps_received = {}
    editor.gui_event_queue = queue.Queue()
    editor.gui_event_queue_lock = threading.Lock()
    master = mock.Mock()
    wp = SimpleNamespace(wploader=mission(3), loading_waypoints=False,
                         cmd_wp=mock.Mock(), wp_ftp_download=mock.Mock())
    modules = {'wp': wp, 'misseditor': SimpleNamespace(me_main=editor)}
    state = SimpleNamespace(
        module=modules.get, public_modules=modules, master=lambda: master,
        multi_instance={}, instance_count={}, command_map={}, completions={}, completion_functions={},
        vehicle_type='plane', functions=SimpleNamespace(get_mav_param=lambda *args: 40),
        settings=SimpleNamespace(target_system=1, target_component=1, guidedalt=100, flytoframe='AboveHome'))
    editor.mpstate = state
    events = queue.Queue()
    worker = mission_editor.MissionEditorEventThread(editor, events, threading.Lock())

    def process(kind, **args):
        events.put(me_event.MissionEditorEvent(kind, **args))
        worker.time_to_quit = False
        with mock.patch.object(mission_editor.time, 'sleep',
                               side_effect=lambda _: setattr(worker, 'time_to_quit', True)):
            worker.run()

    return SimpleNamespace(editor=editor, state=state, master=master, wp=wp, process=process)


def test_draft_never_changes_transfer_cache_or_sends_to_controller(backend):
    original = backend.wp.wploader
    draft = mission(5)
    backend.process(me_event.MEE_MAP_MISSION, wploader=draft)
    assert get_mission_for_map(backend.state) is draft
    assert backend.wp.wploader is original
    assert backend.wp.wploader.count() == 3
    assert not backend.wp.loading_waypoints
    assert backend.master.mock_calls == []
    backend.wp.cmd_wp.assert_not_called()
    backend.wp.wp_ftp_download.assert_not_called()
    backend.process(me_event.MEE_MAP_MISSION, wploader=None)
    assert get_mission_for_map(backend.state) is original


def test_draft_only_applies_to_current_vehicle_and_open_editor(backend):
    backend.process(me_event.MEE_MAP_MISSION, wploader=mission(5))
    backend.state.settings.target_system = 2
    assert get_mission_for_map(backend.state) is backend.wp.wploader
    backend.state.settings.target_system = 1
    backend.editor.time_to_quit = True
    assert get_mission_for_map(backend.state) is backend.wp.wploader


def test_map_replaces_route_and_labels_then_restores_controller_mission(backend):
    from MAVProxy.modules import mavproxy_map
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    with mock.patch.object(mp_slipmap, 'MPSlipMap'):
        display = mavproxy_map.MapModule(backend.state)
    display.map_settings.showwpnum = True
    display.map_settings.loitercircle = True
    display.idle_task()
    draft = mission(5)
    draft.wp(4).command = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
    draft.wp(4).param3 = 50
    backend.process(me_event.MEE_MAP_MISSION, wploader=draft)
    display.map.add_object.reset_mock()
    display.idle_task()
    objects = [call.args[0] for call in display.map.add_object.call_args_list]
    polygons = [obj for obj in objects if isinstance(obj, mp_slipmap.SlipPolygon)]
    assert polygons[0].points == draft.polygon_list()[0]
    assert polygons[0].popup_menu is None  # draft sequences cannot address the vehicle
    labels = [obj.label for obj in objects if isinstance(obj, mp_slipmap.SlipLabel)]
    assert display.label_for_waypoint(4) in labels
    assert any(isinstance(obj, mp_slipmap.SlipCircle) for obj in objects)
    assert get_mission_for_map(backend.state).count() == 5
    # Deleting everything still clears the old route.
    backend.process(me_event.MEE_MAP_MISSION, wploader=mission(1))
    display.map.add_object.reset_mock()
    display.idle_task()
    assert any(isinstance(call.args[0], mp_slipmap.SlipClearLayer)
               for call in display.map.add_object.call_args_list)
    backend.process(me_event.MEE_MAP_MISSION, wploader=None)
    display.idle_task()
    assert display.mission_list == backend.wp.wploader.view_list()
    assert backend.master.mock_calls == []


@pytest.mark.parametrize('count', [0, 3])
def test_mavlink_read_delivers_complete_mission_atomically(backend, count):
    backend.process(me_event.MEE_READ_WPS, use_ftp=False)
    backend.wp.cmd_wp.assert_called_once_with(['list'])
    backend.editor.process_mavlink_packet(mavutil.mavlink.MAVLink_mission_count_message(1, 1, count))
    original = mission(count)
    for seq in reversed(range(count)):
        assert backend.editor.gui_event_queue.empty()
        backend.editor.process_mavlink_packet(original.wp(seq))
    event = backend.editor.gui_event_queue.get_nowait()
    assert event.type == me_event.MEGE_READ_MISSION
    assert event.get_arg('wploader').count() == count
    assert [w.seq for w in event.get_arg('wploader').wpoints] == list(range(count))
    assert backend.editor.gui_event_queue.empty()
    assert not backend.editor.reading_mission


def test_console_read_after_editor_upload_cannot_overwrite_draft(backend):
    backend.state.settings.wp_use_mission_int = False
    backend.process(me_event.MEE_WRITE_WPS, use_ftp=False, count=2)
    for seq in range(2):
        backend.process(me_event.MEE_WRITE_WP_NUM, num=seq, frame=3, cmd_id=16,
                        p1=0, p2=0, p3=0, p4=0, lat=-35, lon=149, alt=100)
    assert backend.master.mav.send.call_count == 2
    draft = mission(5)
    backend.process(me_event.MEE_MAP_MISSION, wploader=draft)
    backend.master.reset_mock()

    backend.editor.process_mavlink_packet(mavutil.mavlink.MAVLink_mission_count_message(1, 1, 2))
    for item in mission(2).wpoints:
        backend.editor.process_mavlink_packet(item)
    assert backend.editor.gui_event_queue.empty()
    assert get_mission_for_map(backend.state) is draft
    assert backend.master.mock_calls == []


@pytest.mark.parametrize('old_count', [-1, 3])
def test_stale_receive_counters_cannot_enable_incremental_gui_updates(backend, old_count):
    backend.editor.num_wps_expected = old_count
    backend.editor.process_mavlink_packet(mavutil.mavlink.MAVLink_mission_count_message(1, 1, 3))
    for item in mission(3).wpoints:
        backend.editor.process_mavlink_packet(item)
    assert backend.editor.gui_event_queue.empty()
