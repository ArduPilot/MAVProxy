'''Real wx coverage: SURVEY_GUI_TEST=1 python3 -m pytest tests/test_survey_gui.py.'''

import os
import queue
import threading
import time
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil, mavwp

pytestmark = pytest.mark.skipif(os.environ.get('SURVEY_GUI_TEST') != '1',
                                reason='requires a GUI display (SURVEY_GUI_TEST=1)')


@pytest.fixture(autouse=True)
def wx_callback_errors():
    # wx reports callback failures through excepthook instead of raising into
    # pytest. Include those failures, especially during deferred destruction.
    with mock.patch('sys.excepthook') as errors:
        yield
        errors.assert_not_called()


@pytest.fixture(scope='module')
def gui():
    wx = pytest.importorskip('wx')
    from MAVProxy.modules.lib import wx_util
    with mock.patch.object(wx_util, 'safe', True):
        from MAVProxy.modules.mavproxy_misseditor import missionEditorFrame as frame
        from MAVProxy.modules.mavproxy_misseditor.survey_dialog import SurveyDialog
    app = wx.App.Get() or wx.App(False)
    yield SimpleNamespace(wx=wx, app=app, frame=frame, dialog=SurveyDialog)


def mission():
    loader = mavwp.MAVWPLoader()
    # Grid rows 0..5 have mission sequences 1..6; jump targets include each
    # boundary around an insertion after sequence 1, plus immutable home.
    for seq, (cmd, target) in enumerate([(16, 0), (16, 0), (16, 0),
                                        (177, 0), (177, 1), (177, 2), (177, 6)]):
        loader.add(mavutil.mavlink.MAVLink_mission_item_message(
            1, 1, seq, 0 if seq == 0 else 3, cmd, 0, 1,
            target, 2 if cmd == 177 else 0, 0, 0,
            -35.363262 + seq * 0.001, 149.165238, 580 if seq == 0 else 120))
    return loader


@pytest.fixture
def editor(gui):
    terrain = mock.Mock()
    terrain.GetElevation.return_value = 600
    with mock.patch.object(gui.frame.mp_elevation, 'ElevationModel', return_value=terrain):
        editor = gui.frame.MissionEditorFrame(SimpleNamespace(object_queue=queue.Queue()), parent=None)
    editor.set_event_queue(queue.Queue())
    editor.set_event_queue_lock(threading.Lock())
    editor.set_gui_event_queue(queue.Queue())
    editor.set_gui_event_queue_lock(threading.Lock())
    editor.text_ctrl_wp_default_alt.SetValue('100')
    editor.load_wploader(mission())
    editor.grid_mission.SetGridCursor(0, 0)
    yield editor
    if editor:
        editor.timer.Stop()
        editor.Destroy()
    gui.app.Yield()


def dialog_for(gui, editor):
    row = editor.grid_mission.GetGridCursorRow()
    return gui.dialog(editor, row, editor.survey_origin(),
                      editor.grid_mission.GetCellValue(row, 8),
                      editor.grid_mission.GetCellValue(row, 7))


def wait_for(gui, condition, timeout=3):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'timed out waiting for wx event'
        gui.app.Yield()
        time.sleep(0.01)


def preview_points(editor):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    result = None
    while not editor.event_queue.empty():
        event = editor.event_queue.get_nowait()
        if isinstance(event, me_event.MissionEditorEvent) and event.type == me_event.MEE_SURVEY_PREVIEW:
            result = event.get_arg('points')
    return result


def set_control(gui, control, value):
    control.SetValue(value)
    if isinstance(control, gui.wx.Slider):
        event = gui.wx.CommandEvent(gui.wx.EVT_SLIDER.typeId, control.GetId())
    elif isinstance(control, gui.wx.SpinCtrl):
        event = gui.wx.SpinEvent(gui.wx.EVT_SPINCTRL.typeId, control.GetId())
    else:
        event = gui.wx.SpinDoubleEvent(gui.wx.EVT_SPINCTRLDOUBLE.typeId, control.GetId())
    event.SetEventObject(control)
    control.GetEventHandler().ProcessEvent(event)


def last_map_mission(editor):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    events = [event for event in list(editor.event_queue.queue)
              if isinstance(event, me_event.MissionEditorEvent) and event.type == me_event.MEE_MAP_MISSION]
    assert events
    return events[-1].get_arg('wploader')


@pytest.mark.parametrize('command,lat,lon,enabled', [
    ('NAV_WAYPOINT', '-35', '149', True), ('NAV_WAYPOINT', '0', '149', True),
    ('NAV_WAYPOINT', '-35', '0', True), ('NAV_WAYPOINT', '0', '0', False),
    ('NAV_WAYPOINT', 'nan', '149', False), ('NAV_WAYPOINT', '-35', 'inf', False),
    ('NAV_WAYPOINT', '', '149', False), ('NAV_WAYPOINT', '91', '149', False),
    ('DO_JUMP', '-35', '149', False), ('INVALID', '-35', '149', False),
])
def test_button_gate(gui, editor, command, lat, lon, enabled):
    grid = editor.grid_mission
    grid.SetCellValue(0, 0, command)
    grid.SetCellValue(0, 5, lat)
    grid.SetCellValue(0, 6, lon)
    event = gui.wx.UpdateUIEvent(editor.button_survey.GetId())
    editor.button_survey.GetEventHandler().ProcessEvent(event)
    assert event.GetEnabled() == enabled
    assert (editor.survey_origin() is not None) == enabled
    grid.DeleteRows(0, grid.GetNumberRows())
    assert editor.survey_origin() is None


@pytest.mark.parametrize('grid_frame,altitude,name,agl', [
    ('Rel', '120', 'AboveHome', 100), ('AGL', '120', 'AGL', 120),
    ('Abs', '700', 'AMSL', 100),
])
def test_defaults_live_edits_and_frame(gui, editor, grid_frame, altitude, name, agl):
    editor.grid_mission.SetCellValue(0, 8, grid_frame)
    editor.grid_mission.SetCellValue(0, 7, altitude)
    dialog = dialog_for(gui, editor)
    try:
        assert dialog.controls['height'].GetValue() == float(altitude)
        assert dialog.frame_choice.GetStringSelection() == name
        assert dialog.controls['length'].GetValue() == 500
        assert dialog.result.height_agl == agl
        initial = preview_points(editor)
        set_control(gui, dialog.controls['rotation'], 90)
        wait_for(gui, lambda: dialog.result is not None)
        assert preview_points(editor) != initial
        set_control(gui, dialog.controls['height'], -10000)
        wait_for(gui, lambda: dialog.due is None)
        assert not dialog.write_button.IsEnabled()
        assert preview_points(editor) == []
        assert 'above ground' in dialog.status.GetLabel()
        set_control(gui, dialog.controls['height'], int(altitude))
        set_control(gui, dialog.controls['overlap'], 50)
        wait_for(gui, lambda: dialog.result is not None)
        assert dialog.write_button.IsEnabled()
    finally:
        dialog.Destroy()
    assert preview_points(editor) == []


def test_spin_controls_and_full_rotation_range_update_preview(gui, editor):
    dialog = dialog_for(gui, editor)
    try:
        for key in ('length', 'breadth', 'height'):
            assert isinstance(dialog.controls[key], gui.wx.SpinCtrl)
            assert isinstance(dialog.controls[key].GetValue(), int)
        for key in ('fov', 'overlap'):
            assert isinstance(dialog.controls[key], gui.wx.SpinCtrlDouble)
        rotation = dialog.controls['rotation']
        assert isinstance(rotation, gui.wx.Slider)
        assert (rotation.GetMin(), rotation.GetMax()) == (-180, 180)
        for value in (-180, 0, 180):
            set_control(gui, rotation, value)
            wait_for(gui, lambda: dialog.result is not None)
            from MAVProxy.modules.lib import mp_util
            bearing = mp_util.gps_bearing(*dialog.result.points[0], *dialog.result.points[1])
            assert abs(mp_util.wrap_180(bearing - value)) < 0.01
        for key, value in (('length', 601), ('breadth', 701),
                           ('height', 150), ('fov', 75.5), ('overlap', 60.5)):
            previous = preview_points(editor)
            set_control(gui, dialog.controls[key], value)
            wait_for(gui, lambda: dialog.result is not None)
            assert dialog.controls[key].GetValue() == value
            assert preview_points(editor) != previous
    finally:
        dialog.Destroy()


@pytest.mark.parametrize('grid_frame,altitude', [('Rel', '120'), ('AGL', '120'), ('Abs', '700')])
def test_modal_write_inserts_and_preserves_jumps(gui, editor, grid_frame, altitude):
    grid = editor.grid_mission
    grid.SetCellValue(0, 8, grid_frame)
    grid.SetCellValue(0, 7, altitude)
    before = editor.survey_snapshot()
    dialog = dialog_for(gui, editor)
    points = dialog.result.points
    count = len(points)
    gui.wx.CallAfter(dialog.on_write, None)
    try:
        assert dialog.ShowModal() == gui.wx.ID_OK
    finally:
        dialog.Destroy()
    assert grid.GetNumberRows() == len(before[0]) + count
    assert editor.survey_snapshot()[1:] == before[1:]
    assert tuple(grid.GetCellValue(0, col) for col in range(9)) == before[0][0]
    for i, point in enumerate(points, 1):
        assert grid.GetCellValue(i, 0) == 'NAV_WAYPOINT'
        assert all(float(grid.GetCellValue(i, col)) == 0 for col in range(1, 5))
        assert (float(grid.GetCellValue(i, 5)), float(grid.GetCellValue(i, 6))) == pytest.approx(point)
        assert float(grid.GetCellValue(i, 7)) == float(altitude)
        assert grid.GetCellValue(i, 8) == grid_frame
    assert tuple(grid.GetCellValue(count + 1, col) for col in range(9)) == before[0][1]
    assert [float(grid.GetCellValue(count + i, 1)) for i in range(2, 6)] == [0, 1, 2 + count, 6 + count]
    assert editor.label_sync_state.GetLabel() == 'MODIFIED'
    draft = last_map_mission(editor)
    assert draft.count() == grid.GetNumberRows() + 1
    assert draft.wp(1).z == float(altitude)
    assert draft.wp(2).frame == {'Rel': 3, 'AGL': 10, 'Abs': 0}[grid_frame]
    assert preview_points(editor) == []


def test_terrain_retry_and_agl_without_terrain(gui, editor):
    editor.ElevationModel.GetElevation.return_value = None
    dialog = dialog_for(gui, editor)
    try:
        assert dialog.result is None
        assert not dialog.write_button.IsEnabled()
        editor.ElevationModel.GetElevation.return_value = 600
        wait_for(gui, lambda: dialog.result is not None)
        dialog.frame_choice.SetStringSelection('AGL')
        editor.ElevationModel.GetElevation.return_value = None
        dialog.update_preview()
        assert dialog.result.height_agl == 120
    finally:
        dialog.Destroy()


def test_above_home_requires_received_home(gui, editor):
    editor.home_received = False
    # A plausible default label must not masquerade as a received home.
    editor.label_home_alt_value.SetLabel('0.0')
    editor.ElevationModel.GetElevation.return_value = 0
    dialog = dialog_for(gui, editor)
    try:
        assert not dialog.write_button.IsEnabled()
        assert 'Home altitude is unavailable' in dialog.status.GetLabel()
        dialog.frame_choice.SetStringSelection('AGL')
        dialog.update_preview()
        assert dialog.result.height_agl == 120
    finally:
        dialog.Destroy()


def test_unsupported_frame_requires_explicit_choice(gui, editor):
    editor.grid_mission.SetCellValue(0, 8, '')
    dialog = dialog_for(gui, editor)
    try:
        assert dialog.frame_choice.GetSelection() == gui.wx.NOT_FOUND
        assert not dialog.write_button.IsEnabled()
        assert 'unsupported' in dialog.status.GetLabel()
        dialog.frame_choice.SetStringSelection('AGL')
        dialog.update_preview()
        assert dialog.write_button.IsEnabled()
    finally:
        dialog.Destroy()


def test_invalid_jump_cannot_partially_insert_survey(gui, editor):
    editor.grid_mission.SetCellValue(2, 1, 'bad')
    before = editor.survey_snapshot()
    dialog = dialog_for(gui, editor)
    try:
        dialog.on_write(None)
        assert editor.survey_snapshot() == before
        assert not dialog.write_button.IsEnabled()
        assert 'jump target' in dialog.status.GetLabel()
    finally:
        dialog.Destroy()


def test_incoming_mission_invalidates_dialog(gui, editor):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    dialog = dialog_for(gui, editor)
    try:
        assert preview_points(editor)
        editor.gui_event_queue.put(me_event.MissionEditorEvent(me_event.MEGE_CLEAR_MISS_TABLE))
        wait_for(gui, lambda: 'Mission changed' in dialog.status.GetLabel())
        assert dialog.result is None
        assert not dialog.write_button.IsEnabled()
        assert preview_points(editor) == []
        dialog.on_write(None)
        assert editor.grid_mission.GetNumberRows() == 0
    finally:
        dialog.Destroy()


@pytest.mark.parametrize('close', [False, True])
def test_cancel_and_window_close_leave_mission_unchanged(gui, editor, close):
    before = editor.survey_snapshot()
    dialog = dialog_for(gui, editor)
    gui.wx.CallAfter(dialog.Close if close else lambda: dialog.EndModal(gui.wx.ID_CANCEL))
    try:
        assert dialog.ShowModal() == gui.wx.ID_CANCEL
    finally:
        dialog.Destroy()
    assert editor.survey_snapshot() == before
    assert preview_points(editor) == []


def test_late_destroy_events_do_not_access_deleted_dialog(gui, editor):
    dialog = dialog_for(gui, editor)
    window_id = dialog.GetId()
    child_id = dialog.controls['length'].GetId()
    dialog.Destroy()
    gui.app.Yield()
    assert not dialog  # the native wx window has been deleted
    assert preview_points(editor) == []

    # Child notifications can propagate after native dialog destruction.
    # An own-window notification may also arrive after explicit cleanup.
    for event_id in (child_id, window_id):
        event = mock.Mock()
        event.GetId.return_value = event_id
        dialog.on_destroy(event)
        event.Skip.assert_called_once()
    assert preview_points(editor) is None  # cleanup remains idempotent


def test_parent_destroy_stops_survey_timer_and_clears_preview(gui, editor):
    dialog = dialog_for(gui, editor)
    assert preview_points(editor)
    editor.timer.Stop()
    editor.Destroy()  # native parent teardown bypasses SurveyDialog.Destroy
    gui.app.Yield()
    assert not dialog
    assert dialog.closed
    assert not dialog.timer.IsRunning()
    assert preview_points(editor) == []


def test_read_only_viewer_hides_survey(gui):
    with mock.patch.object(gui.frame.mp_elevation, 'ElevationModel') as terrain:
        terrain.return_value.GetElevation.return_value = 600
        viewer = gui.frame.MissionEditorFrame(None, parent=None, read_only=True, wploader=mission())
    try:
        assert not viewer.button_survey.IsShown()
        assert viewer.survey_origin() is None
    finally:
        viewer.timer.Stop()
        viewer.Destroy()
        gui.app.Yield()


@pytest.mark.parametrize('action', ['edit', 'delete', 'add', 'up', 'down', 'split'])
def test_grid_edits_publish_modified_map_mission(gui, editor, action):
    grid = editor.grid_mission
    event = mock.Mock()
    if action == 'edit':
        grid.SetCellValue(0, 5, '-35.5')
        editor.on_mission_grid_cell_changed(event)
    elif action == 'add':
        editor.add_wp_below_pushed(event)
    elif action == 'split':
        grid.SetCellValue(2, 0, 'NAV_WAYPOINT')
        grid.SetGridCursor(2, 0)
        editor.split_pushed(event)
    else:
        event.GetRow.return_value = 1
        event.GetCol.return_value = {'delete': 9, 'up': 10, 'down': 11}[action]
        with mock.patch.object(gui.wx, 'MessageDialog') as confirm:
            confirm.return_value.ShowModal.return_value = gui.wx.ID_YES
            editor.on_mission_grid_cell_left_click(event)
    assert editor.label_sync_state.GetLabel() == 'MODIFIED'
    draft = last_map_mission(editor)
    assert draft.count() == grid.GetNumberRows() + 1
    for row in range(grid.GetNumberRows()):
        item = draft.wp(row + 1)
        assert item.seq == row + 1
        assert item.x == float(grid.GetCellValue(row, 5))
        assert item.param1 == float(grid.GetCellValue(row, 1))
    from MAVProxy.modules.mavproxy_misseditor import me_event
    assert all(event.type not in (me_event.MEE_WRITE_WPS, me_event.MEE_WRITE_WP_NUM)
               for event in list(editor.event_queue.queue))


@pytest.mark.parametrize('ftp', [True, False])
@pytest.mark.parametrize('edit_during_read', [True, False])
def test_read_restores_original_mission_unless_edited_during_download(gui, editor, ftp, edit_during_read):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    original = editor.survey_snapshot()
    editor.grid_mission.DeleteRows(1)
    editor.set_modified_state(True)
    editor.checkbox_mavftp.SetValue(ftp)
    editor.read_wp_pushed(mock.Mock())
    assert editor.label_sync_state.GetLabel() == 'MODIFIED'
    assert last_map_mission(editor) is not None
    if edit_during_read:
        editor.grid_mission.SetCellValue(0, 7, '123')
        editor.set_modified_state(True)
    editor.process_gui_event(me_event.MissionEditorEvent(
        me_event.MEGE_FTP_MISSION if ftp else me_event.MEGE_READ_MISSION, wploader=mission()))
    if edit_during_read:
        assert editor.label_sync_state.GetLabel() == 'MODIFIED'
        assert last_map_mission(editor).wp(1).z == 123
    else:
        assert editor.survey_snapshot() == original
        assert editor.label_sync_state.GetLabel() == 'SYNCED'
        assert last_map_mission(editor) is None


def test_incoming_mission_can_publish_map_without_holding_incoming_lock(gui, editor):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    editor.gui_event_queue.put(me_event.MissionEditorEvent(me_event.MEGE_LOAD_MISSION, wploader=mission()))
    handler = editor.process_gui_event
    lock_was_free = []

    def process(event):
        acquired = editor.gui_event_queue_lock.acquire(blocking=False)
        lock_was_free.append(acquired)
        if acquired:
            editor.gui_event_queue_lock.release()
        handler(event)

    with mock.patch.object(editor, 'process_gui_event', side_effect=process):
        editor.time_to_process_gui_events(None)
    assert lock_was_free and all(lock_was_free)
    assert last_map_mission(editor) is None


@pytest.mark.parametrize('ftp', [True, False])
def test_incomplete_edit_survives_read_and_automatic_loader_refresh(gui, editor, ftp):
    from MAVProxy.modules.mavproxy_misseditor import me_event
    editor.checkbox_mavftp.SetValue(ftp)
    editor.read_wp_pushed(mock.Mock())
    editor.grid_mission.SetCellValue(0, 5, '')
    editor.on_mission_grid_cell_changed(mock.Mock())
    assert last_map_mission(editor) is None  # invalid draft was not published
    for kind in (me_event.MEGE_FTP_MISSION if ftp else me_event.MEGE_READ_MISSION,
                 me_event.MEGE_LOAD_MISSION):
        editor.process_gui_event(me_event.MissionEditorEvent(kind, wploader=mission()))
        assert editor.grid_mission.GetCellValue(0, 5) == ''
        assert editor.mission_modified
        assert editor.label_sync_state.GetLabel() == 'MODIFIED'
    # A subsequent explicit Read still replaces the incomplete edit.
    editor.read_wp_pushed(mock.Mock())
    editor.process_gui_event(me_event.MissionEditorEvent(
        me_event.MEGE_FTP_MISSION if ftp else me_event.MEGE_READ_MISSION, wploader=mission()))
    assert not editor.mission_modified
    assert float(editor.grid_mission.GetCellValue(0, 5)) == mission().wp(1).x


@pytest.mark.parametrize('height', ['', 'bad', 'nan', 'inf', '-inf', '100001', '-10001'])
def test_survey_button_rejects_invalid_altitude_without_opening_dialog(gui, editor, height):
    editor.grid_mission.SetCellValue(0, 7, height)
    with mock.patch.object(gui.dialog, 'ShowModal') as show:
        editor.survey_pushed(mock.Mock())
    show.assert_not_called()
    assert 'waypoint altitude' in editor.GetStatusBar().GetStatusText()


def test_fractional_starting_height_is_explicitly_rounded(gui, editor):
    editor.grid_mission.SetCellValue(0, 7, '120.75')
    dialog = dialog_for(gui, editor)
    try:
        assert dialog.controls['height'].GetValue() == 121
        assert 'rounded' in dialog.controls['height'].GetToolTipText()
    finally:
        dialog.Destroy()


def test_unknown_home_does_not_publish_a_fabricated_home(gui, editor):
    editor.home_received = False
    editor.set_modified_state(True)
    assert last_map_mission(editor) is None
    assert editor.mission_modified
