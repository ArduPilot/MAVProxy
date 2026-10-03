'''Survey dimensions, camera footprint and the main-thread map overlay.'''

import math
import queue
import threading
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_misseditor import survey, me_event
from MAVProxy.modules.mavproxy_misseditor.survey_preview import SurveyPreview


HERE = (-35.363262, 149.165238)


def generate(**kwargs):
    values = dict(origin=HERE, length=500, breadth=500, height_agl=100,
                  fov=60, rotation=0, overlap=70)
    values.update(kwargs)
    return survey.generate_survey(**values)


@pytest.mark.parametrize('origin', [HERE, (75, 20), (0, 179.999), (0, -179.999)])
@pytest.mark.parametrize('rotation', [0, 37, 90, 180, 270, -90, 450])
def test_dimensions_rotation_and_alternating_lanes(origin, rotation):
    result = generate(origin=origin, rotation=rotation)
    points = result.points
    assert points[0] == pytest.approx(origin)
    for i in range(0, len(points), 2):
        a, b = points[i:i + 2]
        assert mp_util.gps_distance(*a, *b) == pytest.approx(500, abs=0.3)
        heading = rotation + (180 if (i // 2) % 2 else 0)
        assert abs(mp_util.wrap_180(mp_util.gps_bearing(*a, *b) - heading)) < 0.05
        assert -90 < a[0] < 90 and -180 <= a[1] <= 180
        if i:
            assert mp_util.gps_distance(*points[i - 1], *a) == pytest.approx(result.spacing, abs=0.3)
    # Near ends of first and last lane span the full breadth.
    near_end = points[-2] if (len(points) // 2 - 1) % 2 == 0 else points[-1]
    assert mp_util.gps_distance(*origin, *near_end) == pytest.approx(500, abs=0.05)
    assert abs(mp_util.wrap_180(mp_util.gps_bearing(*origin, *near_end) - rotation - 90)) < 0.05


def test_overlap_is_at_least_requested_and_edges_are_covered():
    result = generate()
    swath = 200 * math.tan(math.radians(30))
    assert 1 - result.spacing / swath >= 0.7
    assert result.spacing * (len(result.points) // 2 - 1) == pytest.approx(500)
    assert len(result.points) == 32
    assert len(generate(overlap=80).points) > len(result.points)
    assert len(generate(height_agl=200).points) < len(result.points)
    assert len(generate(breadth=1).points) == 4


@pytest.mark.parametrize('kwargs', [
    {'length': 0}, {'breadth': -1}, {'height_agl': 0}, {'fov': 0},
    {'fov': 180}, {'overlap': -1}, {'overlap': 100}, {'rotation': float('nan')},
    {'height_agl': float('inf')}, {'origin': (91, 10)}, {'origin': (10, 181)},
    {'origin': (89.99999, 20)}, {'length': 1e300}, {'fov': 1e-300},
    {'overlap': 99.999999}, {'height_agl': 1e308},
])
def test_invalid_or_excessive_surveys_are_rejected(kwargs):
    with pytest.raises(ValueError):
        generate(**kwargs)


def test_frame_native_altitude_and_derived_agl_are_distinct():
    assert survey.camera_height(120, 'AGL') == 120
    assert survey.camera_height(120, 'AboveHome', 580, 600) == 100
    assert survey.camera_height(700, 'AMSL', None, 600) == 100
    assert survey.camera_height(-10, 'AMSL', None, -110) == 100


@pytest.mark.parametrize('args', [
    (100, 'AboveHome', None, 500), (100, 'AboveHome', 500, None),
    (100, 'AMSL', None, 500), (0, 'AGL'), (float('nan'), 'AGL'),
    (100, 'bad'), (100, 'AMSL', None, float('nan')),
])
def test_missing_or_invalid_altitude_reference_is_rejected(args):
    with pytest.raises(ValueError):
        survey.camera_height(*args)


def test_preview_coalesces_handles_map_reload_and_clears():
    preview = SurveyPreview()
    a, b = mock.Mock(), mock.Mock()
    preview.set_points([(0, 1), (0, 2)])
    points = [(1, 1), (1, 2), (2, 2)]
    event = me_event.MissionEditorEvent(me_event.MEE_SURVEY_PREVIEW, points=points)
    preview.set_points(event.get_arg('points'))
    preview.draw([])
    preview.draw([a])
    obj = a.add_object.call_args.args[0]
    assert obj.points == tuple(points)
    preview.draw([a])
    assert a.add_object.call_count == 1
    preview.draw([b])
    a.remove_object.assert_called_once_with(preview.key)
    b.add_object.assert_called_once()
    preview.set_points([])
    preview.draw([b])
    b.remove_object.assert_called_once_with(preview.key)
    preview.set_points(points)
    preview.draw([b])
    preview.close()
    assert b.remove_object.call_count == 2
    preview.set_points(points)  # late event cannot restore an abandoned preview
    preview.draw([b])
    assert b.add_object.call_count == 2


def test_preview_draws_open_route_in_rgb_blue():
    preview = SurveyPreview()
    display = mock.Mock()
    preview.set_points([(10, 10), (10, 30), (30, 30)])
    preview.draw([display])
    polygon = display.add_object.call_args.args[0]
    pixels = np.zeros((45, 45, 3), dtype=np.uint8)
    polygon.draw(pixels, lambda point: tuple(int(v) for v in point), None)
    # Map tiles and wx.BitmapFromBuffer use RGB, without a channel swap.
    assert tuple(pixels[30, 20]) == (0, 0, 255)
    assert tuple(pixels[20, 20]) == (0, 0, 0)  # no closing diagonal


def test_event_thread_hands_preview_to_idle_and_child_exit_cleans_up():
    from MAVProxy.modules.mavproxy_misseditor import mission_editor
    editor = mission_editor.MissionEditorMain.__new__(mission_editor.MissionEditorMain)
    editor.survey_preview = SurveyPreview()
    editor.event_queue = queue.Queue()
    editor.event_queue_lock = threading.Lock()
    editor.child = mock.Mock()
    editor.child.is_alive.return_value = True
    editor.last_unload_check_time = 0
    editor.unload_check_interval = 0
    editor.last_wp_change = 0
    editor.close_window = mock.Mock()
    editor.mavlink_message_queue_handler = mock.Mock()
    editor.time_to_quit = False
    editor.map_mission = None
    display = mock.Mock()
    editor.mpstate = SimpleNamespace(
        public_modules={'map': SimpleNamespace(map=display)},
        module=lambda name: SimpleNamespace(loading_waypoint_lasttime=0))
    editor.event_thread = mission_editor.MissionEditorEventThread(
        editor, editor.event_queue, editor.event_queue_lock)
    points = [(1, 1), (1, 2)]
    editor.event_queue.put(me_event.MissionEditorEvent(me_event.MEE_SURVEY_PREVIEW, points=points))
    editor.event_queue.put(me_event.MissionEditorEvent(me_event.MEE_TIME_TO_QUIT))
    editor.event_thread.start()
    editor.event_thread.join(3)
    assert not editor.event_thread.is_alive()
    display.add_object.assert_not_called()
    editor.idle_task()
    assert display.add_object.call_args.args[0].points == tuple(points)
    editor.child.is_alive.return_value = False
    editor.idle_task()
    display.remove_object.assert_called_once_with(editor.survey_preview.key)
    assert editor.needs_unloading
