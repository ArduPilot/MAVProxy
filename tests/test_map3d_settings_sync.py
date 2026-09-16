'''the 3D map's own controls and the settings they stand for.  The view runs
in a process of its own, so what it is told and what it tells come and go
on two queues; however those cross, the view and whatever started it --
the map3d module, or MAVExplorer -- end up agreeing'''

import os
import queue
import threading
from types import SimpleNamespace

import pytest

HOME = (-35.363262, 149.165238, 584.0)


def connect(map3d_frame):
    '''a Map3D with plain queues, and the frame at the other end of them'''
    pytest.importorskip("vtk")
    import vtk
    from MAVProxy.modules.mavproxy_map3d.map3d import Map3D
    from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
    viewer = Map3D.__new__(Map3D)
    viewer.child = SimpleNamespace(is_alive=lambda: True)
    viewer.object_queue = queue.Queue()
    viewer.event_queue = queue.Queue()
    viewer._origin_set = True
    em = ElementManager(vtk.vtkRenderer(), HOME[0], HOME[1], 1.0)
    em.set_home(HOME[2])
    return (viewer, map3d_frame(em, state=viewer))


def deliver(viewer, frame):
    '''the view takes everything it has been sent'''
    from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
    while True:
        try:
            msg = viewer.object_queue.get_nowait()
        except queue.Empty:
            return
        Map3DFrame.handle(frame, msg)


def labels_on_the_view(frame):
    frame.labels_check.SetValue(True)
    frame.on_labels_toggle(None)


def style_on_the_view(frame):
    frame.style_choice.SetStringSelection('plain')
    frame.on_style_choice(None)


def follow_on_the_view(frame):
    frame.follow_button.SetValue(False)
    frame.on_follow_toggle(None)


# for each control on the view: what the user does there, the command which
# says otherwise, how the module and the view each hold the result, and what
# the command sets
CONTROLS = {
    'labels': (labels_on_the_view, ['set', 'showlabels', 'false'],
               lambda module: module.map3d_settings.showlabels,
               lambda frame: [frame.mission_labelled,
                              frame.labels_check.GetValue(),
                              frame.elements.mission_labelled],
               lambda held: held is False),
    'style': (style_on_the_view, ['set', 'missionpath', 'geometry'],
              lambda module: module.map3d_settings.missionpath,
              lambda frame: [frame.mission_style,
                             frame.style_choice.GetStringSelection(),
                             frame.elements.mission_style],
              lambda held: held == 'geometry'),
    'follow': (follow_on_the_view, ['follow'],
               lambda module: module.follow,
               lambda frame: [frame.follow, frame.follow_button.GetValue()],
               lambda held: held is True),
}


class TestSettingsSync(object):

    @pytest.mark.parametrize('control', sorted(CONTROLS))
    def test_a_command_given_while_the_view_is_changed(self, control,
                                                       map3d_frame,
                                                       map3d_module):
        '''the user changes a control on the view, and before the module
        has heard of it, a command says otherwise'''
        (on_the_view, command, in_the_module, in_the_view, _) = CONTROLS[control]
        (viewer, frame) = connect(map3d_frame)
        module = map3d_module(viewer)
        on_the_view(frame)
        module.cmd_map3d(command)
        module.idle_task()
        deliver(viewer, frame)
        # nothing more is on its way either side
        module.idle_task()
        assert viewer.event_queue.empty()
        deliver(viewer, frame)
        for held in in_the_view(frame):
            assert held == in_the_module(module)

    @pytest.mark.parametrize('control', sorted(CONTROLS))
    def test_a_command_given_after_the_view_is_changed(self, control,
                                                       map3d_frame,
                                                       map3d_module):
        '''the same, the other way round: the command comes second, and
        is what both end up with'''
        (on_the_view, command, in_the_module, in_the_view,
         commanded) = CONTROLS[control]
        (viewer, frame) = connect(map3d_frame)
        module = map3d_module(viewer)
        on_the_view(frame)
        module.idle_task()
        deliver(viewer, frame)
        module.cmd_map3d(command)
        deliver(viewer, frame)
        # it is the command which stands, not the control
        assert commanded(in_the_module(module))
        for held in in_the_view(frame):
            assert held == in_the_module(module)

    def test_mavexplorer_and_its_views_agree(self, map3d_frame, mavexplorer,
                                             monkeypatch):
        '''MAVExplorer has the same crossing, between a view's checkbox and
        its own set command'''
        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        settings = MPSettings([
            MPSetting('showdirection', bool, True),
            MPSetting('showlabels', bool, False),
            MPSetting('labelsize', int, 14),
            MPSetting('missionpath', str, 'flown',
                      choice=['flown', 'geometry', 'plain'])])
        monkeypatch.setattr(mavexplorer, 'mestate',
                            SimpleNamespace(settings=settings), raising=False)
        views = [connect(map3d_frame), connect(map3d_frame)]
        monkeypatch.setattr(mavexplorer, 'map3d_views',
                            [viewer for (viewer, _) in views])
        labels_on_the_view(views[0][1])
        style_on_the_view(views[0][1])
        mavexplorer.cmd_set(['showlabels', 'false'])
        mavexplorer.poll_map3d_views()
        for (viewer, frame) in views:
            deliver(viewer, frame)
        for (viewer, frame) in views:
            assert viewer.event_queue.empty()
            assert CONTROLS['labels'][3](frame) == [settings.showlabels] * 3
            assert CONTROLS['style'][3](frame) == [settings.missionpath] * 3
        # where the two crossed, what MAVExplorer heard last stands: the
        # view's, which it read after the command
        assert settings.showlabels is True


def display():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


@pytest.mark.skipif(not display(), reason='needs a display to build a window')
class TestToolbar(object):
    '''the frame as it is really built, which the other tests stand in for'''

    def frame(self):
        wx = pytest.importorskip("wx")
        pytest.importorskip("vtk")
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
        self.app = wx.GetApp() or wx.App(False)
        closed = threading.Semaphore()
        closed.acquire()
        state = SimpleNamespace(
            title='test', width=600, height=400, follow=True, fpvfov=90.0,
            terrain_brightness=1.25, terrain_shading=True,
            terrain_wireframe=False, service=None, zexag=1.0,
            object_queue=queue.Queue(), event_queue=queue.Queue(),
            close_window=closed)
        frame = Map3DFrame(state)
        frame.timer.Stop()
        return frame

    def click(self, control, kind):
        import wx
        event = wx.CommandEvent(kind, control.GetId())
        event.SetEventObject(control)
        control.GetEventHandler().ProcessEvent(event)

    def events(self, frame):
        out = []
        while not frame.state.event_queue.empty():
            out.append(frame.state.event_queue.get_nowait())
        return out

    def test_the_controls_start_as_the_view_draws(self):
        from MAVProxy.modules.mavproxy_map3d.map3d import MISSION_STYLES
        frame = self.frame()
        try:
            assert frame.labels_check.GetValue() is False
            assert frame.style_choice.GetStringSelection() == MISSION_STYLES[0]
            assert (list(frame.style_choice.GetStrings()) ==
                    list(MISSION_STYLES))
            assert frame.follow_button.GetValue() is True
        finally:
            frame.Destroy()

    def test_the_controls_tell_the_parent(self):
        import wx
        frame = self.frame()
        try:
            frame.labels_check.SetValue(True)
            self.click(frame.labels_check, wx.wxEVT_CHECKBOX)
            frame.style_choice.SetStringSelection('plain')
            self.click(frame.style_choice, wx.wxEVT_CHOICE)
            assert self.events(frame) == [('mission_labels', True),
                                          ('mission_style', 'plain')]
        finally:
            frame.Destroy()

    def test_the_controls_show_what_the_view_is_told(self):
        frame = self.frame()
        try:
            frame.handle(('mission_labels', True))
            frame.handle(('mission_style', 'geometry'))
            assert frame.labels_check.GetValue() is True
            assert frame.style_choice.GetStringSelection() == 'geometry'
            # and being told is not the user changing them
            assert self.events(frame) == []
        finally:
            frame.Destroy()
