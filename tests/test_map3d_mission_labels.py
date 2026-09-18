'''the 3D map labelling each mission item, as the 2D map does'''

import pytest

from pymavlink import mavutil

mavlink = mavutil.mavlink

HOME = (-35.363262, 149.165238, 584.0)


def items():
    '''home, a waypoint, a loiter and a land start, as the map is given them'''
    from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
    return [
        MissionItem(HOME[0], HOME[1], HOME[2], 0,
                    mavlink.MAV_CMD_NAV_WAYPOINT, 0),
        MissionItem(HOME[0] + 0.01, HOME[1], HOME[2] + 100, 0,
                    mavlink.MAV_CMD_NAV_LOITER_TURNS, 1),
        MissionItem(HOME[0] + 0.02, HOME[1] + 0.01, HOME[2] + 100, 0,
                    mavlink.MAV_CMD_DO_LAND_START, 2),
    ]


LABELS = ['0', '1(LT)', '2(DLS)']


def elements():
    pytest.importorskip("vtk")
    import vtk
    from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
    em = ElementManager(vtk.vtkRenderer(), HOME[0], HOME[1], 1.0)
    em.set_home(HOME[2])
    return em


def labels(em):
    '''the text and position of each number drawn'''
    import vtk
    return [(a.GetInput(), a.GetPosition())
            for a in em.actors.get('mission', [])
            if isinstance(a, vtk.vtkBillboardTextActor3D)]


def label_sizes(em):
    '''how big each of those is drawn'''
    import vtk
    return [a.GetTextProperty().GetFontSize()
            for a in em.actors.get('mission', [])
            if isinstance(a, vtk.vtkBillboardTextActor3D)]


def recording_map(pushed):
    '''a map which notes the label settings it is sent'''
    from types import SimpleNamespace
    return SimpleNamespace(
        is_alive=lambda: True, set_fpv_fov=lambda fov: None,
        set_mission_arrows=lambda enable: None,
        set_mission_labels=lambda enable: pushed.append(('labels', enable)),
        set_mission_label_size=lambda size: pushed.append(('size', size)),
        set_mission_style=lambda style: None,
        set_render_settings=lambda *args: None)


class TestMissionLabels(object):

    def test_the_labels_are_drawn_only_when_they_are_asked_for(self):
        em = elements()
        em.set_mission(items())
        assert labels(em) == []
        em.set_mission_labels(True)
        drawn = labels(em)
        assert [text for (text, _) in drawn] == LABELS
        # each beside the item it numbers
        for ((_, at), marker) in zip(drawn, em.mission_markers):
            assert at == pytest.approx(marker)
        # and they go again when they are not wanted
        em.set_mission_labels(False)
        assert labels(em) == []

    def test_the_labels_are_drawn_the_size_they_are_asked_for(self):
        from MAVProxy.modules.mavproxy_map3d.map3d import (
            MISSION_LABEL_SIZE, MISSION_LABEL_SIZES)
        em = elements()
        em.set_mission(items())
        em.set_mission_labels(True)
        assert label_sizes(em) == [MISSION_LABEL_SIZE] * len(LABELS)
        em.set_mission_label_size(24)
        assert label_sizes(em) == [24] * len(LABELS)
        # and only to a size there is some reading
        (smallest, largest) = MISSION_LABEL_SIZES
        em.set_mission_label_size(smallest - 1)
        assert label_sizes(em) == [smallest] * len(LABELS)
        em.set_mission_label_size(largest + 1)
        assert label_sizes(em) == [largest] * len(LABELS)
        # a size set while they are not drawn is the size they come back at
        em.set_mission_labels(False)
        em.set_mission_label_size(18)
        em.set_mission_labels(True)
        assert label_sizes(em) == [18] * len(LABELS)

    def test_a_mission_drawn_again_keeps_its_labels(self):
        em = elements()
        em.set_mission_labels(True)
        em.set_mission(items())
        assert [text for (text, _) in labels(em)] == LABELS
        # an item with no position of its own is not drawn, so it is not
        # labelled either, and the numbers stay those of the items
        from MAVProxy.modules.mavproxy_map3d.map3d import MissionItem
        mission = items()
        mission[1] = MissionItem(0, 0, 0, 3, mavlink.MAV_CMD_DO_CHANGE_SPEED, 1)
        em.set_mission(mission)
        assert [text for (text, _) in labels(em)] == ['0', '2(DLS)']

    def test_the_labels_are_the_ones_the_2d_map_uses(self):
        """both maps call an item the same thing"""
        from MAVProxy.modules.lib import mp_util
        assert mp_util.mission_item_label(
            3, mavlink.MAV_CMD_NAV_WAYPOINT) == '3'
        for (command, suffix) in (
                (mavlink.MAV_CMD_DO_LAND_START, 'DLS'),
                (mavlink.MAV_CMD_NAV_TAKEOFF, 'TOff'),
                (mavlink.MAV_CMD_NAV_LOITER_TIME, 'LTime'),
                (mavlink.MAV_CMD_NAV_VTOL_LAND, 'VL'),
                (mp_util.MAV_CMD_NAV_ARC_WAYPOINT, 'AW'),
                (mp_util.MAV_CMD_DO_ORBIT, 'Orbit')):
            assert mp_util.mission_item_label(
                4, command) == '4(%s)' % suffix
        # and the 2D map's own labelling is the same labelling
        pytest.importorskip("wx")
        from types import SimpleNamespace
        from MAVProxy.modules import mavproxy_map
        module = mavproxy_map.MapModule.__new__(mavproxy_map.MapModule)
        wp = SimpleNamespace(command=mavlink.MAV_CMD_DO_LAND_START)
        module.mpstate = SimpleNamespace(
            module=lambda name: SimpleNamespace(
                wploader=SimpleNamespace(wp=lambda i: wp)))
        assert module.label_for_waypoint(2) == '2(DLS)'

    def test_the_viewer_is_told_to_label_the_mission(self, map3d_frame):
        pytest.importorskip("vtk")
        pytest.importorskip("wx")
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d.map3d import Map3D
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
        sent = []
        viewer = Map3D.__new__(Map3D)
        viewer.child = SimpleNamespace(is_alive=lambda: True)
        viewer.object_queue = SimpleNamespace(put=sent.append)
        em = elements()
        frame = map3d_frame(em)
        viewer.set_mission(items())
        viewer.set_mission_labels(True)
        viewer.set_mission_label_size(22)
        for msg in sent:
            Map3DFrame.handle(frame, msg)
        assert [text for (text, _) in labels(em)] == LABELS
        assert label_sizes(em) == [22] * len(LABELS)
        # and the checkbox shows what the map is doing
        assert frame.labels_check.GetValue() is True

    def test_the_live_map_settings(self, map3d_module):
        from MAVProxy.modules.mavproxy_map3d.map3d import (
            MISSION_LABEL_SIZE, MISSION_LABEL_SIZES)
        pushed = []
        module = map3d_module(recording_map(pushed))
        # off unless it is asked for, since a mission of any size is busy
        assert module.map3d_settings.showlabels is False
        assert module.map3d_settings.labelsize == MISSION_LABEL_SIZE
        module.cmd_map3d(['set', 'showlabels', 'true'])
        assert ('labels', True) in pushed
        assert module.map3d_settings.showlabels is True
        module.cmd_map3d(['set', 'labelsize', '20'])
        assert ('size', 20) in pushed
        assert module.map3d_settings.labelsize == 20
        # and a size there is no reading is not taken
        module.cmd_map3d(['set', 'labelsize',
                          str(MISSION_LABEL_SIZES[1] + 1)])
        assert module.map3d_settings.labelsize == 20

    def test_the_checkbox_on_the_map_labels_the_mission(self, map3d_frame,
                                                        map3d_module):
        em = elements()
        em.set_mission(items())
        events = []
        frame = map3d_frame(em, events)
        frame.labels_check.SetValue(True)
        frame.on_labels_toggle(None)
        assert [text for (text, _) in labels(em)] == LABELS
        # and the setting follows the box, so it is what a view opened next
        # is drawn with
        assert events == [('mission_labels', True)]
        pushed = []
        module = map3d_module(recording_map(pushed))
        module.map.check_events = lambda: events
        module.idle_task()
        assert module.map3d_settings.showlabels is True
        # and the view is told what the setting now is
        assert pushed == [('labels', True)]

    def test_a_map_with_no_scene_yet_keeps_what_it_is_told(self, map3d_frame):
        '''the live map is started before the vehicle has said where it is,
        so what it is told to draw waits for a scene to draw it in'''
        # the fixture skips without wx, which the frame's module needs
        frame = map3d_frame()
        from MAVProxy.modules.mavproxy_map3d.map3d_ui import Map3DFrame
        Map3DFrame.handle(frame, ('mission_labels', True))
        Map3DFrame.handle(frame, ('mission_label_size', 20))
        Map3DFrame.handle(frame, ('mission_style', 'plain'))
        Map3DFrame.handle(frame, ('mission_arrows', True))
        assert frame.labels_check.GetValue() is True
        assert frame.style_choice.GetStringSelection() == 'plain'
        # which is what the scene is built with, once there is one
        Map3DFrame.handle(frame, ('origin', HOME[0], HOME[1], HOME[2]))
        Map3DFrame.handle(frame, ('mission', items(), None))
        em = frame.elements
        assert label_sizes(em) == [20] * len(LABELS)
        assert em.mission_style == 'plain'
        assert em.mission_arrows is True

    def test_mavexplorer_labels_a_view_it_opens(self, monkeypatch, mavexplorer):
        mx = mavexplorer
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import map3d
        labelled = []

        class Viewer(object):
            def __init__(self, title=None):
                pass

            def is_alive(self):
                return True

            def set_mission_labels(self, enable):
                labelled.append(enable)

            def __getattr__(self, name):
                return lambda *args, **kwargs: None
        monkeypatch.setattr(map3d, 'Map3D', Viewer)
        monkeypatch.setattr(map3d, 'missing_packages', lambda: [])

        def message(kind, **fields):
            m = SimpleNamespace(_timestamp=0, **fields)
            m.get_type = lambda: kind
            return m
        rows = [message('POS', Lat=HOME[0], Lng=HOME[1], Alt=HOME[2]),
                message('POS', Lat=HOME[0] + 0.01, Lng=HOME[1],
                        Alt=HOME[2] + 50)]
        log = SimpleNamespace(rewind=lambda: None, mav_type=None)
        log.recv_match = lambda **kwargs: rows.pop(0) if rows else None
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(
            mlog=log, settings=SimpleNamespace(
                condition=None, showdirection=True, showlabels=True,
                labelsize=20, sync_xmap=False, missionpath='geometry')),
            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [])
        mx.cmd_map3d([])
        assert labelled == [True]

    def test_mavexplorer_labels_a_mission_it_draws(self, monkeypatch, mavexplorer):
        from types import SimpleNamespace
        mx = mavexplorer
        labelled = []
        sizes = []
        view = SimpleNamespace(
            is_alive=lambda: True,
            set_mission_arrows=lambda enable: None,
            set_mission_labels=labelled.append,
            set_mission_label_size=sizes.append,
            set_mission_style=lambda style: None,
            mission_to_fly=None)
        settings = SimpleNamespace(showdirection=True, showlabels=True,
                                   labelsize=20, missionpath='geometry',
                                   command=lambda args: None)
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(settings=settings),
                            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [view])
        mx.cmd_set(['showlabels', 'true'])
        assert labelled == [True]
        assert sizes == [20]

    def test_mavexplorer_takes_the_checkbox_on_a_view(self, monkeypatch, mavexplorer):
        '''a view labelled from its own checkbox sets the setting, so every
        view is labelled and one opened later is too'''
        from types import SimpleNamespace
        mx = mavexplorer
        labelled = []
        other = SimpleNamespace(
            is_alive=lambda: True, check_events=lambda: [],
            set_mission_arrows=lambda enable: None,
            set_mission_labels=labelled.append,
            set_mission_label_size=lambda size: None,
            set_mission_style=lambda style: None, mission_to_fly=None)
        view = SimpleNamespace(
            is_alive=lambda: True,
            check_events=lambda: [('mission_labels', True)],
            set_mission_arrows=lambda enable: None,
            set_mission_labels=lambda enable: None,
            set_mission_label_size=lambda size: None,
            set_mission_style=lambda style: None, mission_to_fly=None)
        settings = SimpleNamespace(showdirection=True, showlabels=False,
                                   labelsize=14, missionpath='geometry')
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(settings=settings),
                            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [view, other])
        mx.poll_map3d_views()
        assert settings.showlabels is True
        assert labelled == [True]
