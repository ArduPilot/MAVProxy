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


def explorer():
    '''MAVExplorer, which MAVProxy/tools is not a package to import'''
    pytest.importorskip("wx")
    pytest.importorskip("lxml")
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                        'MAVProxy', 'tools', 'MAVExplorer.py')
    spec = importlib.util.spec_from_file_location('mavexplorer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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

    def test_the_viewer_is_told_to_label_the_mission(self):
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
        frame = SimpleNamespace(terrain=object(), elements=em)
        viewer.set_mission(items())
        viewer.set_mission_labels(True)
        for msg in sent:
            Map3DFrame.handle(frame, msg)
        assert [text for (text, _) in labels(em)] == LABELS

    def test_the_live_map_setting(self):
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import Map3DModule
        from MAVProxy.modules.lib import mp_settings
        module = Map3DModule.__new__(Map3DModule)
        labelled = []
        module.map3d_settings = mp_settings.MPSettings([
            ('fpvfov', float, 90.0), ('terrainbrightness', float, 1.25),
            ('terrainshading', bool, True), ('terrainwireframe', bool, False),
            ('showdirection', bool, True), ('showlabels', bool, False),
            ('missionpath', str, 'flown')])
        module.map = SimpleNamespace(
            is_alive=lambda: True, set_fpv_fov=lambda fov: None,
            set_mission_arrows=lambda enable: None,
            set_mission_labels=labelled.append,
            set_mission_style=lambda style: None,
            set_render_settings=lambda *args: None)
        module.send_mission = lambda: None
        # off unless it is asked for, since a mission of any size is busy
        assert module.map3d_settings.showlabels is False
        module.cmd_map3d(['set', 'showlabels', 'true'])
        assert labelled == [True]
        assert module.map3d_settings.showlabels is True

    def test_mavexplorer_labels_a_view_it_opens(self, monkeypatch):
        mx = explorer()
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
                sync_xmap=False, missionpath='geometry')), raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [])
        mx.cmd_map3d([])
        assert labelled == [True]

    def test_mavexplorer_labels_a_mission_it_draws(self, monkeypatch):
        from types import SimpleNamespace
        mx = explorer()
        labelled = []
        view = SimpleNamespace(
            is_alive=lambda: True,
            set_mission_arrows=lambda enable: None,
            set_mission_labels=labelled.append,
            set_mission_style=lambda style: None,
            mission_to_fly=None)
        settings = SimpleNamespace(showdirection=True, showlabels=True,
                                   missionpath='geometry',
                                   command=lambda args: None)
        monkeypatch.setattr(mx, 'mestate', SimpleNamespace(settings=settings),
                            raising=False)
        monkeypatch.setattr(mx, 'map3d_views', [view])
        mx.cmd_set(['showlabels', 'true'])
        assert labelled == [True]
