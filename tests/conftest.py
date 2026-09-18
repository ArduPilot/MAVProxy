'''shared test setup'''

import os

import pytest


@pytest.fixture(autouse=True)
def restore_mavlink20():
    """put MAVLINK20 back as it was after each test.  MAVExplorer.py sets it
    when it is loaded, and pymavlink's mission loader reads it to choose the
    messages it builds, but the dialect pymavlink has already loaded does not
    change with it: tests which run after one loading MAVExplorer.py would
    otherwise build messages the loaded dialect cannot take"""
    before = os.environ.get('MAVLINK20')
    yield
    if before is None:
        os.environ.pop('MAVLINK20', None)
    else:
        os.environ['MAVLINK20'] = before


class Widget(object):
    '''as much of a wx control as the 3D viewer's frame uses'''

    def __init__(self, value):
        self.value = value

    def SetValue(self, value):
        self.value = value

    def GetValue(self):
        return self.value

    def SetStringSelection(self, value):
        self.value = value

    def GetStringSelection(self):
        return self.value

    def Enable(self, enable=True):
        pass


class Terrain(object):
    '''as much of the 3D viewer's terrain manager as the frame uses, with
    nothing fetched'''
    mesh_revision = 0
    tiles = []

    def __init__(self, *args, **kwargs):
        self.render_settings = (kwargs.get('brightness'),
                                kwargs.get('shading'),
                                kwargs.get('wireframe'))

    def height_at(self, lat, lon):
        return 0.0

    def set_render_settings(self, brightness, shading, wireframe):
        self.render_settings = (brightness, shading, wireframe)

    def update(self, camera):
        pass


@pytest.fixture
def map3d_frame(monkeypatch):
    """build the 3D viewer's frame without wx: its real message handling,
    controls and scene building, with the widgets, the renderer's window and
    the terrain stood in for.  The camera dialog's tests build their wx
    object the same way.

    elements is the ElementManager to draw into, or None for a frame which
    has no scene yet.  state is what the frame was started with -- a Map3D,
    to connect it to one -- and otherwise events collects what the frame
    tells its parent"""
    def build(elements=None, events=None, state=None):
        pytest.importorskip("wx")
        pytest.importorskip("vtk")
        import vtk
        from types import SimpleNamespace
        from MAVProxy.modules.mavproxy_map3d import map3d_ui
        from MAVProxy.modules.mavproxy_map3d.map3d import (
            MISSION_LABEL_SIZE, MISSION_STYLES)
        # a scene is built with the real element manager and camera, over
        # terrain and imagery which are never fetched
        monkeypatch.setattr(map3d_ui, 'TerrainManager', Terrain)
        monkeypatch.setattr(map3d_ui.mp_tile, 'MPTile',
                            lambda **kwargs: None)
        if state is None:
            state = SimpleNamespace(event_queue=SimpleNamespace(
                put=(events if events is not None else []).append))
        for (name, value) in (('service', None), ('zexag', 1.0),
                              ('height', 800)):
            if not hasattr(state, name):
                setattr(state, name, value)
        frame = map3d_ui.Map3DFrame.__new__(map3d_ui.Map3DFrame)
        frame.state = state
        frame.terrain = None if elements is None else Terrain()
        frame.elements = elements
        frame.ren = vtk.vtkRenderer()
        frame.widget = SimpleNamespace(SetInteractorStyle=lambda style: None)
        frame.status_actor = SimpleNamespace(SetInput=lambda text: None)
        frame.GetClientSize = lambda: (1100, 800)
        frame.render = lambda: None
        frame.tc = None
        frame.follow = False
        frame.fpv_enabled = False
        frame.fpv_fov = 90.0
        frame.terrain_brightness = 1.25
        frame.terrain_shading = True
        frame.terrain_wireframe = False
        frame.saved_map_view = None
        frame.vehicle_type = None
        frame.kml_features = []
        frame.kml_refresh_due = None
        frame.overlay_mesh_revision = -1
        frame.mission_arrows = False
        frame.mission_labelled = False
        frame.mission_label_size = MISSION_LABEL_SIZE
        frame.mission_style = MISSION_STYLES[0]
        frame.labels_check = Widget(frame.mission_labelled)
        frame.style_choice = Widget(frame.mission_style)
        frame.follow_button = Widget(frame.follow)
        return frame
    return build


@pytest.fixture
def map3d_module():
    """build the live map3d module with the settings it makes, around the
    map it is given: there is no other module for it to ask anything of,
    and no mission to fly"""
    def build(map=None):
        from types import SimpleNamespace
        from MAVProxy.modules import mavproxy_map3d
        module = mavproxy_map3d.Map3DModule.__new__(
            mavproxy_map3d.Map3DModule)
        module.map3d_settings = mavproxy_map3d.make_settings()
        module.map = map
        module.follow = True
        module.send_mission = lambda: None
        module.mpstate = SimpleNamespace(module=lambda name: None)
        module.kml_change_state = None
        module.terrain_resolved = False
        module.ground_heading_changed = False
        module.reset_flown_track()
        return module
    return build


@pytest.fixture
def mavexplorer():
    '''MAVExplorer, which MAVProxy/tools is not a package to import'''
    pytest.importorskip("wx")
    pytest.importorskip("lxml")
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                        'MAVProxy', 'tools', 'MAVExplorer.py')
    spec = importlib.util.spec_from_file_location('mavexplorer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
