"""Regression tests for the smartcamera module loading (issue #1565)."""

import sys
import types

import pytest


@pytest.fixture
def stub_piexif(monkeypatch):
    """piexif is only an optional MAVProxy dependency, so stub it: the
    smartcamera package imports it for EXIF writing at runtime, and the
    tests here only exercise the import."""
    if "piexif" not in sys.modules:
        monkeypatch.setitem(sys.modules, "piexif", types.ModuleType("piexif"))


def test_smartcamera_module_imports(stub_piexif):
    """The module must import as a package: bare intra-package imports and
    Python 2 stdlib names broke `module load smartcamera`."""
    from MAVProxy.modules import mavproxy_smartcamera

    assert mavproxy_smartcamera.SmartCameraModule is not None
    assert callable(mavproxy_smartcamera.init)
    assert mavproxy_smartcamera.SmartCameraWebCam is not None
    assert mavproxy_smartcamera.SmartCamera_SonyQX is not None


def test_smartcamera_config_defaults(stub_piexif):
    """The ported config reader returns defaults when no config file exists."""
    from MAVProxy.modules.mavproxy_smartcamera import sc_config

    assert sc_config.config.get_integer("camera0", "type", 0) == 0
    assert sc_config.config.get_string("general", "WirelessPort", "wlan0") == "wlan0"


def test_smartcamera_video_imports(stub_piexif):
    """The video helper shares the same intra-package import fix."""
    from MAVProxy.modules.mavproxy_smartcamera import sc_video

    assert sc_video.SmartCameraVideo is not None


def test_smartcamera_ssdp_imports(stub_piexif):
    """The SSDP helper must import with its Python 3 stdlib names."""
    from MAVProxy.modules.mavproxy_smartcamera import ssdp

    assert callable(ssdp.discover)
