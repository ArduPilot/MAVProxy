'''the instructor module's GNSS switch against the SITL parameters'''

from types import SimpleNamespace
from unittest import mock

import pytest

pytest.importorskip('wx')

from MAVProxy.modules.lib import wx_util  # noqa: E402

with mock.patch.object(wx_util, 'safe', True):
    from MAVProxy.modules.mavproxy_instructor import InstructorModule  # noqa: E402


def make_module(params):
    '''an instructor module, without its window, whose vehicle has params'''
    sets = []

    def param_set(name, value, retries=3):
        sets.append((name, value))

    module = InstructorModule.__new__(InstructorModule)
    module.mpstate = SimpleNamespace(functions=SimpleNamespace(
        get_mav_param=lambda name, default=None: params.get(name, default), param_set=param_set))
    return (module, sets)


@pytest.mark.parametrize('disable, enable', [(True, 0), (False, 1)])
@pytest.mark.parametrize('params', [{'SIM_GPS1_ENABLE': 1}, {}])
def test_gnss_switch_sets_sim_gps1_enable(disable, enable, params):
    '''also before the parameters are downloaded'''
    (module, sets) = make_module(params)
    module.disable_gnss(disable)
    assert sets == [('SIM_GPS1_ENABLE', enable)]


@pytest.mark.parametrize('disable, value', [(True, 1), (False, 0)])
def test_gnss_switch_of_older_sitl(disable, value):
    (module, sets) = make_module({'SIM_GPS_DISABLE': 0})
    module.disable_gnss(disable)
    assert sets == [('SIM_GPS_DISABLE', value)]
