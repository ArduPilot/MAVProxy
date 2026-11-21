'''the misc module commands that read and set vehicle parameters'''

import contextlib
import io
from types import SimpleNamespace

import pytest

from MAVProxy.modules import mavproxy_misc


def make_module(params, fields=None):
    '''a misc module whose vehicle has params and sends fields; param_set
    writes to params'''
    sets = []
    fields = fields or {}

    def param_set(name, value, retries=3):
        sets.append((name, value))
        params[name] = value

    master = SimpleNamespace(field=lambda msg, field, default=None: fields.get((msg, field), default))
    state = SimpleNamespace(
        command_map={}, completions={}, public_modules={}, master=lambda: master,
        functions=SimpleNamespace(get_mav_param=lambda name, default=None: params.get(name, default),
                                  param_set=param_set))
    return (mavproxy_misc.init(state), sets)


def run_up(module, args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        module.cmd_up(args)
    return out.getvalue()


def test_up_adjusts_ptch_trim_deg():
    (module, sets) = make_module({'PTCH_TRIM_DEG': 1.5})
    assert run_up(module, ['2']).strip() == 'Adjusting PTCH_TRIM_DEG from 1.5 to 3.5'
    assert sets == [('PTCH_TRIM_DEG', 3.5)]


def test_up_adjusts_trim_pitch_cd_of_older_planes():
    (module, sets) = make_module({'TRIM_PITCH_CD': 150})
    run_up(module, [])
    assert sets == [('TRIM_PITCH_CD', 650)]


def test_up_refuses_a_large_adjustment():
    (module, sets) = make_module({'PTCH_TRIM_DEG': 0.0})
    assert 'too large' in run_up(module, ['11'])
    assert sets == []


def test_up_without_a_trim_parameter():
    (module, sets) = make_module({})
    assert run_up(module, []).strip() == 'Existing trim value unknown!'
    assert sets == []


@pytest.mark.parametrize('params, temperature', [
    ({'BARO_GND_TEMP': 40.0}, 40.0),
    ({'BARO_GND_TEMP': 0.0}, 21),
    ({'GND_TEMP': 30.0}, 30.0),
    ({'BARO_GND_TEMP': 0.0, 'GND_TEMP': 30.0}, 21),
    ({}, 21),
])
def test_ground_temperature(params, temperature):
    (module, _) = make_module(params)
    assert module.ground_temperature() == temperature


def test_qnh_estimate_uses_baro_gnd_temp():
    fields = {('GPS_RAW_INT', 'alt'): 584000, ('SCALED_PRESSURE', 'press_abs'): 945.0}
    (warm, _) = make_module({'BARO_GND_TEMP': 40.0}, fields)
    (unset, _) = make_module({'BARO_GND_TEMP': 0.0}, fields)
    # the QNH an altimeter set to it reads 584 m with, at that temperature
    assert warm.altitude_difference(warm.qnh_estimate(), 945.0, 40.0) == pytest.approx(584.0, abs=0.1)
    assert unset.altitude_difference(unset.qnh_estimate(), 945.0, 21) == pytest.approx(584.0, abs=0.1)
    assert warm.qnh_estimate() < unset.qnh_estimate() - 3
