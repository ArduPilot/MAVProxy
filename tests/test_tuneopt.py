'''the tuneopt command against the TUNE parameters Copter has'''

import contextlib
import io
from types import SimpleNamespace

from MAVProxy.modules import mavproxy_tuneopt


def make_module(params):
    '''a tuneopt module whose vehicle has params; param_set writes to them'''
    sets = []

    def param_set(name, value, retries=3):
        sets.append((name, value))
        params[name] = value

    state = SimpleNamespace(
        command_map={}, completions={}, vehicle_type='copter',
        functions=SimpleNamespace(get_mav_param=lambda name, default=None: params.get(name, default),
                                  param_set=param_set))
    return (mavproxy_tuneopt.init(state), sets)


def run(module, args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        module.cmd_tuneopt(args)
    return out.getvalue()


def test_set_writes_tune_min_and_max():
    (module, sets) = make_module({'TUNE': 0, 'TUNE_MIN': 0.0, 'TUNE_MAX': 0.0})
    run(module, ['set', 'RateRollkP', '0.1', '0.3'])
    assert sets == [('TUNE', '49'), ('TUNE_MIN', 0.1), ('TUNE_MAX', 0.3)]


def test_show_reads_tune_min_and_max():
    (module, _) = make_module({'TUNE': 49, 'TUNE_MIN': 0.1, 'TUNE_MAX': 0.3})
    assert run(module, ['show']).strip() == 'TUNE is currently set to RateRollkP LOW=0.100000 HIGH=0.300000'


def test_tune_low_and_high_of_older_copters():
    '''Copter 3.6 and earlier hold 1000 times the value in TUNE_LOW and TUNE_HIGH'''
    (module, sets) = make_module({'TUNE': 0, 'TUNE_LOW': 0, 'TUNE_HIGH': 0})
    run(module, ['set', 'RateRollkP', '0.1', '0.3'])
    assert sets == [('TUNE', '49'), ('TUNE_LOW', 100.0), ('TUNE_HIGH', 300.0)]
    assert 'LOW=0.100000 HIGH=0.300000' in run(module, ['show'])


def test_older_copters_get_their_options():
    '''Copter 3.6 and earlier number LoiterSpeed 42 and scale RCFeel and
    MotorYawHeadroom differently in TUNE_LOW and TUNE_HIGH'''
    (module, sets) = make_module({'TUNE': 0, 'TUNE_LOW': 0, 'TUNE_HIGH': 0})
    run(module, ['set', 'LoiterSpeed', '0.1', '0.3'])
    assert sets == [('TUNE', '42'), ('TUNE_LOW', 100.0), ('TUNE_HIGH', 300.0)]
    sets.clear()
    assert run(module, ['set', 'MotorYawHeadroom', '10', '20']).strip() == 'Invalid Tune option: MotorYawHeadroom'
    assert sets == []


def test_new_names_until_the_parameters_are_known():
    (module, sets) = make_module({})
    run(module, ['set', 'LoiterSpeed', '1', '5'])
    assert sets == [('TUNE', '60'), ('TUNE_MIN', 1.0), ('TUNE_MAX', 5.0)]


def test_invalid_range_writes_nothing():
    (module, sets) = make_module({'TUNE': 0, 'TUNE_MIN': 0.0, 'TUNE_MAX': 0.0})
    assert run(module, ['set', 'RateRollkP', '0.15', 'bad']).strip() == 'Usage: tuneopt set OPTION LOW HIGH'
    assert sets == []


def test_show_without_a_range():
    (module, _) = make_module({'TUNE': 49})
    assert run(module, ['show']).strip() == 'TUNE is currently set to RateRollkP, range unknown'


def test_options_are_distinct():
    for options in [mavproxy_tuneopt.tune_options, mavproxy_tuneopt.legacy_tune_options]:
        values = list(options.values())
        assert len(values) == len(set(values))
