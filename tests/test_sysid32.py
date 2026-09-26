"""Route slave messages using full-width dialect targets without changing bytes."""
import ast
from pathlib import Path
import socket
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from MAVProxy.modules.lib.mp_settings import MPSetting

SOURCE = Path(__file__).resolve().parents[1] / 'MAVProxy/mavproxy.py'


def slave_forwarder(state):
    # mavproxy.py starts the application at import time; isolate this real handler.
    tree = ast.parse(SOURCE.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'process_mavlink')
    namespace = dict(mpstate=state, socket=socket)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['process_mavlink']


@pytest.mark.parametrize('target', [None, 0, 255, 256, 0x80000000, 0xFFFFFFFF])
@pytest.mark.parametrize('legacy', [False, True])
def test_forward_target_and_packet_identity(target, legacy):
    message = SimpleNamespace(get_msgbuf=lambda: b'original signed packet')
    if legacy:
        if target is not None:
            message.target_system = target
    else:
        message.get_target_system = lambda: target
        # An equivalent field (e.g. MANUAL_CONTROL.target) need not be named target_system.
        message.target = target
    output = Mock()
    state = SimpleNamespace(
        settings=SimpleNamespace(mavfwd=True, mavfwd_link=0, mavfwd_signing=False),
        status=SimpleNamespace(setup_mode=False, watch=None, counters={'Slave': 0}),
        mav_master=[output], master=Mock(return_value=output), logqueue=None)
    slave = SimpleNamespace(recv=lambda: b'wire', first_byte=False,
                            mav=SimpleNamespace(parse_buffer=lambda _: [message]))
    slave_forwarder(state)(slave)
    state.master.assert_called_once_with(-1 if target is None else target)
    output.write.assert_called_once_with(b'original signed packet')
    assert state.status.counters['Slave'] == 1


@pytest.mark.parametrize('name', ['source_system', 'target_system'])
def test_full_range_settings_do_not_require_signed_spin_control(name):
    tree = ast.parse(SOURCE.read_text())
    call = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name) and node.func.id == 'MPSetting'
                and node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == name)
    setting = eval(compile(ast.Expression(call), str(SOURCE), 'eval'), {'MPSetting': MPSetting})
    assert setting.increment is None
    for value in [0, 255, 256, 0x80000000, 0xFFFFFFFF]:
        assert setting.set(str(value))
        assert setting.value == value
    assert not setting.set('-1')
    assert not setting.set('4294967296')
