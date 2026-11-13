'''cmdlong's command retry queue, driven by a fake link and clock.

The retry tests need a pymavlink with COMMAND_LONG/COMMAND_ACK.command_opaque_id
(mavlink/mavlink#2458) and are skipped without it; the fallback tests check
what happens with a pymavlink that doesn't have the field.
'''
from types import SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from MAVProxy.modules import mavproxy_cmdlong

# MAVProxy runs with MAVLINK20 set; MAVLink 1 dialects have no extension fields
HAVE_FIELD = 'command_opaque_id' in mavlink2.MAVLink_command_long_message.fieldnames
needs_field = pytest.mark.skipif(not HAVE_FIELD, reason="pymavlink lacks command_opaque_id")
needs_no_field = pytest.mark.skipif(HAVE_FIELD, reason="pymavlink has command_opaque_id")

PAUSE_CONTINUE = mavlink2.MAV_CMD_DO_PAUSE_CONTINUE


class FakeMav(object):
    def __init__(self):
        self.sent = []

    def send(self, m):
        self.sent.append(m)


class Harness(object):
    def __init__(self, monkeypatch):
        self.now = 1000.0
        monkeypatch.setattr(mavproxy_cmdlong, 'time', SimpleNamespace(time=lambda: self.now))
        monkeypatch.setattr(mavutil, 'mavlink', mavlink2)
        self.master = SimpleNamespace(mav=FakeMav())
        module = mavproxy_cmdlong.CmdlongModule.__new__(mavproxy_cmdlong.CmdlongModule)
        module.mpstate = SimpleNamespace(
            master=lambda: self.master,
            settings=SimpleNamespace(target_system=1, target_component=1))
        module.last_command_retry_ms = 0
        module.command_opaque_id_seq = 1
        module.command_queue = {}
        module.command_retry_permitted_sysids = set()
        self.module = module
        self.sent = self.master.mav.sent

    def advance(self, dt):
        self.now += dt
        self.module.retry_commands()

    def ack(self, opaque_id, result=mavlink2.MAV_RESULT_ACCEPTED, srcsys=1, command=PAUSE_CONTINUE):
        nargs = len(mavlink2.MAVLink_command_ack_message.fieldnames)
        args = [command, result] + [0] * (nargs - 2)
        if HAVE_FIELD:
            args[-1] = opaque_id
        m = mavlink2.MAVLink_command_ack_message(*args)
        m._header = SimpleNamespace(srcSystem=srcsys, srcComponent=1)
        self.module.mavlink_packet(m)

    def resume(self):
        self.module.cmd_resume([])
        return self.sent[-1].command_opaque_id

    def pause(self):
        self.module.cmd_pause([])
        return self.sent[-1].command_opaque_id

    def permit_retries(self):
        '''the vehicle has shown it supports command_opaque_id'''
        self.ack(self.resume())
        del self.sent[:]


@pytest.fixture
def h(monkeypatch):
    return Harness(monkeypatch)


@needs_field
def test_ids_wrap_without_using_zero(h):
    h.module.command_opaque_id_seq = 0xFFFFFFFF
    assert h.resume() == 0xFFFFFFFF
    assert h.resume() == 1


@needs_field
def test_no_retry_before_vehicle_acks_an_id(h):
    h.resume()
    h.advance(6)
    assert len(h.sent) == 1
    assert h.module.command_queue == {}


@needs_field
def test_acked_command_is_not_resent(h):
    h.permit_retries()
    h.ack(h.resume())
    assert h.module.command_queue == {}
    h.advance(6)
    h.advance(6)
    assert len(h.sent) == 1


@needs_field
def test_lost_ack_is_retried_with_the_same_id(h):
    h.permit_retries()
    rid = h.resume()
    h.advance(3)
    assert len(h.sent) == 1
    h.advance(3)
    assert [m.command_opaque_id for m in h.sent] == [rid, rid]
    h.advance(6)
    assert [m.command_opaque_id for m in h.sent] == [rid, rid, rid]
    # the last resend still gets its ACK window
    assert rid in h.module.command_queue
    h.advance(2)
    h.ack(rid)
    assert rid not in h.module.command_queue
    h.advance(6)
    assert len(h.sent) == 3


@needs_field
def test_unacked_command_is_retried_twice_then_dropped(h):
    h.permit_retries()
    rid = h.resume()
    for _ in range(5):
        h.advance(6)
    assert len(h.sent) == 3
    assert rid not in h.module.command_queue


@needs_field
def test_pause_cancels_pending_resume(h):
    h.permit_retries()
    resume_id = h.resume()
    pause_id = h.pause()
    assert pause_id != resume_id
    assert h.sent[-1].param1 == 0
    assert list(h.module.command_queue) == [pause_id]
    h.advance(6)
    assert h.sent[-1].command_opaque_id == pause_id
    assert h.sent[-1].param1 == 0


@needs_field
def test_mismatched_acks_are_ignored(h):
    h.permit_retries()
    rid = h.resume()
    h.ack(rid, srcsys=2)
    assert rid in h.module.command_queue
    h.ack(rid, command=mavlink2.MAV_CMD_DO_SET_MODE)
    assert rid in h.module.command_queue
    h.ack(rid, result=mavlink2.MAV_RESULT_IN_PROGRESS)
    assert rid in h.module.command_queue
    h.ack(0)
    assert rid in h.module.command_queue
    h.ack(rid)
    assert rid not in h.module.command_queue


@needs_no_field
def test_without_field_commands_are_sent_once(h):
    h.module.cmd_resume([])
    assert len(h.sent) == 1
    assert h.module.command_queue == {}
    h.ack(0)
    h.advance(6)
    h.advance(6)
    assert len(h.sent) == 1
