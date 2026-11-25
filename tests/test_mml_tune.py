'''tests for the MML tune parser and the tuneplayer module's player'''

import time
import wave
import io

import pytest

from MAVProxy.modules.lib import mml_tune
from MAVProxy.modules import mavproxy_tuneplayer


def summary(tune_string):
    tune = mml_tune.parse(tune_string)
    return [(round(t.frequency, 2), round(t.duration, 4)) for t in tune.tones], tune.repeat


def test_note_and_gap():
    # a quarter note at 120bpm is 0.5s: 7/8 note then 1/8 gap ("MN")
    assert summary("MFT120L4O4A") == ([(1760.0, 0.4375), (0, 0.0625)], False)


def test_lower_case_and_whitespace():
    assert summary("mf t120 l4 o4 a") == summary("MFT120L4O4A")


def test_legato_staccato():
    assert summary("MLT120L4O4A") == ([(1760.0, 0.5)], False)
    assert summary("MST120L4O4A") == ([(1760.0, 0.375), (0, 0.125)], False)


def test_sharp_flat_and_octaves():
    tones, _ = summary("MLO4AA#A-<A>>A")
    freqs = [f for (f, d) in tones]
    assert freqs == [1760.0, 1864.66, 1661.22, 880.0, 3520.0]


def test_dots_and_rests():
    tones, _ = summary("MLT120L4A.P4R8.")
    assert tones == [(1760.0, 0.75), (0, 0.5), (0, 0.375)]


def test_repeat():
    assert summary("MBC")[1] is True
    assert summary("MBMFC")[1] is False


def test_errors_stop_playback():
    # playback stops at the first character not understood...
    tones, repeat = summary("MBMLCDXE")
    assert len(tones) == 2
    # ...and a stopped tune is not repeated
    assert repeat is False
    # tempos under 32 and note lengths of zero stop playback
    assert summary("T20C") == ([], False)
    assert summary("L0C") == ([], False)
    assert summary("N85") == ([], False)


def test_numbers_are_eight_bit():
    # as on the vehicle, T300 is a tempo of 300 & 0xff == 44
    assert summary("MLT300C") == summary("MLT44C")


def test_volume():
    tune = mml_tune.parse("V0CV255C")
    assert tune.tones[0].volume == 0
    assert tune.tones[2].volume == 1


def test_nul_terminates():
    assert summary("C\0D") == summary("C")


def test_synthesise_length():
    tune = mml_tune.parse("MLT120L4A")
    for waveform in ("square", "sine"):
        pcm = mml_tune.synthesise(tune, sample_rate=8000, waveform=waveform)
        assert len(pcm) == 2 * 4000
        assert any(pcm)
    w = wave.open(io.BytesIO(mml_tune.to_wav(pcm, 8000)))
    assert w.getnframes() == 4000
    assert w.getframerate() == 8000


class FakePlayback(object):
    def __init__(self, backend, duration):
        self.backend = backend
        self.end = time.time() + duration
        self.stopped = False

    def is_playing(self):
        return not self.stopped and time.time() < self.end

    def stop(self):
        self.stopped = True
        self.backend.stops += 1


class FakeBackend(object):
    name = "fake"

    def __init__(self):
        self.starts = 0
        self.stops = 0

    def start(self, pcm, duration):
        self.starts += 1
        return FakePlayback(self, duration)


def make_request(tune_string):
    return mavproxy_tuneplayer._Request(mml_tune.parse(tune_string), 0.5, "square")


def wait_until(condition, timeout=5):
    tstart = time.time()
    while not condition():
        if time.time() - tstart > timeout:
            pytest.fail("timed out")
        time.sleep(0.01)


def test_player_repeats_until_stopped():
    backend = FakeBackend()
    player = mavproxy_tuneplayer._Player(backend)
    try:
        # about 0.1s long, repeating
        player.play(make_request("MBT255L32C"))
        wait_until(lambda: backend.starts >= 3)
        player.play(None)
        wait_until(lambda: backend.stops >= 1)
        starts = backend.starts
        time.sleep(0.3)
        assert backend.starts == starts
    finally:
        player.close()


def test_player_new_tune_replaces_old():
    backend = FakeBackend()
    player = mavproxy_tuneplayer._Player(backend)
    try:
        player.play(make_request("T32L1C"))
        wait_until(lambda: backend.starts == 1)
        player.play(make_request("T32L1D"))
        wait_until(lambda: backend.starts == 2)
        assert backend.stops == 1
    finally:
        player.close()


def test_module_loads_without_tune_messages(monkeypatch):
    # pymavlink's ArduPilot dialect may not have PLAY_TUNE_V2 or TUNE_FORMAT
    import importlib
    from pymavlink import mavutil
    for name in ('TUNE_FORMAT_QBASIC1_1', 'TUNE_FORMAT_MML_MODERN', 'MAVLINK_MSG_ID_PLAY_TUNE_V2'):
        monkeypatch.delattr(mavutil.mavlink, name, raising=False)
    try:
        module = importlib.reload(mavproxy_tuneplayer)
        assert set(module.SUPPORTED_FORMATS.keys()) == {1, 2}
    finally:
        monkeypatch.undo()
        importlib.reload(mavproxy_tuneplayer)


class SlowRequest(mavproxy_tuneplayer._Request):
    def get_pcm(self):
        time.sleep(0.3)
        return super().get_pcm()


def test_player_skips_tune_superseded_while_synthesising():
    backend = FakeBackend()
    player = mavproxy_tuneplayer._Player(backend)
    try:
        player.play(SlowRequest(mml_tune.parse("T32L1C"), 0.5, "square"))
        player.play(None)
        time.sleep(0.6)
        assert backend.starts == 0
    finally:
        player.close()


def test_failed_player_command_removes_wav(monkeypatch, tmp_path):
    import tempfile
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    backend = mavproxy_tuneplayer._CommandBackend([str(tmp_path / "no-such-player")])
    with pytest.raises(OSError):
        backend.start(mml_tune.synthesise(mml_tune.parse("C")), 0.5)
    assert list(tmp_path.iterdir()) == []


class FailingRequest(mavproxy_tuneplayer._Request):
    def get_pcm(self):
        raise ValueError("cannot convert float NaN to integer")


def test_player_survives_synthesis_error():
    backend = FakeBackend()
    player = mavproxy_tuneplayer._Player(backend)
    try:
        player.play(FailingRequest(mml_tune.parse("MBT32L1C"), 0.5, "square"))
        player.play(make_request("T255L32C"))
        wait_until(lambda: backend.starts == 1)
    finally:
        player.close()


def test_player_other_vehicle_does_not_stop_alarm():
    backend = FakeBackend()
    player = mavproxy_tuneplayer._Player(backend)
    try:
        # vehicle 1's repeating alarm, about 0.1s long
        player.play(make_request("MBT255L32C"), (1, 1))
        wait_until(lambda: backend.starts >= 2)
        # vehicle 2's stop doesn't affect it
        player.play(None, (2, 1))
        starts = backend.starts
        wait_until(lambda: backend.starts >= starts + 2)
        # vehicle 2's tune interrupts it, then the alarm carries on
        player.play(make_request("T255L32D"), (2, 1))
        starts = backend.starts
        wait_until(lambda: backend.starts >= starts + 3)
        # vehicle 1's stop ends it
        player.play(None, (1, 1))
        time.sleep(0.2)
        starts = backend.starts
        time.sleep(0.3)
        assert backend.starts == starts
    finally:
        player.close()


class FakeMessage(object):
    def __init__(self, tune, src_system=1):
        self.target_system = 255
        self.target_component = 0
        self.format = 2
        self.tune = tune
        self.src_system = src_system

    def get_type(self):
        return 'PLAY_TUNE_V2'

    def get_srcSystem(self):
        return self.src_system

    def get_srcComponent(self):
        return 1


def test_disabling_module_stops_alarm():
    class FakeSettings(object):
        source_system = 255
        source_component = 230

    class FakeMPState(object):
        settings = FakeSettings()
        public_modules = {}
        completions = {}
        completion_functions = {}
        command_map = {}

    backend = FakeBackend()
    module = mavproxy_tuneplayer.TunePlayerModule(FakeMPState())
    module.player = mavproxy_tuneplayer._Player(backend)
    module.player_setting = module.tuneplayer_settings.player
    try:
        module.mavlink_packet(FakeMessage("MBT255L32C"))
        wait_until(lambda: backend.starts >= 2)
        module.tuneplayer_settings.set('enabled', False)
        time.sleep(0.2)
        starts = backend.starts
        time.sleep(0.3)
        assert backend.starts == starts
    finally:
        module.unload()
