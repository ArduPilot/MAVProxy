#!/usr/bin/env python3
'''
Tune player module

Plays the tunes vehicles send to this ground station in PLAY_TUNE_V2
messages.  ArduPilot does this when its NTF_OPTIONS "Send tunes to GCS"
bit is set, so the operator hears the vehicle's alarms however far away
it is.

Sound is played with the simpleaudio Python package if it is installed,
with winsound on Windows, or else with the first of paplay, pw-play,
aplay or afplay found on the PATH.  "tuneplayer set player COMMAND"
chooses a different command; it is given the name of a WAV file.

Commands:
  tuneplayer play TUNE        play a tune locally, e.g. to try one out
  tuneplayer stop             stop the tune that is playing
  tuneplayer wav FILE TUNE    write a tune to a WAV file
  tuneplayer status           show what the module is doing
  tuneplayer set NAME VALUE   change a setting

AP_FLAKE8_CLEAN
'''

import os
import queue
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mml_tune
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

SAMPLE_RATE = 22050

# TUNE_FORMAT values.  These are not in every dialect pymavlink provides
# (e.g. ArduPilot's, before it gained PLAY_TUNE_V2), so don't rely on
# mavutil.mavlink having them
TUNE_FORMAT_QBASIC1_1 = 1
TUNE_FORMAT_MML_MODERN = 2

SUPPORTED_FORMATS = {
    TUNE_FORMAT_QBASIC1_1: "QBasic 1.1",
    TUNE_FORMAT_MML_MODERN: "MML",
}


class _Playback(object):
    '''a sound which is playing'''

    def is_playing(self):
        return False

    def stop(self):
        pass


class _SimpleAudioPlayback(_Playback):
    def __init__(self, play_object):
        self.play_object = play_object

    def is_playing(self):
        return self.play_object.is_playing()

    def stop(self):
        self.play_object.stop()


class _SimpleAudioBackend(object):
    name = "simpleaudio"

    def __init__(self):
        import simpleaudio
        self.simpleaudio = simpleaudio

    def start(self, pcm, duration):
        return _SimpleAudioPlayback(self.simpleaudio.play_buffer(pcm, 1, 2, SAMPLE_RATE))


class _FilePlayback(_Playback):
    '''a WAV file being played by something else; removes the file once done'''

    def __init__(self, path):
        self.path = path

    def remove_file(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass


class _CommandPlayback(_FilePlayback):
    def __init__(self, path, command):
        super().__init__(path)
        try:
            self.process = subprocess.Popen(command + [path],
                                            stdin=subprocess.DEVNULL,
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.DEVNULL)
        except Exception:
            self.remove_file()
            raise

    def is_playing(self):
        if self.process.poll() is None:
            return True
        self.remove_file()
        return False

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.remove_file()


def _write_wav(pcm):
    fd, path = tempfile.mkstemp(prefix="mavproxy-tune-", suffix=".wav")
    with os.fdopen(fd, "wb") as f:
        f.write(mml_tune.to_wav(pcm, SAMPLE_RATE))
    return path


class _CommandBackend(object):
    def __init__(self, command):
        self.command = command
        self.name = " ".join(command)

    def start(self, pcm, duration):
        return _CommandPlayback(_write_wav(pcm), self.command)


class _WinsoundPlayback(_FilePlayback):
    def __init__(self, path, duration, winsound):
        super().__init__(path)
        self.winsound = winsound
        self.end_time = time.time() + duration
        try:
            winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC)
        except Exception:
            self.remove_file()
            raise

    def is_playing(self):
        if time.time() < self.end_time:
            return True
        self.remove_file()
        return False

    def stop(self):
        self.winsound.PlaySound(None, 0)
        self.remove_file()


class _WinsoundBackend(object):
    name = "winsound"

    def __init__(self):
        import winsound
        self.winsound = winsound

    def start(self, pcm, duration):
        return _WinsoundPlayback(_write_wav(pcm), duration, self.winsound)


# commands tried, in order, when looking for something to play sound with
PLAYER_COMMANDS = [
    ["paplay"],
    ["pw-play"],
    ["aplay", "-q"],
    ["afplay"],
]


def find_backend(player):
    '''return a backend for the "player" setting, or None if none is available'''
    if player != "auto":
        if player == "simpleaudio":
            return _SimpleAudioBackend()
        if player == "winsound":
            return _WinsoundBackend()
        return _CommandBackend(shlex.split(player))
    try:
        return _SimpleAudioBackend()
    except ImportError:
        pass
    if sys.platform.startswith("win"):
        return _WinsoundBackend()
    for command in PLAYER_COMMANDS:
        if shutil.which(command[0]) is not None:
            return _CommandBackend(command)
    return None


class _Request(object):
    def __init__(self, tune, volume, waveform):
        self.tune = tune
        self.volume = volume
        self.waveform = waveform
        self.duration = tune.duration()
        self.repeat = tune.repeat
        self.pcm = None

    def get_pcm(self):
        # synthesised in the player's thread, as this can take a while
        if self.pcm is None:
            self.pcm = mml_tune.synthesise(self.tune,
                                           sample_rate=SAMPLE_RATE,
                                           volume=self.volume,
                                           waveform=self.waveform)
        return self.pcm


class _Player(object):
    '''plays one tune at a time in a thread of its own.

    Each tune comes from a source (a vehicle, or None for this GCS).  A
    new tune replaces the one playing, and an empty one (None) stops the
    tune playing if it came from the same source.  A repeating ("MB")
    tune is remembered until its source stops it or sends another, so a
    tune from one vehicle doesn't end another vehicle's alarm: it plays
    again once the other tune is done.  A stop from this GCS stops
    everything.'''

    _QUIT = object()

    def __init__(self, backend):
        self.backend = backend
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self.run, name="tuneplayer")
        self.thread.daemon = True
        self.thread.start()

    def play(self, request, source=None):
        self.queue.put((source, request))

    def close(self):
        self.queue.put(self._QUIT)
        self.thread.join(2)

    def start(self, request):
        '''start playing a request.  Returns the playback, None if the
        tune could not be played, or the request if something arrived
        while it was being synthesised'''
        try:
            pcm = request.get_pcm()
        except Exception as ex:
            print("tuneplayer: failed to synthesise tune: %s" % ex)
            return None
        if not self.queue.empty():
            # a newer tune, a stop or close() arrived while this one was
            # being synthesised; deal with that first
            return request
        try:
            return self.backend.start(pcm, request.duration)
        except Exception as ex:
            print("tuneplayer: failed to play tune with %s: %s" % (self.backend.name, ex))
            return None

    def run(self):
        playback = None
        current = None    # (source, request) playing or to be started
        pending = False   # current is still to be started
        repeating = {}    # source -> its repeating request, oldest first
        while True:
            try:
                item = self.queue.get(timeout=0.05)
            except queue.Empty:
                item = None
            if item is self._QUIT:
                if playback is not None:
                    playback.stop()
                return
            if item is not None:
                source, request = item
                if request is None:
                    if source is None:
                        repeating.clear()
                    else:
                        repeating.pop(source, None)
                    if current is None or (source is not None and current[0] != source):
                        # nothing of this source's is playing
                        continue
                    current = None
                else:
                    repeating.pop(source, None)
                    if request.repeat:
                        repeating[source] = request
                    current = (source, request)
                pending = current is not None
                if playback is not None:
                    playback.stop()
                    playback = None
            if playback is not None:
                if playback.is_playing():
                    continue
                playback = None
                if current is not None and repeating.get(current[0]) is current[1]:
                    # "MB" tunes play until stopped
                    pending = True
                else:
                    current = None
            if current is None and len(repeating) > 0:
                # carry on with the most recent alarm still repeating
                source = list(repeating.keys())[-1]
                current = (source, repeating[source])
                pending = True
            if not pending:
                continue
            result = self.start(current[1])
            if result is current[1]:
                # superseded while synthesising; start it after the
                # queue has been dealt with if it is still wanted
                continue
            pending = False
            if result is None:
                # couldn't be played; don't keep trying
                if repeating.get(current[0]) is current[1]:
                    del repeating[current[0]]
                current = None
            playback = result


class TunePlayerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "tuneplayer", "play tunes sent by vehicles", public=True, multi_vehicle=True)
        self.tuneplayer_settings = mp_settings.MPSettings([
            ('enabled', bool, True),
            ('volume', float, 0.5),
            ('waveform', str, 'square'),
            ('player', str, 'auto'),
            ('verbose', bool, False),
        ])
        self.add_command('tuneplayer', self.cmd_tuneplayer, "play tunes sent by vehicles",
                         ['play', 'stop', 'status', 'wav (FILENAME)', 'set (TUNEPLAYERSETTING)'])
        self.add_completion_function('(TUNEPLAYERSETTING)', self.tuneplayer_settings.completion)
        self.tuneplayer_settings.set_callback(self.setting_changed)
        self.player = None
        self.player_setting = None
        self.tunes_received = 0
        self.last_tune = None
        self.warned_formats = set()
        if not hasattr(mavutil.mavlink, 'MAVLINK_MSG_ID_PLAY_TUNE_V2'):
            # message not in this dialect
            print("tuneplayer: PLAY_TUNE_V2 is not in this MAVLink dialect; tunes from vehicles can't be played")

    def setting_changed(self, setting):
        if setting.name == 'enabled' and not setting.value and self.player is not None:
            # stop anything a vehicle asked for, e.g. a repeating alarm
            self.player.play(None)

    def usage(self):
        return "Usage: tuneplayer <play TUNE|stop|wav FILENAME TUNE|status|set NAME VALUE>"

    def cmd_tuneplayer(self, args):
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "play" and len(args) > 1:
            self.play_tune(" ".join(args[1:]))
        elif args[0] == "stop":
            self.play_tune("")
        elif args[0] == "wav" and len(args) > 2:
            self.cmd_wav(args[1], " ".join(args[2:]))
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.tuneplayer_settings.command(args[1:])
        else:
            print(self.usage())

    def status(self):
        player = self.get_player()
        if player is None:
            backend = "none found"
        else:
            backend = player.backend.name
        return ("tuneplayer: enabled=%s player=%s tunes received=%u last tune=%s" %
                (self.tuneplayer_settings.enabled, backend, self.tunes_received, self.last_tune))

    def make_request(self, tune):
        return _Request(tune, self.tuneplayer_settings.volume, self.tuneplayer_settings.waveform)

    def cmd_wav(self, filename, tune_string):
        tune = mml_tune.parse(tune_string)
        with open(filename, "wb") as f:
            f.write(mml_tune.to_wav(self.make_request(tune).get_pcm(), SAMPLE_RATE))
        print("tuneplayer: wrote %.1f seconds of audio to %s" % (tune.duration(), filename))

    def get_player(self):
        '''return the player for the current "player" setting, creating it if needed'''
        setting = self.tuneplayer_settings.player
        if self.player_setting == setting:
            # already looked for this setting; self.player is None if
            # nothing was found
            return self.player
        if self.player is not None:
            self.player.close()
            self.player = None
        self.player_setting = setting
        try:
            backend = find_backend(setting)
        except Exception as ex:
            print("tuneplayer: can't use player %s: %s" % (setting, ex))
            backend = None
        if backend is None:
            if setting == "auto":
                print("tuneplayer: no way to play sound found; install simpleaudio or one of %s" %
                      ", ".join([c[0] for c in PLAYER_COMMANDS]))
            return None
        self.player = _Player(backend)
        return self.player

    def play_tune(self, tune_string, source=None):
        '''play a tune from source (None for this GCS); an empty tune
        stops the one playing'''
        player = self.get_player()
        if player is None:
            return
        if tune_string == "":
            player.play(None, source)
            return
        tune = mml_tune.parse(tune_string)
        if tune.duration() <= 0:
            player.play(None, source)
            return
        player.play(self.make_request(tune), source)

    def tune_is_for_us(self, m):
        # a tune with target_system 0 is for every system, i.e. for
        # vehicles to play; only play tunes addressed to us
        if m.target_system != self.settings.source_system:
            return False
        return m.target_component in (0, self.settings.source_component)

    def mavlink_packet(self, m):
        if m.get_type() != 'PLAY_TUNE_V2':
            return
        if not self.tune_is_for_us(m):
            return
        if not self.tuneplayer_settings.enabled:
            return
        if m.format not in SUPPORTED_FORMATS:
            if m.format not in self.warned_formats:
                self.warned_formats.add(m.format)
                print("tuneplayer: ignoring tunes in unsupported format %u" % m.format)
            return
        self.tunes_received += 1
        self.last_tune = m.tune
        if self.tuneplayer_settings.verbose:
            print("tuneplayer: %u/%u: %s tune %r" %
                  (m.get_srcSystem(), m.get_srcComponent(), SUPPORTED_FORMATS[m.format], m.tune))
        self.play_tune(m.tune, (m.get_srcSystem(), m.get_srcComponent()))

    def unload(self):
        if self.player is not None:
            self.player.close()
            self.player = None


def init(mpstate):
    '''initialise module'''
    return TunePlayerModule(mpstate)
