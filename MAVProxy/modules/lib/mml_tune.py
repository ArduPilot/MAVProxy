'''
Parse and synthesise the tune strings played on ArduPilot's buzzer and
carried in PLAY_TUNE and PLAY_TUNE_V2 messages.

The parser follows ArduPilot's libraries/AP_Notify/MMLPlayer.cpp so that
a tune sounds the same here as it does on the vehicle.  That includes
ArduPilot's quirks:
 - numbers are held in 8 bits, so "T300" is a tempo of 44
 - octave 4's A is 1760Hz
 - "MB" repeats the whole tune rather than playing it in the background
 - playback stops at the first thing which is not understood

Only the Python standard library is used.

AP_FLAKE8_CLEAN
'''

import io
import math
import struct
import wave

# ArduPilot's note numbers run from 1 to 84
MAX_NOTE = 84

_WHITESPACE = ' \t\n\v\f\r'
_DIGITS = '0123456789'
# semitones above C of each of the notes A to G
_NOTE_TAB = [9, 11, 0, 2, 4, 5, 7]


class Tone(object):
    '''one note or rest.  frequency is 0 for a rest'''

    def __init__(self, frequency, duration, volume):
        self.frequency = frequency  # Hz
        self.duration = duration    # seconds
        self.volume = volume        # 0 to 1

    def __repr__(self):
        return "Tone(%.2f, %.4f, %.3f)" % (self.frequency, self.duration, self.volume)


class Tune(object):
    '''a parsed tune.  repeat is True if the tune plays until stopped'''

    def __init__(self, tones, repeat):
        self.tones = tones
        self.repeat = repeat

    def duration(self):
        '''length of one play-through of the tune in seconds'''
        return sum([t.duration for t in self.tones])


class _Stop(Exception):
    '''playback stops here'''
    pass


class _Parser(object):
    MODE_NORMAL = 0
    MODE_LEGATO = 1
    MODE_STACCATO = 2

    def __init__(self, string):
        # tunes arrive as C strings; anything after a NUL is ignored
        self.string = string.split('\0', 1)[0]
        self.next = 0
        self.tempo = 120
        self.default_note_length = 4
        self.note_mode = self.MODE_NORMAL
        self.octave = 4
        self.volume = 255
        self.repeat = False
        self.tones = []

    def next_char(self):
        while self.next < len(self.string) and self.string[self.next] in _WHITESPACE:
            self.next += 1
        if self.next >= len(self.string):
            return '\0'
        c = self.string[self.next]
        if 'a' <= c <= 'z':
            c = c.upper()
        return c

    def next_number(self):
        ret = 0
        while True:
            c = self.next_char()
            if c == '\0' or c not in _DIGITS:
                return ret
            # MMLPlayer keeps these in a uint8_t
            ret = (ret * 10 + ord(c) - ord('0')) & 0xFF
            self.next += 1

    def next_dots(self):
        ret = 0
        while self.next_char() == '.':
            ret += 1
            self.next += 1
        return ret

    def rest_duration(self, rest_length, dots):
        whole_note_period = 240.0 / self.tempo
        if rest_length == 0:
            rest_length = 1
        rest_period = whole_note_period / rest_length
        dot_extension = rest_period * 0.5
        while dots > 0:
            rest_period += dot_extension
            dot_extension *= 0.5
            dots -= 1
        return rest_period

    def silence(self, duration):
        self.tones.append(Tone(0, duration, 0))

    def parse(self):
        try:
            while self.next_action():
                pass
        except _Stop:
            # a stopped tune is not repeated
            self.repeat = False
        return Tune(self.tones, self.repeat)

    def next_action(self):
        '''add the next note or rest to self.tones.  Returns False at the
        end of the tune; raises _Stop where MMLPlayer would stop'''
        note = 0
        note_length = 0
        while note == 0:
            c = self.next_char()
            if c == '\0':
                return False
            self.next += 1

            if c == 'V':
                self.volume = self.next_number()
            elif c == 'L':
                self.default_note_length = self.next_number()
                if self.default_note_length == 0:
                    raise _Stop()
            elif c == 'O':
                self.octave = min(self.next_number(), 6)
            elif c == '<':
                if self.octave > 0:
                    self.octave -= 1
            elif c == '>':
                if self.octave < 6:
                    self.octave += 1
            elif c == 'M':
                c = self.next_char()
                if c == '\0':
                    raise _Stop()
                self.next += 1
                if c == 'N':
                    self.note_mode = self.MODE_NORMAL
                elif c == 'L':
                    self.note_mode = self.MODE_LEGATO
                elif c == 'S':
                    self.note_mode = self.MODE_STACCATO
                elif c == 'F':
                    self.repeat = False
                elif c == 'B':
                    self.repeat = True
                else:
                    raise _Stop()
            elif c in 'RP':
                num = self.next_number()
                dots = self.next_dots()
                self.silence(self.rest_duration(num, dots))
                return True
            elif c == 'T':
                self.tempo = self.next_number()
                if self.tempo < 32:
                    raise _Stop()
            elif c == 'N':
                note = self.next_number()
                note_length = self.default_note_length
                if note > MAX_NOTE:
                    raise _Stop()
                if note == 0:
                    num = self.next_number()
                    dots = self.next_dots()
                    self.silence(self.rest_duration(num, dots))
                    return True
            elif c in 'ABCDEFG':
                note = _NOTE_TAB[ord(c) - ord('A')] + self.octave * 12 + 1
                c = self.next_char()
                if c in '#+':
                    if note < MAX_NOTE:
                        note += 1
                    self.next += 1
                elif c == '-':
                    if note > 1:
                        note -= 1
                    self.next += 1
                note_length = self.next_number()
                if note_length == 0:
                    note_length = self.default_note_length
            else:
                raise _Stop()

        if self.tempo == 0 or note_length == 0:
            raise _Stop()

        note_period = 240.0 / self.tempo / note_length
        if self.note_mode == self.MODE_NORMAL:
            silence_duration = note_period / 8
        elif self.note_mode == self.MODE_STACCATO:
            silence_duration = note_period / 4
        else:
            silence_duration = 0
        note_period -= silence_duration

        dot_extension = note_period * 0.5
        dots = self.next_dots()
        while dots > 0:
            note_period += dot_extension
            dot_extension *= 0.5
            dots -= 1

        frequency = 880.0 * math.exp(math.log(2.0) * (note - 46) / 12.0)
        frequency = min(max(frequency, 10), 22000)
        volume = min(max(self.volume / 255.0, 0), 1)

        self.tones.append(Tone(frequency, note_period, volume))
        if silence_duration > 0:
            self.silence(silence_duration)
        return True


def parse(string):
    '''parse a tune string, returning a Tune'''
    return _Parser(string).parse()


def synthesise(tune, sample_rate=22050, volume=0.5, waveform='square'):
    '''return one play-through of tune as 16-bit mono PCM samples.
    volume is 0 to 1; waveform is "square" or "sine"'''
    out = bytearray()
    fade_samples = max(1, int(0.003 * sample_rate))  # avoid clicks
    for tone in tune.tones:
        count = int(round(tone.duration * sample_rate))
        if count <= 0:
            continue
        amplitude = 32767 * min(max(volume, 0), 1) * tone.volume
        if waveform == 'square':
            # square waves sound much louder than sine waves
            amplitude *= 0.3
        if tone.frequency <= 0 or amplitude <= 0:
            out += bytes(2 * count)
            continue
        step = tone.frequency / sample_rate
        fade = min(fade_samples, count // 2)
        samples = []
        for i in range(count):
            phase = (i * step) % 1.0
            if waveform == 'sine':
                value = math.sin(2 * math.pi * phase)
            else:
                value = 1.0 if phase < 0.5 else -1.0
            gain = 1.0
            if i < fade:
                gain = i / float(fade)
            elif i >= count - fade:
                gain = (count - 1 - i) / float(fade)
            samples.append(int(value * amplitude * gain))
        out += struct.pack('<%uh' % count, *samples)
    return bytes(out)


def to_wav(pcm, sample_rate=22050):
    '''wrap 16-bit mono PCM samples in a WAV file'''
    buf = io.BytesIO()
    w = wave.open(buf, 'wb')
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(sample_rate)
    w.writeframes(pcm)
    w.close()
    return buf.getvalue()
