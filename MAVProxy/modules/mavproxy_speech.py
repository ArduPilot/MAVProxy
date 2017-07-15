#!/usr/bin/env python
'''tune command handling'''

import time, os, math
from MAVProxy.modules.lib import mp_module

class SpeechAnnounce(object):

    def __init__(self, msg_type, entry_name, sayEveryS=30.0, sayAs=None, enabled=True):
        self.msg_type = msg_type
        self.entry_name = entry_name
        self.sayEveryS = sayEveryS
        self.sayAs = sayAs

        self.enabled = True
        self.lastTime = 0

    def match(self, msg):
        return self.enabled and (msg.get_type() == self.msg_type)

    def maybe_say(self, msg, backend):
        if not self.match(msg):
            return

        now = time.time()
        if (now - self.lastTime > self.sayEveryS):
            self.lastTime = now
            value = getattr(msg, self.entry_name, 0)
            backend("%s %s" % (self.preface(), self.convert(value)))

    def preface(self):
        '''Returns the text announced before the value'''
        if self.sayAs:
            return self.sayAs
        else:
            return self.entry_name

    def convert(self, value):
        '''Returns the announce text representation of the value'''
        return value

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def say_every(self, sayEveryS):
        self.sayEveryS = sayEveryS

    def __str__(self):
        if not self.enabled:
            return "[DISABLED] %s.%s" % (self.msg_type, self.entry_name)
        else:
            return "%s.%s as \"%s\" every %0.0f seconds" % (
                self.msg_type, 
                self.entry_name, 
                self.preface(), 
                self.sayEveryS)

# class ConditionalSpeechAnnounce(SpeechAnnounce):
#
#     def __init__(self, msg_type, entry_name, sayEveryS=5.0, sayAs=None, condition=)

# class OnChangeSpeechAnnounce(SpeechAnnounce):
#
#     def __init__(self, msg_type, entry_name, sayEveryS=5.0, sayAs=None, condition=)


class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.add_command('speech', self.cmd_speech, "text-to-speech", ['<test>'])

        self.msgs_to_announce = [
            SpeechAnnounce('ALTITUDE', 'altitude_relative', sayAs="altitude", sayEveryS=30.0),
            SpeechAnnounce('VFR_HUD',  'airspeed', sayEveryS=10.0),
            SpeechAnnounce('VFR_HUD',  'groundSpeed'),
        ]

        self.old_mpstate_say_function = self.mpstate.functions.say
        self.mpstate.functions.say = self.print_and_say
        try:
            self.settings.get('speech')
        except AttributeError:
            self.settings.append(('speech', int, 1))
        try:
            self.settings.set('speech_voice', '')
        except AttributeError:
            self.settings.append(('speech_voice', str, ''))
        self.kill_speech_dispatcher()
        for (backend_name,backend) in [("speechd",self.say_using_speechd), ("espeak",self.say_using_espeak), ("speech", self.say_using_speech), ("say", self.say_using_say)]:
            try:
                backend("")
                self.say_backend = backend
                print("Using speech backend '%s'" % backend_name)
                return
            except Exception:
                pass
        self.say_backend = None
        print("No speech available")

    def kill_speech_dispatcher(self):
        '''kill speech dispatcher processs'''
        if not 'HOME' in os.environ:
            return
        pidpath = os.path.join(os.environ['HOME'], '.speech-dispatcher',
                               'pid', 'speech-dispatcher.pid')
        if os.path.exists(pidpath):
            try:
                import signal
                pid = int(open(pidpath).read())
                if pid > 1 and os.kill(pid, 0) is None:
                    print("Killing speech dispatcher pid %u" % pid)
                    os.kill(pid, signal.SIGINT)
                    time.sleep(1)
            except Exception as e:
                pass


    def unload(self):
        '''unload module'''
        self.settings.set('speech', 0)
        if self.mpstate.functions.say == self.mpstate.functions.say:
            self.mpstate.functions.say = self.old_mpstate_say_function
        self.kill_speech_dispatcher()

    #
    # Different speech backends.
    #

    def say_using_speechd(self, text, priority='important'):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        import speechd
        self.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
        self.speech.set_output_module('festival')
        self.speech.set_language('en')
        self.speech.set_priority(priority)
        self.speech.set_punctuation(speechd.PunctuationMode.SOME)
        self.speech.speak(text)
        self.speech.close()

    def say_using_espeak(self, text, priority='important'):
        '''speak some text using espeak'''
        from espeak import espeak
        if self.settings.speech_voice:
            espeak.set_voice(self.settings.speech_voice)
        espeak.synth(text)

    def say_using_speech(self, text, priority='important'):
        '''speak some text using speech module'''
        import speech
        speech.say(text)

    def say_using_say(self, text, priority='important'):
        '''speak some text using macOS say command'''
        import subprocess
        subprocess.check_call(["say",text])

    #
    # External Interface
    #

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()
        if type == "STATUSTEXT":
            # say some statustext values
            if msg.text.startswith("Tuning: "):
                self.say(msg.text[8:])
        else:
            for announcements in self.msgs_to_announce:
                announcements.maybe_say(msg, self.say)

    def print_and_say(self, text, priority='important'):
        self.console.writeln(text)
        self.say(text, priority)

    def say(self, text, priority='important'):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        if self.settings.speech and self.say_backend is not None:
            self.say_backend(text, priority=priority)

    #
    # Command line configuration commands.
    # NOTE: Speech can be enabled or disabled and changed from main mpstate settings.
    #
    def cmd_speech(self, args):
        '''speech commands'''
        usage = "usage: speech <say|list|enable|disable|every>"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == "say":
            if len(args) < 2:
                print("usage: speech say <text to say>")
                return
            self.say(" ".join(args[1::]))
        elif args[0] == "enable":
            if len(args) < 2:
                self.settings.set('speech', 1)
            else:
                try:
                    self.msgs_to_announce[int(args[1])].enable()
                    print "Enabling", self.msgs_to_announce[int(args[1])]
                except Exception:
                    print("usage: speech enable (<index>)")
        elif args[0] == "disable":
            if len(args) < 2:
                self.settings.set('speech', 0)
            else:
                try:
                    print "Disabling", self.msgs_to_announce[int(args[1])]
                    self.msgs_to_announce[int(args[1])].disable()
                except Exception:
                    print("usage: speech disable (<index>)")
        elif args[0] == "list":
            print "speech module will announce:"
            for idx, announcement in enumerate(self.msgs_to_announce):
                print "#%d: %s" % (idx, announcement)
        elif args[0] == "every":
            try:
                self.msgs_to_announce[int(args[1])].say_every(float(args[2]))
                print self.msgs_to_announce[int(args[1])]
            except Exception:
                print("usage: speech every <index> <seconds>")

        else:
            print(usage)
            return


def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
