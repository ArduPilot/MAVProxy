#!/usr/bin/env python
'''tune command handling'''

import time, os
from MAVProxy.modules.lib import mp_module

class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.add_command('speech', self.cmd_speech, "text-to-speech", ['<say|list_voices>'])

        self.old_mpstate_say_function = self.mpstate.functions.say
        self.mpstate.functions.say = self.say
        try:
            self.settings.set('speech', 1)
        except AttributeError:
            self.settings.append(('speech', int, 1))
        try:
            self.settings.set('speech_voice', '')
        except AttributeError:
            self.settings.append(('speech_voice', str, ''))
        self.kill_speech_dispatcher()
        for (backend_name,backend) in [("speechd",self.say_speechd), ("espeak",self.say_espeak), ("speech", self.say_speech), ("say", self.say_say)]:
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

    def say_speechd(self, text, priority='important'):
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

    def say_espeak(self, text, priority='important'):
        '''speak some text using espeak'''
        from espeak import espeak
        if self.settings.speech_voice:
            espeak.set_voice(self.settings.speech_voice)
        espeak.synth(text)

    def say_speech(self, text, priority='important'):
        '''speak some text using speech module'''
        import speech
        speech.say(text)

    def say_say(self, text, priority='important'):
        '''speak some text using macOS say command'''
        import subprocess
        subprocess.check_call(["say",text])

    def say(self, text, priority='important'):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        self.console.writeln(text)
        if self.settings.speech and self.say_backend is not None:
            self.say_backend(text, priority=priority)

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()
        if type == "STATUSTEXT":
            # say some statustext values
            if msg.text.startswith("Tuning: "):
                self.say(msg.text[8:])

    def list_voices(self):
        from espeak import espeak
        voices = espeak.list_voices()
        vlist = [ v.name for v in voices ]
        print(vlist)

    def cmd_speech(self, args):
        '''speech commands'''
        usage = "usage: speech <say>"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == "say":
            if len(args) < 2:
                print("usage: speech say <text to say>")
                return
            self.say(" ".join(args[1::]))
        if args[0] == "list_voices":
            self.list_voices()


def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
