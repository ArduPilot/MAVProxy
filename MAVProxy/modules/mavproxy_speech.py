#!/usr/bin/env python
'''tune command handling'''

import time, os
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_util

class SpeechCommandSay(object):
    def __init__(self, text, priority):
        self.text = text
        self.timestamp = time.time()
        self.priority = priority

class SpeechCommandSetVoice(object):
    def __init__(self, voice):
        self.voice = voice

class SpeechCommandUnload(object):
    def __init__(self):
        pass

class SpeechCommandListVoices(object):
    def __init__(self):
        pass
    
class SpeechBackend():
    '''
    speech control backend
    '''

    def __init__(self, settings):
        self.settings = settings
        self.max_age = 5
        self.parent_pipe, self.child_pipe = multiproc.Pipe()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def send(self, command):
        '''send a command to the backend'''
        self.parent_pipe.send(command)

    def child_task(self):
        '''child process - this does the speech output'''
        mp_util.child_close_fds()
        self.pyttsx3_engine = None
        self.kill_speech_dispatcher()
        self.say_backend = None
        self.voice = ''
        for (backend_name,backend) in [("espeak",self.say_espeak),
                                       ("speechd",self.say_speechd),
                                       ("speech", self.say_speech),
                                       ("say", self.say_say),
                                       ("pyttsx3", self.say_pyttsx3)]:
            try:
                backend("")
                self.say_backend = backend
                print("Using speech backend '%s'" % backend_name)
                break
            except Exception:
                pass
        if self.say_backend is None:
            print("No speech available")

        while True:
            time.sleep(0.1)
            if self.pyttsx3_engine is not None:
                self.pyttsx3_engine.iterate()
            if self.child_pipe.poll():
                cmd = self.child_pipe.recv()
                if isinstance(cmd, SpeechCommandSay):
                    if self.say_backend is not None and time.time() - cmd.timestamp < self.max_age:
                        self.say_backend(cmd.text, cmd.priority)
                elif isinstance(cmd, SpeechCommandListVoices):
                    self.list_voices()
                elif isinstance(cmd, SpeechCommandSetVoice):
                    self.voice = cmd.voice


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
        if self.voice:
            espeak.set_voice(self.voice)
        espeak.synth(text)
        while espeak.is_playing():
            time.sleep(0.1)

    def say_speech(self, text, priority='important'):
        '''speak some text using speech module'''
        import speech
        speech.say(text)

    def say_pyttsx3(self, text, priority='important'):
        '''speak some text using pyttsx3 module'''
        if self.pyttsx3_engine is None:
            import pyttsx3
            self.pyttsx3_engine = pyttsx3.init()
            if self.voice:
                self.pyttsx3_engine.setProperty('voice', self.voice)
            self.pyttsx3_engine.startLoop(False)
            return
        if len(text) > 0:
            if self.voice:
                self.pyttsx3_engine.setProperty('voice', self.voice)
            self.pyttsx3_engine.say(text)

    def say_say(self, text, priority='important'):
        '''speak some text using macOS say command'''
        import subprocess
        subprocess.check_call(["say",text])

    def list_voices(self):
        if self.say_backend == self.say_espeak:
            from espeak import espeak
            voices = espeak.list_voices()
            vlist = [ v.name for v in voices ]
            print(vlist)
        elif self.say_backend == self.say_pyttsx3:
            vlist = [ v.name for v in self.pyttsx3_engine.getProperty('voices') ]
            print(vlist)
        else:
            print("Backend can't list voices")

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
        self.settings.set_callback(self.settings_callback)
        self.backend = SpeechBackend(self.settings)

    def settings_callback(self, setting):
        '''handle changes in settings'''
        if setting.name == "speech_voice":
            self.backend.send(SpeechCommandSetVoice(setting.value))

    def unload(self):
        '''unload module'''
        self.backend.send(SpeechCommandUnload())
        self.settings.set('speech', 0)
        if self.mpstate.functions.say == self.mpstate.functions.say:
            self.mpstate.functions.say = self.old_mpstate_say_function

    def say(self, text, priority='important'):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        self.console.writeln(text)
        if self.settings.speech:
            self.backend.send(SpeechCommandSay(text, priority))

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()
        if type == "STATUSTEXT":
            # say some statustext values
            speech_prefixes = [ "Tuning: ", "@" ]
            for s in speech_prefixes:
                if msg.text.startswith(s):
                    self.say(msg.text[len(s):])

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
            self.backend.send(SpeechCommandListVoices())

def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
