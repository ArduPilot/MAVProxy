#!/usr/bin/env python
'''This module is modified from mavproxy_speech.py but is only intended to provide voice directions to a blind surfer.
To do this, SERVO_OUTPUT_RAW mavlink message is read every 5 seconds (0.2Hz) and distance to next waypoint from NAV_CONTROLLER_OUTPUT is announced every 10 seconds (0.1Hz)'''

import time, os
from MAVProxy.modules.lib import mp_module

class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.add_command('speech', self.cmd_speech, "text-to-speech", ['<test>'])

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
'''
    Modified code to create voice directions to surfer from MAVLINK message
    need to figure out how to set message interval
    MAV_CMD_SET_MESSAGE_INTERVAL(SERVO_OUTPUT_RAW,5000)	
    MAV_CMD_SET_MESSAGE_INTERVAL(NAV_CONTROLLER_OUTPUT,10000) 
'''

	def mavlink_packet(self, msg):
        type = msg.get_type()
        if type == "SERVO_OUTPUT_RAW":
			      if 2000=>SERVO_OUTPUT_RAW.servo1_raw>1800, self.say("hard right")
			      elif 1800>SERVO_OUTPUT_RAW.servo1_raw>1550, self.say("right")
			      elif 1550>SERVO_OUTPUT_RAW.servo1_raw>1450, self.say("on course")
			      elif 1450>SERVO_OUTPUT_RAW.servo1_raw>12000, self.say("left")
			      elif 1000=<SERVO_OUTPUT_RAW.servo1_raw<1200, self.say("hard left")
			      else self.say("error error")
		    elif type == "NAV_CONTROLLER_OUTPUT":
			      self.say(NAV_CONTROLLER_OUTPUT.wp_dist "meters to next waypoint")
		    else ''' do nothing '''

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


def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
