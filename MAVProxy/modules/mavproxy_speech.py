#!/usr/bin/env python
'''tune command handling'''

import time, os

def name():
    '''return module name'''
    return "speech"

def description():
    '''return module description'''
    return "speech output handling"

def kill_speech_dispatcher():
    '''kill speech dispatcher processs'''
    pidpath = os.path.join(os.environ['HOME'], '.speech-dispatcher', 'pid', 'speech-dispatcher.pid')
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

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.functions.say = say
    mpstate.settings.append(('speech', int, 1))
    kill_speech_dispatcher()
    say('')

def unload():
    '''unload module'''
    kill_speech_dispatcher()

def say(text, priority='important'):
    '''speak some text'''
    ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
    mpstate.console.writeln(text)
    if mpstate.settings.speech:
        import speechd
        mpstate.status.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
        mpstate.status.speech.set_output_module('festival')
        mpstate.status.speech.set_language('en')
        mpstate.status.speech.set_priority(priority)
        mpstate.status.speech.set_punctuation(speechd.PunctuationMode.SOME)
        mpstate.status.speech.speak(text)
        mpstate.status.speech.close()
