#!/usr/bin/env python
'''

Speech Engine for audio announcements of MAVLink messages during flight.

Suggested usage: Add initialization commands to .mavinit.scr

Example commands:
    module load mavproxy_speech
    speech add ALTITUDE altitude_relative -r 30 -n "altitude"
    speech add VFR_HUD airspeed -r 10 -f %0.0f -s 1.0 -n "airspeed"

'''

import time, os, math
from MAVProxy.modules.lib import mp_module

class SpeechAnnounce(object):
    '''
    SpeechAnnouncement objects encapsulate all functionality and responsibility
    of generating speech for a single mavlink message entry.

    The default SpeechAnnounce module will provide mavlink-to-text and rate-limiting
    capabilities.
    '''

    def __init__(self, msg_type, entry_name, sayEveryS=30.0, sayAs=None, scale=1, formatstr="%0.0f"):
        self.msg_type = msg_type
        self.entry_name = entry_name
        self.sayEveryS = sayEveryS
        self.sayAs = sayAs
        self.scale = scale
        self.formatstr = formatstr

        self.enabled = True
        self.lastTime = 0

    def match(self, msg):
        '''Returns true if this announce is ready to handle the given message'''
        return (msg.get_type() == self.msg_type)

    def dispatch(self, msg, backend):
        '''Dispatch this announcement IF it can handle the given message'''
        if self.enabled and self.match(msg):
            self.maybe_say(msg, backend)

    def maybe_say(self, msg, backend):
        '''Calls backend with this announce for the given message IF the announce passes internal checks'''
        now = time.time()
        if (now - self.lastTime > self.sayEveryS):
            self.lastTime = now
            value = getattr(msg, self.entry_name, 0)
            backend(self.to_speech_text(value))

    def to_speech_text(self, value):
        '''Returns the text announced before the value'''
        return "%s %s" % (self.get_speech_name(), self.convert(value))

    def get_speech_name(self):
        if self.sayAs:
            return self.sayAs
        else:
            return self.entry_name

    def convert(self, value):
        '''Returns the announce text representation of the value'''
        return self.formatstr % value #(value*self.scale)

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
            return "%s.%s as \"%s\" every %0.0f seconds, scaling by %0.4f and formatting as %s" % (
                self.msg_type, 
                self.entry_name, 
                self.get_speech_name(), 
                self.sayEveryS,
                self.scale,
                self.formatstr)

class ConditionalSpeechAnnounce(SpeechAnnounce):
    '''
    A SpeechAnnounce subclass that provides rate-limited announcements
    only when a mavlink value is outside an expected range.
    '''

    def __init__(self, msg_type, entry_name, sayEveryS=5.0, sayAs=None, scale=1, formatstr="%0.0f", range=[float("inf"), float("inf")]):
        super(ConditionalSpeechAnnounce, self).__init__(msg_type, entry_name, sayEveryS=sayEveryS, sayAs=sayAs, scale=scale, formatstr=formatstr)
        self.range = range

    def maybe_say(self, msg, backend):
        value = getattr(msg, self.entry_name, None)
        if value < self.range[0] or value > self.range[1]:
            super(ConditionalSpeechAnnounce, self).maybe_say(msg, backend)

    def to_speech_text(self, value):
        if value < self.range[0]:
            return "%s below minimum! Expected minimum %s, current value %s." % (
                self.get_speech_name(), self.convert(self.range[0]), self.convert(value))
        if value > self.range[1]:
            return "%s above maximum! Expected maximum %s, current value %s." % (
                self.get_speech_name(), self.convert(self.range[1]), self.convert(value))  

class OnChangeSpeechAnnounce(SpeechAnnounce):
    '''
    A SpeechAnnounce subclass that provides rate-limited announcements
    only when a mavlink value changes.
    '''

    def __init__(self, msg_type, entry_name, sayEveryS=5.0, sayAs=None, scale=1, formatstr="%0.0f"):
        super(OnChangeSpeechAnnounce, self).__init__(msg_type, entry_name, sayEveryS=sayEveryS, sayAs=sayAs, scale=scale, formatstr=formatstr)
        self.prev_value = float("inf")

    def maybe_say(self, msg, backend):
        new_value = getattr(msg, self.entry_name, 0)
        if self.prev_value == float("inf"):
            self.prev_value = new_value
        if self.prev_value != new_value:
            super(OnChangeSpeechAnnounce, self).maybe_say(msg, backend)

    def to_speech_text(self, value):
        ret = "%s changing! Previous value is %s. New value is %s" % (self.get_speech_name(), self.convert(self.prev_value), self.convert(value))
        self.prev_value = value
        return ret


class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.add_command('speech', self.cmd_speech, "text-to-speech", ['<test>'])

        self.msgs_to_announce = []

        self.old_mpstate_say_function = self.mpstate.functions.say
        self.mpstate.functions.say = self.print_and_say
        self.say_all_text = True
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
                announcements.dispatch(msg, self.say)

    def print_and_say(self, text, priority='important'):
        self.console.writeln(text)
        if self.say_all_text:
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
        usage = "usage: speech <say|list|enable|disable|add|remove|every>"
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
                if (args[1] == "TEXT"):
                    self.say_all_text = True
                else:
                    try:
                        self.msgs_to_announce[int(args[1])].enable()
                        print "Enabling", self.msgs_to_announce[int(args[1])]
                    except Exception:
                        print("usage: speech enable (<index>|TEXT)")
        elif args[0] == "disable":
            if len(args) < 2:
                self.settings.set('speech', 0)
            else:
                if (args[1] == "TEXT"):
                    self.say_all_text = False
                else:
                    try:
                        print "Disabling", self.msgs_to_announce[int(args[1])]
                        self.msgs_to_announce[int(args[1])].disable()
                    except Exception:
                        print("usage: speech disable (<index>|TEXT)")
        elif args[0] == "list":
            print "speech module will announce:"
            for idx, announcement in enumerate(self.msgs_to_announce):
                print "#%d: %s" % (idx, announcement)
        elif args[0] == "add":
            usage = """usage: speech 
add <msg_type> <msg_entry> (-r <say_every_seconds>) (-n <say_as>) (-s <scale>) (-f \"<formatstr>\")
add <msg_type> <msg_entry> -t CONDITIONAL (--max <val>) (--min <val>) (-r <say_every_seconds>) (-n <say_as>) (-s <scale>) (-f \"<formatstr>\")
add <msg_type> <msg_entry> -t ONCHANGE (-r <say_every_seconds>) (-n <say_as>) (-s <scale>) (-f \"<formatstr>\")"""
            if len(args) < 3:
                print usage
            else:
                try:
                    msg_type = args[1]
                    entry_name = args[2]
                    say_every = 30.0
                    say_as = None
                    scale = 1
                    formatstr = "%0.0f"
                    what_type = "REGULAR"
                    maxval = float("inf")
                    minval = float("inf")
                    if len(args) > 3:
                        optargs = args[3::]
                        while len(optargs) > 0:
                            if optargs[0] == "-r":
                                say_every = float(optargs[1])
                            if optargs[0] == "-n":
                                say_as = optargs[1]
                            if optargs[0] == "-s":
                                say_ay = float(optargs[1])
                            if optargs[0] == "-f":
                                formatstr = optargs[1]
                            if optargs[0] == "-t":
                                what_type = optargs[1]
                            if optargs[0] == "--max":
                                maxval = float(optargs[1])
                            if optargs[0] == "--min":
                                minval = float(optargs[1])
                            optargs = optargs[2::]
                    if what_type == "REGULAR":
                        self.msgs_to_announce.append(SpeechAnnounce(msg_type, entry_name, say_every, say_as, scale, formatstr))
                    elif what_type == "CONDITIONAL":
                        say_every = 5.0
                        self.msgs_to_announce.append(ConditionalSpeechAnnounce(msg_type, entry_name, say_every, say_as, scale, formatstr, range=[minval, maxval]))
                    elif what_type == "ONCHANGE":
                        say_every = 5.0
                        self.msgs_to_announce.append(OnChangeSpeechAnnounce(msg_type, entry_name, say_every, say_as, scale, formatstr))
                    else:
                        print usage
                        return
                except Exception as e:
                    print usage

        elif args[0] == "remove":
            try:
                del self.msgs_to_announce[int(args[1])]
            except Exception:
                print "usage: speech remove <index>"
        elif args[0] == "every":
            try:
                self.msgs_to_announce[int(args[1])].say_every(float(args[2]))
                print self.msgs_to_announce[int(args[1])]
            except Exception:
                print("usage: speech every <index> <seconds>")

    #msg_type, entry_name, sayEveryS=30.0, sayAs=None, enabled=True, scale=1, formatstr="%0.0f"

        else:
            print(usage)
            return


def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
