"""
  MAVProxy vario module
  Uses fluidsynth and soundfonts driving alsa
"""

import re, os, sys, threading, time
import pygame, numpy

mpstate = None

global_sample_rate = 44100


class status_soundgen(object):
    def __init__(self, mpstate):
        self.mpstate = mpstate
        
        self.unload = threading.Event()
        self.unload.clear()
        
        self.timeout_time = 0
        
        self.play_link_down = threading.Event()
        self.play_link_lost = threading.Event()
        self.play_link_ok = threading.Event()
        
        self.last_play_time = time.time()

        self.amplitude = 0.5
        self.oldamplitude = 0
  
        pygame.mixer.pre_init(frequency=global_sample_rate,size=-16,channels=1)
        pygame.init()
  
        self.sounds_thread = threading.Thread(target=self.announce_sounds_app)
        self.sounds_thread.daemon = True
        self.sounds_thread.start()
                
    
    def sine_array(self, hz, peak, cycles=1):
        length = cycles * global_sample_rate / float(hz)
        omega = numpy.pi * 2 * cycles / length
        xvalues = numpy.arange(int(length)) * omega 
        return (peak * numpy.sin(xvalues))

    def makesoundarray(self, frequency, duration, amplitude):
        cycles = int(duration * frequency / 1000)
        if(self.amplitude > 1.0):
            self.amplitude = 1.0
        sarry = self.sine_array(frequency, amplitude * 2**15, cycles )
        sarry = sarry.astype(numpy.int16)
        return sarry

#    def playsound(self, frequency, duration, amplitude):
#        cycles = duration * frequency / 1000
#        if(self.amplitude > 1.0):
#            self.amplitude = 1.0
#        a = sine_array(1000, amplitude * 2**15, int(cycles) )
#        a = a.astype(numpy.int16)
#        sound = pygame.sndarray.make_sound(a)
#        sound.play()

    def makesounds(self):
        sarry = self.makesoundarray(1000, 20, self.amplitude)
        sarry2 = self.makesoundarray(500, 20, self.amplitude)
        sarry = numpy.concatenate([sarry, sarry2])
        self.link_lost_sound = pygame.sndarray.make_sound(sarry)

        sarry = self.makesoundarray(700, 50, self.amplitude)
        sarry2 = self.makesoundarray(1000, 50, self.amplitude)
        sarry = numpy.concatenate([sarry, sarry2])
        self.link_ok_sound = pygame.sndarray.make_sound(sarry)
            
    def announce_sounds_app(self):
        while(not self.unload.is_set() and (not mpstate.status.exit) ):
            
            if(self.amplitude != self.oldamplitude):
                self.oldamplitude = self.amplitude
                self.makesounds()
                
            if( self.play_link_lost.is_set() ):     #mpstate.status_sounds
                self.last_play_time = time.time()
                self.play_link_lost.clear()
                self.link_lost_sound.play()
                
            if( self.play_link_ok.is_set()):
                self.last_play_time = time.time()
                self.play_link_ok.clear()
                self.link_ok_sound.play()
            time.sleep(0.2)
#            playsound(1500,200)
 
        
def name():
    '''return module name'''
    return "vario"

def description():
    '''return module description'''
    return "voice and sound announcement"

def cmd_status_sounds(args):
    '''announce command'''
    if(len(args) == 2):
        if(args[0] == "amplitude"):
            mpstate.status_sound.amplitude = float(args[1])
            return 1
    return 0

    

#===============================================================================
#    state = mpstate.graph_state
# 
#    if len(args) == 0:
#        # list current graphs
#        for i in range(len(state.graphs)):
#            print("Graph %u: %s" % (i, state.graphs[i].fields))
#        return
# 
#    # start a new graph
#    state.graphs.append(Graph(args[:]))
#===============================================================================


def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate

    mpstate.status_sound = status_soundgen(mpstate)
    
    mpstate.command_map['status_sounds'] = (cmd_status_sounds, "status sounds settings adjust")


def unload():
    '''unload module'''
    mpstate.status_sound.unload.set()
        
def idle_task():
    now = time.time()
    if(now > mpstate.status_sound.last_play_time + 1.0):
#        if(mpstate.mav_master.mavserial.linkerror == True):
#            mpstate.status_sound.play_link_lost.set()
#        else:
        if(mpstate.status.heartbeat_error == 1):
            mpstate.status_sound.play_link_lost.set()
    
