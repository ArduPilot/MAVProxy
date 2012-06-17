import math
import fluidsynth
import time

import threading, Queue
import sys,os

#from scipy import interpolate
#from operator import itemgetter
 
#chunk = numpy.concatenate(chunks) * 0.25


def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)



class varioSoundGen(threading.Thread):
    def __init__(self, rising_soundfont, falling_soundfont):
        threading.Thread.__init__(self)
                
        self._stop = threading.Event()

        self.key = 60.00
        self.pulsing = False
        self.outAmplitude = int(50)
        
        self.new_risingSoundfont = rising_soundfont
        self.new_fallingSoundfont = falling_soundfont
        
    
    def stop(self):
        print("sound generator thread request stop")
        self._stop.set()

    def stopped(self):
        return not self.isAlive()

    def setKey(self, key):
        self.key = key

    def setPulsing(self, pulsing):
        self.pulsing = pulsing

    def setAmplitude(self, amplitude):
        if(amplitude > 1):
            self.outAmplitude = 100
        elif(amplitude < 0):
            self.outAmplitude = 0
        else:
            self.outAmplitude = amplitude * 100
        
    def run(self):
        print("sound generator thread starting")
        self._stop.clear()

        fs = fluidsynth.Synth()
        fs.start(driver='alsa')

        self.outAmplitude = 0
        
        pulseOnTime = 0.1
        pulseOffTime = 0.2
        
        while(not self._stop.isSet() ):
      
            if(self.new_risingSoundfont != ""):
#                if getattr(self, 'sfid_rising', None) is not None:
#                    fs.program_deselect(0, self.sfid_rising, 0, 0)
                self.sfid_rising = fs.sfload(self.new_risingSoundfont)
                fs.program_select(0, self.sfid_rising, 0, 0)
                self.new_risingSoundfont = ""

            if(self.new_fallingSoundfont != ""):
 #               if getattr(self, 'sfid_falling', None) is not None:
 #                   fs.program_deselect(1, self.sfid_falling, 0, 0)
                self.sfid_falling = fs.sfload(self.new_fallingSoundfont)
                fs.program_select(1, self.sfid_falling, 0, 0)
                self.new_fallingSoundfont = ""
            
            pulsekey = int(self.key)       
            if(self.pulsing == True):
      #          if(self.pulsing == True):
                fs.noteon(0, pulsekey, int(self.outAmplitude) )
                time.sleep(0.05)
                fs.noteoff(0, pulsekey)
                time.sleep(0.2)
            else:
                fs.noteon(1, pulsekey, int(self.outAmplitude) )
                time.sleep(0.5)
                fs.noteoff(1, pulsekey)
  #          else :
  #              fs.noteon(0, 60, 30)
                        
        
        print("sound generator thread end")


class vario():
    def __init__(self, rising_soundfont, falling_soundfont):
        self.soundGen = varioSoundGen(rising_soundfont, falling_soundfont)
        self.soundGen.start()
        
        self.deadband   = 50.0
        self.maxRate    = 300.0
        self.minRate    = -300.0
        self.maxRisingKey  = 80.0
        self.minRisingKey  = 60.0
        self.maxFallingKey = 25.0
        self.minFallingKey = 55.0
        
        self.volume = 0.5;

    def __del__(self):
        self.soundGen.stop()
        
    def setRate(self, rate):
        if(rate > self.deadband):
            if(rate < self.maxRate):
                self.soundGen.setKey( ( (rate-self.deadband) * (self.maxRisingKey - self.minRisingKey) / (self.maxRate - self.deadband)) + self.minRisingKey)
                self.soundGen.setPulsing(True)
            else:
                self.soundGen.setKey(self.maxRisingKey)
                self.soundGen.setPulsing(False)
            self.soundGen.setAmplitude(self.volume)
            
        elif(rate < -self.deadband):
            if(rate < self.minRate):
                self.soundGen.setKey(self.maxFallingKey)
                self.soundGen.setPulsing(True)
                self.soundGen.setAmplitude(self.volume)
            else:
                self.soundGen.setPulsing(False)
                freq = rate+self.deadband
                freq = freq / (self.minRate + self.deadband)
                freq = freq * (self.minFallingKey - self.maxFallingKey)
                freq = self.minFallingKey - freq
                self.soundGen.setKey( freq )
                self.soundGen.setAmplitude(self.volume)
        else:
            self.soundGen.setAmplitude(0.0)
    
    
    def setVolume(self, volume):
        self.volume = volume;
        
    def setRisingSoundfont(self, SFName):
        self.soundGen.new_risingSoundfont = SFName

    def setFallingSoundfont(self, SFName):
        self.soundGen.new_fallingSoundfont = SFName

           