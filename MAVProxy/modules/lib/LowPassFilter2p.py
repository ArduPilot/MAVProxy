#!/usr/bin/env python

from math import *

class LowPassFilter2p:
    def __init__(self, sample_freq, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.sample_freq = sample_freq
        self.a1 = 0.0
        self.a2 = 0.0
        self.b0 = 0.0
        self.b1 = 0.0
        self.b2 = 0.0
        self.delay_element_1 = None
        self.delay_element_2 = None
        self.set_cutoff_frequency(sample_freq, cutoff_freq)

    def set_cutoff_frequency(self, sample_freq, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        if self.cutoff_freq <= 0.0:
            return
        fr = self.sample_freq/self.cutoff_freq
        ohm = tan(pi/fr)
        c = 1.0+2.0*cos(pi/4.0)*ohm + ohm*ohm
        self.b0 = ohm*ohm/c
        self.b1 = 2.0*self.b0
        self.b2 = self.b0
        self.a1 = 2.0*(ohm*ohm-1.0)/c
        self.a2 = (1.0-2.0*cos(pi/4.0)*ohm+ohm*ohm)/c

    def apply(self, sample):
        if self.delay_element_1 is None:
            self.delay_element_1 = sample
            self.delay_element_2 = sample
        delay_element_0 = sample - (self.delay_element_1 * self.a1) - (self.delay_element_2 * self.a2)
        output = (delay_element_0 * self.b0) + (self.delay_element_1 * self.b1) + (self.delay_element_2 * self.b2)
    
        self.delay_element_2 = self.delay_element_1
        self.delay_element_1 = delay_element_0

        return output

if __name__ == "__main__":
    from pymavlink.rotmat import Vector3
    filter = LowPassFilter2p(1000.0, 98.0)

    v2 = filter.apply(Vector3(1,2,3))
    print(v2)
