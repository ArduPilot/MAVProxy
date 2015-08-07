"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Copyright 2015 3D Robotics, Inc

Authors:
- Tim Ryan
- Jonathan Challinger
"""

import time
from math import *
import os
from multiprocessing import Process, Queue
import matplotlib
import numpy as np
from scipy import signal
import random
from collections import deque
import matplotlib.pyplot as plt
from scipy.fftpack import fftfreq
import sys
from matplotlib.widgets import Slider
from scipy.interpolate import interp1d

class SpecPlotter:
    def __init__(self,q,freqs,x_max,hist_time,time_res,maxscale):
        self.HORIZONTAL_PIXELS=len(freqs)+1
        self.q=q
        self.freqs = freqs
        self.x_max=x_max
        self.hist_time=hist_time
        self.time_res=time_res
        self.maxscale = maxscale

        self.hist_segments = floor(self.hist_time/self.time_res)
        plt.ion()
        self.figure, (self.ax, self.ax2) = plt.subplots(2)
        self.figure.canvas.mpl_connect('close_event', self.exit_evt)
        self.Z = np.zeros((self.hist_segments,self.HORIZONTAL_PIXELS))
        self.highest_peak = 0.0 if maxscale is None else maxscale
        self.scale = self.highest_peak
        self.scale_slider_touched = False
        plt.subplots_adjust(bottom=0.1)
        self.ax_scale_slider = plt.axes([0.15,0.03,0.7,0.03])
        self.scale_slider = Slider(self.ax_scale_slider,'Scale',0,self.highest_peak,self.highest_peak)
        self.scale_slider.on_changed(self.scale_slider_changed)
        self.X = np.linspace(self.freqs[0],self.freqs[-1],self.HORIZONTAL_PIXELS)
        self.row = np.zeros(len(self.freqs))

    def update(self):
        while True:
            while not self.q.empty():
                self.row = self.q.get()
                self.Z = np.vstack((self.Z,interp1d(self.freqs,self.row,kind='linear')(self.X)))
                if self.Z.shape[0] > self.hist_segments:
                    self.Z = np.delete(self.Z, 0, axis=0)
                peak_height = np.abs(self.row).max()
                if self.highest_peak < peak_height:
                    self.highest_peak = peak_height
                    if self.maxscale is None:
                        if self.scale == self.scale_slider.valmax:
                            self.scale = self.highest_peak
                        self.ax_scale_slider.set_xlim(0,self.highest_peak)
                        self.scale_slider.valmax = self.highest_peak
                        self.scale_slider.set_val(self.scale)


            self.ax.clear()
            self.ax.imshow(self.Z, extent=(0,self.x_max,0,self.hist_time), cmap='Greys', vmin=0, vmax=self.scale, interpolation='none', aspect='auto')
            self.ax.set_title('Spectrogram')
            self.ax.set_xlabel('Hz')

            self.ax2.clear()
            self.ax2.plot(self.freqs, self.row)

            self.ax2.axis([0,self.x_max,0,self.scale])

            # Draw canvas
            plt.draw()
            plt.pause(0.001)

    def exit_evt(self,evt):
        os._exit(0)

    def scale_slider_changed(self, v):
        self.scale = v


def graph_process(q,n_freqs,x_max,hist_time,time_res,maxscale):
    p = SpecPlotter(q,n_freqs,x_max,hist_time,time_res,maxscale)
    while True:
        p.update()

class LiveSpectrogram:
    def __init__(self, sample_freq, freq_res, time_res=1, hist_time=10.0, maxscale=None):
        self.sample_freq = sample_freq
        self.samples_between_fft = max(round(time_res*sample_freq),1)
        time_res = self.samples_between_fft/sample_freq
        n_freqs=int(sample_freq/(freq_res*2))
        self.segment_length = int(n_freqs*2-2)
        freqs = np.abs(np.fft.fftfreq(self.segment_length)*self.sample_freq)[0:n_freqs]
        self.samples = deque()
        self.times = deque()

        self.q = Queue(2048)
        p = Process(target=graph_process, args=(self.q,freqs,sample_freq/2,hist_time,time_res,maxscale))
        p.daemon = True
        p.start()
        self.p = p

        self.samples_since_fft = 0
        self.integ = 0.0

    def is_alive(self):
        return self.p.is_alive()

    def new_sample(self, s, t=None):
        if not self.is_alive():
            self.close()
        if t is None:
            t = time.time()
        self.samples.append(s)
        self.times.append(t)
        self.samples_since_fft += 1

        if len(self.samples) > max(self.segment_length,self.samples_between_fft):
            self.samples.popleft()
            self.times.popleft()

        if len(self.samples) >= self.segment_length and self.samples_since_fft >= self.samples_between_fft:
            _,y = signal.welch(self.samples, nperseg=self.segment_length, scaling='density', noverlap=0)
            self.q.put(y)
            self.samples_since_fft = 0

    def close(self):
        self.p.terminate()
