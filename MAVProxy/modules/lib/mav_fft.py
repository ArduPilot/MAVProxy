#!/usr/bin/env python

'''
extract ISBH and ISBD messages from AP_Logging files and produce FFT plots
'''

import numpy
import os
import pylab
import sys
import time

from pymavlink import mavutil
from MAVProxy.modules.lib.multiproc_util import MPDataLogChildTask

class MavFFT(MPDataLogChildTask):
    '''A class used to launch `mavfft_display` in a child process'''

    def __init__(self, *args, **kwargs):
        '''
        Parameters
        ----------
        mlog : DFReader
            A dataflash or telemetry log
        xlimits: MAVExplorer.XLimits
            An object capturing timestamp limits
        '''

        super(MavFFT, self).__init__(*args, **kwargs)

        # all attributes are implicitly passed to the child process 
        self.xlimits = kwargs['xlimits']

    # @override
    def child_task(self):
        '''Launch `mavfft_display`'''

        # run the fft tool
        mavfft_display(self.mlog, self.xlimits.timestamp_in_range)

def mavfft_display(mlog, timestamp_in_range):
    '''display fft for raw ACC data in logfile'''

    '''object to store data about a single FFT plot'''
    class PlotData(object):
        def __init__(self, ffth):
            self.seqno = -1
            self.fftnum = ffth.N
            self.sensor_type = ffth.type
            self.instance = ffth.instance
            self.sample_rate_hz = ffth.smp_rate
            self.multiplier = ffth.mul
            self.data = {}
            self.data["X"] = []
            self.data["Y"] = []
            self.data["Z"] = []
            self.holes = False
            self.freq = None

        def add_fftd(self, fftd):
            if fftd.N != self.fftnum:
                print("Skipping ISBD with wrong fftnum (%u vs %u)\n" % (fftd.fftnum, self.fftnum))
                return
            if self.holes:
                print("Skipping ISBD(%u) for ISBH(%u) with holes in it" % (fftd.seqno, self.fftnum))
                return
            if fftd.seqno != self.seqno+1:
                print("ISBH(%u) has holes in it" % fftd.N)
                self.holes = True
                return
            self.seqno += 1
            self.data["X"].extend(fftd.x)
            self.data["Y"].extend(fftd.y)
            self.data["Z"].extend(fftd.z)

        def prefix(self):
            if self.sensor_type == 0:
                return "Accel"
            elif self.sensor_type == 1:
                return "Gyro"
            else:
                return "?Unknown Sensor Type?"

        def tag(self):
            return str(self)

        def __str__(self):
            return "%s[%u]" % (self.prefix(), self.instance)

    print("Processing log for ISBH and ISBD messages")

    things_to_plot = []
    plotdata = None
    start_time = time.time()
    mlog.rewind()

    while True:
        m = mlog.recv_match(type=['ISBH','ISBD'])
        if m is None:
            break
        in_range = timestamp_in_range(m._timestamp)
        if in_range < 0:
            continue
        if in_range > 0:
            break
        msg_type = m.get_type()
        if msg_type == "ISBH":
            if plotdata is not None:
                # close off previous data collection
                things_to_plot.append(plotdata)
            # initialise plot-data collection object
            plotdata = PlotData(m)
            continue

        if msg_type == "ISBD":
            if plotdata is None:
                continue
            plotdata.add_fftd(m)

    if len(things_to_plot) == 0:
        print("No FFT data. Did you set INS_LOG_BAT_MASK?")
        return
    time_delta = time.time() - start_time
    print("Extracted %u fft data sets" % len(things_to_plot))

    sum_fft = {}
    freqmap = {}
    count = 0

    first_freq = None
    for thing_to_plot in things_to_plot:
        for axis in [ "X","Y","Z" ]:
            d = numpy.array(thing_to_plot.data[axis])/float(thing_to_plot.multiplier)
            if len(d) == 0:
                print("No data?!?!?!")
                continue
            
            avg = numpy.sum(d) / len(d)
            d -= avg
            d_fft = numpy.fft.rfft(d)
            if thing_to_plot.tag() not in sum_fft:
                sum_fft[thing_to_plot.tag()] = {
                    "X": 0,
                    "Y": 0,
                    "Z": 0
                }
            sum_fft[thing_to_plot.tag()][axis] = numpy.add(sum_fft[thing_to_plot.tag()][axis], d_fft)
            count += 1
            freq = numpy.fft.rfftfreq(len(d), 1.0/thing_to_plot.sample_rate_hz)
            freqmap[thing_to_plot.tag()] = freq

    for sensor in sum_fft:
        pylab.figure(str(sensor))
        for axis in [ "X","Y","Z" ]:
            pylab.plot(freqmap[sensor], numpy.abs(sum_fft[sensor][axis]/count), label=axis)
        pylab.legend(loc='upper right')
        pylab.xlabel('Hz')

    pylab.show()
