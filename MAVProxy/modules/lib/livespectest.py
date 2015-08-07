import livespec
from math import *
import time
import random

s = livespec.LiveSpectrogram(100, .5, .1, hist_time=60)

f = 0.0
phi = 0.0
dt = .01
while True:
    # RANDOM DATA
    time.sleep(dt)
    phi += f*2*pi*dt
    f += dt*.25
    s.new_sample(sin(phi)+random.gauss(0,2), time.time())
    if f > 50:
        f = 0.0
    if not s.is_alive():
        break


s.close()
