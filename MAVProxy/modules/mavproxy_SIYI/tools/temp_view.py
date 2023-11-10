#!/usr/bin/env python3

import sys
import numpy as np
import cv2
import time
from MAVProxy.modules.lib.mp_image import MPImage

FNAME=sys.argv[1]

# little-endian 16 bit data
# Celsius temp = Raw data/64-273.15.

print('Importing: ', FNAME)
a = np.fromfile(FNAME, dtype='>u2')
# get in Kelvin
a = (a / 64.0)

C_TO_KELVIN = 273.15

#a = np.clip(a, C_TO_KELVIN+50, C_TO_KELVIN+200)

maxv = a.max()
minv = a.min()
print("Max=%.3fC Min=%.3fC" % (maxv-273.15, minv-273.15))
if maxv <= minv:
    print("Bad range")
    sys.exit(1)

# convert to 0 to 255
a = (a - minv) * 65535.0 / (maxv - minv)

# convert to uint8 greyscale as 640x512 image
a = a.astype(np.uint16)
a = a.reshape(512, 640)

#im.set_image(a)
cv2.imshow('Grey16', a)
while True:
    cv2.waitKey(1)
