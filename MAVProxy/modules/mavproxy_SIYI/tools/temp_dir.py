#!/usr/bin/env python3

import sys
import numpy as np
import cv2
import time
import glob
import os
from MAVProxy.modules.lib.mp_image import MPImage

DNAME=sys.argv[1]

def display_file(fname):
    print('Importing: ', fname)
    a = np.fromfile(fname, dtype='>u2')
    if len(a) != 640 * 512:
        print("Bad size %u" % len(a))
        return
    # get in Kelvin
    a = (a / 64.0)

    C_TO_KELVIN = 273.15

    #a = np.clip(a, C_TO_KELVIN+50, C_TO_KELVIN+200)

    maxv = a.max()
    minv = a.min()
    print("Max=%.3fC Min=%.3fC" % (maxv-273.15, minv-273.15))
    if maxv <= minv:
        print("Bad range")
        return

    # convert to 0 to 255
    a = (a - minv) * 65535.0 / (maxv - minv)

    # convert to uint8 greyscale as 640x512 image
    a = a.astype(np.uint16)
    a = a.reshape(512, 640)

    cv2.imshow('Grey16', a)

def find_newest(dname):
    flist = glob.glob(dname + "/siyi*bin")
    return max(flist, key=os.path.getctime)

last_file = None

while True:
    fname = find_newest(DNAME)
    if fname != last_file:
        last_file = fname
        display_file(fname)
    cv2.waitKey(1)
