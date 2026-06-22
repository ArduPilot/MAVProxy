#!/usr/bin/env python3

# save images in therm cap

import sys
import numpy as np
import cv2
import time
import glob
import os
from MAVProxy.modules.lib.mp_image import MPImage
from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--min-temp", default=None, type=float, help="min temperature")
parser.add_argument("--siyi-log", default=None, type=float, help="SIYI binlog")
parser.add_argument("dirname", default=None, type=str, help="directory")
parser.add_argument("-w", "--write", default=False, action='store_true', help="Write the images instead of displaying them")
parser.add_argument("-s", "--startimg", default=0, type=int, help="Image to start with")
args = parser.parse_args()

DNAME=args.dirname
mouse_temp=-1
tmin = -1
tmax = -1
last_data = None
C_TO_KELVIN = 273.15

def update_title():
    global tmin, tmax, mouse_temp
    cv2.setWindowTitle('Thermal', "Thermal: (%.1fC to %.1fC) %.1fC" % (tmin, tmax, mouse_temp))

def click_callback(event, x, y, flags, param):
    global last_data, mouse_temp
    if last_data is None:
        return
    p = last_data[y*640+x]
    mouse_temp = p - C_TO_KELVIN
    update_title()

def display_file(fname,w):
    global mouse_temp, tmin, tmax, last_data
    if not w:
        print('Importing: ', fname)
    a = np.fromfile(fname, dtype='>u2')
    if len(a) != 640 * 512:
        print("Bad size %u" % len(a))
        return
    # get in Kelvin
    a = (a / 64.0)

    C_TO_KELVIN = 273.15

    maxv = a.max()
    minv = a.min()

    tmin = minv - C_TO_KELVIN
    tmax = maxv - C_TO_KELVIN

    if args.min_temp is not None and tmax < args.min_temp:
        return

    if not w:
        print("Max=%.3fC Min=%.3fC" % (tmax, tmin))
    if maxv <= minv:
        print("Bad range")
        return

    last_data = a

    max_to = 65535.0
    if w:
        max_to = 255.0
    # convert to 0 to 255 or 65535
    a = (a - minv) * max_to / (maxv - minv)

    # convert to greyscale as 640x512 image
    if w:
        a = a.astype(np.uint8)
        a = a.reshape(512, 640)
        if not os.path.exists('tem_jpg'):
            os.makedirs('tem_jpg')
        jpg_file = 'tem_jpg/'+os.path.basename(fname).removesuffix('.bin')+f"_max{tmax:.1f}_min{tmin:.1f}.jpg"
        success = cv2.imwrite(jpg_file,a)
        if not success:
            print("Failed to write image to "+jpg_file)
            exit(1)
    else:
        a = a.astype(np.uint16)
        a = a.reshape(512, 640)
        cv2.imshow('Thermal', a)
        cv2.setMouseCallback('Thermal', click_callback)
        cv2.setWindowTitle('Thermal', "Thermal: (%.1fC to %.1fC) %.1fC" % (tmin, tmax, mouse_temp))
        if cv2.waitKey(0) & 0xFF == ord('q'):
            exit(0)
    

flist = sorted(glob.glob(DNAME + "/*bin"))
w = args.write

if not w:
    print("Press q to quit. Press any other key to advance to next frame.")

if w and os.path.exists('tem_jpg'):
    print("tem_jpg directory already exists. Continue? y to continue, anything else to exit.")
    rep = input()
    if rep != "y":
        exit(0)

for i in range(args.startimg,len(flist)):
    display_file(flist[i],w)
    if w and i % 100 == 0:
        print(f"Progress: {i}")
