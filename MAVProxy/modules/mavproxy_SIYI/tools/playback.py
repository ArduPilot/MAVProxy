#!/usr/bin/env python3
'''
playback videos with temperature data
'''

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter
from MAVProxy.modules.lib.mp_image import MPImageOSD_HorizonLine
import time
import numpy as np
import cv2
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
import math

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("video", help="video file")
parser.add_argument("--seek", default=None, type=float, help="seek start percentage")
parser.add_argument("--fps", default=None, type=float, help="max playback FPS")
parser.add_argument("--siyi-log", default=None, help="SIYI log file")
parser.add_argument("--video-idx", type=int, default=1, help="SIYI video index")
parser.add_argument("--thermal", action='store_true', help="assume thermal")
parser.add_argument("--attitude", action='store_true', help="show attitude in title")
parser.add_argument("--threshold", type=int, default=60, help="temperature threshold")
parser.add_argument("--min-threshold", type=int, default=250, help="temperature threshold min pixel")
parser.add_argument("--aspect-ratio", type=float, default=1280.0/720.0, help="aspect ratio")
parser.add_argument("--start-frame", type=int, default=None, help="start frame")
parser.add_argument("--fov", type=float, default=88.0, help="horizontal FOV")
args = parser.parse_args()

if args.siyi_log is not None:
    mlog = mavutil.mavlink_connection(args.siyi_log)
else:
    mlog = None

im = MPImage(
    title="Camera View",
    mouse_events=True,
    mouse_movement_events=True,
    width=640,
    height=480,
    key_events=True,
    can_drag=False,
    can_zoom=False,
    auto_size=False,
    auto_fit=True,
    fps = 30,
    )

tstart = time.time()
im.set_video(args.video)
if args.fps is not None:
    im.set_fps_max(args.fps)

if args.seek is not None:
    im.seek_percentage(args.seek)

if args.start_frame is not None:
    im.seek_frame(args.start_frame)
    
def create_colormap_threshold(threshold):
    '''create a yellow->red colormap for a given threshold'''
    def pixel(c):
        p = np.zeros((1,1,3), np.uint8)
        p[:] = c
        return p

    lightyellow = pixel((255,255,0))
    red = pixel((255,0,0))
    white = pixel((threshold,threshold,threshold))
    black = pixel((0,0,0))

    threshold = mp_util.constrain(threshold, 1, 255)

    lut1 = cv2.resize(np.concatenate((black,white), axis=0), (1,threshold), interpolation=cv2.INTER_CUBIC)
    lut2 = cv2.resize(np.concatenate((lightyellow,red), axis=0), (1,256-threshold), interpolation=cv2.INTER_CUBIC)
    lut = np.concatenate((lut1, lut2), axis=0)
    return lut

def create_colormap_dict():
    '''create a yellow->red colormap for all thresholds'''
    ret = dict()
    for i in range(256):
        ret[i] = create_colormap_threshold(i)
    return ret

def get_pixel_temp(event):
    """get temperature of a pixel"""
    if event.pixel is None:
        return -1
    global tmax, tmin
    v = event.pixel[0]
    gmin = 0
    gmax = 255
    return tmin + ((v - gmin) / float(gmax - gmin)) * (tmax - tmin)

d = create_colormap_dict()
im.set_colormap(d)
im.set_colormap_index(256)

tmax = 0
tmin = 0
frame_counter = 0
paused = False
threshold_value = -1
fps = args.fps

def check_events():
    global frame_counter, paused, im, threshold_value, fps
    for event in im.events():
        if isinstance(event, MPImageFrameCounter):
            frame_counter = event.frame
            continue
        if getattr(event,'ClassName',None) == 'wxKeyEvent':
            print('key %u' % event.KeyCode)
            if event.KeyCode == ord('P'):
                print("PAUSE")
                paused = not paused
                im.set_fps_max(0 if paused else fps)
            if event.KeyCode == ord('F'):
                fps *= 1.1
                print("Speed %.1f" % fps)
                im.set_fps_max(0 if paused else fps)
            if event.KeyCode == ord('S'):
                fps /= 1.1
                print("Speed %.1f" % fps)
                im.set_fps_max(0 if paused else fps)
            if event.KeyCode == ord('N'):
                fps = args.fps
                print("Speed %.1f" % fps)
                im.set_fps_max(0 if paused else fps)
        if getattr(event,'ClassName',None) == "wxMouseEvent":
            if event.pixel is not None:
                spot_temp = get_pixel_temp(event)
                if args.thermal:
                    print("TEMP: %.1f px=%u range=(%.1f %.1f) tv=%d" % (spot_temp, event.pixel[0], tmin, tmax, threshold_value))
                if args.attitude:
                    ATT = mlog.messages.get('ATT',None)
                    SIEN = mlog.messages.get('SIEN',None)
                    GPS = mlog.messages.get('GPS',None)
                    if ATT is None or SIEN is None or GPS is None:
                        return
                    print("SIEN (%.1f %.1f %.1f) ATT (%.1f %.1f %.1f)" % (SIEN.R, SIEN.P, SIEN.Y, ATT.Roll, ATT.Pitch, ATT.Yaw))



def att_from_encoders(ATT,SIEN):
    from pymavlink.rotmat import Matrix3
    from math import radians, degrees
    m1 = Matrix3()
    # we use zero yaw as this is in vehicle frame
    m1.from_euler(radians(ATT.Roll),radians(ATT.Pitch),0)
    m1.rotate_312(radians(SIEN.R),radians(SIEN.P),radians(SIEN.Y))
    (r,p,y) = m1.to_euler()
    return degrees(r),degrees(p),degrees(y)

def add_horizon_line(label, roll_deg, pitch_deg, color):
    '''draw a horizon line for given roll/pitch'''
    global im
    im.add_OSD(MPImageOSD_HorizonLine(label, roll_deg, pitch_deg, args.fov, color))

def show_attitude():
    global mlog, im, frame_counter
    SIGA = mlog.messages.get('SIGA',None)
    ATT = mlog.messages.get('ATT',None)
    SIEN = mlog.messages.get('SIEN',None)
    SIEA = mlog.messages.get('SIEA',None)
    GPS = mlog.messages.get('GPS',None)
    SIVL = mlog.messages.get('SIVL',None)
    if SIGA is None or ATT is None or SIEN is None or GPS is None or SIEA is None:
        return
    from pymavlink.rotmat import Matrix3
    from pymavlink.rotmat import Vector3
    from math import radians, degrees
    m = Matrix3()
    m.from_euler312(radians(SIGA.R),radians(SIGA.P),radians(SIGA.Y))
    (r2,p2,y2) = m.to_euler()
    title = "SIGA=(%.1f/%.1f %.1f/%.1f %.1f/%.1f)" % (SIGA.R, degrees(r2), SIGA.P, degrees(p2), SIGA.Y, degrees(y2))

    #title += " SIEA=(%.1f %.1f %.1f)" % (SIEA.R, SIEA.P, SIEA.Y)
    title += " ATT=(%.1f %.1f %.1f)" % (ATT.Roll, ATT.Pitch, ATT.Yaw)

    rpy = att_from_encoders(ATT,SIEN)
    title += " EA2=(%.1f %.1f %.1f)" % (rpy[0],rpy[1],rpy[2])

    title += " SIEN=(%.1f %.1f %.1f)" % (SIEN.R,SIEN.P,SIEN.Y)

    title += " GPS=(%.0fm spd=%.0fm/s)" % (GPS.Alt, GPS.Spd)
    title += " M=%u FC=%u" % (SIVL.Mode, frame_counter)

    im.set_title(title)

    add_horizon_line('hor1', rpy[0], rpy[1], (255,0,255))
    add_horizon_line('hor2', degrees(r2), degrees(p2), (255,255,0))


while True:
    #time.sleep(0.01)
    if mlog is not None:
        m = mlog.recv_match(type=['SITR','SIEN','SIGA','ATT','SIFC','GPS','SIVL'])
        if m is None:
            print("EOF")
            break
        mtype = m.get_type()
        if mtype == 'SIFC':
            vtype = 1 if args.thermal else 0
            if m.Type != vtype:
                continue
            if m.Idx != args.video_idx:
                continue
            while frame_counter < m.Frame:
                check_events()
                time.sleep(0.01)
            if frame_counter > m.Frame+5:
                check_events()
                continue

        if args.attitude:
            show_attitude()
        if args.thermal and mtype == 'SITR':
            threshold = args.threshold
            threshold_value = int(255*(threshold - m.TMin)/max(1,(m.TMax-m.TMin)))
            threshold_value = mp_util.constrain(threshold_value, 0, 255)
            if m.TMax < threshold:
                threshold_value = -1
            else:
                threshold_value = max(threshold_value, args.min_threshold)
            im.set_colormap_index(threshold_value)
            tmax = m.TMax
            tmin = m.TMin
            im.set_title("Range (%.1f %.1f) TV=%d t=%.1f" % (tmin, tmax, threshold_value, frame_counter/30.0))
            print(m.TMin, m.TMax, threshold_value, vtime, frame_counter/30.0)

