#!/usr/bin/env python3

from threading import Thread
import cv2
import os
import socket
import struct
import datetime
import math

import time, sys

from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_util
import numpy as np

EXPECTED_DATA_SIZE = 640 * 512 * 2
C_TO_KELVIN = 273.15

class RawThermal:
    """handle raw thermal image"""

    def __init__(self, siyi, res):
        self.siyi = siyi
        self.uri = ("192.168.144.25", 7345)
        self.im = None
        self.tracking = False
        self.cap = None
        self.res = res
        self.FOV = 24.2
        self.last_frame_t = time.time()
        self.logdir = 'thermal'
        self.should_exit = False
        self.last_data = None
        self.tmin = -1
        self.tmax = -1
        self.mouse_temp = -1
        self.image_count = 0
        self.marker_history = []
        self.last_tstamp = None


        if siyi is not None:
            self.logdir = self.siyi.logdir

        self.im = MPImage(
            title="Raw Thermal",
            mouse_events=True,
            mouse_movement_events=True,
            width=self.res[0],
            height=self.res[1],
            key_events=True,
            can_drag=False,
            can_zoom=False,
            auto_size=False,
            auto_fit=True
        )

        popup = self.im.get_popup_menu()
        popup.add_to_submenu(["Mode"], MPMenuItem("ClickTrack", returnkey="Mode:ClickTrack"))
        popup.add_to_submenu(["Mode"], MPMenuItem("Flag", returnkey="Mode:Flag"))
        popup.add_to_submenu(["Marker"], MPMenuItem("Flame", returnkey="Marker:flame"))
        popup.add_to_submenu(["Marker"], MPMenuItem("Flag", returnkey="Marker:flag"))
        popup.add_to_submenu(["Marker"], MPMenuItem("Barrell", returnkey="Marker:barrell"))

        dname = os.path.join(self.logdir, 'thermal')
        try:
            os.mkdir(dname)
        except Exception as ex:
            pass

        self.thread = Thread(target=self.fetch_loop, name='fetch_loop')
        self.thread.daemon = False
        self.thread.start()

    def fetch_loop(self):
        '''main thread'''
        while True:
            if self.im is None:
                break
            time.sleep(0.25)
            ret = self.fetch_latest()
            if ret is None:
                continue
            (fname, tstamp, data) = ret
            if self.last_tstamp is not None and tstamp == self.last_tstamp:
                continue
            self.last_tstamp = tstamp
            self.display_image(fname, data)
            self.save_image(fname, tstamp, data)

    def in_history(self, latlon):
        '''check if latlon in the history'''
        autoflag_dist = self.siyi.siyi_settings.autoflag_dist
        for (lat1,lon1) in self.marker_history:
            dist = mp_util.gps_distance(lat1,lon1,latlon[0],latlon[1])
            if dist < autoflag_dist:
                return True

        # add to history
        self.marker_history.append(latlon)
        if len(self.marker_history) > self.siyi.siyi_settings.autoflag_history:
            self.marker_history.pop(0)
        return False


    def handle_auto_flag(self):
        if not self.siyi.siyi_settings.autoflag_enable:
            return
        if self.tmax < self.siyi.siyi_settings.autoflag_temp:
            return

        map = self.siyi.module('map')
        if not map:
            return

        width = self.res[0]
        height = self.res[1]
        latlonalt = self.xy_to_latlon(width//2, height//2)
        if latlonalt is None:
            return
        slices = self.siyi.siyi_settings.autoflag_slices
        slice_width = width // slices
        slice_height = height // slices

        data = self.last_data.reshape(height, width)

        for sx in range(slices):
            for sy in range(slices):
                sub = data[sx*slice_width:(sx+1)*slice_width, sy*slice_height:(sy+1)*slice_height]
                maxv = sub.max() - C_TO_KELVIN
                if maxv < self.siyi.siyi_settings.autoflag_temp:
                    continue
                X = (sx*slice_width) + slice_width//2
                Y = (sy*slice_height) + slice_height//2
                latlonalt = self.xy_to_latlon(X, Y)
                if latlonalt is None:
                    continue
                latlon = (latlonalt[0], latlonalt[1])
                if self.in_history(latlon):
                    continue
                map.cmd_map_marker(["flame"], latlon=latlon)

    def display_image(self, fname, data):
        '''display an image'''
        a = np.frombuffer(data, dtype='>u2')
        if len(a) != 640 * 512:
            print("Bad size %u" % len(a))
            return
        # get in Kelvin
        a = (a / 64.0)

        maxv = a.max()
        minv = a.min()

        self.tmin = minv - C_TO_KELVIN
        self.tmax = maxv - C_TO_KELVIN
        if maxv <= minv:
            maxv = minv + 1

        self.last_data = a

        self.handle_auto_flag()

        # convert to 0 to 255
        a = (a - minv) * 255 / (maxv - minv)

        # convert to uint8 greyscale as 640x512 image
        a = a.astype(np.uint8)
        a = a.reshape(512, 640)

        a = cv2.cvtColor(a, cv2.COLOR_GRAY2RGB)
        if self.im is None:
            return
        self.im.set_image(a)
        self.image_count += 1
        self.update_title()
            
    def fetch_latest(self):
        '''fetch a thermal image'''
        tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        timeout = self.siyi.siyi_settings.fetch_timeout
        if timeout >= 0:
            tcp.settimeout(2)
        try:
            tcp.connect(self.uri)
        except Exception:
            return None
        buf = bytearray()

        while True:
            try:
                b = tcp.recv(1024)
            except Exception:
                break
            if not b:
                break
            buf += b

        if len(buf) != 128 + 8 + EXPECTED_DATA_SIZE:
            return None

        fname = buf[:128].decode("utf-8").strip('\x00')
        tstamp, = struct.unpack("<d", buf[128:128+8])
        data = buf[128+8:]
        return fname, tstamp, data

    def save_image(self, fname, tstamp, data):
        '''same thermal image in thermal/ directory'''
        fname = os.path.basename(fname)[:-4]
        tstr = datetime.datetime.fromtimestamp(tstamp).strftime("%Y_%m_%d_%H_%M_%S")
        subsec = tstamp - math.floor(tstamp)
        millis = int(subsec * 1000)
        fname = "%s_%s_%03u.bin" % (fname, tstr, millis)
        fname = os.path.join(self.logdir, 'thermal', fname)
        f = open(fname, 'wb')
        f.write(data)
        f.close()
        os.utime(fname, (tstamp, tstamp))

    def create_colormap_threshold(self, threshold):
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

    def create_colormap_dict(self):
        '''create a yellow->red colormap for all thresholds'''
        ret = dict()
        for i in range(256):
            ret[i] = self.create_colormap_threshold(i)
        return ret

    def set_threshold(self, threshold_value):
        '''set pixel value for thermal colormap'''
        if self.im is not None:
            self.im.set_colormap_index(threshold_value)
    
    def set_title(self, title):
        """set image title"""
        if self.im is None:
            return
        self.im.set_title(title)

    def get_pixel_temp(self, event):
        """get temperature of a pixel"""
        x = event.x
        y = event.y
        if self.last_data is None:
            return -1
        try:
            p = self.last_data[y*640+x]
            return p - C_TO_KELVIN
        except Exception:
            return -1

    def end_tracking(self):
        '''end object tracking'''
        self.tracking = False
        if self.im is not None:
            self.im.end_tracking()

    def update_title(self):
        """update thermal view title"""
        self.set_title("RawThermal(%u): (%.1fC to %.1fC) %.1fC (mode %s)" % (self.image_count, self.tmin, self.tmax,
                                                                                 self.mouse_temp, self.siyi.click_mode))

    def xy_to_latlon(self, x, y):
        '''convert x,y pixel coordinates to a latlon tuple'''
        (xres, yres) = self.res
        x = (2 * x / float(xres)) - 1.0
        y = (2 * y / float(yres)) - 1.0
        aspect_ratio = float(xres) / yres
        FOV = self.FOV
        slant_range = self.siyi.get_slantrange(x, y, FOV, aspect_ratio)
        if slant_range is None:
            return None
        return self.siyi.get_latlonalt(slant_range, x, y, FOV, aspect_ratio)


    def check_events(self):
        """check for image events"""
        if self.im is None:
            return
        if not self.im.is_alive():
            self.im = None
            return
        for event in self.im.events():
            if isinstance(event, MPMenuItem):
                if event.returnkey.startswith("Mode:"):
                    self.siyi.click_mode = event.returnkey[5:]
                    print("ViewMode: %s" % self.siyi.click_mode)
                elif event.returnkey.startswith("Marker:"):
                    self.siyi.handle_marker(event.returnkey[7:])
                elif event.returnkey == "fitWindow":
                    self.im.fit_to_window()
                elif event.returnkey == "fullSize":
                    self.im.full_size()
                continue
            if isinstance(event, MPImageTrackPos):
                if not self.tracking:
                    continue
                latlonalt = self.xy_to_latlon(event.x, event.y)
                if latlonalt is None:
                    return
                self.siyi.set_target(latlonalt[0], latlonalt[1], latlonalt[2])
                continue
            if isinstance(event, MPImageFrameCounter):
                self.frame_counter = event.frame
                continue

            if event.ClassName == "wxMouseEvent":
                self.mouse_temp = self.get_pixel_temp(event)
                self.update_title()
            if (
                event.ClassName == "wxMouseEvent"
                and event.leftIsDown
                and self.siyi is not None
            ):
                latlonalt = self.xy_to_latlon(event.x, event.y)
                if latlonalt is None:
                    continue
                if event.shiftDown:
                    (xres,yres) = self.res
                    twidth = int(yres*0.01*self.siyi.siyi_settings.track_size_pct)
                    self.siyi.end_tracking()
                    self.im.start_tracker(event.x, event.y, twidth, twidth)
                    self.tracking = True
                elif event.controlDown:
                    self.siyi.end_tracking()
                else:
                    self.siyi.camera_click(self.siyi.click_mode, latlonalt)

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("camera_view.py [options]")
    (opts, args) = parser.parse_args()

    c = RawThermal(None, (640, 512))

    while True:
        time.sleep(0.1)
        c.check_events()
