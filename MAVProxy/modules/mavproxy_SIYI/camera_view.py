#!/usr/bin/env python3

from threading import Thread
import cv2

import time, sys

from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_util
import numpy as np

class CameraView:
    """handle camera view image"""

    def __init__(self, siyi, rtsp_url, filename, res, thermal=False, fps=10, video_idx=-1):
        self.siyi = siyi
        self.thermal = thermal
        self.im = None
        self.im_colormap = "MAGMA"
        self.tracking = False
        self.cap = None
        self.res = res
        self.rtsp_url = rtsp_url
        self.filename = filename
        self.last_frame_t = time.time()
        self.fps = fps
        self.frame_counter = -1
        self.video_idx = video_idx

        self.im = MPImage(
            title="Camera View",
            mouse_events=True,
            mouse_movement_events=self.thermal,
            width=self.res[0],
            height=self.res[1],
            key_events=True,
            can_drag=False,
            can_zoom=False,
            auto_size=False,
            auto_fit=True,
            fps = 30,
        )

        popup = self.im.get_popup_menu()
        popup.add_to_submenu(["Mode"], MPMenuItem("ClickTrack", returnkey="Mode:ClickTrack"))
        popup.add_to_submenu(["Mode"], MPMenuItem("Flag", returnkey="Mode:Flag"))
        if self.thermal:
            colormaps = [
                "Threshold",
                "GloryHot",
                "AUTUMN",
                "BONE",
                "JET",
                "WINTER",
                "RAINBOW",
                "OCEAN",
                "SUMMER",
                "SPRING",
                "COOL",
                "HSV",
                "PINK",
                "HOT",
                "PARULA",
                "MAGMA",
                "INFERNO",
                "PLASMA",
                "VIRIDIS",
                "CIVIDIS",
                "TWILIGHT",
                "TWILIGHT_SHIFTED",
                "TURBO",
                "DEEPGREEN",
                "None",
            ]
            for c in colormaps:
                popup.add_to_submenu(
                    ["ColorMap"], MPMenuItem(c, returnkey="COLORMAP_" + c)
                )
        else:
            popup.add_to_submenu(
                ["Lens"], MPMenuItem("WideAngle", returnkey="Lens:wide")
            )
            popup.add_to_submenu(["Lens"], MPMenuItem("Zoom", returnkey="Lens:zoom"))
            popup.add_to_submenu(
                ["Lens"], MPMenuItem("SplitScreen", returnkey="Lens:split")
            )
            for z in range(1, 11):
                popup.add_to_submenu(
                    ["Zoom"], MPMenuItem("%ux" % z, returnkey="Zoom:%u" % z)
                )
        popup.add_to_submenu(["Marker"], MPMenuItem("Flame", returnkey="Marker:flame"))
        popup.add_to_submenu(["Marker"], MPMenuItem("Flag", returnkey="Marker:flag"))
        popup.add_to_submenu(["Marker"], MPMenuItem("Barrell", returnkey="Marker:barrell"))

        gst_pipeline = "rtspsrc location={0} latency=0 buffer-mode=auto ! rtph265depay !  tee name=tee1 tee1. ! queue ! h265parse ! avdec_h265  ! videoconvert ! video/x-raw,format=BGRx ! appsink tee1. ! queue ! h265parse config-interval=15 ! video/x-h265 ! mpegtsmux ! filesink location={1}".format(
            self.rtsp_url, self.filename
        )

        self.im.set_gstreamer(gst_pipeline)
        if self.thermal:
            self.im.set_colormap(self.im_colormap)
        self.im.set_colormap("None")
        self.siyi.cmd_palette(["WhiteHot"])

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
        title += " (mode %s)" % self.siyi.click_mode
        self.im.set_title(title)

    def get_pixel_temp(self, event):
        """get temperature of a pixel"""
        if event.pixel is None:
            return -1
        v = event.pixel[0]
        gmin = 0
        gmax = 255
        if self.siyi is None:
            return -1
        return self.siyi.tmin + ((v - gmin) / float(gmax - gmin)) * (
            self.siyi.tmax - self.siyi.tmin
        )

    def end_tracking(self):
        '''end object tracking'''
        self.tracking = False
        if self.im is not None:
            self.im.end_tracking()

    def update_title(self):
        """update thermal view title"""
        if self.siyi is None:
            return
        if self.thermal:
            self.set_title(
                "Thermal View TEMP=%.2fC RANGE(%.2fC to %.2fC)"
                % (self.siyi.spot_temp, self.siyi.tmin, self.siyi.tmax)
            )
        elif self.siyi.rgb_lens == "zoom":
            self.set_title("Zoom View %.1fx" % self.siyi.last_zoom)
        else:
            self.set_title("Wide View")

    def xy_to_latlon(self, x, y, shape):
        '''convert x,y pixel coordinates to a latlon tuple'''
        (yres, xres, depth) = shape
        x = (2 * x / float(xres)) - 1.0
        y = (2 * y / float(yres)) - 1.0
        aspect_ratio = float(xres) / yres
        if self.thermal:
            FOV = self.siyi.siyi_settings.thermal_fov
        elif self.siyi.rgb_lens == "zoom":
            FOV = self.siyi.siyi_settings.zoom_fov / self.siyi.last_zoom
        else:
            FOV = self.siyi.siyi_settings.wide_fov
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
                if event.returnkey.startswith("COLORMAP_Threshold"):
                    d = self.create_colormap_dict()
                    self.im.set_colormap(d)
                    self.siyi.cmd_palette(["WhiteHot"])
                if event.returnkey.startswith("COLORMAP_GloryHot"):
                    self.im.set_colormap("None")
                    self.siyi.cmd_palette(["GloryHot"])
                elif event.returnkey.startswith("COLORMAP_"):
                    self.im.set_colormap(event.returnkey[9:])
                    self.siyi.cmd_palette(["WhiteHot"])
                elif event.returnkey.startswith("Mode:"):
                    self.siyi.click_mode = event.returnkey[5:]
                    print("ViewMode: %s" % self.siyi.click_mode)
                elif event.returnkey.startswith("Marker:"):
                    self.siyi.handle_marker(event.returnkey[7:])
                elif event.returnkey.startswith("Lens:") and self.siyi is not None:
                    self.siyi.cmd_imode([event.returnkey[5:]])
                elif event.returnkey.startswith("Zoom:") and self.siyi is not None:
                    self.siyi.cmd_zoom([event.returnkey[5:]])
                elif event.returnkey == "fitWindow":
                    self.im.fit_to_window()
                elif event.returnkey == "fullSize":
                    self.im.full_size()
                continue
            if isinstance(event, MPImageTrackPos):
                if not self.tracking:
                    continue
                latlonalt = self.xy_to_latlon(event.x, event.y, event.shape)
                if latlonalt is None:
                    return
                self.siyi.set_target(latlonalt[0], latlonalt[1], latlonalt[2])
                continue
            if isinstance(event, MPImageFrameCounter):
                self.frame_counter = event.frame
                self.siyi.log_frame_counter(self.video_idx, self.thermal, self.frame_counter)
                continue

            if event.ClassName == "wxMouseEvent":
                if event.pixel is not None:
                    self.siyi.spot_temp = self.get_pixel_temp(event)
                    self.update_title()
            if (
                event.ClassName == "wxMouseEvent"
                and event.leftIsDown
                and event.pixel is not None
                and self.siyi is not None
            ):
                latlonalt = self.xy_to_latlon(event.x, event.y, event.shape)
                if latlonalt is None:
                    continue
                if event.shiftDown:
                    (xres,yres) = (event.shape[1], event.shape[0])
                    twidth = int(yres*0.01*self.siyi.siyi_settings.track_size_pct)
                    self.siyi.end_tracking()
                    self.im.start_tracker(event.X, event.Y, twidth, twidth)
                    self.tracking = True
                elif event.controlDown:
                    self.siyi.end_tracking()
                else:
                    self.siyi.camera_click(self.siyi.click_mode, latlonalt)

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("camera_view.py [options]")
    parser.add_option("--rtsp-server", default=None, type=str, help="RTSP UTL")
    parser.add_option("--thermal", action='store_true', help="thermal camera")
    parser.add_option("--fps", type=int, help="frames per second")

    (opts, args) = parser.parse_args()
    if opts.rtsp_server is None:
        print("Must specify an RTSP URL")
        sys.exit(1)

    c = CameraView(None, opts.rtsp_server, "output.mts", (1280, 1024), thermal=opts.thermal, fps=opts.fps)

    while True:
        time.sleep(0.1)
        c.check_events()
