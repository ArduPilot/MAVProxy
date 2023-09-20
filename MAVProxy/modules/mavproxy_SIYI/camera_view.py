#!/usr/bin/env python3

from threading import Thread
import cv2

import time, sys

from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.mavproxy_map import mp_slipmap

class CameraView:
    """handle camera view image"""

    def __init__(self, siyi, rtsp_url, filename, res, thermal=False, fps=10):
        self.siyi = siyi
        self.thermal = thermal
        self.im = None
        self.im_colormap = "MAGMA"
        self.cap = None
        self.res = res
        self.rtsp_url = rtsp_url
        self.filename = filename
        self.mode = "Flag"
        self.last_frame_t = time.time()
        self.fps = fps

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

        gst_pipeline = "rtspsrc location={0} latency=0 buffer-mode=auto ! rtph265depay !  tee name=tee1 tee1. ! queue ! h265parse ! avdec_h265  ! videoconvert ! video/x-raw,format=BGRx ! appsink tee1. ! queue ! h265parse config-interval=15 ! video/x-h265 ! mpegtsmux ! filesink location={1}".format(
            self.rtsp_url, self.filename
        )

        self.im.set_gstreamer(gst_pipeline)
        if self.thermal:
            self.im.set_colormap(self.im_colormap)

    def set_title(self, title):
        """set image title"""
        if self.im is None:
            return
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
                if event.returnkey.startswith("COLORMAP_"):
                    self.im.set_colormap(event.returnkey[9:])
                elif event.returnkey.startswith("Mode:"):
                    self.mode = event.returnkey[5:]
                    print("ViewMode: %s" % self.mode)
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
                latlonalt = self.xy_to_latlon(event.x, event.y, event.shape)
                if latlonalt is None:
                    return
                self.siyi.set_target(latlonalt[0], latlonalt[1], latlonalt[2])
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
                    self.im.start_tracker(event.X, event.Y, twidth, twidth)
                elif event.controlDown:
                    self.im.end_tracker()
                elif self.mode == "ClickTrack":
                    self.siyi.set_target(latlonalt[0], latlonalt[1], latlonalt[2])
                else:
                    latlon = (latlonalt[0], latlonalt[1])
                    self.siyi.mpstate.map.add_object(
                        mp_slipmap.SlipIcon("SIYIClick", latlon, self.siyi.click_icon, layer="SIYI")
                    )

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
