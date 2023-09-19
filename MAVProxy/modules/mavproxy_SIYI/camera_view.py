#!/usr/bin/env python3

from threading import Thread
import cv2

import time, sys

from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.mavproxy_map import mp_slipmap

class CameraView:
    """handle camera view image"""

    def __init__(self, siyi, rtsp_url, filename, res, thermal=False, fps=10):
        self.siyi = siyi
        self.thermal = thermal
        self.im = None
        self.im_colormap = "COLORMAP_MAGMA"
        self.cap = None
        self.raw_frame = None
        self.res = res
        self.rtsp_url = rtsp_url
        self.filename = filename
        self.mode = "Flag"
        self.thread = Thread(target=self.capture)
        self.thread.daemon = True
        self.thread.start()
        self.last_frame_t = time.time()
        self.fps = fps

    def set_title(self, title):
        """set image title"""
        if self.im is None:
            return
        self.im.set_title(title)

    def capture(self):
        """thermal capture thread"""
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
        )

        popup = self.im.get_popup_menu()
        popup.add_to_submenu(
            ["Mode"], MPMenuItem("ClickTrack", returnkey="Mode:ClickTrack")
        )
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

        gst_pipeline = "rtspsrc location={0} latency=0 buffer-mode=auto ! rtph265depay !  tee name=tee1 tee1. ! h265parse ! avdec_h265  ! videoconvert ! video/x-raw,format=BGRx ! videorate ! video/x-raw,framerate={2}/1 ! appsink drop=true tee1. ! queue ! h265parse config-interval=15 ! video/x-h265 ! mpegtsmux ! filesink location={1}".format(
            self.rtsp_url, self.filename, self.fps
        )

        print("gstreamer pipeline is:")
        print("gst-launch-1.0 %s" % gst_pipeline)

        self.cap = cv2.VideoCapture(
            gst_pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.cap or not self.cap.isOpened():
            print("VideoCapture not opened")
            self.cap = None
            return

        while True:
            try:
                _, frame = self.cap.read()
            except Exception:
                break
            if frame is None:
                break
            im = self.im
            if im is None:
                self.cap.release()
                self.cap = None
                return
            self.raw_frame = frame
            now = time.time()
            dt = now - self.last_frame_t
            self.last_frame_t = now
            self.fps = 0.95 * self.fps + 0.05 / dt
            # self.siyi.console.set_status('FPS', 'FPS %.2f' % self.fps, row=6)
            if self.thermal and self.im_colormap is not None:
                cmap = getattr(cv2, self.im_colormap, None)
                if cmap is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    frame = cv2.applyColorMap(frame, cmap)
            im.set_image(frame, bgr=True)
        print("rtsp stream ended for %s" % self.rtsp_url)
        self.im = None
        self.cap.release()
        self.cap = None

    def get_pixel_temp(self, x, y):
        """get temperature of a pixel"""
        v = self.raw_frame[y][x][0]
        gmin = self.raw_frame[..., 0].min()
        gmax = self.raw_frame[..., 0].max()
        if gmax <= gmin:
            return -1
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
                    self.im_colormap = event.returnkey
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
            if event.ClassName == "wxMouseEvent":
                if self.raw_frame is not None:
                    (yres, xres, depth) = self.raw_frame.shape
                    if (
                        self.thermal
                        and self.siyi is not None
                        and event.x < xres
                        and event.y < yres
                        and event.x >= 0
                        and event.y >= 0
                    ):
                        self.siyi.spot_temp = self.get_pixel_temp(event.x, event.y)
                        self.update_title()
            if (
                event.ClassName == "wxMouseEvent"
                and event.leftIsDown
                and self.raw_frame is not None
                and self.siyi is not None
            ):
                (yres, xres, depth) = self.raw_frame.shape
                x = (2 * event.x / float(xres)) - 1.0
                y = (2 * event.y / float(yres)) - 1.0
                aspect_ratio = float(xres) / yres
                if self.thermal:
                    FOV = self.siyi.siyi_settings.thermal_fov
                elif self.siyi.rgb_lens == "zoom":
                    FOV = self.siyi.siyi_settings.zoom_fov / self.siyi.last_zoom
                else:
                    FOV = self.siyi.siyi_settings.wide_fov
                slant_range = self.siyi.get_slantrange(x, y, FOV, aspect_ratio)
                if slant_range is None:
                    return
                latlonalt = self.siyi.get_latlonalt(
                    slant_range, x, y, FOV, aspect_ratio
                )
                if latlonalt is None:
                    return
                latlon = (latlonalt[0], latlonalt[1])
                if self.mode == "ClickTrack":
                    self.siyi.set_target(latlonalt[0], latlonalt[1], latlonalt[2])
                else:
                    self.siyi.mpstate.map.add_object(
                        mp_slipmap.SlipIcon(
                            "SIYIClick", latlon, self.siyi.click_icon, layer="SIYI"
                        )
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
