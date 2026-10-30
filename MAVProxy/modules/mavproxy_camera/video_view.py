"""Small OpenCV-backed viewer for MAVLink video streams."""

import cv2
from urllib.parse import urlsplit

from pymavlink import mavutil

from MAVProxy.modules.lib.mp_image import MPImage, MPImageOSD_Rect, MPImageOSD_None
from MAVProxy.modules.lib.mp_menu import MPMenuItem, MPMenuSubMenu
from MAVProxy.modules.mavproxy_camera import CLICK_MODES

CLICK_MODE_NAMES = {
    "flag": "Flag",
    "roi": "Point gimbal (ROI)",
    "track": "Camera track",
    "marker": "Marker",
}

# a drag shorter than this fraction of the image is a click
DRAG_THRESHOLD = 0.01


def _opencv_has_gstreamer():
    for line in cv2.getBuildInformation().splitlines():
        if line.strip().startswith("GStreamer:"):
            return line.split(":", 1)[1].strip().upper().startswith("YES")
    return False


def video_pipeline(stream, uri, latency):
    """Decode the advertised transport, independently of its video codec."""
    codec = "h265" if stream.encoding == 2 else "h264"
    parsed = urlsplit(uri)
    mpegts = (getattr(stream, 'type', None) == mavutil.mavlink.VIDEO_STREAM_TYPE_MPEG_TS or
              parsed.path.lower().endswith(('.misb', '.ts')))
    location = '"%s"' % uri.replace('\\', '\\\\').replace('"', '\\"')
    if parsed.scheme.lower() in ('rtsp', 'rtsps'):
        source = ("rtspsrc location=%s latency=%u protocols=tcp "
                  "tcp-timeout=3000000 buffer-mode=auto" % (location, max(0, latency)))
        source += " ! rtpmp2tdepay ! tsdemux" if mpegts else " ! rtp%sdepay" % codec
    elif mpegts and parsed.scheme.lower() in ('http', 'https'):
        source = "souphttpsrc location=%s ! tsdemux" % location
    else:
        return ("uridecodebin uri=%s ! videoconvert ! "
                "video/x-raw,format=BGRx ! appsink" % location)
    # Select the video pad: a MISB transport also exposes a KLV data pad.
    return (source + " ! video/x-%s ! %sparse ! avdec_%s ! videoconvert ! "
            "video/x-raw,format=BGRx ! appsink" % (codec, codec, codec))


class VideoView:
    def __init__(self, module, camera, stream, uri, latency):
        self.module = module
        self.camera = camera
        self.stream = stream
        self.uri = uri
        self.title = "%s - %s" % (camera.label(), stream.name)
        self.shown_mode = None
        # where a track mode drag began, 0..1 across and down the image
        self.drag_start = None
        self.image = MPImage(title=self.title, width=stream.resolution_h,
                             height=stream.resolution_v, auto_size=False,
                             auto_fit=True, can_zoom=False, fps=60,
                             mouse_events=True)
        menu = self.image.get_popup_menu()
        menu.add(MPMenuItem("Take photo", returnkey="Camera:Photo"))
        menu.add(MPMenuItem("Toggle recording", returnkey="Camera:Record"))
        menu.add(MPMenuItem("Autofocus", returnkey="Camera:Autofocus"))
        menu.add(MPMenuSubMenu("Click mode", items=[
            MPMenuItem(CLICK_MODE_NAMES[mode], returnkey="ClickMode:" + mode)
            for mode in CLICK_MODES]))
        menu.add(MPMenuItem("Stop tracking", returnkey="Camera:StopTracking"))
        self.image.set_popup_menu(menu)
        self._update_title()
        if _opencv_has_gstreamer():
            # Match the proven mavproxy_SIYI CameraView path while selecting
            # the depayloader and decoder from VIDEO_STREAM_INFORMATION.
            pipeline = video_pipeline(stream, uri, latency)
            self.image.set_gstreamer(pipeline, reconnect=True)
        else:
            # PyPI OpenCV builds generally omit CAP_GSTREAMER. Passing the URI
            # directly lets MPImage use OpenCV's available FFmpeg backend.
            self.image.set_video(uri, reconnect=True)

    def alive(self):
        return self.image is not None and self.image.is_alive()

    def close(self):
        if self.image is not None:
            self.image.terminate()
            self.image = None

    def _click_mode(self):
        with self.module._camera_context(self.camera):
            return self.module.camera_settings.click_mode

    def _update_title(self):
        mode = self._click_mode()
        if mode != self.shown_mode:
            self.shown_mode = mode
            self.image.set_title("%s (click: %s)" % (self.title, CLICK_MODE_NAMES[mode]))

    def _command(self, command):
        self.module.cmd_camera([
            "for", "%u:%u" % (self.camera.system_id, self.camera.component_id)
        ] + command)

    def _menu_event(self, event):
        if event.returnkey == "Camera:Photo":
            self._command(["photo"])
        elif event.returnkey == "Camera:Record":
            self._command(["record", "toggle"])
        elif event.returnkey == "Camera:Autofocus":
            self._command(["focus", "auto"])
        elif event.returnkey == "Camera:StopTracking":
            self.module.stop_tracking(self.camera)
        elif event.returnkey.startswith("ClickMode:"):
            self.module.camera_settings.set("click_mode", event.returnkey[10:])

    def _mouse_event(self, event):
        shape = getattr(event, "shape", None)
        if shape is None:
            # no frame yet
            return
        height, width = shape[0], shape[1]
        point = (event.X / float(width), event.Y / float(height))
        if self._click_mode() != "track":
            self.drag_start = None
            if getattr(event, "leftPressed", False):
                self.module.camera_click(self.camera, self.stream,
                                         event.X, event.Y, width, height)
            return
        if getattr(event, "leftPressed", False):
            self.drag_start = point
        if self.drag_start is None:
            return
        dragged = max(abs(point[0] - self.drag_start[0]),
                      abs(point[1] - self.drag_start[1])) > DRAG_THRESHOLD
        if getattr(event, "leftReleased", False):
            self.image.add_OSD(MPImageOSD_None("track_rect"))
            if dragged:
                self.module.camera_track(self.camera, self.drag_start, point)
            else:
                self.module.camera_track(self.camera, self.drag_start)
            self.drag_start = None
        elif getattr(event, "leftIsDown", False) and dragged:
            self.image.add_OSD(MPImageOSD_Rect("track_rect", self.drag_start,
                                               point, (255, 255, 0), 2))

    def check_events(self):
        if not self.alive():
            return
        for event in self.image.events():
            if isinstance(event, MPMenuItem):
                self._menu_event(event)
            elif getattr(event, "ClassName", "") == "wxMouseEvent":
                self._mouse_event(event)
        self._update_title()
