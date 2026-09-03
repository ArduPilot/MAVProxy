"""Small OpenCV-backed viewer for MAVLink video streams."""

import cv2

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_menu import MPMenuItem


def _opencv_has_gstreamer():
    for line in cv2.getBuildInformation().splitlines():
        if line.strip().startswith("GStreamer:"):
            return line.split(":", 1)[1].strip().upper().startswith("YES")
    return False


class VideoView:
    def __init__(self, module, camera, stream, uri, latency):
        self.module = module
        self.camera = camera
        self.stream = stream
        self.uri = uri
        title = "%s - %s" % (camera.label(), stream.name)
        self.image = MPImage(title=title, width=stream.resolution_h,
                             height=stream.resolution_v, auto_size=False,
                             auto_fit=True, can_zoom=False, fps=30)
        menu = self.image.get_popup_menu()
        menu.add(MPMenuItem("Take photo", returnkey="Camera:Photo"))
        menu.add(MPMenuItem("Toggle recording", returnkey="Camera:Record"))
        menu.add(MPMenuItem("Autofocus", returnkey="Camera:Autofocus"))
        self.image.set_popup_menu(menu)
        codec = "h265" if stream.encoding == 2 else "h264"
        if _opencv_has_gstreamer():
            # Match the proven mavproxy_SIYI CameraView path while selecting
            # the depayloader and decoder from VIDEO_STREAM_INFORMATION.
            pipeline = (
                "rtspsrc location={uri} latency={latency} protocols=tcp "
                "tcp-timeout=3000000 buffer-mode=auto ! rtp{codec}depay ! "
                "{codec}parse ! avdec_{codec} ! videoconvert ! "
                "video/x-raw,format=BGRx ! appsink"
            ).format(uri=uri, latency=max(0, latency), codec=codec)
            self.image.set_gstreamer(pipeline)
        else:
            # PyPI OpenCV builds generally omit CAP_GSTREAMER. Passing the URI
            # directly lets MPImage use OpenCV's available FFmpeg backend.
            self.image.set_video(uri)

    def alive(self):
        return self.image is not None and self.image.is_alive()

    def close(self):
        if self.image is not None:
            self.image.terminate()
            self.image = None

    def check_events(self):
        if not self.alive():
            return
        for event in self.image.events():
            if not isinstance(event, MPMenuItem):
                continue
            if event.returnkey == "Camera:Photo":
                self.module.cmd_photo([])
            elif event.returnkey == "Camera:Record":
                self.module.cmd_record(["toggle"])
            elif event.returnkey == "Camera:Autofocus":
                self.module.cmd_focus(["auto"])
