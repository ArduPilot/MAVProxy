"""
MAVProxy camera view
"""

import sys
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter

from MAVProxy.modules.mavproxy_camtrack.tracker_image import TrackerImage

from pymavlink import mavutil


class CameraView:
    """Handle a camera view"""

    def __init__(self, mpstate, title, rtsp_url, fps=30):
        self.mpstate = mpstate
        self.rtsp_url = rtsp_url

        # TODO: remove hardcoded display size
        display_width = 640
        display_height = 480

        self.frame_counter = -1

        # Commands to autopilot attached cameras and gimbals are addressed
        # to the autopilot component.
        self.camera_sysid = 1  # system id matches vehicle
        self.camera_cmpid = 1  # component id matches autopilot
        self.camera_deviceid = 1  # first flight-stack connected camera

        self.gimbal_sysid = 1  # system id matches vehicle
        self.gimbal_cmpid = 1  # component id matches autopilot
        self.gimbal_deviceid = 1  # first flight-stack connected gimbal

        self.im = TrackerImage(
            title=title,
            mouse_events=True,
            mouse_movement_events=False,
            width=display_width,
            height=display_height,
            key_events=True,
            can_drag=False,
            can_zoom=False,
            auto_size=False,
            auto_fit=True,
            fps=fps,
        )

        # Capture video
        # TODO: provide args to set gstreamer pipeline
        gst_pipeline = (
            f"rtspsrc location={self.rtsp_url} latency=50 "
            "! decodebin "
            "! videoconvert "
            "! video/x-raw,format=(string)BGR "
            "! videoconvert "
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )
        self.im.set_gstreamer(gst_pipeline)

    def set_tracked_rectangle(self, top_left_x, top_left_y, bot_right_x, bot_right_y):
        """Set the tracked rectangle (normalised coords)"""
        self.im.set_tracked_rectangle(top_left_x, top_left_y, bot_right_x, bot_right_y)

    def close(self):
        """Close the GUI"""
        self.im.terminate()

    def is_alive(self):
        """Check if the GUI process is alive"""
        return self.im.is_alive()

    def check_events(self):
        """Check for events"""
        if self.im is None:
            return
        if not self.is_alive():
            return
        for event in self.im.events():
            if isinstance(event, MPImageTrackPos):
                continue
            if isinstance(event, MPImageFrameCounter):
                self.frame_counter = event.frame
                continue
            if (
                hasattr(event, "ClassName")
                and event.ClassName == "wxMouseEvent"
                and event.leftIsDown
            ):
                track_size_pct = 10.0
                if event.shiftDown:
                    (xres, yres) = (event.shape[1], event.shape[0])
                    twidth = int(yres * 0.01 * track_size_pct)

                    self.im.end_tracking()
                    self.send_camera_stop_tracking()

                    self.im.start_tracker(event.X, event.Y, twidth, twidth)

                    # TODO: move / encapsulate
                    # print(f"xres: {xres}, yres: {yres}")
                    # print(f"event.X: {event.X}, event.Y: {event.X}, twidth: {twidth}")
                    top_left_x = event.X / xres
                    top_left_y = event.Y / yres
                    bot_right_x = (event.X + twidth) / xres
                    bot_right_y = (event.Y + twidth) / yres
                    self.send_camera_track_rectangle(
                        top_left_x, top_left_y, bot_right_x, bot_right_y
                    )

                elif event.controlDown:
                    self.im.end_tracking()
                    self.send_camera_stop_tracking()
                else:
                    pass

    def send_camera_track_point(self, point_x, point_y, radius):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT

        NOTE: point coords and radius are normalised to [0, 1]
        """
        src_sys = self.mpstate.master().source_system
        src_cmp = self.mpstate.master().source_component
        tgt_sys = self.camera_sysid
        tgt_cmp = self.camera_cmpid
        # print(
        #     f"Send COMMAND_LONG: CAMERA_TRACK_POINT: "
        #     f"src_sys: {src_sys}, src_cmp: {src_cmp} "
        #     f"tgt_sys: {tgt_sys}, tgt_cmp: {tgt_cmp}"
        # )
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_cmp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,  # command
            0,  # confirmation
            point_x,  # param1
            point_y,  # param2
            radius,  # param3
            target_camera,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

    def send_camera_track_rectangle(
        self, top_left_x, top_left_y, bottom_right_x, bottom_right_y
    ):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE

        NOTE: coordinates are normalised to [0, 1]
        """
        src_sys = self.mpstate.master().source_system
        src_cmp = self.mpstate.master().source_component
        tgt_sys = self.camera_sysid
        tgt_cmp = self.camera_cmpid
        # print(
        #     f"Send COMMAND_LONG: CAMERA_TRACK_RECTANGLE: "
        #     f"src_sys: {src_sys}, src_cmp: {src_cmp} "
        #     f"tgt_sys: {tgt_sys}, tgt_cmp: {tgt_cmp}"
        # )
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_cmp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE,  # command
            0,  # confirmation
            top_left_x,  # param1
            top_left_y,  # param2
            bottom_right_x,  # param3
            bottom_right_y,  # param4
            target_camera,  # param5
            0,  # param6
            0,  # param7
        )

    def send_camera_stop_tracking(self):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING
        """
        src_sys = self.mpstate.master().source_system
        src_cmp = self.mpstate.master().source_component
        tgt_sys = self.camera_sysid
        tgt_cmp = self.camera_cmpid
        # print(
        #     f"Send COMMAND_LONG: CAMERA_STOP_TRACKING: "
        #     f"src_sys: {src_sys}, src_cmp: {src_cmp} "
        #     f"tgt_sys: {tgt_sys}, tgt_cmp: {tgt_cmp}"
        # )
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_cmp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING,  # command
            0,  # confirmation
            target_camera,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )


if __name__ == "__main__":
    from optparse import OptionParser

    parser = OptionParser("camera_view.py [options]")
    parser.add_option("--master", default=None, type=str, help="MAVLink device")
    parser.add_option("--rtsp-server", default=None, type=str, help="RTSP server URL")
    parser.add_option("--sysid", default=255, type=int, help="Source system ID")
    parser.add_option("--cmpid", default=1, type=int, help="Source component ID")

    (opts, args) = parser.parse_args()
    if opts.master is None:
        print("Must specify a MAVLink device")
        sys.exit(1)
    if opts.rtsp_server is None:
        print("Must specify an RTSP server URL")
        sys.exit(1)

    class MockMPState:
        def __init__(self, device, sysid, compid):
            self._device = device
            self._sysid = sysid
            self._cmpid = compid

            self._connection = mavutil.mavlink_connection(
                self._device,
                source_system=self._sysid,
                source_component=self._cmpid,
            )
            print("Searching for vehicle")
            while not self._connection.probably_vehicle_heartbeat(
                self._connection.wait_heartbeat()
            ):
                print(".", end="")
            print("\nFound vehicle")
            print(
                "Heartbeat received (system: {} component: {})".format(
                    self._connection.target_system, self._connection.target_component
                )
            )

        def master(self):
            return self._connection

    mpstate = MockMPState(opts.master, opts.sysid, opts.cmpid)
    camera_view = CameraView(mpstate, "Camera View", opts.rtsp_server)

    while True:
        time.sleep(0.1)
        camera_view.check_events()
