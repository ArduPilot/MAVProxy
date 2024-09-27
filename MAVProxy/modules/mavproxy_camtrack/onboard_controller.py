"""
An onboard controller for managing a tracking camera and gimbal.

Usage
-----

1. Run controller on localhost using an RTSP stream from a Gazebo simulation. 

  python ./onboard_controller.py
      --master 127.0.0.1:14550
      --rtsp-server rtsp://127.0.0.1:8554/camera

2. Run controller on localhost using an RTSP stream from a Herelink
connected to a home WiFi network. 

  python ./onboard_controller.py
      --master 127.0.0.1:14550
      --rtsp-server rtsp://192.168.1.204:8554/fpv_stream

3. Run controller on RPi4 companion computer using an RTSP stream from
a SIYI A8 camea. 

  python ./onboard_controller.py
      --master 192.168.144.171:15001
      --rtsp-server rtsp://192.168.144.25:8554/main.264

Examples 
--------  

Example devices and RTSP servers used in testing.

localhost
device = "127.0.0.1:14550

companion computer - NET virtual serial port
device = "192.168.144.171:15001"

localhost simulation
rtsp_url = "rtsp://127.0.0.1:8554/camera"

home wifi
rtsp_url = "rtsp://192.168.1.204:8554/fpv_stream"

herelink wifi access point
rtsp_url = "rtsp://192.168.43.1:8554/fpv_stream"

SIYI A8 camera
rtsp_url = "rtsp://192.168.144.25:8554/main.264"
"""

import copy
import cv2
import gi
import math
import numpy as np
import queue
import threading
import time

from enum import Enum
from pymavlink import mavutil

from transforms3d import euler
from transforms3d import quaternions

from MAVProxy.modules.lib import mp_util

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class CameraCapFlags(Enum):
    """
    https://mavlink.io/en/messages/common.html#CAMERA_CAP_FLAGS
    """

    CAPTURE_VIDEO = mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_VIDEO
    CAPTURE_IMAGE = mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE
    HAS_MODES = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_MODES
    CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = (
        mavutil.mavlink.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE
    )
    CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = (
        mavutil.mavlink.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE
    )
    HAS_IMAGE_SURVEY_MODE = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE
    HAS_BASIC_ZOOM = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
    HAS_BASIC_FOCUS = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS
    HAS_VIDEO_STREAM = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
    HAS_TRACKING_POINT = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_TRACKING_POINT
    HAS_TRACKING_RECTANGLE = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE
    HAS_TRACKING_GEO_STATUS = mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS


class CameraTrackingStatusFlags(Enum):
    """
    https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_STATUS_FLAGS
    """

    IDLE = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_IDLE
    ACTIVE = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_ACTIVE
    ERROR = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_ERROR


class CameraTrackingMode(Enum):
    """
    https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_MODE
    """

    NONE = mavutil.mavlink.CAMERA_TRACKING_MODE_NONE
    POINT = mavutil.mavlink.CAMERA_TRACKING_MODE_POINT
    RECTANGLE = mavutil.mavlink.CAMERA_TRACKING_MODE_RECTANGLE


class CameraTrackingTargetData(Enum):
    """
    https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_TARGET_DATA
    """

    NONE = mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA_NONE
    EMBEDDED = mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA_EMBEDDED
    RENDERED = mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA_RENDERED
    IN_STATUS = mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA_IN_STATUS


class OnboardController:
    def __init__(self, device, sysid, cmpid, rtsp_url):
        self._device = device
        self._sysid = sysid
        self._cmpid = cmpid
        self._rtsp_url = rtsp_url

        # mavlink connection
        self._connection = None

        # list of controllers to forward mavlink message to
        self._controllers = []
        self._camera_controller = None
        self._gimbal_controller = None

        print(f"Onboard Controller: src_sys: {self._sysid}, src_cmp: {self._cmpid})")

    def _connect_to_mavlink(self):
        """
        Establish a mavlink connection.
        """
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

    def _send_heartbeat_task(self):
        """
        Task to send a heartbeat identifying this as an onboard controller.
        """
        while True:
            self._connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_UNINIT,
                mavlink_version=3,
            )
            time.sleep(1.0)

    def _mavlink_recv_task(self):
        """
        Task to receive a mavlink message and forwward to controllers.
        """
        update_rate = 1000.0
        update_period = 1.0 / update_rate

        while True:

            def __process_message():
                msg = self._connection.recv_match(blocking=True)
                # Apply filters
                if (
                    msg is None
                    or msg.get_type() == "BAD_DATA"
                    # or not hasattr(msg, "target_system")
                    # or not hasattr(msg, "target_component")
                    # or msg.target_system != sysid
                    # or msg.target_component != cmpid
                ):
                    return

                for controller in self._controllers:
                    if not hasattr(controller, "mavlink_packet"):
                        return
                    controller.mavlink_packet(msg)

            start_time = time.time()
            __process_message()
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)

    def run(self):
        """
        Run the onboard controller.

        Connects to mavlink and a video stream then monitors camera track
        requests and updates a tracker and gimbal control as required.
        """
        self._connect_to_mavlink()

        # Connect to video stream
        video_stream = VideoStream(self._rtsp_url)

        # TODO: add retry limit and timeout
        print("Waiting for video stream")
        while not video_stream.frame_available():
            print(".", end="")
            time.sleep(0.1)
        print("\nVideo stream available")

        # Create and register controllers
        self._camera_controller = CameraTrackController(self._connection)
        self._gimbal_controller = GimbalController(self._connection)
        self._controllers.append(self._camera_controller)
        self._controllers.append(self._gimbal_controller)

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self._send_heartbeat_task)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        # Start the mavlink_recv thread
        mavlink_recv_thread = threading.Thread(target=self._mavlink_recv_task)
        mavlink_recv_thread.daemon = True
        mavlink_recv_thread.start()

        # Create tracker
        tracker = TrackerCSTR()
        tracking_rect = None
        tracking_rect_changed = True

        # TODO: ensure consistency of frame updates with GCS.
        fps = 50
        update_rate = fps
        update_period = 1.0 / update_rate
        frame_count = 0
        av_update_time = 0.0
        while True:
            start_time = time.time()

            if video_stream.frame_available():
                frame_count += 1
                frame = copy.deepcopy(video_stream.frame())

                track_type = self._camera_controller.track_type()
                if track_type is CameraTrackType.NONE:
                    if tracking_rect is not None:
                        tracking_rect = None
                        self._gimbal_controller.reset()

                elif track_type is CameraTrackType.RECTANGLE:
                    # TODO: not handling when the tracking rectange changes
                    if tracking_rect is not None:

                        def __compare_rect(rect1, rect2):
                            return (
                                math.isclose(rect1.top_left_x, rect2.top_left_x)
                                and math.isclose(rect1.top_left_y, rect2.top_left_y)
                                and math.isclose(rect1.bot_right_x, rect2.bot_right_x)
                                and math.isclose(rect1.bot_right_y, rect2.bot_right_y)
                            )

                        if not __compare_rect(
                            tracking_rect, self._camera_controller.track_rectangle()
                        ):
                            tracking_rect_changed = True

                    if tracking_rect is None or tracking_rect_changed:
                        tracking_rect_changed = False
                        tracking_rect = self._camera_controller.track_rectangle()
                        nroi = [
                            tracking_rect.top_left_x,
                            tracking_rect.top_left_y,
                            tracking_rect.bot_right_x - tracking_rect.top_left_x,
                            tracking_rect.bot_right_y - tracking_rect.top_left_y,
                        ]
                        tracker.set_normalised_roi(nroi)

                # update tracker and gimbal if tracking active
                if tracking_rect is not None:
                    success, box = tracker.update(frame)
                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        u = x + w // 2
                        v = y + h // 2
                        self._gimbal_controller.update_center(u, v, frame.shape)
                    else:
                        print("Tracking failure detected.")
                        tracking_rect = None
                        self._gimbal_controller.reset()

                # TODO: profiling stats
                # if frame_count % 10 == 0:
                #     av_update_time = av_update_time / 10.0 * 1000
                #     print(f"gimbal controller update: {av_update_time:.0f} ms")
                #     av_update_time = 0.0

            # Rate limit
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)
            av_update_time += elapsed_time


class VideoStream:
    """
    BlueRov video capture class. Adapted to capture a RTSP stream.

    Attributes:
        rtsp_url (string): RTSP URL
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, rtsp_url, latency=50):
        Gst.init(None)

        self.rtsp_url = rtsp_url
        self.latency = latency

        self.latest_frame = self._new_frame = None

        self.video_source = f"rtspsrc location={rtsp_url} latency={latency}"

        # Python does not have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        # Create a sink to get data
        self.video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = [
                "videotestsrc ! decodebin",
                "! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
                "! appsink",
            ]

        command = " ".join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name("appsink0")

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (caps_structure.get_value("height"), caps_structure.get_value("width"), 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return array

    def frame(self):
        """Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """Get frame to update _new_frame"""

        self.start_gst(
            [
                self.video_source,
                self.video_decode,
                self.video_sink_conf,
            ]
        )

        self.video_sink.connect("new-sample", self.callback)

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


class CameraTrackType(Enum):
    """ "
    Camera track types.
    """

    NONE = 0
    POINT = 1
    RECTANGLE = 2


class CameraTrackPoint:
    """
    Camera track point (normalised)

    https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
    """

    def __init__(self, point_x, point_y, radius):
        self.point_x = point_x
        self.point_y = point_y
        self.radius = radius


class CameraTrackRectangle:
    """
    Camera track rectangle (normalised)

    https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
    """

    def __init__(self, top_left_x, top_left_y, bot_right_x, bot_right_y):
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
        self.bot_right_x = bot_right_x
        self.bot_right_y = bot_right_y


class CameraTrackController:
    """
    Controller for onboard camera tracking.
    """

    def __init__(self, connection):
        self._connection = connection
        self._sysid = self._connection.source_system
        self._cmpid = self._connection.source_component

        print(
            f"Camera Track Controller: "
            f"src_sys: {self._sysid}, src_cmp: {self._cmpid}"
        )

        # TODO: this should be supplied by a camera when using hardware
        # camera information
        self._vendor_name = "SIYI"
        self._model_name = "A8"
        self._focal_length = float("nan")
        self._sensor_size_h = float("nan")
        self._sensor_size_v = float("nan")
        self._resolution_h = 0
        self._resolution_v = 0
        self._gimbal_device_id = 1

        # tracking details
        self._lock = threading.Lock()
        self._track_type = CameraTrackType.NONE
        self._track_point = None
        self._track_rect = None

        # Start the mavlink thread
        self._mavlink_in_queue = queue.Queue()
        self._mavlink_out_queue = queue.Queue()
        self._mavlink_thread = threading.Thread(target=self._mavlink_task)
        self._mavlink_thread.daemon = True
        self._mavlink_thread.start()

        # Start the send tracking status thread
        self._send_status_thread = threading.Thread(target=self._send_status_task)
        self._send_status_thread.daemon = True
        self._send_status_thread.start()

    def track_type(self):
        """
        Return the current track type.
        """
        with self._lock:
            track_type = copy.deepcopy(self._track_type)
        return track_type

    def track_point(self):
        """
        Return a point to track if the track type is POINT, else None.
        """
        with self._lock:
            track_point = copy.deepcopy(self._track_point)
        return track_point

    def track_rectangle(self):
        """
        Return a rectangle to track if the track type is RECTANGLE, else None.
        """
        with self._lock:
            track_rect = copy.deepcopy(self._track_rect)
        return track_rect

    def mavlink_packet(self, msg):
        """
        Process a mavlink packet.
        """
        self._mavlink_in_queue.put(msg)

    def _send_camera_information(self):
        """
        AP_Camera must receive camera information, including capability flags,
        before it will accept tracking requests.

        If MAV_CMD_CAMERA_TRACK_POINT or MAV_CMD_CAMERA_TRACK_RECTANGLE result
        in ACK UNSUPPORTED, then this may not have been sent.
        """
        flags = (
            CameraCapFlags.CAPTURE_VIDEO.value
            | CameraCapFlags.CAPTURE_IMAGE.value
            | CameraCapFlags.HAS_MODES.value
            | CameraCapFlags.CAN_CAPTURE_IMAGE_IN_VIDEO_MODE.value
            | CameraCapFlags.CAN_CAPTURE_VIDEO_IN_IMAGE_MODE.value
            | CameraCapFlags.HAS_IMAGE_SURVEY_MODE.value
            | CameraCapFlags.HAS_BASIC_ZOOM.value
            | CameraCapFlags.HAS_BASIC_FOCUS.value
            | CameraCapFlags.HAS_VIDEO_STREAM.value
            | CameraCapFlags.HAS_TRACKING_POINT.value
            | CameraCapFlags.HAS_TRACKING_RECTANGLE.value
            | CameraCapFlags.HAS_TRACKING_GEO_STATUS.value
        )

        def to_uint8_t(string):
            string_encode = string.encode("utf-8")
            return string_encode + b"\0" * (32 - len(string))

        # print(to_uint8_t(self.vendor_name), len(to_uint8_t(self.vendor_name)))

        self._connection.mav.camera_information_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            to_uint8_t(self._vendor_name),  # vendor_name
            to_uint8_t(self._model_name),  # model_name
            (1 << 24) | (0 << 16) | (0 << 8) | 1,  # firmware_version
            self._focal_length,  # focal_length
            self._sensor_size_h,  # sensor_size_h
            self._sensor_size_v,  # sensor_size_v
            self._resolution_h,  # resolution_h
            self._resolution_v,  # resolution_v
            0,  # lens_id
            flags,  # flags
            0,  # cam_definition_version
            b"",  # cam_definition_uri
            self._gimbal_device_id,  # gimbal_device_id
        )
        print("Sent camera information")

    def _send_camera_tracking_image_status(self):
        with self._lock:
            track_type = self._track_type
            track_point = self._track_point
            track_rect = self._track_rect

        # TODO: use trackers bounding box instead of initial target
        if track_type is CameraTrackType.NONE:
            tracking_status = CameraTrackingStatusFlags.IDLE.value
            tracking_mode = CameraTrackingMode.NONE.value
            target_data = CameraTrackingTargetData.NONE.value
            point_x = float("nan")
            point_y = float("nan")
            radius = float("nan")
            rec_top_x = float("nan")
            rec_top_y = float("nan")
            rec_bottom_x = float("nan")
            rec_bottom_y = float("nan")
        elif track_type is CameraTrackType.POINT:
            tracking_status = CameraTrackingStatusFlags.ACTIVE.value
            tracking_mode = CameraTrackingMode.POINT.value
            target_data = CameraTrackingTargetData.IN_STATUS.value
            point_x = track_point.point_x
            point_y = track_point.point_y
            radius = track_point.radius
            rec_top_x = float("nan")
            rec_top_y = float("nan")
            rec_bottom_x = float("nan")
            rec_bottom_y = float("nan")
        elif track_type is CameraTrackType.RECTANGLE:
            tracking_status = CameraTrackingStatusFlags.ACTIVE.value
            tracking_mode = CameraTrackingMode.RECTANGLE.value
            target_data = CameraTrackingTargetData.IN_STATUS.value
            point_x = float("nan")
            point_y = float("nan")
            radius = float("nan")
            rec_top_x = track_rect.top_left_x
            rec_top_y = track_rect.top_left_y
            rec_bottom_x = track_rect.bot_right_x
            rec_bottom_y = track_rect.bot_right_y

        msg = self._connection.mav.camera_tracking_image_status_encode(
            tracking_status,
            tracking_mode,
            target_data,
            point_x,
            point_y,
            radius,
            rec_top_x,
            rec_top_y,
            rec_bottom_x,
            rec_bottom_y,
        )
        self._connection.mav.send(msg)

    def _handle_camera_track_point(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_POINT")
        # Parameters are a normalised point.
        point_x = msg.param1
        point_y = msg.param2
        radius = msg.param3
        print(f"Track point: x: {point_x}, y: {point_y}, radius: {radius}")
        with self._lock:
            self._track_type = CameraTrackType.POINT
            self._track_point = CameraTrackPoint(point_x, point_y, radius)

    def _handle_camera_track_rectangle(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_RECTANGLE")
        # Parameters are a normalised rectangle.
        top_left_x = msg.param1
        top_left_y = msg.param2
        bot_right_x = msg.param3
        bot_right_y = msg.param4
        print(
            f"Track rectangle: x1: {top_left_x}, y1: {top_left_y}, "
            f"x2: {bot_right_x}, y2: {bot_right_y}"
        )
        with self._lock:
            self._track_type = CameraTrackType.RECTANGLE
            self._track_rect = CameraTrackRectangle(
                top_left_x, top_left_y, bot_right_x, bot_right_y
            )

    def _handle_camera_stop_tracking(self, msg):
        print("Got COMMAND_LONG: CAMERA_STOP_TRACKING")
        with self._lock:
            self._track_type = CameraTrackType.NONE
            self._track_point = None
            self._track_rect = None

    def _mavlink_task(self):
        """
        Process mavlink messages relevant to camera tracking.
        """
        self._send_camera_information()

        with self._lock:
            sysid = self._sysid
            cmpid = self._cmpid

        update_rate = 1000.0
        update_period = 1.0 / update_rate

        while True:

            def __process_message():
                if self._mavlink_in_queue.empty():
                    return

                msg = self._mavlink_in_queue.get()
                mtype = msg.get_type()

                if msg and mtype == "COMMAND_LONG":
                    if msg.target_system != sysid:
                        return
                    elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                        self._handle_camera_track_point(msg)
                    elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                        self._handle_camera_track_rectangle(msg)
                    elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING:
                        self._handle_camera_stop_tracking(msg)
                    else:
                        pass

            start_time = time.time()
            __process_message()
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)

    def _send_status_task(self):
        """
        Send camera tracking image status.
        """
        # TODO: stop sending image status when tracking stopped
        # TODO: set streaming rate using MAV_CMD_SET_MESSAGE_INTERVAL

        update_rate = 2.0
        update_period = 1.0 / update_rate

        while True:

            def __send_message():
                self._send_camera_tracking_image_status()

            start_time = time.time()
            __send_message()
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)


class GimbalController:
    """
    Gimbal controller for onboard camera tracking.
    """

    def __init__(self, connection):
        self._connection = connection
        self._sysid = self._connection.source_system
        self._cmpid = self._connection.source_component

        print(f"Gimbal Controller: src_sys: {self._sysid}, src_cmp: {self._cmpid})")

        # Shared variables
        self._control_lock = threading.Lock()
        self._center_x = 0
        self._center_y = 0
        self._width = None
        self._height = None
        self._tracking = False

        # gimbal state
        # TODO: obtain the mount neutral angles from params
        # NOTE: q = [w, x, y, z]
        self._mavlink_lock = threading.Lock()
        self._neutral_x = 0.0
        self._neutral_y = 0.0
        self._neutral_z = 0.0
        self._gimbal_device_flags = None
        self._gimbal_orientation = None
        self._gimbal_failure_flags = None

        # TODO: add options for PI controller gains
        # PI controllers: SIYI A8: 0.1, Gazebo: 0.3
        self._pitch_controller = PI_controller(Pgain=0.5, Igain=0.01, IMAX=1.0)
        self._yaw_controller = PI_controller(Pgain=0.5, Igain=0.01, IMAX=1.0)

        # Start the control thread
        self._control_in_queue = queue.Queue()
        self._control_out_queue = queue.Queue()
        self._control_thread = threading.Thread(target=self._control_task)
        self._control_thread.daemon = True
        self._control_thread.start()

        # Start the mavlink thread
        self._mavlink_thread = threading.Thread(target=self._mavlink_task)
        self._mavlink_thread.daemon = True
        self._mavlink_thread.start()

    def update_center(self, x, y, shape):
        with self._control_lock:
            self._tracking = True
            self._center_x = x
            self._center_y = y
            self._height, self._width, _ = shape
            # print(f"width: {self._width}, height: {self._height}, center: [{x}, {y}]")

    def reset(self):
        with self._control_lock:
            self._tracking = False
            self._center_x = 0.0
            self._center_y = 0.0
            self._pitch_controller.reset_I()
            self._yaw_controller.reset_I()

    def mavlink_packet(self, msg):
        self._control_in_queue.put(msg)

    def _send_gimbal_manager_pitch_yaw_angles(self, pitch, yaw, pitch_rate, yaw_rate):
        """
        Send a mavlink message to set the gimbal pitch and yaw (radians).
        """
        msg = self._connection.mav.gimbal_manager_set_pitchyaw_encode(
            self._connection.target_system,
            self._connection.target_component,
            0,
            0,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate,
        )
        self._connection.mav.send(msg)

    def _control_task(self):
        """
        Gimbal control task.

        When tracking, move the camera to centre on the bounding box
        around the tracked object.

        When not tracking, return the gimbal to its neutral position.
        """
        while True:
            # Record the start time of the loop
            start_time = time.time()

            # Copy shared variables
            with self._control_lock:
                act_x = self._center_x
                act_y = self._center_y
                frame_width = self._width
                frame_height = self._height
                tracking = self._tracking

            # Return gimbal to its neutral orientation when not tracking
            with self._mavlink_lock:
                gimbal_orientation = self._gimbal_orientation

            if not tracking and gimbal_orientation is not None:
                # NOTE: to centre the gimbal when not tracking, we need to know
                # the neutral angles from the MNT1_NEUTRAL_x params, and also
                # the current mount orientation.
                _, ay, az = euler.quat2euler(gimbal_orientation)

                err_pitch = self._neutral_y - ay
                pitch_rate_rads = self._pitch_controller.run(err_pitch)

                err_yaw = self._neutral_z - az
                yaw_rate_rads = self._yaw_controller.run(err_yaw)

                self._send_gimbal_manager_pitch_yaw_angles(
                    float("nan"),
                    float("nan"),
                    pitch_rate_rads,
                    yaw_rate_rads,
                )
            elif frame_width is not None and frame_height is not None:
                tgt_x = 0.5 * frame_width
                tgt_y = 0.5 * frame_height

                if math.isclose(act_x, 0.0) and math.isclose(act_y, 0.0):
                    err_x = 0.0
                    err_y = 0.0
                else:
                    err_x = act_x - tgt_x
                    err_y = -(act_y - tgt_y)

                err_pitch = math.radians(err_y)
                pitch_rate_rads = self._pitch_controller.run(err_pitch)

                err_yaw = math.radians(err_x)
                yaw_rate_rads = self._yaw_controller.run(err_yaw)

                # print(
                #     f"err_x: {err_x}, err_y: {err_y}, "
                #     f"pitch_rate: {pitch_rate_rads}, yaw_rate: {yaw_rate_rads}"
                # )

                self._send_gimbal_manager_pitch_yaw_angles(
                    float("nan"),
                    float("nan"),
                    pitch_rate_rads,
                    yaw_rate_rads,
                )

            # Update at 50Hz
            update_period = 0.02
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)

    def _mavlink_task(self):
        """
        Process mavlink messages relevant to gimbal management.
        """
        update_rate = 1000.0
        update_period = 1.0 / update_rate

        while True:

            def __process_message():
                if self._control_in_queue.empty():
                    return
                msg = self._control_in_queue.get()
                mtype = msg.get_type()
                # NOTE: GIMBAL_DEVICE_ATTITUDE_STATUS is broadcast
                #       (sysid=0, cmpid=0)
                if msg and mtype == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                    with self._mavlink_lock:
                        self._gimbal_device_flags = msg.flags
                        self._gimbal_orientation = msg.q
                        self._gimbal_failure_flags = msg.failure_flags

            start_time = time.time()
            __process_message()
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)


class PI_controller:
    """
    Simple PI controller

    MAVProxy/modules/mavproxy_SIYI/PI_controller (modified)
    """

    def __init__(self, Pgain, Igain, IMAX, gain_mul=1.0, max_rate=math.radians(30.0)):
        self.Pgain = Pgain
        self.Igain = Igain
        self.IMAX = IMAX
        self.gain_mul = gain_mul
        self.max_rate = max_rate
        self.I = 0.0

        self.last_t = time.time()

    def run(self, err, ff_rate=0.0):
        now = time.time()
        dt = now - self.last_t
        if now - self.last_t > 1.0:
            self.reset_I()
            dt = 0
        self.last_t = now
        P = self.Pgain * self.gain_mul
        I = self.Igain * self.gain_mul
        IMAX = self.IMAX
        max_rate = self.max_rate

        out = P * err
        saturated = err > 0 and (out + self.I) >= max_rate
        saturated |= err < 0 and (out + self.I) <= -max_rate
        if not saturated:
            self.I += I * err * dt
        self.I = mp_util.constrain(self.I, -IMAX, IMAX)
        ret = out + self.I + ff_rate
        return mp_util.constrain(ret, -max_rate, max_rate)

    def reset_I(self):
        self.I = 0


class TrackerCSTR:
    """
    Wrapper for cv2.legacy.TrackerCSRT
    """

    def __init__(self):
        self._tracker = cv2.legacy.TrackerCSRT_create()
        self._nroi = None
        self._nroi_changed = False

    def update(self, frame):
        if self._nroi is None or frame is None:
            return False, None

        if self._nroi_changed:
            self._tracker = cv2.legacy.TrackerCSRT_create()
            # denormalise the roi
            height, width, _ = frame.shape
            roi = [
                int(self._nroi[0] * width),  # x
                int(self._nroi[1] * height),  # y
                int(self._nroi[2] * width),  # w
                int(self._nroi[3] * height),  # h
            ]
            print(
                f"TrackerCSTR: nroi: {self._nroi}, roi: {roi}, frame_width: {width}, frame_height: {height}"
            )
            self._tracker.init(frame, roi)
            self._nroi_changed = False

        return self._tracker.update(frame)

    def set_normalised_roi(self, nroi):
        """
        Set the region of interest

        [top_left_x, top_left_y, width, height]
        """
        self._nroi = nroi
        self._nroi_changed = True


if __name__ == "__main__":
    from optparse import OptionParser

    parser = OptionParser("onboard_controller.py [options]")
    parser.add_option("--master", default=None, type=str, help="MAVLink device")
    parser.add_option("--rtsp-server", default=None, type=str, help="RTSP server URL")
    parser.add_option("--sysid", default=1, type=int, help="Source system ID")
    parser.add_option(
        "--cmpid",
        default=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        type=int,
        help="Source component ID",
    )

    (opts, args) = parser.parse_args()
    if opts.master is None:
        print("Must specify a MAVLink device")
        sys.exit(1)
    if opts.rtsp_server is None:
        print("Must specify an RTSP server URL")
        sys.exit(1)

    controller = OnboardController(
        opts.master, opts.sysid, opts.cmpid, opts.rtsp_server
    )
    controller.run()
