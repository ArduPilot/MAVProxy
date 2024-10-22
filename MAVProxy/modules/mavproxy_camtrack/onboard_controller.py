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
a SIYI A8 camera. 

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
import re
import threading
import time

from enum import Enum
from pymavlink import mavutil

# TODO: remember to add to requirements.txt / setup.py
from transforms3d import euler
from transforms3d import quaternions

from MAVProxy.modules.lib import live_graph

from MAVProxy.modules.lib.pid_basic import AC_PID_Basic
from MAVProxy.modules.lib.pid_basic import AP_PIDInfo
from MAVProxy.modules.lib.pid_basic import constrain_float


gi.require_version("Gst", "1.0")
from gi.repository import Gst

# The main loop rate (Hz) is max rate that the main loop will run at.
# In practice it is limited by the video framerate and tracker update time.
MAIN_LOOP_RATE = 100.0

# The requested rates (Hz) for the current gimbal attitude and rate at which
# the commands to update the gimbal pitch and yaw are sent.
GIMBAL_CONTROL_LOOP_RATE = 20.0
GIMBAL_DEVICE_ATTITUDE_STATUS_RATE = 30.0

# The rate (Hz) to send information about the camera image.
CAMERA_SEND_IMAGE_STATUS_RATE = 10.0

# The rate (Hz) at which mavlink messages are pulled off the input queues.
MAVLINK_RECV_RATE = 1000.0


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


class Profiler:
    """
    Basic profiler for measuring execution times.
    """

    def __init__(self):
        self._has_started = False
        self._has_ended = True
        self._start_time_s = 0

        self._count = 0
        self._last_duration_s = -1.0
        self._total_duration_s = 0.0

    def start(self):
        if self._has_started or not self._has_ended:
            raise Exception(f"Profiler already in progress")
            return

        self._has_started = True
        self._has_ended = False
        self._start_time_s = time.time()

    def end(self):
        if not self._has_started or self._has_ended:
            raise Exception(f"Profiler cannot end before start")
            return

        self._has_started = False
        self._has_ended = True
        dt = time.time() - self._start_time_s
        self._count += 1
        self._last_duration_s = dt
        self._total_duration_s += dt

    @property
    def count(self):
        return self._count

    @property
    def last_duration_ms(self):
        if self._last_duration_s >= 0:
            return int(self._last_duration_s * 1e3)
        else:
            return int(-1)

    @property
    def last_duration_us(self):
        if self._last_duration_s >= 0:
            return int(self._last_duration_s * 1e6)
        else:
            return int(-1)

    @property
    def avg_duration_ms(self):
        if self._count > 0:
            return int(self._total_duration_s / self._count * 1e3)
        else:
            return int(-1)

    @property
    def avg_duration_us(self):
        if self._count > 0:
            return int(self._total_duration_s / self._count * 1e6)
        else:
            return int(-1)


def cv2_has_gstreamer():
    """
    Check whether cv2 was compiled with GStreamer support.

    The check involves searching the cv2 build information for a line
    such as:

    GStreamer:                   YES (1.24.7)
    """
    build_info = cv2.getBuildInformation()
    match = re.search(".*GStreamer.*", build_info)
    if match is None:
        return False

    match = re.search("YES", match.group(0))
    if match is None:
        return False

    return match.group(0) == "YES"


class OnboardController:
    """
    Onboard gimbal controller.
    """

    def __init__(
        self,
        device,
        sysid,
        cmpid,
        rtsp_url,
        tracker_name="CSTR",
        enable_graphs=False,
        enable_profiler=False,
        enable_rpi_pipeline=False,
    ):
        self._device = device
        self._sysid = sysid
        self._cmpid = cmpid
        self._rtsp_url = rtsp_url
        self._tracker_name = tracker_name
        self._enable_graphs = enable_graphs
        self._enable_profiler = enable_profiler
        self._enable_rpi_pipeline = enable_rpi_pipeline

        # mavlink connection
        self._connection = None

        # list of controllers to forward mavlink messages
        self._controllers = []
        self._camera_controller = None
        self._gimbal_controller = None

        print(f"Onboard Controller: src_sys: {self._sysid}, src_cmp: {self._cmpid})")

        self._cv2_has_gstreamer = cv2_has_gstreamer()
        print(f"OpenCV has GStreamer: {self._cv2_has_gstreamer}")

    def __del__(self):
        if self._connection is not None:
            self._connection.close()

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
        update_rate = MAVLINK_RECV_RATE  # Hz
        update_period = 1.0 / update_rate

        while True:

            def __process_message():
                msg = self._connection.recv_match(blocking=True)
                # Apply filters
                if msg is None or msg.get_type() == "BAD_DATA":
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
        print(f"Using RTSP server: {self._rtsp_url}")
        if self._cv2_has_gstreamer:
            if self._enable_rpi_pipeline:
                # Pipeline for RPi4 using hardware decode / convert
                gst_pipeline = (
                    f"rtspsrc location={self._rtsp_url} latency=50 "
                    f"! rtph264depay "
                    f"! h264parse "
                    f"! v4l2h264dec "
                    f"! v4l2convert "
                    f"! videoscale "
                    f"! videorate "
                    f"! video/x-raw,format=BGR,width=640,height=360,framerate=10/1 "
                    f"! appsink emit-signals=true sync=false max-buffers=2 drop=true"
                )
            else:
                # Generic autodetected pipeline
                gst_pipeline = (
                    f"rtspsrc location={self._rtsp_url} latency=50 "
                    f"! decodebin "
                    f"! videoconvert "
                    f"! videoscale "
                    f"! videorate "
                    f"! video/x-raw,format=BGR,width=640,height=360,framerate=10/1 "
                    "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
                )

            # From mavproxy_SIYI for H.265.
            # NOTE: RPi4 does not have a hardware decoder for H.265 suitable for
            #       GStreamer.
            # gst_pipeline = (
            #     f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=auto "
            #     f"! rtph265depay "
            #     f"! queue "
            #     f"! h265parse "
            #     f"! avdec_h265  "
            #     f"! videoconvert "
            #     f"! video/x-raw,format=BGRx "
            #     f"! appsink"
            # )

            cap_options = cv2.CAP_GSTREAMER
            video_stream = cv2.VideoCapture(gst_pipeline, cap_options)
            if not video_stream or not video_stream.isOpened():
                print("\nVideo stream not available")
                return
        else:
            video_stream = VideoStream(self._rtsp_url)
            # Timeout if video not available
            video_last_update_time = time.time()
            video_timeout_period = 5.0
            print("Waiting for video stream")
            while not video_stream.frame_available():
                print(".", end="")
                if (time.time() - video_last_update_time) > video_timeout_period:
                    print("\nVideo stream not available")
                    return
                time.sleep(0.1)

        print("\nVideo stream available")

        # Create and register controllers
        self._camera_controller = CameraTrackController(self._connection)
        self._gimbal_controller = GimbalController(
            self._connection, self._enable_graphs, self._enable_profiler
        )
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
        print(f"Using tracker: {self._tracker_name}")
        tracker = TrackerFactory.create_tracker(self._tracker_name)
        tracking_rect = None
        tracking_rect_changed = True

        # TODO: how to ensure consistency of frame updates with GCS?
        update_rate = MAIN_LOOP_RATE  # Hz
        update_period = 1.0 / update_rate

        # TODO: consolidate common code - also used in GUI
        # request gimbal device attitude status
        interval_us = int(1e6 / GIMBAL_DEVICE_ATTITUDE_STATUS_RATE)
        self.send_set_message_interval_message(
            mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
            interval_us,  # requested interval in microseconds
            response_target=1,  # flight-stack default
        )

        # Always collect profile data (low-cost)
        tracker_profiler = Profiler()
        loop_profiler = Profiler()
        loop_with_tracker_profiler = Profiler()
        profile_print_last_time = time.time()
        profile_print_period = 2

        fps_print_last_time = time.time()
        fps_print_period = 2
        fps_start_time = time.time()
        frame_count = 0

        while True:
            loop_start_time = time.time()

            # Check next frame available
            if self._cv2_has_gstreamer:
                frame_available, frame = video_stream.read()
            else:
                frame_available = video_stream.frame_available()

            if frame_available:
                frame_count += 1

                loop_profiler.start()

                # Get RGB frame
                if self._cv2_has_gstreamer:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    frame = video_stream.frame()
    
                track_type = self._camera_controller.track_type()
                if track_type is CameraTrackType.NONE:
                    if tracking_rect is not None:
                        tracking_rect = None
                        self._gimbal_controller.reset()

                elif track_type is CameraTrackType.RECTANGLE:
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
                    loop_with_tracker_profiler.start()

                    tracker_profiler.start()
                    success, box = tracker.update(frame)
                    tracker_profiler.end()

                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        u = x + w // 2
                        v = y + h // 2
                        self._gimbal_controller.update_center(u, v, frame.shape)

                        # update camera controller current tracking state
                        frame_height, frame_width = frame.shape[:2]
                        top_left_x = x / frame_width
                        top_left_y = y / frame_height
                        bot_right_x = (x + w) / frame_width
                        bot_right_y = (y + h) / frame_height
                        self._camera_controller.set_curr_track_rectangle(
                            CameraTrackRectangle(
                                top_left_x, top_left_y, bot_right_x, bot_right_y
                            )
                        )
                    else:
                        print("Tracking failure detected.")
                        tracking_rect = None
                        self._camera_controller.stop_tracking()
                        self._gimbal_controller.reset()
                    loop_with_tracker_profiler.end()

                loop_profiler.end()

            # Profile
            if self._enable_profiler and (
                (time.time() - profile_print_last_time) > profile_print_period
            ):
                profile_print_last_time = time.time()
                print("Onboard controller main loop profile")
                print("-------------------------------------")
                print("time in ms      avg\tlast\tcount")
                print(
                    f"loop:            "
                    f"{loop_profiler.avg_duration_ms}\t"
                    f"{loop_profiler.last_duration_ms}\t"
                    f"{loop_profiler.count}"
                )
                print(
                    f"loop_with_track: "
                    f"{loop_with_tracker_profiler.avg_duration_ms}\t"
                    f"{loop_with_tracker_profiler.last_duration_ms}\t"
                    f"{loop_with_tracker_profiler.count}"
                )
                print(
                    f"tracker:         "
                    f"{tracker_profiler.avg_duration_ms}\t"
                    f"{tracker_profiler.last_duration_ms}\t"
                    f"{tracker_profiler.count}"
                )
                print("-------------------------------------")
                print()

            # FPS
            if (time.time() - fps_print_last_time) > fps_print_period:
                fps_print_last_time = time.time()
                fps = frame_count / (time.time() - fps_start_time)
                print(f"fps: {fps:.1f}, count: {frame_count}, shape: {frame.shape}")

            # Rate limit
            elapsed_time = time.time() - loop_start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)

    def send_set_message_interval_message(
        self, message_id, interval_us, response_target=1
    ):
        self._connection.mav.command_long_send(
            self._connection.target_system,  # target_system
            self._connection.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_id,  # param1
            interval_us,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            response_target,  # param7
        )


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
        self._sysid = self._connection.source_system  # system id matches vehicle
        self._cmpid = 1  # component id matches autopilot

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

        # Tracking details - tracking requested
        self._lock = threading.Lock()
        self._track_type = CameraTrackType.NONE
        self._track_point = None
        self._track_rect = None

        # Current track state
        self._curr_track_point = None
        self._curr_track_rect = None

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

    def set_curr_track_point(self, point):
        """
        Set the currently tracked point.
        """
        with self._lock:
            self._curr_track_point = copy.deepcopy(point)

    def set_curr_track_rectangle(self, rect):
        """
        Set the currently tracked rectangle.
        """
        with self._lock:
            self._curr_track_rect = copy.deepcopy(rect)

    def stop_tracking(self):
        print("CAMERA_STOP_TRACKING")
        with self._lock:
            self._track_type = CameraTrackType.NONE
            self._track_point = None
            self._track_rect = None
            self._curr_track_point = None
            self._curr_track_rect = None

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
            track_point = self._curr_track_point
            track_rect = self._curr_track_rect

        # Set default values
        point_x = float("nan")
        point_y = float("nan")
        radius = float("nan")
        rec_top_x = float("nan")
        rec_top_y = float("nan")
        rec_bottom_x = float("nan")
        rec_bottom_y = float("nan")

        # TODO: track_point and track_rect should not be None
        #       if track_type is not NONE - poss sync issue?
        if track_type is CameraTrackType.NONE:
            tracking_status = CameraTrackingStatusFlags.IDLE.value
            tracking_mode = CameraTrackingMode.NONE.value
            target_data = CameraTrackingTargetData.NONE.value
        elif track_type is CameraTrackType.POINT and track_point is not None:
            tracking_status = CameraTrackingStatusFlags.ACTIVE.value
            tracking_mode = CameraTrackingMode.POINT.value
            target_data = CameraTrackingTargetData.IN_STATUS.value
            point_x = track_point.point_x
            point_y = track_point.point_y
            radius = track_point.radius
        elif track_type is CameraTrackType.RECTANGLE and track_rect is not None:
            tracking_status = CameraTrackingStatusFlags.ACTIVE.value
            tracking_mode = CameraTrackingMode.RECTANGLE.value
            target_data = CameraTrackingTargetData.IN_STATUS.value
            rec_top_x = track_rect.top_left_x
            rec_top_y = track_rect.top_left_y
            rec_bottom_x = track_rect.bot_right_x
            rec_bottom_y = track_rect.bot_right_y
        else:
            return

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
        # NOTE: leave for debugging
        # print(
        #     f"image_status: x: {rec_top_x:.2f}, y: {rec_top_y:.2f}, "
        #     f"w: {(rec_bottom_x - rec_top_x):.2f}, "
        #     f"h: {(rec_bottom_y - rec_top_y):.2f}"
        # )
        self._connection.mav.send(msg)

    def _handle_camera_track_point(self, msg):
        # Parameters are a normalised point.
        point_x = msg.param1
        point_y = msg.param2
        radius = msg.param3
        print(
            f"CAMERA_TRACK_POINT: x: {point_x:.3f}, y: {point_y:.3f}, "
            f"radius: {radius:.3f}"
        )
        with self._lock:
            self._track_type = CameraTrackType.POINT
            self._track_point = CameraTrackPoint(point_x, point_y, radius)

    def _handle_camera_track_rectangle(self, msg):
        # Parameters are a normalised rectangle.
        top_left_x = msg.param1
        top_left_y = msg.param2
        bot_right_x = msg.param3
        bot_right_y = msg.param4
        print(
            f"CAMERA_TRACK_RECTANGLE: x1: {top_left_x:.3f}, y1: {top_left_y:.3f}, "
            f"x2: {bot_right_x:.3f}, y2: {bot_right_y:.3f}"
        )
        with self._lock:
            self._track_type = CameraTrackType.RECTANGLE
            self._track_rect = CameraTrackRectangle(
                top_left_x, top_left_y, bot_right_x, bot_right_y
            )

    def _handle_camera_stop_tracking(self, msg):
        self.stop_tracking()

    def _mavlink_task(self):
        """
        Process mavlink messages relevant to camera tracking.
        """
        self._send_camera_information()

        with self._lock:
            sysid = self._sysid
            cmpid = self._cmpid

        update_rate = MAVLINK_RECV_RATE  # Hz
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

        update_rate = CAMERA_SEND_IMAGE_STATUS_RATE  # Hz
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

    def __init__(self, connection, enable_graphs=False, enable_profiler=False):
        self._connection = connection
        self._sysid = self._connection.source_system  # system id matches vehicle
        self._cmpid = 1  # component id matches autopilot

        print(f"Gimbal Controller: src_sys: {self._sysid}, src_cmp: {self._cmpid})")

        # Shared variables
        self._control_lock = threading.Lock()
        self._center_x = 0
        self._center_y = 0
        self._frame_width = None
        self._frame_height = None
        self._tracking = False

        # Gimbal state
        # TODO: obtain the mount neutral angles from params
        # NOTE: q = [w, x, y, z]
        self._mavlink_lock = threading.Lock()
        self._mnt1_neutral_x = 0.0  # roll
        self._mnt1_neutral_y = 0.0  # pitch
        self._mnt1_neutral_z = 0.0  # yaw
        self._gimbal_device_flags = None
        self._gimbal_orientation = None
        self._gimbal_failure_flags = None

        # TODO: add options for PID controller gains

        # Pitch controller for centring gimbal
        self._pit_controller = AC_PID_Basic(
            initial_p=2.0,
            initial_i=0.01,
            initial_d=0.05,
            initial_ff=0.0,
            initial_imax=1.0,
            initial_filt_E_hz=20.0,
            initial_filt_D_hz=20.0,
        )

        # Yaw controller for centring gimbal
        self._yaw_controller = AC_PID_Basic(
            initial_p=2.0,
            initial_i=0.01,
            initial_d=0.05,
            initial_ff=0.0,
            initial_imax=1.0,
            initial_filt_E_hz=20.0,
            initial_filt_D_hz=20.0,
        )

        # Gimbal pitch controller for tracking
        self._pit_track_controller = AC_PID_Basic(
            initial_p=1.5,
            initial_i=0.01,
            initial_d=0.05,
            initial_ff=0.0,
            initial_imax=1.0,
            initial_filt_E_hz=20.0,
            initial_filt_D_hz=20.0,
        )

        # Gimbal yaw controller for tracking
        self._yaw_track_controller = AC_PID_Basic(
            initial_p=1.5,
            initial_i=0.01,
            initial_d=0.05,
            initial_ff=0.0,
            initial_imax=1.0,
            initial_filt_E_hz=20.0,
            initial_filt_D_hz=20.0,
        )

        # Profiler
        self._enable_profiler = enable_profiler

        # Analysis
        self._enable_graphs = enable_graphs
        self._graph_pid_tune = None
        self._graph_pid_pit = None
        self._graph_pid_yaw = None

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
            self._frame_height, self._frame_width = shape[:2]

    def reset(self):
        with self._control_lock:
            self._tracking = False
            self._center_x = 0.0
            self._center_y = 0.0
            self._pit_track_controller.reset_I()
            self._pit_track_controller.reset_filter()
            self._yaw_track_controller.reset_I()
            self._yaw_track_controller.reset_filter()

    def mavlink_packet(self, msg):
        self._control_in_queue.put(msg)

    def _send_gimbal_manager_pitch_yaw_angle(self, pitch, yaw):
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
            float("nan"),
            float("nan"),
        )
        self._connection.mav.send(msg)

    def _send_gimbal_manager_pitch_yaw_rate(self, pitch_rate, yaw_rate):
        """
        Send a mavlink message to set the gimbal pitch and yaw (radians).
        """
        msg = self._connection.mav.gimbal_manager_set_pitchyaw_encode(
            self._connection.target_system,
            self._connection.target_component,
            0,
            0,
            float("nan"),
            float("nan"),
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
        update_rate = GIMBAL_CONTROL_LOOP_RATE
        update_period = 1.0 / update_rate

        with self._control_lock:
            enable_graphs = self._enable_graphs
            enable_profiler = self._enable_profiler

        if enable_graphs:
            self._add_livegraphs()

        # Profiler - two activities are centring and tracking
        centring_profiler = Profiler()
        tracking_profiler = Profiler()
        profile_print_last_time = time.time()
        profile_print_period = 2

        last_time_s = time.time()
        while True:
            # Record the start time of the loop
            start_time = time.time()

            # Copy shared variables
            with self._control_lock:
                act_x = self._center_x
                act_y = self._center_y
                frame_width = self._frame_width
                frame_height = self._frame_height
                tracking = self._tracking

            # Return gimbal to its neutral orientation when not tracking
            with self._mavlink_lock:
                pit_tgt_rad = self._mnt1_neutral_y
                yaw_tgt_rad = self._mnt1_neutral_z
                gimbal_orientation = self._gimbal_orientation

            # Update dt for the controller I and D terms
            time_now_s = time.time()
            dt = time_now_s - last_time_s
            last_time_s = time_now_s

            if not tracking and gimbal_orientation is not None:
                centring_profiler.start()

                # NOTE: to centre the gimbal when not tracking, we need to know
                # the neutral angles from the MNT1_NEUTRAL_x params, and also
                # the current mount orientation.
                _, pit_act_rad, yaw_act_rad = euler.quat2euler(gimbal_orientation)

                pit_rate_rads = self._pit_controller.update_all(
                    pit_tgt_rad, pit_act_rad, dt
                )
                yaw_rate_rads = self._yaw_controller.update_all(
                    yaw_tgt_rad, yaw_act_rad, dt
                )

                pit_pid_info = self._pit_controller.pid_info
                yaw_pid_info = self._yaw_controller.pid_info

                # NOTE: leave for debugging
                # print(
                #     f"Pit: Tgt: {pit_pid_info.target:.2f}, "
                #     f"Act: {pit_pid_info.actual:.2f}, "
                #     f"Out: {pit_pid_info.out:.2f}"
                # )

                # NOTE: leave for debugging
                # print(
                #     f"Yaw: Tgt: {yaw_pid_info.target:.2f}, "
                #     f"Act: {yaw_pid_info.actual:.2f}, "
                #     f"Out: {yaw_pid_info.out:.2f}"
                # )

                # NOTE: Set pitch and yaw rates to help determine PID gains for tracking
                self._send_gimbal_manager_pitch_yaw_rate(pit_rate_rads, yaw_rate_rads)

                # NOTE: Set pitch and yaw angles directly (no PID controller needed)
                # self._send_gimbal_manager_pitch_yaw_angle(pit_tgt_rad, yaw_tgt_rad)

                centring_profiler.end()

                if enable_graphs:
                    self._update_livegraphs(pit_pid_info, yaw_pid_info)

            elif frame_width is not None and frame_height is not None:
                tracking_profiler.start()

                # work with normalised screen [-1, 1]
                tgt_x = 0.0
                tgt_y = 0.0

                if math.isclose(act_x, 0.0) and math.isclose(act_y, 0.0):
                    act_x = 0.0
                    act_y = 0.0
                else:
                    act_x = (act_x / frame_width - 0.5) * -1.0
                    act_y = act_y / frame_height - 0.5

                pit_rate_rads = self._pit_track_controller.update_all(tgt_y, act_y, dt)
                yaw_rate_rads = self._yaw_track_controller.update_all(tgt_x, act_x, dt)

                # Apply rate limits (90 deg/s)
                MAX_RATE_RAD_S = math.radians(90.0)
                pit_rate_rads = constrain_float(
                    pit_rate_rads, -MAX_RATE_RAD_S, MAX_RATE_RAD_S
                )
                pit_pid_info.out = pit_rate_rads
                yaw_rate_rads = constrain_float(
                    yaw_rate_rads, -MAX_RATE_RAD_S, MAX_RATE_RAD_S
                )
                yaw_pid_info.out = yaw_rate_rads

                pit_pid_info = self._pit_track_controller.pid_info
                yaw_pid_info = self._yaw_track_controller.pid_info

                self._send_gimbal_manager_pitch_yaw_rate(pit_rate_rads, yaw_rate_rads)

                tracking_profiler.end()

                if enable_graphs:
                    self._update_livegraphs(pit_pid_info, yaw_pid_info)

            # Profile
            if enable_profiler and (
                (time.time() - profile_print_last_time) > profile_print_period
            ):
                profile_print_last_time = time.time()
                print("Gimbal controller control loop profile")
                print("-------------------------------------")
                print("time in us      avg\tlast\tcount")
                print(
                    f"centring:        "
                    f"{centring_profiler.avg_duration_us}\t"
                    f"{centring_profiler.last_duration_us}\t"
                    f"{centring_profiler.count}"
                )
                print(
                    f"tracking:        "
                    f"{tracking_profiler.avg_duration_us}\t"
                    f"{tracking_profiler.last_duration_us}\t"
                    f"{tracking_profiler.count}"
                )
                print("-------------------------------------")
                print()

            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)

    def _mavlink_task(self):
        """
        Process mavlink messages relevant to gimbal management.
        """
        update_rate = MAVLINK_RECV_RATE
        update_period = 1.0 / update_rate

        while True:

            def __process_message():
                if self._control_in_queue.empty():
                    return
                msg = self._control_in_queue.get()
                mtype = msg.get_type()
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

    def _add_livegraphs(self):
        self._graph_pid_tune = live_graph.LiveGraph(
            ["PID.PitTgt", "PID.PitAct", "PID.YawTgt", "PID.YawAct"],
            timespan=30,
            title="PID_TUNING",
        )
        self._graph_pid_pit = live_graph.LiveGraph(
            [
                "PID.Out",
                "PID.P",
                "PID.I",
                "PID.D",
            ],
            timespan=30,
            title="PID_TUNING: Pitch",
        )
        self._graph_pid_yaw = live_graph.LiveGraph(
            [
                "PID.Out",
                "PID.P",
                "PID.I",
                "PID.D",
            ],
            timespan=30,
            title="PID_TUNING: Yaw",
        )

    def _update_livegraphs(self, pit_pid_info, yaw_pid_info):
        if self._graph_pid_tune.is_alive():
            self._graph_pid_tune.add_values(
                [
                    pit_pid_info.target,
                    pit_pid_info.actual,
                    yaw_pid_info.target,
                    yaw_pid_info.actual,
                ]
            )
        if self._graph_pid_pit.is_alive():
            self._graph_pid_pit.add_values(
                [
                    pit_pid_info.out,
                    pit_pid_info.P,
                    pit_pid_info.I,
                    pit_pid_info.D,
                ]
            )
        if self._graph_pid_yaw.is_alive():
            self._graph_pid_yaw.add_values(
                [
                    yaw_pid_info.out,
                    yaw_pid_info.P,
                    yaw_pid_info.I,
                    yaw_pid_info.D,
                ]
            )


class TrackerOpenCV:
    """
    Base class for wrappers of OpenCV trackers
    """

    def __init__(self):
        self._tracker = self._create()
        self._nroi = None
        self._nroi_changed = False

    def update(self, frame):
        if self._nroi is None or frame is None:
            return False, None

        if self._nroi_changed:
            self._tracker = self._create()
            # Denormalise the roi
            frame_height, frame_width = frame.shape[:2]
            roi = [
                int(self._nroi[0] * frame_width),  # x
                int(self._nroi[1] * frame_height),  # y
                int(self._nroi[2] * frame_width),  # w
                int(self._nroi[3] * frame_height),  # h
            ]
            # print(
            #     f"Tracker: nroi: {self._nroi}, roi: {roi}, frame_width: {frame_width}, frame_height: {frame_height}"
            # )
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

    def _create(self):
        raise Exception("TrackerOpenCV must not be used directly.")


class TrackerCSTR(TrackerOpenCV):
    """
    Wrapper for cv2.TrackerCSRT
    """

    def __init__(self):
        super().__init__()

    def _create(self):
        return cv2.TrackerCSRT.create()


class TrackerKCF(TrackerOpenCV):
    """
    Wrapper for cv2.TrackerKCF
    """

    def __init__(self):
        super().__init__()

    def _create(self):
        return cv2.TrackerKCF.create()


class TrackerMOSSE(TrackerOpenCV):
    """
    Wrapper for cv2.legacy.TrackerMOSSE
    """

    def __init__(self):
        super().__init__()

    def _create(self):
        return cv2.legacy.TrackerMOSSE_create()


class TrackerFactory:

    @staticmethod
    def choices():
        return ["CSTR", "KCF", "MOSSE"]

    @staticmethod
    def create_tracker(name):
        if name == "CSTR":
            return TrackerCSTR()
        elif name == "KCF":
            return TrackerKCF()
        elif name == "MOSSE":
            return TrackerMOSSE()
        else:
            raise Exception(f"Invalid tracker name: {name}")


if __name__ == "__main__":
    import os
    import sys
    from argparse import ArgumentParser

    parser = ArgumentParser("onboard_controller.py [options]")
    parser.add_argument("--master", required=True, type=str, help="mavlink device")
    parser.add_argument(
        "--rtsp-server", required=True, type=str, help="rtsp server url"
    )
    parser.add_argument("--sysid", default=1, type=int, help="source system id")
    parser.add_argument(
        "--cmpid",
        default=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        type=int,
        help="source component id",
    )
    parser.add_argument(
        "--tracker-name",
        choices=TrackerFactory.choices(),
        default="CSTR",
        type=str,
        help="tracker name",
    )
    parser.add_argument(
        "--enable-graphs",
        action="store_const",
        default=False,
        const=True,
        help="enable live graphs",
    )
    parser.add_argument(
        "--enable-profiler",
        action="store_const",
        default=False,
        const=True,
        help="enable main loop profiler",
    )
    parser.add_argument(
        "--enable-rpi-pipeline",
        action="store_const",
        default=False,
        const=True,
        help="enable hardware pipelane for rpi4",
    )
    args = parser.parse_args()

    controller = OnboardController(
        args.master,
        args.sysid,
        args.cmpid,
        args.rtsp_server,
        args.tracker_name,
        args.enable_graphs,
        args.enable_profiler,
        args.enable_rpi_pipeline,
    )

    while True:
        try:
            controller.run()
        except KeyboardInterrupt:
            controller = None
            EXIT_CODE_CTRL_C = 130
            sys.exit(EXIT_CODE_CTRL_C)
