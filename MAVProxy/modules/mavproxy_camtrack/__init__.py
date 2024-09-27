"""
MAVProxy camera tracking module

Reference
---------

MAVLink documentation

- https://mavlink.io/en/services/gimbal_v2.html
- https://mavlink.io/en/services/camera.html#camera-protocol-v2


ArduPilot MAVLink handlers

- https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp


pymavlink 

- pymavlink.dialects.v20.ardupilotmega.MAVLink
- MAVLink.gimbal_*
- MAVLink.camera_*
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.mavproxy_camtrack.camera_view import CameraView

from pymavlink import mavutil


class CamTrackModule(mp_module.MPModule):
    """A tool to control camera tracking"""

    def __init__(self, mpstate):
        super(CamTrackModule, self).__init__(
            mpstate, "camtrack", "camera tracking module"
        )

        self.mpstate = mpstate

        # GUI
        # TODO: provide args to set RTSP server location
        # localhost simulation
        rtsp_url = "rtsp://127.0.0.1:8554/camera"

        # home wifi
        # rtsp_url = "rtsp://192.168.1.204:8554/fpv_stream"

        # herelink wifi access point
        # rtsp_url = "rtsp://192.168.43.1:8554/fpv_stream"

        # SIYI A8 camera
        # rtsp_url = "rtsp://192.168.144.25:8554/main.264"

        self.camera_view = CameraView(self.mpstate, "Camera Tracking", rtsp_url)

        # TODO: NOTE: unused
        # mavlink messages
        self._last_gimbal_device_information = None
        self._last_gimbal_manager_status = None
        self._last_gimbal_device_information = None
        self._last_gimbal_device_attitude_status = None
        self._last_autopilot_state_for_gimbal_device = None
        self._last_camera_tracking_image_status = None

        # Discovery
        self._do_request_gimbal_manager_information = True
        self._do_request_gimbal_manager_status = True
        self._do_request_gimbal_device_information = True
        self._do_request_autopilot_state_for_gimbal_device = True
        self._do_request_camera_information = True
        self._do_request_camera_tracking_image_status = True

        # data

        # control update rate to GUI
        self._msg_list = []
        self._fps = 30.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9

        # commands
        self.add_command("camtrack", self.cmd_camtrack, "camera tracking")

    def cmd_camtrack(self, args):
        """Control behaviour of commands"""
        if len(args) <= 0:
            print(self.usage())
            return

        if args[0] == "status":
            print(self.status())
            return

        if args[0] == "start":
            print("start tracking")
            return

        if args[0] == "stop":
            print("stop tracking")
            return

        print(self.usage())

    def usage(self):
        """Show help on command line options."""
        return "Usage: camtrack <status|start|stop>"

    def status(self):
        """Return information about the camera tracking state"""
        return [
            str(self._last_gimbal_manager_information),
            str(self._last_gimbal_manager_status),
            str(self._last_gimbal_device_information),
            str(self._last_gimbal_device_attitude_status),
            str(self._last_autopilot_state_for_gimbal_device),
        ]

    def mavlink_packet(self, msg):
        """Handle mavlink packets."""
        mtype = msg.get_type()

        # heartbeat
        if mtype == "HEARTBEAT":
            self.handle_heartbeat(msg)

        # working - must be requested
        elif mtype == "GIMBAL_MANAGER_INFORMATION":
            self.handle_gimbal_manager_information(msg)

        # working - must be requested (should be broadcast)
        elif mtype == "GIMBAL_MANAGER_STATUS":
            self.handle_gimbal_manager_status(msg)

        # not working - limited implementation in AP_Mount
        elif mtype == "GIMBAL_DEVICE_INFORMATION":
            self.handle_gimbal_device_information(msg)

        # working - boradcast
        elif mtype == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            self.handle_gimbal_device_attitude_status(msg)

        # working - must be requested
        elif mtype == "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE":
            self.handle_autopilot_state_for_gimbal_device(msg)

        # working - must be requested
        elif mtype == "CAMERA_INFORMATION":
            self.handle_camera_information(msg)

        elif mtype == "CAMERA_TRACKING_IMAGE_STATUS":
            # TODO: add handler
            print(msg)

        # TODO: NOTE: disabled
        # check command_ack
        elif False:  # mtype == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_POINT: ACCEPTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_POINT: REJECTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_POINT: DENIED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_POINT: UNSUPPORTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_POINT: FAILED")
                else:
                    print(
                        "Got COMMAND_ACK: CAMERA_TRACK_POINT: result: {}".format(
                            msg.result
                        )
                    )

            elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: ACCEPTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: REJECTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: DENIED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: UNSUPPORTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                    print("Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: FAILED")
                else:
                    print(
                        "Got COMMAND_ACK: CAMERA_TRACK_RECTANGLE: result: {}".format(
                            msg.result
                        )
                    )

            elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Got COMMAND_ACK: CAMERA_STOP_TRACKING: ACCEPTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    print("Got COMMAND_ACK: CAMERA_STOP_TRACKING:tracking REJECTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    print("Got COMMAND_ACK: CAMERA_STOP_TRACKING: DENIED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    print("Got COMMAND_ACK: CAMERA_STOP_TRACKING: UNSUPPORTED")
                elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                    print("Got COMMAND_ACK: CAMERA_STOP_TRACKING: FAILED")
                else:
                    print(
                        "Got COMMAND_ACK: CAMERA_STOP_TRACKING: RESULT: {}".format(
                            msg.result
                        )
                    )

            elif msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                print("Got COMMAND_ACK: MAV_CMD_SET_MESSAGE_INTERVAL")

        # TODO: NOTE: disabled
        # check command_long
        elif False:  # mtype == "COMMAND_LONG":
            # TODO: check target_system is for offboard control
            if msg.target_system != self.master.source_system:
                pass
            elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                print(msg)
            elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                print(msg)
            elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING:
                print(msg)

    def handle_heartbeat(self, msg):
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        # is this from an autopilot
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            # print(f"HB: AUTOPILOT: sysid: {sysid}, compid: {compid}")
            # print(mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            pass

        # What type of component?
        if msg.type == mavutil.mavlink.MAV_TYPE_GENERIC:
            print("MAV_TYPE_GENERIC")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_GCS:
        #     print("MAV_TYPE_GCS")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        #     print("MAV_TYPE_FIXED_WING")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        #     print("MAV_TYPE_QUADROTOR")
        elif msg.type == mavutil.mavlink.MAV_TYPE_GIMBAL:
            print("MAV_TYPE_GIMBAL")
            # handle mavlink gimbal component
        elif msg.type == mavutil.mavlink.MAV_TYPE_CAMERA:
            print("MAV_TYPE_CAMERA")
            # handle mavlink camera component

    def handle_gimbal_manager_information(self, msg):
        self._last_gimbal_manager_information = msg

    def handle_gimbal_manager_status(self, msg):
        self._last_gimbal_manager_status = msg

    def handle_gimbal_device_information(self, msg):
        self._last_gimbal_device_information = msg

    def handle_gimbal_device_attitude_status(self, msg):
        self._last_gimbal_device_attitude_status = msg

    def handle_autopilot_state_for_gimbal_device(self, msg):
        self._last_autopilot_state_for_gimbal_device = msg

    def handle_camera_information(self, msg):
        # print(msg)
        pass

    def check_events(self):
        """Check for events on the camera view"""
        self.camera_view.check_events()

        # TODO: check which shutdown events are available in MPImage
        # tell mavproxy to unload the module if the GUI is closed
        # if self.camera_view.close_event.wait(timeout=0.001):
        #     self.needs_unloading = True

    def send_messages(self):
        """Send message list via pipe to GUI at desired update rate"""
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to GUI
            # TODO: check interface in view for pipe updates
            # self.camera_view.parent_pipe_send.send(self._msg_list)
            # reset counters etc
            self._msg_list = []
            self._last_send = time.time()

        # TODO: implement camera and gimbal discovery correctly
        # Discovery - most of these requests are handled in the FC
        #             by GCS_MAVLINK::try_send_message
        if self._do_request_gimbal_manager_information:
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
            )
            self._do_request_gimbal_manager_information = False

        if self._do_request_gimbal_manager_status:
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS
            )
            self._do_request_gimbal_manager_status = False

        # NOTE: only AP_Mount_Gremsy implements handle_gimbal_device_information
        if self._do_request_gimbal_device_information:
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION
            )
            self._do_request_gimbal_device_information = False

        if self._do_request_autopilot_state_for_gimbal_device:
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
            )
            self._do_request_autopilot_state_for_gimbal_device = False

        if self._do_request_camera_information:
            self.send_request_message(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION)
            self._do_request_camera_information = False

        if self._do_request_camera_tracking_image_status:
            self.send_set_message_interval_message(
                mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS,
                1000 * 1000,  # 1Hz
                response_target=1,  # flight-stack default
            )
            self._do_request_camera_tracking_image_status = False

    def send_gimbal_manager_configure(self):
        # Acquire and release control
        primary_sysid = -1
        primary_compid = -1
        secondary_sysid = -1
        secondary_compid = -1
        gimbal_devid = 0
        self.master.mav.command_long_send(
            self.target_system,  # target_system
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,  # command
            0,  # confirmation
            primary_sysid,  # param1
            primary_compid,  # param2
            secondary_sysid,  # param3
            secondary_compid,  # param4
            0,  # param5
            0,  # param6
            gimbal_devid,  # param7
        )

    # MAVProxy.modules.mavproxy_misc.py
    def send_request_message(self, message_id, p1=0):
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
            0,  # confirmation
            message_id,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

    def send_set_message_interval_message(
        self, message_id, interval_us, response_target=1
    ):
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component,  # target_component
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

    def request_camera_information(self):
        # send CAMERA_INFORMATION request
        # mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
        pass

    def request_gimbal_manager_information(self):
        pass

    def request_gimbal_manager_status(self):
        pass

    def request_gimbal_device_information(self):
        pass

    def request_autopilot_state_for_gimbal_device(self):
        pass

    def idle_task(self):
        """Idle tasks"""
        self.check_events()
        self.send_messages()

    def unload(self):
        """Close the GUI and unload module"""

        # close the GUI
        self.camera_view.close()


def init(mpstate):
    """Initialise module"""

    return CamTrackModule(mpstate)
