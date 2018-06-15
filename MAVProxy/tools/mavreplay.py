#!/usr/bin/env python

import argparse
import datetime
import time

from pymavlink import mavutil


class MAVReplay(object):
    MAV_MESSAGE_TYPES = ["HEARTBEAT", "SYS_STATUS", "SYSTEM_TIME", "PING", "CHANGE_OPERATOR_CONTROL", "CHANGE_OPERATOR_CONTROL_ACK", "AUTH_KEY", "SET_MODE", "PARAM_REQUEST_READ", "PARAM_REQUEST_LIST", "PARAM_VALUE", "PARAM_SET", "GPS_RAW_INT", "GPS_STATUS", "SCALED_IMU", "RAW_IMU", "RAW_PRESSURE", "SCALED_PRESSURE", "ATTITUDE", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "RC_CHANNELS_SCALED", "RC_CHANNELS_RAW", "SERVO_OUTPUT_RAW", "MISSION_REQUEST_PARTIAL_LIST", "MISSION_WRITE_PARTIAL_LIST", "MISSION_ITEM", "MISSION_REQUEST", "MISSION_SET_CURRENT", "MISSION_CURRENT", "MISSION_REQUEST_LIST", "MISSION_COUNT", "MISSION_CLEAR_ALL", "MISSION_ITEM_REACHED", "MISSION_ACK", "SET_GPS_GLOBAL_ORIGIN", "GPS_GLOBAL_ORIGIN", "PARAM_MAP_RC", "MISSION_REQUEST_INT", "SAFETY_SET_ALLOWED_AREA", "SAFETY_ALLOWED_AREA", "ATTITUDE_QUATERNION_COV", "NAV_CONTROLLER_OUTPUT", "GLOBAL_POSITION_INT_COV", "LOCAL_POSITION_NED_COV", "RC_CHANNELS", "REQUEST_DATA_STREAM", "DATA_STREAM", "MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE", "MISSION_ITEM_INT", "VFR_HUD", "COMMAND_INT", "COMMAND_LONG", "COMMAND_ACK", "MANUAL_SETPOINT", "SET_ATTITUDE_TARGET", "ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED", "POSITION_TARGET_LOCAL_NED", "SET_POSITION_TARGET_GLOBAL_INT", "POSITION_TARGET_GLOBAL_INT", "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", "HIL_STATE", "HIL_CONTROLS", "HIL_RC_INPUTS_RAW", "HIL_ACTUATOR_CONTROLS", "OPTICAL_FLOW", "GLOBAL_VISION_POSITION_ESTIMATE", "VISION_POSITION_ESTIMATE", "VISION_SPEED_ESTIMATE", "VICON_POSITION_ESTIMATE", "HIGHRES_IMU", "OPTICAL_FLOW_RAD", "HIL_SENSOR", "SIM_STATE", "RADIO_STATUS", "FILE_TRANSFER_PROTOCOL", "TIMESYNC", "CAMERA_TRIGGER", "HIL_GPS", "HIL_OPTICAL_FLOW", "HIL_STATE_QUATERNION", "LOG_REQUEST_LIST", "LOG_ENTRY", "LOG_REQUEST_DATA", "LOG_DATA", "LOG_ERASE", "LOG_REQUEST_END", "GPS_INJECT_DATA", "POWER_STATUS", "SERIAL_CONTROL", "GPS_RTK", "DATA_TRANSMISSION_HANDSHAKE", "ENCAPSULATED_DATA", "DISTANCE_SENSOR", "TERRAIN_REQUEST", "TERRAIN_DATA", "TERRAIN_CHECK", "TERRAIN_REPORT", "ATT_POS_MOCAP", "SET_ACTUATOR_CONTROL_TARGET", "ACTUATOR_CONTROL_TARGET", "ALTITUDE", "RESOURCE_REQUEST", "FOLLOW_TARGET", "CONTROL_SYSTEM_STATE", "BATTERY_STATUS", "AUTOPILOT_VERSION", "LANDING_TARGET", "ESTIMATOR_STATUS", "WIND_COV", "GPS_INPUT", "GPS_RTCM_DATA", "HIGH_LATENCY", "VIBRATION", "HOME_POSITION", "SET_HOME_POSITION", "MESSAGE_INTERVAL", "EXTENDED_SYS_STATE", "ADSB_VEHICLE", "COLLISION", "MEMORY_VECT", "DEBUG_VECT", "NAMED_VALUE_FLOAT", "NAMED_VALUE_INT", "STATUSTEXT", "DEBUG", "SETUP_SIGNING", "BUTTON_CHANGE", "PLAY_TUNE", "CAMERA_INFORMATION", "CAMERA_SETTINGS", "STORAGE_INFORMATION", "CAMERA_CAPTURE_STATUS", "CAMERA_IMAGE_CAPTURED", "FLIGHT_INFORMATION", "MOUNT_ORIENTATION", "LOGGING_DATA", "LOGGING_DATA_ACKED", "LOGGING_ACK", "VIDEO_STREAM_INFORMATION", "SET_VIDEO_STREAM_SETTINGS", "WIFI_CONFIG_AP", "PROTOCOL_VERSION", "UAVCAN_NODE_STATUS", "UAVCAN_NODE_INFO", "PARAM_EXT_REQUEST_READ", "PARAM_EXT_REQUEST_LIST", "PARAM_EXT_VALUE", "PARAM_EXT_SET", "PARAM_EXT_ACK", "OBSTACLE_DISTANCE", "ODOMETRY", "TRAJECTORY"]

    def __init__(self, logfile, output, real_time):
        """
        Wrapper for ingesting and relaying telemetry from a
        MAVLink tlog file.

        Arguments:
            logfile (str): Path to the MAVLink tlog file to replay
            output (str): A socket to output MAVLink telemetry on
            real_time (bool): Should the packets be relayed in real time?
        """
        self.real_time = real_time

        # Set up temporary mavlink connection to gather information about the log
        mlog = mavutil.mavlink_connection(logfile)
        # Get a time packet to synchronize based on
        m_start = None

        m = mlog.recv_match(type="SYSTEM_TIME")
        while m is not None:
            if m.get_type() == "SYSTEM_TIME":
                if m_start is None:
                    m_start = m
                m_end = m
            m = mlog.recv_match(type="SYSTEM_TIME")
        # Real boot time of the system
        self.real_boot_time = datetime.datetime.fromtimestamp( ((m_start.time_unix_usec/1000) - m_start.time_boot_ms) / 1000 )
        print("Log system boot time: %s" % str(self.real_boot_time))
        # Fake boot time of the system for the purposes of the replay
        self.start_time = datetime.datetime.now() - datetime.timedelta(milliseconds=m_start.time_boot_ms)
        print("Replay system boot time: %s" % self.start_time)
        # Estimated time the log will end
        #
        # This is actually the estimated time the last SYSTEM_TIME packet will be sent,
        # but it will be correct within a second or so.
        print("Estimated log end time: %s" % (self.start_time + datetime.timedelta(milliseconds=m_end.time_boot_ms)))

        # Set up mavlink connections for replay
        self.mlog = mavutil.mavlink_connection(logfile)
        self.mout = mavutil.mavlink_connection(output, input=False,
                                          source_system=self.mlog.source_system,
                                          source_component=self.mlog.source_component)

    def run(self):
        """
        Read MAVLink packets and relay them
        """
        m = self.mlog.recv_match(type=self.MAV_MESSAGE_TYPES)
        while m is not None:
            if self.real_time:
                self.wait_for_packet(m)
            self.mout.mav.send(m)
            m = self.mlog.recv_match(type=self.MAV_MESSAGE_TYPES)

    def wait_for_packet(self, m):
        """
        Wait until this packet should be sent

        Arguments:
            m: The packet to wait for
        """
        if hasattr(m, "time_usec"):
            time_next = m.time_usec / 1000
        elif hasattr(m, "time_boot_ms"):
            time_next = m.time_boot_ms
        else:
            return
        sleep_seconds = ((self.start_time + datetime.timedelta(milliseconds=time_next)) - datetime.datetime.now()).total_seconds()
        if sleep_seconds > 0:
            time.sleep(sleep_seconds)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--log', required=True, help="Log file to replay")
    parser.add_argument('-o', '--out', default="127.0.0.1:14550", help="Socket to output telemetry to")
    parser.add_argument('-r', '--real-time', action='store_true', help="Replay telemetry in real time")

    args = parser.parse_args()
    replay = MAVReplay(args.log, args.out, args.real_time)

    replay.run()

if __name__ == '__main__':
    main()