"""
use vicon data to provide VISION_POSITION_ESTIMATE and GPS_INPUT data
"""

import math
import threading
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import LowPassFilter2p
from pymavlink.rotmat import Vector3
from pymavlink.quaternion import Quaternion
from pymavlink import mavutil
from pymavlink import mavextra

from pyvicon import pyvicon


def get_gps_time(tnow):
    """return gps_week and gps_week_ms for current time"""
    leapseconds = 18
    SEC_PER_WEEK = 7 * 86400

    epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
    epoch_seconds = int(tnow - epoch)
    week = int(epoch_seconds) // SEC_PER_WEEK
    t_ms = int(tnow * 1000) % 1000
    week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
    return week, week_ms


class ViconModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(ViconModule, self).__init__(mpstate, "vicon", "vicon", public=False)
        self.console.set_status('VPos', 'VPos -- -- --', row=5)
        self.console.set_status('VAtt', 'VAtt -- -- --', row=5)
        self.vicon_settings = mp_settings.MPSettings(
            [('host', str, "vicon"),
             ('origin_lat', float, -35.363261),
             ('origin_lon', float, 149.165230),
             ('origin_alt', float, 584.0),
             ('vision_rate', int, 14),
             ('vel_filter_hz', float, 30.0),
             ('gps_rate', int, 5),
             ('gps_nsats', float, 16),
             ('object_name', str, None)
             ])
        self.add_command('vicon', self.cmd_vicon, 'VICON control',
                         ["<start>",
                          "<stop>",
                          "set (VICONSETTING)"])
        self.add_completion_function('(VICONSETTING)',
                                     self.vicon_settings.completion)
        self.vicon = None
        self.thread = threading.Thread(target=self.thread_loop)
        self.thread.start()
        self.pos = None
        self.att = None
        self.frame_count = 0
        self.gps_count = 0
        self.vision_count = 0
        self.last_frame_count = 0
        self.vel_filter = LowPassFilter2p.LowPassFilter2p(200.0, 30.0)
        self.actual_frame_rate = 0.0

    def detect_vicon_object(self):
        self.vicon.get_frame()
        object_name = self.vicon_settings.object_name
        if object_name is None:
            # We haven't specified which object we are looking for, so just find the first one
            object_name = self.vicon.get_subject_name(0)
        if object_name is None:
            # No objects found
            return None, None
        segment_name = self.vicon.get_subject_root_segment_name(object_name)
        if segment_name is None:
            # Object we're looking for can't be found
            return None, None
        print("Connected to subject '%s' segment '%s'" % (object_name, segment_name))
        return object_name, segment_name

    def get_vicon_pose(self, object_name, segment_name):

        # get position in mm. Coordinates are in NED
        vicon_pos = self.vicon.get_segment_global_translation(object_name, segment_name)

        if vicon_pos is None:
            # Object is not in view
            return None, None, None, None

        vicon_quat = Quaternion(self.vicon.get_segment_global_quaternion(object_name, segment_name))

        pos_ned = Vector3(vicon_pos * 0.001)
        euler = vicon_quat.euler
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        yaw = math.radians(mavextra.wrap_360(math.degrees(yaw)))

        return pos_ned, roll, pitch, yaw

    def thread_loop(self):
        """background processing"""
        object_name = None
        segment_name = None
        last_pos = None
        last_frame_num = None
        frame_count = 0

        while True:
            if self.vicon is None:
                time.sleep(0.1)
                object_name = None
                continue

            if not object_name:
                object_name, segment_name = self.detect_vicon_object()
                if object_name is None:
                    continue
                last_msg_time = time.time()
                now = time.time()
                last_origin_send = now
                now_ms = int(now * 1000)
                last_gps_send_ms = now_ms
                frame_rate = self.vicon.get_frame_rate()
                frame_dt = 1.0/frame_rate
                last_rate = time.time()
                frame_count = 0
                print("Vicon frame rate %.1f" % frame_rate)

            if self.vicon_settings.gps_rate > 0:
                gps_period_ms = 1000 // self.vicon_settings.gps_rate
            time.sleep(0.01)
            self.vicon.get_frame()
            mav = self.master
            now = time.time()
            now_ms = int(now * 1000)
            frame_num = self.vicon.get_frame_number()

            frame_count += 1
            if now - last_rate > 0.1:
                rate = frame_count / (now - last_rate)
                self.actual_frame_rate = 0.9 * self.actual_frame_rate + 0.1 * rate
                last_rate = now
                frame_count = 0
                self.vel_filter.set_cutoff_frequency(self.actual_frame_rate, self.vicon_settings.vel_filter_hz)

            pos_ned, roll, pitch, yaw = self.get_vicon_pose(object_name, segment_name)
            if pos_ned is None:
                continue
            
            # print(f"XYZ: {pos_ned.x}, {pos_ned.y}, {pos_ned.z}, ")

            if last_frame_num is None or frame_num - last_frame_num > 100 or frame_num <= last_frame_num:
                last_frame_num = frame_num
                last_pos = pos_ned
                continue

            dt = (frame_num - last_frame_num) * frame_dt
            vel = (pos_ned - last_pos) * (1.0/dt)
            last_pos = pos_ned
            last_frame_num = frame_num

            filtered_vel = self.vel_filter.apply(vel)

            if self.vicon_settings.vision_rate > 0:
                dt = now - last_msg_time
                if dt < 1.0 / self.vicon_settings.vision_rate:
                    continue

            last_msg_time = now

            self.pos = pos_ned
            self.att = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
            self.frame_count += 1

            time_us = int(now * 1.0e6)

            if now - last_origin_send > 1 and self.vicon_settings.vision_rate > 0:
                # send a heartbeat msg
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)

                # send origin at 1Hz
                mav.mav.set_gps_global_origin_send(self.target_system,
                                                   int(self.vicon_settings.origin_lat*1.0e7),
                                                   int(self.vicon_settings.origin_lon*1.0e7),
                                                   int(self.vicon_settings.origin_alt*1.0e3),
                                                   time_us)
                last_origin_send = now

            if self.vicon_settings.gps_rate > 0 and now_ms - last_gps_send_ms > gps_period_ms:
                '''send GPS data at the specified rate, trying to align on the given period'''
                self.gps_input_send(now, pos_ned, yaw, filtered_vel)
                last_gps_send_ms = (now_ms//gps_period_ms) * gps_period_ms
                self.gps_count += 1

            if self.vicon_settings.vision_rate > 0:
                # send VISION_POSITION_ESTIMATE
                # we force mavlink1 to avoid the covariances which seem to make the packets too large
                # for the mavesp8266 wifi bridge
                mav.mav.global_vision_position_estimate_send(time_us,
                                                             pos_ned.x, pos_ned.y, pos_ned.z,
                                                             roll, pitch, yaw, force_mavlink1=True)
                self.vision_count += 1

    def gps_input_send(self, time, pos_ned, yaw, gps_vel):
        time_us = int(time * 1.0e6)

        gps_lat, gps_lon = mavextra.gps_offset(self.vicon_settings.origin_lat,
                                               self.vicon_settings.origin_lon,
                                               pos_ned.y, pos_ned.x)
        gps_alt = self.vicon_settings.origin_alt - pos_ned.z
        gps_week, gps_week_ms = get_gps_time(time)
        if self.vicon_settings.gps_nsats >= 6:
            fix_type = 3
        else:
            fix_type = 1
        yaw_cd = int(mavextra.wrap_360(math.degrees(yaw)) * 100)
        if yaw_cd == 0:
            # the yaw extension to GPS_INPUT uses 0 as no yaw support
            yaw_cd = 36000
        self.master.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
                               int(gps_lat * 1.0e7), int(gps_lon * 1.0e7), gps_alt,
                               1.0, 1.0,
                               gps_vel.x, gps_vel.y, gps_vel.z,
                               0.2, 1.0, 1.0,
                               self.vicon_settings.gps_nsats,
                               yaw_cd)

    def cmd_start(self):
        """start vicon"""
        vicon = pyvicon.PyVicon()
        print("Opening Vicon connection to %s" % self.vicon_settings.host)
        vicon.connect(self.vicon_settings.host)
        print("Configuring vicon")
        vicon.set_stream_mode(pyvicon.StreamMode.ClientPull)
        vicon.enable_marker_data()
        vicon.enable_segment_data()
        vicon.enable_unlabeled_marker_data()
        vicon.enable_device_data()

        # Set the axis mapping to the ardupilot convention (North, East, Down)
        vicon.set_axis_mapping(pyvicon.Direction.Forward, pyvicon.Direction.Right, pyvicon.Direction.Down)
        print(vicon.get_axis_mapping())
        print("vicon ready")
        self.vicon = vicon

    def cmd_vicon(self, args):
        """command processing"""
        if len(args) == 0:
            print("Usage: vicon <set|start|stop>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.vicon = None
        elif args[0] == "set":
            self.vicon_settings.command(args[1:])

    def idle_task(self):
        """run on idle"""
        if not self.pos or not self.att or self.frame_count == self.last_frame_count:
            return
        self.last_frame_count = self.frame_count
        self.console.set_status('VPos', 'Vicon: Pos: %.2fN %.2fE %.2fD' % (self.pos.x, self.pos.y, self.pos.z), row=5)
        self.console.set_status('VAtt', ' Att R:%.2f P:%.2f Y:%.2f GPS %u VIS %u RATE %.1f' % (self.att[0], self.att[1], self.att[2],
                                                                                         self.gps_count, self.vision_count,
                                                                                         self.actual_frame_rate), row=5)


def init(mpstate):
    """initialise module"""
    return ViconModule(mpstate)
