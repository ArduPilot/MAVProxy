"""
use SteamVR tracker data to provide VISION_POSITION_ESTIMATE and GPS_INPUT data
openvr module required, https://github.com/cmbruns/pyopenvr
"""

import math
import threading
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink.rotmat import Vector3
from pymavlink import mavutil
from pymavlink import mavextra

import openvr
import numpy as np

from pymavlink.quaternion import Quaternion

"""
This assumes the single arm with the LED is pointed forward
"""

def get_gps_time(tnow):
    '''return gps_week and gps_week_ms for current time'''
    leapseconds = 18
    SEC_PER_WEEK = 7 * 86400

    epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
    epoch_seconds = int(tnow - epoch)
    week = int(epoch_seconds) // SEC_PER_WEEK
    t_ms = int(tnow * 1000) % 1000
    week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
    return week, week_ms


class SteamVR_Module(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SteamVR_Module, self).__init__(mpstate, "steamvr", "steamvr", public=False)
        self.vr_settings = mp_settings.MPSettings(
            [('origin_lat', float, -35.363261), # where the origin will be set to
             ('origin_lon', float, 149.165230),
             ('origin_alt', float, 0),
             ('vision_rate', int, 50), # rate to send vision message
             ('vision_speed', bool, 1), # also send vision speed, this will be sent at the same rate as vision position
             ('gps_rate', int, 0), # rate to send GPS message, should not be used at the same time as vision
             ('gps_nsats', float, 16), # number of satellite to send in GPS message
             ('yaw_offset', float, 0.0)]) # yaw offset of tracker mounting in deg
        self.add_command('steamvr', self.cmd_steamvr, 'SteamVR control',
                         ["<start>",
                          "<stop>",
                          "set (STEAMVRSETTING)"])
        self.add_completion_function('(STEAMVRSETTING)',
                                     self.vr_settings.completion)
        self.vr = None
        self.thread = threading.Thread(target=self.thread_loop)
        self.thread.start()
        self.pos = None
        self.att = None
        self.vel = None
        self.frame_count = 1
        self.last_frame_count = 0
        self.frame_rate = 0
        self.gps_rate = 0
        self.vision_rate = 0
        self.tracker_index = 0
        self.health = False
        self.battery = 0.0
        self.stop = False

    def tracker_q_to_euler(self,r_w,r_x,r_y,r_z):
        # taken from https://steamcommunity.com/app/250820/discussions/0/1728711392744037419/
        test = r_x * r_y + r_z * r_w
        if test > 0.499:
            # singularity at north pole
            yaw = 2 * math.atan2(r_x, r_w)
            pitch = math.pi / 2
            roll = 0
            return [roll,pitch,yaw]
        if test < -0.499:
            # singularity at south pole
            yaw = -2 * math.atan2(r_x, r_w)
            pitch = -math.pi / 2
            roll = 0
            return [roll,pitch,yaw]
        sqx = r_x*r_x
        sqy = r_y*r_y
        sqz = r_z*r_z
        yaw =  math.atan2(2 * r_y * r_w - 2 * r_x * r_z, 1 - 2 * sqy - 2 * sqz)
        pitch = math.asin(2 * test)
        roll = math.atan2(2 * r_x * r_w - 2 * r_y * r_z, 1 - 2 * sqx - 2 * sqz)
        return [roll,pitch,yaw]

    def thread_loop(self):
        '''background processing'''

        poses = []  # will be populated with proper type after first call
        frame_count = 0
        gps_frame_count = 0
        vision_frame_count = 0
        last_msg_time = 0
        last_origin_send = 0
        last_gps_send = 0
        last_rate = 0

        while True:
            # idle until started
            vr = self.vr
            if vr is None:
                time.sleep(0.1)
                continue

            if self.stop:
                self.stop = False
                self.vr.shutdown()
                self.vr = None
                self.frame_count += 1
                continue

            # Read all poses
            # built in rate, ~144hz, may be diffrent for a head set with a different update rate
            poses, _ = vr.VRCompositor().waitGetPoses(poses, None)
            self.frame_count += 1

            # Get the pose for the tracker
            pose = poses[self.tracker_index]

            # track frame rate at 1 hz
            now = time.time()
            frame_count += 1
            if now - last_rate > 1:
                self.battery = vr.VRSystem().getFloatTrackedDeviceProperty(self.tracker_index, vr.Prop_DeviceBatteryPercentage_Float)
                dt = now - last_rate
                last_rate = now
                self.frame_rate = 0.70 * self.frame_rate + 0.3 * (frame_count / dt)
                self.gps_rate = 0.70 * self.gps_rate + 0.3 * (gps_frame_count / dt)
                self.vision_rate = 0.70 * self.vision_rate + 0.3 * (vision_frame_count / dt)
                frame_count = 0
                gps_frame_count = 0
                vision_frame_count = 0


            # check if the pose is valid
            if pose.eTrackingResult != vr.TrackingResult_Running_OK or not pose.bPoseIsValid or not pose.bDeviceIsConnected:
                self.health = False
                continue
            self.health = True

            # read in position and attitude vector
            pose_mat = pose.mDeviceToAbsoluteTracking

            # convert to NED
            pos_ned = Vector3(-pose_mat[2][3], pose_mat[0][3], -pose_mat[1][3])

            # extract the 3x3 rotation matrix
            rot_mat = np.array([ [pose_mat[0][0], pose_mat[0][1], pose_mat[0][2]],[pose_mat[1][0], pose_mat[1][1], pose_mat[1][2]],[pose_mat[2][0], pose_mat[2][1], pose_mat[2][2]] ])

            # rotate about x
            x_mat = np.array([ [1,0,0], [0,0,1], [0,-1,0] ])

            mat = rot_mat.dot(x_mat)

            # get the quaternion
            q = Quaternion(mat)

            # Convert quaternion to roll, pitch and yaw
            # For some reason the built in method does not return the expected result
            # it is highly likely that the DCM is rotated in some intresting way
            # hard to tell because the built in function goes via DCM and the working example is direct
            # the built in DCM to quaternion function is identical
            [pitch,roll,yaw] = self.tracker_q_to_euler(q[0],q[1],q[2],q[3])
            pitch *= -1
            yaw *= -1
            yaw += math.radians(self.vr_settings.yaw_offset + 180)
            yaw = math.radians(mavextra.wrap_360(math.degrees(yaw)))

            # we also have acesees to the rotation rates in rads/s
            # I have not worked out the conversion as it is not currently used
            #rot_rate = pose.vAngularVelocity

            self.pos = pos_ned
            self.att = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

            # read in velocity's
            vel_in = pose.vVelocity
            vel_ned = Vector3(-vel_in[2], vel_in[0], -vel_in[1])
            self.vel = vel_ned

            # match requested send rate
            if self.vr_settings.vision_rate > 0:
                dt = now - last_msg_time
                if dt < 1.0 / self.vr_settings.vision_rate:
                    continue
            last_msg_time = now

            mav = self.master
            time_us = int(now * 1.0e6)
            if now - last_origin_send > 1 and self.vr_settings.vision_rate > 0:
                # send origin at 1Hz
                mav.mav.set_gps_global_origin_send(self.target_system,
                                                   int(self.vr_settings.origin_lat*1.0e7),
                                                   int(self.vr_settings.origin_lon*1.0e7),
                                                   int(self.vr_settings.origin_alt*1.0e3),
                                                   time_us)
                last_origin_send = now

            if self.vr_settings.gps_rate > 0:
                gps_period = 1 / self.vr_settings.gps_rate
                if now - last_gps_send > gps_period:
                    '''send GPS data at the specified rate, trying to align on the given period'''
                    yaw_cd = int(math.degrees(yaw) * 100)
                    if yaw_cd == 0:
                        # the yaw extension to GPS_INPUT uses 0 as no yaw support
                        yaw_cd = 36000

                    gps_lat, gps_lon = mavextra.gps_offset(self.vr_settings.origin_lat,
                                                        self.vr_settings.origin_lon,
                                                        pos_ned.y, pos_ned.x)
                    gps_alt = self.vr_settings.origin_alt - pos_ned.z

                    gps_week, gps_week_ms = get_gps_time(now)

                    if self.vr_settings.gps_nsats >= 6:
                        fix_type = 3
                    else:
                        fix_type = 1
                    mav.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
                                        int(gps_lat*1.0e7), int(gps_lon*1.0e7), gps_alt,
                                        1.0, 1.0,
                                        vel_ned.x, vel_ned.y, vel_ned.z,
                                        0.2, 1.0, 1.0,
                                        self.vr_settings.gps_nsats,
                                        yaw_cd)
                    last_gps_send = now
                    gps_frame_count += 1

            if self.vr_settings.vision_rate > 0:
                # send VISION_POSITION_ESTIMATE
                # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
                # we force mavlink1 to avoid the covariances which seem to make the packets too large
                # for the mavesp8266 wifi bridge
                mav.mav.global_vision_position_estimate_send(time_us,
                                                            pos_ned.x, pos_ned.y, pos_ned.z,
                                                            roll, pitch, yaw, force_mavlink1=True)
                vision_frame_count += 1

                if self.vr_settings.vision_speed:
                    # send VISION_SPEED_ESTIMATE
                    # https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
                    mav.mav.vision_speed_estimate_send(time_us, vel_ned.x, vel_ned.y, vel_ned.z, force_mavlink1=True)

    def cmd_start(self):
        print("Opening SteamVR")
        try:
            vr = openvr
            vr.init(vr.VRApplication_Scene)
        except:
            print("Could not start PyOpenVR, check that it is installed correctly, see:")
            print("https://github.com/cmbruns/pyopenvr")
            return
        poses = []
        poses, _ = vr.VRCompositor().waitGetPoses(poses, None)
        # https://github.com/cmbruns/pyopenvr/blob/927611d5cad0088d66a168483dff207e81e07707/src/translate/openvr.h#L189
        for i in range(vr.k_unMaxTrackedDeviceCount):
            if poses[i].bDeviceIsConnected and vr.VRSystem().getTrackedDeviceClass(i) == vr.TrackedDeviceClass_GenericTracker:
                serial_num = vr.VRSystem().getStringTrackedDeviceProperty(i, vr.Prop_SerialNumber_String)
                model = vr.VRSystem().getStringTrackedDeviceProperty(i, vr.Prop_ModelNumber_String)
                self.battery = vr.VRSystem().getFloatTrackedDeviceProperty(i, vr.Prop_DeviceBatteryPercentage_Float)
                print(("Found " + model + " : " + serial_num + " battery: %.0f%%") % (self.battery*100))
                self.tracker_index = i
                self.vr = vr
                return
        print("Could not find active tracker")

    def cmd_stop(self):
        self.stop = True

    def cmd_steamvr(self, args):
        '''command processing'''
        if len(args) == 0:
            print("Usage: steamvr <set|start|stop>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.cmd_stop()
        elif args[0] == "set":
            self.vr_settings.command(args[1:])

    def idle_task(self):
        '''run on idle'''
        if self.frame_count == self.last_frame_count:
            return
        self.last_frame_count = self.frame_count
        if self.vr is None:
            self.console.set_status('VPos', 'VPos -- -- --', row=5)
            self.console.set_status('VAtt', 'VAtt -- -- --', row=5)
            self.console.set_status('VRate', 'VRate GPS -- VIS -- Frame --', row=5)
            self.console.set_status('VState', 'VState BAT -- Tick --', row=5)
        else:
            if self.health:
                self.console.set_status('VPos', 'VPos %.2f %.2f %.2f' % (self.pos.x, self.pos.y, self.pos.z), row=5)
                self.console.set_status('VAtt', 'VAtt %.2f %.2f %.2f' % (self.att[0], self.att[1], self.att[2]), row=5)
                self.console.set_status('VRate', 'VRate GPS %0.1f VIS %0.1f Frame %.1f' % (self.gps_rate, self.vision_rate, self.frame_rate), row=5)
                self.console.set_status('VState', 'VState BAT %0.0f%% Tick %u' % (self.battery*100, self.frame_count), row=5)
            else:
                self.console.set_status('VPos', 'VPos %.2f %.2f %.2f' % (self.pos.x, self.pos.y, self.pos.z), fg='red', row=5)
                self.console.set_status('VAtt', 'VAtt %.2f %.2f %.2f' % (self.att[0], self.att[1], self.att[2]), fg='red', row=5)
                self.console.set_status('VRate', 'VRate GPS %0.1f VIS %0.1f Frame %.1f' % (self.gps_rate, self.vision_rate, self.frame_rate), fg='red', row=5)
                self.console.set_status('VState', 'VState BAT %0.0f%% Tick %u' % (self.battery*100, self.frame_count), fg='red', row=5)


def init(mpstate):
    '''initialise module'''
    return SteamVR_Module(mpstate)
