#!/usr/bin/env python3
'''
gimbal control module
Andrew Tridgell
January 2015
'''

import sys, os, time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map import mp_slipmap
from pymavlink import mavutil
from pymavlink.rotmat import Vector3, Matrix3, Plane, Line
from math import radians
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *
import pymavlink

class GimbalModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GimbalModule, self).__init__(mpstate, "gimbal", "gimbal control module")
        self.add_command(
            'gimbal',
            self.cmd_gimbal,
            "gimbal link control",
            ['<rate|angle|roi|roivel|mode|status|set>'])
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Mount',
                                      items=[MPMenuItem('GPS targeting Mode', 'GPS Mode', '# gimbal mode gps'),
                                             MPMenuItem('MAVLink targeting Mode', 'MAVLink Mode', '# gimbal mode mavlink'),
                                             MPMenuItem('RC targeting Mode', 'RC Mode', '# gimbal mode rc'),
                                             MPMenuItem('Point At', 'Point At', '# gimbal roi')])
            self.menu_added_map = False
        else:
            self.menu = None

        self.gimbal_settings = mp_settings.MPSettings([
            ("rate_send_hz", float, 1),  # rate at which to send rates
        ])

        self.add_completion_function('(GIMBALSETTING)',
                                     self.gimbal_settings.completion)

        self.rate_send_period = mavutil.periodic_event(self.gimbal_settings.rate_send_hz)
        self.send_rates = False
        self.rates_last_sent = 0

    def idle_task(self):
        '''called on idle'''
        if self.menu is not None and self.module('map') is not None and not self.menu_added_map:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)

        if self.send_rates:
            self.rate_send_period.frequency = self.gimbal_settings.rate_send_hz
            if self.rate_send_period.trigger():
                self.send_gimbal_rates()

    def cmd_gimbal(self, args):
        '''control gimbal'''
        usage = 'Usage: gimbal <rate|angle|roi|roivel|mode|status>'
        if len(args) == 0:
            print(usage)
            return
        if args[0] == 'rate':
            self.send_rates = False  # may be set to true in cmd_gimbal_rate.
            self.cmd_gimbal_rate(args[1:])
        elif args[0] == 'point' or args[0] == 'angle':
            self.cmd_gimbal_angle(args[1:])
            self.send_rates = False
        elif args[0] == 'roi':
            self.cmd_gimbal_roi(args[1:])
            self.send_rates = False
        elif args[0] == 'mode':
            self.cmd_gimbal_mode(args[1:])
            self.send_rates = False
        elif args[0] == 'status':
            self.cmd_gimbal_status(args[1:])
        elif args[0] == 'roivel':
            self.cmd_gimbal_roi_vel(args[1:])
            self.send_rates = False
        elif args[0] == "set":
            self.gimbal_settings.command(args[1:])
        else:
            print(usage)
            return

    def cmd_gimbal_mode(self, args):
        '''control gimbal mode'''
        if len(args) != 1:
            print("usage: gimbal mode <RETRACT|NEUTRAL|GPS|MAVLink|RC>")
            return
        if args[0].upper() == "RETRACT":
            mode = mavutil.mavlink.MAV_MOUNT_MODE_RETRACT
        elif args[0].upper() == "NEUTRAL":
            mode = mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL
        elif args[0].upper() == 'GPS':
            mode = mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT
        elif args[0].upper() == 'MAVLINK':
            mode = mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
        elif args[0].upper() == 'RC':
            mode = mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING
        else:
            print("Unsupported mode %s" % args[0])
            return
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            1,     # confirmation
            mode,  # p1
            1,     # p2 stabilize roll
            1,     # p3 stabilize pitch
            1,     # p4 stalize yaw
            0,     # p5 roll input mode (0 is angle body)
            0,     # p6 pitch input mode
            0,     # p7 yaw input mode
        )

    def cmd_gimbal_roi(self, args):
        '''control roi position'''
        latlon = None
        alt = 0
        if len(args) == 0:
            latlon = self.mpstate.click_location
        elif len(args) == 3:
            latlon = [float(i) for i in args[0:2]]
            alt = float(args[2])
        if latlon is None and len(args) == 0:
            print("No map click position available and no parameters set")
            return
        elif latlon is None:
            print("usage: gimbal roi [LAT LON RELHOMEALT]")
            return
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0, # confirmation
            0, # param1
            0, # param2
            0, # param3
            alt, # param4 - alt (exception to the usual rule about where this goes)
            int(latlon[0]*1e7), # lat
            int(latlon[1]*1e7), # lon
            mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT) # param7

    def cmd_gimbal_roi_vel(self, args):
        '''control roi position and velocity'''
        if len(args) != 0 and len(args) != 3 and len(args) != 6:
            print("usage: gimbal roivel [VEL_NORTH VEL_EAST VEL_DOWN] [ACC_NORTH ACC_EASY ACC_DOWN]")
            return
        latlon = None
        vel = [0,0,0]
        acc = [0,0,0]
        if (len(args) >= 3):
            vel[0:3] = args[0:3]
        if (len(args) == 6):
            acc[0:3] = args[3:6]
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            latlon = (0,0,0)
        self.master.mav.set_roi_global_int_send(0, #time_boot_ms
            1, #target_system
            1, #target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0, #type_mask
            0, #roi_index
            0, #timeout_ms
            int(latlon[0]*1e7), #lat int
            int(latlon[1]*1e7), #lng int
            float(0),       #alt
            float(vel[0]), #vx
            float(vel[1]), #vy
            float(vel[2]), #vz
            float(acc[0]), #ax
            float(acc[1]), #ay
            float(acc[2])) #az

    def cmd_gimbal_rate(self, args):
        '''control gimbal rate'''
        if len(args) != 3:
            print("usage: gimbal rate ROLL PITCH YAW")
            return
        self.rates = (float(args[0]), float(args[1]), float(args[2]))
        self.send_rates = True

    def send_gimbal_rates(self):
        roll, pitch, yaw = self.rates
        self.master.mav.gimbal_manager_set_attitude_send(
            self.target_system,
            self.target_component,
            0,  # flags
            0,  # instance, 0 is primary
            [float("NaN"), float("NaN"), float("NaN"), float("NaN")],
            radians(roll),
            radians(pitch),
            radians(yaw)
        )

    def cmd_gimbal_angle(self, args):
        '''control gimbal pointing'''
        if len(args) != 3:
            print("usage: gimbal angle ROLL PITCH YAW")
            return
        (roll, pitch, yaw) = (float(args[0]), float(args[1]), float(args[2]))
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0, # confirmation
            pitch, # param1
            roll, # param2
            yaw, # param3
            0, # param4
            0, # lat
            0, # lon
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING) # param7

    def cmd_gimbal_status(self, args):
        '''show gimbal status'''
        master = self.master
        if 'GIMBAL_REPORT' in master.messages:
            print(master.messages['GIMBAL_REPORT'])
        else:
            print("No GIMBAL_REPORT messages")

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''

        if not self.mpstate.map:
            # don't draw if no map
            return

        for n in frozenset(['ATTITUDE', 'GLOBAL_POSITION_INT']):
            if not n in self.master.messages:
                return

        gpi = self.master.messages['GLOBAL_POSITION_INT']
        att = self.master.messages['ATTITUDE']

        rotmat_copter_gimbal = Matrix3()
        if m.get_type() == 'GIMBAL_REPORT':
            rotmat_copter_gimbal.from_euler312(m.joint_roll, m.joint_el, m.joint_az)
        elif m.get_type() == 'GIMBAL_DEVICE_ATTITUDE_STATUS':
            r, p, y = pymavlink.quaternion.Quaternion(m.q).euler
            # rotate back from earth-frame on roll and pitch to body-frame
            r -= att.roll
            p -= att.pitch
            rotmat_copter_gimbal.from_euler(r, p, y)
        else:
            return

        # clear the camera icon
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('GimbalView'))

        vehicle_dcm = Matrix3()
        vehicle_dcm.from_euler(att.roll, att.pitch, att.yaw)

        gimbal_dcm = vehicle_dcm * rotmat_copter_gimbal

        lat = gpi.lat * 1.0e-7
        lon = gpi.lon * 1.0e-7
        alt = gpi.relative_alt * 1.0e-3

        # ground plane
        ground_plane = Plane()

        # the position of the camera in the air, remembering its a right
        # hand coordinate system, so +ve z is down
        camera_point = Vector3(0, 0, -alt)

        # get view point of camera when not rotated
        view_point = Vector3(1, 0, 0)

        # rotate view_point to form current view vector
        rot_point = gimbal_dcm * view_point

        # a line from the camera to the ground
        line = Line(camera_point, rot_point)

        # find the intersection with the ground
        pt = line.plane_intersection(ground_plane, forward_only=True)
        if pt is None:
            # its pointing up into the sky
            return None

        (view_lat, view_lon) = mp_util.gps_offset(lat, lon, pt.y, pt.x)

        icon = self.mpstate.map.icon('camera-small-red.png')
        self.mpstate.map.add_object(mp_slipmap.SlipIcon('gimbalview',
                                                        (view_lat,view_lon),
                                                        icon, layer='GimbalView', rotation=0, follow=False))


def init(mpstate):
    '''initialise module'''
    return GimbalModule(mpstate)
