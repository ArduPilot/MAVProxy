#!/usr/bin/env python
'''command long'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class CmdlongModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CmdlongModule, self).__init__(mpstate, "cmdlong")
        self.add_command('setspeed', self.cmd_do_change_speed, "do_change_speed")
        self.add_command('setyaw', self.cmd_condition_yaw, "condition_yaw")
        self.add_command('takeoff', self.cmd_takeoff, "takeoff")
        self.add_command('velocity', self.cmd_velocity, "velocity")
        self.add_command('position', self.cmd_position, "position")
        self.add_command('attitude', self.cmd_attitude, "attitude")
        self.add_command('cammsg', self.cmd_cammsg, "cammsg")
        self.add_command('cammsg_old', self.cmd_cammsg_old, "cammsg_old")
        self.add_command('camctrlmsg', self.cmd_camctrlmsg, "camctrlmsg")
        self.add_command('posvel', self.cmd_posvel, "posvel")
        self.add_command('parachute', self.cmd_parachute, "parachute",
                         ['<enable|disable|release>'])
        self.add_command('long', self.cmd_long, "execute mavlink long command",
                         self.cmd_long_commands())

    def cmd_long_commands(self):
        atts = dir(mavutil.mavlink)
        atts = filter( lambda x : x.lower().startswith("mav_cmd"), atts)
        ret = []
        for att in atts:
            ret.append(att)
            ret.append(str(att[8:]))
        return ret

    def cmd_takeoff(self, args):
        '''take off'''
        if ( len(args) != 1):
            print("Usage: takeoff ALTITUDE_IN_METERS")
            return
        
        if (len(args) == 1):
            altitude = float(args[0])
            print("Take Off started")
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                0, # confirmation
                0, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                altitude) # param7

    def cmd_parachute(self, args):
        '''parachute control'''
        usage = "Usage: parachute <enable|disable|release>"
        if len(args) != 1:
            print(usage)
            return

        cmds = {
            'enable'  : mavutil.mavlink.PARACHUTE_ENABLE,
            'disable' : mavutil.mavlink.PARACHUTE_DISABLE,
            'release' : mavutil.mavlink.PARACHUTE_RELEASE
            }
        if not args[0] in cmds:
            print(usage)
            return
        cmd = cmds[args[0]]
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            0,
            cmd,
            0, 0, 0, 0, 0, 0)

    def cmd_camctrlmsg(self, args):
        '''camctrlmsg'''
        
        print("Sent DIGICAM_CONFIGURE CMD_LONG")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE, # command
            0, # confirmation
            10, # param1
            20, # param2
            30, # param3
            40, # param4
            50, # param5
            60, # param6
            70) # param7

    def cmd_cammsg(self, args):
        '''cammsg'''
  
        print("Sent DIGICAM_CONTROL CMD_LONG")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL, # command
            0, # confirmation
            10, # param1
            20, # param2
            30, # param3
            40, # param4
            50, # param5
            60, # param6
            70) # param7

    def cmd_cammsg_old(self, args):
        '''cammsg_old'''
  
        print("Sent old DIGICAM_CONTROL")
        self.master.mav.digicam_control_send(
            self.settings.target_system,  # target_system
            0, # target_component
            0, 0, 0, 0, 1, 0, 0, 0)

    def cmd_do_change_speed(self, args):
        '''speed value'''
        if ( len(args) != 1):
            print("Usage: speed SPEED_VALUE")
            return
        
        if (len(args) == 1):
            speed = float(args[0])
            print("SPEED %s" % (str(speed)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                0, # confirmation
                0, # param1
                speed, # param2 (Speed value)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def cmd_condition_yaw(self, args):
        '''yaw angle angular_speed angle_mode'''
        if ( len(args) != 3):
            print("Usage: yaw ANGLE ANGULAR_SPEED MODE:[0 absolute / 1 relative]")
            return
        
        if (len(args) == 3):
            angle = float(args[0])
            angular_speed = float(args[1])
            angle_mode = float(args[2])
            print("ANGLE %s" % (str(angle)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                0, # confirmation
                angle, # param1 (angle value)
                angular_speed, # param2 (angular speed value)
                0, # param3
                angle_mode, # param4 (mode: 0->absolute / 1->relative)
                0, # param5
                0, # param6
                0) # param7

    def cmd_velocity(self, args):
        '''velocity x-ms y-ms z-ms'''
        if (len(args) != 3):
            print("Usage: velocity x y z (m/s)")
            return

        if (len(args) == 3):
            x_mps = float(args[0])
            y_mps = float(args[1])
            z_mps = float(args[2])
            print("x:%f, y:%f, z:%f" % (x_mps, y_mps, z_mps))
            self.master.mav.set_position_target_local_ned_send(
                                      0,  # system time in milliseconds
                                      1,  # target system
                                      0,  # target component
                                      8,  # coordinate frame MAV_FRAME_BODY_NED
                                      455,      # type mask (vel only)
                                      0, 0, 0,  # position x,y,z
                                      x_mps, y_mps, z_mps,  # velocity x,y,z
                                      0, 0, 0,  # accel x,y,z
                                      0, 0)     # yaw, yaw rate

    def cmd_position(self, args):
        '''position x-m y-m z-m'''
        if (len(args) != 3):
            print("Usage: position x y z (meters)")
            return

        if (len(args) == 3):
            x_m = float(args[0])
            y_m = float(args[1])
            z_m = float(args[2])
            print("x:%f, y:%f, z:%f" % (x_m, y_m, z_m))
            self.master.mav.set_position_target_local_ned_send(
                                      0,  # system time in milliseconds
                                      1,  # target system
                                      0,  # target component
                                      8,  # coordinate frame MAV_FRAME_BODY_NED
                                      3576,     # type mask (pos only)
                                      x_m, y_m, z_m,  # position x,y,z
                                      0, 0, 0,  # velocity x,y,z
                                      0, 0, 0,  # accel x,y,z
                                      0, 0)     # yaw, yaw rate

    def cmd_attitude(self, args):
        '''attitude q0 q1 q2 q3 thrust'''
        if len(args) != 5:
            print("Usage: attitude q0 q1 q2 q3 thrust (0~1)")
            return

        if len(args) == 5:
            q0 = float(args[0])
            q1 = float(args[1])
            q2 = float(args[2])
            q3 = float(args[3])
            thrust = float(args[4])
            att_target = [q0, q1, q2, q3]
            print("q0:%.3f, q1:%.3f, q2:%.3f q3:%.3f thrust:%.2f" % (q0, q1, q2, q3, thrust))
            self.master.mav.set_attitude_target_send(
                                      0,  # system time in milliseconds
                                      1,  # target system
                                      0,  # target component
                                      63, # type mask (ignore all except attitude + thrust)
                                      att_target, # quaternion attitude
                                      0,  # body roll rate
                                      0,  # body pich rate
                                      0,  # body yaw rate
                                      thrust)  # thrust

    def cmd_posvel(self, args):
        '''posvel mapclick vN vE vD'''
        ignoremask = 511
        latlon = None
        try:
            latlon = self.module('map').click_position
        except Exception:
            pass
        if latlon is None:
            print "set latlon to zeros"
            latlon = [0, 0]
        else:
            ignoremask = ignoremask & 504
            print "found latlon", ignoremask            
        vN = 0
        vE = 0
        vD = 0
        if (len(args) == 3):
            vN = float(args[0])
            vE = float(args[1])
            vD = float(args[2])
            ignoremask = ignoremask & 455

        print "ignoremask",ignoremask
        print latlon
        self.master.mav.set_position_target_global_int_send(
            0,  # system time in ms
            1,  # target system
            0,  # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            ignoremask, # ignore
            int(latlon[0] * 1e7),
            int(latlon[1] * 1e7),
            10,
            vN, vE, vD, # velocity
            0, 0, 0, # accel x,y,z
            0, 0) # yaw, yaw rate

    def cmd_long(self, args):
        '''execute supplied command long'''
        if len(args) < 1:
            print("Usage: long <command> [arg1] [arg2]...")
            return
        command = None
        if args[0].isdigit():
            command = int(args[0])
        else:
            try:
                command = eval("mavutil.mavlink." + args[0])
            except AttributeError as e:
                try:
                    command = eval("mavutil.mavlink.MAV_CMD_" + args[0])
                except AttributeError as e:
                    pass

        if command is None:
            print("Unknown command long ({0})".format(args[0]))
            return

        floating_args = [ float(x) for x in args[1:] ]
        while len(floating_args) < 7:
            floating_args.append(float(0))
        self.master.mav.command_long_send(self.settings.target_system,
                                          self.settings.target_component,
                                          command,
                                          0,
                                          *floating_args)

def init(mpstate):
    '''initialise module'''
    return CmdlongModule(mpstate)
