#!/usr/bin/env python
'''mode command handling'''

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

'''
AP_FLAKE8_CLEAN
'''


class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode", public=True)
        self.add_command('mode', self.cmd_mode, "mode change", self.available_modes())
        self.add_command('guided', self.cmd_guided, "fly to a clicked location on map")
        self.add_command('confirm', self.cmd_confirm, "confirm a command")

    def cmd_mode(self, args):
        '''set arbitrary mode'''
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        if len(args) != 1:
            print('Available modes: ', mode_mapping.keys())
            return
        if args[0].isdigit():
            modenum = int(args[0])
        else:
            mode = args[0].upper()
            if mode not in mode_mapping:
                print('Unknown mode %s: ' % mode)
                return
            modenum = mode_mapping[mode]
        self.master.set_mode(modenum)

    def cmd_confirm(self, args):
        '''confirm a command'''
        if len(args) < 2:
            print('Usage: confirm "Question to display" command <arguments>')
            return
        question = args[0].strip('"')
        command = ' '.join(args[1:])
        if not mp_util.has_wxpython:
            print("No UI available for confirm")
            return
        from MAVProxy.modules.lib import mp_menu
        mp_menu.MPMenuConfirmDialog(question, callback=self.mpstate.functions.process_stdin, args=command)

    def available_modes(self):
        if self.master is None:
            print('No mode mapping available')
            return []
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return []
        return mode_mapping.keys()

    def unknown_command(self, args):
        '''handle mode switch by mode name as command'''
        mode_mapping = self.master.mode_mapping()
        mode = args[0].upper()
        if mode in mode_mapping:
            self.master.set_mode(mode_mapping[mode])
            return True
        return False

    def cmd_guided(self, args):
        '''set GUIDED target'''
        if len(args) > 0:
            if args[0] == "forward":
                return self.cmd_guided_forward(args[1:])

        if len(args) != 1 and len(args) != 3:
            print("Usage: guided ALTITUDE | guided LAT LON ALTITUDE | guided forward METRES")
            return

        if len(args) == 3:
            latitude = float(args[0])
            longitude = float(args[1])
            altitude = float(args[2])
            latlon = (latitude, longitude)
        else:
            latlon = self.mpstate.click_location
            if latlon is None:
                print("No map click position available")
                return
            altitude = float(args[0])

        altitude = self.height_convert_from_units(altitude)

        frame = self.flyto_frame()

        print("Guided %s %s frame %u" % (str(latlon), str(altitude), frame))

        if self.settings.guided_use_reposition:
            self.master.mav.command_int_send(
                self.settings.target_system,
                self.settings.target_component,
                frame,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,  # current
                0,  # autocontinue
                -1,   # p1 - ground speed, -1 is use-default
                mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,   # p2 - flags
                0,   # p3 - loiter radius for Planes, 0 is ignored
                0,   # p4 - yaw - 0 is loiter clockwise
                int(latlon[0]*1.0e7),
                int(latlon[1]*1.0e7),
                altitude
            )
            return

        self.master.mav.mission_item_int_send(
            self.settings.target_system,
            self.settings.target_component,
            0,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, 0, 0, 0,
            int(latlon[0]*1.0e7),
            int(latlon[1]*1.0e7),
            altitude
        )

    def build_pt_ignoremask(self, bits, force_not_accel=False):
        '''creates an ignore bitmask which ignores all bits except the ones passed in'''
        ignore_bits = {
            "X": 1,  # POSITION_TARGET_TYPEMASK_X_IGNORE
            "Y": 2,
            "Z": 4,
            "VX": 8,
            "VY": 16,
            "VZ": 32,
            "AX": 64,
            "AY": 128,
            "AZ": 256,
            "YAW": 1024,
            "YAWRATE": 2048,
        }
        value = 0
        for (n, v) in ignore_bits.items():
            if n not in bits:
                value |= v

        # as of ArduCopter 4.6.0-dev, you can't ignore any axis if you
        # want the others to be honoured.
        for prefix in "", "V", "A":
            for axis in "X", "Y", "Z":
                name = f"{prefix}{axis}"
                if (value & ignore_bits[name]) == 0:
                    # not ignoring this axis, so unmark the other axes as ignored
                    for resetaxis in "X", "Y", "Z":
                        resetname = f"{prefix}{resetaxis}"
                        value = value & ~ignore_bits[resetname]
                    break

        if force_not_accel:
            value |= 512  # POSITION_TARGET_TYPEMASK_FORCE_SET

        return value

    def cmd_guided_forward(self, args):
        if len(args) != 1:
            print("Usage: guided forward METRES")
            return
        offset = args[0]
        # see also "cmd_position" in mavproxy_cmdlong.py
        self.master.mav.set_position_target_local_ned_send(
            0,  # system time in milliseconds
            self.settings.target_system,  # target system
            0,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.build_pt_ignoremask(["X", "YAWRATE"]),     # type mask (pos-x-only)
            float(offset), 0, 0,  # position x,y,z
            0, 0, 0,  # velocity x,y,z
            0, 0, 0,  # accel x,y,z
            0, 0      # yaw, yaw rate
        )

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == 'HIGH_LATENCY2':
            mode_map = mavutil.mode_mapping_bynumber(m.type)
            if mode_map and m.custom_mode in mode_map:
                self.master.flightmode = mode_map[m.custom_mode]


def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
