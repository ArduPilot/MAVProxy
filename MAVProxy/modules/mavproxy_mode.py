#!/usr/bin/env python3
'''mode command handling'''

import collections
import math

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

'''
AP_FLAKE8_CLEAN
'''

# names for MAV_CMD_DO_ORBIT param3, ORBIT_YAW_BEHAVIOUR values; NaN
# leaves the choice to the vehicle.  The ORBIT_YAW_BEHAVIOUR_* constants
# are not present in older pymavlink releases, so take them from
# mavutil.mavlink when available and otherwise fall back to the enum
# values from common.xml
orbit_yaw_behaviours = collections.OrderedDict([
    ("Default", float('NaN')),
    ("FaceCentre", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER', 0)),
    ("InitialHeading", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING', 1)),
    ("Uncontrolled", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_UNCONTROLLED', 2)),
    ("Tangent", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE', 3)),
    ("RCControlled", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED', 4)),
    ("Unchanged", getattr(mavutil.mavlink, 'ORBIT_YAW_BEHAVIOUR_UNCHANGED', 5)),
])

# orbit altitude-frame names mapped to their MAVLink frame (matching
# MPModule.flyto_frame); the non-_INT constants are correct for COMMAND_INT
orbit_frame_map = collections.OrderedDict([
    ('AboveHome', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
    ('AGL', mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT),
    ('AMSL', mavutil.mavlink.MAV_FRAME_GLOBAL),
])
orbit_frames = list(orbit_frame_map.keys())


def orbit_velocity(value):
    '''convert a DO_ORBIT tangential velocity argument to m/s; an empty
    string or 'default' means "use the vehicle default" (NaN).  Raises
    ValueError for anything else which is not a number.'''
    if value.lower() in ('', 'default'):
        return float('nan')
    return float(value)


def orbit_yaw(value):
    '''convert a DO_ORBIT yaw argument to an ORBIT_YAW_BEHAVIOUR value; a
    behaviour name, a numeric enum value, or an empty string ("use the
    vehicle default", NaN).  Raises ValueError for an unknown name that is
    not a number.'''
    if value == '':
        return float('nan')
    behaviours = {name.lower(): v for (name, v) in orbit_yaw_behaviours.items()}
    if value.lower() in behaviours:
        return behaviours[value.lower()]
    return float(value)


# single source of truth for the "orbit" command arguments: drives
# command-line parsing, command-line help, and the map "Orbit Here" popup
# dialog.  See MPModule.parse_key_value_spec / format_key_value_help and
# MPMenuCallMultiTextDropdownDialog.from_arg_spec for the facets consumed.
# Keys without a 'label' (loc/lat/lng) are command-line only: the popup
# takes its centre from the map click.
orbit_arg_spec = collections.OrderedDict([
    ('radius', dict(type=float, required=True,
                    help='metres, positive for clockwise, negative for counter-clockwise',
                    label='Radius (m, -ve for CCW)', default=50)),
    ('alt', dict(type=float, synonyms=['altitude'],
                 help='orbit altitude',
                 label='Altitude', default=lambda ctx: ctx['guidedalt'])),
    ('velocity', dict(type=orbit_velocity,
                      help="tangential velocity in m/s, 'default' for the vehicle default",
                      label='Velocity (m/s)', default='Default')),
    ('orbits', dict(type=float,
                    help='number of circuits to fly, 0 to orbit forever',
                    label='Orbits (0 for forever)', default=0)),
    ('yaw', dict(type=orbit_yaw, options=list(orbit_yaw_behaviours.keys()),
                 help='one of %s or an ORBIT_YAW_BEHAVIOUR enumeration value' %
                      '|'.join(orbit_yaw_behaviours.keys()),
                 label='Yaw Behaviour', default='Default')),
    ('frame', dict(type=str, options=orbit_frames,
                   help='altitude frame',
                   label='Frame', default=lambda ctx: ctx['flytoframe'])),
    ('loc', dict(type=str,
                 help='orbit centre as LAT,LNG or LAT,LNG,ALT; defaults to the map click position')),
    ('lat', dict(type=float, synonyms=['latitude'], help='orbit centre latitude')),
    ('lng', dict(type=float, synonyms=['longitude', 'lon'], help='orbit centre longitude')),
])


class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode", public=True)
        self.add_command('mode', self.cmd_mode, "mode change", [
            '(MODE)'
        ])
        self.add_command('guided', self.cmd_guided, "fly to a clicked location on map")
        self.add_command('orbit', self.cmd_orbit, "orbit around a clicked location on map")
        self.add_command('confirm', self.cmd_confirm, "confirm a command")
        self.add_completion_function('(MODE)', self.complete_available_modes)
        # set while we await the COMMAND_ACK for an orbit we sent, so a
        # rejection can be surfaced prominently rather than buried in the
        # generic COMMAND_ACK console line
        self.orbit_ack_pending = False

    def cmd_mode(self, args):
        '''set arbitrary mode'''
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        if len(args) != 1:
            print('Available modes: ', ', '.join(self.available_modes()))
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

    def complete_available_modes(self, text):
        return self.available_modes()

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

        if len(args) == 2:
            frames = ['AboveHome', 'AGL', 'AMSL']
            if args[1] in frames:
                self.settings.flytoframe = args[1]
            else:
                print("Usage: guided ALTITUDE %s" % '|'.join(frames))
                return
        elif len(args) != 1 and len(args) != 3:
            print("Usage: guided ALTITUDE | guided LAT LON ALTITUDE | guided forward METRES")
            return

        frame = self.flyto_frame()

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

    def cmd_orbit_usage(self):
        '''print usage for the orbit command'''
        print("Usage: orbit radius=RADIUS [alt=ALTITUDE] [velocity=VELOCITY] "
              "[orbits=ORBITS] [yaw=YAW] [frame=%s]" % '|'.join(orbit_frames))
        print("            [loc=LAT,LNG | lat=LAT lng=LNG]")
        for line in self.format_key_value_help(orbit_arg_spec):
            print(line)

    def cmd_orbit(self, args):
        '''send MAV_CMD_DO_ORBIT to orbit around the clicked location'''
        values = {}
        if not self.parse_key_value_spec(args, orbit_arg_spec, values):
            self.cmd_orbit_usage()
            return

        # work out the orbit centre: an explicit location if given, else the map click
        if 'loc' in values:
            if 'lat' in values or 'lng' in values:
                print("specify either loc= or lat=/lng=, not both")
                return
            parts = values['loc'].split(',')
            try:
                if len(parts) not in (2, 3):
                    raise ValueError
                latlon = (float(parts[0]), float(parts[1]))
                if len(parts) == 3 and 'alt' not in values:
                    values['alt'] = float(parts[2])
            except ValueError:
                print("loc must be LAT,LNG or LAT,LNG,ALT")
                return
        elif 'lat' in values or 'lng' in values:
            if 'lat' not in values or 'lng' not in values:
                print("both lat and lng are required")
                return
            latlon = (values['lat'], values['lng'])
        else:
            latlon = self.mpstate.click_location
            if latlon is None:
                print("No map click position available")
                return

        # reject a non-finite or out-of-range centre before it reaches the
        # degE7 integer conversion (which would raise) or the vehicle
        if (not math.isfinite(latlon[0]) or not math.isfinite(latlon[1]) or
                abs(latlon[0]) > 90 or abs(latlon[1]) > 180):
            print("orbit centre out of range: %s" % (str(latlon),))
            return

        # resolve the frame for this orbit only; unlike a bare "guided", an
        # explicit frame= must not rewrite the shared flytoframe setting
        if 'frame' in values:
            if values['frame'] not in orbit_frame_map:
                print("frame must be one of %s" % '|'.join(orbit_frames))
                return
            frame = orbit_frame_map[values['frame']]
        else:
            frame = self.flyto_frame()

        radius = values['radius']
        if 'alt' in values:
            altitude = values['alt']
        else:
            altitude = self.mpstate.settings.guidedalt
        altitude = self.height_convert_from_units(altitude)
        # velocity and yaw are already converted by orbit_arg_spec, and
        # default to NaN ("use the vehicle default") when not supplied
        velocity = values.get('velocity', float('NaN'))
        # orbits is an angle magnitude; the spec's minValue is 0 (direction
        # comes from the sign of radius), so never send a negative p4
        orbits = abs(values.get('orbits', 0))  # 0 orbits forever
        yaw_behaviour = values.get('yaw', float('NaN'))

        print("Orbit %s radius %.1fm alt %.1f frame %u" % (str(latlon), radius, altitude, frame))

        self.master.mav.command_int_send(
            self.settings.target_system,
            self.settings.target_component,
            frame,
            mavutil.mavlink.MAV_CMD_DO_ORBIT,
            0,  # current
            0,  # autocontinue
            radius,  # p1 - radius, +ve clockwise, -ve counter-clockwise
            velocity,  # p2 - tangential velocity, NaN is use-default
            yaw_behaviour,  # p3 - yaw behaviour, NaN is vehicle default
            orbits * 2 * math.pi,  # p4 - angle to orbit in radians, 0 for forever
            int(latlon[0]*1.0e7),
            int(latlon[1]*1.0e7),
            altitude
        )
        # watch for the ack so a rejection (e.g. not in GUIDED) is surfaced
        self.orbit_ack_pending = True

    def handle_orbit_command_ack(self, m):
        '''surface an orbit rejection prominently; the generic COMMAND_ACK
        console line is easily missed by a map-menu user'''
        if not self.orbit_ack_pending:
            return
        if m.command != mavutil.mavlink.MAV_CMD_DO_ORBIT:
            return
        self.orbit_ack_pending = False
        if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return
        try:
            res = mavutil.mavlink.enums["MAV_RESULT"][m.result].name[11:]
        except KeyError:
            res = str(m.result)
        print("Orbit rejected (%s); the vehicle must be in GUIDED mode to orbit" % res)

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
        elif mtype == 'COMMAND_ACK':
            self.handle_orbit_command_ack(m)


def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
