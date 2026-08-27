#!/usr/bin/env python3
'''
Compatibility shim for MAV_CMD entries that are newer than the
installed pymavlink release.

ArduPilot 4.7 added support for MAV_CMD_NAV_ARC_WAYPOINT (MAVLink command
id 36), which lets Copter fly an arc from the previous mission item to
this one instead of a straight line, but no released pymavlink build has
picked up the corresponding mavlink message-definition change yet.
Without an entry in mavutil.mavlink.enums['MAV_CMD'], pymavlink's own
MAVWPLoader.is_location_command() (and everything built on top of it -
mission path drawing, the mission editor's command list, and its
location/distance handling) silently treats MAV_CMD_NAV_ARC_WAYPOINT as
an unrecognised command with no location, so it gets dropped rather than
mis-rendered.

Register the missing enum entry here, once, so the rest of MAVProxy can
treat MAV_CMD_NAV_ARC_WAYPOINT like any other recognised nav command
without special-casing it in every consumer. This becomes a no-op as
soon as a pymavlink release that already defines it is installed.
'''

from pymavlink import mavutil

MAV_CMD_NAV_ARC_WAYPOINT = getattr(mavutil.mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT', 36)

if MAV_CMD_NAV_ARC_WAYPOINT not in mavutil.mavlink.enums['MAV_CMD']:
    # Re-use whatever EnumEntry class this pymavlink build's generated
    # dialect already uses, rather than importing a specific dialect
    # module, so this keeps working regardless of which dialect
    # mavutil.mavlink resolves to.
    enum_entry_class = type(next(iter(mavutil.mavlink.enums['MAV_CMD'].values())))
    entry = enum_entry_class(
        'MAV_CMD_NAV_ARC_WAYPOINT',
        'Navigate to the waypoint given by param5/param6/param7, arriving '
        'via an arc from the previous mission item rather than a straight '
        'line.',
    )
    entry.has_location = True
    entry.param = {
        1: 'Arc Angle. Signed angle subtended by the arc, in degrees '
           '(-359 to 359). Positive is a clockwise turn, negative is '
           'counter-clockwise.',
        2: 'Empty',
        3: 'Empty',
        4: 'Empty',
        5: 'Latitude',
        6: 'Longitude',
        7: 'Altitude',
    }
    mavutil.mavlink.enums['MAV_CMD'][MAV_CMD_NAV_ARC_WAYPOINT] = entry
    if not hasattr(mavutil.mavlink, 'MAV_CMD_NAV_ARC_WAYPOINT'):
        mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT = MAV_CMD_NAV_ARC_WAYPOINT
