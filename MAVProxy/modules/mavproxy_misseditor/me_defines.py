from pymavlink import mavutil
import fnmatch

miss_cmds = {}
frame_enum = {0: "Abs", 3: "Rel", 10: "AGL"}
frame_enum_rev = {v:k for k,v in frame_enum.items()}

# auto-generate the list of mission commands
for cmd in mavutil.mavlink.enums['MAV_CMD']:
    enum = mavutil.mavlink.enums['MAV_CMD'][cmd]
    name = enum.name
    name = name.replace('MAV_CMD_','')
    if name == 'ENUM_END':
        continue
    miss_cmds[cmd] = name

def cmd_reverse_lookup(command_name):
    '''returns 0 if key not found'''
    for key, value in miss_cmds.items():
        if (value.upper() == command_name.upper()):
            return key
    return 0

# a wildcard map from parameter descriptions to column names. If not found in this map
# then the default "Pn" is used
description_map = [
    ('Empty'             , '-'),
    ('Latitude*'         , 'Lat'),
    ('Longitude*'        , 'Lon'),
    ('Altitude*'         , 'Alt'),
    ('Minimum pitch*'    , 'Pitch'),
    ('Yaw*'              , 'Yaw'),
    ('Desired yaw*'      , 'Yaw'),
    ('Radius*'           , 'Radius'),
    ('Turns*'            , 'Turns'),
    ('Seconds*'          , 'Time(s)'),
    ('Delay in seconds*' , 'Time(s)'),
    ('On / Off*'         , 'Enable'),
    ('Descent / Ascend'  , 'Rate'),
    ('Finish Altitude'   , 'Altitude'),
    ('Distance*'         , 'Distance'),
    ('Mode*'             , 'Mode'),
    ('Custom mode*'      , 'CustomMode'),
    ('Sequence*'         , 'Seq'),
    ('Repeat*'           , 'Repeat'),
    ('Speed type*'       , 'SpeedType'),
    ('Speed*'            , 'Speed'),
    ('Throttle*'         , 'Throttle')
    ]

def make_column_label(command_name, description, default):
    '''try to work out a reasonable column name from parameter description'''
    for (pattern, label) in description_map:
        if fnmatch.fnmatch(description, pattern):
            return label
    return default


def get_column_labels(command_name):
    '''return dictionary of column labels if available'''
    cmd = cmd_reverse_lookup(command_name)
    if cmd == 0:
        return {}
    labels = {}
    enum = mavutil.mavlink.enums['MAV_CMD'][cmd]
    for col in enum.param.keys():
        labels[col] = make_column_label(command_name, enum.param[col], "P%u" % col)
    return labels
