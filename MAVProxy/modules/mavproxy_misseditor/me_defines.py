
miss_cmds = {16: 'Waypoint',
        17: 'Loiter Unlim',
        18: 'Loiter Turns',
        19: 'Loiter Time',
        20: 'RTL',
        21: 'Land',
        22: 'Takeoff',
        176: 'Do Set Mode',
        177: 'Do Jump',
        178: 'Do Chng Speed'}

def cmd_reverse_lookup(command_name):
    '''returns 0 if key not found'''
    for key, value in miss_cmds.items():
        if (value.upper() == command_name.upper()):
            return key

    return 0
