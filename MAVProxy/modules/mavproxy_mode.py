#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode", public=True)
        self.add_command('mode', self.cmd_mode, "mode change", self.available_modes())
        self.add_command('guided', self.cmd_guided, "fly to a clicked location on map")

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

    def available_modes(self):
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
        if len(args) != 1 and len(args) != 3:
            print("Usage: guided ALTITUDE | guided LAT LON ALTITUDE")
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

        print("Guided %s %s" % (str(latlon), str(altitude)))
        self.master.mav.mission_item_int_send (self.settings.target_system,
                                           self.settings.target_component,
                                           0,
                                           self.module('wp').get_default_frame(),
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                           2, 0, 0, 0, 0, 0,
                                           int(latlon[0]*1.0e7),
                                           int(latlon[1]*1.0e7),
                                           altitude)

def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
