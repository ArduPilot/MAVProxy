#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode")
        self.add_command('mode', self.cmd_mode, "mode change")
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
        mode = args[0].upper()
        if mode not in mode_mapping:
            print('Unknown mode %s: ' % mode)
            return
        self.master.set_mode(mode_mapping[mode])

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
        if ( len(args) != 1 and len(args) != 3):
            print("Usage: guided ALTITUDE")
            return
        
        if (len(args) == 1):
            try:
                latlon = self.map_state.click_position
            except Exception:
                print("No map available")
                return
            if latlon is None:
                print("No map click position available")
                return        
            altitude = int(args[0])
            print("Guided %s %d" % (str(latlon), altitude))
            self.master.mav.mission_item_send (self.status.target_system,
                                                   self.status.target_component,
                                                   0,
                                                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                   2, 0, 0, 0, 0, 0,
                                                   latlon[0], latlon[1], altitude)
            
        if (len(args) == 3):
            latitude = float(args[0])
            longitude = float(args[1])
            altitude = int(args[2])
            print("Guided %s %s %d" % (str(latitude), str(longitude), altitude))
            self.master.mav.mission_item_send(self.status.target_system,
                                                   self.status.target_component,
                                                   0,
                                                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                   2, 0, 0, 0, 0, 0,
                                                   latitude, longitude, altitude)

def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
