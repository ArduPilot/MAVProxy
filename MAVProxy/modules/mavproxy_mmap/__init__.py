import os
import sys
import webbrowser

import mmap_server

g_module_context = None

from MAVProxy.modules.lib import mp_module

class MMapModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MMapModule, self).__init__(mpstate, 'mmap', 'modest map display')
        self.lat = None
        self.lon = None
        self.alt = None
        self.speed = None
        self.airspeed = None
        self.groundspeed = None
        self.heading = 0
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.server = None
        self.server = mmap_server.start_server('127.0.0.1', port=9999, module_state=self)
        webbrowser.open('http://127.0.0.1:9999/', autoraise=True)

    def unload(self):
        """unload module"""
        self.server.terminate()

    def mavlink_packet(self, m):
        """handle an incoming mavlink packet"""
        mtype = m.get_type()
        if mtype == 'GPS_RAW':
            (self.lat, self.lon) = (m.lat, m.lon)
        elif mtype == 'GPS_RAW_INT':
            (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
        elif mtype == "VFR_HUD":
            self.heading = m.heading
            self.alt = m.alt
            self.airspeed = m.airspeed
            self.groundspeed = m.groundspeed

def init(mpstate):
    '''initialise module'''
    return MMapModule(mpstate)