#!/usr/bin/env python
'''
antenna pointing module
Andrew Tridgell
June 2012
'''

import sys, os, time
from cuav.lib import cuav_util
from MAVProxy.modules.lib import mp_module

class AntennaModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(AntennaModule, self).__init__(mpstate, "antenna", "antenna pointing module")
        self.gcs_location = None
        self.last_bearing = 0
        self.last_announce = 0
        self.add_command('antenna', self.cmd_antenna, "antenna link control")

    def cmd_antenna(self, args):
        '''set gcs location'''
        if len(args) != 2:
            if self.gcs_location is None:
                print("GCS location not set")
            else:
                print("GCS location %s" % str(self.gcs_location))
            return
        self.gcs_location = (float(args[0]), float(args[1]))



    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if self.gcs_location is None and self.module('wp').wploader.count() > 0:
            home = self.module('wp').wploader.wp(0)
            self.gcs_location = (home.x, home.y)
            print("Antenna home set")
        if self.gcs_location is None:
            return
        if m.get_type() == 'GPS_RAW' and self.gcs_location is not None:
            (gcs_lat, gcs_lon) = self.gcs_location
            bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat, m.lon)
        elif m.get_type() == 'GPS_RAW_INT' and self.gcs_location is not None:
            (gcs_lat, gcs_lon) = self.gcs_location
            bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat / 1.0e7, m.lon / 1.0e7)
        else:
            return
        self.console.set_status('Antenna', 'Antenna %.0f' % bearing, row=0)
        if abs(bearing - self.last_bearing) > 5 and (time.time() - self.last_announce) > 15:
            self.last_bearing = bearing
            self.last_announce = time.time()
            self.say("Antenna %u" % int(bearing + 0.5))

def init(mpstate):
    '''initialise module'''
    return AntennaModule(mpstate)
