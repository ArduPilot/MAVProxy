#!/usr/bin/env python
'''
antenna pointing module
Andrew Tridgell
June 2012
'''

import sys, os, time
from cuav.lib import cuav_util

mpstate = None

class module_state(object):
    def __init__(self):
        self.gcs_location = None
        self.last_bearing = 0
        self.last_announce = 0

def name():
    '''return module name'''
    return "antenna"

def description():
    '''return module description'''
    return "antenna pointing module"

def cmd_antenna(args):
    '''set gcs location'''
    state = mpstate.antenna_state
    usage = "antenna lat lon"
    if len(args) != 2:
        if state.gcs_location is None:
            print("GCS location not set")
        else:
            print("GCS location %s" % str(state.gcs_location))
        return
    state.gcs_location = (float(args[0]), float(args[1]))
        
def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.antenna_state = module_state()
    mpstate.command_map['antenna'] = (cmd_antenna, "antenna link control")

def unload():
    '''unload module'''
    pass

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.antenna_state
    if state.gcs_location is None and mpstate.status.wploader.count() > 0:
        home = mpstate.status.wploader.wp(0)
        mpstate.antenna_state.gcs_location = (home.x, home.y)
        print("Antenna home set")
    if state.gcs_location is None:
        return
    if m.get_type() == 'GPS_RAW' and state.gcs_location is not None:
        (gcs_lat, gcs_lon) = state.gcs_location
        bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat, m.lon)
    elif m.get_type() == 'GPS_RAW_INT' and state.gcs_location is not None:
        (gcs_lat, gcs_lon) = state.gcs_location
        bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat/1.0e7, m.lon/1.0e7)
    else:
        return
    mpstate.console.set_status('Antenna', 'Antenna %.0f' % bearing, row=0)
    if abs(bearing - state.last_bearing) > 5 and (time.time() - state.last_announce) > 15:
        state.last_bearing = bearing
        state.last_announce = time.time()
        mpstate.functions.say("Antenna %u" % int(bearing+0.5))
