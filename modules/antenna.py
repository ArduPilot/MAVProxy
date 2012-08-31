#!/usr/bin/env python
'''
antenna pointing module
Andrew Tridgell
June 2012
'''

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'lib'))
import cuav_util

mpstate = None

class module_state(object):
    def __init__(self):
        self.gcs_location = None

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
    if m.get_type() == 'GPS_RAW' and state.gcs_location is not None:
        (gcs_lat, gcs_lon) = state.gcs_location
        bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat, m.lon)
        mpstate.console.set_status('Antenna', 'Antenna %.0f' % bearing, row=0)
    if m.get_type() == 'GPS_RAW_INT' and state.gcs_location is not None:
        (gcs_lat, gcs_lon) = state.gcs_location
        bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, m.lat/1.0e7, m.lon/1.0e7)
        mpstate.console.set_status('Antenna', 'Antenna %.0f' % bearing, row=0)
