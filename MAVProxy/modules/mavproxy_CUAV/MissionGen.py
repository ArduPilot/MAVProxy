#!/usr/bin/env python
'''mission waypoint generator'''

import time, threading, sys, os, numpy, Queue

# use the mission generator code from the cuav repo (see githib.com/stephendade)
from pymavlink import mavutil
from cuav.lib import cuav_missiongenerator


mpstate = None

def name():
    '''return module name'''
    return "MissionGen"

def description():
    '''return module description'''
    return "Mission search area waypoint generator"

def cmd_MissionGen():
    '''create the mission waypoints'''
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..',
                            'cuav', 'data', 'OBC Waypoints.kml')

    gen = cuav_missiongenerator.MissionGenerator(path)
    gen.Process('SA-', 'MB-')
    gen.CreateEntryExitPoints('EL-1,EL-2', 'EL-3,EL-4')

    gen.CreateSearchPattern(width = 150, overlap=50, offset=10, wobble=1, alt=90)
    gen.altitudeCompensation(heightAGL = 90)
    #gen.ExportSearchPattern()

    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component

    #export the waypoints to MAVProxy
    gen.exportToMAVProxy(mpstate.status.wploader)

    #and upload the new waypoints to the APM
    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(mpstate.status.wploader.count())

    '''and exit'''
    unload()

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.MissionGen_state = cmd_MissionGen()
    print("Mission Generator initialised")

def unload():
    '''unload module'''
    mpstate.MissionGen_state = None
    print('Mission Generator unload OK')


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''


