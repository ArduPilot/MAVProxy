#!/usr/bin/env python

'''
Proximity sensor module
Peter Barker, August 2020

Deals with proximity sensor data like 360-degree lidars from the
vehicle.  Plots them on the map, for example.

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.mavproxy_map import mp_slipmap

class proximity(mp_module.MPModule):
    def __init__(self, mpstate, multi_vehicle=True):
        """Initialise module"""
        super(proximity, self).__init__(mpstate, "proximity", "")

        self.proximity_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('proximity',
                         self.cmd_proximity,
                         "proximity module",
                         ['set (PROXIMITYSETTING)',
                         ]
        )

        self.heading_for_vehicle = {}
        self.have_global_position = False

        # lat/lon per system ID
        self.lat_lon_for_vehicle = {}

    def usage(self):
        '''show help on command line options'''
        return "Usage: proximity <set>"

    def cmd_proximity(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "set":
            self.proximity_settings.command(args[1:])
        else:
            print(self.usage())

    def idle_task(self):
        '''called rapidly by mavproxy'''

    def foreach_map(self, closure):
        for mp in self.module_matching('map*'):
            closure(mp.map)

    def mavlink_packet_distance_sensor(self, vehicle, m):
        heading = self.heading_for_vehicle.get(vehicle, 0)
        tlayer = "Distance sensor for %u.%u id=%u" % (
            m.get_srcSystem(), m.get_srcComponent(), m.id)
        slipkey = '%s-POS%u' % (tlayer, m.orientation)
        if not m.get_srcSystem() in self.lat_lon_for_vehicle:
            return
        if m.current_distance == m.max_distance:
            self.foreach_map(lambda a_map : a_map.remove_object(slipkey))
            return

        (lat,lon) = self.lat_lon_for_vehicle[m.get_srcSystem()]
        mav_sensor_rotation_to_degrees = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 0,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 45,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 90,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135 : 135,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 180,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225 : 225,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270 : 270,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315 : 315,
        }
        if m.orientation in mav_sensor_rotation_to_degrees:
            degrees = mav_sensor_rotation_to_degrees[m.orientation]
        else:
#            print("bad orientation (%u)" % m.orientation)
            return
        if m.current_distance >= m.max_distance:
            return

        p = mp_util.gps_newpos(lat, lon, heading+degrees, 0)
        # start angle/end angle are either side of the primary axis,
        # which is rotated to be North
        start_angle = -22.5
        end_angle = 22.5
        self.foreach_map(lambda a_map :
                         a_map.add_object(mp_slipmap.SlipCircle(
                             slipkey,
                             3,
                             p,
                             m.current_distance/100.0,
                             (255, 127, 0),
                             linewidth=3,
                             start_angle=start_angle,
                             end_angle=end_angle,
                             rotation=(-90+(heading+degrees))%360,
                         ))
        )

    def mavlink_packet_obstacle_distance(self, vehicle, m):
        heading = self.heading_for_vehicle.get(vehicle, 0)
        tlayer = "Obstacle Distance for %u.%u type=%u" % (
            m.get_srcSystem(), m.get_srcComponent(), m.sensor_type)
        if m.get_srcSystem() in self.lat_lon_for_vehicle:
            color = (255, 0, 255)
            (lat,lon) = self.lat_lon_for_vehicle[m.get_srcSystem()]
            self.foreach_map(lambda a_map : a_map.add_object(mp_slipmap.SlipClearLayer(tlayer)))
            increment = m.increment_f
            if increment == 0:
                increment = float(m.increment)
            measurement_count = 0
            for i in range(0, 72):
                if m.distances[i] != 65535:
                    measurement_count += 1
            fov = measurement_count*increment
            start_angle = -increment/2.0
            end_angle = increment/2.0
            rotation_start = -90 + heading
            for i in range(0, 72):
                slipkey = '%s-POS%u' % (tlayer, i)
                if m.distances[i] == m.max_distance+1:
                    # no measurement
                    self.foreach_map(lambda a_map : a_map.remove_object(slipkey))
                    measurement_count += 1
                    continue
                distance = m.distances[i] / 100.0 # cm -> m
                circle = mp_slipmap.SlipCircle(
                    slipkey,
                    3,
                    (lat, lon),
                    distance,
                    color,
                    linewidth=3,
                    start_angle=start_angle,
                    end_angle=end_angle,
                    rotation=(m.angle_offset+rotation_start+i*increment)%360,
                )
                if m.distances[i] < m.min_distance:
                    measurement_count += 1
                self.foreach_map(lambda a_map : a_map.add_object(circle))

            slipkey = "%s-range" % tlayer
            if fov == 360:
                # perfect circle
                # add a circle for max-range
                self.foreach_map(lambda a_map : a_map.add_object(mp_slipmap.SlipCircle(
                    slipkey,
                    3,
                    (lat, lon),
                    m.max_distance/100.0,
                    color,
                    linewidth=1,
                )))
            else:
                # add an arc for max-range
                self.foreach_map(lambda a_map : a_map.add_object(mp_slipmap.SlipCircle(
                    slipkey,
                    3,
                    (lat, lon),
                    m.max_distance/100.0,
                    color,
                    linewidth=1,
                    start_angle=-fov/2,
                    end_angle=fov/2,
                    rotation=(m.angle_offset+-90+(heading))%360,
                    add_radii = True,
                )))

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        vehicle = 'Vehicle%u' % m.get_srcSystem()
        if mtype == 'GLOBAL_POSITION_INT':
            (lat, lon, heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
            self.heading_for_vehicle[vehicle] = heading
            self.lat_lon_for_vehicle[m.get_srcSystem()] = (lat,lon)
            if abs(lat) > 1.0e-3 or abs(lon) > 1.0e-3:
                self.have_global_position = True
        elif mtype == 'LOCAL_POSITION_NED' and not self.have_global_position:
            self.heading_for_vehicle[vehicle] = math.degrees(math.atan2(m.vy, m.vx))
        elif mtype == 'DISTANCE_SENSOR':
            self.mavlink_packet_distance_sensor(vehicle, m)
        elif mtype == 'OBSTACLE_DISTANCE':
            self.mavlink_packet_obstacle_distance(vehicle, m)

def init(mpstate):
    '''initialise module'''
    return proximity(mpstate)
