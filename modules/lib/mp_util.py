#!/usr/bin/env python
'''common mavproxy utility functions'''

import math, os

radius_of_earth = 6378100.0 # in meters

def gps_distance(lat1, lon1, lat2, lon2):
	'''return distance between two points in meters,
	coordinates are in degrees
	thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
	from math import radians, cos, sin, sqrt, atan2
	lat1 = radians(lat1)
	lat2 = radians(lat2)
	lon1 = radians(lon1)
	lon2 = radians(lon2)
	dLat = lat2 - lat1
	dLon = lon2 - lon1
	
	a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
	c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
	return radius_of_earth * c


def gps_bearing(lat1, lon1, lat2, lon2):
	'''return bearing between two points in degrees, in range 0-360
	thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
	from math import sin, cos, atan2, radians, degrees
	lat1 = radians(lat1)
	lat2 = radians(lat2)
	lon1 = radians(lon1)
	lon2 = radians(lon2)
	dLat = lat2 - lat1
	dLon = lon2 - lon1    
	y = sin(dLon) * cos(lat2)
	x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon)
	bearing = degrees(atan2(y, x))
	if bearing < 0:
		bearing += 360.0
	return bearing


def gps_newpos(lat, lon, bearing, distance):
	'''extrapolate latitude/longitude given a heading and distance 
	thanks to http://www.movable-type.co.uk/scripts/latlong.html
	'''
	from math import sin, asin, cos, atan2, radians, degrees

	lat1 = radians(lat)
	lon1 = radians(lon)
	brng = radians(bearing)
	dr = distance/radius_of_earth

	lat2 = asin(sin(lat1)*cos(dr) +
		    cos(lat1)*sin(dr)*cos(brng))
	lon2 = lon1 + atan2(sin(brng)*sin(dr)*cos(lat1), 
			    cos(dr)-sin(lat1)*sin(lat2))
	return (degrees(lat2), degrees(lon2))

def gps_offset(lat, lon, east, north):
	'''return new lat/lon after moving east/north
	by the given number of meters'''
	bearing = math.degrees(math.atan2(east, north))
	distance = math.sqrt(east**2 + north**2)
	return gps_newpos(lat, lon, bearing, distance)


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    try:
        os.mkdir(dir)
    except Exception:
        pass
