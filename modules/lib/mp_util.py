#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


def wrap_valid_longitude(lon):
  ''' wrap a longitude value around to always have a value in the range
      [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
  '''
  return (((lon + 180.0) % 360.0) - 180.0)

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
	return (degrees(lat2), wrap_valid_longitude(degrees(lon2)))

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

def polygon_load(filename):
	'''load a polygon from a file'''
	ret = []
        f = open(filename)
        for line in f:
		if line.startswith('#'):
			continue
		line = line.strip()
		if not line:
			continue
		a = line.split()
		if len(a) != 2:
			raise RuntimeError("invalid polygon line: %s" % line)
		ret.append((float(a[0]), float(a[1])))
        f.close()
	return ret


def polygon_bounds(points):
	'''return bounding box of a polygon in (x,y,width,height) form'''
        (minx, miny) = points[0]
        (maxx, maxy) = points[0]
        for p in points:
            minx = min(minx, p[0])
            maxx = max(maxx, p[0])
            miny = min(miny, p[1])
            maxy = max(maxy, p[1])
	return (minx, miny, maxx-minx, maxy-miny)

def bounds_overlap(bound1, bound2):
	'''return true if two bounding boxes overlap'''
	(x1,y1,w1,h1) = bound1
	(x2,y2,w2,h2) = bound2
	if x1+w1 < x2:
		return False
	if x2+w2 < x1:
		return False
	if y1+h1 < y2:
		return False
	if y2+h2 < y1:
		return False
	return True


class object_container:
	'''return a picklable object from an existing object,
	containing all of the normal attributes of the original'''
	def __init__(self, object):
		for v in dir(object):
			if not v.startswith('__') and v not in ['this']:
				try:
					a = getattr(object, v)
					if (hasattr(a, '__call__') or
					    hasattr(a, '__swig_destroy__') or
					    str(a).find('Swig Object') != -1):
						continue
					setattr(self, v, a)
				except Exception:
					pass

def degrees_to_dms(degrees):
	'''return a degrees:minutes:seconds string'''
	deg = int(degrees)
	min = int((degrees - deg)*60)
	sec = ((degrees - deg) - (min/60.0))*60*60
	return u'%u\u00b0%02u\'%04.1f"' % (deg, abs(min), abs(sec))

