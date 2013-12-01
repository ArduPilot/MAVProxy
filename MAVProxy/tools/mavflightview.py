#!/usr/bin/env python

'''
view a mission log on a map
'''

import sys, time, os

from pymavlink import mavutil, mavwp
from MAVProxy.modules.mavproxy_map import mp_slipmap, mp_tile
from MAVProxy.modules.lib import mp_util
import functools

try:
    import cv2.cv as cv
except ImportError:
    import cv

from optparse import OptionParser
parser = OptionParser("mavflightview.py [options]")
parser.add_option("--service", default="MicrosoftSat", help="tile service")
parser.add_option("--mode", default=None, help="flight mode")
parser.add_option("--condition", default=None, help="conditional check on log")
parser.add_option("--mission", default=None, help="mission file (defaults to logged mission)")
parser.add_option("--imagefile", default=None, help="output to image file")
parser.add_option("--flag", default=[], type='str', action='append', help="flag positions")
parser.add_option("--rawgps", action='store_true', default=False, help="use GPS_RAW_INT")
parser.add_option("--debug", action='store_true', default=False, help="show debug info")

(opts, args) = parser.parse_args()

def pixel_coords(latlon, ground_width=0, mt=None, topleft=None, width=None):
    '''return pixel coordinates in the map image for a (lat,lon)'''
    (lat,lon) = (latlon[0], latlon[1])
    return mt.coord_to_pixel(topleft[0], topleft[1], width, ground_width, lat, lon)

def create_imagefile(filename, latlon, ground_width, path_obj, mission_obj, width=600, height=600):
    '''create path and mission as an image file'''
    mt = mp_tile.MPTile(service=opts.service)

    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    while mt.tiles_pending() > 0:
        print("Waiting on %u tiles" % mt.tiles_pending())
        time.sleep(1)
    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    # a function to convert from (lat,lon) to (px,py) on the map
    pixmapper = functools.partial(pixel_coords, ground_width=ground_width, mt=mt, topleft=latlon, width=width)
    path_obj.draw(map_img, pixmapper, None)
    if mission_obj is not None:
        mission_obj.draw(map_img, pixmapper, None)
    cv.CvtColor(map_img, map_img, cv.CV_BGR2RGB)
    cv.SaveImage(filename, map_img)

colourmap = {
    'MANUAL'    : (255,   0,   0),
    'AUTO'      : (  0, 255,   0),
    'LOITER'    : (  0,   0, 255),
    'FBWA'      : (255, 100,   0),
    'RTL'       : (255,   0, 100),
    'STABILIZE' : (100, 255,   0),
    'LAND'      : (  0, 255, 100),
    'STEERING'  : (100,   0, 255),
    'HOLD'      : (  0, 100, 255),
    'ALT_HOLD'  : (255, 100, 100),
    'CIRCLE'    : (100, 255, 100),
    'GUIDED'    : (100, 100, 255),
    'ACRO'      : (255, 255,   0),
    'CRUISE'    : (0,   255, 255)
    }


def mavflightview(filename):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    wp = mavwp.MAVWPLoader()
    if opts.mission is not None:
        wp.load(opts.mission)
    path = []
    while True:
        m = mlog.recv_match(type=['MISSION_ITEM', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'GPS'])
        if m is None:
            break
        if m.get_type() == 'MISSION_ITEM':
            wp.set(m, m.seq)            
            continue
        if m.get_type() == 'GLOBAL_POSITION_INT' and opts.rawgps:
            continue
        if m.get_type() == 'GPS_RAW_INT' and not opts.rawgps:
            continue
        if not mlog.check_condition(opts.condition):
            continue
        if opts.mode is not None and mlog.flightmode.lower() != opts.mode.lower():
            continue
        if m.get_type() == 'GPS':
            status = getattr(m, 'Status', None)
            if status is None:
                status = getattr(m, 'FixType', None)
                if status is None:
                    print("Can't find status on GPS message")
                    print(m)
                    break
            if status < 2:
                continue
            # flash log
            lat = m.Lat
            lng = getattr(m, 'Lng', None)
            if lng is None:
                lng = getattr(m, 'Lon', None)
                if lng is None:
                    print("Can't find longitude on GPS message")
                    print(m)
                    break                    
        else:
            lat = m.lat * 1.0e-7
            lng = m.lon * 1.0e-7
        if lat != 0 or lng != 0:
            if getattr(mlog, 'flightmode','') in colourmap:
                point = (lat, lng, colourmap[mlog.flightmode])
            else:
                point = (lat, lng)
            path.append(point)
    if len(path) == 0:
        print("No points to plot")
        return
    bounds = mp_util.polygon_bounds(path)
    (lat, lon) = (bounds[0]+bounds[2], bounds[1])
    (lat, lon) = mp_util.gps_newpos(lat, lon, -45, 50)
    ground_width = mp_util.gps_distance(lat, lon, lat-bounds[2], lon+bounds[3])
    while (mp_util.gps_distance(lat, lon, bounds[0], bounds[1]) >= ground_width-20 or
           mp_util.gps_distance(lat, lon, lat, bounds[1]+bounds[3]) >= ground_width-20):
        ground_width += 10

    path_obj = mp_slipmap.SlipPolygon('FlightPath', path, layer='FlightPath',
                                      linewidth=2, colour=(255,0,180))
    mission = wp.polygon()
    if len(mission) > 1:
        mission_obj = mp_slipmap.SlipPolygon('Mission', wp.polygon(), layer='Mission',
                                             linewidth=2, colour=(255,255,255))
    else:
        mission_obj = None

    if opts.imagefile:
        create_imagefile(opts.imagefile, (lat,lon), ground_width, path_obj, mission_obj)
    else:
        map = mp_slipmap.MPSlipMap(title=filename,
                                   service=opts.service,
                                   elevation=True,
                                   width=600,
                                   height=600,
                                   ground_width=ground_width,
                                   lat=lat, lon=lon,
                                   debug=opts.debug)
        map.add_object(path_obj)
        if mission_obj is not None:
            map.add_object(mission_obj)

        for flag in opts.flag:
            a = flag.split(',')
            lat = a[0]
            lon = a[1]
            icon = 'flag.png'
            if len(a) > 2:
                icon = a[2] + '.png'
            icon = map.icon(icon)
            map.add_object(mp_slipmap.SlipIcon('icon - %s' % str(flag), (float(lat),float(lon)), icon, layer=3, rotation=0, follow=False))

if len(args) < 1:
    print("Usage: mavflightview.py [options] <LOGFILE...>")
    sys.exit(1)

if __name__ == "__main__":
    # add dialogs
    mavflightview(args[0])
