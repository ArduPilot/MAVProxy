#!/usr/bin/env python

'''
view a mission log on a map
'''

import sys, time, os

from pymavlink import mavutil, mavwp, mavextra
from MAVProxy.modules.mavproxy_map import mp_slipmap, mp_tile
from MAVProxy.modules.lib import mp_util
import functools

try:
    import cv2.cv as cv
except ImportError:
    import cv

def create_map(title):
    '''create map object'''

def pixel_coords(latlon, ground_width=0, mt=None, topleft=None, width=None):
    '''return pixel coordinates in the map image for a (lat,lon)'''
    (lat,lon) = (latlon[0], latlon[1])
    return mt.coord_to_pixel(topleft[0], topleft[1], width, ground_width, lat, lon)

def create_imagefile(options, filename, latlon, ground_width, path_objs, mission_obj, fence_obj, width=600, height=600):
    '''create path and mission as an image file'''
    mt = mp_tile.MPTile(service=options.service)

    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    while mt.tiles_pending() > 0:
        print("Waiting on %u tiles" % mt.tiles_pending())
        time.sleep(1)
    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    # a function to convert from (lat,lon) to (px,py) on the map
    pixmapper = functools.partial(pixel_coords, ground_width=ground_width, mt=mt, topleft=latlon, width=width)
    for path_obj in path_objs:
        path_obj.draw(map_img, pixmapper, None)
    if mission_obj is not None:
        for m in mission_obj:
            m.draw(map_img, pixmapper, None)
    if fence_obj is not None:
        fence_obj.draw(map_img, pixmapper, None)
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
    'CRUISE'    : (0,   255, 255),
    'UNKNOWN'   : (230, 70,  40)
    }


def display_waypoints(wploader, map):
    '''display the waypoints'''
    mission_list = wploader.view_list()
    polygons = wploader.polygon_list()
    map.add_object(mp_slipmap.SlipClearLayer('Mission'))
    for i in range(len(polygons)):
        p = polygons[i]
        if len(p) > 1:
            map.add_object(mp_slipmap.SlipPolygon('mission %u' % i, p,
                                                  layer='Mission', linewidth=2, colour=(255,255,255)))
        labeled_wps = {}
        for i in range(len(mission_list)):
            next_list = mission_list[i]
            for j in range(len(next_list)):
                #label already printed for this wp?
                if (next_list[j] not in labeled_wps):
                    map.add_object(mp_slipmap.SlipLabel(
                        'miss_cmd %u/%u' % (i,j), polygons[i][j], str(next_list[j]), 'Mission', colour=(0,255,255)))  
                    labeled_wps[next_list[j]] = (i,j)

def mavflightview_mav(mlog, options=None, title=None):
    '''create a map for a log file'''
    if not title:
        title='MAVFlightView'
    wp = mavwp.MAVWPLoader()
    if options.mission is not None:
        wp.load(options.mission)
    fen = mavwp.MAVFenceLoader()
    if options.fence is not None:
        fen.load(options.fence)
    path = [[]]
    instances = {}
    ekf_counter = 0
    types = ['MISSION_ITEM','CMD']
    if options.types is not None:
        types.extend(options.types.split(','))
    else:
        types.extend(['GPS','GLOBAL_POSITION_INT'])
        if options.rawgps or options.dualgps:
            types.extend(['GPS', 'GPS_RAW_INT'])
        if options.rawgps2 or options.dualgps:
            types.extend(['GPS2_RAW','GPS2'])
        if options.ekf:
            types.extend(['EKF1', 'GPS'])
        if options.ahr2:
            types.extend(['AHR2', 'AHRS2', 'GPS'])
    print("Looking for types %s" % str(types))

    last_timestamps = {}

    while True:
        try:
            m = mlog.recv_match(type=types)
            if m is None:
                break
        except Exception:
            break

        type = m.get_type()
        
        if type == 'MISSION_ITEM':
            try:
                while m.seq > wp.count():
                    print("Adding dummy WP %u" % wp.count())
                    wp.set(m, wp.count())
                wp.set(m, m.seq)
            except Exception:
                pass
            continue
        if type == 'CMD':
            m = mavutil.mavlink.MAVLink_mission_item_message(0,
                                                             0,
                                                             m.CNum,
                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                             m.CId,
                                                             0, 1,
                                                             m.Prm1, m.Prm2, m.Prm3, m.Prm4,
                                                             m.Lat, m.Lng, m.Alt)
            try:
                while m.seq > wp.count():
                    print("Adding dummy WP %u" % wp.count())
                    wp.set(m, wp.count())
                wp.set(m, m.seq)
            except Exception:
                pass
            continue
        if not mlog.check_condition(options.condition):
            continue
        if options.mode is not None and mlog.flightmode.lower() != options.mode.lower():
            continue
        if type in ['GPS','GPS2']:
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
        elif type in ['EKF1', 'ANU1']:
            pos = mavextra.ekf1_pos(m)
            if pos is None:
                continue
            ekf_counter += 1
            if ekf_counter % options.ekf_sample != 0:
                continue
            (lat, lng) = pos            
        elif type in ['ANU5']:
            (lat, lng) = (m.Alat*1.0e-7, m.Alng*1.0e-7)
        elif type in ['AHR2', 'POS', 'CHEK']:
            (lat, lng) = (m.Lat, m.Lng)
        elif type == 'AHRS2':
            (lat, lng) = (m.lat*1.0e-7, m.lng*1.0e-7)
        else:
            lat = m.lat * 1.0e-7
            lng = m.lon * 1.0e-7

        # automatically add new types to instances
        if type not in instances:
            instances[type] = len(instances)
            while len(instances) >= len(path):
                path.append([])
        instance = instances[type]

        if abs(lat)>0.01 or abs(lng)>0.01:
            fmode = getattr(mlog, 'flightmode','')
            if fmode in colourmap:
                colour = colourmap[fmode]
            else:
                colour = colourmap['UNKNOWN']
            (r,g,b) = colour
            (r,g,b) = (r+instance*80,g+instance*50,b+instance*70)
            if r > 255:
                r = 205
            if g > 255:
                g = 205
            if g < 0:
                g = 0
            if b > 255:
                b = 205
            colour = (r,g,b)
            point = (lat, lng, colour)

            if options.rate == 0 or not type in last_timestamps or m._timestamp - last_timestamps[type] > 1.0/options.rate:
                last_timestamps[type] = m._timestamp
                path[instance].append(point)
    if len(path[0]) == 0:
        print("No points to plot")
        return
    bounds = mp_util.polygon_bounds(path[0])
    (lat, lon) = (bounds[0]+bounds[2], bounds[1])
    (lat, lon) = mp_util.gps_newpos(lat, lon, -45, 50)
    ground_width = mp_util.gps_distance(lat, lon, lat-bounds[2], lon+bounds[3])
    while (mp_util.gps_distance(lat, lon, bounds[0], bounds[1]) >= ground_width-20 or
           mp_util.gps_distance(lat, lon, lat, bounds[1]+bounds[3]) >= ground_width-20):
        ground_width += 10

    path_objs = []
    for i in range(len(path)):
        if len(path[i]) != 0:
            path_objs.append(mp_slipmap.SlipPolygon('FlightPath[%u]-%s' % (i,title), path[i], layer='FlightPath',
                                                    linewidth=2, colour=(255,0,180)))
    plist = wp.polygon_list()
    mission_obj = None
    if len(plist) > 0:
        mission_obj = []
        for i in range(len(plist)):
            mission_obj.append(mp_slipmap.SlipPolygon('Mission-%s-%u' % (title,i), plist[i], layer='Mission',
                                                      linewidth=2, colour=(255,255,255)))
    else:
        mission_obj = None

    fence = fen.polygon()
    if len(fence) > 1:
        fence_obj = mp_slipmap.SlipPolygon('Fence-%s' % title, fen.polygon(), layer='Fence',
                                           linewidth=2, colour=(0,255,0))
    else:
        fence_obj = None

    if options.imagefile:
        create_imagefile(options, options.imagefile, (lat,lon), ground_width, path_objs, mission_obj, fence_obj)
    else:
        global multi_map
        if options.multi and multi_map is not None:
            map = multi_map
        else:
            map = mp_slipmap.MPSlipMap(title=title,
                                       service=options.service,
                                       elevation=True,
                                       width=600,
                                       height=600,
                                       ground_width=ground_width,
                                       lat=lat, lon=lon,
                                       debug=options.debug)
        if options.multi:
            multi_map = map
        for path_obj in path_objs:
            map.add_object(path_obj)
        if mission_obj is not None:
            display_waypoints(wp, map)
        if fence_obj is not None:
            map.add_object(fence_obj)

        for flag in options.flag:
            a = flag.split(',')
            lat = a[0]
            lon = a[1]
            icon = 'flag.png'
            if len(a) > 2:
                icon = a[2] + '.png'
            icon = map.icon(icon)
            map.add_object(mp_slipmap.SlipIcon('icon - %s' % str(flag), (float(lat),float(lon)), icon, layer=3, rotation=0, follow=False))

def mavflightview(filename, options):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    mavflightview_mav(mlog, options, title=filename)

class mavflightview_options(object):
    def __init__(self):
        self.service = "MicrosoftHyb"
        self.mode = None
        self.condition = None
        self.mission = None
        self.fence = None
        self.imagefile = None
        self.flag = []
        self.rawgps = False
        self.rawgps2 = False
        self.dualgps = False
        self.ekf = False
        self.ahr2 = False
        self.debug = False
        self.multi = False
        self.types = None
        self.ekf_sample = 1
        self.rate = 0

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser("mavflightview.py [options]")
    parser.add_option("--service", default="MicrosoftSat", help="tile service")
    parser.add_option("--mode", default=None, help="flight mode")
    parser.add_option("--condition", default=None, help="conditional check on log")
    parser.add_option("--mission", default=None, help="mission file (defaults to logged mission)")
    parser.add_option("--fence", default=None, help="fence file")
    parser.add_option("--imagefile", default=None, help="output to image file")
    parser.add_option("--flag", default=[], type='str', action='append', help="flag positions")
    parser.add_option("--rawgps", action='store_true', default=False, help="use GPS_RAW_INT")
    parser.add_option("--rawgps2", action='store_true', default=False, help="use GPS2_RAW")
    parser.add_option("--dualgps", action='store_true', default=False, help="use GPS_RAW_INT and GPS2_RAW")
    parser.add_option("--ekf", action='store_true', default=False, help="use EKF1 pos")
    parser.add_option("--ahr2", action='store_true', default=False, help="use AHR2 pos")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    parser.add_option("--multi", action='store_true', default=False, help="show multiple flights on one map")
    parser.add_option("--types", default=None, help="types of position messages to show")
    parser.add_option("--ekf-sample", type='int', default=1, help="sub-sampling of EKF messages")
    parser.add_option("--rate", type='int', default=0, help="maximum message rate to display (0 means all points)")
    
    (opts, args) = parser.parse_args()

    if len(args) < 1:
        print("Usage: mavflightview.py [options] <LOGFILE...>")
        sys.exit(1)

    if opts.multi:
        multi_map = None

    for f in args:
        mavflightview(f, opts)
