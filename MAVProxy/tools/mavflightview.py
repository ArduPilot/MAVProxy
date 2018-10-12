#!/usr/bin/env python

'''
view a mission log on a map
'''

import sys, time, os
from math import *

from pymavlink import mavutil, mavwp, mavextra
from MAVProxy.modules.mavproxy_map import mp_slipmap, mp_tile
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
import functools

import cv2

def create_map(title):
    '''create map object'''

def pixel_coords(latlon, ground_width=0, mt=None, topleft=None, width=None):
    '''return pixel coordinates in the map image for a (lat,lon)'''
    (lat,lon) = (latlon[0], latlon[1])
    return mt.coord_to_pixel(topleft[0], topleft[1], width, ground_width, lat, lon)

def create_imagefile(options, filename, latlon, ground_width, path_objs, mission_obj, fence_obj, width=600, height=600, used_flightmodes=[], mav_type=None):
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
    if (options is not None and
        mav_type is not None and
        options.colour_source == "flightmode"):
        tuples = [ (mode, colour_for_flightmode(mav_type, mode))
                   for mode in used_flightmodes.keys() ]
        legend = mp_slipmap.SlipFlightModeLegend("legend", tuples)
        legend.draw(map_img, pixmapper, None)

    map_img = cv2.cvtColor(map_img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(filename, map_img)

map_colours = [ (255,   0,   0),
                (  0, 255,   0),
                (  0,   0, 255),
                (127,   0,   0),
                (  0, 127,   0),
#                (  0,   0, 127),
                (255, 255,   0),
                (255,   0, 255),
                (  0, 255, 255),
                (127, 127,   0),
                (127,   0, 127),
                (  0, 127, 127),

                (255,  191,   0),
                (255,   0,  191),
                (255,  191,  191),

                (191, 255,   0),
                (191,   0, 255),
                (191, 255, 255),

                ( 0, 255,  191),
                ( 0,  191, 255),
                ( 0, 255, 255),
]
colour_map_copter = {}
colour_map_plane = {}
colour_map_rover = {}
colour_map_tracker = {}
colour_map_submarine = {}

for mytuple in ((mavutil.mode_mapping_apm.values(),colour_map_plane),
                (mavutil.mode_mapping_acm.values(),colour_map_copter),
                (mavutil.mode_mapping_rover.values(),colour_map_rover),
                (mavutil.mode_mapping_tracker.values(),colour_map_tracker),
                (mavutil.mode_mapping_sub.values(),colour_map_submarine),
):
    (mode_names, colour_map) = mytuple
    i=0
    for mode_name in mode_names:
        colour_map[mode_name] = map_colours[i]
        i += 1
        if i >= len(map_colours):
            print("Warning: reusing colours!")
            i = 0
    colour_map["UNKNOWN"] = (0, 0, 0)

colourmap_check_done = False
def colourmap_for_mav_type(mav_type):
    # swiped from "def mode_mapping_byname(mav_type):" in mavutil
    map = None
    if mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_TYPE_HELICOPTER,
                    mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                    mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                    mavutil.mavlink.MAV_TYPE_COAXIAL,
                    mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        map = colour_map_copter
    if mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        map = colour_map_plane
    if mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        map = colour_map_rover
    if mav_type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
        map = colour_map_tracker
    if mav_type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
        map = colour_map_submarine
    if map is None:
        print("No colormap for mav_type=%u" % (mav_type,))
        # we probably don't have a valid mode map, so returning
        # anything but the empty map here is probably pointless:
        map = colour_map_plane
    return map

def display_waypoints(wploader, map):
    '''display the waypoints'''
    mission_list = wploader.view_list()
    polygons = wploader.polygon_list()
    map.add_object(mp_slipmap.SlipClearLayer('Mission'))
    for k in range(len(polygons)):
        p = polygons[k]
        if len(p) > 1:
            map.add_object(mp_slipmap.SlipPolygon('mission %u' % k, p,
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

colour_expression_exceptions = dict()
colour_source_min = 255
colour_source_max = 0

def colour_for_point(mlog, point, instance, options):
    global colour_expression_exceptions, colour_source_max, colour_source_min
    '''indicate a colour to be used to plot point'''
    source = getattr(options, "colour_source", "flightmode")
    if source == "flightmode":
        return colour_for_point_flightmode(mlog, point, instance, options)

    # evaluate source as an expression which should return a
    # number in the range 0..255
    try:
        v = eval(source, globals(),mlog.messages)
    except Exception as e:
        str_e = str(e)
        try:
            count = colour_expression_exceptions[str_e]
        except KeyError:
            colour_expression_exceptions[str_e] = 0
            count = 0
        if count > 100:
            print("Too many exceptions processing (%s): %s" % (source, str_e))
            sys.exit(1)
        colour_expression_exceptions[str_e] += 1
        v = 0

    # we don't use evaluate_expression as we want the exceptions...
#    v = mavutil.evaluate_expression(source, mlog.messages)

    if v is None:
        v = 0
    elif isinstance(v, str):
        print("colour expression returned a string: %s" % v)
        sys.exit(1)
    elif v < 0:
        print("colour expression returned %d (< 0)" % v)
        v = 0
    elif v > 255:
        print("colour expression returned %d (> 255)" % v)
        v = 255

    if v < colour_source_min:
        colour_source_min = v
    if v > colour_source_max:
        colour_source_max = v

    r = 255
    g = 255
    b = v
    return (b,b,b)

def colour_for_point_flightmode(mlog, point, instance, options):
    return colour_for_flightmode(getattr(mlog, 'mav_type',None), getattr(mlog, 'flightmode',''), instance)

def colour_for_flightmode(mav_type, fmode, instance=0):
    colourmap = colourmap_for_mav_type(mav_type)
    if fmode in colourmap:
        colour = colourmap[fmode]
    else:
        print("No entry in colourmap for %s" % (str(fmode)))
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
    return colour

def mavflightview_mav(mlog, options=None, flightmode_selections=[]):
    '''create a map for a log file'''
    wp = mavwp.MAVWPLoader()
    if options.mission is not None:
        wp.load(options.mission)
    fen = mavwp.MAVFenceLoader()
    if options.fence is not None:
        fen.load(options.fence)
    all_false = True
    for s in flightmode_selections:
        if s:
            all_false = False
    idx = 0
    path = [[]]
    instances = {}
    ekf_counter = 0
    nkf_counter = 0
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
        if options.nkf:
            types.extend(['NKF1', 'GPS'])
        if options.ahr2:
            types.extend(['AHR2', 'AHRS2', 'GPS'])
    print("Looking for types %s" % str(types))

    last_timestamps = {}
    used_flightmodes = {}

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

        if not all_false and len(flightmode_selections) > 0 and idx < len(options._flightmodes) and m._timestamp >= options._flightmodes[idx][2]:
            idx += 1
        elif (idx < len(flightmode_selections) and flightmode_selections[idx]) or all_false or len(flightmode_selections) == 0:
            used_flightmodes[mlog.flightmode] = 1
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
            elif type in ['NKF1']:
                pos = mavextra.ekf1_pos(m)
                if pos is None:
                    continue
                nkf_counter += 1
                if nkf_counter % options.nkf_sample != 0:
                    continue
                (lat, lng) = pos
            elif type in ['ANU5']:
                (lat, lng) = (m.Alat*1.0e-7, m.Alng*1.0e-7)
            elif type in ['AHR2', 'POS', 'CHEK']:
                (lat, lng) = (m.Lat, m.Lng)
            elif type == 'AHRS2':
                (lat, lng) = (m.lat*1.0e-7, m.lng*1.0e-7)
            elif type == 'ORGN':
                (lat, lng) = (m.Lat, m.Lng)
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
                colour = colour_for_point(mlog, (lat, lng), instance, options)
                point = (lat, lng, colour)

                if options.rate == 0 or not type in last_timestamps or m._timestamp - last_timestamps[type] > 1.0/options.rate:
                    last_timestamps[type] = m._timestamp
                    path[instance].append(point)
    if len(path[0]) == 0:
        print("No points to plot")
        return None

    return [path, wp, fen, used_flightmodes, getattr(mlog, 'mav_type',None)]

def mavflightview_show(path, wp, fen, used_flightmodes, mav_type, options, title=None):
    if not title:
        title='MAVFlightView'

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
        create_imagefile(options, options.imagefile, (lat,lon), ground_width, path_objs, mission_obj, fence_obj, used_flightmodes=used_flightmodes, mav_type=mav_type)
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
                                       debug=options.debug,
                                       show_flightmode_legend=options.show_flightmode_legend)
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

        if options.colour_source == "flightmode":
            tuples = [ (mode, colour_for_flightmode(mav_type, mode))
                       for mode in used_flightmodes.keys() ]
            map.add_object(mp_slipmap.SlipFlightModeLegend("legend", tuples))
        else:
            print("colour-source: min=%f max=%f" % (colour_source_min, colour_source_max))

def mavflightview(filename, options):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    stuff = mavflightview_mav(mlog, options)
    if stuff is None:
        return
    [path, wp, fen, used_flightmodes, mav_type] = stuff
    mavflightview_show(path, wp, fen, used_flightmodes, mav_type, options, title=filename)

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
        self.nkf = False
        self.ahr2 = False
        self.debug = False
        self.multi = False
        self.types = None
        self.ekf_sample = 1
        self.rate = 0
        self._flightmodes = []
        self.colour_source = 'flightmode'

if __name__ == "__main__":
    multiproc.freeze_support()

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
    parser.add_option("--nkf", action='store_true', default=False, help="use NKF1 pos")
    parser.add_option("--ahr2", action='store_true', default=False, help="use AHR2 pos")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    parser.add_option("--multi", action='store_true', default=False, help="show multiple flights on one map")
    parser.add_option("--types", default=None, help="types of position messages to show")
    parser.add_option("--ekf-sample", type='int', default=1, help="sub-sampling of EKF messages")
    parser.add_option("--nkf-sample", type='int', default=1, help="sub-sampling of NKF messages")
    parser.add_option("--rate", type='int', default=0, help="maximum message rate to display (0 means all points)")
    parser.add_option("--colour-source", type="str", default="flightmode", help="expression with range 0f..255f used for point colour")
    parser.add_option("--no-flightmode-legend", action="store_false", default=True, dest="show_flightmode_legend", help="hide legend for colour used for flight modes")

    (opts, args) = parser.parse_args()

    if len(args) < 1:
        print("Usage: mavflightview.py [options] <LOGFILE...>")
        sys.exit(1)

    if opts.multi:
        multi_map = None

    for f in args:
        mavflightview(f, opts)
