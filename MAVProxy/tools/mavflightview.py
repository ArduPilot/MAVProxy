#!/usr/bin/env python

'''
view a mission log on a map
'''

import sys, time, os
import re

from math import *

from pymavlink import mavutil, mavwp, mavextra
from MAVProxy.modules.mavproxy_map import mp_slipmap, mp_tile
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import grapher
from MAVProxy.modules.lib import kmlread
import functools
import random
import cv2

def create_map(title):
    '''create map object'''

def pixel_coords(latlon, ground_width=0, mt=None, topleft=None, width=None):
    '''return pixel coordinates in the map image for a (lat,lon)'''
    (lat,lon) = (latlon[0], latlon[1])
    return mt.coord_to_pixel(topleft[0], topleft[1], width, ground_width, lat, lon)

def create_imagefile(options, filename, latlon, ground_width, path_objs, mission_obj, fence_obj, kml_objects, width=600, height=600, used_flightmodes=[], mav_type=None):
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

    if kml_objects is not None:
        for obj in kml_objects:
            print(obj)
            try:
                obj.draw(map_img, pixmapper, None)
            except Exception:
                pass

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
colour_map_blimp = {}

for mytuple in ((mavutil.mode_mapping_apm.values(),colour_map_plane),
                (mavutil.mode_mapping_acm.values(),colour_map_copter),
                (mavutil.mode_mapping_rover.values(),colour_map_rover),
                (mavutil.mode_mapping_tracker.values(),colour_map_tracker),
                (mavutil.mode_mapping_sub.values(),colour_map_submarine),
                (mavutil.mode_mapping_blimp.values(),colour_map_blimp),
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
    if mav_type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                    mavutil.mavlink.MAV_TYPE_SURFACE_BOAT]:
        map = colour_map_rover
    if mav_type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
        map = colour_map_tracker
    if mav_type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
        map = colour_map_submarine
    if mav_type == mavutil.mavlink.MAV_TYPE_AIRSHIP:
        map = colour_map_blimp
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
    elif source == "type":
        return colour_for_point_type(mlog, point, instance, options)

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
        return v
    if isinstance(v, str):
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

def colour_for_point_type(mlog, point, instance, options):
    colors=[ 'red', 'green', 'blue', 'orange', 'olive', 'cyan', 'magenta', 'brown',
             'violet', 'purple', 'grey', 'black']
    return map_colours[instance]

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
    types = ['MISSION_ITEM', 'MISSION_ITEM_INT', 'CMD']
    if options.types is not None:
        types.extend(options.types.split(','))
    else:
        types.extend(['POS','GLOBAL_POSITION_INT'])
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

    # handle forms like GPS[0], mapping to GPS for recv_match_types
    for i in range(len(types)):
        bracket = types[i].find('[')
        if bracket != -1:
            types[i] = types[i][:bracket]

    recv_match_types = types[:]
    colour_source = getattr(options, "colour_source")
    re_caps = re.compile('[A-Z_][A-Z0-9_]+')

    if colour_source is not None:
        # stolen from mavgraph.py
        caps = set(re.findall(re_caps, colour_source))
        recv_match_types.extend(caps)

    print("Looking for types %s" % str(recv_match_types))

    last_timestamps = {}
    used_flightmodes = {}

    mlog.rewind()

    while True:
        try:
            m = mlog.recv_match(type=recv_match_types)
            if m is None:
                break
        except Exception:
            break

        type = m.get_type()

        if type in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
            try:
                new_m = m
                if type == 'MISSION_ITEM_INT':
                    # create a MISSION_ITEM from MISSION_ITEM_INT
                    new_m = mavutil.mavlink.MAVLink_mission_item_message(
                        0,
                        0,
                        m.seq,
                        m.frame,
                        m.command,
                        m.current,
                        m.autocontinue,
                        m.param1,
                        m.param2,
                        m.param3,
                        m.param4,
                        m.x / 1.0e7,
                        m.y / 1.0e7,
                        m.z
                    )
                while new_m.seq > wp.count():
                    print("Adding dummy WP %u" % wp.count())
                    wp.set(new_m, wp.count())
                wp.set(new_m, m.seq)
            except Exception as e:
                print("Exception: %s" % str(e))
                pass
            continue
        elif type == 'CMD':
            if options.mission is None:
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

        if not type in types:
            # may only be present for colour-source expressions to work
            continue

        if type == 'GPS' and hasattr(m,'I'):
            type = 'GPS[%u]' % m.I

        if not all_false and len(flightmode_selections) > 0 and idx < len(options._flightmodes) and m._timestamp >= options._flightmodes[idx][2]:
            idx += 1
        elif (idx < len(flightmode_selections) and flightmode_selections[idx]) or all_false or len(flightmode_selections) == 0:
            used_flightmodes[mlog.flightmode] = 1
            (lat, lng) = (None,None)
            if type in ['GPS','GPS2']:
                status = getattr(m, 'Status', None)
                nsats = getattr(m, 'NSats', None)
                if status is None:
                    status = getattr(m, 'FixType', None)
                    if status is None:
                        print("Can't find status on GPS message")
                        print(m)
                        break
                if status < 2 and nsats < 5:
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
            elif type in ['NKF1','XKF1']:
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
            elif type == 'SIM':
                (lat, lng) = (m.Lat, m.Lng)
            elif type == 'GUID':
                if (m.Type == 0):
                    (lat, lng) = (m.pX*1.0e-7, m.pY*1.0e-7)
            else:
                if hasattr(m,'Lat'):
                    lat = m.Lat
                if hasattr(m,'Lon'):
                    lng = m.Lon
                if hasattr(m,'Lng'):
                    lng = m.Lng
                if hasattr(m,'lat'):
                    lat = m.lat * 1.0e-7
                if hasattr(m,'lon'):
                    lng = m.lon * 1.0e-7
                if hasattr(m,'latitude'):
                    lat = m.latitude * 1.0e-7
                if hasattr(m,'longitude'):
                    lng = m.longitude * 1.0e-7

            if lat is None or lng is None:
                continue

            # automatically add new types to instances
            if type not in instances:
                instances[type] = len(instances)
                while len(instances) >= len(path):
                    path.append([])
            instance = instances[type]

            # only plot thing we have a valid-looking location for:
            if abs(lat)<=0.01 and abs(lng)<=0.01:
                continue

            colour = colour_for_point(mlog, (lat, lng), instance, options)
            if colour is None:
                continue

            tdays = grapher.timestamp_to_days(m._timestamp)
            point = (lat, lng, colour, tdays)

            if options.rate == 0 or not type in last_timestamps or m._timestamp - last_timestamps[type] > 1.0/options.rate:
                last_timestamps[type] = m._timestamp
                path[instance].append(point)
    if len(path[0]) == 0:
        print("No points to plot")
        return None

    return [path, wp, fen, used_flightmodes, getattr(mlog, 'mav_type',None), instances]

def mavflightview_show(path, wp, fen, used_flightmodes, mav_type, options, instances, title=None, timelim_pipe=None, show_waypoints=True):
    if not title:
        title='MAVFlightView'


    boundary_path = []
    for p in path[0]:
        boundary_path.append((p[0],p[1]))

    fence = fen.polygon()
    if options.fencebounds:
        for p in fence:
            boundary_path.append((p[0],p[1]))

    bounds = mp_util.polygon_bounds(boundary_path)
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
    plist = []
    if options.show_waypoints:
        plist = wp.polygon_list()
    mission_obj = None
    if len(plist) > 0:
        mission_obj = []
        for i in range(len(plist)):
            mission_obj.append(mp_slipmap.SlipPolygon('Mission-%s-%u' % (title,i), plist[i], layer='Mission',
                                                      linewidth=2, colour=(255,255,255)))
    else:
        mission_obj = None

    if len(fence) > 1:
        fence_obj = mp_slipmap.SlipPolygon('Fence-%s' % title, fen.polygon(), layer='Fence',
                                           linewidth=2, colour=(0,255,0))
    else:
        fence_obj = None

    kml = getattr(options,'kml',None)
    if kml is not None:
        kml_objects = load_kml(kml)
    else:
        kml_objects = None

    if options.imagefile:
        create_imagefile(options, options.imagefile, (lat,lon), ground_width, path_objs, mission_obj, fence_obj, kml_objects, used_flightmodes=used_flightmodes, mav_type=mav_type)
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
                                       show_flightmode_legend=options.show_flightmode_legend,
                                       timelim_pipe=timelim_pipe)
        if options.multi:
            multi_map = map
        for path_obj in path_objs:
            map.add_object(path_obj)
        if mission_obj is not None:
            display_waypoints(wp, map)
        if fence_obj is not None:
            map.add_object(fence_obj)

        if kml_objects is not None:
            for obj in kml_objects:
                map.add_object(obj)
            
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
        elif options.colour_source == "type":
            tuples = [ (t, map_colours[instances[t]]) for t in instances.keys() ]
            map.add_object(mp_slipmap.SlipFlightModeLegend("legend", tuples))
        else:
            print("colour-source: min=%f max=%f" % (colour_source_min, colour_source_max))


def load_kml(kml):
    '''load a kml overlay, return list of map objects'''
    print("Loading kml %s" % kml)
    nodes = kmlread.readkmz(kml)
    ret = []
    for n in nodes:
        try:
            point = kmlread.readObject(n)
        except Exception as ex:
            continue

        if point[0] == 'Polygon':
            newcolour = (random.randint(0, 255), 0, random.randint(0, 255))
            curpoly = mp_slipmap.SlipPolygon(point[1], point[2],
                                             layer=2, linewidth=2, colour=newcolour)
            ret.append(curpoly)

        if point[0] == 'Point':
            icon = mp_tile.mp_icon('barrell.png')
            curpoint = mp_slipmap.SlipIcon(point[1], latlon = (point[2][0][0], point[2][0][1]), layer=3, img=icon, rotation=0, follow=False)
            curtext = mp_slipmap.SlipLabel(point[1], point = (point[2][0][0], point[2][0][1]), layer=4, label=point[1], colour=(0,255,255))
            ret.append(curpoint)
            ret.append(curtext)
    return ret


def mavflightview(filename, options):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    stuff = mavflightview_mav(mlog, options)
    if stuff is None:
        return
    [path, wp, fen, used_flightmodes, mav_type, instances] = stuff
    mavflightview_show(path, wp, fen, used_flightmodes, mav_type, options, instances, title=filename)

class mavflightview_options(object):
    def __init__(self):
        self.service = "MicrosoftHyb"
        self.mode = None
        self.condition = None
        self.mission = None
        self.fence = None
        self.fencebounds = False
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
        self.show_waypoints = True

if __name__ == "__main__":
    multiproc.freeze_support()

    from optparse import OptionParser
    parser = OptionParser("mavflightview.py [options]")
    parser.add_option("--service", default="MicrosoftSat", help="tile service")
    parser.add_option("--mode", default=None, help="flight mode")
    parser.add_option("--condition", default=None, help="conditional check on log")
    parser.add_option("--mission", default=None, help="mission file (defaults to logged mission)")
    parser.add_option("--fence", default=None, help="fence file")
    parser.add_option("--fencebounds", action='store_true', help="use fence boundary for zoom")
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
    parser.add_option("--kml", default=None, help="add kml overlay")
    parser.add_option("--hide-waypoints", dest='show_waypoints', action='store_false', help="do not show waypoints", default=True)

    (opts, args) = parser.parse_args()

    try:
        import faulthandler, signal
        try:
            faulthandler.register(signal.SIGUSR1)
        except AttributeError as e:
            pass
    except ImportError:
        pass

    if len(args) < 1:
        print("Usage: mavflightview.py [options] <LOGFILE...>")
        sys.exit(1)

    if opts.multi:
        multi_map = None

    random.seed(1)

    for f in args:
        mavflightview(f, opts)
