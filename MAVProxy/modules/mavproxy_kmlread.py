#!/usr/bin/env python
'''Module to add or clear kml layers on the map

Copyright Stephen Dade 2016
Released under the GNU GPL version 3 or later

AP_FLAKE8_CLEAN
'''

import copy
import random
import re

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
# from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import kmlread

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallFileDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuCheckbox
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSeparator
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu


class KmlReadModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(KmlReadModule, self).__init__(mpstate, "kmlread", "Add kml or kmz layers to map", public=True)
        self.add_command('kml', self.cmd_param, "kml map handling",
                         ["<clear|snapwp|snapfence>",
                          "<load> (FILENAME)", '<layers>'])

        # allayers is all the loaded layers (slimap objects)
        # curlayers is only the layers displayed (checked) on the map (text list of layer names)
        self.allayers = []
        self.curlayers = []
        self.alltextlayers = []
        self.curtextlayers = []
        self.initialised_map_module = False
        self.menu_needs_refreshing = True
        self.map_objects = {}
        self.counter = 0

        # the fence manager
        self.snap_points = []

        # make the initial map menu
        if mp_util.has_wxpython:
            self.menu_fence = MPMenuSubMenu('Set Geofence', items=[])
            self.menu = MPMenuSubMenu(
                'KML Layers',
                items=[
                    MPMenuItem('Clear', 'Clear', '# kml clear'),
                ]
            )

    def cmd_param(self, args):
        '''control kml reading'''
        usage = "Usage: kml <clear | load (filename) | layers | toggle (layername) | colour (layername) (colour) | fence (inc|exc) (layername)> | snapfence | snapwp"  # noqa
        if len(args) < 1:
            print(usage)
            return
        elif args[0] == "clear":
            self.clearkml()
        elif args[0] == "snapwp":
            self.cmd_snap_wp(args[1:])
        elif args[0] == "snapfence":
            self.cmd_snap_fence(args[1:])
        elif args[0] == "load":
            if len(args) != 2:
                print("usage: kml load <filename>")
                return
            self.loadkml(args[1])
        elif args[0] == "layers":
            for layer in self.curlayers:
                print("Found layer: " + layer)
        elif args[0] == "toggle":
            self.togglekml(args[1])
        elif args[0] == "colour" or args[0] == "color":
            self.cmd_colour(args[1:])
        elif args[0] == "fence":
            self.fencekml(args[1:])
        else:
            print(usage)
            return

    def cmd_snap_wp(self, args):
        '''snap waypoints to KML'''
        threshold = 10.0
        if len(args) > 0:
            threshold = float(args[0])
        wpmod = self.module('wp')
        wploader = wpmod.wploader
        changed = False
        for i in range(1, wploader.count()):
            w = wploader.wp(i)
            if not wploader.is_location_command(w.command):
                continue
            lat = w.x
            lon = w.y
            best = None
            best_dist = (threshold+1)*3
            for (snap_lat, snap_lon) in self.snap_points:
                dist = mp_util.gps_distance(lat, lon, snap_lat, snap_lon)
                if dist < best_dist:
                    best_dist = dist
                    best = (snap_lat, snap_lon)
            if best is not None and best_dist <= threshold:
                if w.x != best[0] or w.y != best[1]:
                    w.x = best[0]
                    w.y = best[1]
                    print("Snapping WP %u to %f %f" % (i, w.x, w.y))
                    wploader.set(w, i)
                    changed = True
            elif best is not None:
                if best_dist <= (threshold+1)*3:
                    print("Not snapping wp %u dist %.1f" % (i, best_dist))
        if changed:
            wpmod.send_all_waypoints()

    def cmd_snap_fence(self, args):
        '''snap fence to KML'''

        threshold = 10.0
        if len(args) > 0:
            threshold = float(args[0])

        fencemod = self.module('fence')
        if fencemod is None:
            print("fence module not loaded")
            return

        def fencepoint_snapper(offset, point_obj):
            best = None
            best_dist = (threshold+1)*3
            if isinstance(point_obj, mavutil.mavlink.MAVLink_mission_item_message):
                point = (point_obj.x, point_obj.y)
            elif isinstance(point_obj, mavutil.mavlink.MAVLink_mission_item_int_message):
                point = (point_obj.x * 1e-7, point_obj.y * 1e-7)
            else:
                raise TypeError(f"What's a {type(point_obj)}?")

            for snap_point in copy.copy(self.snap_points):
                if point == snap_point:
                    return

                dist = mp_util.gps_distance(point[0], point[1], snap_point[0], snap_point[1])
                if dist < best_dist:
                    best_dist = dist
                    best = snap_point

            if best is None:
                return

            if best_dist <= threshold:
                print("Snapping fence point %u to %f %f" % (offset, best[0], best[1]))
                if isinstance(point_obj, mavutil.mavlink.MAVLink_mission_item_message):
                    point_obj.x = best[0]
                    point_obj.y = best[1]
                elif isinstance(point_obj, mavutil.mavlink.MAVLink_mission_item_int_message):
                    point_obj.x = best[0] * 1e7
                    point_obj.y = best[1] * 1e7
                else:
                    raise TypeError(f"What's a {type(point_obj)}?")
                self.snapped_fencepoint = True
                return

            if best_dist <= (threshold+1)*3:
                print("Not snapping fence point %u dist %.1f" % (offset, best_dist))

        self.snapped_fencepoint = False
        fencemod.apply_function_to_points(fencepoint_snapper)
        if self.snapped_fencepoint:
            fencemod.push_to_vehicle()

    def cmd_colour(self, args):
        if len(args) < 2:
            print("kml colour LAYERNAME 0xBBGGRR")
            return
        (layername, colour) = args
        layer = self.find_layer(layername)
        if layer is None:
            print(f"No layer {layername}")
            return
        m = re.match(r"(?:0x)?(?P<red>[0-9A-Fa-f]{2})(?P<green>[0-9A-Fa-f]{2})(?P<blue>[0-9A-Fa-f]{2})", colour)
        if m is None:
            print("bad colour")
            return
        (red, green, blue) = (int(m.group("red"), 16),
                              int(m.group("green"), 16),
                              int(m.group("blue"), 16))
        self.remove_map_object(layer)
        layer.set_colour((red, green, blue))
        self.add_map_object(layer)

    def remove_map_object(self, obj):
        '''remove an object from our stored list of objects, and the map
        module if it is loaded'''
        if obj.layer not in self.map_objects:
            raise ValueError(f"{obj.layer=} not in added map_objects")
        if obj.key not in self.map_objects[obj.layer]:
            raise ValueError(f"{obj.key=} not in added map_objects[{obj.key}] (map_objects[{obj.layer}][{obj.key}]")
        del self.map_objects[obj.layer][obj.key]
        if len(self.map_objects[obj.layer]) == 0:
            del self.map_objects[obj.layer]
        map_module = self.mpstate.map
        if map_module is not None:
            map_module.remove_object(obj.key)

    def remove_all_map_objects(self):
        for layer in copy.deepcopy(self.map_objects):
            for key in copy.deepcopy(self.map_objects[layer]):
                self.remove_map_object(self.map_objects[layer][key])

    def add_map_object(self, obj):
        '''add an object to our stored list of objects, and the map
        module if it is loaded'''
        if obj.layer in self.map_objects and obj.key in self.map_objects[obj.layer]:
            raise ValueError(f"Already have self.map_objects[{obj.layer=}][{obj.key=}]")
        if obj.layer not in self.map_objects:
            self.map_objects[obj.layer] = {}
        self.map_objects[obj.layer][obj.key] = obj

        map_module = self.mpstate.map
        if map_module is not None:
            map_module.add_object(obj)

    def add_objects_to_map_module_from_map_objects(self):
        for layer in self.map_objects:
            for obj in self.map_objects[layer].values():
                map_module = self.mpstate.map
                map_module.add_object(obj)

    def fencekml(self, args):
        '''create a geofence from a layername'''
        usage = "kml fence inc|exc layername"
        if len(args) != 2:
            print(usage)
            return

        fencemod = self.module('fence')
        if fencemod is None:
            print("fence module not loaded")
            return

        (inc_or_exc, layername) = (args[0], args[1])
        if inc_or_exc in ["inc", "inclusion"]:
            fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        elif inc_or_exc in ["exc", "exclusion"]:
            fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        else:
            print(usage)
            return

        # find the layer, add it if found
        for layer in self.allayers:
            if layer.key != layername:
                continue

            points_copy = copy.copy(layer.points)
            if points_copy[-1] == points_copy[0]:
                points_copy.pop(-1)

            fencemod.add_polyfence(fence_type, points_copy)
            return

        print("Layer not found")

    def togglekml(self, layername):
        '''toggle the display of a kml'''
        # Strip quotation marks if neccessary
        if layername.startswith('"') and layername.endswith('"'):
            layername = layername[1:-1]
        # toggle layer off (plus associated text element)
        if layername in self.curlayers:
            for layer in self.curlayers:
                if layer == layername:
                    self.remove_map_object(self.find_layer(layer))
                    self.curlayers.remove(layername)
                    if layername in self.curtextlayers:
                        for clayer in self.curtextlayers:
                            if clayer == layername:
                                self.remove_map_object(self.find_layer(clayer))
                                self.curtextlayers.remove(clayer)
        # toggle layer on (plus associated text element)
        else:
            for layer in self.allayers:
                if layer.key == layername:
                    self.add_map_object(layer)
                    self.curlayers.append(layername)
                    for alayer in self.alltextlayers:
                        if alayer.key == layername:
                            self.add_map_object(alayer)
                            self.curtextlayers.append(layername)
        self.menu_needs_refreshing = True

    def find_layer(self, layername):
        for layer in self.allayers:
            if layer.key == layername:
                return layer
        return None

    def clearkml(self):
        '''Clear the kmls from the map'''
        # go through all the current layers and remove them
        self.remove_all_map_objects()
        self.allayers = []
        self.curlayers = []
        self.alltextlayers = []
        self.curtextlayers = []
        self.menu_needs_refreshing = True

    def add_polygon(self, name, coords):
        '''add a polygon to the KML list.  coords is a list of lat/lng tuples in degrees'''
        self.snap_points.extend(coords)

        # print("Adding " + name)
        newcolour = (random.randint(0, 255), 0, random.randint(0, 255))
        layer_name = f"{name}-{self.counter}"
        curpoly = mp_slipmap.SlipPolygon(
            layer_name,
            coords,
            layer=2,
            linewidth=2,
            colour=newcolour,
        )
        self.add_map_object(curpoly)
        self.allayers.append(curpoly)
        self.curlayers.append(layer_name)
        self.counter += 1

    def loadkml(self, filename):
        '''Load a kml from file and put it on the map'''
        # Open the zip file
        nodes = kmlread.readkmz(filename)

        self.snap_points = []

        # go through each object in the kml...
        if nodes is None:
            print("No nodes found")
            return
        for n in nodes:
            try:
                point = kmlread.readObject(n)
            except Exception:
                continue
            if point is None:
                continue

            # and place any polygons on the map
            (pointtype, name, coords) = point
            if pointtype == 'Polygon':
                self.add_polygon(name, coords)

            # and points - barrell image and text
            if pointtype == 'Point':
                # print("Adding " + point[1])
                curpoint = mp_slipmap.SlipIcon(
                    point[1],
                    latlon=(point[2][0][0], point[2][0][1]),
                    layer=3,
                    img='barrell.png',
                    rotation=0,
                    follow=False,
                )
                curtext = mp_slipmap.SlipLabel(
                    point[1],
                    point=(point[2][0][0], point[2][0][1]),
                    layer=4,
                    label=point[1],
                    colour=(0, 255, 255),
                )
                self.add_map_object(curpoint)
                self.add_map_object(curtext)
                self.allayers.append(curpoint)
                self.alltextlayers.append(curtext)
                self.curlayers.append(point[1])
                self.curtextlayers.append(point[1])
        self.menu_needs_refreshing = True

    def idle_task(self):
        '''handle GUI elements'''
        if self.module('map') is None:
            self.initialised_map_module = False
            return
        if not self.initialised_map_module:
            self.menu_needs_refreshing = True
            self.add_objects_to_map_module_from_map_objects()
            self.initialised_map_module = True

        if self.menu_needs_refreshing:
            self.refresh_menu()
            self.menu_needs_refreshing = False

    def refresh_menu(self):
        if True:
            if not mp_util.has_wxpython:
                return

            # (re)create the menu
            # we don't dynamically update these yet due to a wx bug
            self.menu.items = [
                MPMenuItem('Clear', 'Clear', '# kml clear'),
                MPMenuItem(
                    'Load', 'Load', '# kml load ',
                    handler=MPMenuCallFileDialog(
                        flags=('open',),
                        title='KML Load',
                        wildcard='*.kml;*.kmz',
                    )
                ),
                self.menu_fence,
                MPMenuSeparator(),
            ]
            self.menu_fence.items = []
            for layer in self.allayers:
                # if it's a polygon, add it to the "set Geofence" list
                if isinstance(layer, mp_slipmap.SlipPolygon):
                    self.menu_fence.items.append(MPMenuItem(
                        layer.key,
                        layer.key,
                        '# kml fence \"' + layer.key + '\"',
                    ))
                # then add all the layers to the menu, ensuring to
                # check the active layers text elements aren't
                # included on the menu
                if layer.key.endswith('-text'):
                    continue

                checked = layer.key in self.curlayers
                self.menu.items.append(MPMenuCheckbox(
                    layer.key,
                    layer.key,
                    f'# kml toggle \"{layer.key}\"',
                    checked=checked,
                ))
            # and add the menu to the map popu menu
            self.module('map').add_menu(self.menu)
        self.menu_needs_refreshing = False

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''


def init(mpstate):
    '''initialise module'''
    return KmlReadModule(mpstate)
