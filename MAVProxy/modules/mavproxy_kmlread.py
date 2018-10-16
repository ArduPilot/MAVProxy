#!/usr/bin/env python
'''Module to add or clear kml layers on the map

Copyright Stephen Dade 2016
Released under the GNU GPL version 3 or later

'''

import time, math, random
from pymavlink import mavutil, mavwp
from xml.dom.minidom import parseString
from zipfile import ZipFile

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class KmlReadModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(KmlReadModule, self).__init__(mpstate, "kmlread", "Add kml or kmz layers to map")
        self.add_command('kml', self.cmd_param, "kml map handling",
                         ["<clear|snapwp|snapfence>",
                          "<load> (FILENAME)", '<layers>'])
                          
        #allayers is all the loaded layers (slimap objects)
        #curlayers is only the layers displayed (checked) on the map (text list of layer names)
        self.allayers = []
        self.curlayers = []
        self.alltextlayers = []
        self.curtextlayers = []
        self.menu_added_map = False
        self.menu_needs_refreshing = True
        
        #the fence manager
        self.fenceloader = mavwp.MAVFenceLoader()
        self.snap_points = []
        
        #make the initial map menu
        if mp_util.has_wxpython:
            self.menu_fence = MPMenuSubMenu('Set Geofence', items=[])
            self.menu = MPMenuSubMenu('KML Layers',
                                  items=[MPMenuItem('Clear', 'Clear', '# kml clear')])
                          
    def cmd_param(self, args):
        '''control kml reading'''
        usage = "Usage: kml <clear | load (filename) | layers | toggle (layername) | fence (layername)>"
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
        elif args[0] == "fence":
            self.fencekml(args[1])
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
        for i in range(1,wploader.count()):
            w = wploader.wp(i)
            if not wploader.is_location_command(w.command):
                continue
            lat = w.x
            lon = w.y
            best = None
            best_dist = (threshold+1)*3
            for (snap_lat,snap_lon) in self.snap_points:
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
        loader = fencemod.fenceloader
        changed = False
        for i in range(0,loader.count()):
            fp = loader.point(i)
            lat = fp.lat
            lon = fp.lng
            best = None
            best_dist = (threshold+1)*3
            for (snap_lat,snap_lon) in self.snap_points:
                dist = mp_util.gps_distance(lat, lon, snap_lat, snap_lon)
                if dist < best_dist:
                    best_dist = dist
                    best = (snap_lat, snap_lon)
            if best is not None and best_dist <= threshold:
                if best[0] != lat or best[1] != lon:
                    loader.move(i, best[0], best[1])
                    print("Snapping fence point %u to %f %f" % (i, best[0], best[1]))
                    changed = True
            elif best is not None:
                if best_dist <= (threshold+1)*3:
                    print("Not snapping fence point %u dist %.1f" % (i, best_dist))

        if changed:
            fencemod.send_fence()

    def fencekml(self, layername):
        '''set a layer as the geofence'''
        #Strip quotation marks if neccessary
        if layername.startswith('"') and layername.endswith('"'):
            layername = layername[1:-1]
        
        #for each point in the layer, add it in
        for layer in self.allayers:
            if layer.key == layername:
                #clear the current fence
                self.fenceloader.clear()
                if len(layer.points) < 3:
                    return
                self.fenceloader.target_system = self.target_system
                self.fenceloader.target_component = self.target_component
                #send centrepoint  to fence[0] as the return point
                bounds = mp_util.polygon_bounds(layer.points)
                (lat, lon, width, height) = bounds
                center = (lat+width/2, lon+height/2)
                self.fenceloader.add_latlon(center[0], center[1])
                for lat, lon in layer.points:
                    #add point
                    self.fenceloader.add_latlon(lat, lon)
                #and send
                self.send_fence()

    def send_fence(self):
        '''send fence points from fenceloader. Taken from fence module'''
        # must disable geo-fencing when loading
        self.fenceloader.target_system = self.target_system
        self.fenceloader.target_component = self.target_component
        self.fenceloader.reindex()
        action = self.get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
        self.param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE, 3)
        self.param_set('FENCE_TOTAL', self.fenceloader.count(), 3)
        for i in range(self.fenceloader.count()):
            p = self.fenceloader.point(i)
            self.master.mav.send(p)
            p2 = self.fetch_fence_point(i)
            if p2 is None:
                self.param_set('FENCE_ACTION', action, 3)
                return False
            if (p.idx != p2.idx or
                abs(p.lat - p2.lat) >= 0.00003 or
                abs(p.lng - p2.lng) >= 0.00003):
                print("Failed to send fence point %u" % i)
                self.param_set('FENCE_ACTION', action, 3)
                return False
        self.param_set('FENCE_ACTION', action, 3)
        return True

    def fetch_fence_point(self ,i):
        '''fetch one fence point. Taken from fence module'''
        self.master.mav.fence_fetch_point_send(self.target_system,
                                                    self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 3:
            p = self.master.recv_match(type='FENCE_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch point %u" % i)
            return None
        return p
                                    
    def togglekml(self, layername):
        '''toggle the display of a kml'''
        #Strip quotation marks if neccessary
        if layername.startswith('"') and layername.endswith('"'):
            layername = layername[1:-1]
        #toggle layer off (plus associated text element)
        if layername in self.curlayers:
            for layer in self.curlayers:
                if layer == layername:
                    self.mpstate.map.remove_object(layer)
                    self.curlayers.remove(layername)
                    if layername in self.curtextlayers:
                        for clayer in self.curtextlayers:
                            if clayer == layername:
                                self.mpstate.map.remove_object(clayer)
                                self.curtextlayers.remove(clayer)
        #toggle layer on (plus associated text element)
        else:
            for layer in self.allayers:
                if layer.key == layername:
                    self.mpstate.map.add_object(layer)
                    self.curlayers.append(layername)
                    for alayer in self.alltextlayers:
                        if alayer.key == layername:
                            self.mpstate.map.add_object(alayer)
                            self.curtextlayers.append(layername)
        self.menu_needs_refreshing = True
        
    def clearkml(self):
        '''Clear the kmls from the map'''
        #go through all the current layers and remove them
        for layer in self.curlayers:
            self.mpstate.map.remove_object(layer)
        for layer in self.curtextlayers:
            self.mpstate.map.remove_object(layer)
        self.allayers = []
        self.curlayers = []
        self.alltextlayers = []
        self.curtextlayers = []
        self.menu_needs_refreshing = True
                
    def loadkml(self, filename):
        '''Load a kml from file and put it on the map'''
        #Open the zip file
        nodes = self.readkmz(filename)

        self.snap_points = []

        #go through each object in the kml...
        for n in nodes:     
            point = self.readObject(n)

            #and place any polygons on the map
            if self.mpstate.map is not None and point[0] == 'Polygon':
                self.snap_points.extend(point[2])

                #print("Adding " + point[1])
                newcolour = (random.randint(0, 255), 0, random.randint(0, 255))
                curpoly = mp_slipmap.SlipPolygon(point[1], point[2],
                                                             layer=2, linewidth=2, colour=newcolour)
                self.mpstate.map.add_object(curpoly)
                self.allayers.append(curpoly)
                self.curlayers.append(point[1])
                
                
            #and points - barrell image and text
            if self.mpstate.map is not None and point[0] == 'Point':
                #print("Adding " + point[1])
                icon = self.mpstate.map.icon('barrell.png')
                curpoint = mp_slipmap.SlipIcon(point[1], latlon = (point[2][0][0], point[2][0][1]), layer=3, img=icon, rotation=0, follow=False)
                curtext = mp_slipmap.SlipLabel(point[1], point = (point[2][0][0], point[2][0][1]), layer=4, label=point[1], colour=(0,255,255))
                self.mpstate.map.add_object(curpoint)
                self.mpstate.map.add_object(curtext)
                self.allayers.append(curpoint)
                self.alltextlayers.append(curtext)
                self.curlayers.append(point[1])
                self.curtextlayers.append(point[1])
        self.menu_needs_refreshing = True
        

    def idle_task(self):
        '''handle GUI elements'''
        if not self.menu_needs_refreshing:
            return
        if self.module('map') is not None and not self.menu_added_map:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)
        #(re)create the menu
        if mp_util.has_wxpython and self.menu_added_map:
            # we don't dynamically update these yet due to a wx bug
            self.menu.items = [ MPMenuItem('Clear', 'Clear', '# kml clear'), MPMenuItem('Load', 'Load', '# kml load ', handler=MPMenuCallFileDialog(flags=('open',), title='KML Load', wildcard='*.kml;*.kmz')), self.menu_fence, MPMenuSeparator() ]
            self.menu_fence.items = []
            for layer in self.allayers:
                #if it's a polygon, add it to the "set Geofence" list
                if isinstance(layer, mp_slipmap.SlipPolygon):
                    self.menu_fence.items.append(MPMenuItem(layer.key, layer.key, '# kml fence \"' + layer.key + '\"'))
                #then add all the layers to the menu, ensuring to check the active layers
                #text elements aren't included on the menu
                if layer.key in self.curlayers and layer.key[-5:] != "-text":
                    self.menu.items.append(MPMenuCheckbox(layer.key, layer.key, '# kml toggle \"' + layer.key + '\"', checked=True))
                elif layer.key[-5:] != "-text":
                    self.menu.items.append(MPMenuCheckbox(layer.key, layer.key, '# kml toggle \"' + layer.key + '\"', checked=False))
            #and add the menu to the map popu menu
            self.module('map').add_menu(self.menu)
        self.menu_needs_refreshing = False
                            
    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
           
    def readkmz(self, filename):
        '''reads in a kmz file and returns xml nodes'''
        #Strip quotation marks if neccessary
        filename.strip('"')
        #Open the zip file (as applicable)    
        if filename[-4:] == '.kml':
            fo = open(filename, "r")
            fstring = fo.read()
            fo.close()
        elif filename[-4:] == '.kmz':
            zip=ZipFile(filename)
            for z in zip.filelist:
                if z.filename[-4:] == '.kml':
                    fstring=zip.read(z)
                    break
            else:
                raise Exception("Could not find kml file in %s" % filename)
        else:
            raise Exception("Is not a valid kml or kmz file in %s" % filename)
                
        #send into the xml parser
        kmlstring = parseString(fstring)

        #get all the placenames
        nodes=kmlstring.getElementsByTagName('Placemark')
        
        return nodes
            
    def readObject(self, innode):
        '''reads in a node and returns as a tuple: (type, name, points[])'''
        #get name
        names=innode.getElementsByTagName('name')[0].childNodes[0].data.strip()
        
        #get type
        pointType = 'Unknown'
        if len(innode.getElementsByTagName('LineString')) == 0 and len(innode.getElementsByTagName('Point')) == 0:
            pointType = 'Polygon'
        elif len(innode.getElementsByTagName('Polygon')) == 0 and len(innode.getElementsByTagName('Point')) == 0:
            pointType = 'Polygon'
        elif len(innode.getElementsByTagName('LineString')) == 0 and len(innode.getElementsByTagName('Polygon')) == 0:
            pointType = 'Point'
            
        #get coords
        coords = innode.getElementsByTagName('coordinates')[0].childNodes[0].data.strip()
        coordsSplit = coords.split()
        ret_s = []
        for j in coordsSplit:
            jcoord = j.split(',')
            if len(jcoord) == 3 and jcoord[0] != '' and jcoord[1] != '':
                #print("Got lon " + jcoord[0] + " and lat " + jcoord[1])
                ret_s.append((float(jcoord[1]), float(jcoord[0])))
            
        #return tuple
        return (str(pointType), str(names), ret_s)
    

def init(mpstate):
    '''initialise module'''
    return KmlReadModule(mpstate)
    
    
    
