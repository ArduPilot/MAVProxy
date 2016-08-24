#!/usr/bin/env python
'''Module to add or clear kml layers on the map

Copyright Stephen Dade 2016
Released under the GNU GPL version 3 or later

'''

import time, math, random
from pymavlink import mavutil
from cStringIO import StringIO
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
                         ["<clear>",
                          "<load> (FILENAME)", '<layers>'])
        self.curlayers = []
                          
    def cmd_param(self, args):
        '''control kml reading'''
        usage = "Usage: kml <clear | load (filename) | layers>"
        if len(args) < 1:
            print(usage)
            return
        elif args[0] == "clear":
            self.clearkml()
        elif args[0] == "load":
            if len(args) != 2:
                print("usage: kml load <filename>")
                return
            self.loadkml(args[1])
        elif args[0] == "layers":
            for layer in self.curlayers:
                print "Found layer: " + layer
        else:
            print(usage)
            return

    def clearkml(self):
        '''Clear the kml's from the map'''
        #go through all the current layers and remove them
        for layer in self.curlayers:
            self.mpstate.map.remove_object(layer)
        self.curlayers = []
        
    def loadkml(self, filename):
        '''Load a kml from file and put it on the map'''
        #Open the zip file
        nodes = self.readkmz(filename)

        #go through each object in the kml...
        for n in nodes:     
            point = self.readObject(n)
            
            #and place any polygons on the map
            if self.mpstate.map is not None and point[0] == 'Polygon':
                #print "Adding " + point[1]
                newcolour = (random.randint(0, 255), 0, random.randint(0, 255))
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon(point[1], point[2],
                                                             layer=2, linewidth=2, colour=newcolour))
                self.curlayers.append(point[1])
                
                
            #and points - 20m circle
            if self.mpstate.map is not None and point[0] == 'Point':
                #print "Adding " + point[1]
                icon = self.mpstate.map.icon('barrell.png')
                self.mpstate.map.add_object(mp_slipmap.SlipIcon(point[1], latlon = (point[2][0][0], point[2][0][1]), layer=3, img=icon, rotation=0, follow=False))
                self.curlayers.append(point[1])
                
        
    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
           
    def readkmz(self, filename):
        '''reads in a kmz file and returns xml nodes'''
        #Strip quotation marks if neccessary
        if filename.startswith('"') and filename.endswith('"'):
            filename = filename[1:-1]
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
                #print "Got lon " + jcoord[0] + " and lat " + jcoord[1]
                ret_s.append((float(jcoord[1]), float(jcoord[0])))
            
        #return tuple
        return (str(pointType), str(names), ret_s)
    

def init(mpstate):
    '''initialise module'''
    return KmlReadModule(mpstate)
    
    
    
