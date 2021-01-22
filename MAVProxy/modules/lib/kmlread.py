#!/usr/bin/env python

from zipfile import ZipFile
from xml.dom.minidom import parseString

def readkmz(filename):
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
            
def readObject(innode):
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
