#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''


import lxml.etree as etree
from io import BytesIO as SIO
from zipfile import ZipFile
import pathlib

namespaces = {'kml': 'http://www.opengis.net/kml/2.2'}


class Polygon():
    def __init__(self, name, latlon):
        self.name = name
        self.vertexes = latlon


class Point():
    def __init__(self, name, latlon):
        self.name = name
        self.latlon = latlon


def readkmz(filename):
    '''reads in a kmz file and returns xml nodes'''
    # Strip quotation marks if neccessary
    filename.strip('"')
    # Open the zip file (as applicable)
    suffix = pathlib.Path(filename).suffix
    if suffix.lower() == '.kml':
        fo = open(filename, "rb")
        fstring = fo.read()
        fo.close()
    elif suffix.lower() == '.kmz':
        zip = ZipFile(filename)
        fstring = None
        for z in zip.filelist:
            if z.filename[-4:] == '.kml':
                fstring = zip.read(z)
                break
        if fstring is None:
            raise Exception("Could not find kml file in %s" % filename)
    else:
        raise Exception(f"load expects a .kml or .kmz file, got ({suffix}) from ({filename})")

    parser = etree.XMLParser(encoding='utf-8', recover=True)
    tree = etree.parse(SIO(fstring), parser)
    xpath = xpath = ".//kml:Placemark"

    return tree.findall(xpath, namespaces)


def find_tag(node, tagname):
    for c in node.getchildren():
        if c.tag == "{" + namespaces['kml'] + "}" + tagname:
            return c
    return None


def find_tag_recursive(node, tagname):
    for c in node.getchildren():
        if c.tag == "{" + namespaces['kml'] + "}" + tagname:
            return c
        if hasattr(c, 'getchildren'):
            ret = find_tag_recursive(c, tagname)
            if ret is not None:
                return ret
    return None


def readObject(innode):
    '''reads in a node and returns as a tuple: (type, name, points[])'''
    # get name
    name = find_tag(innode, 'name')
    if name is None:
        return None
    point = find_tag(innode, 'Point')
    if point is not None:
        coordinates = find_tag(point, 'coordinates')
        if coordinates is None:
            return None
        s = coordinates.text.split(',')
        return Point(name.text, (float(s[1]), float(s[0])))

    coordinates = find_tag_recursive(innode, 'coordinates')
    if coordinates is not None:
        latlon = []
        for c in coordinates.text.split():
            s = c.split(',')
            latlon.append((float(s[1]), float(s[0])))
        return Polygon(name.text, latlon)

    return ('Unknown', None, None)


if __name__ == '__main__':
    import sys
    nodes = readkmz(sys.argv[1])
    for n in nodes:
        print(readObject(n))
