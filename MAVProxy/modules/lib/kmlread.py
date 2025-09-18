#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''


import lxml.etree as etree
from io import BytesIO as SIO
from zipfile import ZipFile
import pathlib
import re

namespaces = {
    'kml': 'http://www.opengis.net/kml/2.2',
    'gx': 'http://www.google.com/kml/ext/2.2',
}


class Style():
    def __init__(self, id):
        self.id = id
        self.line_colour = None


class StyleMap():
    def __init__(self, id, otherstyle):
        self.id = id
        self.otherstyle = otherstyle


class Polygon():
    def __init__(self, name, latlon, line_colour=None):
        self.name = name
        self.vertexes = latlon
        self.line_colour = line_colour


class Point():
    def __init__(self, name, latlon):
        self.name = name
        self.latlon = latlon


def readkmz(filename):
    '''reads in a kmz file and returns xml nodes'''
    xpath = xpath = ".//kml:Placemark"

    tree = etree_for_filepath(filename)

    return tree.findall(xpath, namespaces)


def etree_for_filepath(filename):
    '''reads in a kmz file and returns lxml.etree.ElementTree'''
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
    return etree.parse(SIO(fstring), parser)


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


class KMLRead():
    def __init__(self, filepath):
        self.filepath = filepath

    def placemark_nodes(self):
        return self.tree.findall(".//kml:Placemark", namespaces)

    def readObject(self, innode):
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
            # a Polygon
            latlon = []
            for c in coordinates.text.split():
                s = c.split(',')
                latlon.append((float(s[1]), float(s[0])))

            styleURL = find_tag_recursive(innode, 'styleUrl')
            line_colour = None
            if styleURL is not None:
                styleurl_name = styleURL.text.lstrip('#')
                if styleurl_name in self.stylemap:
                    stylemap = self.stylemap[styleurl_name]
                    otherstyle = stylemap.otherstyle
                    if otherstyle in self.style:
                        style = self.style[otherstyle]
                        line_colour = style.line_colour

            return Polygon(name.text, latlon, line_colour=line_colour)

        return ('Unknown', None, None)

    def parse(self):
        self.tree = etree_for_filepath(self.filepath)

        # extract styles:
        self.style = {}
        for s in self.tree.findall(".//gx:CascadingStyle", namespaces):
            idname = f"{{{namespaces['kml']}}}id"
            _id = s.get(idname)
            style = Style(_id)
            self.style[_id] = style
            line_style = find_tag_recursive(s, 'LineStyle')
            if line_style is None:
                continue
            colour = find_tag_recursive(line_style, 'color')
            g = re.match("(?P<alpha>..)(?P<r>..)(?P<g>..)(?P<b>..)", colour.text)
            if g is None:
                continue
            style.line_colour = (
                int(g.group("r"), 16),
                int(g.group("g"), 16),
                int(g.group("b"), 16)
            )

        # extract stylemaps:
        self.stylemap = {}
        for s in self.tree.findall(".//kml:StyleMap", namespaces):
            _id = s.get("id")
            styleurl_obj = None
            for pair in s.getchildren():
                if pair is None:
                    continue
                if find_tag_recursive(pair, "key").text == "normal":
                    styleurl_obj = find_tag_recursive(pair, "styleUrl")
                    if styleurl_obj is not None:
                        break
                    break

            if styleurl_obj is None:
                continue
            stylemap = StyleMap(_id, styleurl_obj.text.lstrip('#'))
            self.stylemap[_id] = stylemap


if __name__ == '__main__':
    import sys

    kml = KMLRead(sys.argv[1])
    kml.parse()

    for n in kml.placemark_nodes():
        obj = kml.readObject(n)
        if obj is None:
            continue

#    for n in readkmz(sys.argv[1]):
#        print(readObject(n))
