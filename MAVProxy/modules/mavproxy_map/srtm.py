#!/usr/bin/env python

# Pylint: Disable name warnings
# pylint: disable-msg=C0103

"""Load and process SRTM data. Originally written by OpenStreetMap
Edited by CanberraUAV"""

import sys
if sys.version_info.major < 3:
    from HTMLParser import HTMLParser
    import httplib
else:
    from html.parser import HTMLParser
    import http.client as httplib

import re
import pickle
import os.path
import os
import zipfile
import array
import math
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc

childTileDownload = {}
childFileListDownload = {}
filelistDownloadActive = 0

class NoSuchTileError(Exception):
    """Raised when there is no tile for a region."""
    def __init__(self, lat, lon):
        Exception.__init__(self)
        self.lat = lat
        self.lon = lon

    def __str__(self):
        return "No SRTM tile for %d, %d available!" % (self.lat, self.lon)


class WrongTileError(Exception):
    """Raised when the value of a pixel outside the tile area is requested."""
    def __init__(self, tile_lat, tile_lon, req_lat, req_lon):
        Exception.__init__(self)
        self.tile_lat = tile_lat
        self.tile_lon = tile_lon
        self.req_lat = req_lat
        self.req_lon = req_lon

    def __str__(self):
        return "SRTM tile for %d, %d does not contain data for %d, %d!" % (
            self.tile_lat, self.tile_lon, self.req_lat, self.req_lon)

class InvalidTileError(Exception):
    """Raised when the SRTM tile file contains invalid data."""
    def __init__(self, lat, lon):
        Exception.__init__(self)
        self.lat = lat
        self.lon = lon

    def __str__(self):
        return "SRTM tile for %d, %d is invalid!" % (self.lat, self.lon)

class SRTMDownloader():
    """Automatically download SRTM tiles."""
    def __init__(self, server="firmware.ardupilot.org",
                 directory="/SRTM/",
                 cachedir=None,
                 offline=0,
                 debug=False):

        if cachedir is None:
            try:
                cachedir = os.path.join(os.environ['HOME'], '.tilecache/SRTM')
            except Exception:
                if 'LOCALAPPDATA' in os.environ:
                    cachedir = os.path.join(os.environ['LOCALAPPDATA'], '.tilecache/SRTM')
                else:
                    import tempfile
                    cachedir = os.path.join(tempfile.gettempdir(), 'MAVProxySRTM')

        self.debug = debug
        self.offline = offline
        self.offlinemessageshown = 0
        if self.offline == 1 and self.debug:
            print("Map Module in Offline mode")
        self.first_failure = False
        self.server = server
        self.directory = directory
        self.cachedir = cachedir
        if self.debug:
            print("SRTMDownloader - server=%s, directory=%s." % (self.server, self.directory))
        if not os.path.exists(cachedir):
            mp_util.mkdir_p(cachedir)
        self.filelist = {}
        self.filename_regex = re.compile(
                r"([NS])(\d{2})([EW])(\d{3})\.hgt\.zip")
        self.filelist_file = os.path.join(self.cachedir, "filelist_python")
        self.min_filelist_len = 14500

    def loadFileList(self):
        """Load a previously created file list or create a new one if none is
            available."""
        try:
            data = open(self.filelist_file, 'rb')
        except IOError:
            '''print("No SRTM cached file list. Creating new one!")'''
            if self.offline == 0:
                self.createFileList()
            return
        try:
            self.filelist = pickle.load(data)
            data.close()
            if len(self.filelist) < self.min_filelist_len:
                self.filelist = {}
                if self.offline == 0:
                    self.createFileList()
        except:
            '''print("Unknown error loading cached SRTM file list. Creating new one!")'''
            if self.offline == 0:
                self.createFileList()

    def createFileList(self):
        """SRTM data is split into different directories, get a list of all of
            them and create a dictionary for easy lookup."""
        global childFileListDownload
        global filelistDownloadActive
        mypid = os.getpid()
        if mypid not in childFileListDownload or not childFileListDownload[mypid].is_alive():
            childFileListDownload[mypid] = multiproc.Process(target=self.createFileListHTTP)
            filelistDownloadActive = 1
            childFileListDownload[mypid].start()
            filelistDownloadActive = 0

    def getURIWithRedirect(self, url):
        '''fetch a URL with redirect handling'''
        tries = 0
        while tries < 5:
                conn = httplib.HTTPSConnection(self.server)
                conn.request("GET", url)
                r1 = conn.getresponse()
                if r1.status in [301, 302, 303, 307]:
                    location = r1.getheader('Location')
                    if self.debug:
                        print("redirect from %s to %s" % (url, location))
                    url = location
                    conn.close()
                    tries += 1
                    continue
                data = r1.read()
                conn.close()
                if sys.version_info.major < 3:
                    return data
                else:
                    encoding = r1.headers.get_content_charset()
                    if encoding is not None:
                        return data.decode(encoding)
                    elif ".zip" in url or ".hgt" in url:
                        return data
                    else:
                        return data.decode('utf-8')
        return None

    def createFileListHTTP(self):
        """Create a list of the available SRTM files on the server using
        HTTP file transfer protocol (rather than ftp).
        30may2010  GJ ORIGINAL VERSION
        """
        mp_util.child_close_fds()
        if self.debug:
            print("Connecting to %s" % self.server, self.directory)
        try:
            data = self.getURIWithRedirect(self.directory)
        except Exception:
            return
        parser = parseHTMLDirectoryListing()
        parser.feed(data)
        continents = parser.getDirListing()
        if self.debug:
            print('continents: ', continents)

        for continent in continents:
            if not continent[0].isalpha() or continent.startswith('README'):
                continue
            if self.debug:
                print("Downloading file list for: ", continent)
            url = "%s%s" % (self.directory,continent)
            if self.debug:
                print("fetching %s" % url)
            try:
                data = self.getURIWithRedirect(url)
            except Exception as ex:
                print("Failed to download %s : %s" % (url, ex))
                continue
            parser = parseHTMLDirectoryListing()
            parser.feed(data)
            files = parser.getDirListing()

            for filename in files:
                self.filelist[self.parseFilename(filename)] = (
                            continent, filename)

            '''print(self.filelist)'''
        # Add meta info
        self.filelist["server"] = self.server
        self.filelist["directory"] = self.directory
        tmpname = self.filelist_file + ".tmp"
        with open(tmpname , 'wb') as output:
            pickle.dump(self.filelist, output)
            output.close()
            try:
                os.unlink(self.filelist_file)
            except Exception:
                pass
            try:
                os.rename(tmpname, self.filelist_file)
            except Exception:
                pass
        if self.debug:
            print("created file list with %u entries" % len(self.filelist))

    def parseFilename(self, filename):
        """Get lat/lon values from filename."""
        match = self.filename_regex.match(filename)
        if match is None:
            # TODO?: Raise exception?
            '''print("Filename", filename, "unrecognized!")'''
            return None
        lat = int(match.group(2))
        lon = int(match.group(4))
        if match.group(1) == "S":
            lat = -lat
        if match.group(3) == "W":
            lon = -lon
        return lat, lon

    def getTile(self, lat, lon):
        """Get a SRTM tile object. This function can return either an SRTM1 or
            SRTM3 object depending on what is available, however currently it
            only returns SRTM3 objects."""
        global childFileListDownload
        global filelistDownloadActive
        mypid = os.getpid()
        if mypid in childFileListDownload and childFileListDownload[mypid].is_alive():
            if self.debug:
                print("still getting file list")
            return 0
        elif not os.path.isfile(self.filelist_file) and filelistDownloadActive == 0:
            self.createFileList()
            return 0
        elif not self.filelist:
            if self.debug:
                print("Filelist download complete, loading data ", self.filelist_file)
            data = open(self.filelist_file, 'rb')
            self.filelist = pickle.load(data)
            data.close()

        try:
            continent, filename = self.filelist[(int(lat), int(lon))]
        except KeyError:
            if len(self.filelist) > self.min_filelist_len:
                # we appear to have a full filelist - this must be ocean
                return SRTMOceanTile(int(lat), int(lon))
            return 0

        global childTileDownload
        mypid = os.getpid()
        if not os.path.exists(os.path.join(self.cachedir, filename)):
            if not mypid in childTileDownload or not childTileDownload[mypid].is_alive():
                try:
                    childTileDownload[mypid] = multiproc.Process(target=self.downloadTile, args=(str(continent), str(filename)))
                    childTileDownload[mypid].start()
                except Exception as ex:
                    if mypid in childTileDownload:
                        childTileDownload.pop(mypid)
                    return 0
                '''print("Getting Tile")'''
            return 0
        elif mypid in childTileDownload and childTileDownload[mypid].is_alive():
            '''print("Still Getting Tile")'''
            return 0
        # TODO: Currently we create a new tile object each time.
        # Caching is required for improved performance.
        try:
            return SRTMTile(os.path.join(self.cachedir, filename), int(lat), int(lon))
        except InvalidTileError:
            return 0

    def downloadTile(self, continent, filename):
        #Use HTTP
        mp_util.child_close_fds()
        if self.offline == 1:
            return
        filepath = "%s%s%s" % \
                     (self.directory,continent,filename)
        try:
            data = self.getURIWithRedirect(filepath)
            if data:
                self.ftpfile = open(os.path.join(self.cachedir, filename), 'wb')
                self.ftpfile.write(data)
                self.ftpfile.close()
                self.ftpfile = None
        except Exception as e:
            if not self.first_failure:
                print("SRTM Download failed %s on server %s" % (filepath, self.server))
                self.first_failure = True
            pass


class SRTMTile:
    """Base class for all SRTM tiles.
        Each SRTM tile is size x size pixels big and contains
        data for the area from (lat, lon) to (lat+1, lon+1) inclusive.
        This means there is a 1 pixel overlap between tiles. This makes it
        easier for as to interpolate the value, because for every point we
        only have to look at a single tile.
        """
    def __init__(self, f, lat, lon):
        try:
            zipf = zipfile.ZipFile(f, 'r')
        except Exception:
            raise InvalidTileError(lat, lon)
        names = zipf.namelist()
        if len(names) != 1:
            raise InvalidTileError(lat, lon)
        data = zipf.read(names[0])
        self.size = int(math.sqrt(len(data)/2)) # 2 bytes per sample
        # Currently only SRTM1/3 is supported
        if self.size not in (1201, 3601):
            raise InvalidTileError(lat, lon)
        self.data = array.array('h', data)
        self.data.byteswap()
        if len(self.data) != self.size * self.size:
            raise InvalidTileError(lat, lon)
        self.lat = lat
        self.lon = lon

    @staticmethod
    def _avg(value1, value2, weight):
        """Returns the weighted average of two values and handles the case where
            one value is None. If both values are None, None is returned.
        """
        if value1 is None:
            return value2
        if value2 is None:
            return value1
        return value2 * weight + value1 * (1 - weight)

    def calcOffset(self, x, y):
        """Calculate offset into data array. Only uses to test correctness
            of the formula."""
        # Datalayout
        # X = longitude
        # Y = latitude
        # Sample for size 1201x1201
        #  (   0/1200)     (   1/1200)  ...    (1199/1200)    (1200/1200)
        #  (   0/1199)     (   1/1199)  ...    (1199/1199)    (1200/1199)
        #       ...            ...                 ...             ...
        #  (   0/   1)     (   1/   1)  ...    (1199/   1)    (1200/   1)
        #  (   0/   0)     (   1/   0)  ...    (1199/   0)    (1200/   0)
        #  Some offsets:
        #  (0/1200)     0
        #  (1200/1200)  1200
        #  (0/1199)     1201
        #  (1200/1199)  2401
        #  (0/0)        1201*1200
        #  (1200/0)     1201*1201-1
        return x + self.size * (self.size - y - 1)

    def getPixelValue(self, x, y):
        """Get the value of a pixel from the data, handling voids in the
            SRTM data."""
        assert x < self.size, "x: %d<%d" % (x, self.size)
        assert y < self.size, "y: %d<%d" % (y, self.size)
        # Same as calcOffset, inlined for performance reasons
        offset = x + self.size * (self.size - y - 1)
        #print(offset)
        value = self.data[offset]
        if value == -32768:
            return -1 # -32768 is a special value for areas with no data
        return value


    def getAltitudeFromLatLon(self, lat, lon):
        """Get the altitude of a lat lon pair, using the four neighbouring
            pixels for interpolation.
        """
        # print("-----\nFromLatLon", lon, lat)
        lat -= self.lat
        lon -= self.lon
        # print("lon, lat", lon, lat)
        if lat < 0.0 or lat >= 1.0 or lon < 0.0 or lon >= 1.0:
            raise WrongTileError(self.lat, self.lon, self.lat+lat, self.lon+lon)
        x = lon * (self.size - 1)
        y = lat * (self.size - 1)
        # print("x,y", x, y)
        x_int = int(x)
        x_frac = x - int(x)
        y_int = int(y)
        y_frac = y - int(y)
        # print("frac", x_int, x_frac, y_int, y_frac)
        value00 = self.getPixelValue(x_int, y_int)
        value10 = self.getPixelValue(x_int+1, y_int)
        value01 = self.getPixelValue(x_int, y_int+1)
        value11 = self.getPixelValue(x_int+1, y_int+1)
        value1 = self._avg(value00, value10, x_frac)
        value2 = self._avg(value01, value11, x_frac)
        value  = self._avg(value1,  value2, y_frac)
        # print("%4d %4d | %4d\n%4d %4d | %4d\n-------------\n%4d" % (
        #        value00, value10, value1, value01, value11, value2, value))
        return value

class SRTMOceanTile(SRTMTile):
    '''a tile for areas of zero altitude'''
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

    def getAltitudeFromLatLon(self, lat, lon):
        return 0


class parseHTMLDirectoryListing(HTMLParser):

    def __init__(self):
        #print("parseHTMLDirectoryListing.__init__")
        HTMLParser.__init__(self)
        self.title="Undefined"
        self.isDirListing = False
        self.dirList=[]
        self.inTitle = False
        self.inHyperLink = False
        self.currAttrs=""
        self.currHref=""

    def handle_starttag(self, tag, attrs):
        #print("Encountered the beginning of a %s tag" % tag)
        if tag=="title":
            self.inTitle = True
        if tag == "a":
            self.inHyperLink = True
            self.currAttrs=attrs
            for attr in attrs:
                if attr[0]=='href':
                    self.currHref = attr[1]


    def handle_endtag(self, tag):
        #print("Encountered the end of a %s tag" % tag)
        if tag=="title":
            self.inTitle = False
        if tag == "a":
            # This is to avoid us adding the parent directory to the list.
            if self.currHref!="":
                self.dirList.append(self.currHref)
            self.currAttrs=""
            self.currHref=""
            self.inHyperLink = False

    def handle_data(self,data):
        if self.inTitle:
            self.title = data
            '''print("title=%s" % data)'''
            if "Index of" in self.title:
                #print("it is an index!!!!")
                self.isDirListing = True
        if self.inHyperLink:
            # We do not include parent directory in listing.
            if  "Parent Directory" in data:
                self.currHref=""

    def getDirListing(self):
        return self.dirList

#DEBUG ONLY
if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description='srtm test')

    parser.add_argument("--lat", type=float, default=-35.363261)
    parser.add_argument("--lon", type=float, default=149.165230)
    parser.add_argument("--debug", action='store_true', default=False)
    args = parser.parse_args()

    downloader = SRTMDownloader(debug=args.debug)
    downloader.loadFileList()
    import time
    from math import floor
    start = time.time()
    while time.time() - start < 30:
        tile = downloader.getTile(int(floor(args.lat)), int(floor(args.lon)))
        if tile:
            print("Download took %.1fs alt=%.1f" % (time.time()-start, tile.getAltitudeFromLatLon(args.lat, args.lon)))
            break
        time.sleep(0.2)
