#!/usr/bin/env python3
'''
access satellite map tile database

some functions are based on code from mapUtils.py in gmapcatcher

Andrew Tridgell
May 2012
released under GNU GPL v3 or later

conventions:
 - lat,lon is top left corner of area
 - ground_width is width at top of area

see also:
  https://wiki.openstreetmap.org/wiki/Zoom_levels

AP_FLAKE8_CLEAN
'''

import collections
import hashlib
import sys
import threading
import os
import pathlib
import string
import time
import cv2
import numpy as np

from math import log, tan, radians, degrees, sin, cos, exp, pi, asin, atan

if sys.version_info.major < 3:
    from urllib2 import Request as url_request
    from urllib2 import urlopen as url_open
    from urllib2 import URLError as url_error
else:
    from urllib.request import Request as url_request
    from urllib.request import urlopen as url_open
    from urllib.error import URLError as actual_url_error
    from http.client import RemoteDisconnected
    url_error = (RemoteDisconnected, actual_url_error)

from MAVProxy.modules.lib import mp_util


class TileException(Exception):
    '''tile error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)


TILE_SERVICES = {
    # thanks to http://go2log.com/2011/09/26/fetching-tiles-for-offline-map/
    # for the URL mapping info
    "GoogleSat"      : "https://mt${GOOG_DIGIT}.google.com/vt/lyrs=s@812&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleMap"      : "https://mt${GOOG_DIGIT}.google.com/vt/lyrs=m@132&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleTer"      : "https://mt${GOOG_DIGIT}.google.com/vt/v=t@132,r@249&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleChina"    : "http://mt${GOOG_DIGIT}.google.cn/vt/lyrs=m@121&hl=en&gl=cn&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleChinaSat" : "http://mt${GOOG_DIGIT}.google.cn/vt/lyrs=s@121&hl=en&gl=cn&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    # Tianditu (天地图) - WGS-84, fast in China mainland; set TIANDITU_KEY env variable
    # Register free key at: https://console.tianditu.gov.cn/
    # URL format reference: http://lbs.tianditu.gov.cn/server/MapService.html
    "TiandituSat"    : "https://t${TD_DIGIT}.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX=${ZOOM}&TILEROW=${Y}&TILECOL=${X}&tk=${TIANDITU_KEY}",  # noqa:E501
    "TiandituMap"    : "https://t${TD_DIGIT}.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX=${ZOOM}&TILEROW=${Y}&TILECOL=${X}&tk=${TIANDITU_KEY}",  # noqa:E501
    "TiandituTer"    : "https://t${TD_DIGIT}.tianditu.gov.cn/ter_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=ter&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX=${ZOOM}&TILEROW=${Y}&TILECOL=${X}&tk=${TIANDITU_KEY}",  # noqa:E501
    "MicrosoftBrMap" : "http://imakm${MS_DIGITBR}.maplink3.com.br/maps.ashx?v=${QUAD}|t&call=2.2.4",
    "MicrosoftHyb"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/h${QUAD}.png?g=441&mkt=en-us&n=z",
    "MicrosoftSat"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/a${QUAD}.png?g=441&mkt=en-us&n=z",
    "MicrosoftMap"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&n=z",
    "MicrosoftTer"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&shading=hill&n=z",
    "OpenStreetMap"  : "http://tile.openstreetmap.org/${ZOOM}/${X}/${Y}.png",
    "Gulesider DK,NO,SE,FI" : "https://map.eniro.com/geowebcache/service/tms1.0.0/map/${ZOOM}/${X}/${ENI_Y}.png",
    "Kartverket(Norway-Topo)": "https://cache.kartverket.no/v1/wmts/1.0.0/topo/default/webmercator/${ZOOM}/${Y}/${X}.png",
    "Svalbard" : "https://geodata.npolar.no/arcgis/rest/services/Basisdata/NP_Basiskart_Svalbard_WMTS_3857/MapServer/WMTS/tile/1.0.0/Basisdata_NP_Basiskart_Svalbard_WMTS_3857/default/default028mm/${ZOOM}/${Y}/${X}", # noqa:E501
    "MapsNSW" : "http://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_Topo_Map/MapServer/tile/${Z}/${Y}/${X}/jpg",
    "OpenTopoMapA": "https://a.tile.opentopomap.org/${Z}/${X}/${Y}.png",
}


# minimum usable zoom level per service. Most services serve a single
# whole-world tile at zoom 0, but Bing/Microsoft quadkey tiles start at
# zoom 1 (there is no quadkey for zoom 0).
SERVICE_MIN_ZOOM = {
    "GoogleSat": 0,
    "GoogleMap": 0,
    "GoogleTer": 0,
    "GoogleChina": 0,
    "GoogleChinaSat": 0,
    "OpenStreetMap": 0,
    "OpenTopoMapA": 0,
}
DEFAULT_MIN_ZOOM = 1


def service_min_zoom(service):
    '''return the minimum usable zoom level for a tile service'''
    return SERVICE_MIN_ZOOM.get(service, DEFAULT_MIN_ZOOM)


def mercator_y(lat):
    '''Web Mercator projected y for a latitude (dimensionless, unbounded)'''
    lat = mp_util.constrain(lat, -89.9999, 89.9999)
    return log(tan(pi/4 + radians(lat)/2))


def mercator_lat(y):
    '''inverse Web Mercator: latitude in degrees for a projected y'''
    return degrees(2*atan(exp(y)) - pi/2)


# these are the md5sums of "unavailable" tiles
BLANK_TILES = set([
    "d16657bbee25d7f15c583f5c5bf23f50",
    "c0e76e6e90ff881da047c15dbea380c7",
    "d41d8cd98f00b204e9800998ecf8427e",
])

# all tiles are 256x256
TILES_WIDTH = 256
TILES_HEIGHT = 256


class TileServiceInfo:
    '''a lookup object for the URL templates'''
    def __init__(self, x, y, zoom):
        self.X = x
        self.Y = y
        self.Z = zoom
        quadcode = ''
        for i in range(zoom - 1, -1, -1):
            quadcode += str((((((y >> i) & 1) << 1) + ((x >> i) & 1))))
        self.ZOOM = zoom
        self.QUAD = quadcode
        self.OAM_ZOOM = 17 - zoom
        self.GOOG_DIGIT = (x + y) & 3
        self.MS_DIGITBR = (((y & 1) << 1) + (x & 1)) + 1
        self.MS_DIGIT = (((y & 3) << 1) + (x & 1))
        self.Y_DIGIT = (x + y + zoom) % 3 + 1
        self.GALILEO = "Galileo"[0:(3 * x + y) & 7]
        self.ENI_Y = (1 << zoom)-1-y
        self.TD_DIGIT = (x + y) & 7
        self.TIANDITU_KEY = os.environ.get('TIANDITU_KEY', '')

    def __getitem__(self, a):
        return str(getattr(self, a))


class TileInfo:
    '''description of a tile'''
    def __init__(self, tile, zoom, service, offset=(0, 0)):
        self.tile = tile
        (self.x, self.y) = tile
        self.zoom = zoom
        self.service = service
        (self.offsetx, self.offsety) = offset
        self.refresh_time()

    def key(self):
        '''tile cache key'''
        return (self.tile, self.zoom, self.service)

    def refresh_time(self):
        '''reset the request time'''
        self.request_time = time.time()

    def coord(self, offset=(0, 0)):
        '''return lat,lon within a tile given (offsetx,offsety)'''
        (tilex, tiley) = self.tile
        (offsetx, offsety) = offset
        world_tiles = 1 << self.zoom
        x = (tilex + 1.0*offsetx/TILES_WIDTH) / (world_tiles/2.) - 1
        y = (tiley + 1.0*offsety/TILES_HEIGHT) / (world_tiles/2.) - 1
        lon = mp_util.wrap_180(x * 180.0)
        y = exp(-y*2*pi)
        e = (y-1)/(y+1)
        lat = 180.0/pi * asin(e)
        return (lat, lon)

    def size(self):
        '''return tile size as (width,height) in meters'''
        (lat1, lon1) = self.coord((0, 0))
        (lat2, lon2) = self.coord((TILES_WIDTH, 0))
        width = mp_util.gps_distance(lat1, lon1, lat2, lon2)
        (lat2, lon2) = self.coord((0, TILES_HEIGHT))
        height = mp_util.gps_distance(lat1, lon1, lat2, lon2)
        return (width, height)

    def distance(self, lat, lon):
        '''distance of this tile from a given lat/lon'''
        (tlat, tlon) = self.coord((TILES_WIDTH/2, TILES_HEIGHT/2))
        return mp_util.gps_distance(lat, lon, tlat, tlon)

    def path(self):
        '''return relative path of tile image'''
        (x, y) = self.tile
        return os.path.join('%u' % self.zoom,
                            '%u' % y,
                            '%u.img' % x)

    def url(self, service):
        '''return URL for a tile'''
        if service not in TILE_SERVICES:
            raise TileException('unknown tile service %s' % service)
        url = string.Template(TILE_SERVICES[service])
        (x, y) = self.tile
        tile_info = TileServiceInfo(x, y, self.zoom)
        return url.substitute(tile_info)


class TileInfoScaled(TileInfo):
    '''information on a tile with scale information and placement'''
    def __init__(self, tile, zoom, scale, src, dst, service):
        TileInfo.__init__(self, tile, zoom, service)
        self.scale = scale
        (self.srcx, self.srcy) = src
        (self.dstx, self.dsty) = dst


class MPTile:
    '''map tile object'''
    def __init__(self, cache_path=None, download=True, cache_size=500,
                 service="MicrosoftSat", tile_delay=0.3, debug=False,
                 max_zoom=19, refresh_age=30*24*60*60):

        if cache_path is None:
            try:
                cache_path = os.path.join(os.environ['HOME'], '.tilecache')
            except Exception:
                if 'LOCALAPPDATA' in os.environ:
                    cache_path = os.path.join(os.environ['LOCALAPPDATA'], '.tilecache')
                else:
                    import tempfile
                    cache_path = os.path.join(tempfile.gettempdir(), '.tilecache')

        if not os.path.exists(cache_path):
            mp_util.mkdir_p(cache_path)

        self.cache_path = cache_path
        self.max_zoom = max_zoom
        self.min_zoom = service_min_zoom(service)
        self.download = download
        self.cache_size = cache_size
        self.tile_delay = tile_delay
        self.service = service
        self.debug = debug
        self.refresh_age = refresh_age

        if service not in TILE_SERVICES:
            raise TileException('unknown tile service %s' % service)

        # _download_pending is a dictionary of TileInfo objects
        self._download_pending = {}
        self._download_thread = None
        self._loading = mp_icon('loading.jpg')
        self._unavailable = mp_icon('unavailable.jpg')
        self._tile_cache = collections.OrderedDict()

    def set_service(self, service):
        '''set tile service'''
        self.service = service
        self.min_zoom = service_min_zoom(service)

    def get_service(self):
        '''get tile service'''
        return self.service

    def get_service_list(self):
        '''return list of available services'''
        service_list = TILE_SERVICES.keys()
        return sorted(service_list)

    def set_download(self, download):
        '''set download enable'''
        self.download = download

    def coord_to_tile(self, lat, lon, zoom):
        '''convert lat/lon/zoom to a TileInfo'''
        world_tiles = 1 << zoom
        lon = mp_util.wrap_180(lon)
        x = world_tiles / 360.0 * (lon + 180.0)
        tiles_pre_radian = world_tiles / (2 * pi)
        e = sin(radians(lat))
        e = mp_util.constrain(e, -1+1.0e-15, 1-1.0e-15)
        y = world_tiles/2 + 0.5*log((1+e)/(1-e)) * (-tiles_pre_radian)
        offsetx = int((x - int(x)) * TILES_WIDTH)
        offsety = int((y - int(y)) * TILES_HEIGHT)
        return TileInfo((int(x) % world_tiles, int(y) % world_tiles), zoom, self.service, offset=(offsetx, offsety))

    def tile_to_path(self, tile):
        '''return full path to a tile'''
        return os.path.join(self.cache_path, self.service, tile.path())

    def coord_to_tilepath(self, lat, lon, zoom):
        '''return the tile ID that covers a latitude/longitude at
        a specified zoom level
        '''
        tile = self.coord_to_tile(lat, lon, zoom)
        return self.tile_to_path(tile)

    def tiles_pending(self):
        '''return number of tiles pending download'''
        return len(self._download_pending)

    def downloader(self):
        '''the download thread'''
        while self.tiles_pending() > 0:
            time.sleep(self.tile_delay)

            keys = sorted(self._download_pending.keys())

            # work out which one to download next, choosing by request_time
            tile_info = self._download_pending[keys[0]]
            for key in keys:
                if self._download_pending[key].request_time > tile_info.request_time:
                    tile_info = self._download_pending[key]

            url = tile_info.url(self.service)
            path = self.tile_to_path(tile_info)
            key = tile_info.key()

            try:
                if self.debug:
                    print("Downloading %s [%u left]" % (url, len(keys)))
                req = url_request(url)
                req.add_header('User-Agent', 'MAVProxy')

                # try to re-use our cached data:
                try:
                    mtime = os.path.getmtime(path)
                    req.add_header('If-Modified-Since', time.strftime('%a, %d %b %Y %H:%M:%S GMT', time.gmtime(mtime)))
                except Exception:
                    pass

                if url.find('google') != -1:
                    req.add_header('Referer', 'https://maps.google.com/')
                resp = url_open(req)
                headers = resp.info()
            except url_error as e:
                try:
                    if e.getcode() == 304:
                        # cache hit; touch the file to reset its refresh time
                        pathlib.Path(path).touch()
                        self._download_pending.pop(key)
                        continue
                except Exception:
                    pass

                # print('Error loading %s' % url)
                if key not in self._tile_cache:
                    self._tile_cache[key] = self._unavailable
                self._download_pending.pop(key)
                if self.debug:
                    print("Failed %s: %s" % (url, str(e)))
                continue
            if 'content-type' not in headers or headers['content-type'].find('image') == -1:
                if key not in self._tile_cache:
                    self._tile_cache[key] = self._unavailable
                self._download_pending.pop(key)
                if self.debug:
                    print("non-image response %s" % url)
                continue
            else:
                img = resp.read()

            # see if its a blank/unavailable tile
            md5 = hashlib.md5(img).hexdigest()
            if md5 in BLANK_TILES:
                if self.debug:
                    print("blank tile %s" % url)
                    if key not in self._tile_cache:
                        self._tile_cache[key] = self._unavailable
                self._download_pending.pop(key)
                continue

            mp_util.mkdir_p(os.path.dirname(path))
            h = open(path+'.tmp', 'wb')
            h.write(img)
            h.close()
            try:
                os.unlink(path)
            except Exception:
                pass
            os.rename(path+'.tmp', path)
            self._download_pending.pop(key)
        self._download_thread = None

    def start_download_thread(self):
        '''start the downloader'''
        if self._download_thread:
            return
        t = threading.Thread(target=self.downloader)
        t.daemon = True
        self._download_thread = t
        t.start()

    def load_tile_lowres(self, tile):
        '''load a lower resolution tile from cache to fill in a
        map while waiting for a higher resolution tile'''
        if tile.zoom == self.min_zoom:
            return None

        # find the equivalent lower res tile
        (lat, lon) = tile.coord()

        width2 = TILES_WIDTH
        height2 = TILES_HEIGHT

        for zoom2 in range(tile.zoom-1, self.min_zoom-1, -1):
            width2 //= 2
            height2 //= 2

            if width2 == 0 or height2 == 0:
                break

            tile_info = self.coord_to_tile(lat, lon, zoom2)

            # see if its in the tile cache
            key = tile_info.key()
            if key in self._tile_cache:
                img = self._tile_cache[key]
                if np.array_equal(img, np.array(self._unavailable)):
                    continue
            else:
                path = self.tile_to_path(tile_info)
                if not os.path.exists(path):
                    continue
                img = cv2.imread(path)
                if img is None:
                    continue
                # cv2.rectangle(img, (0,0), (TILES_WIDTH-1,TILES_WIDTH-1), (255,0,0), 1)
                # add it to the tile cache
                self._tile_cache[key] = img
                while len(self._tile_cache) > self.cache_size:
                    self._tile_cache.popitem(0)

            # copy out the quadrant we want
            availx = min(TILES_WIDTH - tile_info.offsetx, width2)
            availy = min(TILES_HEIGHT - tile_info.offsety, height2)
            if availx != width2 or availy != height2:
                continue
            roi = img[tile_info.offsety:tile_info.offsety+height2, tile_info.offsetx:tile_info.offsetx+width2]

            # and scale it
            try:
                scaled = cv2.resize(roi, (TILES_HEIGHT, TILES_WIDTH))
            except Exception:
                return None
            # cv.Rectangle(scaled, (0,0), (255,255), (0,255,0), 1)
            return scaled
        return None

    def load_tile(self, tile):
        '''load a tile from cache or tile server'''

        # see if its in the tile cache
        key = tile.key()
        if key in self._tile_cache:
            img = self._tile_cache[key]
            if np.array_equal(img, self._unavailable):
                img = self.load_tile_lowres(tile)
                if img is None:
                    img = self._unavailable
            return img

        path = self.tile_to_path(tile)
        if not os.path.exists(path):
            ret = None
        else:
            ret = cv2.imread(path)
        if ret is not None:
            # cv2.rectangle(ret, (0,0), (TILES_WIDTH-1,TILES_WIDTH-1), (255,0,0), 1)
            # if it is an old tile, then try to refresh
            if os.path.getmtime(path) + self.refresh_age < time.time():
                try:
                    self._download_pending[key].refresh_time()
                except Exception:
                    self._download_pending[key] = tile
                self.start_download_thread()

            # add it to the tile cache
            self._tile_cache[key] = ret
            while len(self._tile_cache) > self.cache_size:
                self._tile_cache.popitem(0)
            return ret

        if not self.download:
            img = self.load_tile_lowres(tile)
            if img is None:
                img = self._unavailable
            return img

        try:
            self._download_pending[key].refresh_time()
        except Exception:
            self._download_pending[key] = tile
        self.start_download_thread()

        img = self.load_tile_lowres(tile)
        if img is None:
            img = self._loading
        return img

    def scaled_tile(self, tile):
        '''return a scaled tile'''
        width = int(TILES_WIDTH / tile.scale)
        height = int(TILES_HEIGHT / tile.scale)
        full_tile = self.load_tile(tile)
        scaled_tile = cv2.resize(full_tile, (height, width))
        return scaled_tile

    def coord_from_area(self, x, y, lat, lon, width, ground_width):
        '''return (lat,lon) for a pixel in an area image
        x is pixel coord to the right from top,left
        y is pixel coord down from top left

        This is the exact inverse of coord_to_pixel, using a consistent
        Web Mercator projection so it works at any scale.
        '''
        pixel_width_equator = (ground_width / float(width)) / cos(radians(lat))
        C = mp_util.radius_of_earth / pixel_width_equator

        lat2 = mercator_lat(mercator_y(lat) - y / C)
        lon2 = mp_util.wrap_180(lon + degrees(x / C))
        return (lat2, lon2)

    def coord_to_pixel(self, lat, lon, width, ground_width, lat2, lon2):
        '''return pixel coordinate (px,py) for position (lat2,lon2)
        in an area image. Note that the results are relative to top,left
        and may be outside the image
        ground_width is width at lat,lon
        px is pixel coord to the right from top,left
        py is pixel coord down from top left

        Uses the exact Web Mercator projection (matching the tile imagery),
        so overlays and clicks line up at any scale.
        '''
        pixel_width_equator = (ground_width / float(width)) / cos(radians(lat))
        C = mp_util.radius_of_earth / pixel_width_equator

        y = C * (mercator_y(lat) - mercator_y(lat2))
        x = C * radians(mp_util.wrap_180(lon2 - lon))
        # for views wider than 180 deg (near whole-planet) a point can be more
        # than half a world east/west of the top-left; pick the wrap nearest
        # the view so it lands on the correct side instead of folding over
        world_px = 2 * pi * C
        if world_px > 0:
            x += round((width / 2.0 - x) / world_px) * world_px
        return (int(round(x)), int(round(y)))

    def choose_zoom(self, lat, lon, width, ground_width, zoom=None):
        '''choose a tile zoom level and the scale factor (native tile pixels
        per screen pixel) for an area of the given ground_width in metres.
        The smallest zoom whose tiles do not need upscaling is chosen.

        scale is derived directly from the Web Mercator projection at the
        top-left latitude so it matches coord_to_pixel exactly (using
        tile.size(), which measures at the tile's own latitude, gives an
        inconsistent scale and breaks down near the poles).'''
        pixel_width = ground_width / float(width)
        coslat = max(1.0e-9, cos(radians(lat)))
        if zoom is None:
            zooms = range(self.min_zoom, self.max_zoom+1)
        else:
            zooms = [zoom]
        scale = 1.0
        for zoom in zooms:
            world_tiles = 1 << zoom
            # metres per native tile pixel at this latitude
            tile_pixel_width = (2*pi*mp_util.radius_of_earth*coslat) / (world_tiles * TILES_WIDTH)
            scale = pixel_width / tile_pixel_width
            if scale >= 1.0:
                break
        return (zoom, scale)

    def area_tile_grid(self, lat, lon, width, height, ground_width, zoom=None):
        '''return (zoom, scale, tile_min, count_x, count_y) describing the
        grid of tiles covering an area. lat/lon is the top left corner.'''
        (zoom, scale) = self.choose_zoom(lat, lon, width, ground_width, zoom)
        tile_min = self.coord_to_tile(lat, lon, zoom)

        # native (full tile resolution) size of the area
        nat_w = max(1, int(round(width * scale)))
        nat_h = max(1, int(round(height * scale)))
        count_x = (nat_w + tile_min.offsetx + TILES_WIDTH - 1) // TILES_WIDTH
        count_y = (nat_h + tile_min.offsety + TILES_HEIGHT - 1) // TILES_HEIGHT
        return (zoom, scale, tile_min, nat_w, nat_h, count_x, count_y)

    def area_to_tile_list(self, lat, lon, width, height, ground_width, zoom=None):
        '''return a list of TileInfoScaled objects needed for
        an area of land, with ground_width in meters, and
        width/height in pixels.

        lat/lon is the top left corner. If unspecified, the
        zoom is automatically chosen to avoid having to grow
        the tiles
        '''
        (zoom, scale, tile_min, nat_w, nat_h,
         count_x, count_y) = self.area_tile_grid(lat, lon, width, height, ground_width, zoom)
        world_tiles = 1 << zoom

        ret = []
        for iy in range(count_y):
            ty = tile_min.y + iy
            if ty < 0 or ty >= world_tiles:
                continue
            # screen placement derived independently per tile (no drift).
            # src/dst are both in screen pixels, matching scaled_tile().
            edge_y = (iy * TILES_HEIGHT - tile_min.offsety) / scale
            srcy = int(round(max(0.0, -edge_y)))
            dsty = int(round(max(0.0, edge_y)))
            for ix in range(count_x):
                tx = (tile_min.x + ix) % world_tiles
                edge_x = (ix * TILES_WIDTH - tile_min.offsetx) / scale
                srcx = int(round(max(0.0, -edge_x)))
                dstx = int(round(max(0.0, edge_x)))
                ret.append(TileInfoScaled((tx, ty), zoom, scale,
                                          (srcx, srcy), (dstx, dsty), self.service))
        return ret

    def area_to_image(self, lat, lon, width, height, ground_width, zoom=None, ordered=True):
        '''return an RGB image for an area of land, with ground_width
        in meters, and width/height in pixels.

        lat/lon is the top left corner. The zoom level is chosen
        automatically.

        Tiles are composited at full tile resolution into a single mosaic
        which is then resized once to the requested size. Doing the resize
        once (instead of per tile) avoids the rounding seams and breakup
        that appeared when zoomed a long way out.'''
        (zoom, scale, tile_min, nat_w, nat_h,
         count_x, count_y) = self.area_tile_grid(lat, lon, width, height, ground_width, zoom)
        world_tiles = 1 << zoom

        mosaic = np.zeros((nat_h, nat_w, 3), np.uint8)

        # collect the tiles and their position in the native mosaic
        tiles = []
        for iy in range(count_y):
            ty = tile_min.y + iy
            if ty < 0 or ty >= world_tiles:
                continue
            for ix in range(count_x):
                tx = (tile_min.x + ix) % world_tiles
                tiles.append((ix, iy, TileInfo((tx, ty), zoom, self.service)))

        # request the tiles nearest the middle last so they get the most
        # recent download request and are fetched first
        if ordered:
            (midlat, midlon) = self.coord_from_area(width/2, height/2, lat, lon, width, ground_width)
            tiles.sort(key=lambda t: t[2].distance(midlat, midlon), reverse=True)

        for (ix, iy, tinfo) in tiles:
            tile_img = self.load_tile(tinfo)
            if tile_img is None:
                continue
            if tile_img.shape[0] != TILES_HEIGHT or tile_img.shape[1] != TILES_WIDTH:
                tile_img = cv2.resize(tile_img, (TILES_WIDTH, TILES_HEIGHT))
            dst_left = ix * TILES_WIDTH - tile_min.offsetx
            dst_top = iy * TILES_HEIGHT - tile_min.offsety
            sx0 = max(0, -dst_left)
            sy0 = max(0, -dst_top)
            dx0 = max(0, dst_left)
            dy0 = max(0, dst_top)
            w = min(TILES_WIDTH - sx0, nat_w - dx0)
            h = min(TILES_HEIGHT - sy0, nat_h - dy0)
            if w <= 0 or h <= 0:
                continue
            mosaic[dy0:dy0+h, dx0:dx0+w] = tile_img[sy0:sy0+h, sx0:sx0+w]

        # resize the assembled mosaic to the requested size in one operation
        if (nat_w, nat_h) != (width, height):
            interp = cv2.INTER_AREA if nat_w >= width else cv2.INTER_LINEAR
            img = cv2.resize(mosaic, (width, height), interpolation=interp)
        else:
            img = mosaic

        # return as an RGB image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img


def mp_icon(filename):
    '''load an icon from the data directory'''
    # we have to jump through a lot of hoops to get an OpenCV image
    # when we may be in a package zip file
    raw = None
    try:
        import importlib.resources
        package = __package__ + ".data"
        if __name__ == "__main__":
            package = "MAVProxy.modules.mavproxy_map.data"
        with importlib.resources.open_binary(package, filename) as stream:
            raw = np.frombuffer(stream.read(), dtype=np.uint8)
    except Exception:
        with open(os.path.join(os.path.dirname(__file__), 'data', filename)).read() as stream:
            raw = np.frombuffer(stream.read(), dtype=np.uint8)
    img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    return img


if __name__ == "__main__":

    from optparse import OptionParser
    parser = OptionParser("mp_tile.py [options]")
    parser.add_option("--lat", type='float', default=-35.362938, help="start latitude")
    parser.add_option("--lon", type='float', default=149.165085, help="start longitude")
    parser.add_option("--width", type='float', default=1000.0, help="width in meters")
    parser.add_option("--service", default="MicrosoftSat", help="tile service")
    parser.add_option("--zoom", default=None, type='int', help="zoom level")
    parser.add_option("--max-zoom", type='int', default=19, help="maximum tile zoom")
    parser.add_option("--delay", type='float', default=1.0, help="tile download delay")
    parser.add_option("--boundary", default=None, help="region boundary")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    (opts, args) = parser.parse_args()

    lat = opts.lat
    lon = opts.lon
    ground_width = opts.width

    if opts.boundary:
        boundary = mp_util.polygon_load(opts.boundary)
        bounds = mp_util.polygon_bounds(boundary)
        lat = bounds[0]+bounds[2]
        lon = bounds[1]
        ground_width = max(mp_util.gps_distance(lat, lon, lat, mp_util.wrap_180(lon+bounds[3])),
                           mp_util.gps_distance(lat, lon, lat-bounds[2], lon))
        print(lat, lon, ground_width)

    mt = MPTile(
        debug=opts.debug,
        service=opts.service,
        tile_delay=opts.delay,
        max_zoom=opts.max_zoom,
    )
    if opts.zoom is None:
        zooms = range(mt.min_zoom, mt.max_zoom+1)
    else:
        zooms = [opts.zoom]
    for zoom in zooms:
        tlist = mt.area_to_tile_list(lat, lon, width=1024, height=1024,
                                     ground_width=ground_width, zoom=zoom)
        print("zoom %u needs %u tiles" % (zoom, len(tlist)))
        for tile in tlist:
            mt.load_tile(tile)
        while mt.tiles_pending() > 0:
            time.sleep(2)
            print("Waiting on %u tiles" % mt.tiles_pending())
    print('Done')
