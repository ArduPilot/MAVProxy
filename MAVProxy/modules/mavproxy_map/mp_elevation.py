#!/usr/bin/python
'''
Wrapper for the SRTM module (srtm.py)
It will grab the altitude of a long,lat pair from the SRTM database
Created by Stephen Dade (stephen_dade@hotmail.com)
'''

import os
import sys
import time

import numpy

from MAVProxy.modules.mavproxy_map import srtm

class ElevationModel():
    '''Elevation Model. Only SRTM for now'''

    def __init__(self, database='srtm', offline=0, debug=False):
        '''Use offline=1 to disable any downloading of tiles, regardless of whether the
        tile exists'''
        self.database = database
        if self.database == 'srtm':
            self.downloader = srtm.SRTMDownloader(offline=offline, debug=debug)
            self.downloader.loadFileList()
            self.tileDict = dict()

        '''Use the Geoscience Australia database instead - watch for the correct database path'''
        if self.database == 'geoscience':
            from MAVProxy.modules.mavproxy_map import GAreader
            self.mappy = GAreader.ERMap()
            self.mappy.read_ermapper(os.path.join(os.environ['HOME'], './Documents/Elevation/Canberra/GSNSW_P756demg'))

    def GetElevation(self, latitude, longitude, timeout=0):
        '''Returns the altitude (m ASL) of a given lat/long pair, or None if unknown'''
        if latitude is None or longitude is None:
            return None
        if self.database == 'srtm':
            TileID = (numpy.floor(latitude), numpy.floor(longitude))
            if TileID in self.tileDict:
                alt = self.tileDict[TileID].getAltitudeFromLatLon(latitude, longitude)
            else:
                tile = self.downloader.getTile(numpy.floor(latitude), numpy.floor(longitude))
                if tile == 0:
                    if timeout > 0:
                        t0 = time.time()
                        while time.time() < t0+timeout and tile == 0:
                            tile = self.downloader.getTile(numpy.floor(latitude), numpy.floor(longitude))
                            if tile == 0:
                                time.sleep(0.1)
                if tile == 0:
                    return None
                self.tileDict[TileID] = tile
                alt = tile.getAltitudeFromLatLon(latitude, longitude)
        if self.database == 'geoscience':
             alt = self.mappy.getAltitudeAtPoint(latitude, longitude)
        return alt


if __name__ == "__main__":

    from optparse import OptionParser
    parser = OptionParser("mp_elevation.py [options]")
    parser.add_option("--lat", type='float', default=-35.052544, help="start latitude")
    parser.add_option("--lon", type='float', default=149.509165, help="start longitude")
    parser.add_option("--database", type='string', default='srtm', help="elevation database")
    parser.add_option("--debug", action='store_true', help="enabled debugging")

    (opts, args) = parser.parse_args()

    EleModel = ElevationModel(opts.database, debug=opts.debug)

    lat = opts.lat
    lon = opts.lon

    '''Do a few lat/long pairs to demonstrate the caching
    Note the +0.000001 to the time. On faster PCs, the two time periods
    may in fact be equal, so we add a little extra time on the end to account for this'''
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon, timeout=10)
    if alt is None:
        print("Tile not available")
        sys.exit(1)
    t1 = time.time()+.000001
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))

    lat = opts.lat+0.001
    lon = opts.lon+0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon, timeout=10)
    t1 = time.time()+.000001
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))

    lat = opts.lat-0.001
    lon = opts.lon-0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon, timeout=10)
    t1 = time.time()+.000001
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))



