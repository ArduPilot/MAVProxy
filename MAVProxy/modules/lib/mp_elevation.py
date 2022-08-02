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

from MAVProxy.modules.lib import srtm

# SRTM1 = 1 arc-second resolution data (~30m)
# SRTM3 = 3 arc-second resolution data (~90m)
TERRAIN_SERVICES = {
    "SRTM1"      : ("terrain.ardupilot.org", "SRTM1"),
    "SRTM3"      : ("terrain.ardupilot.org", "SRTM3")
}

class ElevationModel():
    '''Elevation Model. Only SRTM for now'''

    def __init__(self, database='SRTM3', offline=0, debug=False, cachedir=None):
        '''Use offline=1 to disable any downloading of tiles, regardless of whether the
        tile exists'''
        if database.lower() == 'srtm':
            # compatibility with the old naming
            database = "SRTM3"
        self.database = database
        if self.database in ['SRTM1', 'SRTM3']:
            self.downloader = srtm.SRTMDownloader(offline=offline, debug=debug, directory=self.database, cachedir=cachedir)
            self.downloader.loadFileList()
            self.tileDict = dict()
        elif self.database == 'geoscience':
            '''Use the Geoscience Australia database instead - watch for the correct database path'''
            from MAVProxy.modules.mavproxy_map import GAreader
            self.mappy = GAreader.ERMap()
            self.mappy.read_ermapper(os.path.join(os.environ['HOME'], './Documents/Elevation/Canberra/GSNSW_P756demg'))
        else:
            print("Error: Bad terrain source " + database)
            self.database = None

    def GetElevation(self, latitude, longitude, timeout=0):
        '''Returns the altitude (m ASL) of a given lat/long pair, or None if unknown'''
        if latitude is None or longitude is None:
            return None
        if self.database in ['SRTM1', 'SRTM3']:
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
        elif self.database == 'geoscience':
             alt = self.mappy.getAltitudeAtPoint(latitude, longitude)
        else:
            return None
        return alt


if __name__ == "__main__":

    from argparse import ArgumentParser
    parser = ArgumentParser("mp_elevation.py [options]")
    parser.add_argument("--lat", type=float, default=-35.052544, help="start latitude")
    parser.add_argument("--lon", type=float, default=149.509165, help="start longitude")
    parser.add_argument("--database", type=str, default='SRTM3', help="elevation database", choices=["SRTM1", "SRTM3"])
    parser.add_argument("--debug", action='store_true', help="enabled debugging")

    args = parser.parse_args()

    EleModel = ElevationModel(args.database, debug=args.debug)

    lat = args.lat
    lon = args.lon

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

    lat = args.lat+0.001
    lon = args.lon+0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon, timeout=10)
    t1 = time.time()+.000001
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))

    lat = args.lat-0.001
    lon = args.lon-0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon, timeout=10)
    t1 = time.time()+.000001
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))



