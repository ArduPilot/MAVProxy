#!/usr/bin/python
'''
Wrapper for the SRTM module (srtm.py)
It will grab the altitude of a long,lat pair from the SRTM database
Created by Stephen Dade (stephen_dade@hotmail.com)
'''

import numpy, os, time, sys, srtm

class ElevationModel():
    '''Elevation Model. Only SRTM for now'''

    def __init__(self):
        self.downloader = srtm.SRTMDownloader()
        self.downloader.loadFileList()
        self.tileDict = dict()

    def GetElevation(self, latitude, longitude):
        '''Returns the altitude (m ASL) of a given lat/long pair'''
        if latitude == 0 or longitude == 0:
            return 0
        TileID = (numpy.floor(latitude), numpy.floor(longitude))
        if TileID in self.tileDict:
            alt = self.tileDict[TileID].getAltitudeFromLatLon(latitude, longitude)
        else:
            tile = self.downloader.getTile(numpy.floor(latitude), numpy.floor(longitude))
            self.tileDict[TileID] = tile
            alt = tile.getAltitudeFromLatLon(latitude, longitude)

        return alt


if __name__ == "__main__":

    from optparse import OptionParser
    parser = OptionParser("mp_elevation.py [options]")
    parser.add_option("--lat", type='float', default=-35.362938, help="start latitude")
    parser.add_option("--lon", type='float', default=149.165085, help="start longitude")

    (opts, args) = parser.parse_args()

    EleModel = ElevationModel()

    lat = opts.lat
    lon = opts.lon

    '''Do a few lat/long pairs to demonstrate the caching'''
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon)
    t1 = time.time()
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))

    lat = opts.lat+0.001
    lon = opts.lon+0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon)
    t1 = time.time()
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))

    lat = opts.lat-0.001
    lon = opts.lon-0.001
    t0 = time.time()
    alt = EleModel.GetElevation(lat, lon)
    t1 = time.time()
    print("Altitude at (%.6f, %.6f) is %u m. Pulled at %.1f FPS" % (lat, lon, alt, 1/(t1-t0)))



