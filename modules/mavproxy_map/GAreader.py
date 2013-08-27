#!/usr/bin/env python
'''
Module to read DTM files published by Geoscience Australia
Written by Stephen Dade (stephen_dade@hotmail.com
Chunks are downloaded in 240x240 pixel tiffs, 5x5min (9.3x9.3km) geo area
Thus each pixel is ~39x39m
File format will be 115124N145123W.tiff, where the top left corner is 115.123N, 145.123W
'''

import os
import threading
import time
import urllib2
import Image

import numpy

class downloadThread (threading.Thread):
    '''simple download class thread'''
    def __init__(self, name, site, file):
        threading.Thread.__init__(self)
        self.name = name
        self.site = site
        self.file = file
    def run(self):
        print "Starting " + self.name + "\n"
        with open(self.file, 'wb+') as f:
            try:
                f.write(urllib2.urlopen(self.site).read())
            except urllib2.HTTPError:
                print "Download fail"
        print "Downloaded " + self.name
            
class ERMap:
    '''Class to read GA files'''
    def __init__(self, cachedir=None, offline=0):
    #set up the cache directory
        if cachedir is None:
            try:
                cachedir = os.path.join(os.environ['HOME'], '.tilecache/GeoAus')
            except Exception:
                #cachedir = os.path.join(tempfile.gettempdir(), 'GeoAus')
                cachedir = "C:\GAtmp"

        self.offline = offline
        self.first_failure = False
        self.server = "http://www.ga.gov.au/gisimg/services/topography/dem_1s/ImageServer/WMSServer?"
        self.cachedir = cachedir
        self.imgpixelsize = 240
        self.imggeosize = 0.05
        self.downloadThreads = []
        '''print "SRTMDownloader - server= %s, directory=%s." % \
        (self.server, self.directory)'''
        if not os.path.exists(cachedir):
            mp_util.mkdir_p(cachedir)

    def getAltitudeAtPoint(self, latty, longy):
        '''Return the altitude at a particular long/lat'''
        #Check we're actually in Australia:
        if not (latty < -10 and latty > -40 and longy > 113 and longy < 154):
            print "Outside GeoAustralia DTED range"
            return 0
        
        #find out which chunk of data has this point 115124N145123W.tiff
        # print "x,y", x, y
        x_id = self.positionToTile(latty, 1)
        y_id = self.positionToTile(longy, 1)
        data_chunk = "" + str(x_id) + "N" + str(y_id) + "W.tiff"
        
        #does this file already exist in the cache?
        try:
            data = open(os.path.join(self.cachedir, data_chunk), 'r')
        except IOError:
            #if it's not in the cache, make a new thread and download
            print "Need to download " + data_chunk
            if self.offline == 0:
                self.downloadFile(x_id, y_id)
            return -1
        
        #open up the file
        #print "Opening " + str(os.path.join(self.cachedir, data_chunk))
        im = Image.open(os.path.join(self.cachedir, data_chunk))
        #print im.format, im.size, im.mode
        pixelMap = im.load() #create the pixel map
        
        #figure out pixel indexes        
        x = ((latty - self.positionToTile(latty, 0))/self.imggeosize) * self.imgpixelsize
        y = ((longy - self.positionToTile(longy, 0))/self.imggeosize) * self.imgpixelsize
        #print "long, lat" , latty, longy
        #print "x,y ", x, y
        x_int = int(x)
        x_frac = x - int(x)
        y_int = int(y)
        y_frac = y - int(y)
        
        #print "frac", x_int, x_frac, y_int, y_frac
        value00 = (pixelMap[x_int, y_int])
        value10 = (pixelMap[x_int+1, y_int])
        value01 = (pixelMap[x_int, y_int+1])
        value11 = (pixelMap[x_int+1, y_int+1])
        #print "values ", value00, value10, value01, value11

        value1 = self._avg(value00, value10, x_frac)
        value2 = self._avg(value01, value11, x_frac)
        value  = self._avg(value1,  value2, y_frac)

        return numpy.round(value, 2)

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

    def downloadFile(self, x_int, y_int):
        '''Downloads the file from the GeoScience Australia servers in
        a new thread'''
        for curDown in self.downloadThreads:
            if curDown == str(x_int) + ", " + str(y_int):
                return
        #http://www.ga.gov.au/gisimg/services/topography/dem_1s/ImageServer/WMSServer?
        #request=GetMap&format=image/tiff&crs=CRS:84&width=640&height=480&bbox=151.666,-26.666,152.0,-26.5 
        webaddress = self.server + "request=GetMap&format=image/tiff&crs=CRS:84&width=" + str(self.imgpixelsize) + "&height=" + str(self.imgpixelsize)
        webaddress += "&bbox=" + str(float(y_int)/1000) + "," + str(float(x_int)/1000) + "," + str((float(y_int)/1000)+self.imggeosize) + "," + str((float(x_int)/1000)+self.imggeosize)
        filename = os.path.join(self.cachedir, "" + str(x_int) + "N" + str(y_int) + "W.tiff")
        thread1 = downloadThread(str(x_int) + ", " + str(y_int), webaddress, filename)
        self.downloadThreads.append(thread1)
        print "Site is " + webaddress
        print "Downloading " + filename
        thread1.start()
        
    def positionToTile(self, posn, tilecoords):
        roundpos = (numpy.floor(posn / self.imggeosize) * self.imggeosize)
        if tilecoords:
            return int(roundpos*1000)
        else:
            return roundpos


if __name__ == '__main__':

    mappy = ERMap()

    #get a measure of data quality
    #mappy.getPercentBlank()
    print "Note that script will need to be run twice to download and read cached tiles"

    #test the altitude (around Canberra):
    alt = mappy.getAltitudeAtPoint(-35.274411, 149.097504)
    print "Alt at (-35.274411, 149.097504) is 807m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.239648, 149.126118)
    print "Alt at (-35.239648, 149.126118) is 577m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.362751, 149.165361)
    print "Alt at (-35.362751, 149.165361) is 584m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.306992, 149.194274)
    print "Alt at (-35.306992, 149.194274) is 570m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.261612, 149.542091)
    print "Alt at (-35.261612, 149.542091) is 766m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.052544, 149.509165)
    print "Alt at (-35.052544, 149.509165) is 700m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.045126, 149.257482)
    print "Alt at (-35.045126, 149.257482) is 577m (Google) or " + str(alt)
    alt = mappy.getAltitudeAtPoint(-35.564044, 149.177657)
    print "Alt at (-35.564044, 149.177657) is 1113m (Google) or " + str(alt)




