#!/usr/bin/env python
'''
Module to read DTM files published by Geoscience Australia
Written by Stephen Dade (stephen_dade@hotmail.com
'''

import os
import sys

import numpy


class ERMap:
    '''Class to read GA files'''
    def __init__(self):
        self.header = None
        self.data = None
        self.startlongitude = 0
        self.startlatitude = 0
        self.endlongitude = 0
        self.endlatitude = 0
        self.deltalongitude = 0
        self.deltalatitude = 0

    def read_ermapper(self, ifile):
        '''Read in a DEM file and associated .ers file'''
        ers_index = ifile.find('.ers')
        if ers_index > 0:
            data_file = ifile[0:ers_index]
            header_file = ifile
        else:
            data_file = ifile
            header_file = ifile + '.ers'

        self.header = self.read_ermapper_header(header_file)

        nroflines = int(self.header['nroflines'])
        nrofcellsperlines = int(self.header['nrofcellsperline'])
        self.data = self.read_ermapper_data(data_file, offset=int(self.header['headeroffset']))
        self.data = numpy.reshape(self.data,(nroflines,nrofcellsperlines))

        longy =  numpy.fromstring(self.getHeaderParam('longitude'), sep=':')
        latty =  numpy.fromstring(self.getHeaderParam('latitude'), sep=':')

        self.deltalatitude = float(self.header['ydimension'])
        self.deltalongitude = float(self.header['xdimension'])

        if longy[0] < 0:
            self.startlongitude = longy[0]+-((longy[1]/60)+(longy[2]/3600))
            self.endlongitude = self.startlongitude - int(self.header['nrofcellsperline'])*self.deltalongitude
        else:
            self.startlongitude = longy[0]+(longy[1]/60)+(longy[2]/3600)
            self.endlongitude = self.startlongitude + int(self.header['nrofcellsperline'])*self.deltalongitude
        if latty[0] < 0:
            self.startlatitude = latty[0]-((latty[1]/60)+(latty[2]/3600))
            self.endlatitude = self.startlatitude - int(self.header['nroflines'])*self.deltalatitude
        else:
            self.startlatitude = latty[0]+(latty[1]/60)+(latty[2]/3600)
            self.endlatitude = self.startlatitude + int(self.header['nroflines'])*self.deltalatitude

    def read_ermapper_header(self, ifile):
        # function for reading an ERMapper header from file
        header = {}

        fid = open(ifile,'rt')
        header_string = fid.readlines()
        fid.close()

        for line in header_string:
            if line.find('=') > 0:
                tmp_string = line.strip().split('=')
                header[tmp_string[0].strip().lower()]= tmp_string[1].strip()

        return header

    def read_ermapper_data(self, ifile, data_format = numpy.float32, offset=0):
        # open input file in a binary format and read the input string
        fid = open(ifile,'rb')
        if offset != 0:
            fid.seek(offset)
        input_string = fid.read()
        fid.close()

        # convert input string to required format (Note default format is numpy.float32)
        grid_as_float = numpy.fromstring(input_string,data_format)
        return grid_as_float

    def getHeaderParam(self, key):
         '''Find a parameter in the associated .ers file'''
         return self.header[key]

    def printBoundingBox(self):
        '''Print the bounding box that this DEM covers'''
        print("Bounding Latitude: ")
        print(self.startlatitude)
        print(self.endlatitude)

        print("Bounding Longitude: ")
        print(self.startlongitude)
        print(self.endlongitude)

    def getPercentBlank(self):
        '''Print how many null cells are in the DEM - Quality measure'''
        blank = 0
        nonblank = 0
        for x in self.data.flat:
            if x == -99999.0:
                blank = blank + 1
            else:
                nonblank = nonblank + 1

        print("Blank tiles =  ", blank, "out of ", (nonblank+blank))

    def getAltitudeAtPoint(self, latty, longy):
        '''Return the altitude at a particular long/lat'''
        #check the bounds
        if self.startlongitude > 0 and (longy < self.startlongitude or longy > self.endlongitude):
            return -1
        if self.startlongitude < 0 and (longy > self.startlongitude or longy < self.endlongitude):
            return -1
        if self.startlatitude > 0 and (latty < self.startlatitude or longy > self.endlatitude):
            return -1
        if self.startlatitude < 0 and (latty > self.startlatitude or longy < self.endlatitude):
            return -1

        x = numpy.abs((latty - self.startlatitude)/self.deltalatitude)
        y = numpy.abs((longy - self.startlongitude)/self.deltalongitude)

        #do some interpolation
        # print("x,y", x, y)
        x_int = int(x)
        x_frac = x - int(x)
        y_int = int(y)
        y_frac = y - int(y)
        #print("frac", x_int, x_frac, y_int, y_frac)
        value00 = self.data[x_int, y_int]
        value10 = self.data[x_int+1, y_int]
        value01 = self.data[x_int, y_int+1]
        value11 = self.data[x_int+1, y_int+1]
        #print("values ", value00, value10, value01, value11)

        #check for null values
        if value00 == -99999:
            value00 = 0
        if value10 == -99999:
            value10 = 0
        if value01 == -99999:
            value01 = 0
        if value11 == -99999:
            value11 = 0

        value1 = self._avg(value00, value10, x_frac)
        value2 = self._avg(value01, value11, x_frac)
        value  = self._avg(value1,  value2, y_frac)

        return value

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



if __name__ == '__main__':

    print("./Canberra/GSNSW_P756demg")
    mappy = ERMap()
    mappy.read_ermapper(os.path.join(os.environ['HOME'], './Documents/Elevation/Canberra/GSNSW_P756demg'))

    #print some header data
    mappy.printBoundingBox()

    #get a measure of data quality
    #mappy.getPercentBlank()

    #test the altitude (around Canberra):
    alt = mappy.getAltitudeAtPoint(-35.274411, 149.097504)
    print("Alt at (-35.274411, 149.097504) is 807m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.239648, 149.126118)
    print("Alt at (-35.239648, 149.126118) is 577m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.362751, 149.165361)
    print("Alt at (-35.362751, 149.165361) is 584m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.306992, 149.194274)
    print("Alt at (-35.306992, 149.194274) is 570m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.261612, 149.542091)
    print("Alt at (-35.261612, 149.542091) is 766m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.052544, 149.509165)
    print("Alt at (-35.052544, 149.509165) is 700m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.045126, 149.257482)
    print("Alt at (-35.045126, 149.257482) is 577m (Google) or " + str(alt))
    alt = mappy.getAltitudeAtPoint(-35.564044, 149.177657)
    print("Alt at (-35.564044, 149.177657) is 1113m (Google) or " + str(alt))




