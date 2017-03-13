#!/usr/bin/env python
#***************************************************************************
#                      Copyright 2017 Jaime Machuca
#***************************************************************************
# Title        : sc_ExifWriter.py
#
# Description  : This file is used to write the geotag informaton into the
#                EXIF metadata of a picture file. The file was originaly written
#                by Escadrone and distributed under Apache V2.0 license which is
#                retained by this file. This file may be modified from the original
#                in order to make it compatible with this MAVProxy Module.
#                The original file can be found here:
#
#                https://github.com/Escadrone/Solo-Mapper.git
#
# Environment  : Python 2.7 Code. Intended to be included in MAVproxy as a Module
#
# Responsible  : Jaime Machuca
#
# License      : Licensed under the Apache License, Version 2.0 (the "License");
#                you may not use this file except in compliance with the License.
#                You may obtain a copy of the License at
#
#                http://www.apache.org/licenses/LICENSE-2.0
#
#                Unless required by applicable law or agreed to in writing, software
#                distributed under the License is distributed on an "AS IS" BASIS,
#                WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#                See the License for the specific language governing permissions and
#                limitations under the License.
#
# Editor Used  : Xcode 6.1.1 (6A2008a)
#
#****************************************************************************

#****************************************************************************
# HEADER-FILES (Only those that are needed in this file)
#****************************************************************************

# System Header files and Module Headers

# Module Dependent Headers
import piexif

# Own Headers

#****************************************************************************
# LOCAL DEFINES
#****************************************************************************

#****************************************************************************
# Class name       : ExifWriter
#
# Public Methods   : write_gps
#
# Private Methods  : _decdeg2dms
#
#****************************************************************************

class ExifWriter:

#****************************************************************************
#   Method Name     : write_gps
#
#   Description     : Writes GPS coordinates into the given pictures EXIF
#
#   Parameters      : filename      Path to the image file
#                     lat           Latitude in decimal degrees to write
#                     longit        Longitude in decimal degrees to write
#                     altitude      Altitude above mean sea level in meters
#
#   Return Value    : None
#
#   Author           : Gael Billon, Escadrone
#
#****************************************************************************

    @staticmethod
    def write_gps(filename, lat, longit, altitude): #lat and long are decimals
        gps = {} #create array

        #Altitude
        if altitude<0:
            gps[piexif.GPSIFD.GPSAltitudeRef] = 1; #Below see level
        else:
            gps[piexif.GPSIFD.GPSAltitudeRef] = 0; #Above see level
            gps[piexif.GPSIFD.GPSAltitude]= (int(abs(altitude)*100),100)
  
        #Latitude
        if lat<0:
            gps[piexif.GPSIFD.GPSLatitudeRef] = "S"
        else:
            gps[piexif.GPSIFD.GPSLatitudeRef] = "N"
   
        latd,latm,lats = ExifWriter._decdeg2dms(lat)
        gps[piexif.GPSIFD.GPSLatitude] = [(latd,1),(latm,1),(lats*100,100)];
  
        #Longitude
        if longit<0:
            gps[piexif.GPSIFD.GPSLongitudeRef] = "W"
        else:
            gps[piexif.GPSIFD.GPSLongitudeRef] = "E"
  
        longd,longm,longs = ExifWriter._decdeg2dms(longit)
        gps[piexif.GPSIFD.GPSLongitude] = [(longd,1),(longm,1),(longs*100,100)];

        exifdict =  piexif.load(filename)
        exifdict["GPS"] = gps

        exif_bytes = piexif.dump(exifdict)
        piexif.insert(exif_bytes, filename)
        exifdict = piexif.load(filename)

#****************************************************************************
#   Method Name     : _decdeg2dms
#
#   Description     : Converts decimal degrees to Degrees, Minutes, seconds
#
#   Parameters      : dd    Coordinate in decimal degrees
#
#   Return Value    : None
#
#   Author           : Gael Billon, Escadrone
#
#****************************************************************************

    #method converting decimal format to DMS
    @staticmethod
    def _decdeg2dms(dd):
        dd = abs(dd)
        minutes,seconds = divmod(dd*3600,60)
        degrees,minutes = divmod(minutes,60)
        return (degrees,minutes,seconds)

 
