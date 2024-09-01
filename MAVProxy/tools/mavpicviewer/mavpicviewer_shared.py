#!/usr/bin/env python3

'''
MAV Picture Viewer Shared Classes

Quick and efficient reviewing of images taken from a drone

AP_FLAKE8_CLEAN
'''

import os


# defintions for RGB colours
RGB_YELLOW = (255, 255, 0)
RGB_RED = (255, 0, 0)
RGB_GREEN = (0, 255, 0)
RGB_BLUE = (0, 0, 255)


# Loc (aka Location) class holds lat, lon, alt
class Loc:
    def __init__(self, lat, lon, alt, terr_alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.terr_alt = terr_alt


# Pos (aka Position) class holds X, Y pixel coordinates
class Pos:
    def __init__(self, X, Y):
        self.X = X
        self.Y = Y


# POI (aka Point of Interest) class holds two positions and two locations
class POI:
    def __init__(self, pos1, pos2, loc1, loc2):
        self.pos1 = pos1
        self.pos2 = pos2
        self.loc1 = loc1
        self.loc2 = loc2


# Tempclass holds Temp Max, X, Y pixel coordinates
class TempAndPos:
    def __init__(self, temp_max, temp_pos_x, temp_pos_y):
        self.temp_max = temp_max
        self.temp_pos_x = temp_pos_x
        self.temp_pos_y = temp_pos_y


# return an array of files for a given directory and extension
def get_file_list(directory, extensions):
    '''return file list for a directory'''
    flist = []
    for filename in os.listdir(directory):
        extension = filename.split('.')[-1]
        if extension.lower() in extensions:
            flist.append(os.path.join(directory, filename))
    sorted_list = sorted(flist, key=str.lower)
    return sorted_list


# communication objects for mosaic and image viewer
# set the file list (a dictionary of paths to images)
class SetFilelist:
    def __init__(self, filelist):
        self.filelist = filelist


# set the current file number (aka current image being viewed)
class SetFilenumber:
    def __init__(self, filenumber):
        self.filenumber = filenumber


# set the FOV in degrees
class SetFOV:
    def __init__(self, fov):
        self.fov = fov


# set the Yaw in degrees
class SetYaw:
    def __init__(self, yaw):
        self.yaw = yaw


# close the application
class Close:
    def __init__(self):
        pass


# set the Image's location (lat, lon, alt, terr_alt)
class SetImageLoc:
    def __init__(self, filenumber, loc):
        self.filenumber = filenumber
        self.loc = loc


# set POI for the given filenumber
class SetPOI:
    def __init__(self, filenumber, poi):
        self.filenumber = filenumber
        self.poi = poi


# clear POI for the given filenumber
class ClearPOI:
    def __init__(self, filenumber):
        self.filenumber = filenumber


# clear ALL POIs for all images
class ClearAllPOI:
    def __init__(self):
        pass


# request temperatures from all images
class GetTempForAllImages:
    def __init__(self):
        pass


# set Temp data for the given filenumber
class SetTempAndPos:
    def __init__(self, filenumber, temp_max, temp_pos_x, temp_pos_y):
        self.filenumber = filenumber
        self.temp_max = temp_max
        self.temp_pos_x = temp_pos_x
        self.temp_pos_y = temp_pos_y
