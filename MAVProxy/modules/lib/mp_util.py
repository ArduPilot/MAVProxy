#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''common mavproxy utility functions'''

import gzip
import math
import os
import sys
import platform
import warnings

# Some platforms (CYGWIN and others) many not have the wx library
# use imp to see if wx is on the path
has_wxpython = False

if platform.system() == 'Windows':
    # auto-detection is failing on windows, for an unknown reason
    has_wxpython = True
else:
    import imp
    try:
        imp.find_module('wx')
        has_wxpython = True
    except ImportError as e:
        pass

radius_of_earth = 6378100.0 # in meters

def gps_distance(lat1, lon1, lat2, lon2):
    '''return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
    return radius_of_earth * c


def gps_bearing(lat1, lon1, lat2, lon2):
    '''return bearing between two points in degrees, in range 0-360
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    if bearing < 0:
        bearing += 360.0
    return bearing


def wrap_valid_longitude(lon):
    ''' wrap a longitude value around to always have a value in the range
        [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
    '''
    return (((lon + 180.0) % 360.0) - 180.0)

def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    '''
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = math.radians(bearing)
    dr = distance/radius_of_earth

    lat2 = math.asin(math.sin(lat1)*math.cos(dr) +
                     math.cos(lat1)*math.sin(dr)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dr)*math.cos(lat1),
                             math.cos(dr)-math.sin(lat1)*math.sin(lat2))
    return (math.degrees(lat2), wrap_valid_longitude(math.degrees(lon2)))

def gps_offset(lat, lon, east, north):
    '''return new lat/lon after moving east/north
    by the given number of meters'''
    bearing = math.degrees(math.atan2(east, north))
    distance = math.sqrt(east**2 + north**2)
    return gps_newpos(lat, lon, bearing, distance)


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/") or dir.endswith("\\"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    try:
        os.mkdir(dir)
    except Exception:
        pass

def polygon_load(filename):
    '''load a polygon from a file'''
    ret = []
    f = open(filename)
    for line in f:
        if line.startswith('#'):
            continue
        line = line.strip()
        if not line:
            continue
        a = line.split()
        if len(a) != 2:
            raise RuntimeError("invalid polygon line: %s" % line)
        ret.append((float(a[0]), float(a[1])))
    f.close()
    return ret


def polygon_bounds(points):
    '''return bounding box of a polygon in (x,y,width,height) form'''
    (minx, miny) = (points[0][0], points[0][1])
    (maxx, maxy) = (minx, miny)
    for p in points:
        minx = min(minx, p[0])
        maxx = max(maxx, p[0])
        miny = min(miny, p[1])
        maxy = max(maxy, p[1])
    return (minx, miny, maxx-minx, maxy-miny)

def bounds_overlap(bound1, bound2):
    '''return true if two bounding boxes overlap'''
    (x1,y1,w1,h1) = bound1
    (x2,y2,w2,h2) = bound2
    if x1+w1 < x2:
        return False
    if x2+w2 < x1:
        return False
    if y1+h1 < y2:
        return False
    if y2+h2 < y1:
        return False
    return True


class object_container:
    '''return a picklable object from an existing object,
    containing all of the normal attributes of the original'''
    def __init__(self, object, debug=False):
        for v in dir(object):
            if not v.startswith('__') and v not in ['this', 'ClassInfo', 'EventObject']:
                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter('ignore')
                        a = getattr(object, v)
                        if (hasattr(a, '__call__') or
                            hasattr(a, '__swig_destroy__') or
                            str(a).find('Swig Object') != -1):
                            continue
                        if debug:
                            print(v, a)
                        setattr(self, v, a)
                except Exception:
                    pass

def degrees_to_dms(degrees):
    '''return a degrees:minutes:seconds string'''
    deg = int(degrees)
    min = int((degrees - deg)*60)
    sec = ((degrees - deg) - (min/60.0))*60*60
    return u'%d\u00b0%02u\'%05.2f"' % (deg, abs(min), abs(sec))


class UTMGrid:
    '''class to hold UTM grid position'''
    def __init__(self, zone, easting, northing, hemisphere='S'):
        self.zone = zone
        self.easting = easting
        self.northing = northing
        self.hemisphere = hemisphere

    def __str__(self):
        return "%s %u %u %u" % (self.hemisphere, self.zone, self.easting, self.northing)

    def latlon(self):
        '''return (lat,lon) for the grid coordinates'''
        from MAVProxy.modules.lib.ANUGA import lat_long_UTM_conversion
        (lat, lon) = lat_long_UTM_conversion.UTMtoLL(self.northing, self.easting, self.zone, isSouthernHemisphere=(self.hemisphere=='S'))
        return (lat, lon)


def latlon_to_grid(latlon):
    '''convert to grid reference'''
    from MAVProxy.modules.lib.ANUGA import redfearn
    (zone, easting, northing) = redfearn.redfearn(latlon[0], latlon[1])
    if latlon[0] < 0:
        hemisphere = 'S'
    else:
        hemisphere = 'N'
    return UTMGrid(zone, easting, northing, hemisphere=hemisphere)

def latlon_round(latlon, spacing=1000):
    '''round to nearest grid corner'''
    g = latlon_to_grid(latlon)
    g.easting = (g.easting // spacing) * spacing
    g.northing = (g.northing // spacing) * spacing
    return g.latlon()


def wxToPIL(wimg):
    '''convert a wxImage to a PIL Image'''
    from PIL import Image
    (w,h) = wimg.GetSize()
    d     = wimg.GetData()
    pimg  = Image.new("RGB", (w,h), color=1)
    try:
        pimg.frombytes(d)
    except NotImplementedError:
        # old, removed method:
        pimg.fromstring(d)
    return pimg

def PILTowx(pimg):
    '''convert a PIL Image to a wx image'''
    from MAVProxy.modules.lib.wx_loader import wx
    wimg = wx.EmptyImage(pimg.size[0], pimg.size[1])
    try:
        wimg.SetData(pimg.convert('RGB').tobytes())
    except NotImplementedError:
        # old, removed method:
        wimg.SetData(pimg.convert('RGB').tostring())
    return wimg

def dot_mavproxy(name=None):
    '''return a path to store mavproxy data'''
    if 'HOME' not in os.environ:
        dir = os.path.join(os.environ['LOCALAPPDATA'], '.mavproxy')
    else:
        dir = os.path.join(os.environ['HOME'], '.mavproxy')
    mkdir_p(dir)
    if name is None:
        return dir
    return os.path.join(dir, name)

def download_url(url):
    '''download a URL and return the content'''
    if sys.version_info.major < 3:
        from urllib2 import urlopen as url_open
        from urllib2 import URLError as url_error
    else:
        from urllib.request import urlopen as url_open
        from urllib.error import URLError as url_error
    try:
        resp = url_open(url)
        headers = resp.info()
    except url_error as e:
        print('Error downloading %s' % url)
        return None
    return resp.read()


def download_files(files):
    '''download an array of files'''
    for (url, file) in files:
        print("Downloading %s as %s" % (url, file))
        data = download_url(url)
        if data is None:
            continue
        if url.endswith(".gz") and not file.endswith(".gz"):
            # decompress it...
            data = gzip.decompress(data)
        try:
            open(file, mode='wb').write(data)
        except Exception as e:
            print("Failed to save to %s : %s" % (file, e))


child_fd_list = []

def child_close_fds():
    '''close file descriptors that a child process should not inherit.
       Should be called from child processes.'''
    global child_fd_list
    import os
    while len(child_fd_list) > 0:
        fd = child_fd_list.pop(0)
        try:
            os.close(fd)
        except Exception as msg:
            pass

def child_fd_list_add(fd):
    '''add a file descriptor to list to be closed in child processes'''
    global child_fd_list
    child_fd_list.append(fd)

def child_fd_list_remove(fd):
    '''remove a file descriptor to list to be closed in child processes'''
    global child_fd_list
    try:
        child_fd_list.remove(fd)
    except Exception:
        pass

def quaternion_to_axis_angle(q):
    from pymavlink.rotmat import Vector3
    a, b, c, d = q.q
    n = math.sqrt(b**2 + c**2 + d**2)
    if not n:
        return Vector3(0, 0, 0)
    angle = 2 * math.acos(a)
    if angle > math.pi:
        angle = angle - 2 * math.pi
    return Vector3(angle * b / n, angle * c / n, angle * d / n)

def null_term(str):
    '''null terminate a string for py3'''
    if sys.version_info.major < 3:
        return str
    if isinstance(str, bytes):
        str = str.decode("utf-8")
    idx = str.find("\0")
    if idx != -1:
        str = str[:idx]
    return str

    
def decode_devid(devid, pname):
    '''decode one device ID. Used for 'devid' command in mavproxy and MAVExplorer'''
    devid = int(devid)
    if devid == 0:
        return

    bus_type=devid & 0x07
    bus=(devid>>3) & 0x1F
    address=(devid>>8)&0xFF
    devtype=(devid>>16)

    bustypes = {
        1: "I2C",
        2: "SPI",
        3: "UAVCAN",
        4: "SITL",
        5: "MSP"
        }

    compass_types = {
        0x01 : "DEVTYPE_HMC5883_OLD",
        0x07 : "DEVTYPE_HMC5883",
        0x02 : "DEVTYPE_LSM303D",
        0x04 : "DEVTYPE_AK8963 ",
        0x05 : "DEVTYPE_BMM150 ",
        0x06 : "DEVTYPE_LSM9DS1",
        0x08 : "DEVTYPE_LIS3MDL",
        0x09 : "DEVTYPE_AK09916",
        0x0A : "DEVTYPE_IST8310",
        0x0B : "DEVTYPE_ICM20948",
        0x0C : "DEVTYPE_MMC3416",
        0x0D : "DEVTYPE_QMC5883L",
        0x0E : "DEVTYPE_MAG3110",
        0x0F : "DEVTYPE_SITL",
        0x10 : "DEVTYPE_IST8308",
        0x11 : "DEVTYPE_RM3100_OLD",
        0x12 : "DEVTYPE_RM3100",
        }

    imu_types = {
        0x09 : "DEVTYPE_BMI160",
        0x10 : "DEVTYPE_L3G4200D",
        0x11 : "DEVTYPE_ACC_LSM303D",
        0x12 : "DEVTYPE_ACC_BMA180",
        0x13 : "DEVTYPE_ACC_MPU6000",
        0x16 : "DEVTYPE_ACC_MPU9250",
        0x17 : "DEVTYPE_ACC_IIS328DQ",
        0x21 : "DEVTYPE_GYR_MPU6000",
        0x22 : "DEVTYPE_GYR_L3GD20",
        0x24 : "DEVTYPE_GYR_MPU9250",
        0x25 : "DEVTYPE_GYR_I3G4250D",
        0x26 : "DEVTYPE_GYR_LSM9DS1",
        0x27 : "DEVTYPE_INS_ICM20789",
        0x28 : "DEVTYPE_INS_ICM20689",
        0x29 : "DEVTYPE_INS_BMI055",
        0x2A : "DEVTYPE_SITL",
        0x2B : "DEVTYPE_INS_BMI088",
        0x2C : "DEVTYPE_INS_ICM20948",
        0x2D : "DEVTYPE_INS_ICM20648",
        0x2E : "DEVTYPE_INS_ICM20649",
        0x2F : "DEVTYPE_INS_ICM20602",
        0x30 : "DEVTYPE_INS_ICM20601",
        0x31 : "DEVTYPE_INS_ADIS1647x",
        }

    baro_types = {
        0x01 : "DEVTYPE_BARO_SITL",
        0x02 : "DEVTYPE_BARO_BMP085",
        0x03 : "DEVTYPE_BARO_BMP280",
        0x04 : "DEVTYPE_BARO_BMP388",
        0x05 : "DEVTYPE_BARO_DPS280",
        0x06 : "DEVTYPE_BARO_DPS310",
        0x07 : "DEVTYPE_BARO_FBM320",
        0x08 : "DEVTYPE_BARO_ICM20789",
        0x09 : "DEVTYPE_BARO_KELLERLD",
        0x0A : "DEVTYPE_BARO_LPS2XH",
        0x0B : "DEVTYPE_BARO_MS5611",
        0x0C : "DEVTYPE_BARO_SPL06",
        0x0D : "DEVTYPE_BARO_UAVCAN",
    }
        
    decoded_devname = ""

    if pname.startswith("COMPASS"):
        if bus_type == 3 and devtype == 1:
            decoded_devname = "UAVCAN"
        else:
            decoded_devname = compass_types.get(devtype, "UNKNOWN")

    if pname.startswith("INS"):
        decoded_devname = imu_types.get(devtype, "UNKNOWN")

    if pname.startswith("GND_BARO"):
        decoded_devname = baro_types.get(devtype, "UNKNOWN")
        
    print("%s: bus_type:%s(%u)  bus:%u address:%u(0x%x) devtype:%u(0x%x) %s (%u)" % (
        pname,
        bustypes.get(bus_type,"UNKNOWN"), bus_type,
        bus, address, address, devtype, devtype, decoded_devname,
        devid))
