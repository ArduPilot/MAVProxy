#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''common mavproxy utility functions'''

import gzip
import math
import os
import io
import sys
import platform
import warnings
from math import cos, sin, tan, atan2, sqrt, radians, degrees, pi, log, fmod

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

def wrap_360(angle):
    '''wrap an angle to 0..360 degrees'''
    return math.fmod(angle, 360.0)

def wrap_180(angle):
    '''wrap an angle to -180..180 degrees'''
    if angle >= -180 and angle < 180:
        return angle
    a = math.fmod(angle+180, 360.0)-180.0
    return a

def gps_distance(lat1, lon1, lat2, lon2):
    '''distance between two points in meters along rhumb line
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    and http://www.edwilliams.org/avform147.htm#Rhumb
    '''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)

    if abs(lat2-lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        q = (lat2-lat1)/log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))
    d = sqrt((lat2-lat1)**2 + q**2 * (lon2-lon1)**2)
    return d * radius_of_earth

def gps_bearing(lat1, lon1, lat2, lon2):
    '''return rhumb bearing between two points in degrees, in range 0-360
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    tc = -fmod(atan2(lon1-lon2,log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))),2*pi)
    if tc < 0:
        tc += 2*pi
    return degrees(tc)

def wrap_valid_longitude(lon):
    ''' wrap a longitude value around to always have a value in the range
        [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
    '''
    return (((lon + 180.0) % 360.0) - 180.0)

def constrain(v, minv, maxv):
    if v < minv:
        v = minv
    if v > maxv:
        v = maxv
    return v

def constrain_latlon(latlon):
    epsilon = 1.0e-3
    return (constrain(latlon[0],-90+epsilon, 90-epsilon), wrap_180(latlon[1]))

def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance
    along rhumb line thanks to http://www.movable-type.co.uk/scripts/latlong.html
    '''
    lat1 = constrain(radians(lat), -pi/2+1.0e-15, pi/2-1.0e-15)
    lon1 = radians(lon)
    tc = radians(-bearing)
    d = distance/radius_of_earth

    lat = lat1 + d * cos(tc)
    lat = constrain(lat, -pi/2 + 1.0e-15, pi/2 - 1.0e-15)
    if abs(lat-lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        try:
            dphi = log(tan(lat/2+pi/4)/tan(lat1/2+pi/4))
        except Exception:
            print(degrees(lat),degrees(lat1))
            raise
        q = (lat-lat1)/dphi
    dlon = -d*sin(tc)/q
    lon = fmod(lon1+dlon+pi,2*pi)-pi
    return (degrees(lat), degrees(lon))

def gps_offset(lat, lon, east, north):
    '''return new lat/lon after moving east/north
    by the given number of meters'''
    bearing = degrees(atan2(east, north))
    distance = sqrt(east**2 + north**2)
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
    '''return true if two bounding boxes overlap. coordinates are lat/lon degrees. x is lat, y is lon'''
    (x1,y1,w1,h1) = bound1
    (x2,y2,w2,h2) = bound2
    if x1+w1 < x2:
        return False
    if x2+w2 < x1:
        return False
    if wrap_180((y1+h1)-y2) < 0:
        return False
    if wrap_180((y2+h2)-y1) < 0:
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
        lon = wrap_180(lon)
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
            with gzip.GzipFile(fileobj=io.BytesIO(data)) as gz:
                data = gz.read()
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
        5: "MSP",
        6: "EAHRS"
        }

    compass_types = {
        0x01 : "HMC5883_OLD",
        0x07 : "HMC5883",
        0x02 : "LSM303D",
        0x04 : "AK8963 ",
        0x05 : "BMM150 ",
        0x06 : "LSM9DS1",
        0x08 : "LIS3MDL",
        0x09 : "AK09916",
        0x0A : "IST8310",
        0x0B : "ICM20948",
        0x0C : "MMC3416",
        0x0D : "QMC5883L",
        0x0E : "MAG3110",
        0x0F : "SITL",
        0x10 : "IST8308",
        0x11 : "RM3100",
        0x12 : "RM3100_2",
        0x13 : "MMC5883",
        0x14 : "AK09918",
        }

    imu_types = {
        0x09 : "BMI160",
        0x10 : "L3G4200D",
        0x11 : "ACC_LSM303D",
        0x12 : "ACC_BMA180",
        0x13 : "ACC_MPU6000",
        0x16 : "ACC_MPU9250",
        0x17 : "ACC_IIS328DQ",
        0x18 : "ACC_LSM9DS1",
        0x21 : "GYR_MPU6000",
        0x22 : "GYR_L3GD20",
        0x24 : "GYR_MPU9250",
        0x25 : "GYR_I3G4250D",
        0x26 : "GYR_LSM9DS1",
        0x27 : "INS_ICM20789",
        0x28 : "INS_ICM20689",
        0x29 : "INS_BMI055",
        0x2A : "SITL",
        0x2B : "INS_BMI088",
        0x2C : "INS_ICM20948",
        0x2D : "INS_ICM20648",
        0x2E : "INS_ICM20649",
        0x2F : "INS_ICM20602",
        0x30 : "INS_ICM20601",
        0x31 : "INS_ADIS1647x",
        0x32 : "INS_SERIAL",
        0x33 : "INS_ICM40609",
        0x34 : "INS_ICM42688",
        0x35 : "INS_ICM42605",
        0x37 : "INS_IIM42652",
        0x38 : "INS_BMI270",
        0x39 : "INS_BMI085",
        0x3A : "INS_ICM42670",
        }

    baro_types = {
        0x01 : "BARO_SITL",
        0x02 : "BARO_BMP085",
        0x03 : "BARO_BMP280",
        0x04 : "BARO_BMP388",
        0x05 : "BARO_DPS280",
        0x06 : "BARO_DPS310",
        0x07 : "BARO_FBM320",
        0x08 : "BARO_ICM20789",
        0x09 : "BARO_KELLERLD",
        0x0A : "BARO_LPS2XH",
        0x0B : "BARO_MS5611",
        0x0C : "BARO_SPL06",
        0x0D : "BARO_UAVCAN",
        0x0E : "BARO_MSP",
        0x0F : "BARO_ICP101XX",
        0x10 : "BARO_ICP201XX",
    }

    airspeed_types = {
        0x01 : "AIRSPEED_SITL",
        0x02 : "AIRSPEED_MS4525",
        0x03 : "AIRSPEED_MS5525",
        0x04 : "AIRSPEED_DLVR",
        0x05 : "AIRSPEED_MSP",
        0x06 : "AIRSPEED_SDP3X",
        0x07 : "AIRSPEED_UAVCAN",
        0x08 : "AIRSPEED_ANALOG",
        0x09 : "AIRSPEED_NMEA",
        0x0A : "AIRSPEED_ASP5033",
    }
        
    decoded_devname = ""

    if pname.startswith("COMPASS"):
        if bus_type == 3 and devtype == 1:
            decoded_devname = "UAVCAN"
        elif bus_type == 6 and devtype == 1:
            decoded_devname = "EAHRS"
        else:
            decoded_devname = compass_types.get(devtype, "UNKNOWN")

    if pname.startswith("INS"):
        decoded_devname = imu_types.get(devtype, "UNKNOWN")

    if pname.startswith("GND_BARO"):
        decoded_devname = baro_types.get(devtype, "UNKNOWN")

    if pname.startswith("BARO"):
        decoded_devname = baro_types.get(devtype, "UNKNOWN")

    if pname.startswith("ARSP"):
        decoded_devname = airspeed_types.get(devtype, "UNKNOWN")
        
    print("%s: bus_type:%s(%u)  bus:%u address:%u(0x%x) devtype:%u(0x%x) %s (%u)" % (
        pname,
        bustypes.get(bus_type,"UNKNOWN"), bus_type,
        bus, address, address, devtype, devtype, decoded_devname,
        devid))


def decode_flight_sw_version(flight_sw_version):
    '''decode 32 bit flight_sw_version mavlink parameter - corresponds to encoding in ardupilot GCS_MAVLINK::send_autopilot_version'''
    fw_type_id = (flight_sw_version >>  0) % 256
    patch      = (flight_sw_version >>  8) % 256
    minor      = (flight_sw_version >> 16) % 256
    major      = (flight_sw_version >> 24) % 256
    if (fw_type_id==0):
        fw_type="dev"
    elif (fw_type_id==64):
        fw_type="alpha"
    elif (fw_type_id==128):
        fw_type="beta"
    elif (fw_type_id==192):
        fw_type="rc"
    elif (fw_type_id==255):
        fw_type="official"
    else:
        fw_type="undefined"
    return major,minor,patch,fw_type
