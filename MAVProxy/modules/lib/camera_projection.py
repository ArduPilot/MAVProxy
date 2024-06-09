#!/usr/bin/env python
'''
class to project a camera view onto the map
'''

import numpy
from numpy import array, eye, zeros, uint64
from numpy import linalg, dot, transpose
from numpy import sin, cos, pi
from math import pi
import math
import cv2
from MAVProxy.modules.lib import mp_elevation
from pymavlink.rotmat import Vector3
from MAVProxy.modules.lib import mp_util
import time

class CameraParams:
    '''
    object representing camera parameters. Can be initialised from basic parameters for idealised lens or use fromfile() to loa
    from a calibration file
    '''
    def __init__(self, lens=None, sensorwidth=None, xresolution=None, yresolution=None, K=None, D=None, FOV=None):
        if xresolution is None:
            raise ValueError("xresolution required")
        if yresolution is None:
            raise ValueError("yresolution required")
        if lens is None and FOV is not None:
            # FOV = 2 arctan(xres/(2f))
            if sensorwidth is None:
                # default to 5mm sensor width for FOV calculation
                sensorwidth=5.0 # mm
            # FOV = 2 arctan( sensorwidth/(2f))
            lens = 0.5 * sensorwidth / math.tan(math.radians(FOV)*0.5)
        if lens is None:
            raise ValueError("Lens required")
        if sensorwidth is None:
            raise ValueError("sensorwidth required")
        self.version = 0
        self.sensorwidth = sensorwidth
        self.lens = lens
        self.K = K
        self.D = D
        self.set_resolution(xresolution, yresolution)
        self.FOV = FOV
        if FOV is None:
            self.FOV = math.degrees(2*math.atan(sensorwidth/(2*lens)))

    def set_resolution(self, xresolution, yresolution):
        '''set camera resolution'''
        self.xresolution = xresolution
        self.yresolution = yresolution

        # compute focal length in pixels
        f_p = xresolution * self.lens / self.sensorwidth
        if self.K is None:
            self.K = array([[f_p, 0.0, xresolution/2],[0.0, f_p, yresolution/2], [0.0,0.0,1.0]])
        if self.D is None:
            self.D = array([[0.0, 0.0, 0.0, 0.0, 0.0]])

    def __repr__(self):
        return json.dumps(self.todict(),indent=2)

    def setParams(self, K, D):
        self.K = array(K)
        self.D = array(D)

    def todict(self):
        data = {}
        data['version'] = self.version
        data['lens'] = self.lens
        data['sensorwidth'] = self.sensorwidth
        data['xresolution'] = self.xresolution
        data['yresolution'] = self.yresolution
        if self.K is not None:
            data['K'] = self.K.tolist()
        print("Set K to " + str(self.K))
        print("Set K to " + str(data['K']))
        if self.D is not None:
            data['D'] = self.D.tolist()
        return data

    @staticmethod
    def fromdict(data):
        if data['version'] != 0:
            raise Exception('version %d of camera params unsupported' % (self.version))
        try:
            K = array(data['K'])
            D = array(data['D'])
        except KeyError:
            K = None
            D = None
        ret = CameraParams(lens=data['lens'],
                           sensorwidth=data['sensorwidth'],
                            xresolution=data['xresolution'],
                            yresolution=data['yresolution'],
                            K=K,
                            D=D)
        ret.version = data['version']
        return ret;


    @staticmethod
    def fromstring(strung):
        dic = json.loads(str(strung))
        return CameraParams.fromdict(dic)

    def save(self, filename):
        f = open(filename,"w")
        # dump json form
        f.write(str(self)+"\n")
        f.close()

    @staticmethod
    def fromfile(filename):
        f = open(filename,"r")
        # dump json form
        d = f.read(65535)
        f.close()
        return CameraParams.fromstring(d)

# a simple frame transformation matrix
def rotationMatrix(phi, theta, psi):
    R_phi   = array([[1.0,0.0,0.0],[0.0,cos(phi),sin(phi)],[0.0,-sin(phi),cos(phi)]])
    R_theta = array([[ cos(theta),0.0,-sin(theta)],[0.0,1.0,0.0],[ sin(theta),0.0,cos(theta)]])
    R_psi   = array([[cos(psi),sin(psi),0.0],[-sin(psi),cos(psi),0.0],[0.0,0.0,1.0]])
    R = dot(dot(R_phi,R_theta),R_psi)
    return R


class uavxfer:
    '''
    camera transfer function implementation, thanks to Matt Ridley from CanberraUAV
    '''
    def setCameraParams(self, fu, fv, cu, cv):
        K  = array([[fu, 0.0, cu],[0.0, fv, cv],[0.0, 0.0, 1.0]])
        self.setCameraMatrix(K)

    def setCameraMatrix(self, K):
        K_i = linalg.inv(K)
        self.Tk = eye(4,4)
        self.Tk[:3,:3] = K;
        self.Tk_i = eye(4,4)
        self.Tk_i[:3,:3] = K_i

    def setCameraOrientation(self, roll, pitch, yaw):
        self.Rc = array(eye(4,4))
        self.Rc[:3,:3] = transpose(rotationMatrix(roll, pitch, yaw))
        self.Rc_i = linalg.inv(self.Rc)

    def setPlatformPose(self, north, east, down, roll, pitch, yaw):
        '''set pose, angles in degrees, 0,0,0 is straight down'''
        self.Xp = array([north, east, down, 1.0])
        self.Rp = array(eye(4,4))
        self.Rp[:3,:3] = transpose(rotationMatrix(roll, pitch, yaw))
        self.Rp[:3,3] = array([north, east, down])
        self.Rp_i = linalg.inv(self.Rp)

    def setFlatEarth(self, z):
        self.z_earth = z

    def worldToPlatform(self, north, east, down):
        x_w = array([north, east, down, 1.0])
        x_p = dot(self.Rp_i, x_w)[:3]
        return x_p

    def worldToImage(self, north, east, down):
        x_w = array([north, east, down, 1.0])
        x_p = dot(self.Rp_i, x_w)
        x_c = dot(self.Rc_i, x_p)
        x_i = dot(self.Tk, x_c)
        return x_i[:3]/x_i[2]

    def platformToWorld(self, north, east, down):
        x_p = array([north, east, down, 1.0])
        x_w = dot(self.Rp, x_p)
        return x_w

    def imageToWorld(self, u, v):
        x_i = array([u, v, 1.0, 0.0])
        v_c = dot(self.Tk_i, x_i)
        v_p = dot(self.Rc, v_c)
        v_w = dot(self.Rp, v_p)
        # compute scale for z == z_earth
        scale = (self.z_earth-self.Xp[2])/v_w[2]
        #project from platform to ground
        x_w = scale*v_w + self.Xp;
        return x_w, scale

    def __init__(self, fu=200, fv=200, cu=512, cv=480):
        self.setCameraParams(fu, fv, cu, cv)
        self.Rc = self.Rc_i = array(eye(4,4))
        self.Rp = self.Rp_i = array(eye(4,4))
        self.z_earth = -600

class CameraProjection:
    def __init__(self, C, elevation_model=None, terrain_source="SRTM3"):
        self.C = C
        self.elevation_model = elevation_model
        if elevation_model is None:
            self.elevation_model = mp_elevation.ElevationModel(database=terrain_source)

    def pixel_position_flat(self, xpos, ypos, height_agl, roll_deg, pitch_deg, yaw_deg):
        '''
        find the NED offset on the ground in meters of a pixel in a ground image
        given height above the ground in meters, and pitch/roll/yaw in degrees, the
        lens and image parameters

        The xpos,ypos is from the top-left of the image
        The height_agl is in meters above ground level. Flat earth below camera is assumed
    
        The yaw is from grid north. Positive yaw is clockwise
        The roll is from horiznotal. Positive roll is down on the right
        The pitch is from horiznotal. Positive pitch is up in the front, -90 is straight down

        return result is a tuple, with meters north, east and down of current GPS position
        '''
        xfer = uavxfer()
        xfer.setCameraMatrix(self.C.K)
        xfer.setCameraOrientation( 0.0, 0.0, pi/2 )
        xfer.setFlatEarth(0);
        xfer.setPlatformPose(0, 0, -height_agl, math.radians(roll_deg), math.radians(pitch_deg+90), math.radians(yaw_deg))

        # compute the undistorted points for the ideal camera matrix
        src = numpy.zeros((1,1,2), numpy.float32)
        src[0,0] = (xpos, ypos)
        R = eye(3)
        K = self.C.K
        D = self.C.D
        dst = cv2.undistortPoints(src, K, D, R, K)
        x = dst[0,0][0]
        y = dst[0,0][1]

        # negative scale means camera pointing above horizon
        # large scale means a long way away also unreliable
        (pos_w, scale) = xfer.imageToWorld(x, y)
        if scale < 0:
             return None
        ret = Vector3(pos_w[0], pos_w[1], height_agl)
        return ret

    def get_posned(self, x, y, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg):
        '''
        get a Vector3() NED from the camera position to project onto the ground

        return pos NED from camera as Vector3 or None
        '''
        # get height of terrain below camera
        theight = self.elevation_model.GetElevation(clat, clon)
        if calt_amsl <= theight:
            return None

        # project with flat earth
        pos_ned = self.pixel_position_flat(x, y, calt_amsl-theight, roll_deg, pitch_deg, yaw_deg)
        if pos_ned is None or pos_ned.z <= 0:
            return None

        # iterate to make more accurate, accounting for difference in terrain height at this point
        latlon = mp_util.gps_offset(clat, clon, pos_ned.y, pos_ned.x)

        for i in range(3):
            ground_alt = self.elevation_model.GetElevation(latlon[0], latlon[1])
            if ground_alt is None:
                return None
            sr = pos_ned.length()
            if sr <= 1:
                return None
            posd2 = calt_amsl - ground_alt
            sin_pitch = pos_ned.z / sr
            # adjust for height at this point
            sr2 = sr - (pos_ned.z - posd2) / sin_pitch
            #print("SR: ", pos_ned.z, posd2, sr, sr2)
            pos_ned = pos_ned * (sr2 / sr)
            latlon = mp_util.gps_offset(clat, clon, pos_ned.y, pos_ned.x)
            if latlon is None:
                return None
        return pos_ned
        
    def get_latlonalt_for_pixel(self, x, y, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg):
        '''
        get lat,lon of projected pixel from camera.
        x,y are pixel coordinates, 0,0 is top-left corner

        return is (lat,lon,alt) tuple or None
        '''
        pos_ned = self.get_posned(x, y, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg)
        if pos_ned is None or pos_ned.z <= 0:
            return None

        latlon = mp_util.gps_offset(clat, clon, pos_ned.y, pos_ned.x)
        if latlon is None:
            return None
        return (latlon[0], latlon[1], calt_amsl-pos_ned.z)

    def get_slantrange(self, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg):
        '''
        get slant range to ground for a pixel in the middle of the camera view

        return is slant range in meters or None
        '''
        pos_ned = self.get_posned(self.C.xresolution//2, self.C.yresolution//2, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg)
        if pos_ned is None:
            return None
        return pos_ned.length()
    
    def get_projection(self, clat, clon, calt_amsl, roll_deg, pitch_deg, yaw_deg):
        '''return a list of (lat,lon) tuples drawing the camera view on the terrain'''
        ret = []
        xres = self.C.xresolution
        yres = self.C.yresolution
        for (x,y) in [(0,0), (xres, 0), (xres, yres), (0,yres)]:
            y0 = 0
            while True:
                latlonalt = self.get_latlonalt_for_pixel(x,y+y0,clat,clon,calt_amsl,roll_deg,pitch_deg,yaw_deg)
                if latlonalt is not None:
                    break
                # chop off the top 10 pixels and try again
                y0 += 10
                if y0 >= self.C.yresolution:
                    # give up
                    return None
            ret.append((latlonalt[0],latlonalt[1]))
        ret.append(ret[0])
        return ret


def test_pixel_position():
    C = CameraParams(lens=4.0, sensorwidth=5.0, xresolution=1024, yresolution=768)
    cproj = CameraProjection(C)
    print("FOV: %.1f degrees" % C.FOV)
    pos_ned = cproj.pixel_position_flat(100, 100, 123, 2, -89.9, 0)
    assert abs((pos_ned - Vector3(43.6719, -67.3798, 123)).length()) < 0.01

    pos_ned = cproj.pixel_position_flat(0, 130, 57, 2, -89.9, 0)
    assert abs((pos_ned - Vector3(18.188, -38.4761, 57)).length()) < 0.01

    
if __name__ == "__main__":

    C1 = CameraParams(xresolution=640, yresolution=512, FOV=24.2)
    print("Lens: %.2f" % C1.lens)
    C2 = CameraParams(xresolution=2560, yresolution=1440, FOV=88.0)
    print("Lens: %.2f" % C2.lens)

    test_pixel_position()

    from argparse import ArgumentParser
    parser = ArgumentParser("camera_projection.py [options]")
    parser.add_argument("--lat", type=float, default=-35.363261, help="start latitude")
    parser.add_argument("--lon", type=float, default=149.165230, help="start longitude")
    parser.add_argument("--alt-agl", type=float, default=100.0, help="height AGL")
    parser.add_argument("--roll", type=float, default=0.0, help="roll")
    parser.add_argument("--pitch", type=float, default=-30, help="pitch")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw")
    parser.add_argument("--yaw-delta", type=float, default=0.0, help="yaw delta per loop")
    parser.add_argument("--pitch-delta", type=float, default=-1.0, help="pitch delta per loop")
    parser.add_argument("--service", default="MicrosoftSat", help="tile service")
    parser.add_argument("--offline", action='store_true', default=False, help="no download")
    parser.add_argument("--delay", type=float, default=0.3, help="tile download delay")
    parser.add_argument("--max-zoom", type=int, default=19, help="maximum tile zoom")
    parser.add_argument("--debug", action='store_true', default=False, help="show debug info")
    parser.add_argument("--boundary", default=None, help="show boundary")
    parser.add_argument("--mission", default=[], action='append', help="show mission")
    parser.add_argument("--thumbnail", default=None, help="show thumbnail")
    parser.add_argument("--icon", default=None, help="show icon")
    parser.add_argument("--flag", default=[], type=str, action='append', help="flag positions")
    parser.add_argument("--grid", default=False, action='store_true', help="add a UTM grid")
    parser.add_argument("--verbose", action='store_true', default=False, help="show mount actions")
    parser.add_argument("--terrain-source", type=str, default="SRTM1", choices=["SRTM1", "SRTM3", "None"], help="Elevation model")
    args = parser.parse_args()
    
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    sm = mp_slipmap.MPSlipMap(lat=args.lat,
                              lon=args.lon,
                              download=not args.offline,
                              service=args.service,
                              debug=args.debug,
                              max_zoom=args.max_zoom,
                              elevation=args.terrain_source,
                              tile_delay=args.delay)

    lat = args.lat
    lon = args.lon
    pitch = args.pitch
    pitch_delta = args.pitch_delta
    yaw = args.yaw
    yaw_delta = args.yaw_delta
    elevation_model = mp_elevation.ElevationModel(database=args.terrain_source)
    alt_amsl = args.alt_agl + elevation_model.GetElevation(lat, lon,timeout=10)
    print("Camera alt: %.1f" % alt_amsl)

    C1 = CameraParams(xresolution=2560, yresolution=1440, FOV=88.0)
    C2 = CameraParams(xresolution=640, yresolution=512, FOV=24.2)
    cproj1 = CameraProjection(C1, elevation_model=elevation_model)
    cproj2 = CameraProjection(C2, elevation_model=elevation_model)
    while sm.is_alive():
        p1 = cproj1.get_projection(lat, lon, alt_amsl, args.roll, pitch, yaw)
        if p1 is not None:
            sm.add_object(mp_slipmap.SlipPolygon('projection1', p1, layer=1, linewidth=2, colour=(0,255,0)))
        p2 = cproj2.get_projection(lat, lon, alt_amsl, args.roll, pitch, yaw)
        if p2 is not None:
            sm.add_object(mp_slipmap.SlipPolygon('projection2', p2, layer=1, linewidth=2, colour=(0,0,255)))
        pitch += pitch_delta
        if pitch < -180 - args.pitch:
            pitch_delta *= -1.0
            pitch = -180 - args.pitch
        if pitch > args.pitch:
            pitch_delta *= -1.0
            pitch = args.pitch
        yaw += yaw_delta
        if yaw < 0:
            yaw += 360
        if yaw > 360:
            yaw -= 360
        while not sm.event_queue_empty():
            obj = sm.get_event()
            if isinstance(obj, mp_slipmap.SlipMouseEvent):
                lat = obj.latlon[0]
                lon = obj.latlon[1]
                alt_amsl = args.alt_agl + elevation_model.GetElevation(lat, lon,timeout=10)
        print("Pitch %.1f  Yaw %.1f" % (pitch, yaw))
        time.sleep(0.1)
