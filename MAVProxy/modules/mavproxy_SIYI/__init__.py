'''
control SIYI camera over UDP
'''

'''
TODO:
  circle hottest area?
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
from pymavlink import DFReader
from pymavlink.rotmat import Matrix3
from pymavlink.rotmat import Vector3
import math
from math import radians, degrees
from threading import Thread
import cv2
import traceback
import copy

import socket, time, os, struct

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_image import MPImage
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    from MAVProxy.modules.lib.mp_image import MPImageOSD_HorizonLine
    from MAVProxy.modules.lib.mp_image import MPImageOSD_None

from MAVProxy.modules.mavproxy_SIYI.camera_view import CameraView

SIYI_RATE_MAX_DPS = 90.0
SIYI_HEADER1 = 0x55
SIYI_HEADER2 = 0x66

ACQUIRE_FIRMWARE_VERSION = 0x01
HARDWARE_ID = 0x02
AUTO_FOCUS = 0x04
MANUAL_ZOOM_AND_AUTO_FOCUS = 0x05
MANUAL_FOCUS = 0x06
GIMBAL_ROTATION = 0x07
CENTER = 0x08
ACQUIRE_GIMBAL_CONFIG_INFO = 0x0A
FUNCTION_FEEDBACK_INFO = 0x0B
PHOTO = 0x0C
ACQUIRE_GIMBAL_ATTITUDE = 0x0D
SET_ANGLE = 0x0E
RESET_ATTITUDE = 0x08
ABSOLUTE_ZOOM = 0x0F
READ_RANGEFINDER = 0x15
READ_ENCODERS = 0x26
READ_CONTROL_MODE = 0x27
READ_THRESHOLDS = 0x28
SET_THRESHOLDS = 0x29
READ_VOLTAGES = 0x2A
READ_TEMP_FULL_SCREEN = 0x14
SET_IMAGE_TYPE = 0x11
SET_THERMAL_PALETTE = 0x1B
REQUEST_CONTINUOUS_DATA = 0x25
ATTITUDE_EXTERNAL = 0x22
VELOCITY_EXTERNAL = 0x26
TEMPERATURE_BOX = 0x13
GET_THERMAL_MODE = 0x33
SET_THERMAL_MODE = 0x34
GET_TEMP_FRAME = 0x35
SET_WEAK_CONTROL = 0x71
GET_THERMAL_GAIN = 0x37
SET_THERMAL_GAIN = 0x38
GET_THERMAL_PARAM = 0x39
SET_THERMAL_PARAM = 0x3A
GET_THERMAL_ENVSWITCH = 0x3B
SET_THERMAL_ENVSWITCH = 0x3C
SET_TIME = 0x30

class ThermalParameters:
    def __init__(self, distance, target_emissivity, humidity, air_temperature, reflection_temperature):
        self.distance = distance
        self.target_emissivity = target_emissivity
        self.humidity = humidity
        self.air_temperature = air_temperature
        self.reflection_temperature = reflection_temperature

    def __repr__(self):
        return "Dist=%.2f Emiss=%.2f Hum=%.2f AirTemp=%.2f RefTemp=%.2f" % (self.distance,
                                                                            self.target_emissivity,
                                                                            self.humidity,
                                                                            self.air_temperature,
                                                                            self.reflection_temperature)

    def args(self):
        return [int(self.distance*100), int(self.target_emissivity*100),
                int(self.humidity*100), int(self.air_temperature*100), int(self.reflection_temperature*100)]


def crc16_from_bytes(bytes, initial=0):
    # CRC-16-CCITT
    # Initial value: 0xFFFF
    # Poly: 0x1021
    # Reverse: no
    # Output xor: 0
    # Check string: '123456789'
    # Check value: 0x29B1

    try:
        if isinstance(bytes, basestring):  # Python 2.7 compatibility
            bytes = map(ord, bytes)
    except NameError:
        if isinstance(bytes, str):  # This branch will be taken on Python 3
            bytes = map(ord, bytes)

    crc = initial
    for byte in bytes:
        crc ^= byte << 8
        for bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

class PI_controller:
    '''simple PI controller'''
    def __init__(self, settings, Pgain, Igain, IMAX):
        self.Pgain = Pgain
        self.Igain = Igain
        self.IMAX = IMAX
        self.I = 0.0
        self.settings = settings
        self.last_t = time.time()

    def run(self, err, ff_rate):
        now = time.time()
        dt = now - self.last_t
        if now - self.last_t > 1.0:
            self.reset_I()
            dt = 0
        self.last_t = now
        P = self.settings.get(self.Pgain) * self.settings.gain_mul
        I = self.settings.get(self.Igain) * self.settings.gain_mul
        IMAX = self.settings.get(self.IMAX)
        max_rate = self.settings.max_rate

        out = P*err
        saturated = err > 0 and (out + self.I) >= max_rate
        saturated |= err < 0 and (out + self.I) <= -max_rate
        if not saturated:
            self.I += I*err*dt
        self.I = mp_util.constrain(self.I, -IMAX, IMAX)
        ret = out + self.I + ff_rate
        return mp_util.constrain(ret, -max_rate, max_rate)

    def reset_I(self):
        self.I = 0

class DF_logger:
    '''write to a DF format log'''
    def __init__(self, filename):
        self.outf = open(filename,'wb')
        self.outf.write(bytes([0]))
        self.outf.flush()
        self.mlog = DFReader.DFReader_binary(filename)
        self.outf.seek(0)
        self.formats = {}
        self.last_flush = time.time()

    def write(self, name, fmt, fields, *args):
        if not name in self.formats:
            self.formats[name] = self.mlog.add_format(DFReader.DFFormat(0, name, 0, fmt, fields))
            self.outf.write(self.mlog.make_format_msgbuf(self.formats[name]))
        self.outf.write(self.mlog.make_msgbuf(self.formats[name], args))
        now = time.time()
        if now - self.last_flush > 5:
            self.last_flush = now
            self.outf.flush()


def rate_mapping(desired_rate):
    '''map from a desired rate in deg/sec to a siyi SDK rate'''
    drate = abs(desired_rate)
    rate_map = [
        (70, 98.0),
        (60, 83.4),
        (50, 68.1),
        (40, 53.4),
        (30, 38.6),
        (25, 31.2),
        (20, 23.8),
        (15, 16.2),
        (10.0, 9.0),
        (9.0, 7.5),
        (8.0, 6.0),
        (7.0, 4.7),
        (6.0, 3.0),
        (5.0, 1.75),
        (4.0, 0.0),
        (0.0, 0.0),
        ]
    if drate >= rate_map[0][1]:
        ret = rate_map[0][0]
        if desired_rate < 0:
            ret = -ret
        return ret
    for i in range(1, len(rate_map)):
        (rate_to2, rate_from2) = rate_map[i-1]
        (rate_to1, rate_from1) = rate_map[i]
        if drate <= rate_from2 and drate >= rate_from1:
            p = (drate - rate_from1) / (rate_from2 - rate_from1)
            ret = rate_to1 + p * (rate_to2 - rate_to1)
            if desired_rate < 0:
                ret = -ret
            return ret
    return 0.0

class SIYIModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SIYIModule, self).__init__(mpstate, "SIYI", "SIYI camera support")

        self.add_command('siyi', self.cmd_siyi, "SIYI camera control",
                         ["<rates|connect|autofocus|zoom|yaw|pitch|center|getconfig|angle|photo|recording|lock|follow|fpv|settarget|notarget|thermal|rgbview|tempsnap|get_thermal_mode|thermal_gain|get_thermal_gain|settime>",
                          "<therm_getenv|therm_set_distance|therm_set_emissivity|therm_set_humidity|therm_set_airtemp|therm_set_reftemp|therm_getswitch|therm_setswitch>",
                          "set (SIYISETTING)",
                          "imode <1|2|3|4|5|6|7|8|wide|zoom|split>",
                          "palette <WhiteHot|Sepia|Ironbow|Rainbow|Night|Aurora|RedHot|Jungle|Medical|BlackHot|GloryHot>",
                          "thermal_mode <0|1>",
                          ])

        # filter_dist is distance in metres
        self.siyi_settings = mp_settings.MPSettings([("port", int, 37260),
                                                     ('ip', str, "192.168.144.25"),
                                                     ('yaw_rate', float, 10),
                                                     ('pitch_rate', float, 10),
                                                     ('rates_hz', float, 5),
                                                     ('gain_mul', float, 1.0),
                                                     ('yaw_gain_P', float, 1),
                                                     ('yaw_gain_I', float, 1),
                                                     ('yaw_gain_IMAX', float, 5),
                                                     ('pitch_gain_P', float, 1),
                                                     ('pitch_gain_I', float, 1),
                                                     ('pitch_gain_IMAX', float, 5),
                                                     ('mount_pitch', float, 0),
                                                     ('mount_yaw', float, 0),
                                                     ('lag', float, 0),
                                                     ('target_rate', float, 10),
                                                     ('telem_rate', float, 4),
                                                     ('att_send_hz', float, 10),
                                                     ('mode_hz', float, 0),
                                                     ('temp_hz', float, 5),
                                                     ('rtsp_rgb', str, 'rtsp://192.168.144.25:8554/video1'),
                                                     ('rtsp_thermal', str, 'rtsp://192.168.144.25:8554/video2'),
                                                     #('rtsp_rgb', str, 'rtsp://127.0.0.1:8554/video1'),
                                                     #('rtsp_thermal', str, 'rtsp://127.0.0.1:8554/video2'),
                                                     ('fps_thermal', int, 20),
                                                     ('fps_rgb', int, 20),
                                                     ('logfile', str, 'SIYI_log.bin'),
                                                     ('thermal_fov', float, 24.2),
                                                     ('zoom_fov', float, 62.0),
                                                     ('wide_fov', float, 88.0),
                                                     ('use_lidar', int, 0),
                                                     ('use_encoders', int, 0),
                                                     ('max_rate', float, 30.0),
                                                     ('track_size_pct', float, 5.0),
                                                     ('threshold_temp', int, 50),
                                                     ('threshold_min', int, 240),
                                                     ('los_correction', int, 0),
                                                     ('att_control', int, 0),
                                                     ('therm_cap_rate', float, 0),
                                                     ('show_horizon', int, 0),
                                                     ('track_ROI', int, 1),
                                                     MPSetting('thresh_climit', int, 50, range=(10,50)),
                                                     MPSetting('thresh_volt', int, 80, range=(20,80)),
                                                     MPSetting('thresh_ang', int, 4000, range=(30,4000)),
                                                     MPSetting('thresh_climit_dis', int, 20, range=(10,50)),
                                                     MPSetting('thresh_volt_dis', int, 40, range=(20,80)),
                                                     MPSetting('thresh_ang_dis', int, 40, range=(30,4000)),
                                                         ])
        self.add_completion_function('(SIYISETTING)',
                                     self.siyi_settings.completion)
        self.sock = None
        self.yaw_rate = None
        self.pitch_rate = None
        self.sequence = 0
        self.last_req_send = time.time()
        self.last_version_send = time.time()
        self.have_version = False
        self.console.set_status('SIYI', 'SIYI - -', row=6)
        self.console.set_status('TEMP', 'TEMP -/-', row=6)
        self.yaw_end = None
        self.pitch_end = None
        self.rf_dist = 0
        self.attitude = (0,0,0,0,0,0)
        self.encoders = (0,0,0)
        self.voltages = None
        self.tmax = -1
        self.tmin = -1
        self.spot_temp = -1
        self.tmax_x = None
        self.tmax_y = None
        self.tmin_x = None
        self.tmin_y = None
        self.last_temp_t = None
        self.last_att_t = time.time()
        self.att_dt_lpf = 1.0
        self.last_rf_t = None
        self.last_enc_t = None
        self.last_enc_recv_t = time.time()
        self.last_volt_t = None
        self.last_mode_t = time.time()
        self.last_thresh_t = None
        self.last_thresh_send_t = None
        self.GLOBAL_POSITION_INT = None
        self.ATTITUDE = None
        self.target_pos = None
        self.last_map_ROI = None
        self.icon = self.mpstate.map.icon('camera-small-red.png')
        self.click_icon = self.mpstate.map.icon('flag.png')
        self.last_target_send = time.time()
        self.last_rate_display = time.time()
        self.yaw_controller = PI_controller(self.siyi_settings, 'yaw_gain_P', 'yaw_gain_I', 'yaw_gain_IMAX')
        self.pitch_controller = PI_controller(self.siyi_settings, 'pitch_gain_P', 'pitch_gain_I', 'pitch_gain_IMAX')
        self.logf = DF_logger(os.path.join(self.logdir, self.siyi_settings.logfile))
        self.start_time = time.time()
        self.last_att_send_t = time.time()
        self.last_temp_t = time.time()
        self.thermal_view = None
        self.rgb_view = None
        self.last_zoom = 1.0
        self.rgb_lens = "wide"
        self.bad_crc = 0
        self.control_mode = -1
        self.last_SIEA = time.time()
        self.last_therm_cap = time.time()
        self.thermal_capture_count = 0
        self.last_therm_mode = time.time()

        self.recv_thread = Thread(target=self.receive_thread, name='SIYI_Receive')
        self.recv_thread.daemon = True
        self.recv_thread.start()
        self.have_horizon_lines = False
        self.thermal_param = None

        if mp_util.has_wxpython:
            menu = MPMenuSubMenu('SIYI',
                                 items=[
                                     MPMenuItem('Center', 'Center', '# siyi center '),
                                     MPMenuItem('ModeFollow', 'ModeFollow', '# siyi follow '),
                                     MPMenuItem('ModeLock', 'ModeLock', '# siyi lock '),
                                     MPMenuItem('ModeFPV', 'ModeFPV', '# siyi fpv '),
                                     MPMenuItem('GetConfig', 'GetConfig', '# siyi getconfig '),
                                     MPMenuItem('TakePhoto', 'TakePhoto', '# siyi photo '),
                                     MPMenuItem('AutoFocus', 'AutoFocus', '# siyi autofocus '),
                                     MPMenuItem('ImageSplit', 'ImageSplit', '# siyi imode split '),
                                     MPMenuItem('ImageWide', 'ImageWide', '# siyi imode wide '),
                                     MPMenuItem('ImageZoom', 'ImageZoom', '# siyi imode zoom '),
                                     MPMenuItem('Recording', 'Recording', '# siyi recording '),
                                     MPMenuItem('ClearTarget', 'ClearTarget', '# siyi notarget '),
                                     MPMenuItem('ThermalView', 'Thermalview', '# siyi thermal '),
                                     MPMenuItem('RGBView', 'RGBview', '# siyi rgbview '),
                                     MPMenuItem('ResetAttitude', 'ResetAttitude', '# siyi resetattitude '),
                                     MPMenuSubMenu('Zoom',
                                                   items=[MPMenuItem('Zoom%u'%z, 'Zoom%u'%z, '# siyi zoom %u ' % z) for z in range(1,11)]),
                                     MPMenuSubMenu('ThermalGain',
                                                   items=[MPMenuItem('HighGain', 'HighGain', '# siyi thermal_gain 1'),
                                                          MPMenuItem('LowGain', 'LowGain', '# siyi thermal_gain 0')]),

                                     MPMenuSubMenu('Threshold',
                                                  items=[MPMenuItem('Threshold%u'%z, 'Threshold%u'%z, '# siyi set threshold_temp %u ' % z) for z in range(20,115,5)])])
            map = self.module('map')
            if map is not None:
                map.add_menu(menu)
            console = self.module('console')
            if console is not None:
                console.add_menu(menu)

    def micros64(self):
        return int((time.time()-self.start_time)*1.0e6)

    def millis32(self):
        return int((time.time()-self.start_time)*1.0e3)
    
    def cmd_siyi(self, args):
        '''siyi command parser'''
        usage = "usage: siyi <set|rates>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.siyi_settings.command(args[1:])
        elif args[0] == "connect":
            self.cmd_connect()
        elif args[0] == "rates":
            self.cmd_rates(args[1:])
        elif args[0] == "yaw":
            self.cmd_yaw(args[1:])
        elif args[0] == "pitch":
            self.cmd_pitch(args[1:])
        elif args[0] == "imode":
            self.cmd_imode(args[1:])
        elif args[0] == "autofocus":
            self.send_packet_fmt(AUTO_FOCUS, "<B", 1)
        elif args[0] == "center":
            self.send_packet_fmt(CENTER, "<B", 1)
            self.clear_target()
        elif args[0] == "zoom":
            self.cmd_zoom(args[1:])
        elif args[0] == "getconfig":
            self.send_packet(ACQUIRE_GIMBAL_CONFIG_INFO, None)
        elif args[0] == "angle":
            self.cmd_angle(args[1:])
        elif args[0] == "photo":
            self.send_packet_fmt(PHOTO, "<B", 0)
        elif args[0] == "tempsnap":
            self.send_packet_fmt(GET_TEMP_FRAME, None)
        elif args[0] == "thermal_mode":
            self.send_packet_fmt(SET_THERMAL_MODE, "<B", int(args[1]))
        elif args[0] == "get_thermal_mode":
            self.send_packet_fmt(GET_THERMAL_MODE, None)
        elif args[0] == "get_thermal_gain":
            self.send_packet_fmt(GET_THERMAL_GAIN, None)
        elif args[0] == "thermal_gain":
            self.send_packet_fmt(SET_THERMAL_GAIN, "<B", int(args[1]))
        elif args[0] == "recording":
            self.send_packet_fmt(PHOTO, "<B", 2)
            self.send_packet(FUNCTION_FEEDBACK_INFO, None)
        elif args[0] == "resetattitude":
            self.send_packet(RESET_ATTITUDE, None)
        elif args[0] == "lock":
            self.send_packet_fmt(PHOTO, "<B", 3)
        elif args[0] == "follow":
            self.send_packet_fmt(PHOTO, "<B", 4)
            self.clear_target()
        elif args[0] == "fpv":
            self.send_packet_fmt(PHOTO, "<B", 5)
            self.clear_target()
        elif args[0] == "settarget":
            self.cmd_settarget(args[1:])
        elif args[0] == "notarget":
            self.clear_target()
        elif args[0] == "palette":
            self.cmd_palette(args[1:])
        elif args[0] == "thermal":
            self.cmd_thermal()
        elif args[0] == "rgbview":
            self.cmd_rgbview()
        elif args[0] == "therm_getenv":
            self.send_packet_fmt(GET_THERMAL_PARAM, None)
        elif args[0] == "therm_set_distance":
            self.therm_set_distance(float(args[1]))
        elif args[0] == "therm_set_humidity":
            self.therm_set_humidity(float(args[1]))
        elif args[0] == "therm_set_emissivity":
            self.therm_set_emissivity(float(args[1]))
        elif args[0] == "therm_set_airtemp":
            self.therm_set_airtemp(float(args[1]))
        elif args[0] == "therm_set_reftemp":
            self.therm_set_reftemp(float(args[1]))
        elif args[0] == "therm_getswitch":
            self.send_packet_fmt(GET_THERMAL_ENVSWITCH, None)
        elif args[0] == "therm_setswitch":
            self.send_packet_fmt(SET_THERMAL_ENVSWITCH, "<B", int(args[1]))
        elif args[0] == "settime":
            self.cmd_settime()
        else:
            print(usage)

    def cmd_connect(self):
        '''connect to the camera'''
        self.sock = None
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.connect((self.siyi_settings.ip, self.siyi_settings.port))
        sock.setblocking(True)
        self.sock = sock
        print("Connected to SIYI")

    def cmd_rates(self, args):
        '''update rates'''
        if len(args) < 2:
            print("Usage: siyi rates PAN_RATE PITCH_RATE")
            return
        self.clear_target()
        self.yaw_rate = float(args[0])
        self.pitch_rate = float(args[1])

    def cmd_yaw(self, args):
        '''update yaw'''
        if len(args) < 1:
            print("Usage: siyi yaw ANGLE")
            return
        angle = float(args[0])
        self.yaw_rate = self.siyi_settings.yaw_rate
        self.yaw_end = time.time() + abs(angle)/self.yaw_rate
        if angle < 0:
            self.yaw_rate = -self.yaw_rate

    def cmd_pitch(self, args):
        '''update pitch'''
        if len(args) < 1:
            print("Usage: siyi pitch ANGLE")
            return
        angle = float(args[0])
        self.pitch_rate = self.siyi_settings.pitch_rate
        self.pitch_end = time.time() + abs(angle)/self.pitch_rate
        if angle < 0:
            self.pitch_rate = -self.pitch_rate

    def cmd_imode(self, args):
        '''update image mode'''
        if len(args) < 1:
            print("Usage: siyi imode MODENUM")
            return
        imode_map = { "wide" : 5, "zoom" : 3, "split" : 2 }
        self.rgb_lens = args[0]
        mode = imode_map.get(self.rgb_lens,None)
        if mode is None:
            mode = int(args[0])
        self.send_packet_fmt(SET_IMAGE_TYPE, "<B", mode)
        print("Lens: %s" % args[0])

    def cmd_palette(self, args):
        '''update thermal palette'''
        if len(args) < 1:
            print("Usage: siyi palette PALETTENUM")
            return
        pal_map = { "WhiteHot" : 0, "Sepia" : 2, "Ironbow" : 3, "Rainbow" : 4,
                    "Night" : 5, "Aurora" : 6, "RedHot" : 7, "Jungle" : 8 , "Medical" : 9,
                    "BlackHot" : 10, "GloryHot" : 11}
        pal = pal_map.get(args[0],None)
        if pal is None:
            pal = int(args[0])
        self.send_packet_fmt(SET_THERMAL_PALETTE, "<B", pal)

    def cmd_settime(self):
        '''set camera time'''
        t_us = int(time.time()*1.0e6)
        self.send_packet_fmt(SET_TIME, "<Q", t_us)

    def video_filename(self, base):
        '''get a video file name'''
        if self.logdir is None:
            return base + ".mts"
        i = 1
        while True:
            vidfile = os.path.join(self.logdir, "%s%u.mts" % (base,i))
            if not os.path.exists(vidfile):
                self.logf.write('SIVI', 'QBB', 'TimeUS,Type,Idx',
                                self.micros64(),
                                1 if base=='thermal' else 0,
                                i)
                break
            i += 1
        return vidfile, i

    def cmd_thermal(self):
        '''open thermal viewer'''
        vidfile,idx = self.video_filename('thermal')
        self.thermal_view = CameraView(self, self.siyi_settings.rtsp_thermal,
                                       vidfile, (640,512), thermal=True,
                                       fps=self.siyi_settings.fps_thermal,
                                       video_idx=idx)

    def cmd_rgbview(self):
        '''open rgb viewer'''
        vidfile,idx = self.video_filename('rgb')
        self.rgb_view = CameraView(self, self.siyi_settings.rtsp_rgb,
                                   vidfile, (1280,720), thermal=False,
                                   fps=self.siyi_settings.fps_rgb,
                                   video_idx=idx)

    def check_thermal_events(self):
        '''check for mouse events on thermal image'''
        if self.thermal_view is not None:
            self.thermal_view.check_events()
        if self.rgb_view is not None:
            self.rgb_view.check_events()

    def log_frame_counter(self, video_idx, thermal, frame_counter):
        '''log video frame counter'''
        self.logf.write('SIFC', 'QBBI', 'TimeUS,Type,Idx,Frame',
                        self.micros64(),
                        1 if thermal else 0,
                        video_idx,
                        frame_counter)

    def cmd_zoom(self, args):
        '''set zoom'''
        if len(args) < 1:
            print("Usage: siyi zoom ZOOM")
            return
        self.last_zoom = float(args[0])
        ival = int(self.last_zoom)
        frac = int((self.last_zoom - ival)*10)
        self.send_packet_fmt(ABSOLUTE_ZOOM, "<BB", ival, frac)

    def set_target(self, lat, lon, alt):
        '''set target position'''
        self.target_pos = (lat, lon, alt)
        self.mpstate.map.add_object(mp_slipmap.SlipIcon('SIYI',
                                                        (lat, lon),
                                                        self.icon, layer='SIYI', rotation=0, follow=False))

    def therm_set_distance(self, distance):
        '''set thermal distance'''
        if self.thermal_param is None:
            print("Run therm_getenv first")
            return
        p = copy.copy(self.thermal_param)
        p.distance = distance
        self.send_packet_fmt(SET_THERMAL_PARAM, "<HHHHH", *p.args())

    def therm_set_emissivity(self, emissivity):
        '''set thermal emissivity'''
        if self.thermal_param is None:
            print("Run therm_getenv first")
            return
        p = copy.copy(self.thermal_param)
        p.target_emissivity = emissivity
        self.send_packet_fmt(SET_THERMAL_PARAM, "<HHHHH", *p.args())

    def therm_set_humidity(self, humidity):
        '''set thermal humidity'''
        if self.thermal_param is None:
            print("Run therm_getenv first")
            return
        p = copy.copy(self.thermal_param)
        p.humidity = humidity
        self.send_packet_fmt(SET_THERMAL_PARAM, "<HHHHH", *p.args())

    def therm_set_airtemp(self, airtemp):
        '''set thermal airtemp'''
        if self.thermal_param is None:
            print("Run therm_getenv first")
            return
        p = copy.copy(self.thermal_param)
        p.air_temperature = airtemp
        self.send_packet_fmt(SET_THERMAL_PARAM, "<HHHHH", *p.args())

    def therm_set_reftemp(self, reftemp):
        '''set thermal reftemp'''
        if self.thermal_param is None:
            print("Run therm_getenv first")
            return
        p = copy.copy(self.thermal_param)
        p.reflection_temperature = reftemp
        self.send_packet_fmt(SET_THERMAL_PARAM, "<HHHHH", *p.args())
        
    def clear_target(self):
        '''clear target position'''
        self.target_pos = None
        self.mpstate.map.remove_object('SIYI')
        self.end_tracking()
        self.yaw_rate = None
        self.pitch_rate = None

    def cmd_angle(self, args):
        '''set zoom'''
        if len(args) < 1:
            print("Usage: siyi angle YAW PITCH")
            return
        yaw = -float(args[0])
        pitch = float(args[1])
        self.target_pos = None
        self.clear_target()
        self.send_packet_fmt(SET_ANGLE, "<hh", int(yaw*10), int(pitch*10))
        
    def send_rates(self):
        '''send rates packet'''
        now = time.time()
        if self.siyi_settings.rates_hz <= 0 or now - self.last_req_send < 1.0/self.siyi_settings.rates_hz:
            return
        self.last_req_send = now
        if self.yaw_rate is not None and self.pitch_rate is not None:
            y = rate_mapping(self.yaw_rate)
            p = rate_mapping(self.pitch_rate)
            y = mp_util.constrain(y, -self.siyi_settings.max_rate, self.siyi_settings.max_rate)
            p = mp_util.constrain(p, -self.siyi_settings.max_rate, self.siyi_settings.max_rate)
            scale = 1.0
            y = int(mp_util.constrain(y*scale, -100, 100))
            p = int(mp_util.constrain(p*scale, -100, 100))
            self.send_packet_fmt(GIMBAL_ROTATION, "<bb", y, p)
            self.logf.write('SIGR', 'Qffbb', 'TimeUS,YRate,PRate,YC,PC',
                            self.micros64(), self.yaw_rate, self.pitch_rate, y, p)

            self.send_named_float('YAW_RT', self.yaw_rate)
            self.send_named_float('PITCH_RT', self.pitch_rate)

    def cmd_settarget(self, args):
        '''set target'''
        click = self.mpstate.click_location
        if click is None:
            print("No map click position available")
            return
        lat = click[0]
        lon = click[1]
        alt = self.module('terrain').ElevationModel.GetElevation(lat, lon)
        if alt is None:
            print("No terrain for location")
            return
        self.set_target(lat, lon, alt)

    def request_telem(self):
        '''request telemetry'''
        now = time.time()
        if self.siyi_settings.temp_hz > 0 and now - self.last_temp_t >= 1.0/self.siyi_settings.temp_hz:
            self.last_temp_t = now
            self.send_packet_fmt(READ_TEMP_FULL_SCREEN, "<B", 2)
        if self.last_att_t is None or now - self.last_att_t > 5:
            self.last_att_t = now
            self.send_packet_fmt(REQUEST_CONTINUOUS_DATA, "<BB", 1, self.siyi_settings.telem_rate)
        if self.last_rf_t is None or now - self.last_rf_t > 10:
            self.last_rf_t = now
            self.send_packet_fmt(REQUEST_CONTINUOUS_DATA, "<BB", 2, 1)
        if self.last_enc_t is None or now - self.last_enc_t > 5:
            self.last_enc_t = now
            self.send_packet_fmt(REQUEST_CONTINUOUS_DATA, "<BB", 3, self.siyi_settings.telem_rate)
        if self.last_volt_t is None or now - self.last_volt_t > 5:
            self.last_volt_t = now
            self.send_packet_fmt(REQUEST_CONTINUOUS_DATA, "<BB", 4, self.siyi_settings.telem_rate)
        if self.last_thresh_t is None or now - self.last_thresh_t > 10:
            self.last_thresh_t = now
            self.send_packet_fmt(READ_THRESHOLDS, None)
        if self.siyi_settings.mode_hz > 0 and now - self.last_mode_t > 1.0/self.siyi_settings.mode_hz:
            self.last_mode_t = now
            self.send_packet_fmt(READ_CONTROL_MODE, None)
        if self.siyi_settings.therm_cap_rate > 0 and now - self.last_therm_mode > 2:
            self.last_therm_mode = now
            self.send_packet_fmt(GET_THERMAL_MODE, None)

    def send_attitude(self):
        '''send attitude to gimbal'''
        now = time.time()
        if self.siyi_settings.att_send_hz <= 0 or now - self.last_att_send_t < 1.0/self.siyi_settings.att_send_hz:
            return
        self.last_att_send_t = now
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return
        self.send_packet_fmt(ATTITUDE_EXTERNAL, "<Iffffff",
                             self.millis32(),
                             att.roll, att.pitch, att.yaw,
                             att.rollspeed, att.pitchspeed, att.yawspeed)


    def send_packet(self, command_id, pkt):
        '''send SIYI packet'''
        plen = len(pkt) if pkt else 0
        buf = struct.pack("<BBBHHB", SIYI_HEADER1, SIYI_HEADER2, 1, plen,
                          self.sequence, command_id)
        if pkt:
            buf += pkt
        buf += struct.pack("<H", crc16_from_bytes(buf))
        self.sequence = (self.sequence+1) % 0xffff
        try:
            self.sock.send(buf)
        except Exception:
            pass

    def send_packet_fmt(self, command_id, fmt, *args):
        '''send SIYI packet'''
        if fmt is None:
            fmt = ""
            args = []
        self.send_packet(command_id, struct.pack(fmt, *args))
        args = list(args)
        args.extend([0]*(8-len(args)))
        self.logf.write('SIOU', 'QBffffffff', 'TimeUS,Cmd,P1,P2,P3,P4,P5,P6,P7,P8', self.micros64(), command_id, *args)

    def unpack(self, command_id, fmt, data):
        '''unpack SIYI data and log'''
        fsize = struct.calcsize(fmt)
        if fsize != len(data):
            print("cmd 0x%02x needs %u bytes got %u" % (command_id, fsize, len(data)))
            return None
        v = struct.unpack(fmt, data[:fsize])
        args = list(v)
        args.extend([0]*(12-len(args)))
        self.logf.write('SIIN', 'QBffffffffffff', 'TimeUS,Cmd,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12', self.micros64(), command_id, *args)
        return v

    def parse_data(self, pkt):
        '''parse SIYI packet'''
        while len(pkt) >= 10:
            (h1,h2,rack,plen,seq,cmd) = struct.unpack("<BBBHHB", pkt[:8])
            if plen+10 > len(pkt):
                #print("SIYI: short packet", plen+10, len(pkt))
                break
            self.parse_packet(pkt[:plen+10])
            pkt = pkt[plen+10:]

    def parse_packet(self, pkt):
        '''parse SIYI packet'''
        (h1,h2,rack,plen,seq,cmd) = struct.unpack("<BBBHHB", pkt[:8])
        data = pkt[8:-2]
        crc, = struct.unpack("<H", pkt[-2:])
        crc2 = crc16_from_bytes(pkt[:-2])
        if crc != crc2:
            self.bad_crc += 1
            #print("SIYI: BAD CRC", crc, crc2, self.bad_crc)
            return

        if cmd == ACQUIRE_FIRMWARE_VERSION:
            patch,minor,major,gpatch,gminor,gmajor,zpatch,zminor,zmajor,_,_,_ = self.unpack(cmd, "<BBBBBBBBBBBB", data)
            self.have_version = True
            print("SIYI CAM %u.%u.%u" % (major, minor, patch))
            print("SIYI Gimbal %u.%u.%u" % (gmajor, gminor, gpatch))
            print("SIYI Zoom %u.%u.%u" % (zmajor, zminor, zpatch))
            # change to white hot
            self.send_packet_fmt(SET_THERMAL_PALETTE, "<B", 0)

        elif cmd == ACQUIRE_GIMBAL_ATTITUDE:
            (z,y,x,sz,sy,sx) = self.unpack(cmd, "<hhhhhh", data)
            now = time.time()
            dt = now - self.last_att_t
            self.att_dt_lpf = 0.95 * self.att_dt_lpf + 0.05 * max(dt,0.01)
            self.last_att_t = now
            (roll,pitch,yaw) = (x*0.1, y*0.1, mp_util.wrap_180(-z*0.1))
            self.attitude = (roll,pitch,yaw, sx*0.1, sy*0.1, -sz*0.1)
            self.send_named_float('CROLL', self.attitude[0])
            self.send_named_float('CPITCH', self.attitude[1])
            self.send_named_float('CYAW', self.attitude[2])
            self.send_named_float('CROLL_RT', self.attitude[3])
            self.send_named_float('CPITCH_RT', self.attitude[4])
            self.send_named_float('CYAW_RT', self.attitude[5])
            self.update_status()
            self.logf.write('SIGA', 'Qffffffhhhhhh', 'TimeUS,Y,P,R,Yr,Pr,Rr,z,y,x,sz,sy,sx',
                            self.micros64(),
                                self.attitude[2], self.attitude[1], self.attitude[0],
                                self.attitude[5], self.attitude[4], self.attitude[3],
                                z,y,x,sz,sy,sx)

        elif cmd == ACQUIRE_GIMBAL_CONFIG_INFO:
            res, hdr_sta, res2, record_sta, gim_motion, gim_mount, video = self.unpack(cmd, "<BBBBBBB", data)
            print("HDR: %u" % hdr_sta)
            print("Recording: %u" % record_sta)
            print("GimbalMotion: %u" % gim_motion)
            print("GimbalMount: %u" % gim_mount)
            print("Video: %u" % video)

        elif cmd == READ_RANGEFINDER:
            r, = self.unpack(cmd, "<H", data)
            self.rf_dist = r * 0.1
            self.last_rf_t = time.time()
            self.update_status()
            self.send_named_float('RFND', self.rf_dist)
            SR = self.get_slantrange(0,0,0,1)
            if SR is None:
                SR = -1.0
            self.logf.write('SIRF', 'Qff', 'TimeUS,Dist,SR',
                            self.micros64(),
                            self.rf_dist,
                            SR)

        elif cmd == READ_ENCODERS:
            y,p,r, = self.unpack(cmd, "<hhh", data)
            self.last_enc_t = time.time()
            self.last_enc_recv_t = time.time()
            self.encoders = (r*0.1,p*0.1,-y*0.1)
            self.send_named_float('ENC_R', self.encoders[0])
            self.send_named_float('ENC_P', self.encoders[1])
            self.send_named_float('ENC_Y', self.encoders[2])
            self.logf.write('SIEN', 'Qfff', 'TimeUS,R,P,Y',
                            self.micros64(),
                            self.encoders[0], self.encoders[1], self.encoders[2])
            if self.siyi_settings.show_horizon == 1:
                self.have_horizon_lines = True
                self.show_horizon_lines()
            elif self.have_horizon_lines:
                self.remove_horizon_lines()
                self.have_horizon_lines = False

        elif cmd == READ_VOLTAGES:
            y,p,r,mode,mode_ms, = self.unpack(cmd, "<hhhBI", data)
            self.last_volt_t = time.time()
            self.voltages = (r*0.001,p*0.001,y*0.001)
            self.send_named_float('VLT_R', self.voltages[0])
            self.send_named_float('VLT_P', self.voltages[1])
            self.send_named_float('VLT_Y', self.voltages[2])
            self.logf.write('SIVL', 'QfffBI', 'TimeUS,R,P,Y,Mode,ModeMS',
                            self.micros64(),
                            self.voltages[0], self.voltages[1], self.voltages[2],
                            mode, mode_ms)
            self.control_mode = mode

        elif cmd == READ_THRESHOLDS:
            climit,volt_thresh,ang_thresh, = self.unpack(cmd, "<hhh", data)
            self.last_thresh_t = time.time()
            self.send_named_float('CLIMIT', climit)
            self.send_named_float('VTHRESH', volt_thresh)
            self.send_named_float('ATHRESH', ang_thresh)
            self.logf.write('SITH', 'Qhhh', 'TimeUS,WLimit,VThresh,AErr',
                            self.micros64(),
                            climit, volt_thresh, ang_thresh)
            if self.master.motors_armed():
                new_thresh = (self.siyi_settings.thresh_climit,
                              self.siyi_settings.thresh_volt,
                              self.siyi_settings.thresh_ang)
                weak_control = 0
            else:
                new_thresh = (self.siyi_settings.thresh_climit_dis,
                              self.siyi_settings.thresh_volt_dis,
                              self.siyi_settings.thresh_ang_dis)
                weak_control = 1
            if (climit != new_thresh[0] or volt_thresh != new_thresh[1] or ang_thresh != new_thresh[2]):
                print("SIYI: Setting thresholds (%u,%u,%u) -> (%u,%u,%u)" %
                      (climit,volt_thresh,ang_thresh,new_thresh[0],new_thresh[1],new_thresh[2]))
                self.send_packet_fmt(SET_THRESHOLDS, "<hhh",
                                     new_thresh[0], new_thresh[1], new_thresh[2])
                self.send_packet_fmt(SET_WEAK_CONTROL,"<B", weak_control)

        elif cmd == READ_CONTROL_MODE:
            self.control_mode, = self.unpack(cmd, "<B", data)
            self.send_named_float('CMODE', self.control_mode)
            self.logf.write('SIMO', 'QB', 'TimeUS,Mode',
                            self.micros64(), self.control_mode)

        elif cmd == READ_TEMP_FULL_SCREEN:
            if len(data) < 12:
                print("READ_TEMP_FULL_SCREEN: Expected 12 bytes, got %u" % len(data))
                return
            self.tmax,self.tmin,self.tmax_x,self.tmax_y,self.tmin_x,self.tmin_y = self.unpack(cmd, "<HHHHHH", data)
            self.tmax = self.tmax * 0.01
            self.tmin = self.tmin * 0.01
            self.send_named_float('TMIN', self.tmin)
            self.send_named_float('TMAX', self.tmax)
            self.last_temp_t = time.time()
            frame_counter = -1 if self.thermal_view is None else self.thermal_view.frame_counter
            self.logf.write('SITR', 'QffHHHHi', 'TimeUS,TMin,TMax,TMinX,TMinY,TMaxX,TMaxY,FC',
                            self.micros64(),
                            self.tmin, self.tmax,
                            self.tmin_x, self.tmin_y,
                            self.tmax_x, self.tmax_y,
                            frame_counter)
            if self.thermal_view is not None:
                threshold = self.siyi_settings.threshold_temp
                threshold_value = int(255*(threshold - self.tmin)/max(1,(self.tmax-self.tmin)))
                threshold_value = mp_util.constrain(threshold_value, 0, 255)
                if self.tmax < threshold:
                    threshold_value = -1
                else:
                    threshold_value = max(threshold_value, self.siyi_settings.threshold_min)
                self.thermal_view.set_threshold(threshold_value)
        elif cmd == FUNCTION_FEEDBACK_INFO:
            info_type, = self.unpack(cmd, "<B", data)
            feedback = {
                0: "Success",
                1: "FailPhoto",
                2: "HDR ON",
                3: "HDR OFF",
                4: "FailRecord",
            }
            print("Feedback %s" % feedback.get(info_type, str(info_type)))
        elif cmd == SET_THRESHOLDS:
            ok, = self.unpack(cmd, "<B", data)
            if ok != 1:
                print("Threshold set failure")
        elif cmd == SET_WEAK_CONTROL:
            ok,weak_control, = self.unpack(cmd, "<BB", data)
            if ok != 1:
                print("Weak control set failure")
            else:
                print("Weak control is %u" % weak_control)
        elif cmd == GET_THERMAL_MODE:
            ok, = self.unpack(cmd,"<B", data)
            if self.siyi_settings.therm_cap_rate > 0 and ok != 1:
                print("ThermalMode: %u" % ok)
                self.send_packet_fmt(SET_THERMAL_MODE, "<B", 1)

        elif cmd == SET_THERMAL_MODE:
            ok, = self.unpack(cmd,"<B", data)
            print("SetThermalMode: %u" % ok)

        elif cmd == SET_THERMAL_GAIN:
            ok, = self.unpack(cmd,"<B", data)
            print("SetThermalGain: %u" % ok)

        elif cmd == GET_TEMP_FRAME:
            ok, = self.unpack(cmd,"<B", data)
            if ok:
                self.thermal_capture_count += 1

        elif cmd == GET_THERMAL_ENVSWITCH:
            ok, = self.unpack(cmd,"<B", data)
            print("ThermalEnvSwitch: %u" % ok)

        elif cmd == SET_THERMAL_ENVSWITCH:
            ok, = self.unpack(cmd,"<B", data)
            print("ThermalEnvSwitch: %u" % ok)

        elif cmd == SET_THERMAL_PARAM:
            ok, = self.unpack(cmd,"<B", data)
            print("SetThermalParam: %u" % ok)
            
        elif cmd == GET_THERMAL_PARAM:
            dist,emiss,humidity,airtemp,reftemp, = self.unpack(cmd,"<HHHHH", data)
            self.thermal_param = ThermalParameters(dist*0.01, emiss*0.01, humidity*0.01, airtemp*0.01, reftemp*0.01)
            print("ThermalParam: %s" % self.thermal_param)

        elif cmd == SET_TIME:
            ok, = self.unpack(cmd,"<B", data)
            print("SetTime: %u" % ok)
            
        elif cmd in [SET_ANGLE, CENTER, GIMBAL_ROTATION, ABSOLUTE_ZOOM, SET_IMAGE_TYPE,
                     REQUEST_CONTINUOUS_DATA, SET_THERMAL_PALETTE, MANUAL_ZOOM_AND_AUTO_FOCUS]:
            # an ack
            pass
        else:
            print("SIYI: Unknown command 0x%02x" % cmd)

    def update_title(self):
        '''update thermal view title'''
        if self.thermal_view is not None:
            self.thermal_view.update_title()
        if self.rgb_view is not None:
            self.rgb_view.update_title()

    def update_status(self):
        if self.attitude is None:
            return
        (r,p,y) = self.get_gimbal_attitude()
        self.console.set_status('SIYI', 'SIYI (%.1f,%.1f,%.1f) %.1fHz SR=%.1f M=%d' % (
            r,p,y,
            1.0/self.att_dt_lpf,
            self.rf_dist, self.control_mode), row=6)
        if self.tmin is not None:
            self.console.set_status('TEMP', 'TEMP %.2f/%.2f' % (self.tmin, self.tmax), row=6)
            self.update_title()
        self.console.set_status('TCAP', 'TCAP %u' % self.thermal_capture_count, row=6)


    def check_rate_end(self):
        '''check for ending yaw/pitch command'''
        now = time.time()
        if self.yaw_end is not None and now >= self.yaw_end:
            self.yaw_rate = 0
            self.yaw_end = None
        if self.pitch_end is not None and now >= self.pitch_end:
            self.pitch_rate = 0
            self.pitch_end = None

    def send_named_float(self, name, value):
        '''inject a NAMED_VALUE_FLOAT into the local master input, so it becomes available
           for graphs, logging and status command'''

        # use the ATTITUDE message for srcsystem and time stamps
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return
        msec = att.time_boot_ms
        ename = name.encode('ASCII')
        if len(ename) < 10:
            ename += bytes([0] * (10-len(ename)))
        m = self.master.mav.named_value_float_encode(msec, bytearray(ename), value)
        #m.name = ename
        m.pack(self.master.mav)
        m._header.srcSystem = att._header.srcSystem
        m._header.srcComponent = mavutil.mavlink.MAV_COMP_ID_TELEMETRY_RADIO
        m.name = name
        self.mpstate.module('link').master_callback(m, self.master)

    def get_encoder_attitude(self):
        '''get attitude from encoders in vehicle frame'''
        now = time.time()
        att = self.master.messages.get('ATTITUDE',None)
        if now - self.last_enc_recv_t > 2.0 or att is None:
            return None
        m = Matrix3()
        # we use zero yaw as this is in vehicle frame
        m.from_euler(att.roll,att.pitch,0)
        m.rotate_312(radians(self.encoders[0]),radians(self.encoders[1]),radians(self.encoders[2]))
        (r,p,y) = m.to_euler()
        return degrees(r),degrees(p),mp_util.wrap_180(degrees(y))

    def get_direct_attitude(self):
        '''get extrapolated gimbal attitude, returning r,p,y in degrees in vehicle frame'''
        now = time.time()
        dt = (now - self.last_att_t)+self.siyi_settings.lag
        dt = max(dt,0)
        yaw = self.attitude[2]+self.attitude[5]*dt
        pitch = self.attitude[1]+self.attitude[4]*dt
        yaw = mp_util.wrap_180(yaw)
        roll = self.attitude[0]
        return roll, pitch, yaw

    def get_gimbal_attitude(self):
        '''get extrapolated gimbal attitude, returning yaw and pitch'''
        ret = self.get_encoder_attitude()
        if ret is not None:
            (r,p,y) = ret
            now = time.time()
            if now - self.last_SIEA >= 0.2:
                self.last_SIEA = now
                self.logf.write('SIEA', 'Qfff', 'TimeUS,R,P,Y',
                                self.micros64(),
                                r,p,y)
                self.send_named_float('EA_R', r)
                self.send_named_float('EA_P', p)
                self.send_named_float('EA_Y', y)
        if self.siyi_settings.use_encoders:
            if ret is not None:
                return r,p,y
        return self.get_direct_attitude()

    def get_fov_attitude(self):
        '''get attitude for FOV calculations'''
        (r,p,y) = self.get_gimbal_attitude()
        return (r,p-self.siyi_settings.mount_pitch,mp_util.wrap_180(y-self.siyi_settings.mount_yaw))

    def get_slantrange(self,x,y,FOV,aspect_ratio):
        '''
         get range to ground
         x and y are from -1 to 1, relative to center of camera view
        '''
        if self.rf_dist > 0 and self.siyi_settings.use_lidar > 0:
            # use rangefinder if enabled
            return self.rf_dist
        gpi = self.master.messages.get('GLOBAL_POSITION_INT',None)
        if not gpi:
            return None
        (lat,lon,alt) = gpi.lat*1.0e-7,gpi.lon*1.0e-7,gpi.alt*1.0e-3
        ground_alt = self.module('terrain').ElevationModel.GetElevation(lat, lon)
        if alt <= ground_alt:
            return None
        if self.attitude is None:
            return None
        fov_att = self.get_fov_attitude()
        pitch = fov_att[1]
        if pitch >= 0:
            return None
        pitch -= y*FOV*0.5/aspect_ratio
        pitch = min(pitch, -1)

        # start with flat earth
        sin_pitch = math.sin(abs(math.radians(pitch)))
        sr = (alt-ground_alt) / sin_pitch

        # iterate to make more accurate
        for i in range(3):
            (lat2,lon2,alt2) = self.get_latlonalt(sr,x,y,FOV,aspect_ratio)
            ground_alt2 = self.module('terrain').ElevationModel.GetElevation(lat2, lon2)
            if ground_alt2 is None:
                return None
            # adjust for height at this point
            sr += (alt2 - ground_alt2) / sin_pitch
        return sr



    def get_view_vector(self, x, y, FOV, aspect_ratio):
        '''
        get ground lat/lon given vehicle orientation, camera orientation and slant range
        x and y are from -1 to 1, relative to center of camera view
        positive x is to the right
        positive y is down
        '''
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return None
        v = Vector3(1, 0, 0)
        m = Matrix3()
        fov_att = self.get_fov_attitude()
        (roll,pitch,yaw) = (math.radians(fov_att[0]),math.radians(fov_att[1]),math.radians(fov_att[2]))
        yaw += att.yaw
        FOV_half = math.radians(0.5*FOV)
        yaw += FOV_half*x
        pitch -= y*FOV_half/aspect_ratio
        m.from_euler(roll, pitch, yaw)
        v = m * v
        return v

    def get_latlonalt(self, slant_range, x, y, FOV, aspect_ratio):
        '''
        get ground lat/lon given vehicle orientation, camera orientation and slant range
        x and y are from -1 to 1, relative to center of camera view
        '''
        if slant_range is None:
            return None
        v = self.get_view_vector(x,y,FOV,aspect_ratio)
        if v is None:
            return None
        gpi = self.master.messages.get('GLOBAL_POSITION_INT',None)
        if gpi is None:
            return None
        v *= slant_range
        (lat,lon,alt) = (gpi.lat*1.0e-7,gpi.lon*1.0e-7,gpi.alt*1.0e-3)
        (lat,lon) = mp_util.gps_offset(lat,lon,v.y,v.x)
        return (lat,lon,alt-v.z)

    def get_target_yaw_pitch(self, lat, lon, alt, mylat, mylon, myalt, vehicle_yaw_rad):
        '''get target yaw/pitch in vehicle frame for a target lat/lon'''
        GPS_vector_x = (lon-mylon)*1.0e7*math.cos(math.radians((mylat + lat) * 0.5)) * 0.01113195
        GPS_vector_y = (lat - mylat) * 0.01113195 * 1.0e7
        GPS_vector_z = alt - myalt # was cm
        target_distance = math.sqrt(GPS_vector_x**2 + GPS_vector_y**2)

        # calculate pitch, yaw angles
        pitch = math.atan2(GPS_vector_z, target_distance)
        yaw = math.atan2(GPS_vector_x, GPS_vector_y)
        yaw -= vehicle_yaw_rad
        yaw_deg = mp_util.wrap_180(math.degrees(yaw))
        pitch_deg = math.degrees(pitch)
        return yaw_deg, pitch_deg
    
    def update_target(self):
        '''update position targetting'''
        if not 'GLOBAL_POSITION_INT' in self.master.messages or not 'ATTITUDE' in self.master.messages:
            return

        # added rate of target update

        map_module = self.module('map')
        if self.siyi_settings.track_ROI == 1 and map_module is not None and map_module.current_ROI != self.last_map_ROI:
            self.last_map_ROI = map_module.current_ROI
            (lat, lon, alt) = self.last_map_ROI
            self.clear_target()
            self.set_target(lat, lon, alt)

        if self.target_pos is None or self.attitude is None:
            return

        now = time.time()
        if self.siyi_settings.target_rate <= 0 or now - self.last_target_send < 1.0 / self.siyi_settings.target_rate:
            return
        self.last_target_send = now

        GLOBAL_POSITION_INT = self.master.messages['GLOBAL_POSITION_INT']
        ATTITUDE = self.master.messages['ATTITUDE']
        lat, lon, alt = self.target_pos
        mylat = GLOBAL_POSITION_INT.lat*1.0e-7
        mylon = GLOBAL_POSITION_INT.lon*1.0e-7
        myalt = GLOBAL_POSITION_INT.alt*1.0e-3

        dt = now - GLOBAL_POSITION_INT._timestamp
        vn = GLOBAL_POSITION_INT.vx*0.01
        ve = GLOBAL_POSITION_INT.vy*0.01
        vd = GLOBAL_POSITION_INT.vz*0.01
        (mylat, mylon) = mp_util.gps_offset(mylat, mylon, ve*dt, vn*dt)
        myalt -= vd*dt

        dt = now - ATTITUDE._timestamp
        vehicle_yaw_rad = ATTITUDE.yaw + ATTITUDE.yawspeed*dt

        yaw_deg, pitch_deg = self.get_target_yaw_pitch(lat, lon, alt, mylat, mylon, myalt, vehicle_yaw_rad)

        self.send_named_float('TYAW', yaw_deg)
        self.send_named_float('TPITCH', pitch_deg)
        
        if self.siyi_settings.att_control == 1:
            self.yaw_rate = None
            self.pitch_rate = None
            self.send_packet_fmt(SET_ANGLE, "<hh", int(-yaw_deg*10), int(pitch_deg*10))
            return

        if self.siyi_settings.los_correction == 1:
            GLOBAL_POSITION_INT = self.master.messages['GLOBAL_POSITION_INT']
            (mylat2, mylon2) = mp_util.gps_offset(mylat, mylon, ve*(dt+1), vn*(dt+1))
            vehicle_yaw_rad2 = vehicle_yaw_rad + ATTITUDE.yawspeed
            yaw_deg2, pitch_deg2 = self.get_target_yaw_pitch(lat, lon, alt, mylat2, mylon2, myalt, vehicle_yaw_rad2)

            los_yaw_rate = mp_util.wrap_180(yaw_deg2 - yaw_deg)
            los_pitch_rate = pitch_deg2 - pitch_deg
        else:
            los_yaw_rate = 0.0
            los_pitch_rate = 0.0
        #print(los_yaw_rate, los_pitch_rate)

        cam_roll, cam_pitch, cam_yaw = self.get_gimbal_attitude()
        err_yaw = mp_util.wrap_180(yaw_deg - cam_yaw)
        err_pitch = pitch_deg - cam_pitch

        err_yaw += self.siyi_settings.mount_yaw
        err_yaw = mp_util.wrap_180(err_yaw)
        err_pitch += self.siyi_settings.mount_pitch

        self.yaw_rate = self.yaw_controller.run(err_yaw, los_yaw_rate)
        self.pitch_rate = self.yaw_controller.run(err_pitch, los_pitch_rate)
        self.send_named_float('EYAW', err_yaw)
        self.send_named_float('EPITCH', err_pitch)
        self.logf.write('SIPY', "Qfffff", "TimeUS,CYaw,TYaw,Yerr,I,FF",
                        self.micros64(), cam_yaw, yaw_deg, err_yaw, self.yaw_controller.I, los_yaw_rate)
        self.logf.write('SIPP', "Qfffff", "TimeUS,CPitch,TPitch,Perr,I,FF",
                        self.micros64(), cam_pitch, pitch_deg, err_pitch, self.pitch_controller.I, los_pitch_rate)

    def show_fov1(self, FOV, name, aspect_ratio, color):
        '''show one FOV polygon'''
        points = []
        for (x,y) in [(-1,-1),(1,-1),(1,1),(-1,1),(-1,-1)]:
            latlonalt = self.get_latlonalt(self.get_slantrange(x,y,FOV,aspect_ratio),x,y,FOV,aspect_ratio)
            if latlonalt is None:
                self.mpstate.map.remove_object(name)
                return
            (lat,lon) = (latlonalt[0],latlonalt[1])
            points.append((lat,lon))
        self.mpstate.map.add_object(mp_slipmap.SlipPolygon(name, points, layer='SIYI',
                                                           linewidth=2, colour=color))

    def show_fov(self):
        '''show FOV polygons'''
        self.show_fov1(self.siyi_settings.thermal_fov, 'FOV_thermal', 640.0/512.0, (0,0,128))
        FOV2 = self.siyi_settings.wide_fov
        if self.rgb_lens == "zoom":
            FOV2 = self.siyi_settings.zoom_fov / self.last_zoom
        self.show_fov1(FOV2, 'FOV_RGB', 1280.0/720.0, (0,128,128))

    def end_tracking(self):
        '''end all tracking'''
        if self.rgb_view is not None:
            self.rgb_view.end_tracking()
        if self.thermal_view is not None:
            self.thermal_view.end_tracking()

    def mavlink_packet(self, m):
        '''process a mavlink message'''
        mtype = m.get_type()
        if mtype == 'GPS_RAW_INT':
            # ?!? why off by 18 hours
            gwk, gms = mp_util.get_gps_time(time.time()+18*3600)
            self.logf.write('GPS', "QBIHLLff", "TimeUS,Status,GMS,GWk,Lat,Lng,Alt,Spd",
                            self.micros64(), m.fix_type, gms, gwk, m.lat, m.lon, m.alt*0.001, m.vel*0.01)
        if mtype == 'ATTITUDE':
            self.logf.write('ATT', "Qffffff", "TimeUS,Roll,Pitch,Yaw,GyrX,GyrY,GyrZ",
                            self.micros64(),
                            math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw),
                            math.degrees(m.rollspeed), math.degrees(m.pitchspeed), math.degrees(m.yawspeed))
        if mtype == 'GLOBAL_POSITION_INT':
            try:
                self.show_fov()
            except Exception as ex:
                print(traceback.format_exc())

    def receive_thread(self):
        '''thread for receiving UDP packets from SIYI'''
        while True:
            if self.sock is None:
                time.sleep(0.1)
                continue
            try:
                pkt = self.sock.recv(10240)
            except Exception as ex:
                print("SIYI receive failed", ex)
                continue
            self.parse_data(pkt)

    def therm_capture(self):
        if self.siyi_settings.therm_cap_rate <= 0:
            return
        now = time.time()
        dt = now - self.last_therm_cap
        if dt > 5:
            self.send_packet_fmt(SET_THERMAL_MODE, "<B", 1)
        if dt > 1.0 / self.siyi_settings.therm_cap_rate:
            self.send_packet_fmt(GET_TEMP_FRAME, None)
            self.last_therm_cap = now

    def idle_task(self):
        '''called on idle'''
        if not self.sock:
            return
        if not self.have_version and time.time() - self.last_version_send > 2.0:
            self.last_version_send = time.time()
            self.send_packet(ACQUIRE_FIRMWARE_VERSION, None)
        self.check_rate_end()
        self.update_target()
        self.send_rates()
        self.request_telem()
        self.send_attitude()
        self.check_thermal_events()
        self.therm_capture()

    def show_horizon_lines(self):
        '''show horizon lines'''
        if self.rgb_view is None or self.rgb_view.im is None:
            return
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return
        r,p,y = self.get_encoder_attitude()
        self.rgb_view.im.add_OSD(MPImageOSD_HorizonLine('hor1', r, p, self.siyi_settings.wide_fov, (255,0,255)))
        # convert SIYI 312 to 321
        m = Matrix3()
        m.from_euler312(radians(self.attitude[0]), radians(self.attitude[1]), radians(self.attitude[2]))
        r,p,y = m.to_euler()
        self.rgb_view.im.add_OSD(MPImageOSD_HorizonLine('hor2', degrees(r), degrees(p), self.siyi_settings.wide_fov, (255,255,0)))

    def remove_horizon_lines(self):
        '''remove horizon lines'''
        self.rgb_view.im.add_OSD(MPImageOSD_None('hor1'))
        self.rgb_view.im.add_OSD(MPImageOSD_None('hor2'))

def init(mpstate):
    '''initialise module'''
    return SIYIModule(mpstate)
