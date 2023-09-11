'''
control SIYI camera over UDP
'''

'''
TODO:
  circle hottest area?
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
from pymavlink import DFReader
from pymavlink.rotmat import Matrix3
from pymavlink.rotmat import Vector3
import math
from threading import Thread
import cv2

import socket, time, os, struct

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_image import MPImage
    from MAVProxy.modules.mavproxy_map import mp_slipmap

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
ABSOLUTE_ZOOM = 0x0F
READ_RANGEFINDER = 0x15
READ_TEMP_FULL_SCREEN = 0x14
SET_IMAGE_TYPE = 0x11
SET_THERMAL_PALETTE = 0x1B
REQUEST_CONTINUOUS_ATTITUDE = 0x25
ATTITUDE_EXTERNAL = 0x22
VELOCITY_EXTERNAL = 0x26
TEMPERATURE_BOX = 0x13

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

    def run(self, err):
        now = time.time()
        dt = now - self.last_t
        if now - self.last_t > 1.0:
            self.reset_I()
            dt = 0
        self.last_t = now
        P = self.settings.get(self.Pgain)
        I = self.settings.get(self.Igain)
        IMAX = self.settings.get(self.IMAX)
        out = P*err
        self.I += I*err*dt
        self.I = mp_util.constrain(self.I, -IMAX, IMAX)
        return out + self.I

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

    def write(self, name, fmt, fields, *args):
        if not name in self.formats:
            self.formats[name] = self.mlog.add_format(DFReader.DFFormat(0, name, 0, fmt, fields))
            self.outf.write(self.mlog.make_format_msgbuf(self.formats[name]))
        self.outf.write(self.mlog.make_msgbuf(self.formats[name], args))
        self.outf.flush()

class SIYIModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SIYIModule, self).__init__(mpstate, "SIYI", "SIYI camera support")

        self.add_command('siyi', self.cmd_siyi, "SIYI camera control",
                         ["<rates|connect|autofocus|zoom|yaw|pitch|center|getconfig|angle|photo|recording|lock|follow|fpv|settarget|notarget|thermal|box>",
                          "set (SIYISETTING)",
                          "imode <1|2|3|4|5|6|7|8|wide|zoom|split>",
                          "palette <WhiteHot|Sepia|Ironbow|Rainbow|Night|Aurora|RedHot|Jungle|Medical|BlackHot|GloryHot>"
                          ])

        # filter_dist is distance in metres
        self.siyi_settings = mp_settings.MPSettings([("port", int, 37260),
                                                     ('ip', str, "192.168.144.25"),
                                                     ('rates_hz', float, 5),
                                                     ('yaw_rate', float, 10),
                                                     ('pitch_rate', float, 10),
                                                     ('rates_hz', float, 5),
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
                                                     ('telem_hz', float, 5),
                                                     ('att_send_hz', float, 10),
                                                     ('lidar_hz', float, 10),
                                                     ('temp_hz', float, 5),
                                                     ('thermal_port', int, 7002),
                                                     ('logfile', str, 'SIYI_log.bin'),
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
        self.attitude = None
        self.tmax = -1
        self.tmin = -1
        self.spot_temp = -1
        self.tmax_x = None
        self.tmax_y = None
        self.tmin_x = None
        self.tmin_y = None
        self.last_temp_t = None
        self.last_att_t = None
        self.GLOBAL_POSITION_INT = None
        self.ATTITUDE = None
        self.target_pos = None
        self.last_map_ROI = None
        self.icon = self.mpstate.map.icon('camera-small-red.png')
        self.last_target_send = time.time()
        self.last_rate_display = time.time()
        self.yaw_controller = PI_controller(self.siyi_settings, 'yaw_gain_P', 'yaw_gain_I', 'yaw_gain_IMAX')
        self.pitch_controller = PI_controller(self.siyi_settings, 'pitch_gain_P', 'pitch_gain_I', 'pitch_gain_IMAX')
        self.logf = DF_logger(self.siyi_settings.logfile)
        self.start_time = time.time()
        self.last_att_send_t = time.time()
        self.last_lidar_t = time.time()
        self.last_temp_t = time.time()
        self.thermal_im = None
        self.thermal_cap = None
        self.box_pending = None
        self.last_box_req_t = time.time()

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
                                     MPMenuItem('AutoFocus', 'AutoFocus', '# siyi autofocus '),
                                     MPMenuItem('ImageSplit', 'ImageSplit', '# siyi imode split '),
                                     MPMenuItem('ImageWide', 'ImageWide', '# siyi imode wide '),
                                     MPMenuItem('ImageZoom', 'ImageZoom', '# siyi imode zoom '),
                                     MPMenuItem('Recording', 'Recording', '# siyi recording '),
                                     MPMenuItem('ClearTarget', 'ClearTarget', '# siyi notarget '),
                                     MPMenuItem('ThermalView', 'Thermalview', '# siyi thermal '),
                                     MPMenuItem('Zoom1', 'Zoom1', '# siyi zoom 1 '),
                                     MPMenuItem('Zoom2', 'Zoom2', '# siyi zoom 2 '),
                                     MPMenuItem('Zoom4', 'Zoom4', '# siyi zoom 4 '),
                                     MPMenuItem('Zoom8', 'Zoom8', '# siyi zoom 8 ')])
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
        elif args[0] == "recording":
            self.send_packet_fmt(PHOTO, "<B", 2)
            self.send_packet(FUNCTION_FEEDBACK_INFO, None)
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
        elif args[0] == "box":
            self.cmd_box(args[1:])
        else:
            print(usage)

    def cmd_connect(self):
        '''connect to the camera'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.connect((self.siyi_settings.ip, self.siyi_settings.port))
        self.sock.setblocking(False)
        print("Connected to SIYI")

    def cmd_rates(self, args):
        '''update rates'''
        if len(args) < 2:
            print("Usage: siyi rates PAN_RATE PITCH_RATE")
            return
        self.yaw_rate = float(args[0])
        self.pitch_rate = float(args[1])
        self.clear_target()

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
        mode = imode_map.get(args[0],None)
        if mode is None:
            mode = int(args[0])
        self.send_packet_fmt(SET_IMAGE_TYPE, "<B", mode)

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

    def cmd_box(self, args):
        '''open query temp in box'''
        if len(args) < 4:
            print("usage: xmin ymin xmax ymax")
            return
        xmin = int(args[0])
        ymin = int(args[1])
        xmax = int(args[2])
        ymax = int(args[3])
        self.box_pending = (xmin,xmax,ymin,ymax)

    def thermal_capture(self):
        '''thermal capture thread'''
        while True:
            try:
                _, frame = self.thermal_cap.read()
            except Exception:
                break
            if frame is None:
                break
            self.thermal_im.set_image(frame, bgr=True)
        print("thermal stream ended")
        self.thermal_im = None
        self.thermal_cap = None

    def cmd_thermal(self):
        '''open thermal viewer'''
        self.thermal_cap = cv2.VideoCapture('udp://@:%u' % self.siyi_settings.thermal_port)
        if not self.thermal_cap or not self.thermal_cap.isOpened():
            print('Cannot open thermal UDP stream')
            self.thermal_cap = None
            return
        self.thermal_im = MPImage(title='Thermal View',
                                  mouse_events=True,
                                  width=640,
                                  height=512,
                                  key_events=True,
                                  can_drag = False,
                                  can_zoom = False,
                                  auto_size = False)
        if self.thermal_im is None:
            self.thermal_cap.release()
            self.thermal_cap = None
        self.thermal_thread = Thread(target=self.thermal_capture)
        self.thermal_thread.daemon = True
        self.thermal_thread.start()

    def check_thermal_events(self):
        '''check for mouse events on thermal image'''
        if self.thermal_im is None:
            return
        for event in self.thermal_im.events():
            if isinstance(event, MPMenuItem):
                print(event)
                continue
            if event.ClassName == 'wxMouseEvent' and event.leftIsDown:
                #print(dir(event))
                xmin = max(event.x-10, 1)
                ymin = max(event.y-10, 1)
                xmax = min(event.x+10, 639)
                ymax = min(event.y+10, 511)
                self.box_pending = (xmin,xmax,ymin,ymax)

    def cmd_zoom(self, args):
        '''set zoom'''
        if len(args) < 1:
            print("Usage: siyi zoom ZOOM")
            return
        fval = float(args[0])
        ival = int(fval)
        frac = int((fval - ival)*10)
        self.send_packet_fmt(ABSOLUTE_ZOOM, "<BB", ival, frac)

    def set_target(self, lat, lon, alt):
        '''set target position'''
        self.target_pos = (lat, lon, alt)
        self.mpstate.map.add_object(mp_slipmap.SlipIcon('SIYI',
                                                        (lat, lon),
                                                        self.icon, layer='SIYI', rotation=0, follow=False))

    def clear_target(self):
        '''clear target position'''
        self.target_pos = None
        self.mpstate.map.remove_object('SIYI')
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
        if self.yaw_rate is not None or self.pitch_rate is not None:
            y = self.yaw_rate
            p = self.pitch_rate
            if y is None:
                y = 0.0
            if p is None:
                p = 0.0
            scale = 100.0 / SIYI_RATE_MAX_DPS
            y = int(mp_util.constrain(y*scale, -100, 100))
            p = int(mp_util.constrain(p*scale, -100, 100))
            self.send_packet_fmt(GIMBAL_ROTATION, "<bb", y, p)
            self.logf.write('SIGR', 'Qffbb', 'TimeUS,YRate,PRate,YC,PC',
                            self.micros64(), self.yaw_rate, self.pitch_rate, y, p)

            cam_yaw, cam_pitch, cam_roll = self.get_gimbal_attitude()
            self.send_named_float('CROLL', cam_roll)
            self.send_named_float('CYAW', cam_yaw)
            self.send_named_float('CPITCH', cam_pitch)
            self.send_named_float('CROLL_RT', self.attitude[3])
            self.send_named_float('CPTCH_RT', self.attitude[4])
            self.send_named_float('CYAW_RT', self.attitude[5])
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
        if self.siyi_settings.lidar_hz > 0 and now - self.last_lidar_t >= 1.0/self.siyi_settings.lidar_hz:
            self.last_lidar_t = now
            self.send_packet(READ_RANGEFINDER, None)
        if self.siyi_settings.temp_hz > 0 and now - self.last_temp_t >= 1.0/self.siyi_settings.temp_hz:
            self.last_temp_t = now
            self.send_packet_fmt(READ_TEMP_FULL_SCREEN, "<B", 2)
        if self.last_att_t is None or now - self.last_att_t > 2:
            self.last_att_t = now
            self.send_packet_fmt(REQUEST_CONTINUOUS_ATTITUDE, "<BB", 1, 5)

    def send_attitude(self):
        '''send attitude to gimbal'''
        now = time.time()
        if self.siyi_settings.att_send_hz <= 0 or now - self.last_att_send_t < 1.0/self.siyi_settings.telem_hz:
            return
        self.last_att_send_t = now
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return
        r = Matrix3()
        r.from_euler(att.roll, att.pitch, att.yaw)
        (roll, pitch, yaw) = r.to_euler312()
        self.send_packet_fmt(ATTITUDE_EXTERNAL, "<Iffffff",
                             self.millis32(),
                             roll, pitch, yaw,
                             att.rollspeed, att.pitchspeed, att.yawspeed)
        gpi = self.master.messages.get('GLOBAL_POSITION_INT',None)
        if gpi is None:
            return
        self.send_packet_fmt(VELOCITY_EXTERNAL, "<Ifff",
                             self.millis32(),
                             gpi.vx*0.01,
                             gpi.vy*0.01,
                             gpi.vz*0.01)


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
        v = struct.unpack(fmt, data)
        args = list(v)
        args.extend([0]*(10-len(args)))
        self.logf.write('SIIN', 'QBffffffffff', 'TimeUS,Cmd,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10', self.micros64(), command_id, *args)
        return v

    def euler_312_to_euler_321(self, r, p, y):
        '''convert between euler conventions'''
        m = Matrix3()
        m.from_euler312(r,p,y)
        return m.to_euler()

    def parse_packet(self, pkt):
        '''parse SIYI packet'''
        if len(pkt) < 10:
            return
        (h1,h2,rack,plen,seq,cmd) = struct.unpack("<BBBHHB", pkt[:8])
        data = pkt[8:-2]
        crc, = struct.unpack("<H", pkt[-2:])
        crc2 = crc16_from_bytes(pkt[:-2])
        if crc != crc2:
            return

        if cmd == ACQUIRE_FIRMWARE_VERSION:
            (patch,minor,major,gpatch,gminor,gmajor) = self.unpack(cmd, "<BBBBBB", data[:6])
            print("SIYI CAM %u.%u.%u" % (major, minor, patch))
            print("SIYI Gimbal %u.%u.%u" % (gmajor, gminor, gpatch))
            self.have_version = True

        elif cmd == ACQUIRE_GIMBAL_ATTITUDE:
            (z,y,x,sz,sy,sx) = self.unpack(cmd, "<hhhhhh", data[:12])
            self.last_att_t = time.time()
            (roll,pitch,yaw) = (x*0.1, y*0.1, mp_util.wrap_180(-z*0.1))
            (roll,pitch,yaw) = self.euler_312_to_euler_321(math.radians(roll),math.radians(pitch),math.radians(yaw))
            self.attitude = (math.degrees(roll),math.degrees(pitch),math.degrees(yaw), sx*0.1, sy*0.1, -sz*0.1)
            self.update_status()
            self.logf.write('SIGA', 'Qffffffhhhhhh', 'TimeUS,Y,P,R,Yr,Pr,Rr,z,y,x,sz,sy,sx',
                            self.micros64(),
                                self.attitude[2], self.attitude[1], self.attitude[0],
                                self.attitude[5], self.attitude[4], self.attitude[3],
                                z,y,x,sz,sy,sx)

        elif cmd == ACQUIRE_GIMBAL_CONFIG_INFO:
            res, hdr_sta, res2, record_sta, gim_motion, gim_mount, video = self.unpack(cmd, "<BBBBBBB", data[:7])
            print("HDR: %u" % hdr_sta)
            print("Recording: %u" % record_sta)
            print("GimbalMotion: %u" % gim_motion)
            print("GimbalMount: %u" % gim_mount)
            print("Video: %u" % video)

        elif cmd == READ_RANGEFINDER:
            r, = self.unpack(cmd, "<H", data[:2])
            self.rf_dist = r * 0.1
            self.update_status()

        elif cmd == READ_TEMP_FULL_SCREEN:
            if len(data) < 12:
                print("READ_TEMP_FULL_SCREEN: Expected 12 bytes, got %u" % len(data))
                return
            self.tmax,self.tmin,self.tmax_x,self.tmax_y,self.tmin_x,self.tmin_y = self.unpack(cmd, "<HHHHHH", data[:12])
            self.tmax = self.tmax * 0.01
            self.tmin = self.tmin * 0.01
            self.last_temp_t = time.time()
        elif cmd == FUNCTION_FEEDBACK_INFO:
            info_type, = self.unpack(cmd, "<B", data[:1])
            feedback = {
                0: "Success",
                1: "FailPhoto",
                2: "HDR ON",
                3: "HDR OFF",
                4: "FailRecord",
            }
            print("Feedback %s" % feedback.get(info_type, str(info_type)))
        elif cmd == TEMPERATURE_BOX:
            self.box_pending = None
            startx,starty,endx,endy,temp_max,temp_min,temp_max_x,temp_max_y,temp_min_x,temp_min_y = self.unpack(cmd, "<HHHHHHHHHH", data[:20])
            #print("got box: ", startx, starty)
            if self.thermal_im is not None:
                self.spot_temp = temp_max*0.01
                self.update_title()

        elif cmd in [SET_ANGLE, CENTER, GIMBAL_ROTATION, ABSOLUTE_ZOOM, SET_IMAGE_TYPE,
                     REQUEST_CONTINUOUS_ATTITUDE, SET_THERMAL_PALETTE]:
            # an ack
            pass
        else:
            print("SIYI: Unknown command 0x%02x" % cmd)

    def update_title(self):
        '''update thermal view title'''
        if self.thermal_im is None:
            return
        self.thermal_im.set_title("Thermal View TEMP=%.2fC%s RANGE(%.2fC to %.2fC)" % (
            self.spot_temp, "(*)" if self.box_pending else "", self.tmin, self.tmax))

    def update_status(self):
        if self.attitude is None:
            return
        self.console.set_status('SIYI', 'SIYI (%.1f,%.1f,%.1f) rf=%.1f' % (
            self.attitude[0], self.attitude[1], self.attitude[2],
            self.rf_dist), row=6)
        if self.tmin is not None:
            self.console.set_status('TEMP', 'TEMP %.2f/%.2f' % (self.tmin, self.tmax), row=6)
            self.update_title()

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

    def get_gimbal_attitude(self):
        '''get extrapolated gimbal attitude, returning yaw and pitch'''
        now = time.time()
        dt = (now - self.last_att_t)+self.siyi_settings.lag
        yaw = self.attitude[2]+self.attitude[5]*dt
        pitch = self.attitude[1]+self.attitude[4]*dt
        pitch -= self.siyi_settings.mount_pitch
        yaw -= self.siyi_settings.mount_yaw
        yaw = mp_util.wrap_180(yaw)
        roll = self.attitude[0]
        return yaw, pitch, roll

    def get_latlon(self, slant_range):
        '''get lat/lon given vehicle orientation, camera orientation and slant range'''
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return None
        gpi = self.master.messages.get('GLOBAL_POSITION_INT',None)
        if gpi is None:
            return None
        cam_att = self.attitude
        v = Vector3(slant_range, 0, 0)
        m = Matrix3()
        m.from_euler()

    def update_target(self):
        '''update position targetting'''
        if not 'GLOBAL_POSITION_INT' in self.master.messages or not 'ATTITUDE' in self.master.messages:
            return

        # added rate of target update

        map_module = self.module('map')
        if map_module is not None and map_module.current_ROI != self.last_map_ROI:
            self.last_map_ROI = map_module.current_ROI
            (lat, lon, alt) = self.last_map_ROI
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

        GPS_vector_x = (lon-mylon)*1.0e7*math.cos(math.radians((mylat + lat) * 0.5)) * 0.01113195
        GPS_vector_y = (lat - mylat) * 0.01113195 * 1.0e7
        GPS_vector_z = alt - myalt # was cm
        target_distance = math.sqrt(GPS_vector_x**2 + GPS_vector_y**2)

        dt = now - ATTITUDE._timestamp
        vehicle_yaw_rad = ATTITUDE.yaw + ATTITUDE.yawspeed*dt

        # calculate pitch, yaw angles
        pitch = math.atan2(GPS_vector_z, target_distance)
        yaw = math.atan2(GPS_vector_x, GPS_vector_y)
        yaw -= vehicle_yaw_rad
        yaw_deg = mp_util.wrap_180(math.degrees(yaw))
        pitch_deg = math.degrees(pitch)

        cam_yaw, cam_pitch, cam_roll = self.get_gimbal_attitude()
        err_yaw = mp_util.wrap_180(yaw_deg - cam_yaw)
        err_pitch = pitch_deg - cam_pitch

        self.yaw_rate = self.yaw_controller.run(err_yaw)
        self.pitch_rate = self.yaw_controller.run(err_pitch)
        self.send_named_float('TYAW', yaw_deg)
        self.send_named_float('TPITCH', pitch_deg)
        self.send_named_float('EYAW', err_yaw)
        self.send_named_float('EPITCH', err_pitch)
        self.logf.write('SIPY', "Qffff", "TimeUS,CYaw,TYaw,Yerr,I",
                        self.micros64(), cam_yaw, yaw_deg, err_yaw, self.yaw_controller.I)
        self.logf.write('SIPP', "Qffff", "TimeUS,CPitch,TPitch,Perr,I",
                        self.micros64(), cam_pitch, pitch_deg, err_pitch, self.pitch_controller.I)

    def get_gps_time(self, tnow):
        '''return gps_week and gps_week_ms for current time'''
        leapseconds = 18
        SEC_PER_WEEK = 7 * 86400

        epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
        epoch_seconds = int(tnow - epoch)
        week = int(epoch_seconds) // SEC_PER_WEEK
        t_ms = int(tnow * 1000) % 1000
        week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
        return week, week_ms

    def mavlink_packet(self, m):
        '''process a mavlink message'''
        mtype = m.get_type()
        if mtype == 'GPS_RAW_INT':
            # ?!? why off by 18 hours
            gwk, gms = self.get_gps_time(time.time()+18*3600)
            self.logf.write('GPS', "QBIHLLff", "TimeUS,Status,GMS,GWk,Lat,Lng,Alt,Spd",
                            self.micros64(), m.fix_type, gms, gwk, m.lat, m.lon, m.alt*0.001, m.vel*0.01)
        if mtype == 'ATTITUDE':
            self.logf.write('ATT', "Qffffff", "TimeUS,Roll,Pitch,Yaw,GyrX,GyrY,GyrZ",
                            self.micros64(),
                            math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw),
                            math.degrees(m.rollspeed), math.degrees(m.pitchspeed), math.degrees(m.yawspeed))


    def check_box_pending(self):
        now = time.time()
        if self.box_pending is None or now - self.last_box_req_t < 0.2:
            return
        self.last_box_req_t = now
        (xmin,xmax,ymin,ymax) = self.box_pending
        #print("send box: ", xmin, ymin)
        self.send_packet_fmt(TEMPERATURE_BOX, "<HHHHB", xmin, ymin, xmax, ymax, 1)
        self.update_title()

    def idle_task(self):
        '''called on idle'''
        if not self.sock:
            return
        self.check_rate_end()
        self.update_target()
        self.send_rates()
        self.request_telem()
        self.send_attitude()
        self.check_thermal_events()
        self.check_box_pending()
        if not self.have_version and time.time() - self.last_version_send > 1.0:
            self.last_version_send = time.time()
            self.send_packet(ACQUIRE_FIRMWARE_VERSION, None)
        try:
            pkt = self.sock.recv(10240)
        except Exception as ex:
            return
        self.parse_packet(pkt)

def init(mpstate):
    '''initialise module'''
    return SIYIModule(mpstate)
