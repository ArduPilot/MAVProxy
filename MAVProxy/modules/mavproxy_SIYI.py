'''
control SIYI camera over UDP
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil

import socket, time, os, struct

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
ABSOLUTE_ZOOM = 0x0F

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


class SIYIModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SIYIModule, self).__init__(mpstate, "SIYI", "SIYI camera support")

        self.add_command('siyi', self.cmd_siyi, "SIYI camera control",
                         ["<rates|connect>","set (SIYISETTING)"])

        # filter_dist is distance in metres
        self.siyi_settings = mp_settings.MPSettings([("port", int, 37260),
                                                     ('ip', str, "192.168.144.25"),
                                                     ('rates_hz', float, 5),
                                                     ('att_hz', float, 5)])
        self.add_completion_function('(SIYISETTING)',
                                     self.siyi_settings.completion)
        self.sock = None
        self.rates = (0.0, 0.0)
        self.sequence = 0
        self.last_rates_send = time.time()
        self.last_version_send = time.time()
        self.last_att_send = time.time()
        self.have_version = False
        self.console.set_status('SIYI', 'SIYI - -', row=6)

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
        self.rates = (float(args[0]), float(args[1]))

    def send_rates(self):
        '''send rates packet'''
        now = time.time()
        if self.siyi_settings.rates_hz <= 0 or now - self.last_rates_send < 1.0/self.siyi_settings.rates_hz:
            return
        self.last_rates_send = now
        pkt = struct.pack("<bb",
                          int(100.0*self.rates[0]/SIYI_RATE_MAX_DPS),
                          int(100.0*self.rates[1]/SIYI_RATE_MAX_DPS))
        self.send_packet(GIMBAL_ROTATION, pkt)

    def request_attitude(self):
        '''request attitude'''
        now = time.time()
        if self.siyi_settings.att_hz <= 0 or now - self.last_att_send < 1.0/self.siyi_settings.att_hz:
            return
        self.last_att_send = now
        self.send_packet(ACQUIRE_GIMBAL_ATTITUDE, None)

    def send_packet(self, command_id, pkt):
        '''send SIYI packet'''
        plen = len(pkt) if pkt else 0
        buf = struct.pack("<BBBHHB", SIYI_HEADER1, SIYI_HEADER2, 1, plen,
                          self.sequence, command_id)
        if pkt:
            buf += pkt
        buf += struct.pack("<H", crc16_from_bytes(buf))
        self.sequence += 1
        try:
            self.sock.send(buf)
        except Exception:
            pass

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
            (patch,minor,major) = struct.unpack("<BBB", data[:3])
            print("SIYI CAM %u.%u.%u" % (major, minor, patch))
            (patch,minor,major) = struct.unpack("<BBB", data[3:6])
            print("SIYI Gimbal %u.%u.%u" % (major, minor, patch))
            self.have_version = True

        elif cmd == ACQUIRE_GIMBAL_ATTITUDE:
            (z,y,x) = struct.unpack("<hhh", data[:6])
            self.console.set_status('SIYI', 'SIYI %.1f %.1f %.1f' % (x*0.1, y*0.1, mp_util.wrap_180(-z*0.1)), row=6)

    def idle_task(self):
        '''called on idle'''
        if not self.sock:
            return
        self.send_rates()
        self.request_attitude()
        if not self.have_version and time.time() - self.last_version_send > 1.0:
            self.last_version_send = time.time()
            self.send_packet(ACQUIRE_FIRMWARE_VERSION, None)

        try:
            pkt = self.sock.recv(10240)
            self.parse_packet(pkt)
        except Exception:
            pass

def init(mpstate):
    '''initialise module'''
    return SIYIModule(mpstate)
