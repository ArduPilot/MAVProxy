'''
setup a listening TCP socket for forwarding u-center connections to a GPS
via MAVLink serial-control
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import socket
import time
import errno

class UcenterModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(UcenterModule, self).__init__(mpstate, "ucenter", "ucenter forwarding")

        self.add_command('ucenter', self.cmd_ucenter, "ucenter control",
                         ["<start|stop|restart>","set (UCENTERSETTING)"])

        self.ucenter_settings = mp_settings.MPSettings([("port", int, 2001),
                                                        ('devnum', int, 2),
                                                        ('baudrate', int, 115200),
                                                        ('delay', float, 0.01),
                                                        ('debug', int, 0)])
        self.add_completion_function('(UCENTERSETTING)',
                                     self.ucenter_settings.completion)
        self.listen_sock = None
        self.sock = None
        self.last_write = time.time()
        self.last_baudrate = 0
        self.last_devnum = -1

    def cmd_ucenter(self, args):
        '''ucenter command parser'''
        usage = "usage: ucenter <set|start|restart|stop>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.ucenter_settings.command(args[1:])
        elif args[0] == "start":
            self.start_listener()
        elif args[0] == "stop":
            self.stop_listener()
        elif args[0] == "restart":
            self.stop_listener()
            self.start_listener()
        else:
            print(usage)

    def start_listener(self):
        '''start listening for packets'''
        if self.listen_sock is not None:
            self.listen_sock.close()
        if self.sock is not None:
            self.sock.close()
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', self.ucenter_settings.port))
        self.listen_sock.setblocking(False)
        self.listen_sock.listen(1)
        print("ucenter listening on port %u" % self.ucenter_settings.port)

    def stop_listener(self):
        '''stop listening for packets'''
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.listen_sock is not None:
            self.listen_sock.close()
            self.listen_sock = None
        self.last_baudrate = 0

    def debug(self, s):
        '''debug write'''
        if self.ucenter_settings.debug <= 0:
            return
        print(s)

    def write(self, b):
        '''write some bytes to remove port'''
        if len(b) > 0:
            self.debug("sending '%s' (0x%02x) of len %u" % (b, ord(b[0]), len(b)))
        elif self.ucenter_settings.debug > 1:
            self.debug("sending empty request")
        while True:
            # note that we send a single empty buffer on len(b)==0
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))

            if self.last_baudrate != self.ucenter_settings.baudrate or self.last_devnum != self.ucenter_settings.devnum:
                baudrate = self.ucenter_settings.baudrate
                self.last_baudrate = baudrate
                self.last_devnum = self.ucenter_settings.devnum
                print("ucenter requesting baudrate %u" % baudrate)
            else:
                baudrate = 0

            self.master.mav.serial_control_send(self.ucenter_settings.devnum,
                                                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                                0,
                                                baudrate,
                                                n,
                                                buf)
            b = b[n:]
            self.last_write = time.time()
            if len(b) == 0:
                break

    def idle_task(self):
        '''called on idle'''
        if self.sock is None and self.listen_sock is None:
            return
        if self.sock is None:
            try:
                conn_sock, addr = self.listen_sock.accept()
            except Exception as e:
                if e.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                    print("ucenter listen fail")
                    self.stop_listener()
                    return
                return
            self.sock = conn_sock
            self.sock.setblocking(False)
            print("ucenter connection from %s" % str(addr))

        now = time.time()

        try:
            pkt = self.sock.recv(1000)
        except socket.error as e:
            if e.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                print("ucenter connection closed")
                self.sock.close()
                self.sock = None
                return
            # send empty packet if idle
            pkt = ''
        if len(pkt) == 0 and now - self.last_write < self.ucenter_settings.delay:
            return
        self.write(pkt)

    def mavlink_packet(self, m):
        '''process SERIAL_CONTROL packets'''
        if m.get_type() != 'SERIAL_CONTROL':
            return
        if self.sock is None:
            return
        if m.count == 0:
            return
        data = m.data
        data = m.data[:m.count]
        buf = ''.join(str(chr(x)) for x in data)
        self.debug("got reply len %u" % len(buf))
        try:
            self.sock.send(buf)
        except socket.error as e:
            if e.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                print("ucenter connection write error")
                self.sock.close()
                self.sock = None


def init(mpstate):
    '''initialise module'''
    return UcenterModule(mpstate)
