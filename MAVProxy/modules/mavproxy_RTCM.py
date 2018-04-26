#!/usr/bin/env python
'''
support for a GCS attached RTCM system
'''

import socket, errno
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from time import sleep

class RTCMModule(mp_module.MPModule):
    def __init__(self, mpstate, **kwargs):
        """
        RTK GPS injection client using GPS_RTCM_DATA MavLink messages

        Waits until the the RTK GPS has been initialized before starting to send data

        Parameters:
           addr: IP addr descriptor <protocol:ip:port>
               protocol: either tcp or udp
               ip: IP address
               port: IP port number
           inst: 1 for GPS1,  for GPS2, defaults to 1

        Usage inside mavproxy:
            module load RTCM:{"addr":"protocol:ip:port","inst":"2"}
          or:
            module load RTCM
            rtcm addr protocol:ip:port
            rtcm inst 2

        Usage from the command line:
            --load-module "RTCM:'{"addr":"protocol:ip:port","inst":"2"}'"
          or:
            --load-module RTCM --cmd "rtcm addr protocol:ip:port","rtcm inst 2"
        """
        super(RTCMModule, self).__init__(mpstate, "RTCM", "RTCM message injection for RTK GPSs", public = True)
        self.add_command('rtcm', self.cmd_rtcm, "RTCM message injection for RTK GPSs", ["<addr|inst|show|start|stop>"])
        self.port = None
        self.receiving = False
        self.should_send = True
        self.sending = False

        self.addr = kwargs.get("addr", None)
        if self.addr is None:
            print("No active input! Set a source addr by using 'rtcm addr <protocol:ip:port>'")
        else:
            self.cmd_rtcm_addr(self.addr)

        self.cmd_handle_instance(kwargs.get("inst", '1'))


    def cmd_rtcm(self, args):
        '''handle output commands'''
        if len(args) < 1 or args[0] == "show":
            self.cmd_rtcm_show()

        elif args[0] == "addr":
            if len(args) != 2:
                print("Usage: rtcm addr <protocol:ip:port>")
                return
            if args[1] == self.addr: # Return if they match
                return
            self.cmd_rtcm_close()
            self.cmd_rtcm_addr(args[1])

        elif args[0] == "inst":
            if len(args) != 2:
                print("Usage: rtcm inst <GPS_INSTANCE_NUMBER>")
                return
            self.cmd_handle_instance(args[1])

        elif args[0] == "start":
            if not self.receiving:
                print("No active input! Set a source addr by using 'rtcm addr <protocol:ip:port>'")
                return
            self.should_send = True

        elif args[0] == "stop":
            print("RTCM injection stopped")
            self.should_send = False

        else:
            print("Usage: rtcm <addr|inst|show|start|stop>")


    def cmd_handle_instance(self, arg):
        if arg == '1':
            self.gps_message = 'GPS_RAW_INT'
        elif arg == '2':
            self.gps_message = 'GPS2_RAW'
        else:
            print("Destination GPS instance must be set to 1 or 2.")
            return
        self.inst = arg


    def cmd_rtcm_show(self):
        '''lists active outputs'''
        if not self.receiving:
            print("No active input!")
        else:
            print("RTCM source is %s:%s:%s" % (self.port_proto, self.portip, self.portnum))
        if self.sending:
            print("RTCM destination is GPS%s (injection was enabled by %s.FIX_TYPE!=NO_GPS message)" % (self.inst, self.gps_message))
        else:
            if self.should_send:
                print("RTCM destination is GPS%s (injection was disabled by %s.FIX_TYPE==NO_GPS message)" % (self.inst, self.gps_message))
            else:
                print("RTCM destination is GPS%s (injection stoped by user command)" % self.inst)


    def cmd_rtcm_addr(self, args):
        '''Connects to IP:port using the defined protocol'''
        retries=3
        addr = args.split(":")
        if len(addr) != 3:
            print("Check the format of string. Should be <udp/tcp:ip:port>")
            return
        self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port_proto = addr[0]
        self.portip = addr[1]
        self.portnum = int(addr[2])
        if self.port_proto == "tcp":
            self.port.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        elif self.port_proto == "udp":
            self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        else:
            print("Check the protocol. Unrecognised protocol '%s'. Should be 'udp' or 'tcp'" % addr[0])
            self.port = None
            return
        while retries >= 0:
            retries -= 1
            if retries <= 0:
                self.port.connect((self.portip, self.portnum))
            else:
                try:
                    self.port.connect((self.portip, self.portnum))
                    break
                except Exception as e:
                    if retries > 0:
                        print(e, "sleeping")
                        sleep(1)
                    continue
        self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        self.sequence = 0
        print("Listening for RTCM packets on %s:%s:%s" % (self.port_proto, self.portip, self.portnum))
        self.receiving = True
        self.addr = args


    def cmd_rtcm_close(self):
        '''removes IP and port'''
        if not self.receiving:
            return
        self.receiving = False
        self.port.shutdown(socket.SHUT_RDWR)
        self.port.close()
        self.port = None
        print("Closed %s:%s:%s" % (self.port_proto, self.portip, self.portnum))


    def idle_task(self):
        '''called in idle time'''
        if not self.receiving or not self.sending:
            return
        try:
            data = self.port.recv(2048) # Attempt to read up to 2048 bytes.
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            elif e.errno == errno.WSAECONNRESET:
                # reconnect
                self.cmd_rtcm_close()
                self.cmd_rtcm_addr(self.addr)
                return
            raise
        if len(data) > 720:
            print("RTCM: GPS inject data too large: %u bytes, GPS_RTCM_DATA only supports up to 720 bytes" % len(data))
            return
        data_ptr = 0
        remain = len(data)
        fragment = 0

        try:
            while (remain > 0):
                if (remain >= 180):
                    send_len = 180
                else:
                    send_len = remain
                self.master.mav.gps_rtcm_data_send(
                    ((self.sequence % 32) << 3) + (fragment << 1) + int(len(data) <= 180),
                    send_len,
                    bytearray(data[data_ptr:data_ptr+send_len].ljust(180, '\0')))
                if (remain == 180 and fragment != 3):
                    self.master.mav.gps_rtcm_data_send(
                        ((self.sequence % 32) << 3) + (fragment << 1) + 1,
                        0,
                        bytearray(['\0'] * 180))
                #print("RTCM data: %u seq   %u frag    %u len   %d total     %d\n" % (self.sequence % 32, fragment, send_len, len(data), ((self.sequence % 32) << 3) + (fragment << 1) + int(remain >= 180)))
                fragment += 1
                data_ptr += send_len
                remain   -= send_len

            self.sequence += 1

        except Exception,e:
            print "RTCM: GPS Inject Failed:", e


    def mavlink_packet(self, m):
        """handle an incoming mavlink packet"""
        if self.receiving and self.should_send:
            if m.get_type() == self.gps_message:
                gps_inited = (m.fix_type != mavutil.ardupilotmega.GPS_FIX_TYPE_NO_GPS)
                if self.sending and not gps_inited:
                    print("GPS%s not initialized, RTCM injection stopped" % self.inst)
                if not self.sending and gps_inited:
                    print("GPS%s initialized, RTCM injection started" % self.inst)
                # only send RTCM injections when the GPS receiver on the vehicle reports that it is fully initialized
                self.sending = gps_inited
        else:
            self.sending = False


def init(mpstate, **kwargs):
    '''initialise module'''
    return RTCMModule(mpstate, **kwargs)
