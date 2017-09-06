#!/usr/bin/env python
'''
support for a GCS attached RTCM system
'''

import socket, errno
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from time import sleep

class RTCMModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RTCMModule, self).__init__(mpstate, "RTCM", "RTCM injection support", public = True)
        self.add_command('rtcm', self.cmd_rtcm, "RTCM output control", ["<list|add|remove>"])
        self.port = None

    def cmd_rtcm(self, args):
        '''handle output commands'''
        if len(args) < 1 or args[0] == "list":
            self.cmd_rtcm_list()
        elif args[0] == "add":
            if len(args) != 2:
                print("Usage: rtcm add OUTPUT")
                return
            self.cmd_rtcm_add(args[1:])
        elif args[0] == "remove":
            if len(args) != 2:
                print("Usage: rtcm remove OUTPUT")
                return
            self.cmd_rtcm_remove(args[1:])
        else:
            print("Usage: rtcmout <list|add|remove>")

    def cmd_rtcm_list(self):
        '''lists active outputs'''
        if self.port == None:
            print("No active input!")
        else:
            print("Active input from %s://%s:%s" % (self.port_proto, self.portip, self.portnum))

    def cmd_rtcm_add(self,args):
        '''Adds IP and port to port'''
        retries=3
        addr = args[0].split(":")
        if len(addr) != 3:
            print("Check the format of string. Should be udp/tcp:IP:PORT")
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
        self.port.setblocking(0)
        self.sequence = 0
        print("Listening for RTCM packets on %s://%s:%s" % (self.port_proto, self.portip, self.portnum))

    def cmd_rtcm_remove(self,args):
        '''removes IP and port'''
        if self.port == None:
            print("No active port is running")
            return
        addr = args[0].split(":")
        if self.port_proto == addr[0] and self.portip == addr[1] and self.portnum == int(addr[2]):
            self.port.shutdown(socket.SHUT_RDWR)
            self.port.close()
            print("Removed %s://%s:%s" % (addr[0],addr[1],addr[2]))
            self.port = None
        else:
            print("Port %s://%s:%s is not used by the module" % (addr[0],addr[1],addr[2]))

    def idle_task(self):
        '''called in idle time'''
        if self.port == None:
            return
        try:
            data = self.port.recv(1024) # Attempt to read up to 1024 bytes.
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
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

def init(mpstate):
    '''initialise module'''
    return RTCMModule(mpstate)
