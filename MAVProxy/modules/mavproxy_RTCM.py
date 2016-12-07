#!/usr/bin/env python
'''
support for a GCS attached RTCM system
'''

import socket, errno
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class RTCMModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RTCMModule, self).__init__(mpstate, "RTCM", "RTCM injection support")
        #self.portip = '192.168.43.25'
        self.portip = '192.168.2.15'
        self.portnum = 9000
        #self.portip = raw_input("Enter the IP Address of the of the GPS base\n")
        #self.portnum = raw_input("Enter the port number of the GPS Base\n")
        self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.connect((self.portip, int(self.portnum)))
        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        self.sequence = 0
        print("Listening for RTCM packets on UDP://%s:%s" % (self.portip, self.portnum))

    def idle_task(self):
        '''called in idle time'''
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
