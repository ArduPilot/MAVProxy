#!/usr/bin/env python
'''
support for a GCS attached DGPS system
'''

import socket, errno
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class DGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DGPSModule, self).__init__(mpstate, "DGPS", "DGPS injection support")
        self.portnum = 13320
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind(("127.0.0.1", self.portnum))
        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        print "Listening for DGPS packets on UDP://%s:%s" % ("127.0.0.1", self.portnum)

    def idle_task(self):
        '''called in idle time'''
        try:
            data = self.port.recv(200)
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise
        if len(data) > 110:
            print("DGPS data too large: %u bytes" % len(data))
            return
        try:

            self.master.mav.gps_inject_data_send(
                self.target_system,                                  
                self.target_component,
                len(data), 
                bytearray(data.ljust(110, '\0')))

        except Exception,e:
            print "DGPS Failed:", e

def init(mpstate):
    '''initialise module'''
    return DGPSModule(mpstate)
