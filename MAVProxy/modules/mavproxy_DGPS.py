#!/usr/bin/env python
'''
support for a GCS attached DGPS system
'''

import socket, errno
from pymavlink import mavutil

class dgps_state(object):
    def __init__(self):
        self.portnum = 13320

def idle_task():
    '''called in idle time'''
    state = mpstate.dgps_state
    try:
        data = state.port.recv(200)
    except socket.error as e:
        if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            return
        raise
    if len(data) > 110:
        print("DGPS data too large: %u bytes" % len(data))
        return
    mpstate.master().mav.gps_inject_data_send(mpstate.status.target_system,
                                              mpstate.status.target_component,
                                              len(data), data)
    
def name():
    '''return module name'''
    return "DGPS"

def description():
    '''return module description'''
    return "DGPS injection support"

def mavlink_packet(pkt):
    pass

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    state = dgps_state()
    mpstate.dgps_state = state

    state.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    state.port.bind(("127.0.0.1", state.portnum))
    mavutil.set_close_on_exec(state.port.fileno())
    state.port.setblocking(0)
