#!/usr/bin/env python
'''
HIL module
Andrew Tridgell
December 2012

This interfaces to Tools/autotest/jsbsim/runsim.py to run the JSBSim flight simulator
'''

import sys, os, time, socket, errno, struct, math
from math import degrees, radians
mpstate = None

class module_state(object):
    def __init__(self):
        self.last_sim_send_time = time.time()
        self.last_apm_send_time = time.time()
        self.rc_channels_scaled = None
        self.hil_state_msg = None
        sim_in_address  = ('127.0.0.1', 5501)
        sim_out_address  = ('127.0.0.1', 5502)

        self.sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_in.bind(sim_in_address)
        self.sim_in.setblocking(0)

        self.sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_out.connect(sim_out_address)
        self.sim_out.setblocking(0)


def name():
    '''return module name'''
    return "HIL"

def description():
    '''return module description'''
    return "HIL module"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.hil_state = module_state()

def unload():
    '''unload module'''
    state = mpstate.hil_state
    state.sim_in.close()
    state.sim_out.close()

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.hil_state
    if m.get_type() == 'RC_CHANNELS_SCALED':
        state.rc_channels_scaled = m

def idle_task():
    '''called from main loop'''
    check_sim_in()
    check_sim_out()
    check_apm_out()

def convert_body_frame(phi, theta, phiDot, thetaDot, psiDot):
    '''convert a set of roll rates from earth frame to body frame'''
    p = phiDot - psiDot*math.sin(theta)
    q = math.cos(phi)*thetaDot + math.sin(phi)*psiDot*math.cos(theta)
    r = math.cos(phi)*psiDot*math.cos(theta) - math.sin(phi)*thetaDot
    return (p, q, r)



def check_sim_in():
    '''check for FDM packets from runsim'''
    state = mpstate.hil_state
    try:
        pkt = state.sim_in.recv(17*8 + 4)
    except socket.error as e:
        if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            raise
        return        
    if len(pkt) != 17*8 + 4:
        # wrong size, discard it
        print("wrong size %u" % len(pkt))
        return
    (latitude, longitude, altitude, heading, v_north, v_east, v_down,
     ax, ay, az,
     phidot, thetadot, psidot,
     roll, pitch, yaw,
     vcas, check) = struct.unpack('<17dI', pkt)
    (p, q, r) = convert_body_frame(radians(roll), radians(pitch), radians(phidot), radians(thetadot), radians(psidot))

    try:
        state.hil_state_msg = mpstate.master().mav.hil_state_encode(int(time.time()*1e6),
                                                                    radians(roll),
                                                                    radians(pitch),
                                                                    radians(yaw),
                                                                    p,
                                                                    q,
                                                                    r,
                                                                    int(latitude*1.0e7),
                                                                    int(longitude*1.0e7),
                                                                    int(altitude*1.0e3),
                                                                    int(v_north*100),
                                                                    int(v_east*100),
                                                                    0,
                                                                    int(ax*1000/9.81),
                                                                    int(ay*1000/9.81),
                                                                    int(az*1000/9.81))
    except Exception:
        return
    

def scale_channel(ch, value):
    '''scale a channel to 1000/1500/2000'''
    master = mpstate.master()
    v = value/10000.0
    if v < -1:
        v = -1
    elif v > 1:
        v = 1
    if ch in [2,4]:
        v = -v
    if ch == 3:
        if v < 0:
            v = 0
        return int(1000 + v*1000)
    return int(1500 + v*500)


def check_sim_out():
    '''check if we should send new servos to flightgear'''
    state = mpstate.hil_state
    now = time.time()
    if now - state.last_sim_send_time < 0.02 or state.rc_channels_scaled is None:
        return
    state.last_sim_send_time = now

    servos = []
    for ch in range(1,9):
        servos.append(scale_channel(ch, getattr(state.rc_channels_scaled, 'chan%u_scaled' % ch)))
    servos.extend([0,0,0, 0,0,0])
    buf = struct.pack('<14H', *servos)
    try:
        state.sim_out.send(buf)
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise
        return        
        

def check_apm_out():
    '''check if we should send new data to the APM'''
    state = mpstate.hil_state
    now = time.time()
    if now - state.last_apm_send_time < 0.02:
        return
    state.last_apm_send_time = now
    if state.hil_state_msg is not None:
        mpstate.master().mav.send(state.hil_state_msg)
        
