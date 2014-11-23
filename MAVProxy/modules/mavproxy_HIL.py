#!/usr/bin/env python
'''
HIL module
Andrew Tridgell
December 2012

This interfaces to Tools/autotest/jsbsim/runsim.py to run the JSBSim flight simulator
'''

import sys, os, time, socket, errno, struct, math
from math import degrees, radians
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

class HILModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(HILModule, self).__init__(mpstate, "HIL", "HIL simulation")
        self.last_sim_send_time = time.time()
        self.last_apm_send_time = time.time()
        self.rc_channels_scaled = mavutil.mavlink.MAVLink_rc_channels_scaled_message(0, 0, 0, 0, -10000, 0, 0, 0, 0, 0, 0)
        self.hil_state_msg = None
        sim_in_address  = ('127.0.0.1', 5501)
        sim_out_address  = ('127.0.0.1', 5502)

        self.sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_in.bind(sim_in_address)
        self.sim_in.setblocking(0)

        self.sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_out.connect(sim_out_address)
        self.sim_out.setblocking(0)

        # HIL needs very fast idle loop calls
        if self.settings.select_timeout > 0.001:
            self.settings.select_timeout = 0.001

    def unload(self):
        '''unload module'''
        self.sim_in.close()
        self.sim_out.close()

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'RC_CHANNELS_SCALED':
            self.rc_channels_scaled = m

    def idle_task(self):
        '''called from main loop'''
        self.check_sim_in()
        self.check_sim_out()
        self.check_apm_out()

    def check_sim_in(self):
        '''check for FDM packets from runsim'''
        try:
            pkt = self.sim_in.recv(17*8 + 4)
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
        (p, q, r) = self.convert_body_frame(radians(roll), radians(pitch), radians(phidot), radians(thetadot), radians(psidot))

        try:
            self.hil_state_msg = self.master.mav.hil_state_encode(int(time.time()*1e6),
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




    def check_sim_out(self):
        '''check if we should send new servos to flightgear'''
        now = time.time()
        if now - self.last_sim_send_time < 0.02 or self.rc_channels_scaled is None:
            return
        self.last_sim_send_time = now

        servos = []
        for ch in range(1,9):
            servos.append(self.scale_channel(ch, getattr(self.rc_channels_scaled, 'chan%u_scaled' % ch)))
        servos.extend([0,0,0, 0,0,0])
        buf = struct.pack('<14H', *servos)
        try:
            self.sim_out.send(buf)
        except socket.error as e:
            if not e.errno in [ errno.ECONNREFUSED ]:
                raise
            return


    def check_apm_out(self):
        '''check if we should send new data to the APM'''
        now = time.time()
        if now - self.last_apm_send_time < 0.02:
            return
        self.last_apm_send_time = now
        if self.hil_state_msg is not None:
            self.master.mav.send(self.hil_state_msg)

    def convert_body_frame(self, phi, theta, phiDot, thetaDot, psiDot):
        '''convert a set of roll rates from earth frame to body frame'''
        p = phiDot - psiDot*math.sin(theta)
        q = math.cos(phi)*thetaDot + math.sin(phi)*psiDot*math.cos(theta)
        r = math.cos(phi)*psiDot*math.cos(theta) - math.sin(phi)*thetaDot
        return (p, q, r)

    def scale_channel(self, ch, value):
        '''scale a channel to 1000/1500/2000'''
        v = value/10000.0
        if v < -1:
            v = -1
        elif v > 1:
            v = 1
        if ch == 3 and self.mpstate.vehicle_type != 'rover':
            if v < 0:
                v = 0
            return int(1000 + v*1000)
        return int(1500 + v*500)

def init(mpstate):
    '''initialise module'''
    return HILModule(mpstate)
