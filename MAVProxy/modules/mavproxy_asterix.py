'''
support for asterix SDPS data, setup for OBC 2018

This listens for SDPS on UDP and translates to ADSB_VEHICLE messages
'''

import pickle
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil

import asterix, socket, time

class Track:
    def __init__(self, adsb_pkt):
        self.pkt = adsb_pkt
        self.last_time = 0

    def update(self, pkt, tnow):
        dt = tnow - self.last_time
        if dt < 0 or dt > 10:
            self.last_time = tnow
            return
        if dt < 0.1:
            return
        self.last_time = tnow
        dist = mp_util.gps_distance(self.pkt.lat*1e-7, self.pkt.lon*1e-7,
                                    pkt.lat*1e-7, pkt.lon*1e-7)
        heading = mp_util.gps_bearing(self.pkt.lat*1e-7, self.pkt.lon*1e-7,
                                      pkt.lat*1e-7, pkt.lon*1e-7)
        spd = dist / dt
        pkt.heading = int(heading*100)
        pkt.hor_velocity = int(spd * 100)
        if pkt.hor_velocity > 65535:
            pkt.hor_velocity = 65535
        self.pkt = pkt

class AsterixModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(AsterixModule, self).__init__(mpstate, "asterix", "asterix SDPS data support")
        self.threat_vehicles = {}
        self.active_threat_ids = []  # holds all threat ids the vehicle is evading

        self.add_command('asterix', self.cmd_asterix, "asterix control",
                         ["<start|stop>","set (ASTERIXSETTING)"])

        self.asterix_settings = mp_settings.MPSettings([("port", int, 45454),
                                                        ('debug', int, 0)])
        self.sock = None
        self.tracks = {}
        self.tnow = 0
        self.start_listener()

    def cmd_asterix(self, args):
        '''asterix command parser'''
        usage = "usage: asterix <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.asterix_settings.command(args[1:])
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
        if self.sock is not None:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.asterix_settings.port))
        self.sock.setblocking(False)
        print("Started on port %u" % self.asterix_settings.port)

    def stop_listener(self):
        '''stop listening for packets'''
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        self.tracks = {}
        
    def idle_task(self):
        '''called on idle'''
        if self.sock is None:
            return
        try:
            pkt = self.sock.recv(10240)
        except Exception:
            return
        try:
            if pkt[0] == '(':
                # pickled packet
                try:
                    amsg = [pickle.loads(pkt)]
                except pickle.UnpicklingError:
                    amsg = asterix.parse(pkt)
            else:
                amsg = asterix.parse(pkt)
        except Exception:
            print("bad packet")
            return
        
        for m in amsg:
            if self.asterix_settings.debug > 1:
                print(m)
            lat = m['I105']['Lat']['val']
            lon = m['I105']['Lon']['val']
            alt_f = m['I130']['Alt']['val']
            climb_rate_fps = m['I220']['RoC']['val']
            sac = m['I010']['SAC']['val']
            sic = m['I010']['SIC']['val']
            trkn = m['I040']['TrkN']['val']
            # fake ICAO_address
            icao_address = trkn & 0xFFFF
            squawk = icao_address
            adsb_pkt = self.master.mav.adsb_vehicle_encode(icao_address,
                                                           int(lat*1e7),
                                                           int(lon*1e7),
                                                           mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC,
                                                           int(alt_f*304.8), # mm
                                                           0, # heading
                                                           0, # hor vel
                                                           int(climb_rate_fps * 30.48),
                                                           "%08x" % icao_address,
                                                           100 + (trkn // 10000),
                                                           1.0,
                                                           (mavutil.mavlink.ADSB_FLAGS_VALID_COORDS |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_HEADING),
                                                           squawk)
            if icao_address in self.tracks:
                self.tracks[icao_address].update(adsb_pkt, self.tnow)
            else:
                self.tracks[icao_address] = Track(adsb_pkt)
            if self.asterix_settings.debug > 0:
                print(adsb_pkt)
            self.master.mav.send(adsb_pkt)
            adsb_mod = self.module('adsb')
            if adsb_mod:
                # the adsb module is loaded, display on the map
                adsb_mod.mavlink_packet(adsb_pkt)

    def mavlink_packet(self, m):
        '''get time from mavlink ATTITUDE'''
        if m.get_type() == 'ATTITUDE':
            self.tnow = m.time_boot_ms*0.001
            
def init(mpstate):
    '''initialise module'''
    return AsterixModule(mpstate)
