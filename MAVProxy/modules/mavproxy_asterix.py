'''
support for asterix SDPS data, setup for OBC 2018

This listens for SDPS on UDP and translates to ADSB_VEHICLE messages
'''

import time
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil

import asterix, socket, time

class Track:
    def __init__(self, adsb_pkt):
        self.pkt = adsb_pkt
        self.last_time = time.time()

    def update(self, pkt):
        t = time.time()
        dt = t - self.last_time
        dist = mp_util.gps_distance(self.pkt.lat*1e-7, self.pkt.lon*1e-7,
                                    pkt.lat*1e-7, pkt.lon*1e-7)
        heading = mp_util.gps_bearing(self.pkt.lat*1e-7, self.pkt.lon*1e-7,
                                      pkt.lat*1e-7, pkt.lon*1e-7)
        spd = dist / dt
        pkt.heading = int(heading*100)
        pkt.hor_velocity = int(spd * 100)
        self.pkt = pkt
        self.last_time = t

class AsterixModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(AsterixModule, self).__init__(mpstate, "asterix", "asterix SDPS data support")
        self.threat_vehicles = {}
        self.active_threat_ids = []  # holds all threat ids the vehicle is evading

        self.add_command('asterix', self.cmd_asterix, "asterix control",
                         ["<start|stop>","set (ASTERIXSETTING)"])

        self.asterix_settings = mp_settings.MPSettings([("port", int, 45454)])
        self.sock = None
        self.tracks = {}

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
        
    def idle_task(self):
        '''called on idle'''
        if self.sock is None:
            return
        try:
            pkt = self.sock.recv(10240)
        except Exception:
            return
        amsg = asterix.parse(pkt)
        for m in amsg:
            lat = m['I105']['Lat']['val']
            lon = m['I105']['Lon']['val']
            alt_f = m['I130']['Alt']['val']
            climb_rate_fps = m['I220']['RoC']['val']
            sac = m['I010']['SAC']['val']
            sic = m['I010']['SIC']['val']
            trkn = m['I040']['TrkN']['val']
            # fake ICAO_address
            icao_address = sac << 16 | sic << 8 | trkn
            squawk = sac << 12 | sic << 8 | trkn
            adsb_pkt = self.master.mav.adsb_vehicle_encode(icao_address,
                                            lat*1e7,
                                            lon*1e7,
                                            mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC,
                                            alt_f*304.8, # mm
                                            0, # heading
                                            0, # hor vel
                                            climb_rate_fps * 30.48,
                                            "%08x" % icao_address,
                                            mavutil.mavlink.ADSB_EMITTER_TYPE_UNASSIGNED,
                                            1.0,
                                            0,
                                            squawk)
            if icao_address in self.tracks:
                self.tracks[icao_address].update(adsb_pkt)
            else:
                self.tracks[icao_address] = Track(adsb_pkt)
            self.master.mav.send(adsb_pkt)
            adsb_mod = self.module('adsb')
            if adsb_mod:
                # the adsb module is loaded, display on the map
                adsb_mod.mavlink_packet(adsb_pkt)
            
def init(mpstate):
    '''initialise module'''
    return AsterixModule(mpstate)
