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

        # filter_dist is distance in metres
        self.asterix_settings = mp_settings.MPSettings([("port", int, 45454),
                                                        ('debug', int, 0),
                                                        ('filter_dist', int, -1),
                                                        ('filter_use_vehicle2', bool, True),
        ])
        self.add_completion_function('(ASTERIXSETTING)',
                                     self.asterix_settings.completion)
        self.sock = None
        self.tracks = {}
        self.tnow = 0
        self.start_listener()

        # storage for vehicle positions, used for filtering
        self.vehicle_lat = None
        self.vehicle_lon = None
        self.vehicle2_lat = None
        self.vehicle2_lon = None
        self.adsb_packets_sent = 0
        self.adsb_packets_not_sent = 0
        self.adsb_byterate = 0 # actually bytes...
        self.adsb_byterate_update_timestamp = 0
        self.adsb_last_packets_sent = 0

    def print_status(self):
        print("ADSB packets sent: %u" % self.adsb_packets_sent)
        print("ADSB packets not sent: %u" % self.adsb_packets_not_sent)
        print("ADSB bitrate: %u bytes/s" % int(self.adsb_byterate))

    def cmd_asterix(self, args):
        '''asterix command parser'''
        usage = "usage: asterix <set|start|stop|restart|status>"
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
        elif args[0] == "status":
            self.print_status()
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

    def set_secondary_vehicle_position(self, m):
        '''store second vehicle position for filtering purposes'''
        if m.get_type() != 'GLOBAL_POSITION_INT':
            return
        (lat, lon, heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
        if abs(lat) < 1.0e-3 and abs(lon) > 1.0e-3:
            return

    def should_send_adsb_pkt(self, adsb_pkt):
        if self.asterix_settings.filter_dist <= 0:
            return True

        # only filter packets out if vehicle's position is known:
        if self.vehicle_lat is None:
            return True

        adsb_pkt_lat = adsb_pkt.lat*1.0e-7
        adsb_pkt_lon = adsb_pkt.lon*1.0e-7

        dist = mp_util.gps_distance(adsb_pkt_lat,
                                    adsb_pkt_lon,
                                    self.vehicle_lat,
                                    self.vehicle_lon)
        if dist <= self.asterix_settings.filter_dist:
            return True

        if self.asterix_settings.filter_use_vehicle2:
            # only filter packets out if vehicle's position is known:
            if self.vehicle2_lat is None:
                return True
            dist = mp_util.gps_distance(adsb_pkt_lat,
                                        adsb_pkt_lon,
                                        self.vehicle2_lat,
                                        self.vehicle2_lon)
            if dist <= self.asterix_settings.filter_dist:
                return True

        return False

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

            # consider filtering this packet out; if it's not close to
            # either home or the vehicle position don't send it
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
            # send on all links
            if self.should_send_adsb_pkt(adsb_pkt):
                self.adsb_packets_sent += 1
                for i in range(len(self.mpstate.mav_master)):
                    conn = self.mpstate.mav_master[i]
                    conn.mav.send(adsb_pkt)
            else:
                self.adsb_packets_not_sent += 1

            adsb_mod = self.module('adsb')
            if adsb_mod:
                # the adsb module is loaded, display on the map
                adsb_mod.mavlink_packet(adsb_pkt)


        now = time.time()
        delta = now - self.adsb_byterate_update_timestamp
        if delta > 5:
            self.adsb_byterate_update_timestamp = now
            bytes_per_adsb_packet = 38 # FIXME: find constant
            self.adsb_byterate = (self.adsb_packets_sent - self.adsb_last_packets_sent)/delta * bytes_per_adsb_packet
            self.adsb_last_packets_sent = self.adsb_packets_sent

    def mavlink_packet(self, m):
        '''get time from mavlink ATTITUDE'''
        if m.get_type() == 'ATTITUDE':
            self.tnow = m.time_boot_ms*0.001
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.vehicle_lat = m.lat*1.0e-7
            self.vehicle_lon = m.lon*1.0e-7

def init(mpstate):
    '''initialise module'''
    return AsterixModule(mpstate)
