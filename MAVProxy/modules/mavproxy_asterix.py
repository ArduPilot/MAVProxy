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

import asterix, socket, time, os, struct

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
        if dist > 0.01:
            heading = mp_util.gps_bearing(self.pkt.lat*1e-7, self.pkt.lon*1e-7,
                                        pkt.lat*1e-7, pkt.lon*1e-7)
            spd = dist / dt
            pkt.heading = int(heading*100)
            new_vel = int(spd * 100)
            #print(pkt.ICAO_address, new_vel*0.01, pkt.hor_velocity*0.01)
            pkt.hor_velocity = new_vel
        if pkt.hor_velocity > 65535:
            pkt.hor_velocity = 65535
        self.pkt = pkt

class VehiclePos(object):
    def __init__(self, GPI):
        self.lat = GPI.lat * 1.0e-7
        self.lon = GPI.lon * 1.0e-7
        self.alt = GPI.alt * 1.0e-3
        self.vx = GPI.vx * 1.0e-2
        self.vy = GPI.vy * 1.0e-2

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
                                                        ('filter_dist_xy', int, 1000),
                                                        ('filter_dist_z', int, 250),
                                                        ('filter_time', int, 20),
                                                        ('wgs84_to_AMSL', float, -41.2),
                                                        ('filter_use_vehicle2', bool, True),
        ])
        self.add_completion_function('(ASTERIXSETTING)',
                                     self.asterix_settings.completion)
        self.sock = None
        self.tracks = {}
        self.start_listener()

        # storage for vehicle positions, used for filtering
        self.vehicle_pos = None
        self.vehicle2_pos = None

        self.adsb_packets_sent = 0
        self.adsb_packets_not_sent = 0
        self.adsb_byterate = 0 # actually bytes...
        self.adsb_byterate_update_timestamp = 0
        self.adsb_last_packets_sent = 0
        if self.logdir is not None:
            logpath = os.path.join(self.logdir, 'asterix.log')
        else:
            logpath = 'asterix.log'
        self.logfile = open(logpath, 'wb')
        self.pkt_count = 0
        self.console.set_status('ASTX', 'ASTX --/--', row=6)

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
        if abs(lat) < 1.0e-3 and abs(lon) < 1.0e-3:
            return
        self.vehicle2_pos = VehiclePos(m)

    def could_collide_hor(self, vpos, adsb_pkt):
        '''return true if vehicle could come within filter_dist_xy meters of adsb vehicle in timeout seconds'''
        margin = self.asterix_settings.filter_dist_xy
        timeout = self.asterix_settings.filter_time
        alat = adsb_pkt.lat * 1.0e-7
        alon = adsb_pkt.lon * 1.0e-7
        avel = adsb_pkt.hor_velocity * 0.01
        vvel = sqrt(vpos.vx**2 + vpos.vy**2)
        dist = mp_util.gps_distance(vpos.lat, vpos.lon, alat, alon)
        dist -= avel * timeout
        dist -= vvel * timeout
        if dist <= margin:
            return True
        return False

    def could_collide_ver(self, vpos, adsb_pkt):
        '''return true if vehicle could come within filter_dist_z meters of adsb vehicle in timeout seconds'''
        if adsb_pkt.emitter_type < 100 or adsb_pkt.emitter_type > 104:
            return True
        margin = self.asterix_settings.filter_dist_z
        vtype = adsb_pkt.emitter_type - 100
        valt = vpos.alt
        aalt1 = adsb_pkt.altitude * 0.001
        if vtype == 2:
            # weather, always yes
            return True
        if vtype == 4:
            # bird of prey, always true
            return True
        # planes and migrating birds have 150m margin
        aalt2 = aalt1 + adsb_pkt.ver_velocity * 0.01 * self.asterix_settings.filter_time
        altsep1 = abs(valt - aalt1)
        altsep2 = abs(valt - aalt2)
        if altsep1 > 150 + margin and altsep2 > 150 + margin:
            return False
        return True

    def should_send_adsb_pkt(self, adsb_pkt):
        if self.vehicle_pos is not None:
            if self.could_collide_hor(self.vehicle_pos, adsb_pkt) and self.could_collide_ver(self.vehicle_pos, adsb_pkt):
                return True

        if self.vehicle2_pos is not None:
            if self.could_collide_hor(self.vehicle2_pos, adsb_pkt) and self.could_collide_ver(self.vehicle2_pos, adsb_pkt):
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
            if pkt.startswith(b'PICKLED:'):
                pkt = pkt[8:]
                # pickled packet
                try:
                    amsg = [pickle.loads(pkt)]
                except pickle.UnpicklingError:
                    amsg = asterix.parse(pkt)
            else:
                amsg = asterix.parse(pkt)
            self.pkt_count += 1
            self.console.set_status('ASTX', 'ASTX %u/%u' % (self.pkt_count, self.adsb_packets_sent), row=6)
        except Exception:
            print("bad packet")
            return
        try:
            logpkt = b'AST:' + struct.pack('<dI', time.time(), len(pkt)) + pkt
            self.logfile.write(logpkt)
        except Exception:
            pass
        
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

            # use squawk for time in 0.1 second increments. This allows for old msgs to be discarded on vehicle
            # when using more than one link to vehicle
            squawk = (int(self.mpstate.attitude_time_s * 10) & 0xFFFF)

            alt_m = alt_f * 0.3048

            # asterix is WGS84, ArduPilot uses AMSL, which is EGM96
            alt_m += self.asterix_settings.wgs84_to_AMSL

            # consider filtering this packet out; if it's not close to
            # either home or the vehicle position don't send it
            adsb_pkt = self.master.mav.adsb_vehicle_encode(icao_address,
                                                           int(lat*1e7),
                                                           int(lon*1e7),
                                                           mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC,
                                                           int(alt_m*1000), # mm
                                                           0, # heading
                                                           0, # hor vel
                                                           int(climb_rate_fps * 0.3048 * 100), # cm/s
                                                           "%08x" % icao_address,
                                                           100 + (trkn // 10000),
                                                           1.0,
                                                           (mavutil.mavlink.ADSB_FLAGS_VALID_COORDS |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY |
                                                            mavutil.mavlink.ADSB_FLAGS_VALID_HEADING),
                                                           squawk)
            if icao_address in self.tracks:
                self.tracks[icao_address].update(adsb_pkt, self.get_time())
            else:
                self.tracks[icao_address] = Track(adsb_pkt)
            if self.asterix_settings.debug > 0:
                print(adsb_pkt)
            # send on all links
            if self.should_send_adsb_pkt(adsb_pkt):
                self.adsb_packets_sent += 1
                for i in range(len(self.mpstate.mav_master)):
                    conn = self.mpstate.mav_master[i]
                    #if adsb_pkt.hor_velocity < 1:
                    #    print(adsb_pkt)
                    conn.mav.send(adsb_pkt)
            else:
                self.adsb_packets_not_sent += 1

            adsb_mod = self.module('adsb')
            if adsb_mod:
                # the adsb module is loaded, display on the map
                adsb_mod.mavlink_packet(adsb_pkt)

            try:
                for sysid in self.mpstate.sysid_outputs:
                    # fwd to sysid clients
                    adsb_pkt.pack(self.mpstate.sysid_outputs[sysid].mav)
                    self.mpstate.sysid_outputs[sysid].write(adsb_pkt.get_msgbuf())
            except Exception:
                pass
                
        now = time.time()
        delta = now - self.adsb_byterate_update_timestamp
        if delta > 5:
            self.adsb_byterate_update_timestamp = now
            bytes_per_adsb_packet = 38 # FIXME: find constant
            self.adsb_byterate = (self.adsb_packets_sent - self.adsb_last_packets_sent)/delta * bytes_per_adsb_packet
            self.adsb_last_packets_sent = self.adsb_packets_sent

    def mavlink_packet(self, m):
        '''get time from mavlink ATTITUDE'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if abs(m.lat) < 1000 and abs(m.lon) < 1000:
                return
            self.vehicle_pos = VehiclePos(m)

def init(mpstate):
    '''initialise module'''
    return AsterixModule(mpstate)

if __name__ == '__main__':
    import sys
    logname = sys.argv[1]
    logf = open(logname, 'rb')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.connect(('', 45454))
    t0 = None
    tstart = time.time()
    while True:
        header = logf.read(16)
        if header[0:4] != 'AST:':
            print("Bad header", header[0:4])
            break
        (t,len) = struct.unpack('<dI', header[4:16])
        pkt = logf.read(len)
        if t0 is None:
            t0 = t
        tdiff = t - t0
        time.sleep(tdiff)
        t0 = t
        sock.send(pkt)
        #amsg = asterix.parse(pkt)
        #print(amsg)


