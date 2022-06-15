#!/usr/bin/env python
'''
silvus radio support
To use add this in mavinit.scr for vehicle:

 module load silvus
 silvus set gnd_ip 172.20.167.5
 silvus set air_ip 172.20.168.66
 silvus set nmea_ip 172.20.167.5
 silvus set nmea_port 11234 # UDP port for NMEA input to radio

thanks to Felix from Amber Technologies for the code this came from
'''

import time
import datetime
import socket
import requests
import threading
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil

class SilvusModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SilvusModule, self).__init__(mpstate, "Silvus", "Silvus output")
        # filter_dist is distance in metres
        self.silvus_settings = mp_settings.MPSettings([("gnd_ip", str, ""),
                                                       ("air_ip", str, ""),
                                                       ("nmea_ip", str, ""),
                                                       ("nmea_port", int, -1),
                                                       ('log_dt', float, 1.0),
                                                       ('debug', int, 0),
                                                       ])
        self.add_completion_function('(SILVUSSETTING)',
                                     self.silvus_settings.completion)
        self.add_command('silvus', self.cmd_silvus, "silvus control",
                         ["status", "set (SILVUSSETTING)"])
        self.last_nmea_send = time.time()
        self.last_log_time = time.time()

        self.thread = threading.Thread(target=self.thread_loop)
        self.thread.start()
        self.values = {}

    def cmd_silvus(self, args):
        '''silvus commands'''
        if args[0] == "set":
            self.silvus_settings.command(args[1:])
        elif args[0] == "status":
            self.cmd_status()

    def nmea_checkstr(self, msg):
        d = msg[1:]
        cs = 0
        for i in d:
            cs ^= ord(i)
        return "*%02X\r\n" % cs

    def send_nmea(self):
        '''send a NMEA packet to a radio, so the radio knows its position for logging and display'''

        if not self.silvus_settings.nmea_ip or self.silvus_settings.nmea_port <= 0:
            print("no setup")
            return

        now_time = time.time()
        if now_time - self.last_nmea_send < 1.0:
            return
        self.last_nmea_send = now_time

        gps = self.master.messages.get('GPS_RAW_INT', None)
        if gps is None or gps.fix_type < 3:
            return

        lat = gps.lat * 1.0e-7
        lon = gps.lon * 1.0e-7
        alt = gps.alt * 1.0e-3
        nsat = gps.satellites_visible
        hdop = gps.eph/100.0
        fix = gps.fix_type
        speed = ((gps.vel/100.0)/1852.0)*3600 # knots
        course = gps.cog/100.0

        output = ""
        utc_sec = now_time
        tm_t = time.gmtime(utc_sec)
        dstr = "%02d%02d%02d" % (tm_t.tm_mday, tm_t.tm_mon, tm_t.tm_year % 100)
        subsecs = utc_sec - int(utc_sec);
        tstr = "%02d%02d%05.3f" % (tm_t.tm_hour, tm_t.tm_min, tm_t.tm_sec + subsecs)
        deg = abs(lat)
        minutes = (deg - int(deg))*60
        latstr = "%02d%08.5f,%c" % (int(deg), minutes, 'S' if lat < 0 else 'N')
        deg = abs(lon)
        minutes = (deg - int(deg))*60
        lonstr = "%03d%08.5f,%c" % (int(deg), minutes, 'W' if lon < 0 else 'E')

        gga = "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,," % (tstr, latstr, lonstr, fix, nsat, hdop, alt)
        output = output + gga + self.nmea_checkstr(gga)

        rmc = "$GPRMC,%s,%s,%s,%s,%.2f,%.2f,%s,," % (tstr, fix, latstr, lonstr, speed, course, dstr)
        output = output + rmc + self.nmea_checkstr(rmc)

        if self.silvus_settings.debug > 1:
            print("NMEA: %s" % output)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.sendto(output.encode("UTF-8"), (self.silvus_settings.nmea_ip, self.silvus_settings.nmea_port))
        except Exception as ex:
            print("Silvus NMEA send fail: %s" % ex)

    def url(self, nodeip, api):
        return "http://%s/%s" % (nodeip, api)

    # Get Frequency == freq (nodeip)
    def get_freq(self, nodeip):
        data = '{"jsonrpc":"2.0","method":"freq","id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        freql = (response.json()["result"])
        freq = int(freql[0])
        return freq

    # Get NoiseLevel == noise(nodeip)
    def get_noise(self, nodeip):
        data = '{"jsonrpc":"2.0","method":"noise_level","id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        noisel = (response.json()["result"])
        return int(noisel[0])

    # Get Neighbor RSSI == nbr_rssi(nodeip, localnode)
    def get_rssi(self, nodeip, remotenode):
        data = '{"jsonrpc":"2.0","method":"nbr_rssi","params":["' + remotenode + '"],"id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        nbr_rssi = (response.json()["result"])
        return nbr_rssi

    # Get network Status, output as an array
    def network_status(self, nodeip):
        data = '{"jsonrpc":"2.0","method":"network_status","id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        netstat = (response.json()["result"])
        return netstat

    # Get GPS status
    def get_gps_state(self, nodeip):
        data = '{"jsonrpc":"2.0","method":"gps_mode","id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        gpsstat = (response.json()["result"])
        return gpsstat

    # Get max throughput between nodes
    def get_throughput(self, nodeip, remotenode):
        data = '{"jsonrpc":"2.0","method":"link_throughput","params":["' + remotenode + '", "1"],"id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        nbr_tp = (response.json()["result"])
        nbr_tp = (nbr_tp)[0]
        return nbr_tp

    # Returns the TX MCS
    def get_neighbor_mcs(self, nodeip, remotenode):
        data = '{"jsonrpc":"2.0","method":"nbr_mcs","params":["' + remotenode + '"],"id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        nbr_mcs = (response.json()["result"])
        nbr_mcs = (nbr_mcs)[0]
        # print(data)
        return nbr_mcs

    # Returns the RX MCS
    def get_neighbor_mcs_rx(self, nodeip, remotenode):
        data = '{"jsonrpc":"2.0","method":"nbr_mcs_rx","params":["' + remotenode + '"],"id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        nbr_mcs_rx = (response.json()["result"])
        nbr_mcs_rx = (nbr_mcs_rx)[0]
        # print(data)
        return nbr_mcs_rx

    # Get GPS coords, output as an array
    def get_gps_coords(self, nodeip):
        data = '{"jsonrpc":"2.0","method":"gps_coordinates","id":"sbkb5u0c"}'
        response = requests.post(self.url(nodeip, 'streamscape_api'), data=data)
        coords = (response.json()["result"])
        return coords

    # convert last 2 octets of IP to a nodeid
    def ip_to_id(self, ip):
        ip = ip.split('.')
        return ((int(ip[2]) * 256) + int(ip[3]))

    def send_named_float(self, name, value):
        '''inject a NAMED_VALUE_FLOAT into the local master input, so it becomes available
           for graphs, logging and status command'''

        # use the ATTITUDE message for srcsystem and time stamps
        att = self.master.messages.get('ATTITUDE',None)
        if att is None:
            return
        msec = att.time_boot_ms
        ename = name.encode('ASCII')
        if len(ename) < 10:
            ename += bytes([0] * (10-len(ename)))
        m = self.master.mav.named_value_float_encode(msec, bytearray(ename), value)
        #m.name = ename
        m.pack(self.master.mav)
        m._header.srcSystem = att._header.srcSystem
        m._header.srcComponent = mavutil.mavlink.MAV_COMP_ID_TELEMETRY_RADIO
        m.name = name
        self.mpstate.module('link').master_callback(m, self.master)
        if self.silvus_settings.debug > 0:
            print(m)

    def get_radio_data(self):
        now = time.time()
        if now - self.last_log_time < self.silvus_settings.log_dt:
            return
        self.last_log_time = now

        localip = self.silvus_settings.gnd_ip
        remoteip = self.silvus_settings.air_ip

        if len(localip.split('.')) != 4:
            return
        if len(remoteip.split('.')) != 4:
            return

        localnode = str(self.ip_to_id(localip))
        remotenode = str(self.ip_to_id(remoteip))

        self.values['TXMCS'] = float(self.get_neighbor_mcs(localip, remotenode))
        self.values['RXMCS'] = float(self.get_neighbor_mcs_rx(localip, remotenode))
        rssi = self.get_rssi(localip, remotenode)
        self.values['TXRSSI1'] = float(rssi[0])
        self.values['TXRSSI2'] = float(rssi[1])
        self.values['TXRSSI3'] = float(rssi[2])
        self.values['TXRSSI4'] = float(rssi[3])
        rssi = self.get_rssi(remoteip, localnode)
        self.values['RXRSSI1'] = float(rssi[0])
        self.values['RXRSSI2'] = float(rssi[1])
        self.values['RXRSSI3'] = float(rssi[2])
        self.values['RXRSSI4'] = float(rssi[3])
        self.values['LOCNSE'] = float(self.get_noise(localip))
        self.values['REMNSE'] = float(self.get_noise(remoteip))
        self.values['LINKSNR'] = float(self.network_status(localip)[2])
        self.values['LOCTPUT'] = float(self.get_throughput(localip, remotenode))
        self.values['REMTPUT'] = float(self.get_throughput(remoteip, localnode))

        for f in self.values:
            self.send_named_float('SR_' + f, self.values[f])

    def cmd_status(self):
        for f in sorted(self.values.keys()):
            print("%20s %.1f" % (f, self.values[f]))

    def thread_loop(self):
        while True:
            time.sleep(self.silvus_settings.log_dt)
            try:
                self.send_nmea()
                self.get_radio_data()
            except Exception as ex:
                if self.silvus_settings.debug > 0:
                    print(ex)


def init(mpstate):
    '''initialise module'''
    return SilvusModule(mpstate)
