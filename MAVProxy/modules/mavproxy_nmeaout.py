#!/usr/bin/env python
'''
nmea serial output module
Matthew Ridley
August 2012

UAV outback challenge search and rescue rules
5.15 Situational Awareness Requirement

It is highly desirable that teams provide:
 - an NMEA 0183 serial output with GPRMC and GPGGA sentences for
   aircraft current location

'''

import sys, os, serial
from MAVProxy.modules.lib import mp_module

class NMEAModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NMEAModule, self).__init__(mpstate, "NMEA", "NMEA output")
        self.port = None
        self.baudrate = 4800
        self.data = 8
        self.parity = 'N'
        self.stop = 1
        self.serial = None
        self.output_time = 0.0
        self.add_command('nmea', self.cmd_nmea, "nmea control")

    def cmd_nmea(self, args):
        '''set nmea'''
        usage = "nmea port [baudrate data parity stop]"
        if len(args) == 0:
            if self.port is None:
                print("NMEA output port not set")
                print(usage)
            else:
                print("NMEA output port %s, %d, %d, %s, %d" % (str(self.port), self.baudrate, self.data, str(self.parity), self.stop))
            return
        if len(args) > 0:
            self.port = str(args[0])
        if len(args) > 1:
            self.baudrate = int(args[1])
        if len(args) > 2:
            self.data = int(args[2])
        if len(args) > 3:
            self.parity = str(args[3])
        if len(args) > 4:
            self.stop = int(args[4])

        if self.serial is not None:
            self.serial.close()
        self.serial = None
        if len(args) > 0:
            if self.port.startswith("/dev/"):
                try:
                    self.serial = serial.Serial(self.port, self.baudrate, self.data, self.parity, self.stop)
                except serial.SerialException as se:
                    print("Failed to open output port %s:%s" % (self.port, se.message))
            else:
                self.serial = open(self.port, mode='w')
            

    def format_date(self, utc_sec):
        import time
        tm_t = time.gmtime(utc_sec)
        return "%02d%02d%02d" % (tm_t.tm_mday, tm_t.tm_mon, tm_t.tm_year % 100)

    def format_time(self, utc_sec):
        import time
        tm_t = time.gmtime(utc_sec)
        subsecs = utc_sec - int(utc_sec);
        return "%02d%02d%05.3f" % (tm_t.tm_hour, tm_t.tm_min, tm_t.tm_sec + subsecs)

    def format_lat(self, lat):
        deg = abs(lat)
        minutes = (deg - int(deg))*60
        return "%02d%08.5f,%c" % (int(deg), minutes, 'S' if lat < 0 else 'N')

    def format_lon(self, lon):
        deg = abs(lon)
        minutes = (deg - int(deg))*60
        return "%03d%08.5f,%c" % (int(deg), minutes, 'W' if lon < 0 else 'E')

	# tst = "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68"
	# print ("*%02X" % nmea_checksum(tst))

    def nmea_checksum(self, msg):
        d = msg[1:]
        cs = 0
        for i in d:
            cs ^= ord(i)
        return cs

    def format_gga(self, utc_sec, lat, lon, fix, nsat, hdop, alt):
        fmt = "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,"
        msg = fmt % (self.format_time(utc_sec), self.format_lat(lat), self.format_lon(lon), fix, nsat, hdop, alt)
        return msg + "*%02X" % self.nmea_checksum(msg)

    def format_rmc(self, utc_sec, fix, lat, lon, speed, course):
        fmt = "$GPRMC,%s,%s,%s,%s,%.2f,%.2f,%s,,"
        msg = fmt % (self.format_time(utc_sec), fix, self.format_lat(lat), self.format_lon(lon),
                     speed, course, self.format_date(utc_sec))
        return msg + "*%02X" % self.nmea_checksum(msg)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        import time

        now_time = time.time()
        if abs(self.output_time - now_time) < 1.0:
            return

        if m.get_type() == 'GPS_RAW_INT':
            # for GPRMC and GPGGA
            utc_sec = now_time
            fix_status = 'A' if (m.fix_type > 1) else 'V'
            lat = m.lat/1.0e7
            lon = m.lon/1.0e7

            # for GPRMC
            knots = ((m.vel/100.0)/1852.0)*3600
            course = m.cog/100.0

            # for GPGGA
            fix_quality = 1 if (m.fix_type > 1) else 0 # 0/1 for (in)valid or 2 DGPS
            num_sat = m.satellites_visible
            hdop = m.eph/100.0
            altitude = m.alt/1000.0

            # print format_gga(utc_sec, lat, lon, fix_quality, num_sat, hdop, altitude)
            # print format_rmc(utc_sec, fix_status, lat, lon, knots, course)
            gga = self.format_gga(utc_sec, lat, lon, fix_quality, num_sat, hdop, altitude)
            rmc = self.format_rmc(utc_sec, fix_status, lat, lon, knots, course)

            self.output_time = now_time
            #print(gga+'\r')
            #print(rmc+'\r')
            #print(self.serial)
            if self.serial is not None:
                self.serial.write(gga + '\r\n')
                self.serial.write(rmc + '\r\n')
                self.serial.flush()
     

def init(mpstate):
    '''initialise module'''
    return NMEAModule(mpstate)
