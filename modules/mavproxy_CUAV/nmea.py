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
from cuav.lib import cuav_util

mpstate = None

class module_state(object):
    def __init__(self):
        self.port = None
        self.baudrate = 4800
        self.data = 8
        self.parity = 'N'
        self.stop = 1
        self.serial = None
        self.output_time = 0.0

def name():
    '''return module name'''
    return "nmea"

def description():
    '''return module description'''
    return "nmea serial output module"

def cmd_nmea(args):
    '''set nmea'''
    state = mpstate.nmea_state
    usage = "nmea port [baudrate data parity stop]"
    if len(args) == 0:
      if state.port is None:
        print("NMEA output port not set")
        print usage
      else:
        print("NMEA output port %s, %d, %d, %s, %d" % (str(state.port), state.baudrate, state.data, str(state.parity), state.stop))
      return
    if len(args) > 0:
      state.port = str(args[0])
    if len(args) > 1:
      state.baudrate = int(args[1])
    if len(args) > 2:
      state.data = int(args[2])
    if len(args) > 3:
      state.parity = str(args[3])
    if len(args) > 4:
      state.stop = int(args[4])

    if state.serial is not None:
        state.serial.close()
    state.serial = None
    if (len(args) > 0):
        if state.port.startswith("/dev/"):
            try:
                state.serial = serial.Serial(state.port, state.baudrate, state.data, state.parity, state.stop)
            except serial.SerialException as se:
                print("Failed to open output port %s:%s" % (state.port, se.message))
        else:
            state.serial = open(state.port, mode='w')
            


def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.nmea_state = module_state()
    mpstate.command_map['nmea'] = (cmd_nmea, "nmea control")

def unload():
    '''unload module'''
    pass

def format_date(utc_sec):
    import time
    tm_t = time.gmtime(utc_sec)
    return "%02d%02d%02d" % (tm_t.tm_mday, tm_t.tm_mon, tm_t.tm_year % 100)

def format_time(utc_sec):
    import time
    tm_t = time.gmtime(utc_sec)
    subsecs = utc_sec - int(utc_sec);
    return "%02d%02d%05.3f" % (tm_t.tm_hour, tm_t.tm_min, tm_t.tm_sec + subsecs)

def format_lat(lat):
    deg = abs(lat)
    minutes = (deg - int(deg))*60
    return "%02d%08.5f,%c" % (int(deg), minutes, 'S' if lat < 0 else 'N')

def format_lon(lon):
    deg = abs(lon)
    minutes = (deg - int(deg))*60
    return "%03d%08.5f,%c" % (int(deg), minutes, 'W' if lon < 0 else 'E')

#tst = "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68"
#print ("*%02X" % nmea_checksum(tst))

def nmea_checksum(msg):
    d = msg[1:]
    cs = 0
    for i in d:
      cs ^= ord(i)
    return cs

def format_gga(utc_sec, lat, lon, fix, nsat, hdop, alt):
    fmt = "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,"
    msg = fmt % (format_time(utc_sec), format_lat(lat), format_lon(lon), fix, nsat, hdop, alt)
    return msg + "*%02X" % nmea_checksum(msg)

def format_rmc(utc_sec, fix, lat, lon, speed, course):
    fmt = "$GPRMC,%s,%s,%s,%s,%.2f,%.2f,%s,,"
    msg = fmt % (format_time(utc_sec), fix, format_lat(lat), format_lon(lon), speed, course, format_date(utc_sec))
    return msg + "*%02X" % nmea_checksum(msg)

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    import time
    state = mpstate.nmea_state

    now_time = time.time()
    if abs(state.output_time - now_time) < 1.0:
      return

    if m.get_type() == 'GPS_RAW_INT':
      #for GPRMC and GPGGA
      utc_sec = now_time
      fix_status = 'A' if (m.fix_type > 1) else 'V'
      lat = m.lat/1.0e7
      lon = m.lon/1.0e7

      #for GPRMC
      knots = ((m.vel/100.0)/1852.0)*3600
      course = m.cog/100.0

      #for GPGGA
      fix_quality = 1 if (m.fix_type > 1) else 0 # 0/1 for (in)valid or 2 DGPS
      num_sat = m.satellites_visible
      hdop = m.eph/100.0
      altitude = m.alt/1000.0

      #print format_gga(utc_sec, lat, lon, fix_quality, num_sat, hdop, altitude)
      #print format_rmc(utc_sec, fix_status, lat, lon, knots, course)
      gga = format_gga(utc_sec, lat, lon, fix_quality, num_sat, hdop, altitude)
      rmc = format_rmc(utc_sec, fix_status, lat, lon, knots, course)

      state.output_time = now_time
      #print gga+'\r'
      #print rmc+'\r'
      if state.serial is not None:
        state.serial.write(gga + '\r\n')
        state.serial.write(rmc + '\r\n')
        state.serial.flush()
        
