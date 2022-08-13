#!/usr/bin/env python
'''
nmea GPS connector
connect to a NMEA GPS on a serial port and provide this as location position
'''

import sys, os, serial, time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

try:
    import pynmea2
except ImportError as e:
    print('please install pynmea2 package with "sudo apt install python3-nmea2" or "python -m pip install pynmea2"')

class NMEAGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NMEAGPSModule, self).__init__(mpstate, "NMEAGPS", "NMEA input")
        self.nmeagps_settings = mp_settings.MPSettings([
            ("port", str, None),
            ("baudrate", int, 9600),
            ])
        self.add_completion_function('(NMEAGPSSETTING)',
                                     self.nmeagps_settings.completion)
        self.add_command('nmeagps', self.cmd_nmeagps, "nmea GPS input control",
                         ["<status|connect|disconnect>", "set (NMEAGPSSETTING)"])
        self.port = None
        self.position = mp_util.mp_position()

    def cmd_nmeagps(self, args):
        '''nmeagps commands'''
        usage = "nmeagps <set|connect|disconnect|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.nmeagps_settings.command(args[1:])
        elif args[0] == "connect":
            self.cmd_connect()
        elif args[0] == "disconnect":
            self.cmd_disconnect()
        elif args[0] == "status":
            self.cmd_status()
        else:
            print(usage)

    def cmd_connect(self):
        '''connect to GPS'''
        try:
            self.port = serial.Serial(self.nmeagps_settings.port, self.nmeagps_settings.baudrate)
        except Exception as ex:
            print("Failed to open %s : %s" % (self.nmeagps_settings.port, ex))

    def cmd_disconnect(self):
        '''disconnect from GPS'''
        if self.port is not None:
            self.port.close()
            self.port = None
        else:
            print("GPS not connected")

    def cmd_status(self):
        '''status'''
        if self.port is None:
            print("GPS not connected")
            return
        if self.position.timestamp is None:
            print("No position")
            return
        print(self.position)

    def idle_task(self):
        '''check for new data'''
        if self.port is None:
            return
        line = self.port.readline()
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError as e:
            return
        if not line.startswith("$"):
            return
        if line[3:6] == "GGA":
            msg = pynmea2.parse(line)
            self.position.num_sats = int(msg.num_sats)
            self.position.latitude = msg.latitude
            self.position.longitude = msg.longitude
            self.position.altitude = msg.altitude
            self.position.timestamp = time.time()
            self.mpstate.position = self.position
        if line[3:6] == "RMC":
            msg = pynmea2.parse(line)
            if msg.true_course is not None:
                self.position.ground_course = msg.true_course
            else:
                self.position.ground_course = 0.0
            self.position.ground_speed = msg.spd_over_grnd
            self.position.timestamp = time.time()
            self.mpstate.position = self.position

def init(mpstate):
    '''initialise module'''
    return NMEAGPSModule(mpstate)
