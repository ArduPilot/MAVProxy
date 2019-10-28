#!/usr/bin/env python
'''
movinghome module
André Kjellstrup, Norce

This module can update the home position of the ArduPilot vehicle to the position of a moving GCS.
requires package python-nmea2

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math
import serial
try:
    import pynmea2
except ImportError as e:
    print("\n!!! missing package !!! do: 'sudo apt install python-nmea2' -movinghome will not work without it")
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class movinghome(mp_module.MPModule):
    def __init__(self, mpstate):
        #Initialise module
        super(movinghome, self).__init__(mpstate, "movinghome", "")
        #latest GCS coordinates
        self.lat = 0
        self.lon = 0
        self.alt = 0
        #last/set home coordinates
        self.lath = 0
        self.lonh = 0  
        self.alth = 0
        #misc settings
        self.device = "/dev/ttyUSB0"
        self.baud = 4800
        self.updating=False
        self.radius = 15 #min travelled distance (m) before update
        self.check_interval = 3 # seconds
        self.last_check = time.time()
        self.fresh = True #fresh start/first movement
        self.dist = 0
        self.last_decode_error_print = 0

        print("\nDefault NMEA source is: %s at %s baud, change if needed before turning on." % (self.device , self.baud))
        self.add_command('movinghome', self.cmd_movinghome, "movinghome module")

            

    def cmd_movinghome(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print("Usage: movinghome <status|on|off|radius|device|baud>\n On/Off: enable position update.\n Radius defines the threshold in meters (2D) from last position, when exceeded, home position is updated.\n device and baud is the serial port setup for NMEA device.")
        elif args[0] == "status":
            self.status()
        elif args[0] == "on":
            self.movinghome_on()
        elif args[0] == "off":
            self.movinghome_off()
        elif args[0] == "radius":
            if len(args) < 2:
                print("Usage: moving base minimum travel radius <RADIUS>")
                return
            self.radius=float(args[1])
        elif args[0] == "device":
            if len(args) < 2:
                print("Usage: device name; device /dev/ttyUSB2")
                return
            self.device=args[1]
        elif args[0] == "baud":
            if len(args) < 2:
                print("Usage: baud rate; baud 9600")
                return
            self.baud=args[1]
        else:
            print(self.usage)

    def status(self):
        #Returns information about module'''
        if self.updating == True:
            print("Last known GCS position lat %(lat)f lon=%(lon)f  max radius=%(max).1fm" %
                   {"lat": self.lath,
                    "lon": self.lonh,
                    "max": self.radius,
                   })
        else:
            print("Not updating home")
        print("Radius is %sm \nInterval is %ss \nDevice is %s at %s baud\n" % (self.radius, self.check_interval, self.device, self.baud ))


    def movinghome_on(self):
        #self.ser = serial.Serial('/dev/ttyUSB0',4800)
        self.ser = serial.Serial(self.device,self.baud)
        self.updating=True
        self.lath = 0 # ensure push of current home.
        print("Home position will be updated if GCS moves > %(max).1fm" %
               {"max": self.radius,
               })

    def movinghome_off(self):
        self.updating = False
        print("Home position will not be updated.")

    def idle_task(self):
        #Called frequently by mavproxy
        if self.updating == True:
            data = self.ser.readline()
            try:
                data = data.decode("ascii")
            except UnicodeDecodeError as e:
                # this is probably a baudrate issue
                now = time.time()
                if now - self.last_decode_error_print > 10:
                    print("movinghome: decode error; baudrate issue?")
                    self.last_decode_error_print = now
                return
            if (data.startswith("$GPGGA")):
                msg = pynmea2.parse(data)
                if int(msg.num_sats) > 5:
                    #convert LAT
                    DD = int(float(msg.lat)/100)
                    MM = float(msg.lat) - DD * 100
                    self.lat = DD + MM/60
                    if msg.lat_dir == "S":
                        self.lat = -self.lat;
                    #convert LON
                    DD = int(float(msg.lon)/100)
                    MM = float(msg.lon) - DD * 100
                    self.lon = DD + MM/60
                    if msg.lon_dir == "W":
                        self.lon = -self.lon;

                now = time.time()
                if now-self.last_check > self.check_interval:
                    self.last_check = now
                    #check if we moved enough
                    self.dist = self.haversine(self.lon, self.lat, self.alt, self.lonh, self.lath, self.alth)
                    if self.dist > self.radius:
                        if self.fresh == True:
                            self.say("GCS position set as home")    
                            self.fresh = False
                        else:
                            message = "GCS moved "
                            message2 = message + "%.0f" % self.dist + "meters"
                            self.say("%s: %s" % (self.name,message2))
                            message2_enc = message2.encode(bytes)
                            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message2)
                        self.console.writeln("home position updated")

                        self.master.mav.command_int_send(
                        self.settings.target_system, self.settings.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                        1, # (1, set current location as home)
                        0, # move on
                        0, # param1
                        0, # param2
                        0, # param3
                        0, # param4
                        int(self.lat*1e7), # param5
                        int(self.lon*1e7), # param6
                        0) # param7

                        self.lath = self.lat
                        self.lonh = self.lon
                        #print data
                        self.console.writeln("%s: %s %s GNSS Quality %s Sats %s "% (self.name,self.lat,self.lon,msg.gps_qual,msg.num_sats))


    def haversine(self, lon1, lat1, alt1, lon2, lat2, alt2):
        r_earth = 6371000
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        d = c*r_earth
        return math.sqrt(d**2+(alt1 - alt2)**2)


def init(mpstate):
    return movinghome(mpstate)
