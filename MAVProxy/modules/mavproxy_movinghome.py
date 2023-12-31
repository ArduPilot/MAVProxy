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

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class movinghome(mp_module.MPModule):
    def __init__(self, mpstate):
        # Initialise module
        super(movinghome, self).__init__(mpstate, "movinghome", "")
        # last/set home coordinates
        self.lath = 0
        self.lonh = 0
        self.alth = 0
        # misc settings
        self.updates_enabled = False
        self.radius = 15 # min travelled distance (m) before update
        self.check_interval = 3 # seconds
        self.last_check = time.time()
        self.fresh = True # fresh start/first movement
        self.dist = 0

        self.add_command('movinghome', self.cmd_movinghome, "movinghome module")



    def cmd_movinghome(self, args):
        '''control behaviour of the module'''

        usage = "Usage: movinghome <status|on|off|radius|device|baud>\n On/Off: enable position update.\n Radius defines the threshold in meters (2D) from last position, when exceeded, home position is updated."

        if len(args) == 0:
            print(usage)
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
        else:
            print(usage)

    def status(self):
        # Returns information about module'''
        if self.updates_enabled == True:
            print("Last known GCS position lat %(lat)f lon=%(lon)f  max radius=%(max).1fm" %
                   {"lat": self.lath,
                    "lon": self.lonh,
                    "max": self.radius,
                   })
        else:
            print("Home position updates not enabled")
        print("Radius is %sm \nInterval is %ss \n" % (self.radius, self.check_interval))


    def movinghome_on(self):
        self.updates_enabled = True
        self.lath = 0 # ensure push of current home.
        print("Home position will be updated if GCS moves > %(max).1fm" %
               {"max": self.radius,
               })

    def movinghome_off(self):
        self.updates_enabled = False
        print("Home position will not be updated.")

    def idle_task(self):

        # Called frequently by mavproxy
        if not self.updates_enabled:
            return

        position = self.mpstate.position

        time_now = time.time()
        # TODO maybe add a check on position.timestamp here
        # e.g. if not position or (position && (time.time() - position.timestamp > some falue)
        # print error and return


        # check if we have a position and the position contains actual data from
        # a GGA message, if there is a latitude the rest exists as well
        if position and position.latitude:
            if position.num_sats > 5 and (time_now - self.last_check > self.check_interval) :

                self.last_check = time_now
                # check if we moved enough
                self.dist = self.haversine(
                        position.longitude,
                        position.latitude,
                        position.altitude,
                        self.lonh,
                        self.lath,
                        self.alth
                        )

                if self.dist > self.radius:
                    if self.fresh == True:
                        self.say("GCS position set as home")
                        self.fresh = False
                    else:
                        message = "GCS moved %.0f" % self.dist + "meters"
                        self.say("%s: %s" % (self.name, message))
                        self.master.mav.statustext_send(
                                mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                message.encode("utf-8")
                                )
                    self.console.writeln("Home position updated")

                    self.master.mav.command_int_send(
                        self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                        1, # (1, set current location as home)
                        0, # move on
                        0, # param1
                        0, # param2
                        0, # param3
                        0, # param4
                        int(position.latitude * 1e7), # param5
                        int(position.longitude * 1e7), # param6
                        0 # param7
                    )

                    self.lath = position.latitude
                    self.lonh = position.longitude
                    # self.console.writeln("%s: %s %s GNSS Quality %s Sats %s " % (self.name, self.lat, self.lon, msg.gps_qual, msg.num_sats))


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
