#!/usr/bin/env python
"""
park Module
André Kjellstrup, 2019

This module notifies the user if the vehicle moves more than a preset distance(default is 2m)
in lateral or vertical direction from the "parked" point.
"""

from pymavlink import mavutil
import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util


class park(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(park, self).__init__(mpstate, "park", "")
        # latest coordinates
        self.lat = 0
        self.lon = 0
        self.alt = 0
        # parked coordinates
        self.latp = 0
        self.lonp = 0
        self.altp = 0
        # misc settings
        self.parked = False
        self.radius = 2  # max distance
        self.check_interval = 1  # 1=check every second
        self.last_check = time.time()
        self.notify_interval = 4  # minimum seconds between notifications
        self.last_notification = time.time()
        self.dist = 0
        self.usage = "Usage: park <status|on|off|radius>\n On: enable departure warning.\n Radius defines the" \
                     " distance in meters (3D) from parked position, when exceeded an alarm is raised."
#        self.park_settings = mp_settings.MPSettings(
#            [('verbose', bool, False),('versee', bool, True),])
        self.add_command('park', self.cmd_park, "park module")

    def cmd_park(self, args):
        """control behaviour of the module"""
        if len(args) == 0:
            print(self.usage)
        elif args[0] == "status":
            self.status()
        elif args[0] == "on":
            self.park_on()
        elif args[0] == "off":
            self.park_off()
        elif args[0] == "radius":
            if len(args) < 2:
                print("Usage: park radius <RADIUS>")
                return
            self.radius = float(args[1])
        else:
            print(self.usage)

    def status(self):
        """returns information about module"""
        if self.parked:
            print("Parked at lat:", self.lat, "lon:", self.lon, "alt:", self.alt, "max radius", self.radius, "m")
        else:
            print("Not parked")
            
    def park_on(self):
        """set park"""
        self.latp = self.lat
        self.lonp = self.lon
        self.altp = self.alt
        self.parked = True
        print("Parked at lat:", self.lat, "lon:", self.lon, "alt:", self.alt,
              "you will be notified if it moves >", self.radius, "m")

    def park_off(self):
        self.parked = False
        print("Park warning is off")

    def idle_task(self):
        """called frequently by mavproxy"""
        now = time.time()
        if self.parked and now-self.last_check > self.check_interval:
            self.last_check = now
            self.dist = mp_util.gps_distance(self.lat, self.lon, self.latp, self.lonp)
            if (self.dist > self.radius or abs(self.alt - self.altp) > self.radius) and \
                    now-self.last_notification > self.notify_interval:
                self.last_notification = now
                message = "Vehicle moved "
                message2 = message + "%.1f" % self.dist + "meters"
                self.say("%s: %s" % (self.name, message2))
                self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message2)

    def mavlink_packet(self, m):
        """handle mavlink packets"""
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.lat = m.lat * 1.0e-7
            self.lon = m.lon * 1.0e-7
            self.alt = m.relative_alt * 1.0e-3


def init(mpstate):
    """initialise module"""
    return park(mpstate)
