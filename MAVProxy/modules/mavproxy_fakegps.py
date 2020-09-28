#!/usr/bin/env python
'''fake GPS input using GPS_INPUT packet'''

import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class FakeGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FakeGPSModule, self).__init__(mpstate, "fakegps")
        self.last_send = time.time()
        self.FakeGPS_settings = mp_settings.MPSettings([("nsats", int, 16),
                                                        ("lat", float, -35.363261),
                                                        ("lon", float, 149.165230),
                                                        ("alt", float, 584.0),
                                                        ("rate", float, 5)])
        self.add_command('fakegps', self.cmd_FakeGPS, "fakegps control",
                         ["<status>", "set (FAKEGPSSETTING)"])
        self.add_completion_function('(FAKEGPSSETTING)',
                                     self.FakeGPS_settings.completion)
        if mp_util.has_wxpython:
            map = self.module('map')
            if map is not None:
                menu = MPMenuSubMenu('FakeGPS',
                                    items=[MPMenuItem('SetPos', 'SetPos', '# fakegps setpos'),
                                           MPMenuItem('SetPos (with alt)', 'SetPosAlt', '# fakegps setpos ',
                                                    handler=MPMenuCallTextDialog(title='Altitude (m)', default=self.mpstate.settings.guidedalt))])
                map.add_menu(menu)

    def cmd_FakeGPS(self, args):
        '''fakegps command parser'''
        usage = "usage: fakegps <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.FakeGPS_settings.command(args[1:])
        elif args[0] == "setpos":
            self.cmd_setpos(args[1:])
        else:
            print(usage)

    def cmd_setpos(self, args):
        '''set pos from map'''
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return
        (lat,lon) = latlon
        self.FakeGPS_settings.lat = lat
        self.FakeGPS_settings.lon = lon
        if len(args) > 0:
            self.FakeGPS_settings.alt = float(args[0])

    def get_gps_time(self, tnow):
        '''return gps_week and gps_week_ms for current time'''
        leapseconds = 18
        SEC_PER_WEEK = 7 * 86400

        epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
        epoch_seconds = int(tnow - epoch)
        week = int(epoch_seconds) // SEC_PER_WEEK
        t_ms = int(tnow * 1000) % 1000
        week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
        return week, week_ms

    def idle_task(self):
        '''called on idle'''
        now = time.time()
        if now - self.last_send < 1.0 / self.FakeGPS_settings.rate:
            return
        self.last_send = now
        gps_lat = self.FakeGPS_settings.lat
        gps_lon = self.FakeGPS_settings.lon
        gps_alt = self.FakeGPS_settings.alt
        gps_vel = [0, 0, 0]

        gps_week, gps_week_ms = self.get_gps_time(now)

        time_us = int(now*1.0e6)

        nsats = self.FakeGPS_settings.nsats
        if nsats >= 6:
            fix_type = 3
        else:
            fix_type = 1
        self.master.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
                                       int(gps_lat*1.0e7), int(gps_lon*1.0e7), gps_alt,
                                       1.0, 1.0,
                                       gps_vel[0], gps_vel[1], gps_vel[2],
                                       0.2, 1.0, 1.0,
                                       nsats)


def init(mpstate):
    '''initialise module'''
    return FakeGPSModule(mpstate)
