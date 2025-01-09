#!/usr/bin/env python3
'''
Soaring Module
Ryan Friedman

This module displays the estimated soaring thermals on the map.
A circle is drawn with the estimated radius from ArduSoar's estimated thermal location.

Note that this module uses NAMED_VALUE_FLOAT messages from the autopilot,
a temporary measure until a proper mavlink message is decided upon.

AP_FLAKE8_CLEAN
'''

from MAVProxy.modules.lib import mp_module, mp_util
from MAVProxy.modules.mavproxy_map import mp_slipmap

import time


class soar(mp_module.MPModule):
    _SOAR_THERMAL_MAP_OBJ_ID = "soar-thermal"
    _CLEAR_STALE_THERMAL_TIME = 5.0

    def __init__(self, mpstate):
        """Initialise module"""
        super(soar, self).__init__(mpstate, "soar", "")
        self._strength = None
        self._radius = None
        self._x = None
        self._y = None
        self._last_draw_time = time.time()

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        if m.get_type() == 'NAMED_VALUE_FLOAT' and m.name.startswith("SOAR"):
            if m.name == "SOAREKFX0":
                self._strength = m.value
            elif m.name == "SOAREKFX1":
                self._radius = m.value
            elif m.name == "SOAREKFX2":
                self._x = m.value
            elif m.name == "SOAREKFX3":
                self._y = m.value
            else:
                raise NotImplementedError(m.name)

            self.draw_thermal_estimate()

    def idle_task(self):
        '''called rapidly by mavproxy'''

        if time.time() - self._last_draw_time > self._CLEAR_STALE_THERMAL_TIME:
            self.clear_thermal_estimate()

    def draw_thermal_estimate(self):

        if self._radius is None:
            return
        if self._x is None:
            return
        if self._y is None:
            return

        wp_module = self.module('wp')
        if wp_module is None:
            return
        home = wp_module.get_home()
        if home is None:
            return

        home_lat = home.x
        home_lng = home.y

        (thermal_lat, thermal_lon) = mp_util.gps_offset(home_lat, home_lng, self._y, self._x)

        slipcircle = mp_slipmap.SlipCircle(
            self._SOAR_THERMAL_MAP_OBJ_ID, # key
            "thermals", # layer
            (thermal_lat, thermal_lon), # latlon
            self._radius, # radius
            (0, 255, 255),
            linewidth=2)
        for mp in self.module_matching('map*'):
            self._last_draw_time = time.time()
            mp.map.add_object(slipcircle)

    def clear_thermal_estimate(self):
        for mp in self.module_matching('map*'):
            mp.map.remove_object(self._SOAR_THERMAL_MAP_OBJ_ID)


def init(mpstate):
    '''initialise module'''
    return soar(mpstate)
