"""
  MAVProxy terrain handling module
"""

import os, sys, math, time

from MAVProxy.modules.mavproxy_map import mp_elevation
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module

class TerrainModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TerrainModule, self).__init__(mpstate, "terrain", "terrain handling", public=False)

        self.ElevationModel = mp_elevation.ElevationModel()
        self.current_request = None
        self.sent_mask = 0
        self.last_send_time = time.time()
        self.verbose = False

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()
        master = self.master
        # add some status fields
        if type == 'TERRAIN_REQUEST':
            self.current_request = msg
            self.sent_mask = 0
            
    def send_terrain_data_bit(self, bit):
        '''send some terrain data'''
        lat = self.current_request.lat * 1.0e-7
        lon = self.current_request.lon * 1.0e-7
        bit_spacing = self.current_request.grid_spacing * 4
        (lat, lon) = mp_util.gps_offset(lat, lon,
                                        east=bit_spacing * (bit % 8),
                                        north=bit_spacing * (bit // 8))
        data = []
        for i in range(4*4):
            y = i % 4
            x = i // 4
            (lat2,lon2) = mp_util.gps_offset(lat, lon,
                                             east=self.current_request.grid_spacing * y,
                                             north=self.current_request.grid_spacing * x)
            alt = self.ElevationModel.GetElevation(lat2, lon2)
            if alt is None:
                print("no alt ", lat2, lon2)
                return
            data.append(int(alt))
        self.master.mav.terrain_data_send(self.current_request.lat,
                                          self.current_request.lon,
                                          self.current_request.grid_spacing,
                                          bit,
                                          data)
        self.last_send_time = time.time()
        self.sent_mask |= 1<<bit
        if self.verbose and bit == 55:
            lat = self.current_request.lat * 1.0e-7
            lon = self.current_request.lon * 1.0e-7
            print("--lat=%f --lon=%f %.1f" % (
                lat, lon, self.ElevationModel.GetElevation(lat, lon)))
            (lat2,lon2) = mp_util.gps_offset(lat, lon,
                                             east=32*self.current_request.grid_spacing,
                                             north=28*self.current_request.grid_spacing)
            print("--lat=%f --lon=%f %.1f" % (
                lat2, lon2, self.ElevationModel.GetElevation(lat2, lon2)))
                                             

    def send_terrain_data(self):
        '''send some terrain data'''
        for bit in range(56):
            if self.current_request.mask & (1<<bit) and self.sent_mask & (1<<bit) == 0:
                self.send_terrain_data_bit(bit)
                return
        # no bits to send
        self.current_request = None
        self.sent_mask = 0

    def idle_task(self):
        '''called when idle'''
        if self.current_request is None:
            return
        if time.time() - self.last_send_time < 0.2:
            # limit to 5 per second
            return
        self.send_terrain_data()

def init(mpstate):
    '''initialise module'''
    return TerrainModule(mpstate)
