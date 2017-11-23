#!/usr/bin/env python
'''
support for GPS_INPUT message
'''

import socket, errno
import json
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class GPSInputModule(mp_module.MPModule):

    IGNORE_FLAG_ALL =  (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_ALT |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)

    def __init__(self, mpstate):
        super(GPSInputModule, self).__init__(mpstate, "GPSInput", "GPS_INPUT message support")
        self.add_command('GPSInput.port', self.cmd_port, 'Port selection', ['<25100>'])
        self.data = {
            'time_usec' : 0,                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
            'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
            'ignore_flags' : self.IGNORE_FLAG_ALL,  # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
            'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
            'time_week' : 0,                        # (uint16_t) GPS week number
            'fix_type' : 0,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            'lat' : 0,                              # (int32_t) Latitude (WGS84), in degrees * 1E7
            'lon' : 0,                              # (int32_t) Longitude (WGS84), in degrees * 1E7
            'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
            'hdop' : 0,                             # (float) GPS HDOP horizontal dilution of position in m
            'vdop' : 0,                             # (float) GPS VDOP vertical dilution of position in m
            'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
            'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
            'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
            'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
            'satellites_visible' : 0                # (uint8_t) Number of satellites visible.
        }
        
        self.BUFFER_SIZE = 4096
        self.ip="127.0.0.1"
        self.portnum = 25100
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind((self.ip, self.portnum))
        self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print("Listening for GPS Input packets on UDP://%s:%s" % (self.ip, self.portnum))


    def idle_task(self):
        '''called in idle time'''
        
        try:
            datagram = self.port.recvfrom(self.BUFFER_SIZE)
            data = json.loads(datagram[0])
            
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise
        
        for key in data.keys():
            self.data[key] = data[key]
        
        try:
            self.master.mav.gps_input_send(
                self.data['time_usec'],
                self.data['gps_id'],
                self.data['ignore_flags'],
                self.data['time_week_ms'],
                self.data['time_week'],
                self.data['fix_type'],
                self.data['lat'],
                self.data['lon'],
                self.data['alt'],
                self.data['hdop'],
                self.data['vdop'],
                self.data['vn'],
                self.data['ve'],
                self.data['vd'],
                self.data['speed_accuracy'],
                self.data['horiz_accuracy'],
                self.data['vert_accuracy'],
                self.data['satellites_visible'])
        
        except Exception as e:
            print("GPS Input Failed:", e)


    def cmd_port(self, args):
        'handle port selection'
        if len(args) != 1:
            print("Usage: port <number>")
            return
        
        self.port.close()
        self.portnum = int(args[0])
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind((self.ip, self.portnum))
        self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print("Listening for GPS INPUT packets on UDP://%s:%s" % (self.ip, self.portnum))


def init(mpstate):
    '''initialise module'''
    return GPSInputModule(mpstate)
