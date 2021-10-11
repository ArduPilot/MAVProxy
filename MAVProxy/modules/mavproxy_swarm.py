#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

#TODO: Make Swarming happen in a different thread
class SwarmModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SwarmModule, self).__init__(mpstate, "swarm control", public=True)
        self.add_command('swarm', self.cmd_swarm, "swarm control", ["<draw|radius>"])
        #self.add_command('swarm_radius', self.__decode_swarm_radius, "Decode Swarm Radius MAV_CMD")

        self.swarm_type = None
        self.swarm_radius = 5000 #m
        self.swarm_radius_latlon = None

        self.last_swarm_vehicle_sent = time.time()
        self.last_swarm_vehicle_aircraft_id = None
        self.last_swarm_vehicle_lat = None
        self.last_swarm_vehicle_lon = None
        self.last_swarm_vehicle_altMSL = None
        self.last_swarm_vehicle_lat_tgt = 0 #TODO make an actual value
        self.last_swarm_vehicle_lon_tgt = 0 #TODO make an actual value
        self.last_swarm_vehcile_alt_tgt = 0 #TODO make an actual value

    def cmd_swarm(self, args):
        if len(args) < 1:
            self.print_usage()
            return
        elif args[0] == "draw":
            self.cmd_draw()
        elif args[0] == "radius":
            if self.mpstate.click_location is not None:
                self.swarm_radius_latlon = self.mpstate.click_location
            else:
                print("No map click position available for swarm radius")
                return

            if len(args) >= 2:
                #self.swarm_radius_altMSL = float(args[2])
                self.cmd_radius(args.pop())
            else:
                print("Swarm radius must be supplied")

    def cmd_radius(self, radius):
        if len(radius) < 2:
            print("Must suppply a radius")
            
        if radius.isnumeric():
            self.swarm_radius = float(radius)
            print(self.swarm_radius) 
        else:
            print("Radius must be a number")
            return

        self.master.mav.command_long_send(self.settings.target_system,
                    self.settings.target_component,
                    mavutil.mavlink.MAV_CMD_SWARM_RADIUS,
                    0, self.swarm_radius, 0, 0, 0, 
                    self.swarm_radius_latlon[0], self.swarm_radius_latlon[1], 0,)

    def cmd_draw(self):
        self.swarm_type = self.mav_param.get('SWRM_TYPE',0)
        #print(self.swarm_type)
        if self.swarm_type == 1:
            return #TODO draw radius

#****************************************************************************
#   Method Name     : __decode_swarm_radius
#   Description     : Decode and process the MAV_CMD_SWARM_RADIUS command
#   Parameters      : CommandLong Message
#   Return Value    : None
#   Author          : Michael Day
#****************************************************************************
    def __decode_swarm_radius(self, mCommand_Long):
        print(mCommand_Long.param1)
        return

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == "COMMAND_LONG":
            if m.command == mavutil.mavlink.MAV_CMD_SWARM_RADIUS:
                print ("Got MAV_CMD_SWARM_RADIUS")
                self.__decode_swarm_radius(m)
        elif mtype == "SWARM_VEHICLE":
            pass
        elif mtype == "GLOBAL_POSITION_INT":
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                #self.packets_mytarget += 1
                (self.last_swarm_vehicle_lat, self.last_swarm_vehicle_lon, self.last_swarm_vehicle_altMSL) = (m.lat, m.lon, m.alt)

                self.send_swarm_vehicle()
        elif mtype == "SWARM_COMMLINK_STATUS":
            pass
            #TODO: process swarm_commlink_status

            #else:
            #    self.packets_othertarget += 1

    def idle_task(self):
        pass

    def send_swarm_vehicle(self):
        if time.time() > self.last_swarm_vehicle_sent + 5:
            if self.last_swarm_vehicle_aircraft_id is None:
                self.last_swarm_vehicle_aircraft_id= int(self.mav_param.get('SYSID_THISMAV',0))
            
            #TODO GPS timestamp
            self.master.mav.swarm_vehicle_send(0, #GPS_timestamp goes here
                self.last_swarm_vehicle_aircraft_id, 0, 
                0, #state_nav
                0, #speed
                0, #course over ground
                0, #effective radius
                self.last_swarm_vehicle_lat, self.last_swarm_vehicle_lon, self.last_swarm_vehicle_altMSL,
                self.last_swarm_vehicle_lat_tgt, self.last_swarm_vehicle_lon_tgt,
                self.last_swarm_vehcile_alt_tgt)

            self.last_swarm_vehicle_sent = time.time()
        
    def print_usage(self):
        print("Usage: swarm <draw|radius>")           
            
def init(mpstate):
    '''initialise module'''
    return SwarmModule(mpstate)
