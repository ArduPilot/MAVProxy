#!/usr/bin/env python
'''mode command handling'''

import time, os, platform
from pymavlink import mavutil, mavwp

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

#TODO: Make Swarming happen in a different thread
class SwarmModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SwarmModule, self).__init__(mpstate, "swarm control", "swarm mangagement", public = True)
        #self.swarmloader_by_sysid = {}

        #TODO: add clear, move. Also Finish radius
        self.add_command('swarm', self.cmd_swarm, "swarm control", ["<list>","<load|save> (FILENAME)"]) 
        #self.add_command('swarm_radius', self.__decode_swarm_radius, "Decode Swarm Radius MAV_CMD")

        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('SwarmROI', items=[MPMenuItem('Clear', 'Clear', '# swarmROI clear')])
            #TODO: add logic for clear
        #                           items=[MPMenuItem('Clear', 'Clear', '# rally clear'),
        #                                  MPMenuItem('List', 'List', '# rally list'),
        #                                  MPMenuItem('Load', 'Load', '# rally load ',
        #                                             handler=MPMenuCallFileDialog(flags=('open',),
        #                                                                          title='Rally Load',
        #                                                                          wildcard='RallyPoints(*.txt,*.rally,*.ral)|*.txt;*.rally;*.ral')),
        #                                  MPMenuItem('Save', 'Save', '# rally save ',
        #                                             handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
        #                                                                          title='Rally Save',
        #                                                                          wildcard='RallyPoints(*.txt,*.rally,*.ral)|*.txt;*.rally;*.ral')),
        #                                  MPMenuItem('Add', 'Add', '# rally add ',
        #                                             handler=MPMenuCallTextDialog(title='Rally Altitude (m)',
        #                                                                          default=100))])

        #TODO: add continue_mode from fence
        #print("Swarm continue-mode: ", self.continue_mode)
        #print("Swarm logdir: ", self.logdir)
        self.have_swarm_roi = False
        #if self.continue_mode and self.logdir is not None:
        #     fencetxt = os.path.join(self.logdir, 'fence.txt')
        #     if os.path.exists(fencetxt):
        #         self.fenceloader.load(fencetxt)
        #         self.have_swarm_roi = True
        #         print("Loaded fence from %s" % fencetxt)

        self.menu_added_console = False
        self.menu_added_map = False

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

        # self.swarmloader_by_sysid[self.target_system] = self.swarmloader()

        self.init_swarmloader()
        
    def init_swarmloader(self):
        self.swarmloader = mavwp.MAVSwarmLoader()
        self.swarmloader.target_system = 0
        self.swarmloader.target_component = 0

    # @property
    # def swarmloader(self):
    #     '''swarm loader by sysid'''
    #     print("Before if")
    #     if not self.target_system in self.swarmloader_by_sysid:
    #         print("After if")
    #         self.swarmloader_by_sysid[self.target_system] = mavwp.MAVSwarmLoader()
    #     return self.swarmloader_by_sysid[self.target_system]

    def cmd_swarm(self, args):
        if len(args) < 1:
            self.print_usage()
            return  

        elif args[0] == "list":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return
                
            self.cmd_list()

        elif args[0] == "list":
            self.cmd_list()

        elif args[0] == "load":
            if len(args) < 2:
                print("Usage: swarm load <filename>")
                return
            self.cmd_load([args.pop()])

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

        elif args[0] == "save":
            if len(args) < 2:
                print("Usage: swarm save <filename>")
                return
            self.cmd_save([args.pop()])

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

    def cmd_load(self, args):
        #TODO: wait until ardu params loaded before calling cmd_load

        if (len(args) < 1):
            print("Usage: swarm load filename")
            return

        try:
            self.swarmloader.load(args[0].strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (args[1], msg))
            return

        self.mpstate.functions.param_set('SWRM_POLY_PTS',self.swarmloader.count(), retries=3)

        print("Loaded %u swarm ROI points from %s" % (self.swarmloader.count(), args[0]))

    def cmd_save(self, args):
        if (len(args) < 1):
            print("Usage: rally save filename")
            return

        self.swarmloader.save(args[0].strip('"'))

        print("Saved swarm ROI file %s" % args[0])

    def cmd_list(self):
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        self.swarm_type = self.mav_param.get('SWRM_TYPE',0)

        if self.swarm_type == 1: #polygons
            swarm_count = self.mav_param.get('SWRM_POLY_PTS',0)

            points = self.swarmloader.polygon()
            if points:
                pass
            else:
                print("No points in Swarm ROI, please load a .swrm or similar file")
                return

            if len(points) < 3:
                print("Swarm ROI must contain at least 3 points, (preferably a convex polygon)")
                return

            self.swarmloader.target_system = self.target_system
            self.swarmloader.target_component = self.target_component

            self.send_swarm_roi()
            self.have_swarm_roi = True

            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('SwarmROIPoly'))
            self.mpstate.map.add_object(mp_slipmap.SlipPolygon('SwarmROIPoly', points, # append first element to close polygon
                                                          layer='swarmROIpoly', linewidth=2, colour=(0,255,255)))

            if self.logdir is not None:
                fname = 'swarm.txt'
                if self.target_system > 1:
                    fname = 'swarm_%u.txt' % self.target_system
                swarm_file_path = os.path.join(self.logdir, fname)
                self.swarmloader.save(swarm_file_path)
                print("Saved swarm ROI points to %s" % swarm_file_path)       

        #elif radius
            #TODO: list radius

        else:
            #TODO: don't print this mesage if params haven't loaded yet
            #AND print a message stating that they haven't
            print("Can't list: SWRM_TYPE ardupilot parameter is 0")
    
    def send_swarm_roi(self):
        '''send swarm ROI from swarmloader'''
        # # must disable geo-fencing when loading
        self.swarmloader.target_system = self.target_system
        self.swarmloader.target_component = self.target_component
        self.swarmloader.reindex()
        #action = self.get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
        # self.param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE, 3)
        self.param_set('SWRM_POLY_PTS', self.swarmloader.count(), 3)

        for i in range(self.swarmloader.count()):
            p = self.swarmloader.point(i)
            self.master.mav.send(p)

        return True

    def fetch_swarm_point(self ,i):
        '''fetch one swarm point'''
        self.master.mav.swarm_fetch_point_send(self.target_system,
                                            self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 3:
            p = self.master.recv_match(type='FENCE_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch point %u" % i)
            return None
        return p

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
        '''called on idle'''
        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

    def send_swarm_vehicle(self):
        if time.time() > self.last_swarm_vehicle_sent + 5:
            if self.last_swarm_vehicle_aircraft_id is None:
                self.last_swarm_vehicle_aircraft_id= int(self.mav_param.get('SYSID_THISMAV',0))
            
            #TODO GPS timestamp
            self.master.mav.swarm_vehicle_send(0, #GPS_timestamp goes here
                self.last_swarm_vehicle_aircraft_id, 
                0, #squadron
                0, #state_nav
                0, #speed
                0, #course over ground
                0, #effective radius
                self.last_swarm_vehicle_lat, self.last_swarm_vehicle_lon, self.last_swarm_vehicle_altMSL,
                self.last_swarm_vehicle_lat_tgt, self.last_swarm_vehicle_lon_tgt,
                self.last_swarm_vehcile_alt_tgt,
                0,) #swarm_roi

            self.last_swarm_vehicle_sent = time.time()
        
    def print_usage(self):
        print("Usage: swarm <list> | <load|save> (FILENAME)")       
            
def init(mpstate):
    '''initialise module'''
    return SwarmModule(mpstate)
