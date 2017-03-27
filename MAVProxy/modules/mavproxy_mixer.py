#!/usr/bin/env python
'''mixer command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from ctypes import c_int, c_uint, c_float

from MAVProxy.modules.lib import mp_module
from samba.netcmd.gpo import cmd_list

mixer_type_parameter_counts = [0,0,5,4,1,5,4]
    

class MixerState:
    MIXER_STATE_WAITING = 0
    MIXER_STATE_MIXER_GET_PARAMETER = 1
    MIXER_STATE_MIXER_SET_PARAMETER = 2
    MIXER_STATE_MIXER_GET_ALL = 3
    MIXER_STATE_MIXER_GET_MISSING = 4
    MIXER_STATE_MIXER_SAVE = 100
    
    '''this class is separated to make it possible to use the mixer
       functions on a secondary connection'''
    def __init__(self, mav_param, vehicle_name):
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.vehicle_name = vehicle_name
        self.state = self.MIXER_STATE_WAITING
        self.timeout = 1.0;
        self.get_retry_max = 10
        
        self._get_retries = 0
        self.mixer_data = []
        self._last_time = 0
        self._get_group = 0

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'MIXER_PARAMETER':
            self.handle_mixer_parameter(master,m)
    
    def compare_mixer_data_index(self, msg1, msg2):
        if(msg1.index == msg2.index):
            return True
        return False

    def handle_mixer_parameter(self, master, m):
        #Either replace or add to data
        print("Got mixer parameter index:%u mixer:%u sub:%u value:%.2f" % (m.index, m.mixer_index, m.mixer_sub_index, m.param_values[0]) )
        found = False
        index = 0
        for data in self.mixer_data:
            if self.compare_mixer_data_index(data, m):
                self.mixer_data[index] = m
                found = True
            index = index + 1
        if not found:
            self.mixer_data.append(m)

        self._last_time = time.time();
        
        if((self.state != self.MIXER_STATE_MIXER_GET_ALL) and (self.state != self.MIXER_STATE_MIXER_GET_MISSING)):
            self.state = self.MIXER_STATE_WAITING    
        elif(self.state == self.MIXER_STATE_MIXER_GET_MISSING):
            self.get_missing(master)
        return True

    def mixer_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return


    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: mixer <count|all|missing|save|sub|type|get|set|conn>"
        if len(args) < 1:
            print(usage)
            return
        elif args[0] == "all":
            if len(args) == 2:
                self.cmd_all(args, master)
            else:
                print(usage)
        elif args[0] == "missing":
            if len(args) == 2:
                self.cmd_missing(args, master)
            else:
                print(usage)
        elif args[0] == "save":
            if len(args) == 2:
                self.cmd_save(args, master)
            else:
                print(usage)
        elif args[0] == "get":
            if len(args) == 3:
                self.cmd_get(args, master)
            else:
                print(usage)
        elif args[0] == "set":
            if len(args) >= 4:
                self.cmd_set(args, master)
            else:
                print(usage)
        else:
            print(usage)


    def cmd_all(self, args, master):
        '''get all data for mixers in a group'''
        self._get_group = int(args[1])
        self.state = self.MIXER_STATE_MIXER_GET_ALL
        self._last_time = time.time();
        self.get_all(master, self._get_group)
        print("Requested all parameters for group %s" % (args[1]))                

    def get_all(self, master, group):
        mav = master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_PARAM_LIST, 0, 
                                           group, 0, 0, 0, 0, 0, 0)

    def cmd_get(self, args, master):
        '''get a mixer or submixer parameter in a group'''
        self.request_parameter(master, int(args[1]), int(args[2]) )        
        print("Requested parameter for group:%s index:%s" % (args[1],args[2]) )

    def request_parameter(self, master, group, index):
        mav = master
        self._get_group = group
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_PARAM_READ, 0, 
                           self._get_group, index, 0, 0, 0, 0, 0)
        self.state = self.MIXER_STATE_MIXER_GET_PARAMETER
        self._last_time = time.time();
        

    def cmd_set(self, args, master):
        '''get a mixer or submixer parameter in a group'''
        mav = master

        group = int(args[1])
        parameter = int(args[2])
        value = float(args[3])
        
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                   mavutil.mavlink.MAV_CMD_SET_MIXER_PARAMETER, 0, 
                   group, parameter, 0, 0, 0, 0, value)
        
        self.state = self.MIXER_STATE_MIXER_SET_PARAMETER
        self._last_time = time.time();
        print("Requested parameter set group:%s index:%s value:%s" % (args[1],args[2],args[3]))

    def check(self, master):
        if(self.state != self.MIXER_STATE_WAITING):
            if time.time() > (self._last_time + self.timeout):
                print("Mixer data request timeout")
                if(self.state == self.MIXER_STATE_MIXER_GET_ALL):
                    print("")
                    print("Collected mixer data messages")
                    print("")
                    for data in self.mixer_data:
                        print(data)
                    self.state = self.MIXER_STATE_WAITING;
                elif self.state == self.MIXER_STATE_MIXER_GET_MISSING:
                    self._get_retries = self._get_retries + 1
                    if(self._get_retries > self.get_retry_max):
                        print("Maximum retries exceeded while getting missing data")
                        self.state = self.MIXER_STATE_WAITING;
                    else:
                        print("Retry get missing data")
                        self.get_missing(master)
                else:
                    print("Timeout waiting for response to mixer data request")
                    self.state = self.MIXER_STATE_WAITING;

    def cmd_save(self, args, master):
        mav = master
        self._get_group = int(args[1])
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                   mavutil.mavlink.MAV_CMD_REQUEST_MIXER_STORE, 0, 
                   self._get_group, 0, 0, 0, 0, 0, 0)
        self.state = self.MIXER_STATE_WAITING
        self._last_time = time.time();

    def cmd_missing(self, args, master):
        print("Get missing mixer data for group %s" % (args[1]))
        self._get_retries = 0
        self._get_group = int(args[1])
        self.get_missing(master)

    def get_missing(self, master, group):
        print("Get missing mixer data for group %u" % group)
        self._get_retries = 0
        self._get_group = int(args[1])
        self.get_missing(master)                        

    def get_missing(self, master):
        print("TODO: get_missing not working yet")
        #Check for missing mixer count
#         if(count(self.mixer_data) == 0):
#           self.
# 
#         for data in self.mixer_data:
#           
        self.state = self.MIXER_STATE_WAITING
        print("Completed collecting missing mixer data")
                        
                                

class MixerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MixerModule, self).__init__(mpstate, "mixer", "mixer handling", public = True)
        self.pstate = MixerState(self.mav_param, self.vehicle_name)
        self.add_command('mixer', self.cmd_mixer, "mixer handling",
                         ["<all> (GROUP)",
                          "<missing> (GROUP)",
                          "<save> (GROUP)",
                          "<get> (GROUP) (PARAM_INDEX)",
                          "<set> (GROUP) (PARAM_INDEX) (VALUE)"])
#         if self.continue_mode and self.logdir != None:
#             parmfile = os.path.join(self.logdir, 'mav.parm')
#                 self.pstate.mav_param_set = set(self.mav_param.keys())

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        self.pstate.handle_mavlink_packet(self.master, m)

    def idle_task(self):
        '''handle missing parameters'''
        self.pstate.vehicle_name = self.vehicle_name
        self.pstate.check(self.master)

    def cmd_mixer(self, args):
        '''control parameters'''
        self.pstate.handle_command(self.master, self.mpstate, args)


def init(mpstate):
    '''initialise module'''
    return MixerModule(mpstate)
