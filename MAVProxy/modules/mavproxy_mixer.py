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
    MIXER_STATE_LIST_MIXER_COUNT = 1
    MIXER_STATE_LIST_SUB_COUNT = 2
    MIXER_STATE_LIST_MIXER_TYPE = 3
    MIXER_STATE_MIXER_COUNT = 4
    MIXER_STATE_SUBMIXER_COUNT = 5
    MIXER_STATE_MIXER_TYPE = 6
    MIXER_STATE_MIXER_GET_PARAMETER = 7
    MIXER_STATE_MIXER_SET_PARAMETER = 8
    MIXER_STATE_MIXER_GET_TYPE_COUNT = 9
    MIXER_STATE_MIXER_GET_MISSING = 50
    MIXER_STATE_MIXER_GET_ALL = 100
    MIXER_STATE_MIXER_SAVE = 112
    
    '''this class is separated to make it possible to use the mixer
       functions on a secondary connection'''
    def __init__(self, mav_param, vehicle_name):
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.vehicle_name = vehicle_name
        self.state = self.MIXER_STATE_WAITING;
        self.timeout = 1.0;
        self.get_retry_max = 10
        
        self._get_retries = 0
        self.mixer_data = []
        self._last_time = 0
        self._get_group = 0

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'MIXER_DATA':
            self.handle_mixer_data(master,m)
    
    def compare_mixer_data_index(self, msg1, msg2):
        if( (msg1.mixer_group == msg2.mixer_group) and
            (msg1.mixer_index == msg2.mixer_index) and
            (msg1.mixer_sub_index == msg2.mixer_sub_index) and
            (msg1.parameter_index == msg2.parameter_index) and 
            (msg1.data_type == msg2.data_type) ):
            return True
        return False

    def handle_mixer_data(self, master, m):
        if(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT):
            print("Group:%u mixer count:%u" % (m.mixer_group, m.data_value))

        elif(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT):
            print("Group:%u mixer:%u submixer count:%u" % (m.mixer_group, m.mixer_index, m.data_value))

        elif(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE):
            print("Group:%u mixer:%u submixer:%u type:%u" % (m.mixer_group, m.mixer_index,m.mixer_sub_index, m.data_value))

        elif(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_PARAMETER):
            print("Group:%u mixer:%u submixer:%u index:%u value:%.4f" % (m.mixer_group, m.mixer_index,m.mixer_sub_index, m.parameter_index, m.param_value))

        elif(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXERTYPE_COUNT):
            print("Group:%u mixer type count:%u" % (m.mixer_group, m.data_value))

        elif(m.data_type == 112):
            if(m.data_value > 0):
                print("Saved mixer group:%u with size:%u" % (m.mixer_group, m.data_value))
            else:
                print("Save mixer group:%u failed" % m.mixer_group)
        else:
            print("Received unknown data type in MIXER_DATA message")
            return False

        #Either replace or add to data
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
        usage="Usage: mixer <count|all|missing|save|sub|type|get|set>"
        if len(args) < 1:
            print(usage)
            return
        elif args[0] == "all":
            if len(args) == 2:
                self.cmd_all(args, master)
            else:
                print(usage)
        elif args[0] == "count":
            if len(args) == 2:
                self.cmd_count(args, master)
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
        elif args[0] == "types":
            if len(args) == 2:
                self.cmd_types(args, master)
            else:
                print(usage)
        elif args[0] == "sub":
            if len(args) == 3:
                self.cmd_sub(args, master)
            else:
                print(usage)
        elif args[0] == "type":
            if len(args) == 4:
                self.cmd_type(args, master)
            else:
                print(usage)
        elif args[0] == "get":
            if len(args) == 5:
                self.cmd_get(args, master)
            else:
                print(usage)
        elif args[0] == "set":
            if len(args) == 6:
                self.cmd_set(args, master)
            else:
                print(usage)
        else:
            print(usage)


    def cmd_count(self, args, master):
        print("Requested mixer count for group %s" % (args[1]))
        self.request_count(master, int(args[1]))
    
    def request_count(self, master, group):
        '''get a count of the mixers in a group'''
        mav = master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_DATA, 0, 
                                           group, 0, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT, 0, 0)
        self._last_time = time.time();
        self.state = self.MIXER_STATE_MIXER_COUNT
              

    def cmd_all(self, args, master):
        '''get all data for mixers in a group'''
        self.mixer_data = []              
        mav = master
        self._get_group = int(args[1])
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_SEND_ALL, 0, 
                                           self._get_group, 0, 0, 0, 0, 0, 0)
        self.state = self.MIXER_STATE_MIXER_GET_ALL
        self._last_time = time.time();
        print("Requested all data for group %s" % (args[1]))                

    def cmd_types(self, args, master):
        '''get count of mixer types for a group'''
        self.mixer_data = []              
        mav = master
        self._get_group = int(args[1])
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_TYPE_COUNT, 0, 
                                           self._get_group, 0, 0, 0, 0, 0, 0)
        self.state = self.MIXER_STATE_MIXER_GET_TYPE_COUNT
        self._last_time = time.time();
        print("Requested count of mixer types for group %s" % (args[1]))


    def cmd_sub(self, args, master):
        '''get a count of the sub mixers in a mixer in a group'''
        self.request_sub(master, int(args[1]), int(args[2]))
        print("Requested sub mixer count for group:%s mixer:%s" % (args[1], args[2]))
        
    def request_sub(self, master, group, mixer):
        mav = master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                   mavutil.mavlink.MAV_CMD_REQUEST_MIXER_DATA, 0, 
                                   group, mixer, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT, 0, 0)
#         mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
#                                         group, mixer, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT)
        self._last_time = time.time();
        self.state = self.MIXER_STATE_SUBMIXER_COUNT
      

    def cmd_type(self, args, master):
        '''get the mixer type of a mixer/submixer in a group'''
        self.request_type(master, int(args[1]), int(args[2]), int(args[3]))
        print("Requested mixer type for group:%s mixer:%s submixer:%s" % (args[1],args[2],args[3]))
        
    def request_type(self, master, group, mixer, submixer):
        mav = master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_DATA, 0, 
                           group, mixer, submixer, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE, 0, 0)

#         mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
#                                         group, mixer, submixer, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE)
        self.state = self.MIXER_STATE_MIXER_TYPE
        self._last_time = time.time();
      

    def cmd_get(self, args, master):
        '''get a mixer or submixer parameter in a group'''
        self.request_parameter(master, int(args[1]), int(args[2]), int(args[3]), int(args[4]))
        print("Requested parameter for group:%s mixer:%s submixer:%s index:%s" % (args[1],args[2],args[3],args[4]))

    def request_parameter(self, master, group, mixer, submixer, parameter):
        mav = master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                           mavutil.mavlink.MAV_CMD_REQUEST_MIXER_DATA, 0, 
                           group, mixer, submixer, 0, mavutil.mavlink.MIXER_DATA_TYPE_PARAMETER, 0, 0)
        self.state = self.MIXER_STATE_MIXER_GET_PARAMETER
        self._last_time = time.time();
        

    def cmd_set(self, args, master):
        '''get a mixer or submixer parameter in a group'''
        mav = master

        group = int(args[1])
        mixer = int(args[2])
        submixer = int(args[3])
        parameter = int(args[4])
        value = float(args[5])
        
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                   mavutil.mavlink.MAV_CMD_SET_MIXER_PARAMETER, 0, 
                   group, mixer, submixer, parameter, value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 0)
        
        self.state = self.MIXER_STATE_MIXER_SET_PARAMETER
        self._last_time = time.time();
        print("Requested parameter for group:%s mixer:%s submixer:%s index:%s value:%s" % (args[1],args[2],args[3],args[4],args[5]))

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
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        self._get_group, 0, 0, 0, 112)
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
        #Check for missing mixer count
        mixer_count_msg = None
        for data in self.mixer_data:
            if (data.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT):
                mixer_count_msg = data
        if(mixer_count_msg is None):
            print("Get missing mixer count for group:%u" % self._get_group)
            self.request_count(master, self._get_group)
            self.state = self.MIXER_STATE_MIXER_GET_MISSING
            return
        mixer_count = mixer_count_msg.data_value
        #Check for missing mixer submixer counts and collect submixer counts
        submixer_counts = [mixer_count]
        for mixer in range(0, mixer_count):
            mixer_sub_msg = None
            for data in self.mixer_data:
                if (data.data_type == mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT):
                    if(data.mixer_index == mixer):
                        mixer_sub_msg = data
                        submixer_counts[mixer] = mixer_sub_msg.data_value   #Store the submixer count for later
            if(mixer_sub_msg is None):
                print("Get missing submixer count for group:%u mixer:%u" %(self._get_group, mixer))
                self.request_sub(master, self._get_group, mixer)
                self.state = self.MIXER_STATE_MIXER_GET_MISSING
                return
        #Check for missing mixer types        
        mixer_types = {}
        for mixer in range(0, mixer_count):
            for submixer in range(0, submixer_counts[mixer]+1):   #Add one for the root mixer
                mixer_type_msg = None
                for data in self.mixer_data:
                    if (data.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE):
                        if( (data.mixer_index == mixer) and (data.mixer_sub_index == submixer) ):
                            mixer_type_msg = data
                            mixer_types[mixer, submixer] = mixer_type_msg.data_value
                if(mixer_type_msg is None):
                    print("Get missing mixer type for group:%u mixer:%u submixer:%u" %(self._get_group, mixer, submixer))
                    self.request_type(master, self._get_group, mixer, submixer)
                    self.state = self.MIXER_STATE_MIXER_GET_MISSING
                    return
        #Check for missing parameter data
        self.state = self.MIXER_STATE_WAITING;
        for mixer in range(0, mixer_count):
            for submixer in range(0, submixer_counts[mixer]+1):   #Add one for the root mixer
                mixer_type = mixer_types[mixer, submixer]
                for parameter in range(0, mixer_type_parameter_counts[mixer_type]):
                    parameter_msg = None
                    for data in self.mixer_data:
                        if (data.data_type == mavutil.mavlink.MIXER_DATA_TYPE_PARAMETER):
                            if( (data.mixer_index == mixer) and (data.mixer_sub_index == submixer) and (data.parameter_index == parameter)):
                                parameter_msg = data
                    if(parameter_msg is None):
                        print("Get missing parameter value for group:%u mixer:%u submixer:%u parameter:%u" %(self._get_group, mixer, submixer, parameter))
                        self.request_parameter(master, self._get_group, mixer, submixer, parameter)
                        self.state = self.MIXER_STATE_MIXER_GET_MISSING
                        return
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
                          "<count> (GROUP)",
                          "<types> (GROUP)",
                          "<sub> (GROUP) (MIXER)",
                          "<type> (GROUP) (MIXER) (SUBMIXER)",
                          "<get> (GROUP) (MIXER) (SUBMIXER) (PARAM_INDEX)",
                          "<set> (GROUP) (MIXER) (SUBMIXER) (PARAM_INDEX) (VALUE)"])
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
