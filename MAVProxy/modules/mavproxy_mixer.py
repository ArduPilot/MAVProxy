#!/usr/bin/env python
'''mixer command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from ctypes import c_int, c_uint, c_float

from MAVProxy.modules.lib import mp_module
from samba.netcmd.gpo import cmd_list

class MixerState:
    MIXER_STATE_WAITING = 0
    MIXER_STATE_LIST_MIXER_COUNT = 1
    MIXER_STATE_LIST_SUB_COUNT = 2
    MIXER_STATE_LIST_MIXER_TYPE = 3
    MIXER_STATE_MIXER_COUNT = 4
    MIXER_STATE_SUBMIXER_COUNT = 5
    MIXER_STATE_MIXER_TYPE = 6
    MIXER_STATE_MIXER_GET_PARAMETER = 7
    
    '''this class is separated to make it possible to use the parameter
       functions on a secondary connection'''
    def __init__(self, mav_param, vehicle_name):
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.vehicle_name = vehicle_name
        self.state = self.MIXER_STATE_WAITING;
        self.mixer_group = mavutil.mavlink.MIXER_GROUP_LOCAL
        self.mixer_index = 0
        self.mixer_sub_index = 0
        self.param_index = 0
        
        self.mixer_count = 0;
        self.sub_mixer_count = []

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'MIXER_DATA':
            print("Received mixer data of type:%u" % (m.data_type))
            self.handle_mixer_data(master,m)

    def handle_mixer_data(self, master, m):
        if(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT):
            self.mixer_count = m.param_type
            print("Group:%u mixer:%u mixer count:%u" % (m.mixer_group, m.param_type))
            self.state = self.MIXER_STATE_WAITING

        if(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT):
            print("Group:%u mixer:%u submixer count:%u" % (m.mixer_group, m.mixer_index, m.param_type))

        if(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE):
            print("Group:%u mixer:%u submixer:%u type:%u" % (m.mixer_group, m.mixer_index,m.mixer_sub_index, m.param_type))

        if(m.data_type == mavutil.mavlink.MIXER_DATA_TYPE_PARAMETER):
            print("Group:%u mixer:%u submixer:%u index:%u value:%.4f" % (m.mixer_group, m.mixer_index,m.mixer_sub_index, m.mixer_index, m.param_value))


    def mixer_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return


    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: mixer <count|sub|type|get|set>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "count":
            if len(args) == 2:
                self.cmd_count(args, master)
            else:
                print(usage)
        if args[0] == "sub":
            if len(args) == 3:
                self.cmd_sub(args, master)
            else:
                print(usage)
        if args[0] == "type":
            if len(args) == 4:
                self.cmd_type(args, master)
            else:
                print(usage)
        if args[0] == "get":
            if len(args) == 5:
                self.cmd_get(args, master)
            else:
                print(usage)
        else:
            print(usage)

        
    def cmd_list(self, args, master):
        '''do a full 3D accel calibration'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        mavutil.mavlink.MIXER_GROUP_LOCAL, 0, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT)
        self.state = self.MIXER_STATE_LIST_MIXER_COUNT
        print("Requested mixer list")

    def cmd_count(self, args, master):
        '''get a count of the mixers in a group'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        int(args[1]), 0, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT)
        self.state = self.MIXER_STATE_MIXER_COUNT
        print("Requested mixer count for group %s" % (args[1]))                

    def cmd_sub(self, args, master):
        '''get a count of the sub mixers in a mixer in a group'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        int(args[1]), int(args[2]), 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_SUBMIXER_COUNT)
        self.state = self.MIXER_STATE_SUBMIXER_COUNT
        print("Requested sub mixer count for group:%s mixer:%s" % (args[1], args[2])) 

    def cmd_type(self, args, master):
        '''get the mixer type of a mixer/submixer in a group'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        int(args[1]), int(args[2]), int(args[3]), 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXTYPE)
        self.state = self.MIXER_STATE_MIXER_TYPE
        print("Requested mixer type for group:%s mixer:%s submixer:%s" % (args[1],args[2],args[3]))

    def cmd_get(self, args, master):
        '''get a mixer or submixer parameter in a group'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        int(args[1]), int(args[2]), int(args[3]), int(args[4]), mavutil.mavlink.MIXER_DATA_TYPE_PARAMETER)
        self.state = self.MIXER_STATE_MIXER_GET_PARAMETER
        print("Requested parameter for group:%s mixer:%s submixer:%s index:%s" % (args[1],args[2],args[3],args[4]))



class MixerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MixerModule, self).__init__(mpstate, "mixer", "mixer handling", public = True)
        self.pstate = MixerState(self.mav_param, self.vehicle_name)
        self.add_command('mixer', self.cmd_mixer, "mixer handling",
                         ["<>",
                          "<count> (GROUP)",
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
#        self.pstate.fetch_check(self.master)

    def cmd_mixer(self, args):
        '''control parameters'''
        self.pstate.handle_command(self.master, self.mpstate, args)


def init(mpstate):
    '''initialise module'''
    return MixerModule(mpstate)
