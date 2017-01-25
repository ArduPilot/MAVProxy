#!/usr/bin/env python
'''mixer command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.lib import mp_module
from samba.netcmd.gpo import cmd_list

class MixerState:
    '''this class is separated to make it possible to use the parameter
       functions on a secondary connection'''
    def __init__(self, mav_param, vehicle_name):
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.vehicle_name = vehicle_name

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'MIXER_DATA':
            param_id = "%.16s" % m.param_id
            # Note: the xml specifies param_index is a uint16, so -1 in that field will show as 65535
            # We accept both -1 and 65535 as 'unknown index' to future proof us against someday having that
            # xml fixed.
            print("Received mixer data")

                        

    def mixer_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return


    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: mixer <list|params|set>"
        if len(args) < 1:
            print(usage)
            return
        print("we are getting to where we want to be and the first argument is")
        print args[0]
        if args[0] == "list":
            print("we almost got to where we want to be")
            if len(args) == 2:
                print("we got to where we want to be")
                self.cmd_list(args, master)
            else:
                print("near miss - not enough arguments")
                print(usage)
        else:
            print(usage)
        print("arguments")
        print(args)

        
    def cmd_list(self, args, master):
        '''do a full 3D accel calibration'''
        mav = master
        mav.mav.mixer_data_request_send(mav.target_system, mav.target_component,
                                        mavutil.mavlink.MIXER_GROUP_LOCAL, 0, 0, 0, mavutil.mavlink.MIXER_DATA_TYPE_MIXER_COUNT)
        print("Requested mixer list")
        

class MixerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MixerModule, self).__init__(mpstate, "mixer", "mixer handling", public = True)
        self.pstate = MixerState(self.mav_param, self.vehicle_name)
        self.add_command('mixer', self.cmd_mixer, "mixer handling",
                         ["<>",
                          "<list> (GROUP)",
                          "<params> (GROUP) (MIXER) (SUBMIXER)",
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
