#!/usr/bin/env python
'''enable run-time addition and removal of UDP clients , just like --out on the cnd line'''
''' TO USE: 
    module load output     ( once ) 
    output 10.11.12.13 14550  ( as many times as you like) 
    module unload output   ( once, cleans up all the outputs you dynamically created ) 
'''    

import time, math
from pymavlink import mavutil


from MAVProxy.modules.lib import mp_module

class OutputModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OutputModule, self).__init__(mpstate, "output", "output UDP")
        self.add_command('output', self.cmd_output, "output change")
        '''the places this module is currently outputting to..'''
        self.current = []  
        print("Module output loaded")


    def cmd_output(self, args):
        if len(args) != 2:
            print('You must pass exactly 2 args: ip_address and udp_port. eg: "output 10.11.12.13 14550" ')
            return

        ip = args[0]
        udpport = args[1]
        baud = 57600  # irrelevant for udp streams, whatever.
        port = ip+":"+udpport
        self.current.append(port)
        print('connecting UDP stream to ip_address:'+ip+" with udp_port:"+udpport)
        new = mavutil.mavlink_connection(port, baud=int(baud), input=False)
        self.mpstate.mav_outputs.append(new)

         
    def unknown_command(self, args):
        '''handle mode switch by mode name as command'''
        return False

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        master = self.master
        
    def unload(self):
        '''unload module'''
        master = self.master
        # for each of the ones loaded dynamically, we unload them all here. 
        # presumes no other module ever unloads or loads outputsdynamically...
        for d in self.current:
            self.mpstate.mav_outputs.pop() # remove the most recently loaded one, TODO choose the matching one
            self.current.pop() 
            


def init(mpstate):
    '''initialise module'''
    return OutputModule(mpstate)
