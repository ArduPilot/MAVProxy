#!/usr/bin/env python
'''gopro control over mavlink for the solo-gimbal

To use this module connect to a Solo with a GoPro installed on the gimbal.
'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

class GoProModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(GoProModule, self).__init__(mpstate, "gopro", "gopro handling")

        self.add_command('gopro', self.cmd_gopro,   'gopro control', [
                                        'status',
                                        'shutter <start|stop>',
                                        'mode <video|camera>',
                                        'power <on|off>'])

    def cmd_gopro(self, args):
        '''gopro commands'''
        usage = "status, shutter <start|stop>, mode <video|camera>, power <on|off>"
        mav = self.master.mav

        if args[0] == "status":
            self.cmd_gopro_status(args[1:])
            return

        if args[0] == "shutter":
            name = args[1].lower()
            if name == 'start':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_SHUTTER, 1)
                return
            elif name == 'stop':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_SHUTTER, 0)
                return
            else:
                print("unrecognized")
                return

        if args[0] == "mode":
            name = args[1].lower()
            if name == 'video':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, 0)
                return
            elif name == 'camera':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, 1)
                return
            else:
                print("unrecognized")
                return

        if args[0] == "power":
            name = args[1].lower()
            if name == 'on':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_POWER, 1)
                return
            elif name == 'off':
                mav.gopro_set_request_send(0, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_POWER, 0)
                return
            else:
                print("unrecognized")
                return

        print(usage)

    def cmd_gopro_status(self, args):
        '''show gopro status'''
        master = self.master
        if 'GOPRO_HEARTBEAT' in master.messages:
            print(master.messages['GOPRO_HEARTBEAT'])
        else:
            print("No GOPRO_HEARTBEAT messages")

def init(mpstate):
    '''initialise module'''
    return GoProModule(mpstate)
