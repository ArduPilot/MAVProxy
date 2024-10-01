# use optitrack data to provide ATT_POS_MOCAP data
# it works with optitrack motion capture cameras and optitrack motive tracker software (https://optitrack.com/software/motive/)
# yuan-chu tai

import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_optitrack import NatNetClient

class optitrack(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(optitrack, self).__init__(mpstate, "optitrack", "optitrack")
        self.optitrack_settings = mp_settings.MPSettings(
            [('server', str, '127.0.0.1'),
            ('client', str, '127.0.0.1'),
            ('msg_intvl_ms', int, 75),
            ('obj_id', int, 1),
            ('print_lv', int, 0),
            ('multicast', bool, True)]
        )
        self.add_command('optitrack', self.cmd_optitrack, "optitrack control", ['<start>', '<stop>', 'set (OPTITRACKSETTING)'])
        self.streaming_client = NatNetClient.NatNetClient()
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streaming_client.rigid_body_listener = self.receive_rigid_body_frame
        self.last_msg_time = 0
        self.started = False

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receive_rigid_body_frame(self, new_id, position, rotation):
        if (new_id == self.optitrack_settings.obj_id):
            now = time.time()
            if (now - self.last_msg_time) > (self.optitrack_settings.msg_intvl_ms * 0.001):
                time_us = int(now * 1.0e6)
                self.master.mav.att_pos_mocap_send(time_us, (rotation[3], rotation[0], rotation[2], -rotation[1]), position[0], position[2], -position[1])
                self.last_msg_time = now

    def usage(self):
        '''show help on command line options'''
        return "Usage: optitrack <start|stop|set>"

    def cmd_start(self):
        self.streaming_client.set_client_address(self.optitrack_settings.client)
        self.streaming_client.set_server_address(self.optitrack_settings.server)
        self.streaming_client.set_print_level(self.optitrack_settings.print_lv)
        self.streaming_client.set_use_multicast(self.optitrack_settings.multicast)
        self.streaming_client.run()
        self.started = True

    def cmd_optitrack(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "start":
            self.cmd_start()
        elif args[0] == "stop":
            if self.started:
                self.started = False
                self.streaming_client.shutdown()
        elif args[0] == "set":
            self.optitrack_settings.command(args[1:])
        else:
            print(self.usage())

def init(mpstate):
    '''initialise module'''
    return optitrack(mpstate)
