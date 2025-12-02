#!/usr/bin/env python3
'''
support for a GCS attached DGPS system
'''

import socket, errno
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_settings import MPSetting

class DGPSModule(mp_module.MPModule):

    @staticmethod
    def default_settings():
        return mp_settings.MPSettings([
                MPSetting("portnum", int, 13320),
                MPSetting("ip", str, "127.0.0.1"),
            ])

    def __init__(self, mpstate):
        super(DGPSModule, self).__init__(mpstate, "DGPS", "DGPS injection support for SBP/RTCP/UBC")
        self.dgps_settings = DGPSModule.default_settings()
        self.inject_seq_nr = 0
        self.cmdname = "dgps"
        
        self.create_port()
        self.add_command(
            self.cmdname,
            self.cmd_dgps,
            f"{self.cmdname} control",
            [
                "set (DGPS_SETTING)",
            ],
        )
        self.add_completion_function(
            "(DGPS_SETTING)", self.dgps_settings.completion
        )
        self.dgps_settings.set_callback(self.on_setting_set)

    def create_port(self):
        print(f"DGPS: Listening for RTCM packets on UDP://{self.dgps_settings.get('ip')}:{self.dgps_settings.get('portnum')}")
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind((self.dgps_settings.get('ip'), self.dgps_settings.get("portnum")))
        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)

    def cmd_dgps(self, args):
        """
        dgps commands
        """
        usage = f"usage: {self.cmdname} <set>"
        if len(args) < 1:
            print(usage)
        elif args[0] == "set":
            self.cmd_set(args[1:])
        else:
            print(usage)

    def cmd_set(self, args):
        """
        Modify a setting
        """
        self.dgps_settings.command(args)
    
    def send_rtcm_msg(self, data):

        if self.master is None:
            return

        msglen = 180
        
        if (len(data) > msglen * 4):
            print("DGPS: Message too large", len(data))
            return
        
        # How many messages will we send?
        msgs = 0
        if (len(data) % msglen == 0):
            msgs = len(data) // msglen
        else:
            msgs = (len(data) // msglen) + 1

        for a in range(0, msgs):
            
            flags = 0
            
            # Set the fragment flag if we're sending more than 1 packet.
            if (msgs) > 1:
                flags = 1
            
            # Set the ID of this fragment
            flags |= (a & 0x3) << 1
            
            # Set an overall sequence number
            flags |= (self.inject_seq_nr & 0x1f) << 3
            
            
            amount = min(len(data) - a * msglen, msglen)
            datachunk = data[a*msglen : a*msglen + amount]

            self.master.mav.gps_rtcm_data_send(
                flags,
                len(datachunk),
                bytearray(datachunk.ljust(180, b'\0')))
        
        # Send a terminal 0-length message if we sent 2 or 3 exactly-full messages.     
        if (msgs < 4) and (len(data) % msglen == 0) and (len(data) > msglen):
            flags = 1 | (msgs & 0x3)  << 1 | (self.inject_seq_nr & 0x1f) << 3
            self.master.mav.gps_rtcm_data_send(
                flags,
                0,
                bytearray("".ljust(180, '\0')))
            
        self.inject_seq_nr += 1

    def on_setting_set(self, setting):
        print("Settings changed:", setting.name)
        if setting.name == "portnum" or setting.name == "ip":
            if self.port:
                self.port.close()
            self.create_port()

    def idle_task(self):
        '''called in idle time'''
        try:
            data = self.port.recv(1024) # Attempt to read up to 1024 bytes.
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise
        try:
            self.send_rtcm_msg(data)

        except Exception as e:
            print("DGPS: GPS Inject Failed:", e)

def init(mpstate):
    '''initialise module'''
    return DGPSModule(mpstate)

