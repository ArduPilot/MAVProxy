"""
send NTRIP data to flight controller
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import ntrip
from MAVProxy.modules.lib import mp_settings


class NtripModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(NtripModule, self).__init__(mpstate, "ntrip", "ntrip", public=False)
        self.ntrip_settings = mp_settings.MPSettings(
            [('caster', str, None),
             ('port', int, 2101),
             ('username', str, 'IBS'),
             ('password', str, 'IBS'),
             ('mountpoint', str, None)])
        self.add_command('ntrip', self.cmd_ntrip, 'NTRIP control',
                         ["<status>",
                          "<start>",
                          "<stop>",
                          "set (NTRIPSETTING)"])
        self.add_completion_function('(NTRIPSETTING)',
                                     self.ntrip_settings.completion)
        self.pos = None
        self.pkt_count = 0
        self.last_pkt = None
        self.ntrip = None
        self.start_pending = False

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if msg.get_type() in ['GPS_RAW_INT', 'GPS2_RAW']:
            if msg.fix_type >= 3:
                self.pos = (msg.lat*1.0e-7, msg.lon*1.0e-7, msg.alt*1.0e-3)

    def idle_task(self):
        '''called on idle'''
        if self.start_pending and self.ntrip is None and self.pos is not None:
            self.cmd_start()
        if self.ntrip is None:
            return
        data = self.ntrip.read()
        if data is None:
            return
        blen = len(data)
        send_data = bytearray(data[:blen])
        if blen < 110:
            send_data.extend(bytearray([0]*(110-blen)))
        self.master.mav.gps_inject_data_send(0, 0, blen, send_data)
        self.pkt_count += 1
        self.last_pkt = time.time()

    def cmd_ntrip(self, args):
        '''ntrip command handling'''
        if len(args) <= 0:
            print("Usage: ntrip <start|stop|status|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.ntrip = None
            self.start_pending = False
        elif args[0] == "status":
            self.ntrip_status()
        elif args[0] == "set":
            self.ntrip_settings.command(args[1:])

    def ntrip_status(self):
        '''show ntrip status'''
        now = time.time()
        if self.ntrip is None:
            print("ntrip: Not started")
        elif self.last_pkt is None:
            print("ntrip: no data")
        else:
            print("ntrip: %u packets, last %.1fs ago" % (self.pkt_count, now - self.last_pkt))

    def cmd_start(self):
        '''start ntrip link'''
        if self.ntrip_settings.caster is None:
            print("Require caster")
            return
        if self.ntrip_settings.mountpoint is None:
            print("Require mountpoint")
            return
        if self.pos is None:
            print("Start delayed pending position")
            self.start_pending = True
            return
        user = self.ntrip_settings.username + ":" + self.ntrip_settings.password
        self.ntrip = ntrip.NtripClient(user=user,
                                       port=self.ntrip_settings.port,
                                       caster=self.ntrip_settings.caster,
                                       mountpoint=self.ntrip_settings.mountpoint,
                                       lat=self.pos[0],
                                       lon=self.pos[1],
                                       height=self.pos[2])
        print("NTRIP started")
        self.start_pending = False
            

def init(mpstate):
    '''initialise module'''
    return NtripModule(mpstate)
