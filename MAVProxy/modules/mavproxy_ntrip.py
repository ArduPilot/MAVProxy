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
             ('mountpoint', str, None),
             ('logfile', str, None),
             ('sendalllinks', bool, False),
             ('sendmul', int, 1)])
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
        self.last_restart = None
        self.last_rate = None
        self.rate_total = 0
        self.ntrip = None
        self.start_pending = False
        self.rate = 0
        self.logfile = None
        self.id_counts = {}
        self.last_by_id = {}

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if msg.get_type() in ['GPS_RAW_INT', 'GPS2_RAW']:
            if msg.fix_type >= 3:
                self.pos = (msg.lat*1.0e-7, msg.lon*1.0e-7, msg.alt*1.0e-3)

    def log_rtcm(self, data):
        '''optionally log rtcm data'''
        if self.ntrip_settings.logfile is None:
            return
        if self.logfile is None:
            self.logfile = open(self.ntrip_settings.logfile, 'wb')
        if self.logfile is not None:
            self.logfile.write(data)

    def idle_task(self):
        '''called on idle'''
        if self.start_pending and self.ntrip is None and self.pos is not None:
            self.cmd_start()
        if self.ntrip is None:
            return
        data = self.ntrip.read()
        if data is None:
            now = time.time()
            if (self.last_pkt is not None and
                now - self.last_pkt > 15 and
                (self.last_restart is None or now - self.last_restart > 30)):
                print("NTRIP restart")
                self.ntrip = None
                self.start_pending = True
                self.last_restart = now
            return
        if time.time() - self.ntrip.dt_last_gga_sent > 2:
            self.ntrip.setPosition(self.pos[0], self.pos[1])
            self.ntrip.send_gga()
        self.log_rtcm(data)

        rtcm_id = self.ntrip.get_ID()
        if not rtcm_id in self.id_counts:
            self.id_counts[rtcm_id] = 0
            self.last_by_id[rtcm_id] = data[:]
        self.id_counts[rtcm_id] += 1

        blen = len(data)
        if blen > 4*180:
            # can't send this with GPS_RTCM_DATA
            return
        total_len = blen
        self.rate_total += blen * self.ntrip_settings.sendmul

        if blen > 180:
            flags = 1 # fragmented
        else:
            flags = 0
        # add in the sequence number
        flags |= (self.pkt_count & 0x1F) << 3

        fragment = 0
        while blen > 0:
            send_data = bytearray(data[:180])
            frag_len = len(send_data)
            data = data[frag_len:]
            if frag_len < 180:
                send_data.extend(bytearray([0]*(180-frag_len)))
            if self.ntrip_settings.sendalllinks:
                links = self.mpstate.mav_master
            else:
                links = [self.master]
            for link in links:
                for d in range(self.ntrip_settings.sendmul):
                    link.mav.gps_rtcm_data_send(flags | (fragment<<1), frag_len, send_data)
            fragment += 1
            blen -= frag_len
        self.pkt_count += 1

        now = time.time()
        if now - self.last_rate > 1:
            dt = now - self.last_rate
            rate_now = self.rate_total / float(dt)
            self.rate = 0.9 * self.rate + 0.1 * rate_now
            self.last_rate = now
            self.rate_total = 0
        self.last_pkt = now

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
            return
        elif self.last_pkt is None:
            print("ntrip: no data")
            return
        frame_size = 0
        for id in sorted(self.id_counts.keys()):
            print(" %4u: %u (len %u)" % (id, self.id_counts[id], len(self.last_by_id[id])))
            frame_size += len(self.last_by_id[id])
        print("ntrip: %u packets, %.1f bytes/sec last %.1fs ago framesize %u" % (self.pkt_count, self.rate, now - self.last_pkt, frame_size))

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
        self.last_rate = time.time()
        self.rate_total = 0


def init(mpstate):
    '''initialise module'''
    return NtripModule(mpstate)
