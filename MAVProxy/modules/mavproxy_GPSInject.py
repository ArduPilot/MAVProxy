"""
inject data to GPS modules from a file or URL
primarily used to inject uBlox AssistNow data
"""

import random
import time
import os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
import urllib.request

OFFLINE_MBX = "https://firmware.ardupilot.org/AssistNow/OFFLINE.UBX"

class GPSInjectModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(GPSInjectModule, self).__init__(mpstate, "gpsinject", "gpsinject", public=False)
        self.gpsinject_settings = mp_settings.MPSettings(
            [('source', str, OFFLINE_MBX),
             ('send_rate_kps', float, 2.0),
             ('repeat', int, 2),
             ('gps_mask', int, 0)])
        self.add_command('gpsinject', self.cmd_gpsinject, 'GPSInject control',
                         ["<status|start|stop>",
                          "set (GPSINJECTSETTING)"])
        self.add_completion_function('(GPSINJECTSETTING)',
                                     self.gpsinject_settings.completion)
        self.buf = None
        self.sent_bytes = 0
        self.sent_count = 0
        self.started = False
        self.start_pending = False
        self.last_send = None

    def idle_task(self):
        '''called on idle'''
        if not self.started and not self.start_pending:
            return
        if self.buf is None:
            source = self.gpsinject_settings.source
            if source.startswith("http"):
                req = urllib.request.urlopen(self.gpsinject_settings.source)
                if req is not None:
                    self.buf = req.read()
            elif os.path.isfile(self.gpsinject_settings.source):
                self.buf = open(self.gpsinject_settings.source,'rb').read()
            if self.buf is None:
                print("GPSInject: Bad source %s" % source)
                self.started = False
                self.start_pending = False
                return
            print("GPSInject: retrieved %u bytes" % len(self.buf))

        if self.start_pending:
            GPS_RAW_INT = self.master.messages.get("GPS_RAW_INT", None)
            GPS2_RAW = self.master.messages.get("GPS2_RAW", None)
            if GPS_RAW_INT is None and GPS2_RAW is None:
                return
            have_gps = False
            have_gps = (GPS_RAW_INT is not None and GPS_RAW_INT.fix_type >= 1) or (GPS2_RAW is not None and GPS2_RAW.fix_type >= 1)
            if have_gps:
                self.started = True

        if not self.started:
            return

        max_send = 110
        now = time.time()
        sec_per_byte = 1.0/(1024.0*self.gpsinject_settings.send_rate_kps)
        if self.last_send is not None and now - self.last_send < max_send*sec_per_byte:
            return
        if self.last_send is None:
            cansend = max_send
        else:
            cansend = int((now - self.last_send) / sec_per_byte)
        self.last_send = now

        while cansend > 0:
            n = min(max_send, len(self.buf) - self.sent_bytes)
            n = min(n, cansend)
            msg = self.buf[self.sent_bytes:self.sent_bytes+n]
            self.master.mav.gps_inject_data_send(
                self.target_system,
                self.target_component,
                len(msg),
                bytearray(msg.ljust(max_send, bytes([0]))))
            self.sent_bytes += n
            if self.sent_bytes == len(self.buf):
                self.sent_bytes = 0
                self.sent_count += 1
                if self.sent_count == self.gpsinject_settings.repeat:
                    print("GPSInject: done")
                    self.started = False
                    self.start_pending = False
                    break
            cansend -= n

    def cmd_gpsinject(self, args):
        '''GPSInject command handling'''
        if len(args) <= 0:
            print("Usage: gpsinject <start|stop|status|set>")
            return
        if args[0] == "start":
            self.buf = None
            self.sent_bytes = 0
            self.sent_count = 0
            self.started = False
            self.start_pending = True
            self.last_send = None
        if args[0] == "stop":
            self.start_pending = False
            self.started = False
        elif args[0] == "status":
            self.gpsinject_status()
        elif args[0] == "set":
            self.gpsinject_settings.command(args[1:])

    def gpsinject_status(self):
        '''show GPS inject status'''
        now = time.time()
        if not self.started:
            print("GPSInject: Not started")
            return
        if self.buf is None:
            print("GPSInject: download pending")
            return
        if self.start_pending:
            print("GPSInject: start pending GPS")
            return
        print("GPSInject: sent %u/%u bytes, repeat=%u" % (self.sent_bytes, len(self.buf), self.sent_count))


def init(mpstate):
    '''initialise module'''
    return GPSInjectModule(mpstate)
