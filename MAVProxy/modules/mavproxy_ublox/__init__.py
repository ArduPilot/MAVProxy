#!/usr/bin/env python
'''
UBX UBLOX Module
Peter Barker, April 2017

This module allows one to interact with the UBlox GPS's "Multi-GNSS-Assist" ("MGA") functionality.

To use the uBlox API you will need to create a file containing your API token in ~/.mavproxy/ublox/api_token
'''

import os
import struct
import time

import ublox as ub

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class ublox(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(ublox, self).__init__(mpstate, "ublox", "")
        self.auto = True
        self.verbose = False
        self.api_token = self.read_api_token()

        self.auto_failed = None

        self.last_auto = 0
        self.fix_type = 0
        self.time_boot_ms = 0
        self.mga_offline_last_check = 0
        self.mga_offline_data_uploaded = None
        self.state_dir = mp_util.dot_mavproxy('ublox')
        self.mga_cachedir = os.path.join(self.state_dir, "mga")
        try:
            os.makedirs(self.mga_cachedir)
        except OSError as e:
            if e.errno != 17:
                raise e
        self.mga_dbd_cachefile = os.path.join(self.mga_cachedir, "dbd.ubx")
        self.mga_offline = ub.mga.MGAOfflineCache(cachefile=os.path.join(self.mga_cachedir, "offline.ubx"), token=self.api_token)
        if self.mga_offline.should_request_fresh_data():
            print("Apparently should request fresh data")
#        self.mga_offline.start_update_thread()
        self.ublox_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('auto', bool, False),
          ])
        self.add_command('ublox', self.cmd_ublox, "ublox module", ['status','set (AUTO)', 'reset', 'mga'])

    def token_file(self):
        return os.path.join(mp_util.dot_mavproxy('ublox'), "api_token")

    # read API token from separate file in dot-mavproxy; you probably
    # want this file non-world-readable, as opposed to .mavinit.scr
    def read_api_token(self):
        fh = open(self.token_file(), "r")
        content = fh.read()
        fh.close()
        content = content.rstrip()
        return content

    def usage(self):
        '''show help on command line options'''
        return "Usage: ublox <status|mga|reset>"

    def cmd_ublox(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.cmd_status())
        elif args[0] == "set":
            self.ublox_settings.command(args[1:])
        elif args[0] == "reset":
            self.cmd_ublox_reset(args[1:])
        elif args[0] == "mga":
            self.cmd_ublox_mga(args[1:])
        else:
            print(self.usage())

    def cmd_status(self):
        '''returns information about module'''
        return("status called" % ())

    def cmd_ublox_reset(self, args):
        '''attempts to cold-reboot (factory-reboot) gps module'''
        print("Sending uBlox reset")
        msg = struct.pack("<HBB", 0xFFFF, 0x0, 0)
        self.master.mav.gps_inject_data_send(
            self.target_system,
            self.target_component,
            len(msg),
            bytearray(msg.ljust(110, '\0')))

    def cmd_ublox_mga(self, args):
        '''returns information about module'''
        print("ublox mga called" % ())

    def idle_upload_mga_dbd(self):
        # still need to answer the question of downloading the MGA
        # database from the ublox.  Do we ask for ask for ArduPilot
        # for a raw passthrough of the UBlox stream?
#        age = self.dbd_cache_age
#        if age > date.date(1,0,0): # ignore data older than one day
#            return False
#        print("Would upload data")
        return False

    def idle_upload_mga_offline(self):
        now = time.time()
        if now - self.mga_offline_last_check < 5:
            return
        self.mga_offline_last_check = now
        date = self.mga_offline.get_data_date_closest_to_now()
        if date is None:
            print("No offline data")
            return False
        if (self.mga_offline_data_uploaded is not None and
            self.mga_offline_data_uploaded == date):
            print("Already uploaded data for (%s)" % str(date))
            return
        # send all data past his point; the driver in ArduPilot will filter
        # to send just the closest to the GPS
        data = self.mga_offline.messages_for_date(date)
        print("Uploading MGA-Offline for %s" % str(data))
 #       print("Uploadable data: %s" % str(data))
        if len(data) == 0:
            return False
        for msg in data:
            raw = msg.raw()
            if len(raw) > 110:
                print("offline data message too long")
                continue
#            print("Sending message (%s)" % (str(msg)))
            self.master.mav.gps_inject_data_send(
                self.target_system,
                self.target_component,
                len(raw),
                bytearray(raw.ljust(110, '\0')))
            self.mga_offline_data_uploaded = date
        return True

    def idle_upload_mga_online(self):
        return False

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if self.fix_type >= 3:
            # don't do anything if we have a fix
            return
        if self.mpstate.status.armed:
            # do not upload data over telemetry link if we are armed
            # perhaps we should just limit rate instead.
            return
        now = time.time()
        if self.auto:
            if now-self.last_auto > 2:
                self.last_auto = now
                print("Doing auto things")
                if self.idle_upload_mga_dbd():
                    self.auto_status = "DBD uploaded"

                if not self.idle_upload_mga_online():
                    self.auto_status = "Online uploaded"

                if not self.idle_upload_mga_offline():
                    self.auto_status = "Offline uploaded"


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        # can't use status.msgs as we need this filtered to our target system
        if (self.settings.target_system != 0 and
            self.settings.target_system != m.get_srcSystem()):
            return
        if m.get_type() == 'GPS_RAW_INT':
            self.fix_type = m.fix_type
        if m.get_type() == 'SYSTEM_TIME':
            self.time_boot_ms = m.time_boot_ms

    def unload(self):
        print("Unload called")
        self.mga_offline.stop_update_thread()

def init(mpstate):
    '''initialise module'''
    return ublox(mpstate)
