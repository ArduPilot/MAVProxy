#!/usr/bin/env python
'''
control MAVLink2 signing
'''    

from pymavlink import mavutil
import time, struct, math, sys

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class SigningModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SigningModule, self).__init__(mpstate, "signing", "signing control", public=True)
        self.add_command('signing', self.cmd_signing, "signing control",
                         ["<setup|remove|disable|key>"])
        self.allow = None

    def cmd_signing(self, args):
        '''handle link commands'''
        usage = "signing: <setup|remove|disable|key> passphrase"
        if len(args) == 0:
            print(usage)
        elif args[0] == 'setup':
            self.cmd_signing_setup(args[1:])
        elif args[0] == 'key':
            self.cmd_signing_key(args[1:])
        elif args[0] == 'disable':
            self.cmd_signing_disable(args[1:])
        elif args[0] == 'remove':
            self.cmd_signing_remove(args[1:])
        else:
            print(usage)

    def passphrase_to_key(self, passphrase):
        '''convert a passphrase to a 32 byte key'''
        import hashlib
        h = hashlib.new('sha256')
        h.update(passphrase)
        return h.digest()

    def cmd_signing_setup(self, args):
        '''setup signing key on board'''
        if len(args) == 0:
            print("usage: signing setup passphrase")
            return
        if not self.master.mavlink20():
            print("You must be using MAVLink2 for signing")
            return
        passphrase = args[0]
        key = self.passphrase_to_key(passphrase)
        secret_key = []
        for b in key:
            secret_key.append(ord(b))

        epoch_offset = 1420070400
        now = max(time.time(), epoch_offset)
        initial_timestamp = int((now - epoch_offset)*1e5)
        self.master.mav.setup_signing_send(self.target_system, self.target_component,
                                           secret_key, initial_timestamp)
        print("Sent secret_key")
        self.cmd_signing_key([passphrase])


    def allow_unsigned(self, mav, msgId):
        '''see if an unsigned packet should be allowed'''
        if self.allow is None:
            self.allow = {
                mavutil.mavlink.MAVLINK_MSG_ID_RADIO : True,
                mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS : True 
                }
        if msgId in self.allow:
            return True
        if self.settings.allow_unsigned:
            return True
        return False

    def cmd_signing_key(self, args):
        '''set signing key on connection'''
        if len(args) == 0:
            print("usage: signing setup passphrase")
            return
        if not self.master.mavlink20():
            print("You must be using MAVLink2 for signing")
            return
        passphrase = args[0]
        key = self.passphrase_to_key(passphrase)
        self.master.setup_signing(key, sign_outgoing=True, allow_unsigned_callback=self.allow_unsigned)
        print("Setup signing key")

    def cmd_signing_disable(self, args):
        '''disable signing locally'''
        self.master.disable_signing()
        print("Disabled signing")

    def cmd_signing_remove(self, args):
        '''remove signing from server'''
        if not self.master.mavlink20():
            print("You must be using MAVLink2 for signing")
            return
        self.master.mav.setup_signing_send(self.target_system, self.target_component, [0]*32, 0)
        self.master.disable_signing()
        print("Removed signing")
        
def init(mpstate):
    '''initialise module'''
    return SigningModule(mpstate)
