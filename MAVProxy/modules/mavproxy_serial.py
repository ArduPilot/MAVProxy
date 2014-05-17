#!/usr/bin/env python
'''serial_control MAVLink handling'''

import time, os, fnmatch, sys
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_module

class SerialModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SerialModule, self).__init__(mpstate, "serial", "serial control handling")
        self.add_command('serial', self.cmd_serial,
                         'remote serial control',
                         ['<lock|unlock|send>',
                          'set (SERIALSETTING)'])
        self.serial_settings = mp_settings.MPSettings(
            [ ('port', int, 0),
              ('baudrate', int, 57600),
              ('timeout', int, 500)
              ]
            )
        self.add_completion_function('(SERIALSETTING)', self.serial_settings.completion)
        self.locked = False

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'SERIAL_CONTROL':
            data = m.data[:m.count]
            s = ''.join(str(chr(x)) for x in data)
            sys.stdout.write(s)

    def serial_lock(self, lock):
        '''lock or unlock the port'''
        mav = self.master.mav
        if lock:
            flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
            self.locked = True
        else:
            flags = 0
            self.locked = False
        mav.serial_control_send(self.serial_settings.port,
                                flags,
                                0, 0, 0, [0]*70)

    def serial_send(self, args):
        '''send some bytes'''
        mav = self.master.mav
        flags = 0
        if self.locked:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        if self.serial_settings.timeout != 0:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
        if self.serial_settings.timeout >= 500:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI

        s = ' '.join(args)
        s = s.replace('\\r', '\r')
        s = s.replace('\\n', '\n')
        buf = [ord(x) for x in s]
        buf.extend([0]*(70-len(buf)))
        mav.serial_control_send(self.serial_settings.port,
                                flags,
                                self.serial_settings.timeout,
                                self.serial_settings.baudrate,
                                len(s), buf)

    def cmd_serial(self, args):
        '''serial control commands'''
        usage = "Usage: serial <lock|unlock|set|send>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "lock":
            self.serial_lock(True)
        elif args[0] == "unlock":
            self.serial_lock(False)
        elif args[0] == "set":
            self.serial_settings.command(args[1:])
        elif args[0] == "send":
            self.serial_send(args[1:])
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return SerialModule(mpstate)
