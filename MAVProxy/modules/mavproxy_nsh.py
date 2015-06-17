#!/usr/bin/env python
'''remote nsh console handling'''

import time, os, fnmatch, sys, time
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_module

class NSHModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NSHModule, self).__init__(mpstate, "nsh", "remote nsh shell")
        self.add_command('nsh', self.cmd_nsh,
                         'nsh shell control',
                         ['<start|stop>'])
        self.last_packet = time.time()
        self.last_check = time.time()
        self.started = False

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'SERIAL_CONTROL':
            data = m.data[:m.count]
            if m.count > 0:
                s = ''.join(str(chr(x)) for x in data)
                if self.mpstate.system == 'Windows':
                    # strip nsh ansi codes
                    s = s.replace("\033[K","")
                sys.stdout.write(s)
                self.last_packet = time.time()

    def stop(self):
        '''stop nsh input'''
        self.mpstate.rl.set_prompt(self.status.flightmode + "> ")
        self.mpstate.functions.input_handler = None
        self.started = False

    def send(self, line):
        '''send some bytes'''
        line = line.strip()
        if line == ".":
            self.stop()
            return
        mav = self.master.mav
        line += "\r\n"
        buf = [ord(x) for x in line]
        buf.extend([0]*(70-len(buf)))
        mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI,
                                0, 0,
                                len(line), buf)

    def idle_task(self):
        '''handle mavlink packets'''
        if not self.started:
            return
        now = time.time()
        if now - self.last_packet < 1:
            timeout = 0.05
        else:
            timeout = 0.2
        if now - self.last_check > timeout:
            self.last_check = now
            mav = self.master.mav
            mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                                    mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI,
                                    0, 0,
                                    0, [0]*70)
            
            
    def cmd_nsh(self, args):
        '''nsh shell commands'''
        usage = "Usage: nsh <start|stop>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "start":
            self.mpstate.functions.input_handler = self.send
            self.started = True
            self.mpstate.rl.set_prompt("")
        elif args[0] == "stop":
            self.stop()
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return NSHModule(mpstate)
