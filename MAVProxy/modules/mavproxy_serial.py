#!/usr/bin/env python
'''serial_control MAVLink handling'''

from __future__ import print_function

import hexdump
import os
import select
import socket
import sys
import time
from cStringIO import StringIO

from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_module

class SerialModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SerialModule, self).__init__(mpstate, "serial",
                                           "serial control handling")

        self.locked = False

        self.init_commands()
        self.init_settings()
        self.init_proxy()

    def init_proxy(self):
        self.proxy_active = False
        self.proxy_client = None

    def init_commands(self):
            self.add_command('serial', self.cmd_serial,
                             'remote serial control',
                             ['<lock|unlock|send>',
                              'proxy <start|stop|status>',
                              'set (SERIALSETTING)'])

    def init_settings(self):
            self.serial_settings = mp_settings.MPSettings(
                [
                    ('port', int, 0),
                    ('speed', int, 57600),
                    ('timeout', int, 500),
                    ('proxyaddr', str, '127.0.0.1'),
                    ('proxyport', int, 2167),
                    ('proxydebug', bool, False),
                ]
            )
            self.add_completion_function('(SERIALSETTING)',
                                         self.serial_settings.completion)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'SERIAL_CONTROL':
            data = bytearray(m.data[:m.count])
            s = bytes(data)

            if self.serial_settings.proxydebug:
                print('SER->PIX', repr(s))

            if self.proxy_active and self.proxy_client is not None:
                    self.proxy_buffer.append(s)
                    self.proxy_poll.register(
                        self.proxy_client, select.POLLIN|select.POLLOUT)
            else:
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

    def check_proxy_active(self):
        if self.proxy_active:
            print('Serial proxy is active.')

        return self.proxy_active

    def cmd_serial_lock(self, lock):
        if self.check_proxy_active():
            return

        return self.serial_lock(lock)

    def serial_send(self, data):
        mav = self.master.mav
        flags = 0
        if self.locked:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        if self.serial_settings.timeout != 0:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
        if self.serial_settings.timeout >= 500:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI

        if self.serial_settings.proxydebug:
            print('PIX->SER', repr(data))
        buf = bytearray(data)
        buf.extend([0]*(70-len(buf)))
        mav.serial_control_send(self.serial_settings.port,
                                flags,
                                self.serial_settings.timeout,
                                self.serial_settings.speed,
                                len(data), buf)

    def cmd_serial_send(self, args):
        '''send some bytes'''
        if self.check_proxy_active():
            return

        s = ' '.join(args)
        s = s.replace('\\r', '\r')
        s = s.replace('\\n', '\n')
        return self.serial_send(s)

    def cmd_serial(self, args):
        '''serial control commands'''
        usage = "Usage: serial <lock|unlock|set|send>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "lock":
            self.cmd_serial_lock(True)
        elif args[0] == "unlock":
            self.cmd_serial_lock(False)
        elif args[0] == "set":
            self.serial_settings.command(args[1:])
        elif args[0] == "send":
            self.cmd_serial_send(args[1:])
        elif args[0] == "proxy":
            self.cmd_proxy(args[1:])
        else:
            print(usage)

    def cmd_proxy(self, args):
        if args[0] == "start":
            self.proxy_start()
        elif args[0] == "stop":
            self.proxy_stop()
        elif args[0] == "status":
            self.proxy_status()

    def proxy_start(self):
        if self.proxy_active:
            print('Serial proxy is already active')
            return

        print('Starting proxy on {}:{}'.format(
            self.serial_settings.proxyaddr,
            self.serial_settings.proxyport))

        self.serial_lock(True)
        self.proxy_active = True
        self.proxy_buffer = []
        self.proxy_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.proxy_sock.setsockopt(socket.SOL_SOCKET,
                                   socket.SO_REUSEADDR, 1)
        self.proxy_sock.bind((
            self.serial_settings.proxyaddr,
            self.serial_settings.proxyport
        ))
        self.proxy_sock.listen(1)
        self.proxy_client = None

        self.proxy_poll = select.poll()
        self.proxy_poll.register(self.proxy_sock, select.POLLIN)

    def proxy_stop(self):
        if not self.proxy_active:
            print('Serial proxy is not active')
            return

        self.proxy_active = False
        if self.proxy_client is not None:
            self.proxy_client.close()
            self.proxy_client = None

        self.proxy_poll = None
        self.proxy_sock.close()
        self.proxy_buffer = None
        self.serial_lock(False)

    def proxy_status(self):
        if not self.proxy_active:
            print("Serial proxy is not active")
            return

        print("Serial proxy is active on {}:{}".format(
            self.serial_settings.proxyaddr,
            self.serial_settings.proxyport))

    def disconnect_client(self, fd):
        print('Client disconnected')
        self.proxy_poll.unregister(fd)
        os.close(fd)
        self.proxy_client = None
        self.buffer = []

    def idle_task(self):
        if not self.proxy_active:
            return

        ready = self.proxy_poll.poll(0)
        if not ready:
            return

        for fd, event in ready:
            if fd == self.proxy_sock.fileno():
                clsock, claddr = self.proxy_sock.accept()

                if not self.proxy_client:
                    print('New connection from', claddr)
                    self.proxy_client = clsock
                    self.proxy_poll.register(clsock, select.POLLIN)
                else:
                    print('Refusing connection from', claddr)
                    clsock.close()
            else:
                if event & select.POLLOUT and self.proxy_buffer:
                    buf = ''.join(self.proxy_buffer)
                    nb = self.proxy_client.send(buf)
                    buf = buf[nb:]

                    if buf:
                        # if there is still data left to send
                        self.proxy_buffer = [buf]
                    else:
                        # no data left to send, so we can drop POLLOUT
                        self.proxy_poll.register(fd, select.POLLIN)

                if event & select.POLLIN:
                    data = os.read(fd, 1024)
                    if not data:
                        self.disconnect_client(fd)
                    if self.serial_settings.proxydebug:
                        print('NET->PIX', repr(data))
                    self.serial_send(data)

                if event & select.POLLHUP:
                    self.disconnect_client(fd)


def init(mpstate):
    '''initialise module'''
    return SerialModule(mpstate)
