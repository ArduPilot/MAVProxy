#!/usr/bin/env python
'''
Json Server Module
Patrick Jose Pereira
April 2018
'''

import os, time, sys
import socket
import threading

from MAVProxy.modules.lib import mp_module

class JsonServerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(JsonServerModule, self).__init__(mpstate, "jsonserver", "jsonserver module")
        # Configure socket
        self.socket_ip = "127.0.0.1"
        self.socket_port = 4777
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Configure task
        self.last_pack = time.time()
        self.pack_interval = 1

        self.add_command('jsonserver', self.cmds, "jsonserver module", ['address 127.0.0.1:4777','freq 1'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: jsonserver <address|freq>"

    def cmds(self, args):
        '''control behaviour of the module'''
        if not args:
            print(self.usage())
            return
        if args[0] == "address":
            if len(args) == 2:
                address = args[1].split(':')
                if len(address) == 2:
                    self.socket_ip = address[0]
                    self.socket_port = int(address[1])
                    return
            print("usage: jsonserver address <ip:port>")
        elif args[0] == "freq":
            if len(args) == 2:
                self.pack_interval = 1/float(args[1])
                return
            print("usage: jsonserver freq <hz>")
        else:
            print(self.usage())

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now - self.last_pack > self.pack_interval:
            msg_keys = self.mpstate.status.msgs.keys()
            data = '{'
            for m in msg_keys[:-1]:
                data += self.jsonIt(self.mpstate.status.msgs[m]) + ','
            data += self.jsonIt(self.mpstate.status.msgs[msg_keys[-1]])
            data += '}'
            self.send(data)
            self.last_pack = now

    def send(self, data):
        self.sock.sendto(data, (self.socket_ip, self.socket_port))

    def jsonIt(self, msg):
        ret = '\"%s\": {' % msg._type
        for a in msg._fieldnames:
            v = getattr(msg, a)
            ret += '\"%s\" : \"%s\", ' % (a, v)
        ret = ret[0:-2] + '}'
        return ret

def init(mpstate):
    '''initialise module'''
    return JsonServerModule(mpstate)
