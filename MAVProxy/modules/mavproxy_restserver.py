#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Server Module
Patrick Jose Pereira
April 2018
'''

import time
import json
import socket
from threading import Thread

from flask import Flask
from werkzeug.serving import make_server
from MAVProxy.modules.lib import mp_module

def mavlink_to_json(msg):
    '''Translate mavlink python messages in json string'''
    ret = '\"%s\": {' % msg._type
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret += '\"%s\" : \"%s\", ' % (fieldname, data)
    ret = ret[0:-2] + '}'
    return ret

def mpstatus_to_json(status):
    '''Translate MPStatus in json string'''
    msg_keys = list(status.msgs.keys())
    data = '{'
    for key in msg_keys[:-1]:
        data += mavlink_to_json(status.msgs[key]) + ','
    data += mavlink_to_json(status.msgs[msg_keys[-1]])
    data += '}'
    return data

class RestServer():
    '''Rest Server'''
    def __init__(self):
        # Set log level and remove flask output
        import logging
        self.log = logging.getLogger('werkzeug')
        self.log.setLevel(logging.ERROR)

        # Server variables
        self.app = None
        self.run_thread = None
        self.address = 'localhost'
        self.port = 5000

        # Save status
        self.status = None
        self.server = None

    def update_dict(self, mpstate):
        '''We don't have time to waste'''
        self.status = mpstate.status

    def set_ip_port(self, ip, port):
        '''set ip and port'''
        self.address = ip
        self.port = port
        self.stop()
        self.start()

    def start(self):
        '''Stop server'''
        # Set flask
        self.app = Flask('RestServer')
        self.add_endpoint()
        # Create a thread to deal with flask
        self.run_thread = Thread(target=self.run)
        self.run_thread.start()

    def running(self):
        '''If app is valid, thread and server are running'''
        return self.app != None

    def stop(self):
        '''Stop server'''
        self.app = None
        if self.run_thread:
            self.run_thread = None
        if self.server:
            self.server.shutdown()
            self.server = None

    def run(self):
        '''Start app'''
        self.server = make_server(self.address, self.port, self.app, threaded=True)
        self.server.serve_forever()

    def request(self, arg=None):
        '''Deal with requests'''
        if not self.status:
            return '{"result": "No message"}'

        try:
            status_dict = json.loads(mpstatus_to_json(self.status))
        except Exception as e:
            print(e)
            return

        # If no key, send the entire json
        if not arg:
            return json.dumps(status_dict)

        # Get item from path
        new_dict = status_dict
        args = arg.split('/')
        for key in args:
            if key in new_dict:
                new_dict = new_dict[key]
            else:
                return '{"key": "%s", "last_dict": %s}' % (key, json.dumps(new_dict))

        return json.dumps(new_dict)

    def add_endpoint(self):
        '''Set endpoits'''
        self.app.add_url_rule('/rest/mavlink/<path:arg>', 'rest', self.request)
        self.app.add_url_rule('/rest/mavlink/', 'rest', self.request)

class ServerModule(mp_module.MPModule):
    ''' Server Module '''
    def __init__(self, mpstate):
        super(ServerModule, self).__init__(mpstate, "restserver", "restserver module")
        # Configure server
        self.rest_server = RestServer()

        self.add_command('restserver', self.cmds, \
            "restserver module", ['start', 'stop', 'address 127.0.0.1:4777'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: restserver <address|freq|stop|start>"

    def cmds(self, args):
        '''control behaviour of the module'''
        if not args or len(args) < 1:
            print(self.usage())
            return

        if args[0] == "start":
            if self.rest_server.running():
                print("Rest server already running.")
                return
            self.rest_server.start()
            print("Rest server running: %s:%s" % \
                (self.rest_server.address, self.rest_server.port))

        elif args[0] == "stop":
            if not self.rest_server.running():
                print("Rest server is not running.")
                return
            self.rest_server.stop()

        elif args[0] == "address":
            # Check if have necessary amount of arguments
            if len(args) != 2:
                print("usage: restserver address <ip:port>")
                return

            address = args[1].split(':')
            # Check if argument is correct
            if len(address) == 2:
                self.rest_server.set_ip_port(address[0], int(address[1]))
                return

        else:
            print(self.usage())

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # Update server with last mpstate
        self.rest_server.update_dict(self.mpstate)

    def unload(self):
        '''Stop and kill everything before finishing'''
        self.rest_server.stop()
        pass

def init(mpstate):
    '''initialise module'''
    return ServerModule(mpstate)
