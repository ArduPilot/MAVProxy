#!/usr/bin/env python3
'''SITL RC output transports

AP_FLAKE8_CLEAN
'''

import socket

from pymavlink import mavutil


class UnixDatagramOutput(object):
    '''a reconnecting Unix domain datagram output'''

    def __init__(self, path):
        if not path:
            raise ValueError("Unix domain socket path must be specified")
        self.path = path
        self.port = None

    def _connect(self):
        port = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            port.connect(self.path)
        except OSError:
            port.close()
            return False
        self.port = port
        return True

    def close(self):
        if self.port is not None:
            self.port.close()
            self.port = None

    def write(self, buf):
        if self.port is None and not self._connect():
            return 0
        try:
            return self.port.send(buf)
        except OSError:
            self.close()
            return 0


def connection(device):
    '''open a UDP or Unix domain datagram SITL output'''
    if device.startswith("uds:"):
        return UnixDatagramOutput(device[4:])
    return mavutil.mavudp(device, input=False)
