# thanks to
# https://gist.github.com/dankrause/6000248
#
#   Copyright 2014 Dan Krause
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import socket
import httplib
import StringIO

class SSDPResponse(object):
    class _FakeSocket(StringIO.StringIO):
        def makefile(self, *args, **kw):
            return self
    def __init__(self, response):
        r = httplib.HTTPResponse(self._FakeSocket(response))
        r.begin()
        self.location = r.getheader("location")
        self.usn = r.getheader("usn")
        self.st = r.getheader("st")
        self.cache = r.getheader("cache-control").split("=")[1]
    def __repr__(self):
        return "<SSDPResponse({location}, {st}, {usn})>".format(**self.__dict__)

def discover(service, if_ip=None, timeout=2, retries=1):
    group = ("239.255.255.250", 1900)
    message = "\r\n".join([
        'M-SEARCH * HTTP/1.1',
        'HOST: {0}:{1}',
        'MAN: "ssdp:discover"',
        'ST: {st}','MX: 3','',''])
    socket.setdefaulttimeout(timeout)
    responses = []
    for _ in range(retries):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        if if_ip is not None:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(if_ip))
        sock.sendto(message.format(*group, st=service), group)
        while True:
            try:
                response = SSDPResponse(sock.recv(1024))
                responses.append(response)
            except socket.timeout:
                break
    return responses

# Example:
# import ssdp
# ssdp.discover("roku:ecp")

