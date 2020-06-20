#!/usr/bin/env python
"""
 NTRIP client module
 based on client from http://github.com/jcmb/NTRIP
"""

import socket
import sys
import datetime
import base64
import time
import errno
from MAVProxy.modules.lib import rtcm3
import ssl
from optparse import OptionParser

version = 0.1
useragent = "NTRIP MAVProxy/%.1f" % version


class NtripError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()

    def __str__(self):
        return self.message


class NtripClient(object):
    def __init__(self,
                 user="",
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 V2=False,
                 ):
        if sys.version_info.major >= 3:
            user = bytearray(user, 'ascii')
        self.user = base64.b64encode(user)
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        if not self.mountpoint.startswith("/"):
            self.mountpoint = "/" + self.mountpoint
        self.setPosition(lat, lon)
        self.height = height
        self.ssl = ssl
        self.host = host
        self.V2 = V2
        self.socket = None
        self.found_header = False
        self.sent_header = False
        # RTCM3 parser
        self.rtcm3 = rtcm3.RTCM3()
        self.last_id = None
        if self.port == 443:
            # force SSL on port 443
            self.ssl = True

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon-360)*-1
            self.flagE = "W"
        elif lon < 0 and lon >= -180:
            lon = lon*-1
            self.flagE = "W"
        elif lon < -180:
            lon = lon+360
            self.flagE = "E"
        else:
            self.lon = lon
        if lat < 0:
            lat = lat*-1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon-self.lonDeg)*60
        self.latMin = (lat-self.latDeg)*60

    def getMountPointString(self):
        userstr = self.user
        if sys.version_info.major >= 3:
            userstr = str(userstr, 'ascii')
        mountPointString = "GET %s HTTP/1.0\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
            self.mountpoint, useragent, userstr)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster, self.port)
            mountPointString += hostString
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        return mountPointString

    def getGGAString(self):
        now = datetime.datetime.utcnow()
        ggaString = "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % (
                     now.hour, now.minute, now.second, self.latDeg, self.latMin, self.flagN,
                     self.lonDeg, self.lonMin, self.flagE, self.height)
        checksum = self.calculateCheckSum(ggaString)
        return "$%s*%s\r\n" % (ggaString, checksum)

    def calculateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def get_ID(self):
        '''get ID of last packet'''
        return self.last_id

    def read(self):
        if self.socket is None:
            time.sleep(0.1)
            self.connect()
            return None

        if not self.found_header:
            if not self.sent_header:
                self.sent_header = True
                time.sleep(0.1)
                mps = self.getMountPointString()
                if sys.version_info.major >= 3:
                    mps = bytearray(mps, 'ascii')
                try:
                    self.socket.sendall(mps)
                except Exception:
                    self.socket = None
                    return None
            try:
                casterResponse = self.socket.recv(4096)
            except ssl.SSLWantReadError:
                    return None
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket = None
                casterResponse = ''
            if sys.version_info.major >= 3:
                casterResponse = str(casterResponse, 'ascii')
            header_lines = casterResponse.split("\r\n")
            for line in header_lines:
                if line == "":
                    self.found_header = True
                if line.find("SOURCETABLE") != -1:
                    raise NtripError("Mount point does not exist")
                elif line.find("401 Unauthorized") != -1:
                    raise NtripError("Unauthorized request")
                elif line.find("404 Not Found") != -1:
                    raise NtripError("Mount Point does not exist")
                elif line.find(" 200 OK") != -1:
                    # Request was valid
                    try:
                        gga = self.getGGAString()
                        if sys.version_info.major >= 3:
                            gga = bytearray(gga, 'ascii')
                        self.socket.sendall(gga)
                    except Exception:
                        self.socket = None
                        return None
            return None
        # normal data read
        while True:
            try:
                data = self.socket.recv(1)
            except ssl.SSLWantReadError:
                    return None
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket.close()
                self.socket = None
                return None
            except Exception:
                self.socket.close()
                self.socket = None
                return None
            if len(data) == 0:
                self.socket.close()
                self.socket = None
                return None
            if self.rtcm3.read(data):
                self.last_id = self.rtcm3.get_packet_ID()
                return self.rtcm3.get_packet()

    def connect(self):
        '''connect to NTRIP server'''
        self.sent_header = False
        self.found_header = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.ssl:
            sock = ssl.wrap_socket(sock)
        try:
            error_indicator = sock.connect_ex((self.caster, self.port))
        except Exception:
            return False
        if error_indicator == 0:
            sock.setblocking(0)
            self.socket = sock
            self.rtcm3.reset()
            return True
        return False

    def readLoop(self):
        while True:
            data = self.read()
            if data is None:
                continue
            print("got: ", len(data))

if __name__ == '__main__':
    usage = "NtripClient.py [options] [caster] [port] mountpoint"
    parser = OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS", help="The Ntripcaster username.  Default: %default")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS", help="The Ntripcaster password. Default: %default")
    parser.add_option("-o", "--org", type="string", dest="org", help="Use IBSS and the provided organization for the user. Caster and Port are not needed in this case Default: %default")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg", help="The org that the base is in. IBSS Only, assumed to be the user org")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09, help="Your latitude.  Default: %default")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66, help="Your longitude.  Default: %default")
    parser.add_option("-e", "--height", type="float", dest="height", default=1200, help="Your ellipsoid height.  Default: %default")
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False, help="Use SSL for the connection")
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False, help="Include host header, should be on for IBSS")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False, help="Make a NTRIP V2 Connection")

    (options, args) = parser.parse_args()
    ntripArgs = {}

    ntripArgs['lat'] = options.lat
    ntripArgs['lon'] = options.lon
    ntripArgs['height'] = options.height
    ntripArgs['host'] = options.host

    if options.ssl:
        import ssl
        ntripArgs['ssl'] = True
    else:
        ntripArgs['ssl'] = False

    if options.org:
        if len(args) != 1:
            print("Incorrect number of arguments for IBSS\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user+"." + options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster'] = options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster'] = options.org + ".ibss.trimbleos.com"
        if options.ssl:
            ntripArgs['port'] = 52101
        else:
            ntripArgs['port'] = 2101
        ntripArgs['mountpoint'] = args[0]
    else:
        if len(args) != 3:
            print("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user+":"+options.password
        ntripArgs['caster'] = args[0]
        ntripArgs['port'] = int(args[1])
        ntripArgs['mountpoint'] = args[2]

    ntripArgs['V2'] = options.V2

    n = NtripClient(**ntripArgs)
    n.readLoop()
