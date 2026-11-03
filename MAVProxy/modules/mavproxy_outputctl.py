#!/usr/bin/env python3
'''
add and remove UDP outputs at runtime from a GCS, over a UDP control socket

The UDP equivalent of "output add" and "output remove", for a GCS which
has no MAVProxy console. Each request is one datagram of ASCII text,
answered with one datagram:

    ADD <ip> <port>       OK Added <ip>:<port>
    REMOVE <ip> <port>    OK Removed <ip>:<port>
    LIST                  OK List <n>, then a "<ip>:<port> active" line per output
    CLEAR                 OK Cleared <n> outputs

or "ERR <reason>". Only outputs added this way can be removed this way:
the master links, the --out outputs and those added with "output add"
are never touched. See docs/outputctl.md for the full protocol.

TO USE:
    module load outputctl                 # listen on 127.0.0.1:14600
    outputctl set bind 192.168.2.10       # listen on this address instead
    outputctl set port 14601              # or this port
    outputctl set allow 192.168.2.117     # senders allowed: IPs or networks, comma separated
    outputctl status
    outputctl list
    outputctl clear

AP_FLAKE8_CLEAN
'''

import ipaddress
import re
import select
import socket
import threading
import time
import traceback

from pymavlink import mavutil

from MAVProxy.modules import mavproxy_output
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib.mp_settings import MPSetting

# longest request we look at; real ones are under 40 bytes
MAX_REQUEST_LEN = 256
# requests answered per main loop iteration, so a flood cannot hold up telemetry
MAX_REQUESTS_PER_POLL = 32
# seconds between attempts to open the control socket after a failure
OPEN_RETRY_PERIOD = 5
# seconds between reports of requests from senders which are not allowed
REJECT_REPORT_PERIOD = 5.0
# successful requests of these types are not reported on the console
QUIET_COMMANDS = frozenset(['LIST'])

LIMITED_BROADCAST = ipaddress.IPv4Address('255.255.255.255')


class RequestError(Exception):
    '''a request we answer with ERR'''


def parse_ip(text):
    '''the destination address in a request, which must be a unicast IPv4
    address in canonical dotted quad form'''
    try:
        addr = ipaddress.IPv4Address(text)
    except ValueError:
        raise RequestError("Invalid IP address: %s" % text)
    if str(addr) != text:
        # eg "192.168.002.1", which some parsers read as octal
        raise RequestError("Invalid IP address: %s" % text)
    if addr.is_unspecified or addr.is_multicast or addr == LIMITED_BROADCAST:
        raise RequestError("Invalid IP address: %s (not a unicast address)" % text)
    return text


def parse_port(text):
    '''the destination port in a request'''
    if re.fullmatch('[0-9]{1,5}', text) is None or not 1 <= int(text) <= 65535:
        raise RequestError("Port must be 1-65535")
    return int(text)


def parse_allow(text):
    '''the networks in the allow setting, a comma separated list of IPv4
    addresses and networks such as "127.0.0.1,192.168.2.0/24". Raises
    ValueError if any is invalid'''
    nets = []
    for item in text.split(','):
        item = item.strip()
        if item:
            nets.append(ipaddress.IPv4Network(item, strict=False))
    return nets


def decode_request(data):
    '''the words of a request datagram'''
    if len(data) > MAX_REQUEST_LEN:
        raise RequestError("Request too long (max %u bytes)" % MAX_REQUEST_LEN)
    try:
        text = data.decode('ascii')
    except UnicodeDecodeError:
        raise RequestError("Request is not ASCII text")
    # no control characters, which would reach the console in our reports
    if any(not (32 <= ord(c) < 127 or c in '\t\r\n') for c in text):
        raise RequestError("Request is not printable text")
    words = text.split()
    if len(words) == 0:
        raise RequestError("Empty request")
    return words


def format_destination(ip, port):
    return "%s:%u" % (ip, port)


class OutputCtlModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "outputctl", "UDP control of runtime outputs")
        self.outputctl_settings = mp_settings.MPSettings([
            MPSetting('bind', str, '127.0.0.1', 'control socket address'),
            MPSetting('port', int, 14600, 'control socket port', range=(0, 65535)),
            MPSetting('allow', str, '127.0.0.1', 'allowed senders'),
            MPSetting('max_outputs', int, 16, 'most outputs which can be added', range=(1, 256)),
        ])
        self.outputctl_settings.set_callback(self.setting_changed)
        self.add_command('outputctl', self.cmd_outputctl, "UDP control of runtime outputs",
                         ["<status|list|clear>", "set (OUTPUTCTLSETTING)"])
        self.add_completion_function('(OUTPUTCTLSETTING)', self.outputctl_settings.completion)

        # requests are answered on the main loop, which is also where
        # messages are forwarded to the outputs, so changes we make to the
        # outputs never race the forwarding.  The lock is for unload(),
        # which MAVProxy calls from another thread at exit
        self.lock = threading.RLock()
        self.unloaded = False
        # the outputs we added, in the order we added them: (ip, port) -> connection
        self.dynamic = {}
        self.sock = None
        # the socket is opened from idle_task, so settings from --cmd and
        # startup scripts apply before it is first opened
        self.next_open = 0
        self.open_error = None
        self.allowed = []
        self.update_allowed()
        self.rejected = 0
        self.last_rejected = None
        self.last_reject_report = None

    def setting_changed(self, setting):
        with self.lock:
            if setting.name == 'allow':
                self.update_allowed()
            elif setting.name in ('bind', 'port'):
                self.close_socket()
                self.next_open = 0
                self.open_error = None

    def update_allowed(self):
        allow = self.outputctl_settings.allow
        try:
            self.allowed = parse_allow(allow)
        except ValueError as ex:
            # fail closed: a typo must not open the socket to everyone
            self.allowed = []
            print("outputctl: invalid allow setting '%s' (%s): rejecting all requests" % (allow, ex))
            return
        if len(self.allowed) == 0:
            print("outputctl: allow is empty: rejecting all requests")
        elif any(net.prefixlen == 0 for net in self.allowed):
            print("outputctl: warning: allow %s accepts requests from any host" % allow)

    def sender_allowed(self, ip):
        try:
            addr = ipaddress.IPv4Address(ip)
        except ValueError:
            return False
        return any(addr in net for net in self.allowed)

    def open_socket(self):
        '''open the control socket, or arrange to try again later'''
        bind = self.outputctl_settings.bind
        port = self.outputctl_settings.port
        sock = None
        try:
            try:
                ipaddress.IPv4Address(bind)
            except ValueError:
                raise OSError("bind must be an IPv4 address")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                # Windows: stop another program binding our port to read our requests
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((bind, port))
            sock.setblocking(False)
        except OSError as ex:
            if sock is not None:
                sock.close()
            self.next_open = time.monotonic() + OPEN_RETRY_PERIOD
            # each new reason once, rather than at every retry
            if str(ex) != self.open_error:
                self.open_error = str(ex)
                print("outputctl: cannot listen on %s: %s (retrying every %us)" % (
                    format_destination(bind, port), ex, OPEN_RETRY_PERIOD))
            return
        mavutil.set_close_on_exec(sock.fileno())
        mp_util.child_fd_list_add(sock.fileno())
        self.sock = sock
        self.open_error = None
        print("outputctl: listening on %s for requests from %s" % (
            format_destination(bind, sock.getsockname()[1]), self.outputctl_settings.allow))
        if bind == '0.0.0.0':
            print("outputctl: warning: listening on all interfaces")

    def close_socket(self):
        if self.sock is None:
            return
        mp_util.child_fd_list_remove(self.sock.fileno())
        self.sock.close()
        self.sock = None
        print("outputctl: closed control socket")

    def idle_task(self):
        '''answer any requests waiting on the control socket'''
        with self.lock:
            if self.unloaded:
                return
            if self.sock is None:
                if time.monotonic() >= self.next_open:
                    self.open_socket()
                return
            self.poll()

    def poll(self):
        '''answer the requests waiting on the control socket, without blocking'''
        for _ in range(MAX_REQUESTS_PER_POLL):
            try:
                (readable, _, _) = select.select([self.sock], [], [], 0)
                if not readable:
                    return
                (data, sender) = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except ConnectionResetError:
                # Windows reports an ICMP port unreachable for an earlier reply here
                continue
            except OSError as ex:
                print("outputctl: receive failed: %s; reopening control socket" % ex)
                self.close_socket()
                self.next_open = 0
                return
            reply = self.handle_datagram(data, sender)
            try:
                self.sock.sendto(reply.encode('ascii', 'replace'), sender)
            except OSError as ex:
                print("outputctl: cannot reply to %s: %s" % (format_destination(*sender[:2]), ex))

    def handle_datagram(self, data, sender):
        '''the reply to the request in data, from sender'''
        if not self.sender_allowed(sender[0]):
            self.report_rejected(sender)
            return "ERR Unauthorized sender"
        words = None
        try:
            words = decode_request(data)
            reply = "OK " + self.handle_request(words)
        except RequestError as ex:
            reply = "ERR %s" % ex
        except Exception as ex:
            # answer, and keep MAVProxy running, whatever went wrong
            traceback.print_exc()
            reply = "ERR Internal error: %s" % ex
        if reply.startswith("ERR") or words[0].upper() not in QUIET_COMMANDS:
            request = ' '.join(words) if words is not None else '(not decoded)'
            print("outputctl: %s %s: %s" % (
                format_destination(*sender[:2]), request, reply.split('\n')[0]))
        return reply

    def report_rejected(self, sender):
        '''report requests from senders which are not allowed, at most every few seconds'''
        self.rejected += 1
        self.last_rejected = sender
        now = time.monotonic()
        if self.last_reject_report is not None and now - self.last_reject_report < REJECT_REPORT_PERIOD:
            return
        print("outputctl: rejected %u request(s) from senders not in allow (%s), latest from %s" % (
            self.rejected, self.outputctl_settings.allow, format_destination(*sender[:2])))
        self.rejected = 0
        self.last_reject_report = now

    def handle_request(self, words):
        '''the text of the OK reply to a request, or raise RequestError'''
        command = words[0].upper()
        args = words[1:]
        if command == 'ADD':
            return self.add(*self.destination_args(command, args))
        if command == 'REMOVE':
            return self.remove(*self.destination_args(command, args))
        if command == 'LIST':
            self.no_args(command, args)
            return self.list_outputs()
        if command == 'CLEAR':
            self.no_args(command, args)
            return self.clear()
        raise RequestError("Unknown command: %s" % words[0])

    def destination_args(self, command, args):
        if len(args) != 2:
            raise RequestError("Usage: %s <ip> <port>" % command)
        return (parse_ip(args[0]), parse_port(args[1]))

    def no_args(self, command, args):
        if len(args) != 0:
            raise RequestError("Usage: %s" % command)

    def forget_removed(self):
        '''forget outputs removed behind our back, eg with "output remove"'''
        for (key, conn) in list(self.dynamic.items()):
            if not any(c is conn for c in self.mpstate.mav_outputs):
                print("outputctl: %s was removed by another command" % format_destination(*key))
                del self.dynamic[key]

    def find_configured(self, ip, port):
        '''an output we did not add which already sends to ip:port'''
        for conn in self.mpstate.mav_outputs:
            if any(c is conn for c in self.dynamic.values()):
                continue
            if (isinstance(conn, mavutil.mavudp) and not conn.udp_server and
                    tuple(conn.destination_addr) == (ip, port)):
                return conn
        return None

    def is_own_socket(self, ip, port):
        '''is ip:port a UDP socket of this MAVProxy? Sending our output
        there would feed it back to us, and loop'''
        socks = []
        if self.sock is not None:
            socks.append(self.sock)
        conns = (list(self.mpstate.mav_master or []) + list(self.mpstate.mav_outputs) +
                 list(self.mpstate.sysid_outputs.values()))
        for conn in conns:
            if isinstance(conn, mavutil.mavudp):
                socks.append(conn.port)
        loopback = ipaddress.IPv4Address(ip).is_loopback
        for sock in socks:
            try:
                (bound_ip, bound_port) = sock.getsockname()[:2]
            except OSError:
                # not bound yet
                continue
            # a loopback destination on one of our ports is never a GCS: an
            # output there is bound to that port itself by pymavlink 2.4.50
            # on Windows, so sends to itself, or else reaches nothing
            if bound_port == port and (bound_ip == ip or loopback):
                return True
        return False

    def add(self, ip, port):
        self.forget_removed()
        destination = format_destination(ip, port)
        if (ip, port) in self.dynamic:
            raise RequestError("Destination already exists: %s" % destination)
        if self.find_configured(ip, port) is not None:
            raise RequestError("Destination already exists: %s (configured output)" % destination)
        if self.is_own_socket(ip, port):
            raise RequestError("Destination is a MAVProxy socket: %s" % destination)
        if len(self.dynamic) >= self.outputctl_settings.max_outputs:
            raise RequestError("Too many outputs (max %u)" % self.outputctl_settings.max_outputs)
        try:
            conn = mavproxy_output.add_output(self.mpstate, 'udpout:' + destination)
        except Exception as ex:
            raise RequestError("Failed to open output %s: %s" % (destination, ex))
        self.dynamic[(ip, port)] = conn
        return "Added %s" % destination

    def remove(self, ip, port):
        self.forget_removed()
        destination = format_destination(ip, port)
        conn = self.dynamic.pop((ip, port), None)
        if conn is None:
            if self.find_configured(ip, port) is not None:
                raise RequestError("Not a dynamic output: %s" % destination)
            raise RequestError("Not found: %s" % destination)
        mavproxy_output.remove_output(self.mpstate, conn)
        return "Removed %s" % destination

    def list_outputs(self):
        self.forget_removed()
        lines = ["List %u" % len(self.dynamic)]
        for (ip, port) in self.dynamic:
            lines.append("%s active" % format_destination(ip, port))
        return '\n'.join(lines)

    def clear(self):
        self.forget_removed()
        count = len(self.dynamic)
        self.remove_all()
        return "Cleared %u outputs" % count

    def remove_all(self):
        while self.dynamic:
            (key, conn) = self.dynamic.popitem()
            mavproxy_output.remove_output(self.mpstate, conn)

    def cmd_outputctl(self, args):
        '''handle outputctl commands'''
        usage = "usage: outputctl <status|list|clear|set>"
        with self.lock:
            if len(args) == 0 or args[0] == 'status':
                self.cmd_status()
            elif args[0] == 'list':
                print(self.list_outputs())
            elif args[0] == 'clear':
                print(self.clear())
            elif args[0] == 'set':
                self.outputctl_settings.command(args[1:])
            else:
                print(usage)

    def cmd_status(self):
        if self.sock is not None:
            print("listening on %s for requests from %s" % (
                format_destination(*self.sock.getsockname()[:2]), self.outputctl_settings.allow))
        elif self.open_error is not None:
            print("not listening: cannot listen on %s: %s" % (
                format_destination(self.outputctl_settings.bind, self.outputctl_settings.port), self.open_error))
        else:
            print("not listening")
        print(self.list_outputs())

    def unload(self):
        '''close the control socket and remove the outputs we added'''
        with self.lock:
            self.unloaded = True
            self.close_socket()
            count = len(self.dynamic)
            self.remove_all()
            print("outputctl: removed %u dynamic outputs" % count)
        super().unload()


def init(mpstate):
    '''initialise module'''
    return OutputCtlModule(mpstate)
