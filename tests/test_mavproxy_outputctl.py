#!/usr/bin/env python3
"""runtime UDP outputs added and removed by a GCS over the outputctl
control socket.

The first tests drive the module in-process over real sockets on
localhost, with as much of MAVProxy's state as the output modules use.
OutputCtlEndToEndTest runs a real mavproxy.py, started as it normally
is, between a fake vehicle and real UDP receivers.
"""

import os
import random
import site
import socket
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from types import SimpleNamespace

from pymavlink import mavutil

from MAVProxy.modules import mavproxy_output
from MAVProxy.modules import mavproxy_outputctl
from MAVProxy.modules.lib import mp_util

try:
    # mavproxy.py itself needs more than the modules do, eg wx on Windows
    from MAVProxy import mavproxy  # noqa: F401
    HAVE_MAVPROXY = True
except ImportError:
    HAVE_MAVPROXY = False

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GCS_SENDER = ('127.0.0.1', 5555)


def make_mpstate():
    '''as much of MAVProxy's state as the output modules use'''
    return SimpleNamespace(
        command_map={}, completions={}, completion_functions={},
        public_modules={}, multi_instance={}, instance_count={},
        settings=SimpleNamespace(source_system=255, source_component=230),
        mav_master=[], mav_outputs=[], sysid_outputs={})


def free_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def unused_port():
    '''a free port on localhost, below the ranges ports are given out from
    to sockets which send without binding, as pymavlink's udpout ones do.
    Windows can give such a socket a port already bound on 127.0.0.1, and
    then replies to it go to the other socket'''
    while True:
        port = random.randrange(20000, 30000)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('127.0.0.1', port))
            return port
        except OSError:
            pass
        finally:
            sock.close()


def receiver():
    '''a UDP socket on localhost which outputs can send to'''
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(2)
    return sock


class OutputCtlParseTest(unittest.TestCase):
    '''the checks on the words of a request'''

    def test_ip(self):
        self.assertEqual(mavproxy_outputctl.parse_ip('192.168.2.117'), '192.168.2.117')
        for bad in ('999.1.1.1', '192.168.2', 'localhost', '192.168.002.117', '::1',
                    '0.0.0.0', '224.0.0.1', '255.255.255.255', ''):
            with self.assertRaises(mavproxy_outputctl.RequestError, msg=bad) as cm:
                mavproxy_outputctl.parse_ip(bad)
            self.assertTrue(str(cm.exception).startswith('Invalid IP address: '), bad)

    def test_port(self):
        self.assertEqual(mavproxy_outputctl.parse_port('14550'), 14550)
        self.assertEqual(mavproxy_outputctl.parse_port('1'), 1)
        self.assertEqual(mavproxy_outputctl.parse_port('65535'), 65535)
        for bad in ('0', '65536', '-1', '14550.0', 'abc', '0x10', '123456', '١'):
            with self.assertRaises(mavproxy_outputctl.RequestError, msg=bad) as cm:
                mavproxy_outputctl.parse_port(bad)
            self.assertEqual(str(cm.exception), 'Port must be 1-65535')

    def test_allow(self):
        nets = mavproxy_outputctl.parse_allow('127.0.0.1, 192.168.2.0/24,')
        self.assertEqual([str(n) for n in nets], ['127.0.0.1/32', '192.168.2.0/24'])
        self.assertEqual(mavproxy_outputctl.parse_allow(''), [])
        with self.assertRaises(ValueError):
            mavproxy_outputctl.parse_allow('127.0.0.1,300.1.1.1')

    def test_decode(self):
        self.assertEqual(mavproxy_outputctl.decode_request(b'  add 1.2.3.4\t14550\r\n'),
                         ['add', '1.2.3.4', '14550'])
        for (data, error) in ((b'', 'Empty request'),
                              (b' \n', 'Empty request'),
                              (b'\xffADD', 'Request is not ASCII text'),
                              (b'LIST\x1b[2J', 'Request is not printable text'),
                              (b'L' * 257, 'Request too long (max 256 bytes)')):
            with self.assertRaises(mavproxy_outputctl.RequestError) as cm:
                mavproxy_outputctl.decode_request(data)
            self.assertEqual(str(cm.exception), error)


class OutputCtlModuleTest(unittest.TestCase):
    '''the module, with real sockets on localhost'''

    def setUp(self):
        self.threads = set(threading.enumerate())
        self.mpstate = make_mpstate()
        self.module = mavproxy_outputctl.init(self.mpstate)
        self.module.outputctl_settings.port = 0
        self.addCleanup(self.module.unload)
        self.module.idle_task()
        self.assertIsNotNone(self.module.sock)
        self.control = ('127.0.0.1', self.module.sock.getsockname()[1])
        self.receivers = [receiver() for i in range(3)]
        for r in self.receivers:
            self.addCleanup(r.close)

    def dest(self, i):
        return ('127.0.0.1', self.receivers[i].getsockname()[1])

    def request(self, text, sender=GCS_SENDER):
        return self.module.handle_datagram(text.encode('ascii'), sender)

    def add(self, i):
        return self.request('ADD %s %u' % self.dest(i))

    def remove(self, i):
        return self.request('REMOVE %s %u' % self.dest(i))

    def name(self, i):
        return '%s:%u' % self.dest(i)

    def forward(self, data=b'telemetry'):
        '''what the link module does with each message from the vehicle'''
        for conn in self.mpstate.mav_outputs:
            conn.write(data)

    def assert_receives(self, i, data=b'telemetry'):
        self.assertEqual(self.receivers[i].recvfrom(100)[0], data)

    def assert_receives_nothing(self, i):
        self.receivers[i].settimeout(0.2)
        with self.assertRaises(socket.timeout):
            self.receivers[i].recvfrom(100)

    def test_add_forwards(self):
        self.assertEqual(self.add(0), 'OK Added %s' % self.name(0))
        self.assertEqual(self.add(1), 'OK Added %s' % self.name(1))
        self.assertEqual(len(self.mpstate.mav_outputs), 2)
        # real pymavlink UDP outputs, as "output add" makes
        for conn in self.mpstate.mav_outputs:
            self.assertIsInstance(conn, mavutil.mavudp)
            self.assertFalse(conn.udp_server)
        self.forward()
        self.assert_receives(0)
        self.assert_receives(1)

    def test_duplicate(self):
        self.add(0)
        self.assertEqual(self.add(0), 'ERR Destination already exists: %s' % self.name(0))
        self.assertEqual(len(self.mpstate.mav_outputs), 1)

    def test_configured_outputs_untouched(self):
        configured = mavproxy_output.add_output(self.mpstate, 'udp:%s:%u' % self.dest(2))
        self.addCleanup(mavproxy_output.remove_output, self.mpstate, configured)
        self.assertEqual(self.add(2), 'ERR Destination already exists: %s (configured output)' % self.name(2))
        self.assertEqual(self.remove(2), 'ERR Not a dynamic output: %s' % self.name(2))
        self.add(0)
        self.assertEqual(self.request('CLEAR'), 'OK Cleared 1 outputs')
        self.assertEqual(self.mpstate.mav_outputs, [configured])
        self.forward()
        self.assert_receives(2)
        self.assert_receives_nothing(0)
        self.module.unload()
        self.assertEqual(self.mpstate.mav_outputs, [configured])

    def test_remove(self):
        self.add(0)
        self.add(1)
        conn = self.mpstate.mav_outputs[0]
        fd = conn.port.fileno()
        # fd numbers are reused, so count rather than look for this one
        listed = mp_util.child_fd_list.count(fd)
        self.assertEqual(self.remove(0), 'OK Removed %s' % self.name(0))
        self.assertEqual(conn.port.fileno(), -1)
        self.assertEqual(mp_util.child_fd_list.count(fd), listed - 1)
        self.forward()
        self.assert_receives(1)
        self.assert_receives_nothing(0)
        self.assertEqual(self.remove(0), 'ERR Not found: %s' % self.name(0))

    def test_list(self):
        self.assertEqual(self.request('LIST'), 'OK List 0')
        self.add(1)
        self.add(0)
        self.assertEqual(self.request('list'), 'OK List 2\n%s active\n%s active' % (self.name(1), self.name(0)))
        self.assertEqual(self.request('LIST extra'), 'ERR Usage: LIST')

    def test_clear(self):
        self.add(0)
        self.add(1)
        self.assertEqual(self.request('CLEAR'), 'OK Cleared 2 outputs')
        self.assertEqual(self.mpstate.mav_outputs, [])
        self.assertEqual(self.request('CLEAR'), 'OK Cleared 0 outputs')

    def test_invalid(self):
        for (request, reply) in (
                ('ADD 999.1.1.1 14550', 'ERR Invalid IP address: 999.1.1.1'),
                ('ADD 127.0.0.1 0', 'ERR Port must be 1-65535'),
                ('ADD 127.0.0.1 70000', 'ERR Port must be 1-65535'),
                ('ADD 127.0.0.1', 'ERR Usage: ADD <ip> <port>'),
                ('REMOVE', 'ERR Usage: REMOVE <ip> <port>'),
                ('CLEAR now', 'ERR Usage: CLEAR'),
                ('DELETE 127.0.0.1 14550', 'ERR Unknown command: DELETE')):
            self.assertEqual(self.request(request), reply)
        self.assertEqual(self.mpstate.mav_outputs, [])

    def test_max_outputs(self):
        self.module.outputctl_settings.max_outputs = 2
        self.add(0)
        self.add(1)
        self.assertEqual(self.add(2), 'ERR Too many outputs (max 2)')

    def test_no_loop_to_own_sockets(self):
        # a master link listening on all interfaces, as --master=udp:0.0.0.0:N
        master = mavutil.mavlink_connection('udp:0.0.0.0:0', input=True)
        self.addCleanup(master.close)
        self.mpstate.mav_master = [master]
        master_port = master.port.getsockname()[1]
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % master_port),
                         'ERR Destination is a MAVProxy socket: 127.0.0.1:%u' % master_port)
        self.assertEqual(self.request('ADD %s %u' % self.control),
                         'ERR Destination is a MAVProxy socket: %s:%u' % self.control)
        self.assertEqual(self.mpstate.mav_outputs, [])

    def test_no_loopback_to_own_ports(self):
        # a master link on one address, as --master=udp:172.24.96.1:14552,
        # and an output to the same port on another loopback address
        master = mavutil.mavlink_connection('udp:127.0.0.1:%u' % free_port(), input=True)
        self.addCleanup(master.close)
        self.mpstate.mav_master = [master]
        master_port = master.port.getsockname()[1]
        self.assertEqual(self.request('ADD 127.0.0.2 %u' % master_port),
                         'ERR Destination is a MAVProxy socket: 127.0.0.2:%u' % master_port)
        self.assertEqual(self.mpstate.mav_outputs, [])

    def test_unauthorized(self):
        self.assertEqual(self.request('LIST', ('192.168.2.117', 5555)), 'ERR Unauthorized sender')
        self.module.outputctl_settings.set('allow', '192.168.2.0/24')
        self.assertEqual(self.request('LIST', ('192.168.2.117', 5555)), 'OK List 0')
        self.assertEqual(self.request('ADD 127.0.0.1 14550', ('192.168.3.1', 5555)), 'ERR Unauthorized sender')
        self.assertEqual(self.request('LIST'), 'ERR Unauthorized sender')
        self.assertEqual(self.mpstate.mav_outputs, [])

    def test_invalid_allow_rejects_all(self):
        self.module.outputctl_settings.set('allow', '127.0.0.1,300.1.1.1')
        self.assertEqual(self.module.allowed, [])
        self.assertEqual(self.request('LIST'), 'ERR Unauthorized sender')

    def test_removed_by_output_command(self):
        output = mavproxy_output.init(self.mpstate)
        self.add(0)
        self.add(1)
        output.cmd_output(['remove', '0'])
        self.assertEqual(self.request('LIST'), 'OK List 1\n%s active' % self.name(1))
        self.assertEqual(self.remove(0), 'ERR Not found: %s' % self.name(0))

    def test_internal_error_answered(self):
        def broken(mpstate, device):
            raise RuntimeError('no sockets left')
        original = mavproxy_output.add_output
        mavproxy_output.add_output = broken
        try:
            self.assertEqual(self.add(0), 'ERR Failed to open output %s: no sockets left' % self.name(0))
        finally:
            mavproxy_output.add_output = original
        self.module.list_outputs = None  # makes LIST fail inside the module
        self.assertEqual(self.request('LIST'), "ERR Internal error: 'NoneType' object is not callable")

    def ask(self, client, data):
        '''send a request over the control socket and return the reply'''
        client.sendto(data, self.control)
        client.settimeout(0.01)
        for i in range(200):
            self.module.idle_task()
            try:
                return client.recvfrom(65535)[0].decode('ascii')
            except socket.timeout:
                pass
        self.fail('no reply to %s' % data)

    def test_control_socket(self):
        client = receiver()
        self.addCleanup(client.close)
        self.assertEqual(self.ask(client, ('ADD %s %u\n' % self.dest(0)).encode()), 'OK Added %s' % self.name(0))
        self.assertEqual(self.ask(client, b'LIST'), 'OK List 1\n%s active' % self.name(0))
        self.assertEqual(self.ask(client, b''), 'ERR Empty request')
        self.assertEqual(self.ask(client, b'x' * 2000), 'ERR Request too long (max 256 bytes)')

    def test_reply_to_closed_port(self):
        # on Windows a reply to a closed port makes the next receive fail
        gone = receiver()
        gone.sendto(b'LIST', self.control)
        gone.close()
        for i in range(10):
            self.module.idle_task()
        client = receiver()
        self.addCleanup(client.close)
        self.assertEqual(self.ask(client, b'LIST'), 'OK List 0')

    def test_requests_per_poll_bounded(self):
        client = receiver()
        self.addCleanup(client.close)
        for i in range(mavproxy_outputctl.MAX_REQUESTS_PER_POLL + 5):
            client.sendto(b'LIST', self.control)
        time.sleep(0.2)
        self.module.idle_task()
        replies = 0
        client.settimeout(0.2)
        try:
            while True:
                client.recvfrom(100)
                replies += 1
        except socket.timeout:
            pass
        self.assertEqual(replies, mavproxy_outputctl.MAX_REQUESTS_PER_POLL)

    def test_port_change_reopens(self):
        old_port = self.control[1]
        new_port = free_port()
        self.module.cmd_outputctl(['set', 'port', str(new_port)])
        self.assertIsNone(self.module.sock)
        self.module.idle_task()
        self.assertEqual(self.module.sock.getsockname(), ('127.0.0.1', new_port))
        # the old port was given back
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', old_port))
        sock.close()

    def test_unload_cleans_up(self):
        self.add(0)
        self.add(1)
        conns = list(self.mpstate.mav_outputs)
        control_port = self.control[1]
        self.module.unload()
        self.assertIsNone(self.module.sock)
        self.assertEqual(self.mpstate.mav_outputs, [])
        for conn in conns:
            self.assertEqual(conn.port.fileno(), -1)
        # an unloaded module does not open its socket again
        self.module.idle_task()
        self.assertIsNone(self.module.sock)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', control_port))
        sock.close()
        self.assertEqual(set(threading.enumerate()), self.threads)


class OutputCtlBindTest(unittest.TestCase):
    def test_bind_failure_retries(self):
        blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        blocker.bind(('127.0.0.1', 0))
        port = blocker.getsockname()[1]
        module = mavproxy_outputctl.init(make_mpstate())
        self.addCleanup(module.unload)
        module.outputctl_settings.port = port
        module.idle_task()
        self.assertIsNone(module.sock)
        error = module.open_error
        self.assertIsNotNone(error)
        # no retry until the retry period has passed
        module.idle_task()
        self.assertIsNone(module.sock)
        blocker.close()
        module.next_open = 0
        module.idle_task()
        self.assertEqual(module.sock.getsockname(), ('127.0.0.1', port))
        self.assertIsNone(module.open_error)

    def test_bad_bind_address(self):
        module = mavproxy_outputctl.init(make_mpstate())
        self.addCleanup(module.unload)
        module.outputctl_settings.bind = 'localhost'
        module.idle_task()
        self.assertIsNone(module.sock)
        self.assertEqual(module.open_error, 'bind must be an IPv4 address')


class OutputModuleTest(unittest.TestCase):
    '''the console output commands, which share their code with outputctl'''

    def setUp(self):
        self.mpstate = make_mpstate()
        self.module = mavproxy_output.init(self.mpstate)
        self.receiver = receiver()
        self.addCleanup(self.receiver.close)
        self.device = '127.0.0.1:%u' % self.receiver.getsockname()[1]

    def test_add_list_remove(self):
        self.module.cmd_output(['add', self.device])
        self.assertEqual(len(self.mpstate.mav_outputs), 1)
        conn = self.mpstate.mav_outputs[0]
        self.assertEqual(conn.address, self.device)
        self.assertEqual(conn.mav.srcComponent, 230)
        fd = conn.port.fileno()
        listed = mp_util.child_fd_list.count(fd)
        self.assertGreater(listed, 0)
        conn.write(b'hello')
        self.assertEqual(self.receiver.recvfrom(100)[0], b'hello')
        self.module.cmd_output(['list'])
        self.module.cmd_output(['remove', '0'])
        self.assertEqual(self.mpstate.mav_outputs, [])
        self.assertEqual(conn.port.fileno(), -1)
        self.assertEqual(mp_util.child_fd_list.count(fd), listed - 1)

    def test_remove_by_address(self):
        self.module.cmd_output(['add', self.device])
        self.module.cmd_output(['add', '127.0.0.1:1'])
        self.module.cmd_output(['remove', self.device])
        self.assertEqual([c.address for c in self.mpstate.mav_outputs], ['127.0.0.1:1'])
        mavproxy_output.remove_output(self.mpstate, self.mpstate.mav_outputs[0])

    def test_add_failure_reported(self):
        import io
        from contextlib import redirect_stdout
        out = io.StringIO()
        with redirect_stdout(out):
            self.module.cmd_output(['add', 'udpout:nohost'])
        self.assertIn('Failed to connect to udpout:nohost: UDP ports must be specified as host:port', out.getvalue())
        self.assertEqual(self.mpstate.mav_outputs, [])

    def test_list_replaced_not_changed(self):
        '''a loop over the outputs on another thread keeps the list it started with'''
        conn = mavproxy_output.add_output(self.mpstate, self.device)
        snapshot = self.mpstate.mav_outputs
        mavproxy_output.remove_output(self.mpstate, conn)
        self.assertEqual(snapshot, [conn])
        self.assertEqual(self.mpstate.mav_outputs, [])
        # writing to a removed output is harmless, as it is for --out outputs
        conn.write(b'late')


# the defaults less terrain, which can download the SRTM file list in a
# child process that exit then waits for
MODULES = ('log,signing,wp,rally,fence,ftp,param,relay,tuneopt,arm,mode,'
           'calibration,rc,auxopt,misc,cmdlong,battery,output,adsb,layout')

# console commands for the MAVProxy under test, which has no terminal:
# each line of the file is queued as if typed
CMDFEEDER = textwrap.dedent('''
    import os

    from MAVProxy.modules.lib import mp_module


    class CmdFeeder(mp_module.MPModule):
        def __init__(self, mpstate):
            super().__init__(mpstate, 'cmdfeeder')
            self.path = os.environ['CMDFEEDER_FILE']

        def idle_task(self):
            try:
                with open(self.path) as f:
                    lines = f.read().splitlines()
                os.remove(self.path)
            except OSError:
                return
            for line in lines:
                self.mpstate.input_queue.put(line)

        def unload(self):
            # loaded after outputctl, so this is unloaded after it at exit
            open(self.path + '.unloaded', 'w').close()


    def init(mpstate):
        return CmdFeeder(mpstate)
''')

# on Windows, rline makes a prompt_toolkit session even with
# --non-interactive, and that needs stdout to be a console rather than
# our pipe. The session is never used to prompt, so let it be made on
# prompt_toolkit's dummy input and output
WINDOWS_PIPE_SHIM = textwrap.dedent('''
    import functools
    import sys
    if sys.platform == 'win32':
        import prompt_toolkit
        from prompt_toolkit.input import DummyInput
        from prompt_toolkit.output import DummyOutput
        prompt_toolkit.PromptSession = functools.partial(
            prompt_toolkit.PromptSession, input=DummyInput(), output=DummyOutput())
''')

VEHICLE_SYSID = 7


def udpout_binds_destination_port():
    '''does pymavlink bind udpout sockets to their destination port, as
    2.4.50 does on Windows? Then an output to a receiver on the same host
    shares its port, and the receiver's replies come back to itself'''
    conn = mavutil.mavlink_connection('udpout:127.0.0.1:%u' % unused_port())
    try:
        return conn.port.getsockname()[1] != 0
    except OSError:
        # not bound
        return False
    finally:
        conn.close()


class Vehicle(threading.Thread):
    '''sends HEARTBEAT and ATTITUDE to MAVProxy's master link at 20Hz,
    and counts what it is sent. A plain socket on a port of its own, as
    a pymavlink udpout one may be bound to MAVProxy's port'''

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', unused_port()))
        self.sock.setblocking(False)
        self.master = ('127.0.0.1', port)
        self.mav = mavutil.mavlink.MAVLink(self, srcSystem=VEHICLE_SYSID, srcComponent=1)
        self.commands = []
        self.heartbeats = 0
        self.stop = threading.Event()

    def write(self, buf):
        try:
            self.sock.sendto(buf, self.master)
        except ConnectionResetError:
            pass

    def run(self):
        while not self.stop.is_set():
            self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            self.mav.attitude_send(0, 0.1, 0.2, 0.3, 0, 0, 0)
            while True:
                try:
                    data = self.sock.recvfrom(65535)[0]
                except BlockingIOError:
                    break
                except ConnectionResetError:
                    # MAVProxy not listening yet
                    continue
                for m in self.mav.parse_buffer(data) or []:
                    if m.get_type() == 'COMMAND_LONG':
                        self.commands.append(m)
                    elif m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == 255:
                        self.heartbeats += 1
            time.sleep(0.05)
        self.sock.close()

    def wait_command(self, param1, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if any(c.param1 == param1 for c in self.commands):
                return True
            time.sleep(0.05)
        return False

    def wait_heartbeats(self, timeout=10):
        '''has MAVProxy sent us a heartbeat since we asked?'''
        start = self.heartbeats
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.heartbeats > start:
                return True
            time.sleep(0.05)
        return False


class GCS(object):
    '''a MAVLink receiver for one of MAVProxy's outputs'''

    def __init__(self):
        self.port = unused_port()
        self.conn = mavutil.mavlink_connection('udpin:127.0.0.1:%u' % self.port, source_system=254)

    def drain(self):
        while True:
            try:
                self.conn.port.recvfrom(65535)
            except BlockingIOError:
                return
            except ConnectionResetError:
                continue

    def wait_vehicle(self, mtype='HEARTBEAT', timeout=10):
        '''the next valid message of mtype from the vehicle, or None'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.conn.recv_match(type=mtype, blocking=True, timeout=0.5)
            if m is not None and m.get_srcSystem() == VEHICLE_SYSID:
                return m
        return None

    def receives_nothing(self, period=1.5):
        self.drain()
        return self.conn.recv_match(blocking=True, timeout=period) is None

    def close(self):
        self.conn.close()


@unittest.skipUnless(HAVE_MAVPROXY, 'mavproxy.py dependencies not installed')
class OutputCtlEndToEndTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix='mavproxy-outputctl-')
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        with open(os.path.join(self.tmp, 'cmdfeeder.py'), 'w') as f:
            f.write(CMDFEEDER)
        self.cmd_file = os.path.join(self.tmp, 'commands')
        self.master_port = unused_port()
        self.control_port = unused_port()
        self.vehicle = Vehicle(self.master_port)
        self.vehicle.start()
        self.addCleanup(self.vehicle.join)
        self.addCleanup(self.vehicle.stop.set)
        self.static = GCS()
        self.addCleanup(self.static.close)
        self.lines = []

    def start(self, console=False):
        '''start MAVProxy with its output on our pipe, or else in a hidden
        console of its own on Windows, for which we cannot see its output'''
        # the original command, plus outputctl on a free port. --no-console
        # keeps the GUI console out of the way, and only exists on Windows
        args = [sys.executable, os.path.join(REPO, 'MAVProxy', 'mavproxy.py'),
                '--master=udp:127.0.0.1:%u' % self.master_port,
                '--out=udp:127.0.0.1:%u' % self.static.port,
                '--load-module=outputctl,cmdfeeder',
                '--cmd=outputctl set port %u' % self.control_port,
                '--non-interactive', '--no-startup-scripts',
                '--state-basedir=' + self.tmp,
                '--default-modules=' + MODULES]
        if sys.platform == 'win32':
            args.append('--no-console')
        # HOME keeps the user's MAVProxy files out of it, and moving it
        # must not lose the user's site-packages, which are found from it
        env = dict(os.environ, HOME=self.tmp, PYTHONUNBUFFERED='1',
                   PYTHONUSERBASE=site.getuserbase(),
                   PYTHONPATH=os.pathsep.join([REPO, self.tmp]),
                   CMDFEEDER_FILE=self.cmd_file)
        if console:
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startupinfo.wShowWindow = 0  # SW_HIDE
            self.process = subprocess.Popen(args, creationflags=subprocess.CREATE_NEW_CONSOLE,
                                            startupinfo=startupinfo, cwd=self.tmp, env=env)
            self.addCleanup(self.kill)
            self.wait_listening(60)
            return
        with open(os.path.join(self.tmp, 'sitecustomize.py'), 'w') as f:
            f.write(WINDOWS_PIPE_SHIM)
        self.process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, cwd=self.tmp, env=env)
        self.addCleanup(self.kill)
        reader = threading.Thread(target=self.read_output, daemon=True)
        reader.start()
        self.wait_output('outputctl: listening on 127.0.0.1:%u' % self.control_port, 60)

    def wait_listening(self, timeout):
        '''wait for the control socket to answer'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.assertIsNone(self.process.poll(), 'MAVProxy exited')
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.settimeout(0.5)
                sock.sendto(b'LIST', ('127.0.0.1', self.control_port))
                if sock.recvfrom(65535)[0] == b'OK List 0':
                    return
            except (socket.timeout, ConnectionResetError):
                pass
            finally:
                sock.close()
            time.sleep(0.2)
        self.fail('control socket never answered')

    def read_output(self):
        for line in self.process.stdout:
            self.lines.append(line.decode('utf-8', 'replace').rstrip())

    def output(self):
        return '\n'.join(self.lines)

    def wait_output(self, text, timeout=10, count=1):
        '''wait for text to have been printed count times'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.output().count(text) >= count:
                return
            self.assertIsNone(self.process.poll(), 'MAVProxy exited:\n' + self.output())
            time.sleep(0.05)
        self.fail('no "%s" from MAVProxy:\n%s' % (text, self.output()))

    def kill(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def command(self, *lines):
        '''type lines on the MAVProxy console'''
        with open(self.cmd_file + '.tmp', 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(self.cmd_file + '.tmp', self.cmd_file)
        deadline = time.time() + 10
        while os.path.exists(self.cmd_file):
            self.assertLess(time.time(), deadline, 'commands not read')
            time.sleep(0.05)

    def request(self, text):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(5)
            sock.sendto(text.encode('ascii'), ('127.0.0.1', self.control_port))
            return sock.recvfrom(65535)[0].decode('ascii')
        finally:
            sock.close()

    def test_runtime_outputs(self):
        # 1, 2: started as usual, the master link and the --out output work
        self.start()
        self.assertIsNotNone(self.static.wait_vehicle(), 'static output gets nothing')
        self.assertTrue(self.vehicle.wait_heartbeats(), 'MAVProxy sends nothing to the vehicle')
        self.assertEqual(self.request('LIST'), 'OK List 0')

        gcs1 = GCS()
        gcs2 = GCS()
        self.addCleanup(gcs1.close)
        self.addCleanup(gcs2.close)
        d1 = '127.0.0.1:%u' % gcs1.port
        d2 = '127.0.0.1:%u' % gcs2.port

        # 3, 4, 5: two outputs added while running, both get valid MAVLink
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs1.port), 'OK Added ' + d1)
        self.assertIsNotNone(gcs1.wait_vehicle(), 'first new output gets nothing')
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs2.port), 'OK Added ' + d2)
        for gcs in (gcs1, gcs2, self.static):
            self.assertIsNotNone(gcs.wait_vehicle('HEARTBEAT'))
            att = gcs.wait_vehicle('ATTITUDE')
            self.assertIsNotNone(att)
            self.assertAlmostEqual(att.roll, 0.1, places=5)

        # 6: duplicates, and the outputs MAVProxy was started with
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs1.port), 'ERR Destination already exists: ' + d1)
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % self.static.port),
                         'ERR Destination already exists: 127.0.0.1:%u (configured output)' % self.static.port)
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % self.master_port),
                         'ERR Destination is a MAVProxy socket: 127.0.0.1:%u' % self.master_port)

        # 9: LIST
        self.assertEqual(self.request('LIST'), 'OK List 2\n%s active\n%s active' % (d1, d2))

        # 7: REMOVE stops just that output
        self.assertEqual(self.request('REMOVE 127.0.0.1 %u' % gcs1.port), 'OK Removed ' + d1)
        self.assertTrue(gcs1.receives_nothing(), 'removed output still gets messages')
        self.assertIsNotNone(gcs2.wait_vehicle())
        self.assertIsNotNone(self.static.wait_vehicle())
        self.assertEqual(self.request('REMOVE 127.0.0.1 %u' % gcs1.port), 'ERR Not found: ' + d1)
        self.assertEqual(self.request('REMOVE 127.0.0.1 %u' % self.static.port),
                         'ERR Not a dynamic output: 127.0.0.1:%u' % self.static.port)

        # 10: bad requests are answered, and MAVProxy carries on
        for (request, reply) in (('ADD 999.1.1.1 14550', 'ERR Invalid IP address: 999.1.1.1'),
                                 ('ADD 127.0.0.1 0', 'ERR Port must be 1-65535'),
                                 ('ADD 127.0.0.1', 'ERR Usage: ADD <ip> <port>'),
                                 ('JUNK', 'ERR Unknown command: JUNK')):
            self.assertEqual(self.request(request), reply)

        # an output nothing listens on does not upset the others
        unreachable = unused_port()
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % unreachable), 'OK Added 127.0.0.1:%u' % unreachable)
        time.sleep(1)
        self.assertIsNotNone(gcs2.wait_vehicle())

        # only allowed senders, changeable at runtime from the console
        self.command('outputctl set allow 10.1.2.3', 'outputctl status')
        self.wait_output('for requests from 10.1.2.3')
        self.assertEqual(self.request('LIST'), 'ERR Unauthorized sender')
        self.command('outputctl set allow 127.0.0.1', 'outputctl status')
        self.wait_output('for requests from 127.0.0.1', count=2)
        self.assertEqual(self.request('LIST'), 'OK List 2\n%s active\n127.0.0.1:%u active' % (d2, unreachable))

        # 8: CLEAR removes only what was added
        self.assertEqual(self.request('CLEAR'), 'OK Cleared 2 outputs')
        self.assertTrue(gcs2.receives_nothing(), 'cleared output still gets messages')
        self.assertEqual(self.request('LIST'), 'OK List 0')

        # 11: the master link and --out output are still working
        self.assertIsNotNone(self.static.wait_vehicle())
        self.assertTrue(self.vehicle.wait_heartbeats(), 'MAVProxy no longer sends to the vehicle')

        # unloading the module removes its outputs and frees its port
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs2.port), 'OK Added ' + d2)
        self.assertIsNotNone(gcs2.wait_vehicle())
        self.command('module unload outputctl')
        self.wait_output('Unloaded module outputctl')
        self.assertTrue(gcs2.receives_nothing(), 'output still sent to after unload')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', self.control_port))
        sock.close()
        self.assertIsNotNone(self.static.wait_vehicle())
        self.command('module load outputctl', 'outputctl set port %u' % self.control_port)
        self.wait_output('outputctl: listening on 127.0.0.1:%u' % self.control_port, count=2)
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs2.port), 'OK Added ' + d2)
        self.assertIsNotNone(gcs2.wait_vehicle())

        # 12: a clean exit, which closes the socket and the added output
        self.command('set requireexit True', 'exit')
        self.process.wait(timeout=30)
        time.sleep(0.5)
        output = self.output()
        self.assertIn('Unloading module outputctl', output)
        self.assertIn('outputctl: closed control socket', output)
        self.assertIn('outputctl: removed 1 dynamic outputs', output)
        self.assertNotIn('Traceback', output)
        self.assertNotIn('Internal error', output)

    @unittest.skipUnless(sys.platform == 'win32', 'only Windows needs a console')
    def test_windows_real_console(self):
        '''the same, on a real console with nothing patched, seen only over UDP'''
        self.start(console=True)
        self.assertIsNotNone(self.static.wait_vehicle(), 'static output gets nothing')
        gcs = GCS()
        self.addCleanup(gcs.close)
        destination = '127.0.0.1:%u' % gcs.port
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs.port), 'OK Added ' + destination)
        self.assertIsNotNone(gcs.wait_vehicle(), 'new output gets nothing')
        self.assertEqual(self.request('LIST'), 'OK List 1\n%s active' % destination)
        self.assertEqual(self.request('REMOVE 127.0.0.1 %u' % gcs.port), 'OK Removed ' + destination)
        self.assertTrue(gcs.receives_nothing(), 'removed output still gets messages')
        self.assertIsNotNone(self.static.wait_vehicle())
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs.port), 'OK Added ' + destination)
        self.command('set requireexit True', 'exit')
        self.process.wait(timeout=30)
        # the exit unloaded the modules, outputctl first
        self.assertTrue(os.path.exists(self.cmd_file + '.unloaded'))

    @unittest.skipIf(udpout_binds_destination_port(),
                     'this pymavlink binds outputs to the port of a receiver on this host, '
                     'so the receiver cannot reply to them')
    def test_gcs_reaches_vehicle(self):
        '''an added output is a full output: what its GCS sends reaches the
        vehicle, as from a --out one'''
        self.start()
        gcs = GCS()
        self.addCleanup(gcs.close)
        self.assertEqual(self.request('ADD 127.0.0.1 %u' % gcs.port), 'OK Added 127.0.0.1:%u' % gcs.port)
        self.assertIsNotNone(gcs.wait_vehicle())
        gcs.conn.mav.command_long_send(VEHICLE_SYSID, 1, mavutil.mavlink.MAV_CMD_USER_1, 0, 42, 0, 0, 0, 0, 0, 0)
        self.assertTrue(self.vehicle.wait_command(42), "the added output's GCS cannot reach the vehicle")
        self.assertIsNotNone(self.static.wait_vehicle())
        self.static.conn.mav.command_long_send(VEHICLE_SYSID, 1, mavutil.mavlink.MAV_CMD_USER_1, 0, 43, 0, 0, 0, 0, 0, 0)
        self.assertTrue(self.vehicle.wait_command(43), 'the --out GCS cannot reach the vehicle')
