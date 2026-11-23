#!/usr/bin/env python3
'''
example GCS client for MAVProxy's outputctl module (see outputctl.md)

    python outputctl_client.py ADD 192.168.2.117 14550
    python outputctl_client.py REMOVE 192.168.2.117 14550
    python outputctl_client.py LIST
    python outputctl_client.py CLEAR
    python outputctl_client.py --host 192.168.2.10 --port 14600 LIST

or from a GCS:

    client = OutputCtlClient('127.0.0.1', 14600)
    client.add('192.168.2.117', 14550)
    for (ip, port, status) in client.list():
        ...
    client.remove('192.168.2.117', 14550)
    client.clear()

Needs only the Python standard library.

AP_FLAKE8_CLEAN
'''

import argparse
import socket
import sys


class OutputCtlError(Exception):
    '''an ERR reply, or no reply at all'''


class OutputCtlClient(object):
    def __init__(self, host='127.0.0.1', port=14600, timeout=1.0, retries=3, source=None):
        '''host and port of MAVProxy's control socket. source is the local
        address to send from, for when MAVProxy's allow setting names one
        address of a machine which has several'''
        self.address = (host, port)
        self.timeout = timeout
        self.retries = retries
        self.source = source

    def request(self, text):
        '''send one request and return the lines of its OK reply. Raises
        OutputCtlError for an ERR reply, or when there is no reply'''
        for attempt in range(self.retries):
            # a fresh socket each attempt, so a late reply to an earlier
            # attempt cannot be taken for the reply to this one
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                if self.source is not None:
                    sock.bind((self.source, 0))
                sock.settimeout(self.timeout)
                sock.sendto(text.encode('ascii'), self.address)
                reply = sock.recvfrom(65535)[0].decode('ascii', 'replace')
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows: nothing is listening on the control port
                continue
            finally:
                sock.close()
            lines = reply.split('\n')
            if lines[0].startswith('OK '):
                return lines
            if lines[0].startswith('ERR '):
                raise OutputCtlError(lines[0][4:], attempt)
            raise OutputCtlError('unexpected reply: %r' % reply, attempt)
        raise OutputCtlError('no reply from %s:%u' % self.address, self.retries)

    def add(self, ip, port):
        '''add an output. An "already exists" answer to a retry means an
        earlier attempt worked and its reply was lost'''
        destination = '%s:%u' % (ip, port)
        try:
            return self.request('ADD %s %u' % (ip, port))[0]
        except OutputCtlError as ex:
            (message, attempt) = ex.args
            if attempt > 0 and message == 'Destination already exists: ' + destination:
                return 'OK Added ' + destination
            raise

    def remove(self, ip, port):
        '''remove an added output. A "not found" answer to a retry means an
        earlier attempt worked and its reply was lost'''
        destination = '%s:%u' % (ip, port)
        try:
            return self.request('REMOVE %s %u' % (ip, port))[0]
        except OutputCtlError as ex:
            (message, attempt) = ex.args
            if attempt > 0 and message == 'Not found: ' + destination:
                return 'OK Removed ' + destination
            raise

    def list(self):
        '''the added outputs, as (ip, port, status) tuples'''
        lines = self.request('LIST')
        outputs = []
        for line in lines[1:]:
            (destination, status) = line.split(' ', 1)
            (ip, port) = destination.rsplit(':', 1)
            outputs.append((ip, int(port), status))
        return outputs

    def clear(self):
        '''remove all the added outputs, returning how many there were'''
        # "OK Cleared <n> outputs"
        return int(self.request('CLEAR')[0].split()[2])


def main():
    parser = argparse.ArgumentParser(description="control MAVProxy's runtime UDP outputs")
    parser.add_argument('--host', default='127.0.0.1', help="MAVProxy's control address")
    parser.add_argument('--port', type=int, default=14600, help="MAVProxy's control port")
    parser.add_argument('--source', default=None, help='local address to send from')
    parser.add_argument('--timeout', type=float, default=1.0, help='seconds to wait for each reply')
    parser.add_argument('command', choices=['ADD', 'REMOVE', 'LIST', 'CLEAR', 'add', 'remove', 'list', 'clear'])
    parser.add_argument('ip', nargs='?')
    parser.add_argument('dport', nargs='?', type=int, metavar='port')
    args = parser.parse_args()
    client = OutputCtlClient(args.host, args.port, timeout=args.timeout, source=args.source)
    command = args.command.upper()
    try:
        if command in ('ADD', 'REMOVE'):
            if args.ip is None or args.dport is None:
                parser.error('%s needs an ip and a port' % command)
            print(client.add(args.ip, args.dport) if command == 'ADD' else client.remove(args.ip, args.dport))
        elif command == 'LIST':
            outputs = client.list()
            print('%u outputs' % len(outputs))
            for (ip, port, status) in outputs:
                print('  %s:%u %s' % (ip, port, status))
        else:
            print('cleared %u outputs' % client.clear())
    except OutputCtlError as ex:
        print('ERR %s' % ex.args[0])
        sys.exit(1)


if __name__ == '__main__':
    main()
