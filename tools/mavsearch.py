#!/usr/bin/env python

'''
search a set of log files for a condition
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'modules'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'mavlink', 'pymavlink'))

import mavutil

from optparse import OptionParser
parser = OptionParser("mavsearch.py [options]")
parser.add_option("--condition", default=None, help="conditional check on log")
parser.add_option("--types", default=None, help="message types to look for (comma separated)")
parser.add_option("--stop", action='store_true', help="stop when message type found")
parser.add_option("--stopcondition", action='store_true', help="stop when condition met")

(opts, args) = parser.parse_args()

def mavsearch(filename):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    if opts.types is not None:
        types = opts.types.split(',')
    else:
        types = None
    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break
        if mlog.check_condition(opts.condition):
            print m
            if opts.stopcondition:
                break
        if opts.stop:
            break

if len(args) < 1:
    print("Usage: mavsearch.py [options] <LOGFILE...>")
    sys.exit(1)

for f in args:
    mavsearch(f)
