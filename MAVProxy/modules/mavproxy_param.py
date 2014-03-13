#!/usr/bin/env python
'''param command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util

class param_state(object):
    def __init__(self):
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.fetch_one = 0

def name():
    '''return module name'''
    return "param"

def description():
    '''return module description'''
    return "parameter handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.param_state = param_state()
    mpstate.command_map['param'] = (cmd_param, "parameter handling")
    mpstate.completions['param'] = ["<fetch|download>",
                                    "<set|show|help> (PARAMETER)",
                                    "<load|save> (FILENAME)"]
    if mpstate.continue_mode and mpstate.status.logdir != None:
        parmfile = os.path.join(mpstate.status.logdir, 'mav.parm')
        if os.path.exists(parmfile):
            mpstate.mav_param.load(parmfile)

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.param_state
    if m.get_type() == 'PARAM_VALUE':
        import random
        param_id = "%.16s" % m.param_id
        if m.param_index != -1 and m.param_index not in state.mav_param_set:
            added_new_parameter = True
            state.mav_param_set.add(m.param_index)
        else:
            added_new_parameter = False
        if m.param_count != -1:
            state.mav_param_count = m.param_count
        mpstate.mav_param[str(param_id)] = m.param_value
        if state.fetch_one > 0:
            state.fetch_one -= 1
            mpstate.console.writeln("%s = %f" % (param_id, m.param_value))
        if added_new_parameter and len(state.mav_param_set) == m.param_count:
            mpstate.console.writeln("Received %u parameters" % m.param_count)
            if mpstate.status.logdir != None:
                mpstate.mav_param.save(os.path.join(mpstate.status.logdir, 'mav.parm'), '*', verbose=True)

def idle_task():
    '''handle missing parameters'''
    state = mpstate.param_state
    if state.param_period.trigger():
        if len(state.mav_param_set) == 0:
            mpstate.master().param_fetch_all()
        elif state.mav_param_count != 0 and len(state.mav_param_set) != state.mav_param_count:
            if mpstate.master().time_since('PARAM_VALUE') >= 1:
                diff = set(range(state.mav_param_count)).difference(state.mav_param_set)
                count = 0
                while len(diff) > 0 and count < mpstate.settings.parambatch:
                    idx = diff.pop()
                    mpstate.master().param_fetch_one(idx)
                    count += 1

def param_help_download():
    '''download XML files for parameters'''
    for vehicle in ['APMrover2', 'ArduCopter', 'ArduPlane']:
        url = 'http://autotest.diydrones.com/Parameters/%s/apm.pdef.xml' % vehicle
        path = mp_util.dot_mavproxy("%s.xml" % vehicle)
        print("Downloading %s to %s" % (url, path))
        xml = mp_util.download_url(url)
        try:
            open(path, mode='w').write(xml)
        except Exception as e:
            print("Failed to save %s" % path)

def param_help(args):
    '''show help on a parameter'''
    if len(args) == 0:
        print("Usage: param help PARAMETER_NAME")
        return
    vehicle_map = {
        'plane' : 'ArduPlane',
        'copter' : 'ArduCopter',
        'rover' : 'APMrover2'
        }
    if not mpstate.vehicle_type in vehicle_map:
        print("Unknown vehicle type %s" % str(mpstate.vehicle_type))
        return
    path = mp_util.dot_mavproxy("%s.xml" % vehicle_map[mpstate.vehicle_type])
    if not os.path.exists(path):
        print("Please run 'param download' first")
        return
    xml = open(path).read()
    from lxml import objectify
    objectify.enable_recursive_str()
    tree = objectify.fromstring(xml)
    htree = {}
    for p in tree.vehicles.parameters.param:
        n = p.get('name').split(':')[1]
        htree[n] = p
    for lib in tree.libraries.parameters:
        for p in lib.param:
            n = p.get('name')
            htree[n] = p
    for h in args:
        if h in htree:
            help = htree[h]
            print("%s: %s\n" % (h, help.get('humanName')))
            print(help.get('documentation'))
            try:
                vchild = help.getchildren()[0]
                print("\nValues: ")
                for v in vchild.value:
                    print("\t%s : %s" % (v.get('code'), str(v)))
            except Exception as e:
                pass
        else:
            print("Parameter '%s' not found in documentation" % h)

def cmd_param(args):
    '''control parameters'''
    state = mpstate.param_state
    param_wildcard = "*"
    if len(args) < 1:
        print("usage: param <fetch|set|show|diff|download|help>")
        return
    if args[0] == "fetch":
        if len(args) == 1:
            mpstate.master().param_fetch_all()
            state.mav_param_set = set()
            print("Requested parameter list")
        else:
            for p in mpstate.mav_param.keys():
                if fnmatch.fnmatch(p, args[1].upper()):
                    mpstate.master().param_fetch_one(p)
                    state.fetch_one += 1
                    print("Requested parameter %s" % p)
    elif args[0] == "save":
        if len(args) < 2:
            print("usage: param save <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.save(args[1], param_wildcard, verbose=True)
    elif args[0] == "diff":
        if len(args) < 2:
            if opts.aircraft is not None:
                filename = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(mpstate.status.logdir))), 'mavdefault.txt')
            else:
                print("Usage: param diff <filename>")
        else:
            filename = args[1]
        if len(args) == 3:
            wildcard = args[2]
        else:
            wildcard = '*'
        mpstate.mav_param.diff(filename, wildcard=wildcard)
    elif args[0] == "set":
        if len(args) != 3:
            print("Usage: param set PARMNAME VALUE")
            return
        param = args[1]
        value = args[2]
        if not param.upper() in mpstate.mav_param:
            print("Unable to find parameter '%s'" % param)
            return
        mpstate.functions.param_set(param, value)
    elif args[0] == "load":
        if len(args) < 2:
            print("Usage: param load <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.load(args[1], param_wildcard, mpstate.master())
    elif args[0] == "download":
        param_help_download()
    elif args[0] == "help":
        param_help(args[1:])
    elif args[0] == "show":
        if len(args) > 1:
            pattern = args[1]
        else:
            pattern = "*"
        mpstate.mav_param.show(pattern)
    else:
        print("Unknown subcommand '%s' (try 'fetch', 'save', 'set', 'show', 'load', 'help')" % args[0])

