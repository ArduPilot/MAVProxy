#!/usr/bin/env python
'''param command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.lib import mp_module

class ParamState:
    '''this class is separated to make it possible to use the parameter
       functions on a secondary connection'''
    def __init__(self, mav_param, logdir, vehicle_name, parm_file):
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.fetch_one = dict()
        self.mav_param = mav_param
        self.logdir = logdir
        self.vehicle_name = vehicle_name
        self.parm_file = parm_file
        self.fetch_set = None

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'PARAM_VALUE':
            param_id = "%.16s" % m.param_id
            # Note: the xml specifies param_index is a uint16, so -1 in that field will show as 65535
            # We accept both -1 and 65535 as 'unknown index' to future proof us against someday having that
            # xml fixed.
            if self.fetch_set is not None:
                self.fetch_set.discard(m.param_index)
            if m.param_index != -1 and m.param_index != 65535 and m.param_index not in self.mav_param_set:
                added_new_parameter = True
                self.mav_param_set.add(m.param_index)
            else:
                added_new_parameter = False
            if m.param_count != -1:
                self.mav_param_count = m.param_count
            self.mav_param[str(param_id)] = m.param_value
            if param_id in self.fetch_one and self.fetch_one[param_id] > 0:
                self.fetch_one[param_id] -= 1
                print("%s = %f" % (param_id, m.param_value))
            if added_new_parameter and len(self.mav_param_set) == m.param_count:
                print("Received %u parameters" % m.param_count)
                if self.logdir != None:
                    self.mav_param.save(os.path.join(self.logdir, self.parm_file), '*', verbose=True)
                    self.fetch_set = None
            if self.fetch_set is not None and len(self.fetch_set) == 0:
                self.fetch_check(master, force=True)

    def fetch_check(self, master, force=False):
        '''check for missing parameters periodically'''
        if self.param_period.trigger() or force:
            if master is None:
                return
            if len(self.mav_param_set) == 0:
                master.param_fetch_all()
            elif self.mav_param_count != 0 and len(self.mav_param_set) != self.mav_param_count:
                if master.time_since('PARAM_VALUE') >= 1 or force:
                    diff = set(range(self.mav_param_count)).difference(self.mav_param_set)
                    count = 0
                    while len(diff) > 0 and count < 10:
                        idx = diff.pop()
                        master.param_fetch_one(idx)
                        if self.fetch_set is None:
                            self.fetch_set = set()
                        self.fetch_set.add(idx)
                        count += 1

    def param_help_download(self):
        '''download XML files for parameters'''
        import multiprocessing
        files = []
        for vehicle in ['APMrover2', 'ArduCopter', 'ArduPlane', 'ArduSub', 'AntennaTracker']:
            url = 'http://autotest.ardupilot.org/Parameters/%s/apm.pdef.xml' % vehicle
            path = mp_util.dot_mavproxy("%s.xml" % vehicle)
            files.append((url, path))
            url = 'http://autotest.ardupilot.org/%s-defaults.parm' % vehicle
            if vehicle != 'AntennaTracker':
                # defaults not generated for AntennaTracker ATM
                path = mp_util.dot_mavproxy("%s-defaults.parm" % vehicle)
                files.append((url, path))
        try:
            child = multiprocessing.Process(target=mp_util.download_files, args=(files,))
            child.start()
        except Exception as e:
            print(e)

    def param_help_tree(self):
        '''return a "help tree", a map between a parameter and its metadata.  May return None if help is not available'''
        if self.vehicle_name is None:
            print("Unknown vehicle type")
            return None
        path = mp_util.dot_mavproxy("%s.xml" % self.vehicle_name)
        if not os.path.exists(path):
            print("Please run 'param download' first (vehicle_name=%s)" % self.vehicle_name)
            return None
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
        return htree

    def param_apropos(self, args):
        '''search parameter help for a keyword, list those parameters'''
        if len(args) == 0:
            print("Usage: param apropos keyword")
            return

        htree = self.param_help_tree()
        if htree is None:
            return

        contains = {}
        for keyword in args:
            for param in htree.keys():
                if str(htree[param]).find(keyword) != -1:
                    contains[param] = True
        for param in contains.keys():
            print("%s" % (param,))

    def param_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return

        htree = self.param_help_tree()
        if htree is None:
            return

        for h in args:
            h = h.upper()
            if h in htree:
                help = htree[h]
                print("%s: %s\n" % (h, help.get('humanName')))
                print(help.get('documentation'))
                try:
                    print("\n")
                    for f in help.field:
                        print("%s : %s" % (f.get('name'), str(f)))
                except Exception as e:
                    pass
                try:
                    vchild = help.getchildren()[0]
                    print("\nValues: ")
                    for v in vchild.value:
                        print("\t%s : %s" % (v.get('code'), str(v)))
                except Exception as e:
                    pass
            else:
                print("Parameter '%s' not found in documentation" % h)

    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: param <fetch|save|set|show|load|preload|forceload|diff|download|help>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "fetch":
            if len(args) == 1:
                master.param_fetch_all()
                self.mav_param_set = set()
                print("Requested parameter list")
            else:
                for p in self.mav_param.keys():
                    if fnmatch.fnmatch(p, args[1].upper()):
                        master.param_fetch_one(p)
                        if p not in self.fetch_one:
                            self.fetch_one[p] = 0
                        self.fetch_one[p] += 1
                        print("Requested parameter %s" % p)
        elif args[0] == "save":
            if len(args) < 2:
                print("usage: param save <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.save(args[1], param_wildcard, verbose=True)
        elif args[0] == "diff":
            wildcard = '*'
            if len(args) < 2 or args[1].find('*') != -1:
                if self.vehicle_name is None:
                    print("Unknown vehicle type")
                    return
                filename = mp_util.dot_mavproxy("%s-defaults.parm" % self.vehicle_name)
                if not os.path.exists(filename):
                    print("Please run 'param download' first (vehicle_name=%s)" % self.vehicle_name)
                    return
                if len(args) >= 2:
                    wildcard = args[1]
            else:
                filename = args[1]
                if len(args) == 3:
                    wildcard = args[2]
            print("%-16.16s %12.12s %12.12s" % ('Parameter', 'Defaults', 'Current'))
            self.mav_param.diff(filename, wildcard=wildcard)
        elif args[0] == "set":
            if len(args) < 2:
                print("Usage: param set PARMNAME VALUE")
                return
            if len(args) == 2:
                self.mav_param.show(args[1])
                return
            param = args[1]
            value = args[2]
            if value.startswith('0x'):
                value = int(value, base=16)
            if not param.upper() in self.mav_param:
                print("Unable to find parameter '%s'" % param)
                return
            self.mav_param.mavset(master, param.upper(), value, retries=3)

            if (param.upper() == "WP_LOITER_RAD" or param.upper() == "LAND_BREAK_PATH"):
                #need to redraw rally points
                mpstate.module('rally').rallyloader.last_change = time.time()
                #need to redraw loiter points
                mpstate.module('wp').wploader.last_change = time.time()

        elif args[0] == "load":
            if len(args) < 2:
                print("Usage: param load <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.load(args[1], param_wildcard, master)
        elif args[0] == "preload":
            if len(args) < 2:
                print("Usage: param preload <filename>")
                return
            self.mav_param.load(args[1])
        elif args[0] == "forceload":
            if len(args) < 2:
                print("Usage: param forceload <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.load(args[1], param_wildcard, master, check=False)
        elif args[0] == "download":
            self.param_help_download()
        elif args[0] == "apropos":
            self.param_apropos(args[1:])
        elif args[0] == "help":
            self.param_help(args[1:])
        elif args[0] == "show":
            if len(args) > 1:
                pattern = args[1]
            else:
                pattern = "*"
            self.mav_param.show(pattern)
        elif args[0] == "status":
            print("Have %u/%u params" % (len(self.mav_param_set), self.mav_param_count))
        else:
            print(usage)


class ParamModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ParamModule, self).__init__(mpstate, "param", "parameter handling", public = True)
        self.pstate = ParamState(self.mav_param, self.logdir, self.vehicle_name, 'mav.parm')
        self.add_command('param', self.cmd_param, "parameter handling",
                         ["<download|status>",
                          "<set|show|fetch|help|apropos> (PARAMETER)",
                          "<load|save|diff> (FILENAME)"])
        if self.continue_mode and self.logdir != None:
            parmfile = os.path.join(self.logdir, 'mav.parm')
            if os.path.exists(parmfile):
                mpstate.mav_param.load(parmfile)
                self.pstate.mav_param_set = set(self.mav_param.keys())

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        self.pstate.handle_mavlink_packet(self.master, m)

    def idle_task(self):
        '''handle missing parameters'''
        self.pstate.vehicle_name = self.vehicle_name
        self.pstate.fetch_check(self.master)

    def cmd_param(self, args):
        '''control parameters'''
        self.pstate.handle_command(self.master, self.mpstate, args)

def init(mpstate):
    '''initialise module'''
    return ParamModule(mpstate)
