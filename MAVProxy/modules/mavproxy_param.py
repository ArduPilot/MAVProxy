#!/usr/bin/env python
'''param command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.lib import mp_module

class ParamModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ParamModule, self).__init__(mpstate, "param", "parameter handling", public = True)
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.fetch_one = 0
        self.add_command('param', self.cmd_param, "parameter handling", ["<fetch|download>",
                                        "<set|show|help> (PARAMETER)",
                                        "<load|save> (FILENAME)"])
        if self.continue_mode and self.logdir != None:
            parmfile = os.path.join(self.logdir, 'mav.parm')
            if os.path.exists(parmfile):
                mpstate.mav_param.load(parmfile)
                    
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        state =self
        if m.get_type() == 'PARAM_VALUE':
            param_id = "%.16s" % m.param_id
            if m.param_index != -1 and m.param_index not in state.mav_param_set:
                added_new_parameter = True
                state.mav_param_set.add(m.param_index)
            else:
                added_new_parameter = False
            if m.param_count != -1:
                state.mav_param_count = m.param_count
            self.mav_param[str(param_id)] = m.param_value
            if state.fetch_one > 0:
                state.fetch_one -= 1
                self.console.writeln("%s = %f" % (param_id, m.param_value))
            if added_new_parameter and len(state.mav_param_set) == m.param_count:
                self.console.writeln("Received %u parameters" % m.param_count)
                if self.status.logdir != None:
                    self.mav_param.save(os.path.join(self.status.logdir, 'mav.parm'), '*', verbose=True)
    
    def idle_task(self):
        '''handle missing parameters'''
        state = self
        if state.param_period.trigger():
            if len(state.mav_param_set) == 0:
                self.master.param_fetch_all()
            elif state.mav_param_count != 0 and len(state.mav_param_set) != state.mav_param_count:
                if self.master.time_since('PARAM_VALUE') >= 1:
                    diff = set(range(state.mav_param_count)).difference(state.mav_param_set)
                    count = 0
                    while len(diff) > 0 and count < self.settings.parambatch:
                        idx = diff.pop()
                        self.master.param_fetch_one(idx)
                        count += 1
    
    def param_help_download(self):
        '''download XML files for parameters'''
        import multiprocessing
        files = []
        for vehicle in ['APMrover2', 'ArduCopter', 'ArduPlane']:
            url = 'http://autotest.diydrones.com/Parameters/%s/apm.pdef.xml' % vehicle
            path = mp_util.dot_mavproxy("%s.xml" % vehicle)
            files.append((url, path))
            url = 'http://autotest.diydrones.com/%s-defaults.parm' % vehicle
            path = mp_util.dot_mavproxy("%s-defaults.parm" % vehicle)
            files.append((url, path))
        try:
            child = multiprocessing.Process(target=mp_util.download_files, args=(files,))
            child.start()
        except Exception as e:
            print e
    
    def param_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return
        path = mp_util.dot_mavproxy("%s.xml" % self.vehicle_name)
        if not os.path.exists(path):
            print("Please run 'param download' first (vehicle_name=%s)" % self.vehicle_name)
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
    
    def cmd_param(self, args):
        '''control parameters'''
        state = self
        param_wildcard = "*"
        if len(args) < 1:
            print("usage: param <fetch|set|show|diff|download|help>")
            return
        if args[0] == "fetch":
            if len(args) == 1:
                self.master.param_fetch_all()
                state.mav_param_set = set()
                print("Requested parameter list")
            else:
                for p in self.mav_param.keys():
                    if fnmatch.fnmatch(p, args[1].upper()):
                        self.master.param_fetch_one(p)
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
            self.mav_param.save(args[1], param_wildcard, verbose=True)
        elif args[0] == "diff":
            wildcard = '*'
            if len(args) < 2 or args[1].find('*') != -1:
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
            if not param.upper() in self.mav_param:
                print("Unable to find parameter '%s'" % param)
                return
            self.mpstate.functions.param_set(param, value)
        elif args[0] == "load":
            if len(args) < 2:
                print("Usage: param load <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.load(args[1], param_wildcard, self.master)
        elif args[0] == "download":
            self.param_help_download()
        elif args[0] == "help":
            self.param_help(args[1:])
        elif args[0] == "show":
            if len(args) > 1:
                pattern = args[1]
            else:
                pattern = "*"
            self.mav_param.show(pattern)
        else:
            print("Unknown subcommand '%s' (try 'fetch', 'save', 'set', 'show', 'load', 'help')" % args[0])

def init(mpstate):
    '''initialise module'''
    return ParamModule(mpstate)