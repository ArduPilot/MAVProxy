import time, os
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc

class ParamHelp:
    def __init__(self):
        self.xml_filepath = None
        self.vehicle_name = None
        self.last_pair = (None,None)
        self.last_htree = None

    def param_help_download(self):
        '''download XML files for parameters'''
        files = []
        for vehicle in ['Rover', 'ArduCopter', 'ArduPlane', 'ArduSub', 'AntennaTracker', 'Blimp']:
            url = 'http://autotest.ardupilot.org/Parameters/%s/apm.pdef.xml.gz' % vehicle
            path = mp_util.dot_mavproxy("%s.xml" % vehicle)
            files.append((url, path))
            url = 'http://autotest.ardupilot.org/%s-defaults.parm' % vehicle
            if vehicle != 'AntennaTracker':
                # defaults not generated for AntennaTracker ATM
                path = mp_util.dot_mavproxy("%s-defaults.parm" % vehicle)
                files.append((url, path))
        try:
            child = multiproc.Process(target=mp_util.download_files, args=(files,))
            child.start()
        except Exception as e:
            print(e)

    def param_use_xml_filepath(self, filepath):
        self.xml_filepath = filepath

    def param_help_tree(self, verbose=False):
        '''return a "help tree", a map between a parameter and its metadata.  May return None if help is not available'''
        if self.last_pair == (self.xml_filepath, self.vehicle_name):
            return self.last_htree
        if self.xml_filepath is not None:
            if verbose:
                print("param: using xml_filepath=%s" % self.xml_filepath)
            path = self.xml_filepath
        else:
            if self.vehicle_name is None:
                if verbose:
                    print("Unknown vehicle type")
                return None
            path = mp_util.dot_mavproxy("%s.xml" % self.vehicle_name)
            if not os.path.exists(path):
                if self.vehicle_name == 'APMrover2':
                    path = mp_util.dot_mavproxy("%s.xml" % "Rover")
            if not os.path.exists(path):
                if verbose:
                    print("Please run 'param download' first (vehicle_name=%s)" % self.vehicle_name)
                return None
        if not os.path.exists(path):
            if verbose:
                print("Param XML (%s) does not exist" % path)
            return None
        xml = open(path,'rb').read()
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
        self.last_htree = htree
        self.last_pair = (self.xml_filepath, self.vehicle_name)
        return htree

    def param_set_xml_filepath(self, args):
        self.xml_filepath = args[0]

    def param_apropos(self, args):
        '''search parameter help for a keyword, list those parameters'''
        if len(args) == 0:
            print("Usage: param apropos keyword")
            return

        htree = self.param_help_tree(True)
        if htree is None:
            return

        contains = {}
        for keyword in args:
            keyword = keyword.lower()
            for param in htree.keys():
                if str(htree[param]).lower().find(keyword) != -1:
                    contains[param] = True
        for param in contains.keys():
            print("%s" % (param,))

    def get_Values_from_help(self, help):
        children = help.getchildren()
        if len(children) == 0:
            return []
        vchild = children[0]
        return vchild.getchildren()

    def get_bitmask_from_help(self, help):
        if not hasattr(help, 'field'):
            return None
        field = help.field
        if not hasattr(field, 'attrib'):
            return None
        if field.attrib.get('name',None) != 'Bitmask':
            return None
        a = str(field).split(',')
        ret = {}
        for v in a:
            a2 = v.split(':')
            if len(a2) == 2:
                ret[a2[0]] = a2[1]
        return ret

    def param_info(self, param, value):
        '''return info string for a param value'''
        htree = self.param_help_tree()
        if htree is None:
            return
        param = param.upper()
        if not param in htree:
            return None
        help = htree[param]
        try:
            bitmask = self.get_bitmask_from_help(help)
            if bitmask is not None:
                v = []
                for k in bitmask.keys():
                    if int(value) & (1<<int(k)):
                        v.append(bitmask[k])
                return '|'.join(v)
        except Exception as e:
            print(e)
            pass
        try:
            values = self.get_Values_from_help(help)
            for v in values:
                if int(v.get('code')) == int(value):
                    return v
        except Exception as e:
            pass
        return None

    def param_help(self, args):
        '''show help on a parameter'''
        if len(args) == 0:
            print("Usage: param help PARAMETER_NAME")
            return

        htree = self.param_help_tree(True)
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
                    values = self.get_Values_from_help(help)
                    if len(values):
                        print("\nValues: ")
                        for v in values:
                            print("\t%s : %s" % (v.get('code'), str(v)))
                except Exception as e:
                    print("Caught exception %s" % repr(e))
                    pass
            else:
                print("Parameter '%s' not found in documentation" % h)
            
    def param_check(self, params, args):
        '''Check through parameters for obvious misconfigurations'''
        problems_found = False
        htree = self.param_help_tree(True)
        if htree is None:
            return
        for param in params.keys():
            if param.startswith("SIM_"):
                # no documentation for these ATM
                continue
            value = params[param]
#            print("%s: %s" % (param, str(value)))
            try:
                help = htree[param]
            except KeyError:
                print("%s: not found in documentation" % (param,))
                problems_found = True
                continue

            # we'll ignore the Values field if there's a bitmask field
            # involved as they're usually just examples.
            has_bitmask = False
            for f in getattr(help, "field", []):
                if f.get('name') == "Bitmask":
                    has_bitmask = True
                    break
            if not has_bitmask:
                values = self.get_Values_from_help(help)
                if len(values) == 0:
                    # no prescribed values list
                    continue
                value_values = [float(x.get("code")) for x in values]
                if value not in value_values:
                    print("%s: value %f not in Values (%s)" %
                          (param, value, str(value_values)))
                    problems_found = True

        if problems_found:
            print("Remember to `param download` before trusting the checking!  Also, remember that parameter documentation is for *master*!")

