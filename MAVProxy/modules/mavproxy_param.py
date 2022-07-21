#!/usr/bin/env python
'''param command handling'''

import time, os, fnmatch, struct, sys
from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import param_help
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

try:
    # py2
    from StringIO import StringIO as SIO
except ImportError:
    # py3
    from io import BytesIO as SIO
    
class ParamState:
    '''this class is separated to make it possible to use the parameter
       functions on a secondary connection'''
    def __init__(self, mav_param, logdir, vehicle_name, parm_file, mpstate, sysid):
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.fetch_one = dict()
        self.mav_param = mav_param
        self.logdir = logdir
        self.vehicle_name = vehicle_name
        self.parm_file = parm_file
        self.fetch_set = None
        self.new_sysid_timestamp = time.time()
        self.autopilot_type_by_sysid = {}
        self.param_types = {}
        self.ftp_failed = False
        self.ftp_started = False
        self.ftp_count = None
        self.ftp_send_param = None
        self.mpstate = mpstate
        self.sysid = sysid
        self.param_help = param_help.ParamHelp()
        self.param_help.vehicle_name = vehicle_name

    def use_ftp(self):
        '''return true if we should try ftp for download'''
        if self.ftp_failed:
            return False
        return self.mpstate.settings.param_ftp

    def handle_px4_param_value(self, m):
        '''special handling for the px4 style of PARAM_VALUE'''
        if m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            # already right type
            return m.param_value
        is_px4_params = False
        if m.get_srcComponent() in [mavutil.mavlink.MAV_COMP_ID_UDP_BRIDGE]:
            # ESP8266 uses PX4 style parameters
            is_px4_params = True
        sysid = m.get_srcSystem()
        if self.autopilot_type_by_sysid.get(sysid,-1) in [mavutil.mavlink.MAV_AUTOPILOT_PX4]:
            is_px4_params = True
        if not is_px4_params:
            return m.param_value
        # try to extract px4 param value
        value = m.param_value
        try:
            v = struct.pack(">f", value)
        except Exception:
            return value
        if m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            value, = struct.unpack(">B", v[3:])
        elif m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            value, = struct.unpack(">b", v[3:])
        elif m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            value, = struct.unpack(">H", v[2:])
        elif m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            value, = struct.unpack(">h", v[2:])
        elif m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            value, = struct.unpack(">I", v[0:])
        elif m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            value, = struct.unpack(">i", v[0:])
        # can't pack other types

        # remember type for param set
        self.param_types[m.param_id.upper()] = m.param_type
        return value

    def handle_mavlink_packet(self, master, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'PARAM_VALUE':
            value = self.handle_px4_param_value(m)
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
            self.mav_param[str(param_id)] = value
            if param_id in self.fetch_one and self.fetch_one[param_id] > 0:
                self.fetch_one[param_id] -= 1
                if isinstance(value, float):
                    print("%s = %.7f" % (param_id, value))
                else:
                    print("%s = %s" % (param_id, str(value)))
            if added_new_parameter and len(self.mav_param_set) == m.param_count:
                print("Received %u parameters" % m.param_count)
                if self.logdir is not None:
                    self.mav_param.save(os.path.join(self.logdir, self.parm_file), '*', verbose=True)
                self.fetch_set = None
            if self.fetch_set is not None and len(self.fetch_set) == 0:
                self.fetch_check(master, force=True)
        elif m.get_type() == 'HEARTBEAT':
            if m.get_srcComponent() == 1:
                # remember autopilot types so we can handle PX4 parameters
                self.autopilot_type_by_sysid[m.get_srcSystem()] = m.autopilot

    def fetch_check(self, master, force=False):
        '''check for missing parameters periodically'''
        if self.param_period.trigger() or force:
            if master is None:
                return
            if len(self.mav_param_set) == 0 and not self.ftp_started:
                if not self.use_ftp():
                    master.param_fetch_all()
                else:
                    self.ftp_start()
            elif not self.ftp_started and self.mav_param_count != 0 and len(self.mav_param_set) != self.mav_param_count:
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

    def param_use_xml_filepath(self, filepath):
        self.param_help.xml_filepath = filepath

    def param_set_xml_filepath(self, args):
        self.param_use_xml_filepath(args[0])

    def status(self, master, mpstate):
        return(len(self.mav_param_set), self.mav_param_count)

    def ftp_start(self):
        '''start a ftp download of parameters'''
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            self.ftp_failed = True
            self.ftp_started = False
            return
        self.ftp_started = True
        self.ftp_count = None
        ftp.cmd_get(["@PARAM/param.pck?withdefaults=1"], callback=self.ftp_callback, callback_progress=self.ftp_callback_progress)

    def log_params(self, params):
        '''log PARAM_VALUE messages so that we can extract parameters from a tlog when using ftp download'''
        if not self.mpstate.logqueue:
            return
        usec = int(time.time() * 1.0e6)
        idx = 0
        mav = self.mpstate.master().mav
        editor = self.mpstate.module('paramedit')
        for (name, v, ptype) in params:
            p = mavutil.mavlink.MAVLink_param_value_message(name,
                                                            float(v),
                                                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                                                            len(params),
                                                            idx)
            idx += 1
            # log PARAM_VALUE using the source vehicles sysid but MAV_COMP_ID_MISSIONPLANNER, to allow
            # us to tell that it came from the GCS, but to not corrupt the sequence numbers
            id_saved = (mav.srcSystem, mav.srcComponent)
            mav.srcSystem = self.sysid[0]
            mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
            try:
                buf = p.pack(mav)
                self.mpstate.logqueue.put(bytearray(struct.pack('>Q', usec) + buf))
                # also give to param editor so it can update for changes
                if editor:
                    editor.mavlink_packet(p)
            except Exception:
                pass
            (mav.srcSystem, mav.srcComponent) = id_saved

    def ftp_callback_progress(self, fh, total_size):
        '''callback as read progresses'''
        if self.ftp_count is None and total_size >= 6:
            ofs = fh.tell()
            fh.seek(0)
            buf = fh.read(6)
            fh.seek(ofs)
            magic2,num_params,total_params = struct.unpack("<HHH", buf)
            if magic2 == 0x671b or magic2 == 0x671c:
                self.ftp_count = total_params
        # approximate count
        if self.ftp_count is not None:
            # each entry takes 11 bytes on average
            per_entry_size = 11
            done = min(int(total_size / per_entry_size), self.ftp_count-1)
            self.mpstate.console.set_status('Params', 'Param %u/%u' % (done, self.ftp_count))

    def ftp_callback(self, fh):
        '''callback from ftp fetch of parameters'''
        self.ftp_started = False
        if fh is None:
            # the fetch failed
            self.ftp_failed = True
            return

        magic = 0x671b
        magic_defaults = 0x671c
        data = fh.read()
        magic2,num_params,total_params = struct.unpack("<HHH", data[0:6])
        if magic != magic2 and magic_defaults != magic2:
            print("paramftp: bad magic 0x%x expected 0x%x" % (magic2, magic))
            return
        with_defaults = magic2 == magic_defaults
        data = data[6:]

        # mapping of data type to type length and format
        data_types = {
            1: (1, 'b'),
            2: (2, 'h'),
            3: (4, 'i'),
            4: (4, 'f'),
        }

        count = 0
        params = []
        if with_defaults:
            defaults = []

        if sys.version_info.major < 3:
            pad_byte = chr(0)
            last_name = ''
        else:
            pad_byte = 0
            last_name = bytes()

        while True:
            while len(data) > 0 and data[0] == pad_byte:
                # skip pad bytes
                data = data[1:]

            if len(data) == 0:
                break

            ptype, plen = struct.unpack("<BB", data[0:2])
            flags = (ptype>>4) & 0x0F
            has_default = with_defaults and (flags&1) != 0
            ptype &= 0x0F


            if not ptype in data_types:
                print("paramftp: bad type 0x%x" % ptype)
                return

            (type_len, type_format) = data_types[ptype]
            default_len = type_len if has_default else 0

            name_len = ((plen>>4) & 0x0F) + 1
            common_len = (plen & 0x0F)
            name = last_name[0:common_len] + data[2:2+name_len]
            vdata = data[2+name_len:2+name_len+type_len+default_len]
            last_name = name
            data = data[2+name_len+type_len+default_len:]
            if with_defaults:
                if has_default:
                    v1,v2, = struct.unpack("<" + type_format + type_format, vdata)
                    params.append((name, v1, ptype))
                    defaults.append((name, v2, ptype))
                else:
                    v, = struct.unpack("<" + type_format, vdata)
                    params.append((name, v, ptype))
                    defaults.append((name, v, ptype))
            else:
                v, = struct.unpack("<" + type_format, vdata)
                params.append((name, v, ptype))
            count += 1

        if count != total_params:
            print("paramftp: bad count %u should be %u" % (count, total_params))
            return
        self.param_types = {}
        self.mav_param_set = set()
        self.fetch_one = dict()
        self.fetch_set = None
        self.mav_param.clear()
        self.mav_param_count = total_params

        idx = 0
        for (name, v, ptype) in params:
            # we need to set it to REAL32 to ensure we use write value for param_set
            name = str(name.decode('utf-8'))
            self.param_types[name] = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            self.mav_param_set.add(idx)
            self.mav_param[name] = v
            idx += 1

        self.ftp_failed = False
        self.mpstate.console.set_status('Params', 'Param %u/%u' % (total_params, total_params))
        print("Received %u parameters (ftp)" % total_params)
        if self.logdir is not None:
            self.mav_param.save(os.path.join(self.logdir, self.parm_file), '*', verbose=True)
        self.log_params(params)

        if with_defaults:
            defaults_path = os.path.join(self.logdir, "defaults.parm") if self.logdir else "defaults.parm"
            defparm = mavparm.MAVParmDict()
            for (name, v, ptype) in defaults:
                name = str(name.decode('utf-8'))
                defparm[name] = v
            defparm.save(defaults_path, '*', verbose=False)
            print("Saved %u defaults to %s" % (len(defaults), defaults_path))



    def fetch_all(self, master):
        '''force refetch of parameters'''
        if not self.use_ftp():
            master.param_fetch_all()
            self.mav_param_set = set()
        else:
            self.ftp_start()

    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: param <fetch|ftp|save|set|show|load|preload|forceload|ftpload|diff|download|check|help>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "fetch":
            if len(args) == 1:
                self.fetch_all(master)
                if self.ftp_started:
                    print("Requested parameter list (ftp)")
                else:
                    print("Requested parameter list")
            else:
                found = False
                pname = args[1].upper()
                for p in self.mav_param.keys():
                    if fnmatch.fnmatch(p, pname):
                        master.param_fetch_one(p)
                        if p not in self.fetch_one:
                            self.fetch_one[p] = 0
                        self.fetch_one[p] += 1
                        found = True
                        print("Requested parameter %s" % p)
                if not found and args[1].find('*') == -1:
                    master.param_fetch_one(pname)
                    if pname not in self.fetch_one:
                        self.fetch_one[pname] = 0
                    self.fetch_one[pname] += 1
                    print("Requested parameter %s" % pname)
        elif args[0] == "ftp":
            self.ftp_start()

        elif args[0] == "save":
            if len(args) < 2:
                print("usage: param save <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.save(args[1].strip('"'), param_wildcard, verbose=True)
        elif args[0] == "diff":
            wildcard = '*'
            if len(args) < 2 or args[1].find('*') != -1:
                filename = os.path.join(self.logdir, "defaults.parm")
                if not os.path.exists(filename):
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
            print("defaults path: %s" % filename)
            print("%-16.16s %12.12s %12.12s" % ('Parameter', 'Defaults', 'Current'))
            self.mav_param.diff(filename, wildcard=wildcard)
        elif args[0] == "set":
            if len(args) < 2:
                print("Usage: param set PARMNAME VALUE")
                return
            if len(args) == 2:
                self.param_show(args[1], self.mpstate.settings.param_docs)
                return
            param = args[1]
            value = args[2]
            if value.startswith('0x'):
                value = int(value, base=16)
            if not param.upper() in self.mav_param:
                print("Unable to find parameter '%s'" % param)
                return
            uname = param.upper()
            ptype = None
            if uname in self.param_types:
                ptype = self.param_types[uname]
            self.mav_param.mavset(master, uname, value, retries=3, parm_type=ptype)

            if (param.upper() == "WP_LOITER_RAD" or param.upper() == "LAND_BREAK_PATH"):
                #need to redraw rally points
                mpstate.module('rally').set_last_change(time.time())
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
            self.mav_param.load(args[1].strip('"'), param_wildcard, master)
        elif args[0] == "preload":
            if len(args) < 2:
                print("Usage: param preload <filename>")
                return
            self.mav_param.load(args[1].strip('"'))
        elif args[0] == "forceload":
            if len(args) < 2:
                print("Usage: param forceload <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.mav_param.load(args[1].strip('"'), param_wildcard, master, check=False)
        elif args[0] == "ftpload":
            if len(args) < 2:
                print("Usage: param ftpload <filename> [wildcard]")
                return
            if len(args) > 2:
                param_wildcard = args[2]
            else:
                param_wildcard = "*"
            self.ftp_load(args[1].strip('"'), param_wildcard, master)
        elif args[0] == "download":
            self.param_help.param_help_download()
        elif args[0] == "apropos":
            self.param_help.param_apropos(args[1:])
        elif args[0] == "check":
            self.param_help.param_check(self.mav_param, args[1:])
        elif args[0] == "help":
            self.param_help.param_help(args[1:])
        elif args[0] == "set_xml_filepath":
            self.param_set_xml_filepath(args[1:])
        elif args[0] == "show":
            verbose = self.mpstate.settings.param_docs
            if len(args) > 1:
                pattern = args[1]
                if len(args) > 2 and args[2] == '-v':
                    verbose = True
            else:
                pattern = "*"
            self.param_show(pattern, verbose)
        elif args[0] == "status":
            print("Have %u/%u params" % (len(self.mav_param_set), self.mav_param_count))
        else:
            print(usage)

    def param_show(self, pattern, verbose):
        '''show parameters'''
        k = sorted(self.mav_param.keys())
        for p in k:
            name = str(p).upper()
            if fnmatch.fnmatch(name, pattern.upper()):
                value = self.mav_param.get(name)
                s = "%-16.16s %s" % (name, value)
                if verbose:
                    info = self.param_help.param_info(name, value)
                    if info is not None:
                        s = "%-28.28s # %s" % (s, info)
                print(s)

    def ftp_upload_callback(self, dlen):
        '''callback on ftp put completion'''
        if dlen is None:
            print("Failed to send parameters")
        else:
            if self.ftp_send_param is not None:
                for k in sorted(self.ftp_send_param.keys()):
                    v = self.ftp_send_param.get(k)
                    self.mav_param[k] = v
                self.ftp_send_param = None
            print("Parameter upload done")

    def ftp_upload_progress(self, proportion):
        '''callback from ftp put of parameters'''
        if proportion is None:
            self.mpstate.console.set_status('Params', 'Params ERR')
        else:
            self.mpstate.console.set_status('Params', 'Param %.1f%%' % (100.0*proportion))

    def best_type(self, v):
        '''work out best type for a variable'''
        if not v.is_integer():
            # treat as float
            return 4
        if v >= -128 and v <= 127:
            return 1
        if v >= -32768 and v <= 32767:
            return 2
        return 3

    def str_common_len(self, s1, s2):
        '''return common length between two strings'''
        c = min(len(s1),len(s2))
        for i in range(c):
            if s1[i] != s2[i]:
                return i
        return c

    def ftp_load(self, filename, param_wildcard, master):
        '''load parameters with ftp'''
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            print("Need ftp module")
            return
        newparm = mavparm.MAVParmDict()
        newparm.load(filename, param_wildcard, check=False)
        fh = SIO()
        for k in sorted(newparm.keys()):
            v = newparm.get(k)
            oldv = self.mav_param.get(k, None)
            if oldv is not None and abs(oldv - v) <= newparm.mindelta:
                # not changed
                newparm.pop(k)
        count = len(newparm.keys())
        if count == 0:
            print("No parameter changes")
            return

        fh.write(struct.pack("<HHH", 0x671b, count, count))
        last_param = ""
        for k in sorted(newparm.keys()):
            v = newparm.get(k)
            vtype = self.best_type(v)
            common_len = self.str_common_len(last_param, k)
            b1 = vtype
            b2 = common_len | ((len(k)-(common_len+1))<<4)
            fh.write(struct.pack("<BB", b1, b2))
            fh.write(k[common_len:].encode("UTF-8"))
            if vtype == 1: # int8
                fh.write(struct.pack("<b", int(v)))
            if vtype == 2: # int16
                fh.write(struct.pack("<h", int(v)))
            if vtype == 3: # int32
                fh.write(struct.pack("<i", int(v)))
            if vtype == 4: # float
                fh.write(struct.pack("<f", v))
            last_param = k

        # re-pack with full file length in total_params
        file_len = fh.tell()
        fh.seek(0)
        fh.write(struct.pack("<HHH", 0x671b, count, file_len))
        fh.seek(0)
        self.ftp_send_param = newparm
        print("Sending %u params" % count)
        ftp.cmd_put(["-", "@PARAM/param.pck"],
                    fh=fh, callback=self.ftp_upload_callback, progress_callback=self.ftp_upload_progress)


class ParamModule(mp_module.MPModule):
    def __init__(self, mpstate, **kwargs):
        super(ParamModule, self).__init__(mpstate, "param", "parameter handling", public = True, multi_vehicle=True)
        self.xml_filepath = kwargs.get("xml-filepath", None)
        self.pstate = {}
        self.check_new_target_system()
        self.menu_added_console = False
        self.add_command('param', self.cmd_param, "parameter handling",
                         ["<download|status>",
                          "<set|show|fetch|ftp|help|apropos> (PARAMETER)",
                          "<load|save|diff|forceload|ftpload> (FILENAME)",
                          "<set_xml_filepath> (FILEPATH)"
                         ])
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Parameter',
                                  items=[MPMenuItem('Editor', 'Editor', '# module load paramedit'),
                                         MPMenuItem('Fetch', 'Fetch', '# param fetch'),
                                         MPMenuItem('Load', 'Load', '# param load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Param Load',
                                                                                 wildcard='ParmFiles(*.parm,*.param)|*.parm;*.param')),
                                         MPMenuItem('Save', 'Save', '# param save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Param Save',
                                                                                 wildcard='ParmFiles(*.parm,*.param)|*.parm;*.param')),
                                         MPMenuItem('FTP', 'FTP', '# param ftp'),
                                         MPMenuItem('Update Metadata', 'Update Metadata', '# param download'),
                                             ])

    def get_component_id_list(self, system_id):
        '''get list of component IDs with parameters for a given system ID'''
        ret = []
        for (s,c) in self.mpstate.mav_param_by_sysid.keys():
            if s == system_id:
                ret.append(c)
        return ret

    def add_new_target_system(self, sysid):
        '''handle a new target_system'''
        if sysid in self.pstate:
            return
        if not sysid in self.mpstate.mav_param_by_sysid:
            self.mpstate.mav_param_by_sysid[sysid] = mavparm.MAVParmDict()
            self.new_sysid_timestamp = time.time()
        fname = 'mav.parm'
        if sysid not in [(0,0),(1,1),(1,0)]:

            fname = 'mav_%u_%u.parm' % (sysid[0], sysid[1])
        self.pstate[sysid] = ParamState(self.mpstate.mav_param_by_sysid[sysid], self.logdir, self.vehicle_name, fname, self.mpstate, sysid)
        if self.continue_mode and self.logdir is not None:
            parmfile = os.path.join(self.logdir, fname)
            if os.path.exists(parmfile):
                mpstate.mav_param.load(parmfile)
                self.pstate[sysid].mav_param_set = set(self.mav_param.keys())
        self.pstate[sysid].param_help.xml_filepath = self.xml_filepath

    def get_sysid(self):
        '''get sysid tuple to use for parameters'''
        component = self.target_component
        if component == 0:
            component = 1
        return (self.target_system, component)

    def check_new_target_system(self):
        '''handle a new target_system'''
        sysid = self.get_sysid()
        if sysid in self.pstate:
            return
        self.add_new_target_system(sysid)

    def param_status(self):
        sysid = self.get_sysid()
        pset, pcount = self.pstate[sysid].status(self.master, self.mpstate)
        return (pset, pcount)
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        sysid = (m.get_srcSystem(),m.get_srcComponent())
        self.add_new_target_system(sysid)
        self.pstate[sysid].handle_mavlink_packet(self.master, m)

    def idle_task(self):
        '''handle missing parameters'''
        self.check_new_target_system()
        sysid = self.get_sysid()
        self.pstate[sysid].vehicle_name = self.vehicle_name
        self.pstate[sysid].param_help.vehicle_name = self.vehicle_name
        self.pstate[sysid].fetch_check(self.master)
        if self.module('console') is not None:
            if not self.menu_added_console:
                self.menu_added_console = True
                self.module('console').add_menu(self.menu)
        else:
            self.menu_added_console = False

    def cmd_param(self, args):
        '''control parameters'''
        self.check_new_target_system()
        sysid = self.get_sysid()
        self.pstate[sysid].handle_command(self.master, self.mpstate, args)

    def fetch_all(self):
        '''force fetch of all parameters'''
        self.check_new_target_system()
        sysid = self.get_sysid()
        self.pstate[sysid].fetch_all(self.master)


def init(mpstate, **kwargs):
    '''initialise module'''
    return ParamModule(mpstate, **kwargs)
