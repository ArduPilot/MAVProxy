#!/usr/bin/env python3
'''
param command handling

AP_FLAKE8_CLEAN
'''

import time
import os
import fnmatch
import struct

from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import param_help
from MAVProxy.modules.lib import param_ftp

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_menu import MPMenuCallFileDialog

try:
    # py2
    from StringIO import StringIO as SIO
except ImportError:
    # py3
    from io import BytesIO as SIO

try:
    import queue as Queue
    from queue import Empty
except ImportError:
    import Queue
    from Queue import Empty


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
        self.default_params = None
        self.watch_patterns = set()

        # dictionary of ParamSet objects we are processing:
        self.parameters_to_set = {}
        # a Queue which onto which ParamSet objects can be pushed in a
        # thread-safe manner:
        self.parameters_to_set_input_queue = Queue.Queue()

    class ParamSet():
        '''class to hold information about a parameter set being attempted'''
        def __init__(self, master, name, value, param_type=None, attempts=None):
            self.master = master
            self.name = name
            self.value = value
            self.param_type = param_type
            self.attempts_remaining = attempts
            self.retry_interval = 1  # seconds
            self.last_value_received = None

            if self.attempts_remaining is None:
                self.attempts_remaining = 3

            self.request_sent = 0  # this is a timestamp

        def normalize_parameter_for_param_set_send(self, name, value, param_type):
            '''uses param_type to convert value into a value suitable for passing
            into the mavlink param_set_send binding.  Note that this
            is a copy of a method in pymavlink, in case the user has
            an older version of that library.
            '''
            if param_type is not None and param_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
                # need to encode as a float for sending
                if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
                    vstr = struct.pack(">xxxB", int(value))
                elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
                    vstr = struct.pack(">xxxb", int(value))
                elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
                    vstr = struct.pack(">xxH", int(value))
                elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
                    vstr = struct.pack(">xxh", int(value))
                elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
                    vstr = struct.pack(">I", int(value))
                elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    vstr = struct.pack(">i", int(value))
                else:
                    print("can't send %s of type %u" % (name, param_type))
                    return None
                numeric_value, = struct.unpack(">f", vstr)
            else:
                if isinstance(value, str) and value.lower().startswith('0x'):
                    numeric_value = int(value[2:], 16)
                else:
                    try:
                        numeric_value = float(value)
                    except ValueError:
                        print(f"can't convert {name} ({value}, {type(value)}) to float")
                        return None

            return numeric_value

        def send_set(self):
            numeric_value = self.normalize_parameter_for_param_set_send(self.name, self.value, self.param_type)
            if numeric_value is None:
                print(f"can't send {self.name} of type {self.param_type}")
                self.attempts_remaining = 0
                return
            # print(f"Sending set attempts-remaining={self.attempts_remaining}")
            self.master.param_set_send(
                self.name.upper(),
                numeric_value,
                parm_type=self.param_type,
            )
            self.request_sent = time.time()
            self.attempts_remaining -= 1

        def expired(self):
            if self.attempts_remaining > 0:
                return False
            return time.time() - self.request_sent > self.retry_interval

        def due_for_retry(self):
            if self.attempts_remaining <= 0:
                return False
            return time.time() - self.request_sent > self.retry_interval

        def handle_PARAM_VALUE(self, m, value):
            '''handle PARAM_VALUE packet m which has already been checked for a
            match against self.name.  Returns true if this Set is now
            satisfied.  value is the value extracted and potentially
            manipulated from the packet
            '''
            self.last_value_received = value
            if abs(value - float(self.value)) > 0.00001:
                return False

            return True

        def print_expired_message(self):
            reason = ""
            if self.last_value_received is None:
                reason = " (no PARAM_VALUE received)"
            else:
                reason = f" (invalid returned value {self.last_value_received})"
            print(f"Failed to set {self.name} to {self.value}{reason}")

    def run_parameter_set_queue(self):
        # firstly move anything from the input queue into our
        # collection of things to send:
        try:
            while True:
                new_parameter_to_set = self.parameters_to_set_input_queue.get(block=False)
                self.parameters_to_set[new_parameter_to_set.name] = new_parameter_to_set
        except Empty:
            pass

        # now send any parameter-sets which are due to be sent out,
        # either because they are new or because we need to retry:
        count = 0
        keys_to_remove = []  # remove entries after iterating the dict
        for (key, parameter_to_set) in self.parameters_to_set.items():
            if parameter_to_set.expired():
                parameter_to_set.print_expired_message()
                keys_to_remove.append(key)
                continue
            if not parameter_to_set.due_for_retry():
                continue
            # send parameter set:
            parameter_to_set.send_set()
            # rate-limit to 10 items per call:
            count += 1
            if count > 10:
                break

        # complete purging of expired parameter-sets:
        for key in keys_to_remove:
            del self.parameters_to_set[key]

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
        if self.autopilot_type_by_sysid.get(sysid, -1) in [mavutil.mavlink.MAV_AUTOPILOT_PX4]:
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
            self.handle_mavlink_watch_param_value(master, m)
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

            # if we were setting this parameter then check it's the
            # value we want and, if so, stop setting the parameter
            try:
                if self.parameters_to_set[param_id].handle_PARAM_VALUE(m, value):
                    # print(f"removing set of param_id ({self.parameters_to_set[param_id].value} vs {value})")
                    del self.parameters_to_set[param_id]
            except KeyError:
                pass

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
        return (len(self.mav_param_set), self.mav_param_count)

    def ftp_start(self):
        '''start a ftp download of parameters'''
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            self.ftp_failed = True
            self.ftp_started = False
            return
        self.ftp_started = True
        self.ftp_count = None
        ftp.cmd_get([
            "@PARAM/param.pck?withdefaults=1",
        ],
            callback=self.ftp_callback,
            callback_progress=self.ftp_callback_progress,
        )

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
            magic2, num_params, total_params = struct.unpack("<HHH", buf)
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

        # magic = 0x671b
        # magic_defaults = 0x671c
        data = fh.read()
        pdata = param_ftp.ftp_param_decode(data)
        if pdata is None or len(pdata.params) == 0:
            return
        with_defaults = pdata.defaults is not None

        self.param_types = {}
        self.mav_param_set = set()
        self.fetch_one = dict()
        self.fetch_set = None
        self.mav_param.clear()
        total_params = len(pdata.params)
        self.mav_param_count = total_params

        idx = 0
        for (name, v, ptype) in pdata.params:
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
        self.log_params(pdata.params)

        if with_defaults:
            self.default_params = mavparm.MAVParmDict()
            for (name, v, ptype) in pdata.defaults:
                name = str(name.decode('utf-8'))
                self.default_params[name] = v
            if self.logdir:
                defaults_path = os.path.join(self.logdir, "defaults.parm")
                self.default_params.save(defaults_path, '*', verbose=False)
                print("Saved %u defaults to %s" % (len(pdata.defaults), defaults_path))

    def fetch_all(self, master):
        '''force refetch of parameters'''
        if not self.use_ftp():
            master.param_fetch_all()
            self.mav_param_set = set()
        else:
            self.ftp_start()

    def param_diff(self, args):
        '''handle param diff'''
        wildcard = '*'
        if len(args) < 1 or args[0].find('*') != -1:
            defaults = self.default_params
            if defaults is None:
                print("Cannot find default parameters")
                return
            if len(args) >= 1:
                wildcard = args[0]
        else:
            filename = args[0]
            if not os.path.exists(filename):
                print("Can't find defaults file %s" % filename)
                return
            defaults = mavparm.MAVParmDict()
            defaults.load(filename)
            if len(args) == 2:
                wildcard = args[1]
        print("\nParameter        Current  Default")
        for p in self.mav_param:
            p = str(p).upper()
            if p not in defaults:
                continue
            if self.mav_param[p] == defaults[p]:
                continue
            if fnmatch.fnmatch(p, wildcard.upper()):
                s1 = "%f" % self.mav_param[p]
                s2 = "%f" % defaults[p]
                if s1 == s2:
                    continue
                s = "%-16.16s %s %s" % (str(p), s1, s2)
                if self.mpstate.settings.param_docs and self.vehicle_name is not None:
                    info = self.param_help.param_info(p, self.mav_param[p])
                    if info is not None:
                        s += " # %s" % info
                    info_default = self.param_help.param_info(p, defaults[p])
                    if info_default is not None:
                        s += " (DEFAULT: %s)" % info_default
                print(s)

    def param_savechanged(self, args):
        '''handle param savechanged'''
        if len(args) < 1:
            filename = "changed.parm"
        else:
            filename = args[0]
        defaults = self.default_params
        if defaults is None:
            print("No defaults available")
            return
        f = open(filename, "w")
        count = 0
        for p in self.mav_param:
            p = str(p).upper()
            if p not in defaults:
                continue
            if self.mav_param[p] == defaults[p]:
                continue
            s1 = "%f" % self.mav_param[p]
            s2 = "%f" % defaults[p]
            if s1 == s2:
                continue
            s = "%-16.16s %s" % (str(p), s1)
            f.write("%s\n" % s)
            count += 1
        f.close()
        print("Saved %u parameters to %s" % (count, filename))

    def handle_mavlink_watch_param_value(self, master, m):
        param_id = "%.16s" % m.param_id
        for pattern in self.watch_patterns:
            if fnmatch.fnmatch(param_id, pattern):
                self.mpstate.console.writeln("> %s=%f" % (param_id, m.param_value))

    def param_watch(self, master, args):
        '''command to allow addition of watches for parameter changes'''
        for pattern in args:
            self.watch_patterns.add(pattern)

    def param_unwatch(self, master, args):
        '''command to allow removal of watches for parameter changes'''
        for pattern in args:
            self.watch_patterns.discard(pattern)

    def param_watchlist(self, master, args):
        '''command to show watch patterns for parameter changes'''
        for pattern in self.watch_patterns:
            self.mpstate.console.writeln("> %s" % (pattern))

    def param_bitmask_modify(self, master, args):
        '''command for performing bitmask actions on a parameter'''

        BITMASK_ACTIONS = ['toggle', 'set', 'clear']
        NUM_BITS_MAX = 32

        # Ensure we have at least an action and a parameter
        if len(args) < 2:
            print("Not enough arguments")
            print(f"param bitmask <{'/'.join(BITMASK_ACTIONS)}> <parameter> [bit-index-1 ... bit-index-n]")
            return

        action = args[0]
        if action not in BITMASK_ACTIONS:
            print(f"action must be one of: {', '.join(BITMASK_ACTIONS)}")
            return

        # Grab the parameter argument, and check it exists
        param = args[1]
        if not param.upper() in self.mav_param:
            print(f"Unable to find parameter {param.upper()}")
            return
        uname = param.upper()

        htree = self.param_help.param_help_tree()
        if htree is None:
            # No help tree is available
            print("Download parameters first")
            return

        # Take the help tree and check if parameter is a bitmask
        phelp = htree[uname]
        bitmask_values = self.param_help.get_bitmask_from_help(phelp)
        if bitmask_values is None:
            print(f"Parameter {uname} is not a bitmask")
            return

        # Find the type of the parameter
        ptype = None
        if uname in self.param_types:
            # Get the type of the parameter
            ptype = self.param_types[uname]

        # Now grab the value for the parameter
        value = int(self.mav_param.get(uname))
        if value is None:
            print(f"Could not get a value for parameter {uname}")
            return

        # The next argument is the bit_index - if it exists, handle it
        bit_indices = []
        while len(args) >= 3:
            try:
                # If the bit index is available lets grab it
                arg_bit_index = args[2]
                # Remove the bit index from the args list
                args.pop(2)
                # Try to convert it to int
                bit_indices.append(int(arg_bit_index))
            except ValueError:
                print(f"Invalid bit index: {arg_bit_index}")

        if bit_indices == []:
            # No bit index was specified, but the parameter and action was.
            # Print the available bitmask information.
            print("%s: %s" % (uname, phelp.get('humanName')))
            s = "%-16.16s %s" % (uname, value)
            print(s)

            # Generate the bitmask enabled list
            remaining_bits = value
            out_v = []
            if bitmask_values is not None and len(bitmask_values):
                for (n, v) in bitmask_values.items():
                    out_v.append(f"\t{int(n):3d} [{'x' if value & (1 << int(n)) else ' '}] : {v}")
                    remaining_bits &= ~(1 << int(n))

                # Loop bits 0 to 31, checking if they are remaining, and append
                for i in range(32):
                    if (remaining_bits & (1 << i)):
                        out_v.append(f"\t{i:3d} [{'x' if value & (1 << i) else ' '}] : Unknownbit{i}")

            if out_v is not None and len(out_v) > 0:
                print("\nBitmask: ")
                print("\n".join(out_v))

            # Finally, inform user of the error we experienced
            print("bit index is not specified")

            # We don't have enough information to modify the bitmask, so bail
            return

        # Sanity check the bit indices
        invalid_bits = [bit_index for bit_index in bit_indices if bit_index >= NUM_BITS_MAX]
        if invalid_bits:
            print(f"Cannot perform bitmask action '{action}' on bit(s) {', '.join(str(bit) for bit in invalid_bits)}.")
            return

        # Cycle through the bit indices
        for bit_index in bit_indices:
            # We have enough information to perform an action
            if action == BITMASK_ACTIONS[0]:  # "toggle"
                value = value ^ (1 << bit_index)
            elif action == BITMASK_ACTIONS[1]:  # "set"
                value = value | (1 << bit_index)
            elif action == BITMASK_ACTIONS[2]:  # "clear"
                value = value & ~(1 << bit_index)
            else:
                # We cannot toggle, set or clear
                print("Invalid bitmask action")
                return

        # Update the parameter
        self.set_parameter(master, uname, value, attempts=3, param_type=ptype)

    def set_parameter(self, master, name, value, attempts=None, param_type=None):
        '''convenient intermediate method which determines parameter type for
        lazy callers'''
        if param_type is None:
            param_type = self.param_types.get(name, None)

        self.parameters_to_set_input_queue.put(ParamState.ParamSet(
            master,
            name,
            value,
            attempts=attempts,
            param_type=param_type,
        ))

    def param_revert(self, master, args):
        '''handle param revert'''
        defaults = self.default_params
        if defaults is None:
            print("No defaults available")
            return
        if len(args) == 0:
            print("Usage: param revert PATTERN")
            return
        wildcard = args[0].upper()
        count = 0
        for p in self.mav_param:
            p = str(p).upper()
            if not fnmatch.fnmatch(p, wildcard):
                continue
            if p not in defaults:
                continue
            if self.mav_param[p] == defaults[p]:
                continue
            s1 = "%f" % self.mav_param[p]
            s2 = "%f" % defaults[p]
            if s1 == s2:
                continue
            print("Reverting %-16.16s  %s -> %s" % (p, s1, s2))
            self.set_parameter(master, p, defaults[p], attempts=3)
            count += 1
        print("Reverted %u parameters" % count)

    def handle_command(self, master, mpstate, args):
        '''handle parameter commands'''
        param_wildcard = "*"
        usage="Usage: param <fetch|ftp|save|savechanged|revert|set|show|load|preload|forceload|ftpload|diff|download|check|help|watch|unwatch|watchlist|bitmask>"  # noqa
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
            self.param_diff(args[1:])
        elif args[0] == "savechanged":
            self.param_savechanged(args[1:])
        elif args[0] == "revert":
            self.param_revert(master, args[1:])
        elif args[0] == "watch":
            self.param_watch(master, args[1:])
        elif args[0] == "unwatch":
            self.param_unwatch(master, args[1:])
        elif args[0] == "watchlist":
            self.param_watchlist(master, args[1:])
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
            self.set_parameter(master, uname, value, attempts=3)

            if (param.upper() == "WP_LOITER_RAD" or param.upper() == "LAND_BREAK_PATH"):
                # need to redraw rally points
                # mpstate.module('rally').set_last_change(time.time())
                # need to redraw loiter points
                mpstate.module('wp').wploader.last_change = time.time()
        elif args[0] == "bitmask":
            self.param_bitmask_modify(master, args[1:])
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
            if len(args) < 2:
                print(usage)
                return
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
        k = mp_util.sorted_natural(self.mav_param.keys())
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
                for k in mp_util.sorted_natural(self.ftp_send_param.keys()):
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
        c = min(len(s1), len(s2))
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
        for k in mp_util.sorted_natural(newparm.keys()):
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
        for k in mp_util.sorted_natural(newparm.keys()):
            v = newparm.get(k)
            vtype = self.best_type(v)
            common_len = self.str_common_len(last_param, k)
            b1 = vtype
            b2 = common_len | ((len(k) - (common_len + 1)) << 4)
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
        super(ParamModule, self).__init__(mpstate, "param", "parameter handling", public=True, multi_vehicle=True)
        self.xml_filepath = kwargs.get("xml-filepath", None)
        self.pstate = {}
        self.check_new_target_system()
        self.menu_added_console = False
        bitmask_indexes = "|".join(str(x) for x in range(32))
        self.add_command(
            'param', self.cmd_param, "parameter handling", [
                "<download|status>",
                "<set|show|fetch|ftp|help|apropos|revert> (PARAMETER)",
                "<load|save|savechanged|diff|forceload|ftpload> (FILENAME)",
                "<set_xml_filepath> (FILEPATH)",
                f"<bitmask> <toggle|set|clear> (PARAMETER) <{bitmask_indexes}>"
            ],
        )
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu(
                'Parameter',
                items=[
                    MPMenuItem('Editor', 'Editor', '# module load paramedit'),
                    MPMenuItem('Fetch', 'Fetch', '# param fetch'),
                    MPMenuItem('Load', 'Load', '# param load ',
                               handler=MPMenuCallFileDialog(
                                   flags=('open',),
                                   title='Param Load',
                                   wildcard='ParmFiles(*.parm,*.param)|*.parm;*.param')),
                    MPMenuItem('Save', 'Save', '# param save ',
                               handler=MPMenuCallFileDialog(
                                   flags=('save', 'overwrite_prompt'),
                                   title='Param Save',
                                   wildcard='ParmFiles(*.parm,*.param)|*.parm;*.param')),
                    MPMenuItem('FTP', 'FTP', '# param ftp'),
                    MPMenuItem('Update Metadata', 'Update Metadata', '# param download'),
                ],
            )

    def get_component_id_list(self, system_id):
        '''get list of component IDs with parameters for a given system ID'''
        ret = []
        for (s, c) in self.mpstate.mav_param_by_sysid.keys():
            if s == system_id:
                ret.append(c)
        return ret

    def add_new_target_system(self, sysid):
        '''handle a new target_system'''
        if sysid in self.pstate:
            return
        if sysid not in self.mpstate.mav_param_by_sysid:
            self.mpstate.mav_param_by_sysid[sysid] = mavparm.MAVParmDict()
            self.new_sysid_timestamp = time.time()
        fname = 'mav.parm'
        if sysid not in [(0, 0), (1, 1), (1, 0)]:

            fname = 'mav_%u_%u.parm' % (sysid[0], sysid[1])
        self.pstate[sysid] = ParamState(
            self.mpstate.mav_param_by_sysid[sysid],
            self.logdir,
            self.vehicle_name,
            fname,
            self.mpstate,
            sysid,
        )
        if self.continue_mode and self.logdir is not None:
            parmfile = os.path.join(self.logdir, fname)
            if os.path.exists(parmfile):
                self.mpstate.mav_param.load(parmfile)
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
        sysid = (m.get_srcSystem(), m.get_srcComponent())
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

        self.run_parameter_set_queues()

    def run_parameter_set_queues(self):
        for pstate in self.pstate.values():
            pstate.run_parameter_set_queue()

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
