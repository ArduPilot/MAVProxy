#!/usr/bin/env python

'''
Fleet Module
Peter Barker, March 2022

AP_FLAKE8_CLEAN

TODO:
 - correct firmware
 - correct parameters
 - correct mission
 - correct scripts
 - correct fence
 - correct rally points
 - reasonable log space remaining
 - correct sysid / compid
 - "identify vehicle" by flashing lights, playing tones, maybe a motor test?
 - persist vehicle data
   - should we set data first in the persistent data and copy to vehicle at leisure?
     - would allow for off-line work
'''

import copy
import datetime
import fnmatch
import json
import os
import shutil
import re
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class SeenVehicle(object):
    '''information about a vehicle we've seen traffic from'''

    def __init__(self, sysidcompid):
        self.sysidcompid = sysidcompid

        self.uid = None
        self.last_seen = None
        self.autopilot_version_flight_sw_version = None

    def seen(self):
        '''called to indicate the vehicle's been seen right now - e.g.via
        receipt of a heartbeat with its sysid/compid'''
        self.last_seen = time.time()

    def set_uid(self, uid):
        '''sets the probed UID for the SeenVehicle'''
        self.uid = uid

    def uid_as_string(self):
        '''turns byte array into string'''
        ret = "".join(["%02x" % x for x in self.uid])
        return ret[0:24]

    def key(self):
        '''unique key for this Seen Vehicle.  Because we see via mavlink, this
        is just sysid/compid'''
        return self.sysidcompid

    def mavlink_target(self):
        '''returns tuple of (sysid, compid)'''
        return self.sysidcompid

    # this stuff no longer belongs here:
    # def desired_firmware_version(self):
    #     return (4, 1, 0)

    # def current_firmware_version(self):
    #     return self.autopilot_version_flight_sw_version

    # def correct_firmware_version(self):
    #     return self.current_firmware_version() == self.desired_firmware_version()


class VehicleClass(object):
    '''definition of a Vehicle class, of which a vehicle will have a list'''
    def __init__(self, name, params=None, paramfile=None, calibration_params=None):
        self.name = name
        self.params = params
        self.paramfile = paramfile
        self.calibration_params = calibration_params

    def set_params(self, params):
        self.params = copy.copy(params)


class Vehicle(object):
    '''a Vehicle configuration keyed off UID'''

    def __init__(self, uid, classes=[], name=None):
        self.uid = uid
        self.name = name
        self._classes = classes


class Fleet(mp_module.MPModule):
    '''main module, reads/writes mavlink, configures vehicle, ...'''
    def __init__(self, mpstate):
        """Initialise module"""
        super(Fleet, self).__init__(mpstate, "fleet", "")

        self.fleet_settings = mp_settings.MPSettings([
            ('verbose', bool, False),
            ('probe_interval', int, 15),  # probe each sysid at 15s intervals
            ('config_filepath', str, "fleet.json"),
        ])
        self.add_command('fleet', self.cmd_fleet, "fleet module", ['status', 'set (LOGSETTING)'])

        self.seen_vehicles = {}
        self.probe_list = []
        self.probe_interval = 0   # time between sending probes
        self.last_probe_time = 0

        self.config = None
        self.last_config_filepath = None
        self.last_config_filepath_mtime = None
        self.config_filepath_bad = False
        self.config_filepath = None
        self.last_config_change = None

        self.initial_backup_done = False

    def print(self, message):
        '''simple output wrapper to give context to message'''
        print("Fleet: %s" % message)

    def usage(self):
        '''return base fleet command usage string'''
        return "Usage: fleet <vehicle|class|status|set>"

    def backup_config(self):
        '''backs the configuration file up by appending a date to the
        configuration filename'''
        if self.config_filepath is None:
            self.print("Backup filepath is None")
            return
        backup_filepath = self.config_filepath + "-" + datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        self.print("Backing up config to %s" % backup_filepath)
        shutil.copy(self.config_filepath, backup_filepath)

    def get_classes_for_vehicle(self, vehicle):
        '''returns a list of VehicleClass objects for the vehicle'''
        ret = []
        for c_name in vehicle._classes:
            try:
                c = self.get_config_class_by_name(c_name)
            except KeyError:
                self.print("No such class %s" % c_name)
                return
            ret.append(c)
        return ret

    def get_config_class_by_name(self, classname):
        return self.config["classes"][classname]

    def get_config_vehicle_by_uid(self, uid):
        return self.config["vehicles"][uid]

    def load_param_file(self, filepath, pattern=None):
        '''loads params from filepath, filters by glob pattern and returns dict'''
        if pattern is not None:
            pattern = pattern.upper()

        ret = {}
        printed_nl = False
        with open(filepath) as f:
            while True:
                line = f.readline()
                if line == "":
                    break
                m = re.match(r"\s*(\w+)[^\w.\d]+([.\d]+)", line)
                if m is None:
                    self.print("Did not match line (%s)" % line)
                    return
                (name, value) = (m.group(1), m.group(2))
                name = name.upper()
                if pattern is not None:
                    if not fnmatch.fnmatch(name.upper(), pattern):
                        continue
                value = float(value)
                if not printed_nl:
                    print("\n")
                    printed_nl = True
                self.print("Loaded (%15s = %f)" % (name, value))
                ret[name] = value
                self.last_config_change = time.time()
        return ret

    '''Command handling'''
    def cmd_fleet(self, args):
        '''handle base "fleet" command'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.cmd_fleet_status(args[1:]))
        elif args[0] == "vehicle":
            self.cmd_fleet_vehicle(args[1:])
        elif args[0] == "class":
            self.cmd_fleet_class(args[1:])
        elif args[0] == "param":
            self.cmd_fleet_param(args[1:])
        elif args[0] == "set":
            self.fleet_settings.command(args[1:])
        elif args[0] == "backup_config":
            self.backup_config()
        else:
            print(self.usage())

    def cmd_fleet_class_add(self, args):
        '''command to handle adding a class'''
        if len(args) < 1:
            self.print("fleet class add NAME")
            return

        (name, *args) = args

        self.config["classes"]["name"] = VehicleClass(name)

    def cmd_fleet_class_param(self, args):
        '''command to manipulate class parameters'''
        if len(args) < 1:
            self.print("fleet class param (cal|del|load|loadfromvehicle|set)")
            return

        (cmd, *args) = args

        if cmd == "cal" or cmd == "calibration":
            return self.cmd_fleet_class_param_cal(args)

        if cmd == "del":
            return self.cmd_fleet_class_param_del(args)

        if cmd == "load":
            return self.cmd_fleet_class_param_load(args)

        if cmd == "loadfromvehicle":
            return self.cmd_fleet_class_param_loadfromvehicle(args)

        if cmd == "set":
            return self.cmd_fleet_class_param_set(args)

        self.print("Unknown param subcommand %s" % cmd)

    def cmd_fleet_class_param_load(self, args):
        '''load parameters into a class from a file'''
        if len(args) < 2:
            self.print("load requires classname and filename")
            return
        (classname, filepath, *args) = args

        gotclass = self.get_config_class_by_name(classname)
        if gotclass is None:
            self.print("bad classname")
            return

        if not os.path.exists(filepath):
            self.print("%s does not exist" % filepath)
            return

        pattern = None
        if len(args):
            (pattern, *args) = args

        params = self.load_param_file(filepath, pattern=pattern)
        gotclass.set_params(params)

    def cmd_fleet_class_param_set(self, args):
        if len(args) < 3:
            self.print("set requires classname, parameter name and value")
            return
        (classname, name, value, *args) = args

        gotclass = self.get_config_class_by_name(classname)
        if gotclass is None:
            self.print("bad classname")
            return

        if name in gotclass.params:
            self.print("Old value for (%s) was %f" % gotclass.params[name])

        if name in gotclass.calibration_params:
            self.print("Warning: setting calibration parameter")

        name = name.upper()
        if not re.match(r"^[\w\d]+$"):
            self.print("Invalid parameter name")
            return

        gotclass.params[name] = float(value)

    def cmd_fleet_class_param_loadfromvehicle(self, args):
        if len(args) < 3:
            self.print("loadfromvehicle requires classname uid parametername")
            return
        (classname, vehicle_uid, pattern, *args) = args

        gotclass = self.get_config_class_by_name(classname)
        if gotclass is None:
            self.print("bad classname")
            return

        pattern = pattern.upper()

        # FIXME: this should come from persistent vehicle state (not
        # in existance at time of writing....), not the seen_vehicle:
        seen_vehicle = self.get_seen_vehicle_by_uid(vehicle_uid)
        if seen_vehicle is None:
            self.print("Have not seen %s" % vehicle_uid)
            return

        mavparm = self.mpstate.mav_param_by_sysid[seen_vehicle.mavlink_target()]
        change_made = False
        for (name, value) in sorted(mavparm.items(), key=lambda x : x[0]):
            if not fnmatch.fnmatch(name.upper(), pattern):
                continue
            if name in gotclass.calibration_params:
                self.print("Warning: setting calibration parameter")
            old = ""
            if name in gotclass.params:
                old = " (was %f)" % gotclass.params[name]
            self.print("Setting (%s) to (%f)%s" % (name, value, old))
            gotclass.params[name] = value
            change_made = True

        if change_made:
            self.last_config_change = time.time()

    def cmd_fleet_class_param_cal_setorunset(self, setorunset, args):
        is_cal = setorunset == "set"

        if len(args) < 2:
            self.print("fleet class param config %s CLASSNAME PATTERN" % setorunset)
            return

        (classname, pattern, *args) = args
        pattern = pattern.upper()

        c = self.get_config_class_by_name(classname)
        if c is None:
            self.print("No such class")
            return

        change_made = False
        for name in c.params.keys():
            if not fnmatch.fnmatch(name.upper(), pattern):
                continue
            if is_cal:
                if not c.calibration_params.get(name, 0):
                    self.print("Marking (%s) as a calibration parameter" % (name,))
                    c.calibration_params[name] = 1
                    change_made = True
            else:
                try:
                    del c.calibration_params[name]
                    change_made = True
                except KeyError:
                    continue
        if change_made:
            self.last_config_change = time.time()

    def cmd_fleet_class_param_cal_set(self, args):
        self.cmd_fleet_class_param_cal_setorunset("set", args)

    def cmd_fleet_class_param_cal_unset(self, args):
        self.cmd_fleet_class_param_cal_setorunset("unset", args)

    def cmd_fleet_class_param_cal(self, args):
        '''command to mark parameters as calibration variables'''
        if len(args) < 1:
            self.print("fleet class param cal (set|unset)")
            return

        (cmd, *args) = args

        if cmd == "set":
            return self.cmd_fleet_class_param_cal_set(args)

        if cmd == "unset":
            return self.cmd_fleet_class_param_cal_unset(args)

        self.print("Unknown param config subcommand %s" % cmd)

    def cmd_fleet_class_param_del(self, args):
        if len(args) < 2:
            self.print("del requires classname and pattern (which might be *)")
            return
        (classname, pattern, *args) = args

        gotclass = self.get_config_class_by_name(classname)
        if gotclass is None:
            self.print("bad classname")
            return

        pattern = pattern.upper()
        for name in list(gotclass.params.keys()):
            if not fnmatch.fnmatch(name.upper(), pattern):
                continue
            print("Deleting (%s) (was %f)" % (name, gotclass.params[name]))
            del gotclass.params[name]

    def cmd_fleet_param_loadtovehicle(self, args):
        '''loads parameters to vehicle based on which classes it is in'''
        if len(args) < 2:
            self.print("fleet param loadtovehicle UID PATTERN")
            return

        (vehicle_uid, pattern, *args) = args
        pattern = pattern.upper()

        try:
            vehicle = self.get_config_vehicle_by_uid(vehicle_uid)
        except KeyError:
            self.print("No such vehicle %s" % vehicle_uid)
            return

        # can't set parameters on a vehicle we haven't seen:
        # see TODO list up the top for musings on this
        seen_vehicle = self.get_seen_vehicle_by_uid(vehicle_uid)
        if seen_vehicle is None:
            self.print("Have not seen %s" % vehicle_uid)
            return

        mavparm = self.mpstate.mav_param_by_sysid[seen_vehicle.mavlink_target()]

        expected_values = self.get_expected_parameters_for_vehicle(vehicle)

        param_module = self.mpstate.module('param')
        if param_module is None:
            self.print("No param module")
            return ""

        for (name, oldvalue) in sorted(expected_values.items(), key=lambda x : x[0]):
            if not fnmatch.fnmatch(name.upper(), pattern):
                continue
            if name not in mavparm:
                self.print("%s not in vehicle parameters?" % name)
                continue
            oldvalue = mavparm[name]

            newvalue = expected_values[name]

            self.print("Setting vehicle %s to %f from %f" %
                       (name, newvalue, oldvalue))
            param_module.cmd_param(["set", name, str(newvalue)], sysid=seen_vehicle.mavlink_target())

            change_made = True

        if change_made:
            self.last_config_change = time.time()

    def cmd_fleet_param(self, args):
        '''handle "fleet param" commands'''
        if len(args) < 1:
            self.print("fleet param (loadtovehicle)")
            return

        (cmd, *args) = args

        if cmd == "loadtovehicle":
            return self.cmd_fleet_param_loadtovehicle(args)

        self.print("Unknown fleet param command (%s)" % cmd)

    def cmd_fleet_class(self, args):
        '''handle "fleet class" commands - manipulation of classes vehicles can be in'''
        if len(args) == 0:
            self.print("fleet class (status|param)")
            return

        (cmd, *args) = args

        if cmd == "status":
            for c in self.config["classes"].values():
                out = [c.name]
                if c.params is not None:
                    out.append("paramcount=%u" % len(list(c.params.keys())))
                print(" ".join(out))
                return

        if cmd == "param":
            return self.cmd_fleet_class_param(args)

        self.print("Unknown class command (%s)" % cmd)

    def get_seen_vehicle_by_uid(self, uid):
        for vehicle in self.seen_vehicles.values():
            if vehicle.uid_as_string() == uid:
                return vehicle

        return None

    def cmd_fleet_vehicle(self, args):
        if len(args) < 1:
            self.print("fleet vehicle (status|add|del|setclasses) ...")
            return
        (cmd, *args) = args

        if cmd == "status":
            if len(args) < 1:
                print("Seen vehicles")
                for seen_vehicle in self.seen_vehicles.values():
                    items = []
                    items.append("UID:%s" % seen_vehicle.uid_as_string())
                    items.append("%u/%u" % seen_vehicle.mavlink_target())
                    if seen_vehicle.uid_as_string() in self.config["vehicles"]:
                        items.append("KNOWN")
                    else:
                        items.append("UNKNOWN")
                    print(" ".join(items))
                return

            (vehicle_uid, *args) = args
            vehicle = None
            for seen_vehicle in self.seen_vehicles.values():
                if seen_vehicle.uid_as_string() == vehicle_uid:
                    vehicle = seen_vehicle
                    break
            if vehicle is None:
                self.print("Unknown vehicle (%s)" % vehicle_uid)
                return
            print(self.vehicle_status(vehicle))
            return

        if cmd == "add":
            if len(args) < 1:
                self.print("Need UID for add")
                return
            vehicle_uid = args[0]
            seen_vehicle = self.get_seen_vehicle_by_uid(vehicle_uid)
            if seen_vehicle is None:
                self.print("Have not seen %s" % vehicle_uid)
                return

            vehicle = None
            try:
                vehicle = self.get_config_vehicle_by_uid(vehicle_uid)
            except KeyError:
                pass
            if vehicle is not None:
                self.print("Already have vehicle for (%s)" % vehicle_uid)
                return

            self.config["vehicles"][vehicle_uid] = Vehicle(vehicle_uid)
            self.last_config_change = time.time()

            return

        if cmd == "del":
            if len(args) < 1:
                self.print("Need UID for del")
                return
            vehicle_uid = args[0]

            del self.config["vehicles"][vehicle_uid]
            self.last_config_change = time.time()

            return

        if cmd == "setclasses":
            # set which classes the vehicle is in
            if len(args) < 1:
                self.print("Need UID for setclasses")
                return
            (vehicle_uid, *args) = args
            try:
                vehicle = self.get_config_vehicle_by_uid(vehicle_uid)
            except KeyError:
                self.print("No such UID")
                return
            vehicle._classes = copy.copy(args)
            self.last_config_change = time.time()

            return

        self.print("Unknown vehicle command %s" % str(cmd))

    def vehicle_status(self, vehicle):
        class Item(object):
            def __init__(self, passed, desc, reason):
                self.passed = passed
                self.desc = desc
                self.reason = reason
        items = []

        # check firmware version being run - needs to come from classes
        # desired_fwver = vehicle.desired_firmware_version()
        # current_fwver = vehicle.current_firmware_version()
        # if desired_fwver != current_fwver:
        #     items.append(Item(False, "Firmware Version", "want=%s != got=%s" % (desired_fwver, current_fwver)))
        # else:
        #     items.append(Item(True, "Firmware Version", "want=%s == got=%s" % (desired_fwver, current_fwver)))

        # check parameters
        desired_parameter_values = {
            "ARMING_CHECK": 0,
        }
        params = self.mpstate.mav_param_by_sysid[vehicle.mavlink_target()]
        for (param_name, want_value) in desired_parameter_values.items():
            got_value = params[param_name]
            if want_value != got_value:
                items.append(Item(False, "Parameter %s" % param_name, "want=%f got=%f" % (want_value, got_value)))

        # format and return output
        desc_len = 0
        reason_len = 0
        for item in items:
            if len(item.desc) > desc_len:
                desc_len = len(item.desc)
            if len(item.reason) > reason_len:
                reason_len = len(item.reason)

        format_string = "%3s %" + str(desc_len) + "s  %" + str(reason_len) + "s\n"
        ret = "\n"
        for item in items:
            if item.passed:
                ok = "OK"
            else:
                ok = "BAD"

            ret += format_string % (ok, item.desc, item.reason)

        return ret

    def cmd_fleet_status_listvehicles(self):
        ret = "\n"
        now = time.time()
        for vehicle in self.config["vehicles"].values():
            status = ""
            age = "?"
            seen_vehicle = self.get_seen_vehicle_by_uid(vehicle.uid)
            if seen_vehicle is not None:
                age = "%2.2f" % (now - seen_vehicle.last_seen)

#            if not vehicle.correct_firmware_version():
#                status = "!FW"
#            if not vehicle.correct_firmware_hash():
#                status = "!FWHash"
            if not len(status):
                status = "OK"
            if vehicle.name is None:
                vehicle.name = "[NoName]"
            ret += "%10s (%s): %s %s" % (
                vehicle.name,
                vehicle.uid,
                age,
                status)
        return ret

    def get_expected_parameters_for_vehicle(self, vehicle):
        ret = {}
        for c in self.get_classes_for_vehicle(vehicle):
            if c.params is None:
                print("No params for (%s)" % str(c))
                continue
            ret.update(c.params)
        return ret

    def get_expected_calibration_parameters_for_vehicle(self, vehicle):
        ret = {}
        for c in self.get_classes_for_vehicle(vehicle):
            if c.calibration_params is None:
                print("No params for (%s)" % str(c))
                continue
            ret.update(c.calibration_params)
        return ret

    def cmd_fleet_status_check_params(self, vehicle):
        seen_vehicle = self.get_seen_vehicle_by_uid(vehicle.uid)
        if seen_vehicle is None:
            self.print("%s is not live" % vehicle.uid)
            return ""

        param_module = self.mpstate.module('param')
        if param_module is None:
            self.print("No param module")
            return ""

        (pset, pcount) = param_module.param_status(sysid=seen_vehicle.mavlink_target())

        mavparm = self.mpstate.mav_param_by_sysid[seen_vehicle.mavlink_target()]

        expected_parameters = self.get_expected_parameters_for_vehicle(vehicle)

        expected_calibration_parameters = self.get_expected_calibration_parameters_for_vehicle(vehicle)

        ret = ""
        for (got_param, got_value) in sorted(mavparm.items(), key=lambda x : x[0]):
            if got_param not in expected_parameters and got_param:
                ret += "Vehicle has unknown param (%s = %f)\n" % (got_param, got_value)
                continue
            expected_value = expected_parameters[got_param]
            del expected_parameters[got_param]
            if got_param in expected_calibration_parameters:
                # don't bother checking the value - it's a calibration parameter
                del expected_calibration_parameters[got_param]
                continue
            if abs(expected_value - got_value) < 0.00001:
                continue

            ret += "Vehicle value for (%s) is %f vs expected %f\n" % (got_param, got_value, expected_value)
            continue

        for expected in expected_parameters.keys():
            if expected in expected_calibration_parameters.keys():
                ret += "Vehicle does not have (calibration) parameter (%s)\n" % (expected, )
            else:
                ret += "Vehicle does not have parameter (%s)\n" % (expected, )

        return ret

    def cmd_fleet_status(self, args):
        '''returns seen-vehicle status; either a single vheicle if passed in, or a list of vehicles seen'''
        if len(args) < 1:
            return self.cmd_fleet_status_listvehicles()

        (uid, *args) = args
        try:
            vehicle = self.get_config_vehicle_by_uid(uid)
        except KeyError:
            self.print("No such vehicle (%s)" % uid)
            return

        ret = "Status for %s:" % uid
        ret += self.cmd_fleet_status_check_params(vehicle)
        return ret

    def request_message(self, to_probe, message_id, p1=None):
        (target_sysid, target_compid) = to_probe.mavlink_target()

        if p1 is None:
            p1 = 0

        self.master.mav.command_long_send(
            target_sysid,  # target_system
            target_compid, # target_component
            message_id, # command
            0, # confirmation
            p1, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def send_probe_for_autopilot_version(self, to_probe):
        self.request_message(to_probe,
                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                             p1=mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)

    def do_probes(self):
        if self.fleet_settings.probe_interval == 0:
            # probing disabled
            return
        if len(self.probe_list) == 0:
            self.probe_list = list(self.seen_vehicles.values())
            # filter probe list to recently seen vehicles:
            now = time.time()
            self.probe_list = list(filter(lambda x : now - x.last_seen < 10, self.probe_list))
            if len(self.probe_list) == 0:
                return

            self.probe_list = sorted(self.probe_list, key=lambda x : x.key())
            # calculate interval between us sending probes out, to
            # spread the bandwidth usage out:1
            self.probe_interval = self.fleet_settings.probe_interval / len(self.probe_list)

        now = time.time()
        if now - self.last_probe_time < self.probe_interval:
            return
        self.last_probe_time = now

        # take one down
        (to_probe, *self.probe_list) = self.probe_list

        # pass it around
        self.send_probe_for_autopilot_version(to_probe)

    def config_filepath_mtime(self):
        '''return mtime of the current config file'''
        return os.path.getmtime(self.config_filepath)

    def check_config_need_reload(self):
        '''returns true if config should be reloaded from file on disk'''
        if self.config_filepath_bad:
            return

        if self.config_filepath != self.last_config_filepath:
            return True

        self.config_filepath_bad = False

        mtime = self.config_filepath_mtime()
        if mtime != self.last_config_filepath_mtime:
            return True

        return False

    def check_config_need_save(self):
        '''returns true if config should be saved to disk'''
        if self.config_filepath is None:
            return False

        if not os.path.exists(self.config_filepath):
            print("Creating (%s)" % self.config_filepath)
            self.config = self.new_config()
            return True

        if self.config is None:
            return False

        mtime = self.config_filepath_mtime()
        if mtime != self.last_config_change:
            return True

        return False

    def new_config(self):
        return {
            "version": 1,
            "classes": {"Vehicle": VehicleClass("Vehicle")},
            "vehicles": {},  # UID-keyed hash
        }

    def load_config_version_1(self, string):
        '''parses json string and returns a fleet configuration'''
        def oh(thing):
            '''an object handler for helping to deserialise config version 1'''
            if "__VehicleClass__" in thing:
                kwargs = {}
                if "paramfile" in thing:
                    kwargs["paramfile"] = thing["paramfile"]
                else:
                    kwargs["params"] = thing["params"]
                if "calibration_params" in thing:
                    kwargs["calibration_params"] = thing["calibration_params"]
                else:
                    kwargs["calibration_params"] = {}
                return VehicleClass(thing["name"], **kwargs)
            if "__Vehicle__" in thing:
                return Vehicle(thing["uid"], name=thing["name"], classes=thing["classes"])

            return thing
        return json.loads(string, object_hook=oh)

    def config_text_version_1(self):
        '''returns a json string for current configuration'''
        def oh(thing):
            '''an object handler for helping to serialise config version 1'''
            if isinstance(thing, VehicleClass):
                ret = {
                    "__VehicleClass__": 1,
                    "name": thing.name,
                    "calibration_params": thing.calibration_params,
                }
                if thing.paramfile is not None:
                    ret["paramfile"] = thing.paramfile
                else:
                    ret["params"] = thing.params
                return ret
            if isinstance(thing, Vehicle):
                ret = {
                    "__Vehicle__": 1,
                    "uid": thing.uid,
                    "name": thing.name,
                    "classes": thing._classes,
                }
                return ret
            return thing
        return json.dumps(self.config, default=oh)

    def check_config_load(self):
        if not self.check_config_need_reload():
            return

        self.config = None

        try:
            with open(self.config_filepath) as f:
                text = f.read()
            new_config = json.loads(text)
        except Exception as ex:
            self.print("Load exception: %s (path=%s)" % (str(ex), self.config_filepath))
            self.config_filepath_bad = True
            return
        self.print("Loaded config (%s)" % self.config_filepath)
        if new_config["version"] == 1:
            try:
                self.config = self.load_config_version_1(text)
            except Exception as ex:
                self.print("Load v1 exception: %s (path=%s)" % (str(ex), self.config_filepath))
                self.config_filepath_bad = True
                return
        self.last_config_filepath = self.config_filepath
        self.last_config_filepath_mtime = self.config_filepath_mtime()
        self.last_config_change = self.last_config_filepath_mtime

    def check_config(self):
        p = self.fleet_settings.config_filepath
        if p is None:
            self.config_filepath = None
        elif os.path.isabs(p):
            self.config_filepath = p
        elif self.mpstate.settings.state_basedir is not None:
            self.config_filepath = os.path.join(self.mpstate.settings.state_basedir, p)

        if self.config_filepath is None:
            return

        if not self.initial_backup_done:
            self.backup_config()
            self.initial_backup_done = True

        if self.check_config_need_save():
            text = self.config_text_version_1()
            try:
                with open(self.config_filepath, "w") as f:
                    f.write(text)
            except Exception as ex:
                self.print("Save : %s" % str(ex))
                return
            self.last_config_filepath_mtime = self.config_filepath_mtime()
            self.last_config_change = self.last_config_filepath_mtime

        self.check_config_load()

    def idle_task(self):
        self.check_config()

        if self.config is None:
            return

        self.do_probes()

    def handle_autopilot_version(self, m):
        uid = m.uid2  # lots of zeroes on the end after this

        key = (m.get_srcSystem(), m.get_srcComponent())

        if key not in self.seen_vehicles:
            self.seen_vehicles[key] = SeenVehicle(key)

        vehicle = self.seen_vehicles[key]
        vehicle.seen()

        vehicle.set_uid(uid)

        major = (m.flight_sw_version >> 24) & 0xff
        minor = (m.flight_sw_version >> 16) & 0xff
        patch = (m.flight_sw_version >> 8) & 0xff

        vehicle.autopilot_version_flight_sw_version = (major, minor, patch)

    def mavlink_packet(self, m):
        if self.config is None:
            return

        m_type = m.get_type()

        key = (m.get_srcSystem(), m.get_srcComponent())

        if m_type == 'AUTOPILOT_VERSION':
            self.handle_autopilot_version(m)

            # print("Received autopilot version from %u.%u: %s" %
            #       (m.get_srcSystem(), m.get_srcComponent(),
            #        vehicle.uid_as_string()))
            return

        if m_type == "HEARTBEAT":
            if m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # OK, that's really rude... FIXME
                return
            if key not in self.seen_vehicles:
                self.seen_vehicles[key] = SeenVehicle(key)
            self.seen_vehicles[key].seen()


def init(mpstate):
    '''initialise module'''
    return Fleet(mpstate)
