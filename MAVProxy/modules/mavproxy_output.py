#!/usr/bin/env python
'''enable run-time addition and removal of UDP clients , just like --out on the cnd line'''
''' TO USE:
    output add 10.11.12.13:14550
    output list
    output remove 3      # to remove 3rd output
'''

from pymavlink import mavutil


from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

class OutputModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OutputModule, self).__init__(mpstate, "output", "output control", public=True)
        self.add_command('output', self.cmd_output, "output control", [
            "<list|add|remove|sysid|mavfwd|mavfwd_disarmed>",
            'set_mavfwd (OUTPUT) <1|0>',
            'set_mavfwd_disarmed (OUTPUT) <1|0>',
        ])

    def cmd_output(self, args):
        '''handle output commands'''
        if len(args) < 1 or args[0] == "list":
            self.cmd_output_list()
        elif args[0] == "add":
            if len(args) != 2:
                print("Usage: output add OUTPUT")
                return
            self.cmd_output_add(args[1:])
        elif args[0] == "remove":
            if len(args) != 2:
                print("Usage: output remove OUTPUT")
                return
            self.cmd_output_remove(args[1:])
        elif args[0] == "sysid":
            if len(args) != 3:
                print("Usage: output sysid SYSID OUTPUT")
                return
            self.cmd_output_sysid(args[1:])
        elif args[0] == "set_mavfwd":
            self.cmd_set_mavfwd(args[1:])
        elif args[0] == "set_mavfwd_disarmed":
            self.cmd_set_mavfwd_disarmed(args[1:])
        else:
            print("usage: output <list|add|remove|sysid|set_mavfwd|set_mavfwd_disarmed>")

    def cmd_output_list(self):
        '''list outputs'''
        print("%u outputs" % len(self.mpstate.mav_outputs))
        for i in range(len(self.mpstate.mav_outputs)):
            conn = self.mpstate.mav_outputs[i]
            print("%u: %s" % (i, conn.address))
        if len(self.mpstate.sysid_outputs) > 0:
            print("%u sysid outputs" % len(self.mpstate.sysid_outputs))
            for sysid in self.mpstate.sysid_outputs:
                conn = self.mpstate.sysid_outputs[sysid]
                print("%u: %s" % (sysid, conn.address))

    class ConnectionAttributes():
        pass

    def cmd_output_add(self, args):
        '''add new output'''
        device = args[0]
        print("Adding output %s" % device)
        try:
            conn = mavutil.mavlink_connection(device, input=False, source_system=self.settings.source_system)
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print("Failed to connect to %s" % device)
            return
        self.mpstate.mav_outputs.append(conn)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        conn.mavproxy_attributes = OutputModule.ConnectionAttributes()

    def cmd_output_sysid(self, args):
        '''add new output for a specific MAVLink sysID'''
        sysid = int(args[0])
        device = args[1]
        print("Adding output %s for sysid %u" % (device, sysid))
        try:
            conn = mavutil.mavlink_connection(device, input=False, source_system=self.settings.source_system)
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print("Failed to connect to %s" % device)
            return
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        if sysid in self.mpstate.sysid_outputs:
            self.mpstate.sysid_outputs[sysid].close()
        self.mpstate.sysid_outputs[sysid] = conn

    def find_output(self, device):
        for i in range(len(self.mpstate.mav_outputs)):
            conn = self.mpstate.mav_outputs[i]
            if str(i) == device or conn.address == device:
                return conn, i
        return None, None

    def cmd_output_remove(self, args):
        '''remove an output'''
        device = args[0]
        conn, i = self.find_output(device)
        if conn is None:
            return

        print("Removing output %s" % conn.address)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        conn.close()
        self.mpstate.mav_outputs.pop(i)

    def cmd_set_mavfwd(self, args):
        self.set_mavfwd_attribute('set_mavfwd', 'mavproxy_mavfwd', args)

    def cmd_set_mavfwd_disarmed(self, args):
        self.set_mavfwd_attribute('set_mavfwd_attribute', 'mavproxy_mavfwd_disarmed', args)

    def set_conn_attribute(self, conn, attribute, value):
        if not hasattr(conn, "mavproxy_attributes"):
            conn.mavproxy_attributes = OutputModule.ConnectionAttributes()
        setattr(conn.mavproxy_attributes, attribute, bool(value))

    def set_mavfwd_attribute(self, cmd, attribute, args):
        if len(args) != 2:
            print("Usage: output %s OUTPUT <0|1>" % cmd)
            return
        (device, value) = args
        if str(value) not in frozenset(["0", "1"]):
            print("Usage: output %s OUTPUT <0|1>" % cmd)
            return
        output, _ = self.find_output(device)
        if output is None:
            print("Bad OUTPUT")
            return

        # same conversions as in mp_settings.py:
        if str(value).lower() in ['1', 'true', 'yes']:
            value = True
        elif str(value).lower() in ['0', 'false', 'no']:
            value = False

        self.set_conn_attribute(output, attribute, bool(value))

    def idle_task(self):
        '''called on idle'''
        for m in self.mpstate.mav_outputs:
            m.source_system = self.settings.source_system
            m.mav.srcSystem = m.source_system
            m.mav.srcComponent = self.settings.source_component

def init(mpstate):
    '''initialise module'''
    return OutputModule(mpstate)
