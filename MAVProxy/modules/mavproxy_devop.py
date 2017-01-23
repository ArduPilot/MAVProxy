#!/usr/bin/env python
'''remote low level device operations'''

import time, os, sys
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class DeviceOpModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DeviceOpModule, self).__init__(mpstate, "DeviceOp")
        self.add_command('devop', self.cmd_devop, "device operations",
                         ["<read|write> <spi|i2c>"])
        self.request_id = 1

    def cmd_devop(self, args):
        '''device operations'''
        usage = "Usage: devop <read|write> <spi|i2c> name bus address"
        if len(args) < 5:
            print(usage)
            return

        if args[1] == 'spi':
            bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_SPI
        elif args[1] == 'i2c':
            bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_I2C
        else:
            print(usage)

        if args[0] == 'read':
            self.devop_read(args[2:], bustype)
        elif args[0] == 'write':
            self.devop_write(args[2:], bustype)
        else:
            print(usage)

    def devop_read(self, args, bustype):
        '''read from device'''
        if len(args) < 5:
            print("Usage: devop read <spi|i2c> name bus address regstart count")
            return
        name = args[0]
        bus = int(args[1],base=0)
        address = int(args[2],base=0)
        reg = int(args[3],base=0)
        count = int(args[4],base=0)
        self.master.mav.device_op_read_send(self.target_system,
                                            self.target_component,
                                            self.request_id,
                                            bustype,
                                            bus,
                                            address,
                                            name,
                                            reg,
                                            count)
        self.request_id += 1

    def devop_write(self, args, bustype):
        '''write to a device'''
        usage = "Usage: devop write <spi|i2c> name bus address regstart count <bytes>"
        if len(args) < 5:
            print(usage)
            return
        name = args[0]
        bus = int(args[1],base=0)
        address = int(args[2],base=0)
        reg = int(args[3],base=0)
        count = int(args[4],base=0)
        args = args[5:]
        if len(args) < count:
            print(usage)
            return
        bytes = [0]*128
        for i in range(count):
            bytes[i] = int(args[i],base=0)
        self.master.mav.device_op_write_send(self.target_system,
                                             self.target_component,
                                             self.request_id,
                                             bustype,
                                             bus,
                                             address,
                                             name,
                                             reg,
                                             count,
                                             bytes)
        self.request_id += 1

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "DEVICE_OP_READ_REPLY":
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK: %u bytes" % (m.request_id, m.count))
                for i in range(m.count):
                    reg = i + m.regstart
                    sys.stdout.write("%02x:%02x " % (reg, m.data[i]))
                    if (i+1) % 16 == 0:
                        print("")
                if m.count % 16 != 0:
                    print("")

        if mtype == "DEVICE_OP_WRITE_REPLY":
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK" % m.request_id)

def init(mpstate):
    '''initialise module'''
    return DeviceOpModule(mpstate)
