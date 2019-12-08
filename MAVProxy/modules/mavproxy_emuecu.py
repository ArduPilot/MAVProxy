"""
control EMU ECU system
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class EMUECUModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(EMUECUModule, self).__init__(mpstate, "emuecu", "emuecu", public=False)
        self.emuecu_settings = mp_settings.MPSettings(
            [('port', int, 102)])
        self.add_command('emu', self.cmd_emu, 'EMUECU control',
                         ["<send>",
                          "set (EMUECUSETTING)"])
        self.add_completion_function('(EMUECUSETTING)',
                                     self.emuecu_settings.completion)

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if msg.get_type() == 'SERIAL_CONTROL':
            print(msg)

    def cmd_emu(self, args):
        '''emu command handling'''
        if len(args) <= 0:
            print("Usage: emu <send|set>")
            return
        if args[0] == "send":
            self.cmd_send(args[1:])
        elif args[0] == "set":
            self.emuecu_settings.command(args[1:])

    def cmd_send(self, args):
        '''send command'''
        cmd = ' '.join(args) + '\n'
        buf = [ord(x) for x in cmd]
        buf.extend([0]*(70-len(buf)))
        mav = self.master.mav
        mav.serial_control_send(self.emuecu_settings.port,
                                0,
                                0, 0,
                                len(cmd), buf)


def init(mpstate):
    '''initialise module'''
    return EMUECUModule(mpstate)
