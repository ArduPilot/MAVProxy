#!/usr/bin/env python
'''window layout command handling'''

from MAVProxy.modules.lib import mp_module

class LayoutModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(LayoutModule, self).__init__(mpstate, "layout", "window layout handling", public = False)
        self.add_command('layout', self.cmd_layout,
                         'window layout management',
                         ["<save|load>"])

    def cmd_layout(self, args):
        '''handle layout command'''
        from MAVProxy.modules.lib import win_layout
        if len(args) < 1:
            print("usage: layout <save|load>")
            return
        if args[0] == "load":
            win_layout.load_layout(self.mpstate.settings.vehicle_name)
        elif args[0] == "save":
            win_layout.save_layout(self.mpstate.settings.vehicle_name)

def init(mpstate):
    '''initialise module'''
    return LayoutModule(mpstate)
