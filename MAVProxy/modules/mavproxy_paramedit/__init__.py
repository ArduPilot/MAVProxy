#!/usr/bin/env python
'''
param editor module
Akshath Singhal
June 2019
'''

from MAVProxy.modules.lib import mp_module


class ParamEditorModule(mp_module.MPModule):
    '''
    A Graphical parameter editor for use with MAVProxy
    '''
    def __init__(self, mpstate):
        super(ParamEditorModule, self).__init__(mpstate,
                                                "paramedit", "param edit",
                                                public=True)

        # to work around an issue on MacOS this module is a thin wrapper
        # around a separate ParamEditorMain object
        self.pe_main = None
        self.mpstate = mpstate

    def unload(self):
        '''unload module'''
        if self.pe_main:
            self.pe_main.unload()

    def idle_task(self):
        if not self.pe_main:
            # wait for parameter module to load
            if self.module('param') is None:
                return
            from MAVProxy.modules.mavproxy_paramedit import param_editor
            self.pe_main = param_editor.ParamEditorMain(self.mpstate)
        if self.pe_main:
            if self.pe_main.needs_unloading:
                self.needs_unloading = True
        self.pe_main.idle_task()

    def mavlink_packet(self, m):
        if self.pe_main:
            self.pe_main.mavlink_packet(m)


def init(mpstate):
    '''initialise module'''
    return ParamEditorModule(mpstate)
