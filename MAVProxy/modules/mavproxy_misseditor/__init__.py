#!/usr/bin/env python
'''
mission editor module
Michael Day
June 2104
'''

from MAVProxy.modules.lib import mp_module

class MissionEditorModule(mp_module.MPModule):
    '''
    A Mission Editor for use with MAVProxy
    '''
    def __init__(self, mpstate):
        super(MissionEditorModule, self).__init__(mpstate, "misseditor", "mission editor", public = True)

        # to work around an issue on MacOS this module is a thin wrapper around a separate MissionEditorMain object
        from MAVProxy.modules.mavproxy_misseditor import mission_editor
        self.me_main = mission_editor.MissionEditorMain(mpstate)

    def unload(self):
        '''unload module'''
        self.me_main.unload()

    def idle_task(self):
        self.me_main.idle_task()

    def mavlink_packet(self, m):
        self.me_main.mavlink_packet(m)

    def update_map_click_position(self, new_click_pos):
        self.me_main.update_map_click_position(new_click_pos)

def init(mpstate):
    '''initialise module'''
    return MissionEditorModule(mpstate)
