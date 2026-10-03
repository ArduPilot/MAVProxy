#!/usr/bin/env python3
'''
mission editor module
Michael Day
June 2104
'''

from MAVProxy.modules.lib import mp_module


def get_mission_for_map(mpstate):
    '''Use the editor's draft for display without changing vehicle transfers.'''
    editor = mpstate.module('misseditor')
    if editor is not None:
        draft = editor.me_main.get_map_mission()
        if draft is not None:
            return draft
    wp = mpstate.module('wp')
    return wp.wploader if wp is not None else None


class MissionEditorModule(mp_module.MPModule):
    '''
    A Mission Editor for use with MAVProxy
    '''
    def __init__(self, mpstate):
        super(MissionEditorModule, self).__init__(mpstate, "misseditor", "mission editor", public = True)

        # to work around an issue on MacOS this module is a thin wrapper around a separate MissionEditorMain object
        from MAVProxy.modules.mavproxy_misseditor import mission_editor
        self.me_main = mission_editor.MissionEditorMain(mpstate, self.module('terrain').ElevationModel.database)

    def unload(self):
        '''unload module'''
        self.me_main.unload()

    def idle_task(self):
        self.me_main.idle_task()
        if self.me_main.needs_unloading:
            self.needs_unloading = True

    def mavlink_packet(self, m):
        self.me_main.mavlink_packet(m)

    def click_updated(self):
        self.me_main.update_map_click_position(self.mpstate.click_location)

def init(mpstate):
    '''initialise module'''
    return MissionEditorModule(mpstate)
