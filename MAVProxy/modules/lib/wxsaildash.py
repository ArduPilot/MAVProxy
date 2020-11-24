#!/usr/bin/env python

"""
  MAVProxy sailing dashboard
"""

from MAVProxy.modules.lib import multiproc
import time

class SailingDashboard(object):
    '''
    A sailing dashboard for MAVProxy
    '''

    def __init__(self, title="MAVProxy: Sailing Dashboard"):
        self.title = title

        # create a pipe for communication from the module to the GUI
        self.child_pipe_recv, self.parent_pipe_send = multiproc.Pipe(duplex=False)
        self.close_event = multiproc.Event()
        self.close_event.clear()

        # create and start the child process
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

        # prevent the parent from using the child connection
        self.child_pipe_recv.close()

    def child_task(self):
        '''The child process hosts the GUI elements'''

        # prevent the child from using the parent connection
        self.parent_pipe_send.close()

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.lib.wxsaildash_ui import SailingDashboardFrame 

        # create the wx application and pass self as the state
        app = wx.App()
        app.frame = SailingDashboardFrame(state=self, title=self.title, size=(800, 300))
        app.frame.SetDoubleBuffered(True)
        app.frame.Show()
        app.MainLoop()

        # trigger a close event when the main app window is closed.
        # the event is monitored by the MAVProxy module which will
        # flag the module for unloading
        self.close_event.set()

    def close(self):
        '''Close the GUI'''

        # trigger a close event which is monitored by the 
        # child gui process - it will close allowing the 
        # process to be joined
        self.close_event.set()
        if self.is_alive():
            self.child.join(timeout=2.0)

    def is_alive(self):
        '''Check if the GUI process is alive'''

        return self.child.is_alive()

if __name__ == "__main__":
    '''A stand alone test for the sailing dashboard'''

    multiproc.freeze_support()
    sail_dash = SailingDashboard()
    while sail_dash.is_alive():
        print('sailing dashboard is alive')
        time.sleep(0.5)


