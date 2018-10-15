#!/usr/bin/env python

"""

  MAVProxy realtime graphing module, partly based on the wx graphing
  demo by Eli Bendersky (eliben@gmail.com)

  http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt
"""

import platform
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc

class LiveGraph():
    '''
    a live graph object using wx and matplotlib
    All of the GUI work is done in a child process to provide some insulation
    from the parent mavproxy instance and prevent instability in the GCS

    New data is sent to the LiveGraph instance via a pipe
    '''
    def __init__(self,
                 fields,
                 title='MAVProxy: LiveGraph',
                 timespan=20.0,
                 tickresolution=0.2,
                 colors=[ 'red', 'green', 'blue', 'orange', 'olive', 'cyan', 'magenta', 'brown',
                          'violet', 'purple', 'grey', 'black']):
        self.fields = fields
        self.colors = colors
        self.title  = title
        self.timespan = timespan
        self.tickresolution = tickresolution
        self.values = [None]*len(self.fields)
        self.parent_pipe,self.child_pipe = multiproc.Pipe()
        self.close_graph = multiproc.Event()
        self.close_graph.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()

        import matplotlib, platform
        if platform.system() != "Darwin":
            # on MacOS we can't set WxAgg here as it conflicts with the MacOS version
            matplotlib.use('WXAgg')

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx

        app = wx.App(False)
        from MAVProxy.modules.lib import live_graph_ui
        app.frame = live_graph_ui.GraphFrame(state=self)
        app.frame.Show()
        app.MainLoop()
        
    def add_values(self, values):
        '''add some data to the graph'''
        if self.child.is_alive():
            self.parent_pipe.send(values)

    def close(self):
        '''close the graph'''
        self.close_graph.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if graph is still going'''
        return self.child.is_alive()


if __name__ == "__main__":
    multiproc.freeze_support()
    # test the graph
    import time, math
    import live_graph
    livegraph = live_graph.LiveGraph(['sin(t)', 'cos(t)', 'sin(t+1)',
                           'cos(t+1)', 'sin(t+2)', 'cos(t+2)',
                           'cos(t+1)', 'sin(t+2)', 'cos(t+2)', 'x'],
                          timespan=30,
                          title='Graph Test')
    while livegraph.is_alive():
        t = time.time()
        livegraph.add_values([math.sin(t), math.cos(t),
                              math.sin(t+1), math.cos(t+1),
                              math.sin(t+1), math.cos(t+1),
                              math.sin(t+1), math.cos(t+1),
                              math.sin(t+2), math.cos(t+2)])
        time.sleep(0.05)
