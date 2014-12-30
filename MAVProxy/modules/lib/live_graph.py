#!/usr/bin/env python

"""

  MAVProxy realtime graphing module, partly based on the wx graphing
  demo by Eli Bendersky (eliben@gmail.com)

  http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt
"""

from MAVProxy.modules.lib import mp_util

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
                 colors=[ 'red', 'green', 'blue', 'orange', 'olive', 'cyan', 'magenta', 'brown', 'darkgreen',
                          'violet', 'purple', 'grey', 'black']):
        import multiprocessing
        self.fields = fields
        self.colors = colors
        self.title  = title
        self.timespan = timespan
        self.tickresolution = tickresolution
        self.values = [None]*len(self.fields)

        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_graph = multiprocessing.Event()
        self.close_graph.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()
        import wx, matplotlib
        matplotlib.use('WXAgg')
        app = wx.PySimpleApp()
        app.frame = GraphFrame(state=self)
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

import wx

class GraphFrame(wx.Frame):
    """ The main frame of the application
    """

    def __init__(self, state):
        wx.Frame.__init__(self, None, -1, state.title)
        self.state = state
        self.data = []
        for i in range(len(state.fields)):
            self.data.append([])
        self.paused = False

        self.create_main_panel()

        self.Bind(wx.EVT_IDLE, self.on_idle)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
        self.redraw_timer.Start(1000*self.state.tickresolution)

    def create_main_panel(self):
        from matplotlib.backends.backend_wxagg import \
             FigureCanvasWxAgg as FigCanvas
        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigCanvas(self.panel, -1, self.fig)


        self.close_button = wx.Button(self.panel, -1, "Close")
        self.Bind(wx.EVT_BUTTON, self.on_close_button, self.close_button)

        self.pause_button = wx.Button(self.panel, -1, "Pause")
        self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.Add(self.close_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
        self.hbox1.AddSpacer(1)
        self.hbox1.Add(self.pause_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_LEFT | wx.TOP)

        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)

    def init_plot(self):
        self.dpi = 100
        import pylab, numpy
        from matplotlib.figure import Figure
        self.fig = Figure((6.0, 3.0), dpi=self.dpi)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('white')

        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference
        # to the plotted line series
        #
        self.plot_data = []
        if len(self.data[0]) == 0:
            max_y = min_y = 0
        else:
            max_y = min_y = self.data[0][0]
        for i in range(len(self.data)):
            p = self.axes.plot(
                self.data[i],
                linewidth=1,
                color=self.state.colors[i],
                label=self.state.fields[i],
                )[0]
            self.plot_data.append(p)
            if len(self.data[i]) != 0:
                min_y = min(min_y, min(self.data[i]))
                max_y = max(max_y, max(self.data[i]))

        # create X data
        self.xdata = numpy.arange(-self.state.timespan, 0, self.state.tickresolution)
        self.axes.set_xbound(lower=self.xdata[0], upper=0)
        if min_y == max_y:
            self.axes.set_ybound(min_y, max_y+0.1)
        self.axes.legend(self.state.fields, loc='upper left', bbox_to_anchor=(0, 1.1))

    def draw_plot(self):
        """ Redraws the plot
        """
        import numpy, pylab
        state = self.state

        if len(self.data[0]) == 0:
            print("no data to plot")
            return
        vhigh = max(self.data[0])
        vlow  = min(self.data[0])

        for i in range(1,len(self.plot_data)):
            vhigh = max(vhigh, max(self.data[i]))
            vlow  = min(vlow,  min(self.data[i]))
        ymin = vlow  - 0.05*(vhigh-vlow)
        ymax = vhigh + 0.05*(vhigh-vlow)

        if ymin == ymax:
            ymax = ymin + 0.1
            ymin = ymin - 0.1
        self.axes.set_ybound(lower=ymin, upper=ymax)
        self.axes.grid(True, color='gray')
        pylab.setp(self.axes.get_xticklabels(), visible=True)
        pylab.setp(self.axes.get_legend().get_texts(), fontsize='small')

        for i in range(len(self.plot_data)):
            ydata = numpy.array(self.data[i])
            xdata = self.xdata
            if len(ydata) < len(self.xdata):
                xdata = xdata[-len(ydata):]
            self.plot_data[i].set_xdata(xdata)
            self.plot_data[i].set_ydata(ydata)

        self.canvas.draw()

    def on_pause_button(self, event):
        self.paused = not self.paused

    def on_update_pause_button(self, event):
        label = "Resume" if self.paused else "Pause"
        self.pause_button.SetLabel(label)

    def on_close_button(self, event):
        self.redraw_timer.Stop()
        self.Destroy()

    def on_idle(self, event):
        import time
        time.sleep(self.state.tickresolution*0.5)

    def on_redraw_timer(self, event):
        # if paused do not add data, but still redraw the plot
        # (to respond to scale modifications, grid change, etc.)
        #
        state = self.state
        if state.close_graph.wait(0.001):
            self.redraw_timer.Stop()
            self.Destroy()
            return
        while state.child_pipe.poll():
            state.values = state.child_pipe.recv()
        if self.paused:
            return
        for i in range(len(self.plot_data)):
            if state.values[i] is not None:
                self.data[i].append(state.values[i])
                while len(self.data[i]) > len(self.xdata):
                    self.data[i].pop(0)

        for i in range(len(self.plot_data)):
            if state.values[i] is None or len(self.data[i]) < 2:
                return
        self.draw_plot()

if __name__ == "__main__":
    # test the graph
    import time, math
    livegraph = LiveGraph(['sin(t)', 'cos(t)', 'sin(t+1)',
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
