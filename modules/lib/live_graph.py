"""

  MAVProxy realtime graphing module, partly based on the wx graphing
  demo by Eli Bendersky (eliben@gmail.com)

  http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt
"""

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
                 num_points=100,
                 colors=[ 'red', 'green', 'blue', 'orange', 'olive']):
        import multiprocessing
        self.fields = fields
        self.colors = colors
        self.title  = title
        self.num_points = num_points
        self.values = [None]*len(self.fields)

        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_graph = multiprocessing.Event()
        self.close_graph.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
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
        if self.child.is_alive():
            self.child.join(2)

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
        
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.redraw_timer.Start(200)

    def create_main_panel(self):
        from matplotlib.backends.backend_wxagg import \
             FigureCanvasWxAgg as FigCanvas
        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigCanvas(self.panel, -1, self.fig)

        self.pause_button = wx.Button(self.panel, -1, "Pause")
        self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)
        
        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.Add(self.pause_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
        self.hbox1.AddSpacer(20)
        
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)        
        self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_LEFT | wx.TOP)
        
        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)
    
    def init_plot(self):
        self.dpi = 100
        import pylab
        from matplotlib.figure import Figure
        self.fig = Figure((6.0, 3.0), dpi=self.dpi)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        self.axes.set_title(' '.join(self.state.fields), size=12)
        
        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference 
        # to the plotted line series
        #
        self.plot_data = []
        for i in range(len(self.data)):
            p = self.axes.plot(
                self.data[i], 
                linewidth=1,
                color=self.state.colors[i],
                )[0]
            self.plot_data.append(p)

    def draw_plot(self):
        """ Redraws the plot
        """
        import numpy, pylab
        state = self.state

        vhigh = max(self.data[0])
        vlow  = min(self.data[0])

        for i in range(len(self.plot_data)):
            vhigh = max(vhigh, max(self.data[i]))
            vlow  = min(vlow,  min(self.data[i]))
        ymin = vlow  - 0.05*(vhigh-vlow)
        ymax = vhigh + 0.05*(vhigh-vlow)

        self.axes.set_xbound(lower=-state.num_points, upper=0)
        self.axes.set_ybound(lower=ymin, upper=ymax)
        self.axes.grid(True, color='gray')
        pylab.setp(self.axes.get_xticklabels(), visible=True)
            
        for i in range(len(self.plot_data)):
            self.plot_data[i].set_xdata(numpy.arange(-len(self.data[0]),0))
            self.plot_data[i].set_ydata(numpy.array(self.data[i]))
        
        self.canvas.draw()
    
    def on_pause_button(self, event):
        self.paused = not self.paused
    
    def on_update_pause_button(self, event):
        label = "Resume" if self.paused else "Pause"
        self.pause_button.SetLabel(label)
    
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
            state.value = state.child_pipe.recv()
        if not self.paused:
            for i in range(len(self.plot_data)):
                if state.value[i] is not None:
                    self.data[i].append(state.value[i])
                    if len(self.data[i]) > state.num_points:
                        self.data[i] = self.data[i][len(self.data)-state.num_points:]

        for i in range(len(self.plot_data)):
            if state.value[i] is None or len(self.data[i]) < 2:
                return
        self.draw_plot()
    
