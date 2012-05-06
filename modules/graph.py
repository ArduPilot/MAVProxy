"""
MAVProxy realtime graphing module, partly based on the wx graphing
demo by Eli Bendersky (eliben@gmail.com)

  http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt

"""

import time, mavutil, re, os, sys, wx

mpstate = None


class graph_state(object):
    def __init__(self):
        self.fields = []
        self.field_types = []
        self.colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey' ]
        self.msg_types = set()
        self.num_points = 100
        self.value = None
        self.graph_thread = None

def name():
    '''return module name'''
    return "graph"

def description():
    '''return module description'''
    return "graph control"

def cmd_graph(args):
    '''graph command'''
    import threading
    state = mpstate.graph_state

    if state.graph_thread is not None:
        state.close_graph.set()
        state.graph_thread.join(2.0)
        state.graph_thread = None
        
    state.fields = args[:]
    re_caps = re.compile('[A-Z_]+')
    for f in state.fields:
        caps = set(re.findall(re_caps, f))
        state.msg_types = state.msg_types.union(caps)
        state.field_types.append(caps)
    if not state.fields:
        return
    print("Adding graph: %s" % mpstate.graph_state.fields)

    state.value = []
    for i in range(len(state.fields)):
        f = state.fields[i]
        state.value.append(mavutil.evaluate_expression(f, mpstate.master().messages))

    state.close_graph = threading.Event()

    app = wx.PySimpleApp()
    app.frame = GraphFrame()
    app.frame.Show()
    state.graph_thread = threading.Thread(target=app.MainLoop)
    state.graph_thread.daemon = True
    state.graph_thread.start()

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.graph_state = graph_state()
    mpstate.command_map['graph'] = (cmd_graph, "live graphing")
    print("graph initialised")

def unload():
    '''unload module'''
    state = mpstate.graph_state
    state.close_graph.set()
    if state.graph_thread is not None:
        state.graph_thread.join(2.0)

def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    state = mpstate.graph_state
    if state.fields:
        mtype = msg.get_type()
        if mtype not in state.msg_types:
            return
        for i in range(len(state.fields)):
            if mtype not in state.field_types[i]:
                continue
            f = state.fields[i]
            state.value[i] = mavutil.evaluate_expression(f, mpstate.master().messages)

# The recommended way to use wx with mpl is with the WXAgg
# backend. 
#
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
import numpy
import pylab

class GraphFrame(wx.Frame):
    """ The main frame of the application
    """
    title = 'MAVProxy: graph'
    
    def __init__(self):
        state = mpstate.graph_state
        wx.Frame.__init__(self, None, -1, self.title)
        self.data = []
        for i in range(len(state.fields)):
            self.data.append([])
        self.paused = False
        
        self.create_main_panel()
        
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.redraw_timer.Start(200)

    def create_main_panel(self):
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
        state = mpstate.graph_state

        self.dpi = 100
        self.fig = Figure((6.0, 3.0), dpi=self.dpi)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        self.axes.set_title(' '.join(state.fields), size=12)
        
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
                color=state.colors[i],
                )[0]
            self.plot_data.append(p)

    def draw_plot(self):
        """ Redraws the plot
        """
        state = mpstate.graph_state

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
        state = mpstate.graph_state
        if state.close_graph.wait(0.001):
            self.redraw_timer.Stop()
            self.Destroy()
            return
        if not self.paused:
            for i in range(len(self.plot_data)):
                if state.value[i] is not None:
                    self.data[i].append(state.value[i])
                    if len(self.data[i]) > state.num_points:
                        self.data[i] = self.data[i][len(self.data)-state.num_points:]
        
        self.draw_plot()
    
