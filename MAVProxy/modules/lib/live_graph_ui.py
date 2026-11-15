from MAVProxy.modules.lib.live_graph import LiveGraph
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import icon
import time
import numpy, pylab
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigCanvas
from matplotlib.figure import Figure

class GraphFrame(wx.Frame):
    """ The main frame of the application
    """

    def __init__(self, state: LiveGraph):
        wx.Frame.__init__(self, None, -1, state.title)
        try:
            self.SetIcon(icon.SimpleIcon().get_ico())
        except Exception:
            pass
        self.state = state
        self.sample_dt  = float(getattr(state, "sample_dt", state.tickresolution))   
        self.refresh_dt = float(getattr(state, "refresh_dt", state.tickresolution)) * 10  

        self.paused = False
        self.clear_data = False

        self.create_main_panel()

        self.Bind(wx.EVT_IDLE, self.on_idle)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
        self.redraw_timer.Start(int(1000*self.refresh_dt))

        self.last_yrange = (None, None)

    def create_main_panel(self):
        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigCanvas(self.panel, -1, self.fig)

        self.close_button = wx.Button(self.panel, -1, "Close")
        self.Bind(wx.EVT_BUTTON, self.on_close_button, self.close_button)

        self.pause_button = wx.Button(self.panel, -1, "Pause")
        self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)

        self.clear_button = wx.Button(self.panel, -1, "Clear")
        self.Bind(wx.EVT_BUTTON, self.on_clear_button, self.clear_button)
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_clear_button, self.clear_button)

        did_one = False
        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        for button in self.close_button, self.pause_button, self.clear_button:
            if did_one:
                self.hbox1.Add(self.close_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
                self.hbox1.AddSpacer(1)
                did_one = True
            self.hbox1.Add(button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_LEFT | wx.TOP)

        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)

    def init_plot(self):
        self.dpi = 100
        self.fig = Figure((6.0, 3.0), dpi=self.dpi)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_facecolor('white')
  
        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # create X data
        self.max_points = int(self.state.timespan / self.sample_dt)
        self.half_points = self.max_points // 2
        self.write_idx = 0  
        self.xdata = numpy.arange(-self.state.timespan / 2, self.state.timespan / 2, self.sample_dt)[:self.max_points]
        self.axes.set_xbound(lower=self.xdata[0], upper=self.xdata[-1])

        self.ybuf = [numpy.full(self.max_points, numpy.nan) for _ in range(len(self.state.fields))]

        # plot the data as a line series, and save the reference
        # to the plotted line series
        #
        self.plot_data = []
        if len(self.ybuf[0]) == 0:
            max_y = min_y = 0
        else:
            max_y = min_y = self.ybuf[0][0]
        num_labels = 0 if not self.state.labels else len(self.state.labels)
        labels = []
        for i in range(len(self.ybuf)):
            if i < num_labels and self.state.labels and self.state.labels[i] is not None:
                label = self.state.labels[i]
            else:
                label = self.state.fields[i]
            labels.append(label)
            p = self.axes.plot(
                self.xdata, self.ybuf[i],
                linewidth=1,
                color=self.state.colors[i%len(self.state.colors)],
                label=label
                )[0]
            self.plot_data.append(p)
            if len(self.ybuf[i]) != 0:
                min_y = min(min_y, min(self.ybuf[i]))
                max_y = max(max_y, max(self.ybuf[i]))

        if min_y == max_y:
            self.axes.set_ybound(min_y, max_y+0.1)
        self.axes.legend(labels, loc='upper left', bbox_to_anchor=(0, 1.1))

    def draw_plot(self):
        # gather finite values to auto-scale Y
        finite_vals = []
        for arr in self.ybuf:
            vals = arr[~numpy.isnan(arr)]
            if vals.size:
                finite_vals.append((vals.min(), vals.max()))
        if not finite_vals:
            # nothing to show yet
            return

        vlow  = min(lo for lo, hi in finite_vals)
        vhigh = max(hi for lo, hi in finite_vals)
        if vlow == vhigh:
            pad = 0.1 * (abs(vlow) if vlow != 0 else 1.0)
            vlow -= pad
            vhigh += pad
        else:
            pad = 0.05 * (vhigh - vlow)
            vlow  -= pad
            vhigh += pad

        if (vlow, vhigh) != self.last_yrange:
            self.last_yrange = (vlow, vhigh)
            self.axes.set_ybound(lower=vlow, upper=vhigh)
            self.axes.grid(True, color='gray')
            pylab.setp(self.axes.get_xticklabels(), visible=True)
            leg = self.axes.get_legend()
            if leg:
                pylab.setp(leg.get_texts(), fontsize='small')

        # update lines in place
        for i, line in enumerate(self.plot_data):
            line.set_ydata(self.ybuf[i])

        self.canvas.draw()
        self.canvas.Refresh()

    def on_pause_button(self, event):
        self.paused = not self.paused

    def on_update_pause_button(self, event):
        label = "Resume" if self.paused else "Pause"
        self.pause_button.SetLabel(label)

    def on_update_clear_button(self, event):
        pass

    def on_clear_button(self, event):
        self.clear_data = True

    def on_close_button(self, event):
        self.redraw_timer.Stop()
        self.Destroy()

    def on_idle(self, event):
        time.sleep(self.refresh_dt*0.5)

    def on_redraw_timer(self, event):
        # if paused do not add data, but still redraw the plot
        # (to respond to scale modifications, grid change, etc.)
        #
        # print("redraw")
        state = self.state
        if state.close_graph.wait(0.001):
            self.redraw_timer.Stop()
            self.Destroy()
            return
        
        batch = []
        while state.child_pipe.poll():
            batch.append(state.child_pipe.recv())
            
        if self.paused:
            return

        if self.clear_data:
            self.clear_data = False
            for i in range(len(self.ybuf)):
                self.ybuf[i].fill(numpy.nan)
            self.write_idx = 0

        for msg in batch:
            if msg is None:
                print("msg is none")
                continue

            if self.write_idx >= self.max_points:
                # shift the right half to the left half, clear the right half
                for i in range(len(self.ybuf)):
                    self.ybuf[i][:self.half_points] = self.ybuf[i][self.half_points:]
                    self.ybuf[i][self.half_points:] = numpy.nan
                self.write_idx = self.half_points
                
            for i in range(len(self.plot_data)):
                val = msg[i] if i < len(msg) else None
                if isinstance(val, list):
                    print("ERROR: Cannot plot array of length %d. Use 'graph %s[index]' instead"%(len(val), state.fields[i]))
                    self.redraw_timer.Stop()
                    self.Destroy()
                    return
                if val is not None:
                    self.ybuf[i][self.write_idx] = val
            
            # write once per message, not once per field
            self.write_idx += 1


        for i in range(len(self.plot_data)):
            if len(self.ybuf[i]) < 2:
                print("return from here")
                return
            
        self.draw_plot()
