# from MAVProxy.modules.lib.wx_loader import wx
# from MAVProxy.modules.lib import icon
# import time
# import math
# from collections import deque
# import numpy as np

# class GraphFrame(wx.Frame):
#     """
#     wxPython frame embedding a Matplotlib plot that:
#       - starts with a time window including +future_pad_s seconds into the future,
#       - fills points left→right without moving the x-axis,
#       - when the right edge is reached, shifts the window forward by page_step_s,
#         keeping the x-span constant.
#     """

#     def __init__(self, state):
#         print("ZARI: GraphFrame Constructor")
#         wx.Frame.__init__(self, None, -1, state.title)
#         try:
#             self.SetIcon(icon.SimpleIcon().get_ico())
#         except Exception:
#             pass

#         # ====== config/state ======
#         self.state = state
#         self.tick_s = float(getattr(state, "tickresolution", 0.1))     # seconds between timer ticks
#         self.span_s = float(getattr(state, "timespan", 30.0))          # total visible x-span
#         self.future_pad_s = float(getattr(state, "future_pad_s", 10.0))# initial +future
#         self.page_step_s = float(getattr(state, "page_step_s", 5.0))   # jump size when reaching right edge

#         n_series = len(state.fields)
#         self.t_buf = deque()                                           # timestamps (epoch seconds)
#         self.y_bufs = [deque() for _ in range(n_series)]               # one deque per series

#         # window_start/end are absolute timestamps
#         now = time.time()
#         self.x_end = now + self.future_pad_s
#         self.x_start = self.x_end - self.span_s

#         self.paused = False
#         self.clear_requested = False
#         self.last_yrange = (None, None)

#         self.create_main_panel()

#         # drive updates with a timer
#         self.redraw_timer = wx.Timer(self)
#         self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
#         self.redraw_timer.Start(int(1000 * self.tick_s))

#     # ---------- UI ----------

#     def create_main_panel(self):
#         print("ZARI: create_main_panel")
#         import platform
#         if platform.system() == 'Darwin':
#             from MAVProxy.modules.lib.MacOS import backend_wxagg
#             FigCanvas = backend_wxagg.FigureCanvasWxAgg
#         else:
#             from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigCanvas

#         from matplotlib.figure import Figure

#         self.panel = wx.Panel(self)

#         # Figure/Axes
#         self.dpi = 100
#         self.fig = Figure((6.0, 3.0), dpi=self.dpi)
#         self.axes = self.fig.add_subplot(111)
#         # facecolor compat
#         if hasattr(self.axes, "set_facecolor"):
#             self.axes.set_facecolor("white")
#         else:
#             self.axes.set_axis_bgcolor("white")

#         # pre-create lines (empty) for each series
#         self.lines = []
#         labels = []
#         num_labels = len(getattr(self.state, "labels", []) or [])
#         for i in range(len(self.y_bufs)):
#             label = self.state.labels[i] if (i < num_labels and self.state.labels[i]) else self.state.fields[i]
#             color = self.state.colors[i % len(self.state.colors)]
#             line, = self.axes.plot([], [], linewidth=1, color=color, label=label)
#             self.lines.append(line)
#             labels.append(label)

#         self.axes.legend(labels, loc='upper left', bbox_to_anchor=(0, 1.1))
#         for lbl in self.axes.get_xticklabels() + self.axes.get_yticklabels():
#             lbl.set_fontsize(8)

#         # set initial x-limits (absolute time in seconds)
#         self.axes.set_xbound(lower=self.x_start, upper=self.x_end)
#         self.axes.grid(True)

#         # Canvas
#         self.canvas = FigCanvas(self.panel, -1, self.fig)

#         # Controls
#         self.close_button = wx.Button(self.panel, -1, "Close")
#         self.pause_button = wx.Button(self.panel, -1, "Pause")
#         self.clear_button = wx.Button(self.panel, -1, "Clear")

#         self.Bind(wx.EVT_BUTTON, self.on_close_button, self.close_button)
#         self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
#         self.Bind(wx.EVT_BUTTON, self.on_clear_button, self.clear_button)
#         self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)

#         # Layout
#         hbox = wx.BoxSizer(wx.HORIZONTAL)
#         for i, button in enumerate((self.close_button, self.pause_button, self.clear_button)):
#             if i > 0:
#                 hbox.AddSpacer(5)
#             hbox.Add(button, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)

#         vbox = wx.BoxSizer(wx.VERTICAL)
#         vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
#         vbox.Add(hbox, 0, flag=wx.ALIGN_LEFT | wx.TOP)

#         self.panel.SetSizer(vbox)
#         vbox.Fit(self)

#     # ---------- Data & drawing ----------

#     def _append_sample(self, t_epoch, values):
#         """Append one sample (timestamp + per-series values)."""
#         # initialize deques' maxlen lazily once we know effective capacity
#         # capacity ~= enough points to cover span_s at tick_s cadence, plus a small headroom
#         if getattr(self, "_maxlen", None) is None:
#             cap = int(math.ceil(self.span_s / self.tick_s)) + 64
#             self._maxlen = cap
#             self.t_buf = deque(maxlen=cap)
#             self.y_bufs = [deque(maxlen=cap) for _ in self.y_bufs]

#         self.t_buf.append(t_epoch)
#         for i, y in enumerate(self.y_bufs):
#             val = values[i]
#             if isinstance(val, list):
#                 # same guard as before
#                 raise ValueError(
#                     f"Cannot plot array of length {len(val)} for {self.state.fields[i]}. "
#                     f"Use 'graph {self.state.fields[i]}[index]' instead."
#                 )
#             y.append(val)

#     def _visible_slice(self):
#         """Return numpy arrays (tx, ys) filtered to current [x_start, x_end] window."""
#         if not self.t_buf:
#             return None, None

#         # Convert to numpy for fast slicing
#         tx = np.fromiter(self.t_buf, dtype=float)
#         mask = (tx >= self.x_start) & (tx <= self.x_end)
#         if not mask.any():
#             return None, None

#         txv = tx[mask]
#         ysv = []
#         for ybuf in self.y_bufs:
#             y = np.fromiter(ybuf, dtype=float)
#             ysv.append(y[mask])
#         return txv, ysv

#     def _maybe_advance_window(self, latest_t):
#         """
#         If the newest timestamp reached/passed the right edge,
#         jump the window forward by page_step_s (keeping span constant).
#         """
#         if latest_t >= self.x_end:
#             self.x_start += self.page_step_s
#             self.x_end += self.page_step_s
#             self.axes.set_xbound(lower=self.x_start, upper=self.x_end)

#     def _autoscale_y_from_visible(self, ysv):
#         """Autoscale Y based on currently visible data."""
#         vals = np.concatenate([y for y in ysv if y.size > 0]) if ysv else np.array([])
#         if vals.size == 0:
#             return
#         vmin = float(np.nanmin(vals))
#         vmax = float(np.nanmax(vals))
#         if not np.isfinite(vmin) or not np.isfinite(vmax):
#             return
#         if vmin == vmax:
#             pad = 0.1 if vmin == 0 else abs(vmin) * 0.1
#             vmin -= pad
#             vmax += pad
#         # avoid excessive ybound churn
#         if (vmin, vmax) != self.last_yrange:
#             self.last_yrange = (vmin, vmax)
#             self.axes.set_ybound(lower=vmin, upper=vmax)

#     def draw_plot(self):
#         """Update line data and request a draw (no axis motion unless paging)."""
#         tx, ysv = self._visible_slice()
#         if tx is None:
#             self.canvas.draw_idle()
#             return

#         for line, y in zip(self.lines, ysv):
#             line.set_xdata(tx)
#             line.set_ydata(y)

#         self._autoscale_y_from_visible(ysv)
#         self.canvas.draw_idle()

#     # ---------- Event handlers ----------

#     def on_pause_button(self, event):
#         self.paused = not self.paused

#     def on_update_pause_button(self, event):
#         self.pause_button.SetLabel("Resume" if self.paused else "Pause")

#     def on_clear_button(self, event):
#         self.clear_requested = True

#     def on_close_button(self, event):
#         self.redraw_timer.Stop()
#         self.Destroy()

#     def on_redraw_timer(self, event):
#         """Ingest data from the pipe, page x-window when needed, and redraw."""
#         state = self.state

#         # external close
#         if state.close_graph.wait(0.001):
#             self.redraw_timer.Stop()
#             self.Destroy()
#             return

#         # drain pipe to latest values
#         while state.child_pipe.poll():
#             state.values = state.child_pipe.recv()

#         if self.paused:
#             # still allow the plot to redraw (e.g., window paging on resume)
#             self.draw_plot()
#             return

#         # handle clear request
#         if self.clear_requested:
#             self.clear_requested = False
#             self.t_buf.clear()
#             for y in self.y_bufs:
#                 y.clear()

#         # ingest one sample if available
#         vals = getattr(state, "values", None)
#         if vals is None:
#             return
#         # require at least one non-None series; keep alignment by appending None as NaN
#         now = time.time()
#         try:
#             cleaned = []
#             for v in vals:
#                 if v is None:
#                     cleaned.append(np.nan)
#                 elif isinstance(v, list):
#                     # mirror original behavior (error and close) but raise to keep logic tidy
#                     raise ValueError
#                 else:
#                     cleaned.append(float(v))
#             self._append_sample(now, cleaned)
#         except ValueError:
#             print(f"ERROR: Cannot plot array value. Use 'graph field[index]' instead.")
#             self.redraw_timer.Stop()
#             self.Destroy()
#             return

#         # page window if we've reached the right edge
#         self._maybe_advance_window(now)

#         # draw
#         # only draw when we have >= 2 points visible for each active series
#         tx, ysv = self._visible_slice()
#         if tx is None:
#             return
#         if any(y.size < 2 for y in ysv):
#             return
#         self.draw_plot()
