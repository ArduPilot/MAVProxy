#!/usr/bin/env python3

'''
 core library for graphing in mavexplorer
'''

import ast
import sys, struct, time, os, datetime, platform
import math, re
import matplotlib
if platform.system() == "Windows":
    # wxAgg doesn't properly show the graph values in the lower right on Windows
    matplotlib.use('TkAgg')
elif platform.system() != "Darwin" and os.getenv("MPLBACKEND") is None:
    # on MacOS we can't set WxAgg here as it conflicts with the MacOS version
    matplotlib.use('WXAgg')
from math import *
from pymavlink.mavextra import *
import matplotlib.pyplot as plt
import matplotlib.ticker
from pymavlink import mavutil
import threading
import numpy as np

MAVGRAPH_DEBUG = 'MAVGRAPH_DEBUG' in os.environ

colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey', 'yellow', 'brown', 'darkcyan',
           'cornflowerblue', 'darkmagenta', 'deeppink', 'darkred']

flightmode_colours = [
    (1.0,   0,   0),
    (  0, 1.0,   0),
    (  0,   0, 1.0),

    (  0, 1.0, 1.0),
    (1.0,   0, 1.0),
    (1.0, 1.0,   0),

    (1.0, 0.5,   0),
    (1.0,   0, 0.5),
    (0.5, 1.0,   0),
    (  0, 1.0, 0.5),
    (0.5,   0, 1.0),
    (  0, 0.5, 1.0),
    (1.0, 0.5, 0.5),
    (0.5, 1.0, 0.5),
    (0.5, 0.5, 1.0)
]

edge_colour = (0.1, 0.1, 0.1)

graph_num = 1

tday_base = None
tday_basetime = None

def timestamp_to_days(timestamp, timeshift=0):
    '''convert log timestamp to days, public so that mavflightview can access'''
    global tday_base, tday_basetime
    if tday_base is None:
        try:
            tday_base = matplotlib.dates.date2num(datetime.datetime.fromtimestamp(timestamp+timeshift))
            tday_basetime = timestamp
        except ValueError:
            # this can happen if the log is corrupt
            # ValueError: year is out of range
            return 0
    sec_to_days = 1.0 / (60*60*24)
    return tday_base + (timestamp - tday_basetime) * sec_to_days

class MilliFormatter(matplotlib.dates.AutoDateFormatter):
    '''tick formatter that shows millisecond resolution'''
    def __init__(self, locator):
        super().__init__(locator)
        # don't show day until much wider range
        self.scaled[1.0/(24*60)] = self.scaled[1.0/(24*60*60)]

    def __call__(self, x, pos=0):
        """Return the label for time x at position pos."""
        v = super().__call__(x,pos=pos)
        if v.endswith("000"):
            return v[:-3]
        return v

class MavGraph(object):
    def __init__(self, flightmode_colourmap=None):
        self.lowest_x = None
        self.highest_x = None
        self.mav_list = []
        self.fields = []
        self.condition = None
        self.xaxis = None
        self.marker = None
        self.linestyle = None
        self.show_flightmode = True
        self.legend = 'upper left'
        self.legend2 = 'upper right'
        self.legend_flightmode = 'lower left'
        self.timeshift = 0
        self.labels = None
        self.multi = False
        self.modes_plotted = {}
        self.flightmode_colour_index = 0
        if flightmode_colourmap:
            self.flightmode_colourmap = flightmode_colourmap
        else:
            self.flightmode_colourmap = {}
        self.flightmode_list = None
        self.ax1 = None
        self.ax2 = None
        self.ax_by_num = {}
        self.locator = None
        global graph_num
        self.graph_num = graph_num
        self.start_time = None
        graph_num += 1
        self.draw_events = 0
        self.closing = False
        self.xlim_pipe = None
        self.xlim = None
        self.tday_base = None
        self.tday_basetime = None
        self.title = None
        self.grid = False
        self.xlim_t = None
        if sys.version_info[0] >= 3:
            self.text_types = frozenset([str,])
        else:
            self.text_types = frozenset([unicode, str])
        self.max_message_rate = 0
        self.axis_mode = 'auto'

    def set_axis_mode(self, mode):
        '''set y-axis layout mode: 'auto' (dual for 1-2 axes, multi for 3+),
        'dual' (legacy 2-axis) or 'multi' (all extra axes stacked on the left
        with vertical labels)'''
        if mode not in ('auto', 'dual', 'multi'):
            raise ValueError("axis_mode must be 'auto', 'dual' or 'multi'")
        self.axis_mode = mode

    def set_max_message_rate(self, rate_hz):
        '''set maximum rate we will graph any message'''
        self.max_message_rate = rate_hz
        self.last_message_t = {}

    def add_field(self, field):
        '''add another field to plot'''
        self.fields.append(field)

    def add_mav(self, mav):
        '''add another data source to plot'''
        self.mav_list.append(mav)

    def set_condition(self, condition):
        '''set graph condition'''
        self.condition = condition

    def set_xaxis(self, xaxis):
        '''set graph xaxis'''
        self.xaxis = xaxis

    def set_title(self, title):
        '''set graph title'''
        self.title = title

    def set_grid(self, enable):
        '''enable grid'''
        self.grid = enable
        
    def set_marker(self, marker):
        '''set graph marker'''
        self.marker = marker

    def set_timeshift(self, timeshift):
        '''set graph timeshift'''
        self.timeshift = timeshift

    def set_legend2(self, legend2):
        '''set graph legend2'''
        self.legend2 = legend2

    def set_legend(self, legend):
        '''set graph legend'''
        self.legend = legend

    def set_show_flightmode(self, value):
        '''set to true if flightmodes are to be shown'''
        self.show_flightmode = value

    def set_linestyle(self, linestyle):
        '''set graph linestyle'''
        self.linestyle = linestyle

    def set_multi(self, multi):
        '''set multiple graph option'''
        self.multi = multi

    def _set_multicolor_ylabel(self, ax_n, labels, lines):
        '''attach a y-axis label whose comma-separated parts are each coloured
        to match the line they refer to'''
        from matplotlib.offsetbox import (AnchoredOffsetbox, TextArea, VPacker)
        children = []
        for i, (lbl, line) in enumerate(zip(labels, lines)):
            if i > 0:
                children.append(TextArea(", ", textprops=dict(
                    color='black', rotation=90, ha='center', va='bottom')))
            children.append(TextArea(lbl, textprops=dict(
                color=line.get_color(), rotation=90, ha='center', va='bottom')))
        # VPacker stacks first child on top; reverse so visual order matches
        # the default y-label direction (text reads bottom-to-top)
        vp = VPacker(children=list(reversed(children)),
                     pad=0, sep=2, align='center')
        spine_pos = ax_n.spines['left'].get_position()
        if isinstance(spine_pos, tuple) and spine_pos[0] == 'axes':
            x_axes = spine_pos[1] - 0.04
        else:
            x_axes = -0.04
        box = AnchoredOffsetbox(loc='center right', child=vp, frameon=False,
                                bbox_to_anchor=(x_axes, 0.5),
                                bbox_transform=ax_n.transAxes, borderpad=0)
        ax_n.add_artist(box)
        ax_n.set_ylabel('')

    def make_format(self, current_axis):
        '''build a format_coord that reports y for every active axis'''
        def format_coord(x, y):
            # x, y are data coords on current_axis; project to every other axis
            display_coord = current_axis.transData.transform((x, y))
            if self.xaxis:
                parts = ['x=%.3f' % x]
            else:
                parts = ['x=%s' % self.formatter(x)]
            for n in sorted(self.ax_by_num.keys()):
                ax = self.ax_by_num[n]
                if ax is current_axis:
                    yv = y
                else:
                    yv = ax.transData.inverted().transform(display_coord)[1]
                parts.append('ax%d=%.3f' % (n, yv))
            return ' '.join(parts)
        return format_coord

    def next_flightmode_colour(self):
        '''allocate a colour to be used for a flight mode'''
        if self.flightmode_colour_index > len(flightmode_colours):
            print("Out of colours; reusing")
            self.flightmode_colour_index = 0
        ret = flightmode_colours[self.flightmode_colour_index]
        self.flightmode_colour_index += 1
        return ret

    def flightmode_colour(self, flightmode):
        '''return colour to be used for rendering a flight mode background'''
        if flightmode not in self.flightmode_colourmap:
            self.flightmode_colourmap[flightmode] = self.next_flightmode_colour()
        return self.flightmode_colourmap[flightmode]

    def xlim_changed(self, axsubplot):
        '''called when x limits are changed'''
        xrange = axsubplot.get_xbound()
        xlim = axsubplot.get_xlim()
        if self.draw_events == 0:
            # ignore limit change before first draw event
            return
        if self.xlim_pipe is not None and axsubplot == self.ax1 and xlim != self.xlim:
            self.xlim = xlim
            #print('send', self.graph_num, xlim)
            self.xlim_pipe[1].send(xlim)

    def draw_event(self, evt):
        '''called on draw events'''
        self.draw_events += 1

    def close_event(self, evt):
        '''called on close events'''
        self.closing = True
        
    def rescale_yaxis(self, axis):
        '''rescale Y axes to fit'''
        ylim = [None,None]

        for line in axis.lines:
            xdata = line.get_xdata()
            xlim = axis.get_xlim()
            xidx1 = np.argmax(xdata > xlim[0])
            xidx2 = np.argmax(xdata > xlim[1])
            if xidx2 == 0:
                xidx2 = len(xdata)-1
            ydata = line.get_ydata()[xidx1:xidx2]
            min_v = np.amin(ydata)
            max_v = np.amax(ydata)
            if ylim[0] is None or min_v < ylim[0]:
                ylim[0] = min_v
            if ylim[1] is None or max_v > ylim[1]:
                ylim[1] = max_v
        rng = ylim[1] - ylim[0]
        pad = 0.05 * rng
        if pad == 0:
            pad = 0.001 * ylim[0]
        axis.set_ylim((ylim[0]-pad,ylim[1]+pad))

    def button_click(self, event):
        '''handle button clicks'''
        if getattr(event, 'dblclick', False) and event.button==1:
            for ax in self.ax_by_num.values():
                self.rescale_yaxis(ax)

    def plotit(self, x, y, fields, colors=[], title=None, interactive=True):
        '''plot a set of graphs using date for x axis'''
        if interactive:
            plt.ion()
        self.fig = plt.figure(num=1, figsize=(12,6))
        self.ax1 = self.fig.gca()
        self.ax_by_num = {1: self.ax1}
        self.ax2 = None
        # resolve 'auto' to 'dual' (1-2 axes) or 'multi' (3+ axes) up front
        if self.axis_mode == 'auto':
            max_axis = max(self.axes) if self.axes else 1
            axis_mode = 'multi' if max_axis >= 3 else 'dual'
        else:
            axis_mode = self.axis_mode
        for i in range(0, len(fields)):
            if len(x[i]) == 0: continue
            if self.lowest_x is None or x[i][0] < self.lowest_x:
                self.lowest_x = x[i][0]
            if self.highest_x is None or x[i][-1] > self.highest_x:
                self.highest_x = x[i][-1]
        if self.highest_x is None or self.lowest_x is None:
            return
        self.locator = matplotlib.dates.AutoDateLocator()
        self.ax1.xaxis.set_major_locator(self.locator)
        self.formatter = MilliFormatter(self.locator)
        if not self.xaxis:
            self.ax1.xaxis.set_major_formatter(self.formatter)
            self.ax1.callbacks.connect('xlim_changed', self.xlim_changed)
            self.fig.canvas.mpl_connect('draw_event', self.draw_event)
            self.fig.canvas.mpl_connect('close_event', self.close_event)
        self.fig.canvas.mpl_connect('button_press_event', self.button_click)
        self.fig.canvas.get_default_filename = lambda: ''.join("graph" if self.title is None else
                                                               (x if x.isalnum() else '_' for x in self.title)) + '.png'
        empty = True
        labels_by_axis = {1: []}
        lines_by_axis = {1: []}
        label_strip_re = re.compile(r'^(.*):[2-9]$')

        for i in range(len(fields)):
            if len(x[i]) == 0:
                #print("Failed to find any values for field %s" % fields[i])
                continue
            if i < len(colors):
                color = colors[i]
            else:
                color = 'red'
                (tz, tzdst) = time.tzname

            axis_num = self.axes[i]
            if axis_mode == 'dual' and axis_num > 2:
                # legacy 2-axis layout: anything above axis 1 goes on the right
                axis_num = 2
            if axis_num not in self.ax_by_num:
                new_ax = self.ax1.twinx()
                self.ax_by_num[axis_num] = new_ax
                labels_by_axis[axis_num] = []
                lines_by_axis[axis_num] = []
                if self.grid:
                    new_ax.grid(None)
                    self.ax1.grid(True)
                if not self.xaxis:
                    new_ax.xaxis.set_major_locator(self.locator)
                    new_ax.xaxis.set_major_formatter(self.formatter)
                if axis_mode == 'multi' and axis_num >= 2:
                    # stack additional axes on the LEFT instead of the right
                    new_ax.yaxis.set_ticks_position('left')
                    new_ax.yaxis.set_label_position('left')
                    new_ax.spines['right'].set_visible(False)
                    new_ax.spines['left'].set_visible(True)
            ax = self.ax_by_num[axis_num]

            label = fields[i]
            m = label_strip_re.match(label)
            if m is not None:
                label = m.group(1)
            labels_by_axis[axis_num].append(label)
            if self.custom_labels[i] is not None:
                labels_by_axis[axis_num][-1] = self.custom_labels[i]
            if axis_num == 2 and self.ax2 is None:
                # back-compat alias
                self.ax2 = ax

            new_lines = []
            if self.xaxis:
                if self.marker is not None:
                    marker = self.marker
                else:
                    marker = '+'
                if self.linestyle is not None:
                    linestyle = self.linestyle
                else:
                    linestyle = 'None'
                new_lines = ax.plot(x[i], y[i], color=color, label=fields[i],
                                    linestyle=linestyle, marker=marker)
            else:
                if self.marker is not None:
                    marker = self.marker
                else:
                    marker = 'None'
                if self.linestyle is not None:
                    linestyle = self.linestyle
                else:
                    linestyle = '-'
                if len(y[i]) > 0 and type(y[i][0]) in self.text_types:
                    # assume this is a piece of text to be rendered at a point in time
                    last_text_time = -1
                    last_text = None
                    for n in range(0, len(x[i])):
                        this_text_time = round(x[i][n], 6)
                        this_text = y[i][n]
                        if last_text is None:
                            last_text = "[" + this_text + "]"
                            last_text_time = this_text_time
                        elif this_text_time == last_text_time:
                            last_text += ("[" + this_text + "]")
                        else:
                            ax.text(last_text_time,
                                    10,
                                    last_text,
                                    rotation=90,
                                    alpha=0.6,
                                    verticalalignment='center')
                            last_text = this_text
                            last_text_time = this_text_time
                    if last_text is not None:
                        ax.text(last_text_time,
                                10,
                                last_text,
                                rotation=90,
                                alpha=0.6,
                                verticalalignment='center')
                else:
                    new_lines = ax.plot_date(x[i], y[i], fmt=color, label=fields[i],
                                             linestyle=linestyle, marker=marker, tz=None)
            if new_lines:
                lines_by_axis[axis_num].extend(new_lines)

            empty = False

        # in 'multi' mode stack each extra axis further LEFT and reserve room
        if axis_mode == 'multi':
            extra_nums = sorted(n for n in self.ax_by_num.keys() if n >= 2)
            offset_step = 0.10
            for idx, n in enumerate(extra_nums):
                self.ax_by_num[n].spines['left'].set_position(
                    ("axes", -(idx + 1) * offset_step))
            if extra_nums:
                self.fig.subplots_adjust(
                    left=min(0.5, 0.08 + len(extra_nums) * offset_step))

        # install hover readout on every twinx axis when there are multiple axes
        if len(self.ax_by_num) > 1:
            for n, ax_n in self.ax_by_num.items():
                if n >= 2:
                    ax_n.format_coord = self.make_format(ax_n)

        if self.grid:
            plt.grid()

        if self.show_flightmode != 0:
            alpha = 0.3
            xlim = self.ax1.get_xlim()
            for i in range(len(self.flightmode_list)):
                (mode_name,t0,t1) = self.flightmode_list[i]
                c = self.flightmode_colour(mode_name)
                tday0 = timestamp_to_days(t0, self.timeshift)
                tday1 = timestamp_to_days(t1, self.timeshift)
                if tday0 > xlim[1] or tday1 < xlim[0]:
                    continue
                tday0 = max(tday0, xlim[0])
                tday1 = min(tday1, xlim[1])
                self.ax1.axvspan(tday0, tday1, fc=c, ec=edge_colour, alpha=alpha)
                self.modes_plotted[mode_name] = (c, alpha)

        if empty:
            print("No data to graph")
            return

        if title is not None:
            plt.title(title)
        else:
            title = fields[0]
        if self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title(title)
        else:
            self.fig.canvas.set_window_title(title)

        # in 'multi' mode label each axis vertically; when several fields share
        # an axis use a per-piece coloured composite label so each name appears
        # in its line's colour. Single-field axes additionally get the spine
        # and ticks coloured to match.
        if axis_mode == 'multi':
            for n, ax_n in self.ax_by_num.items():
                labels_n = labels_by_axis.get(n, [])
                lines_n = lines_by_axis.get(n, [])
                if not labels_n:
                    continue
                if len(lines_n) >= 2 and len(lines_n) == len(labels_n):
                    self._set_multicolor_ylabel(ax_n, labels_n, lines_n)
                else:
                    ax_n.set_ylabel(", ".join(labels_n))
                if len(lines_n) == 1:
                    line_color = lines_n[0].get_color()
                    ax_n.tick_params(axis='y', colors=line_color)
                    ax_n.spines['left'].set_color(line_color)
                    ax_n.yaxis.label.set_color(line_color)

        any_data_labels = any(labels_by_axis.get(n) for n in self.ax_by_num)

        if self.show_flightmode != 0:
            mode_patches = []
            for mode in self.modes_plotted.keys():
                (color, alpha) = self.modes_plotted[mode]
                mode_patches.append(matplotlib.patches.Patch(color=color,
                                                             label=mode, alpha=alpha*1.5))
            labels = [patch.get_label() for patch in mode_patches]
            if any_data_labels and self.show_flightmode != 2:
                patches_legend = plt.legend(mode_patches, labels, loc=self.legend_flightmode)
                self.fig.gca().add_artist(patches_legend)
            else:
                plt.legend(mode_patches, labels)

        # in 'dual' mode keep the legacy per-axis legend; in 'multi' mode the
        # vertical y-labels (per-piece coloured when shared) do the job
        if axis_mode == 'dual':
            legend_positions = {1: self.legend, 2: self.legend2}
            for n in sorted(self.ax_by_num.keys()):
                labels_n = labels_by_axis.get(n, [])
                if not labels_n:
                    continue
                self.ax_by_num[n].legend(
                    labels_n, loc=legend_positions.get(n, self.legend2))

    def add_data(self, t, msg, vars):
        '''add some data'''
        mtype = msg.get_type()
        for i in range(0, len(self.fields)):
            if mtype not in self.field_types[i]:
                continue
            f = self.fields[i]
            has_instance = False
            ins_value = None
            if mtype in self.instance_types[i]:
                instance_field = getattr(msg,'instance_field',None)
                if instance_field is None and hasattr(msg,'fmt'):
                    instance_field = getattr(msg.fmt,'instance_field')
                if instance_field is not None:
                    ins_value = getattr(msg,instance_field,None)
                    if ins_value is None or not str(ins_value) in self.instance_types[i][mtype]:
                        continue
                    if not mtype in vars or not isinstance(vars[mtype], dict):
                        vars[mtype] = dict()

                    vars[mtype][ins_value] = msg
                    if isinstance(ins_value, str):
                        mtype_instance = '%s[%s]' % (mtype, ins_value)
                        mtype_instance_str = '%s["%s"]' % (mtype, getattr(msg, instance_field))
                        f = f.replace(mtype_instance, mtype_instance_str)
                    has_instance = True

            # allow for capping the displayed message rate
            if self.max_message_rate > 0:
                mtype_ins = (mtype,ins_value,f)
                mt = msg._timestamp
                if mtype_ins in self.last_message_t:
                    dt = mt - self.last_message_t[mtype_ins]
                    if dt < 1.0 / self.max_message_rate:
                        continue
                self.last_message_t[mtype_ins] = mt

            simple = self.simple_field[i]
            v = None
            if simple is not None and not has_instance:
                try:
                    v = getattr(vars[simple[0]], simple[1])
                except Exception as ex:
                    if MAVGRAPH_DEBUG:
                        print(ex)
            if v is None:
                try:
                    v = mavutil.evaluate_expression(f, vars)
                except Exception as ex:
                    if MAVGRAPH_DEBUG:
                        print(ex)
            if v is None:
                continue
            if self.xaxis is None:
                xv = t
            else:
                xv = mavutil.evaluate_expression(self.xaxis, vars)
                if xv is None:
                    continue
            self.y[i].append(v)
            self.x[i].append(xv)

    def process_mav(self, mlog, flightmode_selections):
        '''process one file'''
        self.vars = {}
        idx = 0
        all_false = True
        for s in flightmode_selections:
            if s:
                all_false = False

        self.num_fields = len(self.fields)

        self.custom_labels = [None] * self.num_fields
        for i in range(self.num_fields):
            if self.fields[i].endswith(">"):
                a2 = self.fields[i].rfind("<")
                if a2 != -1:
                    self.custom_labels[i] = self.fields[i][a2+1:-1]
                    self.fields[i] = self.fields[i][:a2]

        # pre-calc which y-axis each field uses (digit suffix :2..:9, with
        # axes >= 3 rendered as additional offset right-hand spines)
        axis_re = re.compile(r'^(.*):([2-9])$')
        for i in range(self.num_fields):
            f = self.fields[i]
            m = axis_re.match(f)
            if m is not None:
                self.axes[i] = int(m.group(2))
                f = m.group(1)
            if f.endswith(":1"):
                self.first_only[i] = True
                f = f[:-2]
            self.fields[i] = f

        # see which fields are simple
        self.simple_field = []
        for i in range(0, self.num_fields):
            f = self.fields[i]
            m = re.match('^([A-Z][A-Z0-9_]*)[.]([A-Za-z_][A-Za-z0-9_]*)$', f)
            if m is None:
                self.simple_field.append(None)
            else:
                self.simple_field.append((m.group(1),m.group(2)))

        if len(self.flightmode_list) > 0:
            # prime the timestamp conversion
            timestamp_to_days(self.flightmode_list[0][1], self.timeshift)

        try:
            reset_state_data()
        except Exception:
            pass

        all_messages = {}

        while True:
            msg = mlog.recv_match(type=self.msg_types)
            if msg is None:
                break
            mtype = msg.get_type()
            if not mtype in all_messages or not isinstance(all_messages[mtype],dict):
                all_messages[mtype] = msg
            if mtype not in self.msg_types:
                continue
            if self.condition:
                if not mavutil.evaluate_condition(self.condition, all_messages):
                    continue
            tdays = timestamp_to_days(msg._timestamp, self.timeshift)

            if all_false or len(flightmode_selections) == 0:
                self.add_data(tdays, msg, all_messages)
            else:
                if idx < len(self.flightmode_list) and msg._timestamp >= self.flightmode_list[idx][2]:
                    idx += 1
                elif (idx < len(flightmode_selections) and flightmode_selections[idx]):
                    self.add_data(tdays, msg, all_messages)

    def xlim_change_check(self, idx):
        '''handle xlim change requests from queue'''
        try:
            if not self.xlim_pipe[1].poll():
                return
            xlim = self.xlim_pipe[1].recv()
            if xlim is None:
                return
        except Exception:
            return
        #print("recv: ", self.graph_num, xlim)
        if self.ax1 is not None and xlim != self.xlim:
            self.xlim = xlim
            self.fig.canvas.toolbar.push_current()
            #print("setting: ", self.graph_num, xlim)
            self.ax1.set_xlim(xlim)

    def xlim_timer(self):
        '''called every 0.1s to check for xlim change'''
        if self.closing:
            return
        self.xlim_change_check(0)

    def process(self, flightmode_selections, _flightmodes, block=True):
        '''process and display graph'''
        self.msg_types = set()
        self.multiplier = []
        self.field_types = []
        self.instance_types = []
        self.xlim = None
        self.flightmode_list = _flightmodes

        # work out msg types we are interested in
        self.x = []
        self.y = []
        self.modes = []
        self.axes = []
        self.first_only = []
        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        re_instance = re.compile(r'([A-Z_][A-Z0-9_]+)\[([0-9A-Z_]+)\]')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
            instances = set(re.findall(re_instance, f))
            itypes = dict()
            for (itype,ivalue) in instances:
                if not itype in itypes:
                    itypes[itype] = set()
                itypes[itype].add(ivalue)
            self.instance_types.append(itypes)
            self.y.append([])
            self.x.append([])
            self.axes.append(1)
            self.first_only.append(False)

        timeshift = self.timeshift

        for fi in range(0, len(self.mav_list)):
            mlog = self.mav_list[fi]
            self.process_mav(mlog, flightmode_selections)


    def show(self, lenmavlist, block=True, xlim_pipe=None, output=None):
        '''show graph'''
        if self.closing:
            return
        if xlim_pipe is not None:
            xlim_pipe[0].close()
        self.xlim_pipe = xlim_pipe
        if self.labels is not None:
            labels = self.labels.split(',')
            if len(labels) != len(fields)*lenmavlist:
                print("Number of labels (%u) must match number of fields (%u)" % (
                    len(labels), len(fields)*lenmavlist))
                return
        else:
            labels = None

        for fi in range(0, lenmavlist):
            timeshift = 0
            for i in range(0, len(self.x)):
                if self.first_only[i] and fi != 0:
                    self.x[i] = []
                    self.y[i] = []
            if labels:
                lab = labels[fi*len(self.fields):(fi+1)*len(self.fields)]
            else:
                lab = self.fields[:]
            if self.multi:
                col = colors[:]
            else:
                col = colors[fi*len(self.fields):]
            interactive = True
            if output is not None:
                interactive = False
            self.plotit(self.x, self.y, lab, colors=col, title=self.title, interactive=interactive)
            for i in range(0, len(self.x)):
                self.x[i] = []
                self.y[i] = []

        if self.xlim_pipe is not None and output is None:
            import matplotlib.animation
            self.ani = matplotlib.animation.FuncAnimation(self.fig, self.xlim_change_check,
                                                          frames=10, interval=20000,
                                                          repeat=True, blit=False)
            if self.xlim_t is None:
                self.xlim_t = self.fig.canvas.new_timer(interval=100)
                self.xlim_t.add_callback(self.xlim_timer)
                self.xlim_t.start()

        if output is None:
            plt.draw()
            plt.show(block=block)
        elif output.endswith(".html"):
            import mpld3
            html = mpld3.fig_to_html(self.fig)
            f_out = open(output, 'w')
            f_out.write(html)
            f_out.close()
        else:
            plt.savefig(output, bbox_inches='tight', dpi=200)


class MavHistogram(MavGraph):
    '''Histogram of a single log data field with live time-range filtering.'''

    def __init__(self, flightmode_colourmap=None):
        super().__init__(flightmode_colourmap)
        self.bins = 50
        self.show_stats = True
        self._all_x = None
        self._all_y = None

    def set_bins(self, bins):
        '''set number of histogram bins'''
        self.bins = bins

    def set_show_stats(self, show_stats):
        '''set whether to overlay median and std dev lines'''
        self.show_stats = show_stats

    def _draw_histogram(self, y_data):
        '''redraw histogram axes with the supplied data'''
        self.ax1.cla()
        label = self.fields[0] if self.fields else ''
        title = self.title if self.title else 'Histogram: ' + label
        self.ax1.set_title(title)
        self.ax1.hist(y_data, bins=self.bins, color=colors[0], alpha=0.7, label=label,
                      weights=np.ones(len(y_data)) / len(y_data) * 100.0)
        self.ax1.set_xlabel(label)
        self.ax1.set_ylabel('Frequency (%)')
        self.ax1.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter('%.1f%%'))
        self.ax1.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(nbins=15))
        self.ax1.xaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())
        self.ax1.tick_params(axis='x', which='major', labelrotation=45)

        if self.show_stats and len(y_data) > 0:
            median = np.median(y_data)
            mean   = np.mean(y_data)
            std    = np.std(y_data)

            stat_styles = [
                (median, 'black',  '-',  2.0, 'Median %.3g' % median),
                (mean - std,   'orange', '--', 1.5, u'\u00b11\u03c3 (%.3g)' % std),
                (mean + std,   'orange', '--', 1.5, None),
                (mean - 2*std, 'red',    ':',  1.5, u'\u00b12\u03c3 (%.3g)' % (2*std)),
                (mean + 2*std, 'red',    ':',  1.5, None),
                (mean - 3*std, 'purple', '-.',  1.2, u'\u00b13\u03c3 (%.3g)' % (3*std)),
                (mean + 3*std, 'purple', '-.',  1.2, None),
            ]
            for (xval, col, ls, lw, lbl) in stat_styles:
                self.ax1.axvline(x=xval, color=col, linestyle=ls,
                                 linewidth=lw, label=lbl)

        self.ax1.legend(loc=self.legend)
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

    def _hist_xlim_timer(self):
        '''called every 100 ms to check for time-range updates from other graphs'''
        if self.closing or self.xlim_pipe is None:
            return
        try:
            if not self.xlim_pipe[1].poll():
                return
            xlim = self.xlim_pipe[1].recv()
        except Exception:
            return
        if xlim == self.xlim or xlim is None:
            return
        self.xlim = xlim
        mask = (self._all_x >= xlim[0]) & (self._all_x <= xlim[1])
        y_filtered = self._all_y[mask]
        if len(y_filtered) == 0:
            return
        self._draw_histogram(y_filtered)

    def show(self, lenmavlist, block=True, xlim_pipe=None, output=None):
        '''show histogram plot in a new figure'''
        if xlim_pipe is not None:
            xlim_pipe[0].close()
        self.xlim_pipe = xlim_pipe

        if not self.x or len(self.x[0]) == 0:
            print("No data for histogram")
            return

        self._all_x = np.array(self.x[0])
        self._all_y = np.array(self.y[0])

        label = self.fields[0] if self.fields else ''
        title = self.title if self.title else 'Histogram: ' + label

        interactive = output is None
        if interactive:
            plt.ion()

        self.fig, ax = plt.subplots(figsize=(10, 6))
        self.ax1 = ax

        self._draw_histogram(self._all_y)

        self.fig.canvas.get_default_filename = lambda: ''.join(
            'histogram' if self.title is None else
            (c if c.isalnum() else '_' for c in title)) + '.png'
        if self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title(title)
        self.fig.canvas.mpl_connect('close_event', self.close_event)

        if output is None:
            if xlim_pipe is not None:
                self.xlim_t = self.fig.canvas.new_timer(interval=100)
                self.xlim_t.add_callback(self._hist_xlim_timer)
                self.xlim_t.start()
            plt.draw()
            plt.show(block=block)
        elif output.endswith('.html'):
            import mpld3
            html = mpld3.fig_to_html(self.fig)
            with open(output, 'w') as f_out:
                f_out.write(html)
        else:
            plt.savefig(output, bbox_inches='tight', dpi=200)


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
    parser.add_argument("--planner", action='store_true', help="use planner file format")
    parser.add_argument("--condition", default=None, help="select packets by a condition")
    parser.add_argument("--labels", default=None, help="comma separated field labels")
    parser.add_argument("--legend", default='upper left', help="default legend position")
    parser.add_argument("--legend2", default='upper right', help="default legend2 position")
    parser.add_argument("--marker", default=None, help="point marker")
    parser.add_argument("--linestyle", default=None, help="line style")
    parser.add_argument("--xaxis", default=None, help="X axis expression")
    parser.add_argument("--title", default=None, help="set title")
    parser.add_argument("--multi", action='store_true', help="multiple files with same colours")
    parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
    parser.add_argument("--show-flightmode", default=True,
                        help="Add background colour to plot corresponding to current flight mode.  Cannot be specified with --xaxis.")
    parser.add_argument("--dialect", default="all", help="MAVLink dialect")
    parser.add_argument("--output", default=None, help="provide an output format")
    parser.add_argument("--timeshift", type=float, default=0, help="shift time on first graph in seconds")
    parser.add_argument("--grid", action='store_true', help="show a grid")
    parser.add_argument("--axis-mode", default='auto',
                        choices=['auto', 'dual', 'multi'],
                        help="y-axis layout: 'auto' picks dual for 1-2 axes "
                             "and multi for 3+, 'dual' forces the legacy 2-axis "
                             "layout, 'multi' stacks each :N axis on the left "
                             "with vertical labels")
    parser.add_argument("logs_fields", metavar="<LOG or FIELD>", nargs="+")
    args = parser.parse_args()

    mg = MavGraph()

    filenames = []
    for f in args.logs_fields:
        if os.path.exists(f):
            mlog = mavutil.mavlink_connection(f, notimestamps=args.notimestamps,
                                              zero_time_base=args.zero_time_base,
                                              dialect=args.dialect)
            mg.add_mav(mlog)
        else:
            mg.add_field(f)
    mg.set_condition(args.condition)
    mg.set_xaxis(args.xaxis)
    mg.set_marker(args.marker)
    mg.set_legend(args.legend)
    mg.set_legend2(args.legend2)
    mg.set_multi(args.multi)
    mg.set_title(args.title)
    mg.set_grid(args.grid)
    mg.set_show_flightmode(args.show_flightmode)
    mg.set_axis_mode(args.axis_mode)
    mg.process([],[],0)
    mg.show(len(mg.mav_list), output=args.output)
