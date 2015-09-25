#!/usr/bin/env python

'''
 core library for graphing in mavexplorer
'''

import sys, struct, time, os, datetime
import math, re
import matplotlib
from math import *
from pymavlink.mavextra import *
import pylab
from pymavlink import mavutil

colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey', 'yellow', 'brown', 'darkcyan',
           'cornflowerblue', 'darkmagenta', 'deeppink', 'darkred']

colourmap = {
    'apm' : {
        'MANUAL'    : (1.0,   0,   0),
        'AUTO'      : (  0, 1.0,   0),
        'LOITER'    : (  0,   0, 1.0),
        'FBWA'      : (1.0, 0.5,   0),
        'RTL'       : (  1,   0, 0.5),
        'STABILIZE' : (0.5, 1.0,   0),
        'LAND'      : (  0, 1.0, 0.5),
        'STEERING'  : (0.5,   0, 1.0),
        'HOLD'      : (  0, 0.5, 1.0),
        'ALT_HOLD'  : (1.0, 0.5, 0.5),
        'CIRCLE'    : (0.5, 1.0, 0.5),
        'POSITION'  : (1.0, 0.0, 1.0),
        'GUIDED'    : (0.5, 0.5, 1.0),
        'ACRO'      : (1.0, 1.0,   0),
        'CRUISE'    : (  0, 1.0, 1.0)
        },
    'px4' : {
        'MANUAL'    : (1.0,   0,   0),
        'SEATBELT'  : (  0.5, 0.5,   0),
        'EASY'      : (  0, 1.0,   0),
        'AUTO'    : (  0,   0, 1.0),
        'UNKNOWN'    : (  1.0,   1.0, 1.0)
        }
    }

edge_colour = (0.1, 0.1, 0.1)

class MavGraph(object):
    def __init__(self):
        self.lowest_x = None
        self.highest_x = None
        self.mav_list = []
        self.fields = []
        self.condition = None
        self.xaxis = None
        self.marker = None
        self.linestyle = None
        self.flightmode = None
        self.legend = 'upper left'
        self.legend2 = 'upper right'
        self.timeshift = 0
        self.labels = None
        self.multi = False

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

    def set_flightmode(self, flightmode):
        '''set graph flightmode'''
        self.flightmode = flightmode

    def set_linestyle(self, linestyle):
        '''set graph linestyle'''
        self.linestyle = linestyle

    def set_multi(self, multi):
        '''set multiple graph option'''
        self.multi = multi

    def make_format(self, current, other):
        # current and other are axes
        def format_coord(x, y):
            # x, y are data coordinates
            # convert to display coords
            display_coord = current.transData.transform((x,y))
            inv = other.transData.inverted()
            # convert back to data coords with respect to ax
            ax_coord = inv.transform(display_coord)
            xstr = self.formatter(x)
            y2 = ax_coord[1]
            if self.xaxis:
                return ('x=%.3f Left=%.3f Right=%.3f' % (x, y2, y))
            else:
                return ('x=%s Left=%.3f Right=%.3f' % (xstr, y2, y))
        return format_coord

    def plotit(self, x, y, fields, colors=[]):
        '''plot a set of graphs using date for x axis'''
        pylab.ion()
        fig = pylab.figure(num=1, figsize=(12,6))
        ax1 = fig.gca()
        ax2 = None
        xrange = 0.0
        for i in range(0, len(fields)):
            if len(x[i]) == 0: continue
            if self.lowest_x is None or x[i][0] < self.lowest_x:
                self.lowest_x = x[i][0]
            if self.highest_x is None or x[i][-1] > self.highest_x:
                self.highest_x = x[i][-1]
        if self.highest_x is None or self.lowest_x is None:
            return
        xrange = self.highest_x - self.lowest_x
        xrange *= 24 * 60 * 60
        self.formatter = matplotlib.dates.DateFormatter('%H:%M:%S')
        interval = 1
        intervals = [ 1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600,
                      900, 1800, 3600, 7200, 5*3600, 10*3600, 24*3600 ]
        for interval in intervals:
            if xrange / interval < 15:
                break
        locator = matplotlib.dates.SecondLocator(interval=interval)
        if not self.xaxis:
            ax1.xaxis.set_major_locator(locator)
            ax1.xaxis.set_major_formatter(self.formatter)
        empty = True
        ax1_labels = []
        ax2_labels = []

        for i in range(0, len(fields)):
            if len(x[i]) == 0:
                print("Failed to find any values for field %s" % fields[i])
                continue
            if i < len(colors):
                color = colors[i]
            else:
                color = 'red'
                (tz, tzdst) = time.tzname

            if self.axes[i] == 2:
                if ax2 == None:
                    ax2 = ax1.twinx()
                    ax2.format_coord = self.make_format(ax2, ax1)
                ax = ax2
                if not self.xaxis:
                    ax2.xaxis.set_major_locator(locator)
                    ax2.xaxis.set_major_formatter(self.formatter)
                label = fields[i]
                if label.endswith(":2"):
                    label = label[:-2]
                ax2_labels.append(label)
            else:
                ax1_labels.append(fields[i])
                ax = ax1
            
            if self.xaxis:
                if self.marker is not None:
                    marker = self.marker
                else:
                    marker = '+'
                if self.linestyle is not None:
                    linestyle = self.linestyle
                else:
                    linestyle = 'None'
                ax.plot(x[i], y[i], color=color, label=fields[i],
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
                ax.plot_date(x[i], y[i], color=color, label=fields[i],
                             linestyle=linestyle, marker=marker, tz=None)

            empty = False
            if self.flightmode is not None:
                for i in range(len(self.modes)-1):
                    c = colourmap[self.flightmode].get(self.modes[i][1], edge_colour)
                    ax1.axvspan(self.modes[i][0], self.modes[i+1][0], fc=c, ec=edge_colour, alpha=0.1)
                c = colourmap[self.flightmode].get(self.modes[-1][1], edge_colour)
                ax1.axvspan(self.modes[-1][0], ax1.get_xlim()[1], fc=c, ec=edge_colour, alpha=0.1)

            if ax1_labels != []:
                ax1.legend(ax1_labels,loc=self.legend)
            if ax2_labels != []:
                ax2.legend(ax2_labels,loc=self.legend2)
            if empty:
                print("No data to graph")
                return


    def add_data(self, t, msg, vars, flightmode):
        '''add some data'''
        mtype = msg.get_type()
        if self.flightmode is not None and (len(self.modes) == 0 or self.modes[-1][1] != flightmode):
            self.modes.append((t, flightmode))
        for i in range(0, len(self.fields)):
            if mtype not in self.field_types[i]:
                continue
            f = self.fields[i]
            if f.endswith(":2"):
                self.axes[i] = 2
                f = f[:-2]
            if f.endswith(":1"):
                self.first_only[i] = True
                f = f[:-2]
            v = mavutil.evaluate_expression(f, vars)
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


    def process_mav(self, mlog, timeshift):
        '''process one file'''
        self.vars = {}
        while True:
            msg = mlog.recv_msg()
            if msg is None:
                break
            if msg.get_type() not in self.msg_types:
                continue
            if self.condition:
                if not mavutil.evaluate_condition(self.condition, mlog.messages):
                    continue
            tdays = matplotlib.dates.date2num(datetime.datetime.fromtimestamp(msg._timestamp+timeshift))
            self.add_data(tdays, msg, mlog.messages, mlog.flightmode)

    def process(self, block=True):
        '''process and display graph'''
        self.msg_types = set()
        self.multiplier = []
        self.field_types = []

        # work out msg types we are interested in
        self.x = []
        self.y = []
        self.modes = []
        self.axes = []
        self.first_only = []
        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
            self.y.append([])
            self.x.append([])
            self.axes.append(1)
            self.first_only.append(False)

        if self.labels is not None:
            labels = self.labels.split(',')
            if len(labels) != len(fields)*len(self.mav_list):
                print("Number of labels (%u) must match number of fields (%u)" % (
                    len(labels), len(fields)*len(self.mav_list)))
                return
        else:
            labels = None
            
        timeshift = self.timeshift

        for fi in range(0, len(self.mav_list)):
            mlog = self.mav_list[fi]
            self.process_mav(mlog, timeshift)
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
            self.plotit(self.x, self.y, lab, colors=col)
            for i in range(0, len(self.x)):
                self.x[i] = []
                self.y[i] = []
        pylab.draw()

    def show(self, block=True):
        '''show graph'''
        pylab.show(block=block)

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
    parser.add_argument("--multi", action='store_true', help="multiple files with same colours")
    parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
    parser.add_argument("--flightmode", default=None,
                        help="Choose the plot background according to the active flight mode of the specified type, e.g. --flightmode=apm for ArduPilot or --flightmode=px4 for PX4 stack logs.  Cannot be specified with --xaxis.")
    parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
    parser.add_argument("--output", default=None, help="provide an output format")
    parser.add_argument("--timeshift", type=float, default=0, help="shift time on first graph in seconds")
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
    mg.set_flightmode(args.flightmode)
    mg.process()
    mg.show()
