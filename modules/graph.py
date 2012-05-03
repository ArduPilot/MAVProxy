#!/usr/bin/env python
'''live graphing'''

import time, mavutil, re, pylab, matplotlib

mpstate = None

class graph_state(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.axes = []
        self.first_only = []
        self.fields = []
        self.field_types = []
        self.colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey' ]
        self.msg_types = set()
        self.fig = None
        self.num_points = 100

def name():
    '''return module name'''
    return "graph"

def description():
    '''return module description'''
    return "graph control"

def plotit(x, y, fields, colors=[]):
    '''plot a set of graphs using date for x axis'''

    if mpstate.graph_state.line is not None:
        for i in range(0, len(fields)):
            mpstate.graph_state.line[i].set_ydata(y[i])
        ax1 = mpstate.graph_state.fig.gca()
        ax1.relim()
        ax1.autoscale_view(True,True,True)
        pylab.draw()
        return

    pylab.ion()
    ax1 = mpstate.graph_state.fig.gca()
    ax2 = None

    mpstate.graph_state.line = []

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
        if mpstate.graph_state.axes[i] == 2:
            if ax2 == None:
                ax2 = ax1.twinx()
            ax = ax2
            label = fields[i]
            if label.endswith(":2"):
                label = label[:-2]
            ax2_labels.append(label)
        else:
            ax1_labels.append(fields[i])
            ax = ax1
        line, = ax.plot(x[i], y[i], color=color, label=fields[i],
                        linestyle='-', marker='None')
        mpstate.graph_state.line.append(line)
        pylab.draw()
        empty = False
    if ax1_labels != []:
        ax1.legend(ax1_labels,loc='best')
    if ax2_labels != []:
        ax2.legend(ax2_labels,loc='best')
    if empty:
        print("No data to graph")
        return
    pylab.draw()

def add_data(msg, vars):
    '''add some data'''
    mtype = msg.get_type()
    if mtype not in mpstate.graph_state.msg_types:
        return
    updated = False
    for i in range(0, len(mpstate.graph_state.fields)):
        if mtype not in mpstate.graph_state.field_types[i]:
            continue
        f = mpstate.graph_state.fields[i]
        if f.endswith(":2"):
            mpstate.graph_state.axes[i] = 2
            f = f[:-2]
        if f.endswith(":1"):
            mpstate.graph_state.first_only[i] = True
            f = f[:-2]
        v = mavutil.evaluate_expression(f, vars)
        if v is None:
            continue
        mpstate.graph_state.y[i].append(v)
        if len(mpstate.graph_state.y[i]) > mpstate.graph_state.num_points:
            mpstate.graph_state.y[i].pop(0)
        if len(mpstate.graph_state.y[i]) < mpstate.graph_state.num_points:
            mpstate.graph_state.y[i].extend([v]*(mpstate.graph_state.num_points-len(mpstate.graph_state.y[i])))
        updated = True
    if updated:
        plotit(mpstate.graph_state.x, mpstate.graph_state.y,
               mpstate.graph_state.fields[:], colors=mpstate.graph_state.colors)



def cmd_graph(args):
    '''graph command'''
    mpstate.graph_state.fields = args[:]
    mpstate.graph_state.x = []
    mpstate.graph_state.y = []
    mpstate.graph_state.axes = []
    mpstate.graph_state.first_only = []
    mpstate.graph_state.field_types = []
    mpstate.graph_state.line = None

    if mpstate.graph_state.fig is not None:
        pylab.close()
    mpstate.graph_state.fig = pylab.figure(num=1, figsize=(12,6))

    re_caps = re.compile('[A-Z_]+')
    for f in mpstate.graph_state.fields:
        caps = set(re.findall(re_caps, f))
        mpstate.graph_state.msg_types = mpstate.graph_state.msg_types.union(caps)
        mpstate.graph_state.field_types.append(caps)
        mpstate.graph_state.y.append([])
        mpstate.graph_state.axes.append(1)
        mpstate.graph_state.first_only.append(False)
        mpstate.graph_state.x.append(pylab.arange(0, mpstate.graph_state.num_points, 1))
    print("Adding graph: %s" % mpstate.graph_state.fields)

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.graph_state = graph_state()
    mpstate.command_map['graph'] = (cmd_graph, "live graphing")
    print("graph initialised")

def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    if mpstate.graph_state.fields:
        add_data(msg, mpstate.master().messages)
