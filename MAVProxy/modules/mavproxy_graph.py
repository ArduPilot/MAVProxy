"""
  MAVProxy realtime graphing module

  uses lib/live_graph.py for display
"""

from pymavlink import mavutil
import re, os, sys

mpstate = None

from MAVProxy.modules.lib import live_graph

class graph_state(object):
    def __init__(self):
        self.timespan = 20
        self.tickresolution = 0.2
        self.graphs = []
        
def name():
    '''return module name'''
    return "graph"

def description():
    '''return module description'''
    return "graph control"

def cmd_graph(args):
    '''graph command'''
    state = mpstate.graph_state

    if len(args) == 0:
        # list current graphs
        for i in range(len(state.graphs)):
            print("Graph %u: %s" % (i, state.graphs[i].fields))
        return

    elif args[0] == "help":
        print("graph <timespan|tickresolution|expression>")
    elif args[0] == "timespan":
        if len(args) == 1:
            print("timespan: %.1f" % state.timespan)
            return
        state.timespan = float(args[1])
    elif args[0] == "tickresolution":
        if len(args) == 1:
            print("tickresolution: %.1f" % state.tickresolution)
            return
        state.tickresolution = float(args[1])
    else:
        # start a new graph
        state.graphs.append(Graph(args[:]))


def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.graph_state = graph_state()
    mpstate.command_map['graph'] = (cmd_graph, "[expression...] add a live graph")
    print("graph initialised")

def unload():
    '''unload module'''
    state = mpstate.graph_state
    for g in state.graphs:
        g.close()
    state.graphs = []
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    state = mpstate.graph_state

    # check for any closed graphs
    for i in range(len(state.graphs)-1, -1, -1):
        if not state.graphs[i].is_alive():
            state.graphs[i].close()
            state.graphs.pop(i)

    # add data to the rest
    for g in state.graphs:
        g.mavlink_packet(msg)


class Graph():
    '''a graph instance'''
    def __init__(self, fields):
        state = mpstate.graph_state
        self.fields = fields[:]
        self.field_types = []
        self.msg_types = set()

        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
        print("Adding graph: %s" % self.fields)

        self.values = [None]*len(self.fields)
        self.livegraph = live_graph.LiveGraph(self.fields,
                                              timespan=state.timespan,
                                              tickresolution=state.tickresolution,
                                              title=self.fields[0])

    def is_alive(self):
        '''check if this graph is still alive'''
        if self.livegraph:
            return self.livegraph.is_alive()
        return False        

    def close(self):
        '''close this graph'''
        if self.livegraph:
            self.livegraph.close()
        self.livegraph = None

    def mavlink_packet(self, msg):
        '''add data to the graph'''
        mtype = msg.get_type()
        if mtype not in self.msg_types:
            return
        for i in range(len(self.fields)):
            if mtype not in self.field_types[i]:
                continue
            f = self.fields[i]
            self.values[i] = mavutil.evaluate_expression(f, mpstate.master().messages)
        if self.livegraph is not None:
            self.livegraph.add_values(self.values)
