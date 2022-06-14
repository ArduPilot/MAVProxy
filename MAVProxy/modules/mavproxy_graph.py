"""
  MAVProxy realtime graphing module

  uses lib/live_graph.py for display
"""

from pymavlink import mavutil
import re, os, sys

from MAVProxy.modules.lib import live_graph

from MAVProxy.modules.lib import mp_module

class GraphModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GraphModule, self).__init__(mpstate, "graph", "graph control")
        self.timespan = 20
        self.tickresolution = 0.2
        self.graphs = []
        self.add_command('graph', self.cmd_graph, "[expression...] add a live graph",
                         ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)',
                          'legend',
                          'timespan',
                          'tickresolution'])
        self.legend = {
        }
        

    def cmd_graph(self, args):
        '''graph command'''
        if len(args) == 0:
            # list current graphs
            for i in range(len(self.graphs)):
                print("Graph %u: %s" % (i, self.graphs[i].fields))
            return

        elif args[0] == "help":
            print("graph <timespan|tickresolution|expression>")
        elif args[0] == "timespan":
            if len(args) == 1:
                print("timespan: %.1f" % self.timespan)
                return
            self.timespan = float(args[1])
        elif args[0] == "tickresolution":
            if len(args) == 1:
                print("tickresolution: %.1f" % self.tickresolution)
                return
            self.tickresolution = float(args[1])
        elif args[0] == "legend":
            self.cmd_legend(args[1:])
        else:
            # start a new graph
            self.graphs.append(Graph(self, args[:]))

    def cmd_legend(self, args):
        '''setup legend for graphs'''
        if len(args) == 0:
            for leg in self.legend.keys():
                print("%s -> %s" % (leg, self.legend[leg]))
        elif len(args) == 1:
            leg = args[0]
            if leg in self.legend:
                print("Removing legend %s" % leg)
                self.legend.pop(leg)
        elif len(args) >= 2:
            leg = args[0]
            leg2 = args[1]
            print("Adding legend %s -> %s" % (leg, leg2))
            self.legend[leg] = leg2

    def unload(self):
        '''unload module'''
        for g in self.graphs:
            g.close()
        self.graphs = []

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''

        # check for any closed graphs
        for i in range(len(self.graphs) - 1, -1, -1):
            if not self.graphs[i].is_alive():
                self.graphs[i].close()
                self.graphs.pop(i)

        # add data to the rest
        for g in self.graphs:
            g.add_mavlink_packet(msg)


def init(mpstate):
    '''initialise module'''
    return GraphModule(mpstate)

class Graph():
    '''a graph instance'''
    def __init__(self, state, fields):
        self.fields = fields[:]
        self.field_types = []
        self.msg_types = set()
        self.state = state

        for i in range(len(self.fields)):
            # special handling for NAMED_VALUE_FLOAT
            m = re.match("^NAMED_VALUE_FLOAT\[([A-Z0-9_]+)\]\.(.*)$", self.fields[i])
            if m:
                self.fields[i] = 'NAMED_VALUE_FLOAT["%s"].%s' % (m.group(1), m.group(2))

        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
        print("Adding graph: %s" % self.fields)

        fields = [ self.pretty_print_fieldname(x) for x in fields ]

        self.values = [None] * len(self.fields)
        self.livegraph = live_graph.LiveGraph(fields,
                                              timespan=state.timespan,
                                              tickresolution=state.tickresolution,
                                              title=self.fields[0])

    def pretty_print_fieldname(self, fieldname):
        if fieldname in self.state.legend:
            return self.state.legend[fieldname]
        return fieldname

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

    def add_mavlink_packet(self, msg):
        '''add data to the graph'''
        mtype = msg.get_type()
        if mtype not in self.msg_types:
            return
        have_value = False
        for i in range(len(self.fields)):
            if mtype not in self.field_types[i]:
                continue
            f = self.fields[i]
            self.values[i] = mavutil.evaluate_expression(f, self.state.master.messages)
            if self.values[i] is not None:
                have_value = True
        if have_value and self.livegraph is not None:
            self.livegraph.add_values(self.values)
