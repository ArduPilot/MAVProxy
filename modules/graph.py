"""
MAVProxy realtime graphing module, partly based on the wx graphing
demo by Eli Bendersky (eliben@gmail.com)

  http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt

"""

import mavutil, re, os, sys

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import live_graph

class graph_state(object):
    def __init__(self):
        self.fields = []
        self.field_types = []
        self.msg_types = set()
        self.num_points = 100
        self.livegraph = None
        
def name():
    '''return module name'''
    return "graph"

def description():
    '''return module description'''
    return "graph control"

def cmd_graph(args):
    '''graph command'''
    state = mpstate.graph_state

    if state.livegraph is not None:
        # close any previous graph
        state.livegraph.close()
        state.livegraph = None
        
    state.fields = args[:]
    re_caps = re.compile('[A-Z_]+')
    for f in state.fields:
        caps = set(re.findall(re_caps, f))
        state.msg_types = state.msg_types.union(caps)
        state.field_types.append(caps)
    if not state.fields:
        return
    print("Adding graph: %s" % state.fields)

    state.values = [None]*len(state.fields)
    state.livegraph = live_graph.LiveGraph(state.fields,
                                           title='MAVProxy: graph')


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
    if state.livegraph is not None:
        state.livegraph.close()
        state.livegraph = None
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    state = mpstate.graph_state
    if not state.fields:
        return
    mtype = msg.get_type()
    if mtype not in state.msg_types:
        return
    for i in range(len(state.fields)):
        if mtype not in state.field_types[i]:
            continue
        f = state.fields[i]
        state.values[i] = mavutil.evaluate_expression(f, mpstate.master().messages)
    if state.livegraph is not None:
        state.livegraph.queue.put(state.values)
