#!/usr/bin/env python
'''
log analysis program
Andrew Tridgell December 2014
'''

import sys, struct, time, os, datetime
import math, re
import Queue
import fnmatch
import threading, multiprocessing
from math import *
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import grapher
from MAVProxy.modules.lib import mavmemlog
from pymavlink.mavextra import *
from MAVProxy.modules.lib.mp_menu import *
import MAVProxy.modules.lib.mp_util as mp_util
from pymavlink import mavutil
from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.graphdefinition import GraphDefinition
from lxml import objectify
import pkg_resources

#Global var to hold the GUI menu element
TopMenu = None

class MEStatus(object):
    '''status object to conform with mavproxy structure for modules'''
    def __init__(self):
        self.msgs = {}

class MEState(object):
    '''holds state of MAVExplorer'''
    def __init__(self):
        self.input_queue = Queue.Queue()
        self.rl = None
        self.console = wxconsole.MessageConsole(title='MAVExplorer')
        self.exit = False
        self.status = MEStatus()
        self.settings = MPSettings(
            [ MPSetting('marker', str, '+', 'data marker', tab='Graph'),
              MPSetting('condition', str, None, 'condition'),
              MPSetting('xaxis', str, None, 'xaxis'),
              MPSetting('linestyle', str, None, 'linestyle'),
              MPSetting('show_flightmode', bool, True, 'show flightmode'),
              MPSetting('legend', str, 'upper left', 'legend position'),
              MPSetting('legend2', str, 'upper right', 'legend2 position')
              ]
            )

        self.mlog = None
        self.command_map = command_map
        self.completions = {
            "set"       : ["(SETTING)"],
            "condition" : ["(VARIABLE)"],
            "graph"     : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)'],
            "map"       : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)']
            }
        self.aliases = {}
        self.graphs = []
        self.flightmode_selections = []
        self.last_graph = GraphDefinition('Untitled', '', '', [], None)

def have_graph(name):
    '''return true if we have a graph of the given name'''
    for g in mestate.graphs:
        if g.name == name:
            return True
    return False

def menu_callback(m):
    '''called on menu selection'''
    if m.returnkey.startswith('# '):
        cmd = m.returnkey[2:]
        if m.handler is not None:
            if m.handler_result is None:
                return
            cmd += m.handler_result
        process_stdin(cmd)
    elif m.returnkey == 'menuSettings':
        wxsettings.WXSettings(mestate.settings)
    elif m.returnkey.startswith("mode-"):
        idx = int(m.returnkey[5:])
        mestate.flightmode_selections[idx] = m.IsChecked()
    elif m.returnkey.startswith("loadLog"):
        print "File: " + m.returnkey[8:]
    elif m.returnkey == 'quit':
        mestate.console.close()
        mestate.exit = True
        print "Exited. Press Enter to continue."
        sys.exit(0)
        
    else:
        print('Unknown menu selection: %s' % m.returnkey)


def flightmode_menu():
    '''construct flightmode menu'''
    modes = mestate.mlog.flightmode_list()
    ret = []
    idx = 0
    for (mode,t1,t2) in modes:
        modestr = "%s %us" % (mode, (t2-t1))
        ret.append(MPMenuCheckbox(modestr, modestr, 'mode-%u' % idx))
        idx += 1
        mestate.flightmode_selections.append(False)
    return ret


def graph_menus():
    '''return menu tree for graphs (recursive)'''
    ret = MPMenuSubMenu('Graphs', [])
    for i in range(len(mestate.graphs)):
        g = mestate.graphs[i]
        path = g.name.split('/')
        name = path[-1]
        path = path[:-1]
        ret.add_to_submenu(path, MPMenuItem(name, name, '# graph :%u' % i))
    return ret

def setup_file_menu():
    global TopMenu
    TopMenu = MPMenuTop([])
    TopMenu.add(MPMenuSubMenu('MAVExplorer',
                           items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                  MPMenuItem('&Open\tCtrl+O', 'Open Log', '# loadLog ',
                                            handler=MPMenuCallFileDialog(
                                                                        flags=('open',),
                                                                        title='Logfile Load',
                                                                        wildcard='*.tlog;*.log;*.BIN;*.bin')),
                                  MPMenuItem('&Quit\tCtrl+Q', 'Quit', 'quit')]))
    mestate.console.set_menu(TopMenu, menu_callback)
    
def setup_menus():
    '''setup console menus'''
    global TopMenu
    TopMenu.add(MPMenuSubMenu('Display',
                           items=[MPMenuItem('Map', 'Map', '# map'),
                                  MPMenuItem('Save Graph', 'Save', '# save'),
                                  MPMenuItem('Reload Graphs', 'Reload', '# reload')]))
    TopMenu.add(graph_menus())
    TopMenu.add(MPMenuSubMenu('FlightMode', items=flightmode_menu()))

    mestate.console.set_menu(TopMenu, menu_callback)

def expression_ok(expression):
    '''return True if an expression is OK with current messages'''
    expression_ok = True
    fields = expression.split()
    for f in fields:
        try:
            if f.endswith(':2'):
                f = f[:-2]
            if mavutil.evaluate_expression(f, mestate.status.msgs) is None:
                expression_ok = False                        
        except Exception:
            expression_ok = False
            break
    return expression_ok

def load_graph_xml(xml, filename, load_all=False):
    '''load a graph from one xml string'''
    ret = []
    try:
        root = objectify.fromstring(xml)
    except Exception:
        return []
    if root.tag != 'graphs':
        return []
    if not hasattr(root, 'graph'):
        return []
    for g in root.graph:
        name = g.attrib['name']
        expressions = [e.text for e in g.expression]
        if load_all:
            ret.append(GraphDefinition(name, e, g.description.text, expressions, filename))
            continue
        if have_graph(name):
            continue
        for e in expressions:
            if expression_ok(e):
                ret.append(GraphDefinition(name, e, g.description.text, expressions, filename))
                break
    return ret

def load_graphs():
    '''load graphs from mavgraphs.xml'''
    mestate.graphs = []
    gfiles = ['mavgraphs.xml']
    if 'HOME' in os.environ:
        for dirname, dirnames, filenames in os.walk(os.path.join(os.environ['HOME'], ".mavproxy")):
            for filename in filenames:
                if filename.lower().endswith('.xml'):
                    gfiles.append(os.path.join(dirname, filename))
    elif 'LOCALAPPDATA' in os.environ:
        for dirname, dirnames, filenames in os.walk(os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy")):
            for filename in filenames:
                if filename.lower().endswith('.xml'):
                    gfiles.append(os.path.join(dirname, filename))
                    
    for file in gfiles:
        if not os.path.exists(file):
            continue
        graphs = load_graph_xml(open(file).read(), file)
        if graphs:
            mestate.graphs.extend(graphs)
            mestate.console.writeln("Loaded %s" % file)
    # also load the built in graphs
    dlist = pkg_resources.resource_listdir("MAVProxy", "tools/graphs")
    for f in dlist:
        raw = pkg_resources.resource_stream("MAVProxy", "tools/graphs/%s" % f).read()
        graphs = load_graph_xml(raw, None)
        if graphs:
            mestate.graphs.extend(graphs)
            mestate.console.writeln("Loaded %s" % f)
    mestate.graphs = sorted(mestate.graphs, key=lambda g: g.name)

def graph_process(fields, mavExpLog, mavExpFlightModeSel, mavExpSettings):
    '''process for a graph'''
    mavExpLog.reduce_by_flightmodes(mavExpFlightModeSel)
    
    mg = grapher.MavGraph()
    mg.set_marker(mavExpSettings.marker)
    mg.set_condition(mavExpSettings.condition)
    mg.set_xaxis(mavExpSettings.xaxis)
    mg.set_linestyle(mavExpSettings.linestyle)
    mg.set_show_flightmode(mavExpSettings.show_flightmode)
    mg.set_legend(mavExpSettings.legend)
    mg.add_mav(mavExpLog)
    for f in fields:
        mg.add_field(f)
    mg.process()
    mg.show()

def display_graph(graphdef):
    '''display a graph'''
    mestate.console.write("Expression: %s\n" % ' '.join(graphdef.expression.split()))
    child = multiprocessing.Process(target=graph_process, args=[graphdef.expression.split(), mestate.mlog, mestate.flightmode_selections, mestate.settings])
    child.start()

def cmd_graph(args):
    '''graph command'''
    usage = "usage: graph <FIELD...>"
    if len(args) < 1:
        print(usage)
        return
    if args[0][0] == ':':
        i = int(args[0][1:])
        g = mestate.graphs[i]
        expression = g.expression
        args = expression.split()
        mestate.console.write("Added graph: %s\n" % g.name)
        if g.description:
            mestate.console.write("%s\n" % g.description, fg='blue')
        mestate.rl.add_history("graph %s" % ' '.join(expression.split()))
        mestate.last_graph = g
    else:
        expression = ' '.join(args)
        mestate.last_graph = GraphDefinition('Untitled', expression, '', [expression], None)
    display_graph(mestate.last_graph)

def map_process(args, MAVExpLog, MAVExpFlightModes, MAVExpSettings):
    '''process for a graph'''
    from mavflightview import mavflightview_mav, mavflightview_options
    MAVExpLog.reduce_by_flightmodes(MAVExpFlightModes)
    
    options = mavflightview_options()
    options.condition = MAVExpSettings.condition
    if len(args) > 0:
        options.types = ','.join(args)
    mavflightview_mav(MAVExpLog, options)

def cmd_map(args):
    '''map command'''
    child = multiprocessing.Process(target=map_process, args=[args, mestate.mlog, mestate.flightmode_selections, mestate.settings])
    child.start()

def cmd_set(args):
    '''control MAVExporer options'''
    mestate.settings.command(args)

def cmd_condition(args):
    '''control MAVExporer conditions'''
    if len(args) == 0:
        print("condition is: %s" % mestate.settings.condition)
        return
    mestate.settings.condition = ' '.join(args)
    if len(mestate.settings.condition) == 0 or mestate.settings.condition == 'clear':
        mestate.settings.condition = None

def cmd_reload(args):
    '''reload graphs'''
    mestate.console.writeln('Reloading graphs', fg='blue')
    load_graphs()
    setup_menus()
    mestate.console.write("Loaded %u graphs\n" % len(mestate.graphs))

def save_graph(graphdef, mestate):
    '''save a graph as XML'''
    if graphdef.filename is None:
        if 'HOME' in os.environ:
            dname = os.path.join(os.environ['HOME'], '.mavproxy')
            if os.path.exists(dname):
                mp_util.mkdir_p(dname)
                graphdef.filename = os.path.join(dname, 'mavgraphs.xml')
        elif 'LOCALAPPDATA' in os.environ:
            dname = os.path.join(os.environ['LOCALAPPDATA'], 'MAVProxy')
            if os.path.exists(dname):
                mp_util.mkdir_p(dname)
                graphdef.filename = os.path.join(dname, 'mavgraphs.xml')
        else:
            graphdef.filename = 'mavgraphs.xml'
    if graphdef.filename is None:
        mestate.console.writeln("No file to save graph to", fg='red')
        return
    try:
        graphs = load_graph_xml(open(graphdef.filename).read(), graphdef.filename, load_all=True)
    except Exception:
        graphs = []
    found_name = False
    for i in range(len(graphs)):
        if graphs[i].name == graphdef.name:
            graphs[i] = graphdef
            found_name = True
            break
    if not found_name:
        graphs.append(graphdef)
    mestate.console.writeln("Saving %u graphs to %s" % (len(graphs), graphdef.filename))
    f = open(graphdef.filename, "w")
    f.write("<graphs>\n\n")
    for g in graphs:
        f.write(" <graph name='%s'>\n" % g.name.strip())
        if g.description is None:
            g.description = ''
        f.write("  <description>%s</description>\n" % g.description.strip())
        for e in g.expressions:
            f.write("  <expression>%s</expression>\n" % e.strip())
        f.write(" </graph>\n\n")
    f.write("</graphs>\n")
    f.close()

def save_callback(operation, graphdef):
    '''callback from save thread'''
    if operation == 'test':
        for e in graphdef.expressions:
            if expression_ok(e):
                graphdef.expression = e
                display_graph(graphdef)
                return
        mestate.console.writeln('Invalid graph expressions', fg='red')
        return
    if operation == 'save':
        save_graph(graphdef, mestate)

def save_process(MAVExpLastGraph):
    '''process for saving a graph'''
    from MAVProxy.modules.lib import wx_processguard
    from MAVProxy.modules.lib.wx_loader import wx
    from MAVProxy.modules.lib.wxgrapheditor import GraphDialog
    app = wx.App(False)
    frame = GraphDialog('Graph Editor',
                        MAVExpLastGraph,
                        save_callback)
    frame.ShowModal()
    frame.Destroy()
    

def cmd_save(args):
    '''save a graph'''
    child = multiprocessing.Process(target=save_process, args=[mestate.last_graph])
    child.start()

def cmd_param(args):
    '''show parameters'''
    if len(args) > 0:
        wildcard = args[0]
    else:
        wildcard = '*'
    k = sorted(mestate.mlog.params.keys())
    for p in k:
        if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
            print("%-16.16s %f" % (str(p), mestate.mlog.params[p]))
            
def cmd_loadfile(args):
    '''callback from menu to load a log file'''
    if len(args) != 1:
        print "Error loading file"
        return
    loadfile(args[0])
    
def loadfile(args):    
    '''load a log file (path given by arg)'''
    mestate.console.write("Loading %s...\n" % args)
    t0 = time.time()
    mlog = mavutil.mavlink_connection(args, notimestamps=False,
                                      zero_time_base=False)
    mestate.mlog = mavmemlog.mavmemlog(mlog, progress_bar)
    mestate.status.msgs = mlog.messages
    t1 = time.time()
    mestate.console.write("\ndone (%u messages in %.1fs)\n" % (mestate.mlog._count, t1-t0))

    load_graphs()
    setup_menus()

def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)

    line = line.strip()
    if not line:
        return

    args = line.split()
    cmd = args[0]
    if cmd == 'help':
        k = command_map.keys()
        k.sort()
        for cmd in k:
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    if cmd == 'exit':
        mestate.exit = True
        return

    if not cmd in command_map:
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command %s: %s" % (args[1:], str(e)))

def input_loop():
    '''wait for user input'''
    while mestate.exit != True:
        try:
            if mestate.exit != True:
                line = raw_input(mestate.rl.prompt)
        except EOFError:
            mestate.exit = True
            sys.exit(1)
        mestate.input_queue.put(line)

def main_loop():
    '''main processing loop, display graphs and maps'''
    while True:
        if mestate is None or mestate.exit:
            return
        while not mestate.input_queue.empty():
            line = mestate.input_queue.get()
            cmds = line.split(';')
            for c in cmds:
                process_stdin(c)
        time.sleep(0.1)

command_map = {
    'graph'      : (cmd_graph,     'display a graph'),
    'set'        : (cmd_set,       'control settings'),
    'reload'     : (cmd_reload,    'reload graphs'),
    'save'       : (cmd_save,      'save a graph'),
    'condition'  : (cmd_condition, 'set graph conditions'),
    'param'      : (cmd_param,     'show parameters'),
    'map'        : (cmd_map,       'show map view'),
    'loadLog'    : (cmd_loadfile,  'load a log file'),
    }

def progress_bar(pct):
    if pct % 2 == 0:
        mestate.console.write('#')

if __name__ == "__main__":
    multiprocessing.freeze_support()
    mestate = MEState()
    setup_file_menu()
    
    mestate.rl = rline.rline("MAV> ", mestate)
    
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("files", metavar="<FILE>", nargs="?")
    args = parser.parse_args() 

    #If specified, open the log file
    if args.files != None and len(args.files) != 0: 
        loadfile(args.files)     

    # run main loop as a thread
    mestate.thread = threading.Thread(target=main_loop, name='main_loop')
    mestate.thread.daemon = True
    mestate.thread.start()

    # input loop
    while mestate.rl != None and mestate.exit != True:        
        try:
            try:           
                line = raw_input(mestate.rl.prompt)
            except EOFError:
                mestate.exit = True
                break
            mestate.input_queue.put(line)
        except KeyboardInterrupt:
            mestate.exit = True
            break
            
