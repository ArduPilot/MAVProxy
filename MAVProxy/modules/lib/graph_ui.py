from MAVProxy.modules.lib import grapher
import platform
if platform.system() == 'Darwin':
    from billiard import Process, forking_enable, freeze_support
else:
    from multiprocessing import Process, freeze_support

graph_count = 1

class Graph_UI(object):
    """docstring for ClassName"""
    def __init__(self, mestate):
        self.mestate = mestate
        self.xlim = None
        global graph_count
        self.count = graph_count
        graph_count += 1

    def display_graph(self, graphdef):
        '''display a graph'''
        if 'mestate' in globals():
            self.mestate.console.write("Expression: %s\n" % ' '.join(graphdef.expression.split()))
        else:
            self.mestate.child_pipe_send_console.send("Expression: %s\n" % ' '.join(graphdef.expression.split()))
        #mestate.mlog.reduce_by_flightmodes(mestate.flightmode_selections)

        #setup the graph, then pass to a new process and display
        self.mg = grapher.MavGraph()
        self.mg.set_marker(self.mestate.settings.marker)
        self.mg.set_condition(self.mestate.settings.condition)
        self.mg.set_xaxis(self.mestate.settings.xaxis)
        self.mg.set_linestyle(self.mestate.settings.linestyle)
        self.mg.set_show_flightmode(self.mestate.settings.show_flightmode)
        self.mg.set_legend(self.mestate.settings.legend)
        self.mg.add_mav(self.mestate.mlog)
        for f in graphdef.expression.split():
            self.mg.add_field(f)
        from multiprocessing_queue import makeIPCQueue
        self.xlim_notify_queue = makeIPCQueue()
        self.xlim_change_queue = makeIPCQueue()
        self.mg.process(self.mestate.flightmode_selections, self.mestate.mlog._flightmodes,
                        xlim_notify_queue=self.xlim_notify_queue,
                        xlim_change_queue=self.xlim_change_queue)
        self.lenmavlist = len(self.mg.mav_list)
        if platform.system() == 'Darwin':
            forking_enable(False)
        #Important - mg.mav_list is the full logfile and can be very large in size
        #To avoid slowdowns in Windows (which copies the vars to the new process)
        #We need to empty this var when we're finished with it
        self.mg.mav_list = []
        child = Process(target=self.mg.show, args=[self.lenmavlist, ])
        child.start()
        self.mestate.mlog.rewind()

    def check_xlim_change(self):
        '''check for new X bounds'''
        if not self.xlim_notify_queue:
            return None
        xlim = None
        while not self.xlim_notify_queue.empty():
            xlim = self.xlim_notify_queue.get()
        if xlim != self.xlim:
            return xlim
        return None

    def set_xlim(self, xlim):
        '''set new X bounds'''
        if self.xlim_change_queue and self.xlim != xlim:
            self.xlim_change_queue.put(xlim, block=False)
            self.xlim = xlim
