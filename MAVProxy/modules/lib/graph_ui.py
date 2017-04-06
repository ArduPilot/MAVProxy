from MAVProxy.modules.lib import grapher
import platform
if platform.system() == 'Darwin':
	from billiard import Pipe, Process, Event, forking_enable, freeze_support
else:
	from multiprocessing import Pipe, Process, Event, freeze_support

class Graph_UI(object):
	"""docstring for ClassName"""
	def __init__(self, mestate):
		self.mestate = mestate

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
	    self.mg.process(self.mestate.flightmode_selections, self.mestate.mlog._flightmodes)
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
