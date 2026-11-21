"""
  MAVProxy realtime graphing module

  uses lib/live_graph.py for display
"""

from pymavlink import mavutil
import re, os, sys, time

from MAVProxy.modules.lib import live_graph

from MAVProxy.modules.lib import mp_module

class LogStream():
    '''an onboard log message field the vehicle has been asked to stream
    as a named value (LOG_STREAM_NAMED_VALUE)'''
    def __init__(self, msg_name, field_name, instance, stream_name):
        self.msg_name = msg_name
        self.field_name = field_name
        self.instance = instance
        self.stream_name = stream_name
        self.last_request_time = 0
        self.last_value_time = 0
        self.request_interval = 3
        # the vehicle the stream was requested from, recorded when the
        # first request is sent:
        self.target_system = None
        self.target_component = None
        # when a stop was sent, or None while the stream is wanted:
        self.stop_time = None
        # times the stream has been moved to a generated name after
        # the vehicle refused its current one:
        self.rename_count = 0

    def name(self):
        '''the natural name for the field (before truncation to the
        10-character named-value name field)'''
        if self.instance < 0:
            return "%s.%s" % (self.msg_name, self.field_name)
        return "%s[%u].%s" % (self.msg_name, self.instance, self.field_name)

class GraphModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "graph", "graph control")
        self.timespan = 20
        self.tickresolution = 0.2
        self.graphs = []
        # log-field streams requested from the vehicle, keyed by the
        # name the values arrive under.  A field whose natural name is
        # already in use by a different field is streamed under a
        # generated name instead:
        self.log_streams = {}
        # names the vehicle has refused; never offered again this
        # session:
        self.retired_names = set()
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

    def request_log_stream(self, expression):
        '''if expression is a field of an onboard log message
        (eg. "ATT.Roll" or "XKF4[1].SS") rather than a mavlink expression,
        ask the vehicle to stream that field as a named value
        (LOG_STREAM_NAMED_VALUE) and return the name the value will be
        streamed under'''
        m = re.match(r'^([A-Z][A-Z0-9_]{0,3})(?:\[(\d+)\])?\.([A-Za-z][A-Za-z0-9_]{0,15})(?:<[^>]*>)?$', expression)
        if m is None:
            return None
        if hasattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + m.group(1)):
            # this is a mavlink message name; treat as a normal expression
            return None
        msg_class = getattr(mavutil.mavlink, 'MAVLink_log_stream_named_value_message', None)
        if msg_class is None or 'interval_us' not in msg_class.fieldnames:
            print("graph: pymavlink is too old to stream %s" % expression)
            return None
        instance = -1 if m.group(2) is None else int(m.group(2))
        if instance > 32767:
            print("graph: instance in %s too large to stream" % expression)
            return None
        stream = LogStream(m.group(1), m.group(3), instance, None)
        # an existing stream of this field is reused whatever name it
        # streams under; one being stopped is restarted:
        for existing in self.log_streams.values():
            if existing.name() == stream.name():
                if existing.stop_time is not None:
                    existing.stop_time = None
                    self.send_log_stream_request(existing)
                return existing.stream_name
        # the natural stream name is "MSG.Field" (or "MSG[N].Field")
        # truncated to fit the 10-character named-value name field; if
        # that name is already in use (including by a stream still
        # being stopped), stream under a generated name instead:
        base = stream.name()[:10]
        key = base
        count = 0
        while key in self.log_streams or key in self.retired_names:
            count += 1
            suffix = "~%u" % count
            key = base[:10 - len(suffix)] + suffix
        if key != base:
            print("graph: streaming %s as %s" % (expression, key))
        stream.stream_name = key
        self.log_streams[key] = stream
        self.send_log_stream_request(stream)
        return key

    def rename_stream(self, stream):
        '''move a stream to a generated name and re-request it,
        retiring its old name for the session; used when the vehicle
        refuses the name, eg. one still streaming a different field
        requested by a previous MAVProxy session'''
        old = stream.stream_name
        if stream.last_value_time != 0:
            # the name has been delivering values -- a leftover stream,
            # or our own working stream renamed by a refusal meant for
            # another requester sharing this channel; stop it rather
            # than leave it running under a retired name:
            self.send_log_stream_request(stream, interval_us=-1)
        self.retired_names.add(old)
        del self.log_streams[old]
        stream.last_value_time = 0
        base = stream.name()[:10]
        key = base
        count = 0
        while key in self.log_streams or key in self.retired_names:
            count += 1
            suffix = "~%u" % count
            key = base[:10 - len(suffix)] + suffix
        print("graph: streaming %s as %s" % (stream.name(), key))
        stream.stream_name = key
        self.log_streams[key] = stream
        for g in self.graphs:
            g.named_value_keys = [key if k == old else k
                                  for k in g.named_value_keys]
        self.send_log_stream_request(stream)

    def send_log_stream_request(self, stream, interval_us=0):
        '''(re-)send the request for a log-field stream; the vehicle
        treats a repeated request as a no-op.  An interval_us of -1
        stops the stream instead'''
        if self.master is None:
            return
        if stream.target_system is None:
            stream.target_system = self.settings.target_system
            stream.target_component = self.settings.target_component

        sent = []

        def send(mav):
            mav.log_stream_named_value_send(
                stream.target_system,
                stream.target_component,
                stream.msg_name.encode('utf-8'),
                stream.field_name.encode('utf-8'),
                stream.instance,
                stream.stream_name.encode('utf-8'),
                interval_us)
            sent.append(mav)

        if interval_us < 0:
            # the vehicle stops a stream per channel, and with
            # redundant links an earlier request may have gone out on
            # a different link; stop on every link carrying the
            # vehicle.  Links are mapped by the component ids seen on
            # them, so a target_component of 0 looks up the autopilot:
            compid = stream.target_component
            if compid == 0:
                compid = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            self.mpstate.foreach_mav(stream.target_system, compid, send)
        if not sent:
            # the stream's vehicle may be on a different link from the
            # currently-selected one:
            master = self.mpstate.master(stream.target_system)
            if master is None:
                return
            send(master.mav)
        stream.last_request_time = time.time()

    def idle_task(self):
        '''re-request any stream an open graph is waiting on -- covering
        a request sent with no link up or lost in transit, a vehicle
        reboot and a change of target vehicle -- and stop any stream no
        open graph wants any more'''
        now = time.time()
        wanted = set()
        for g in self.graphs:
            wanted.update(k for k in g.named_value_keys if k is not None)
        for key in list(self.log_streams.keys()):
            stream = self.log_streams[key]
            if key not in wanted:
                if stream.target_system is None:
                    # the vehicle was never asked for this stream
                    del self.log_streams[key]
                    continue
                if self.master is None:
                    continue
                if stream.stop_time is None or (
                        stream.last_value_time > stream.stop_time and
                        now - stream.last_request_time >= 3):
                    # stop the stream; the stop is re-sent while values
                    # keep arriving (a lost stop).  The entry is kept,
                    # reserving the name for the rest of the session,
                    # so a stream surviving a lost stop can not be
                    # plotted under another field's label:
                    self.send_log_stream_request(stream, interval_us=-1)
                    stream.stop_time = now
                continue
            if now - stream.last_value_time < 3:
                continue
            if now - stream.last_request_time < stream.request_interval:
                continue
            if (self.master is not None and
                    stream.target_system is not None and
                    stream.target_system != self.settings.target_system):
                # the stream follows the selected vehicle; stop it on
                # the vehicle it was requested from first:
                self.send_log_stream_request(stream, interval_us=-1)
                stream.target_system = self.settings.target_system
                stream.target_component = self.settings.target_component
            self.send_log_stream_request(stream)
            if stream.last_value_time == 0:
                # a request the vehicle refuses draws a warning each
                # time it is repeated; back off until a value arrives
                stream.request_interval = min(stream.request_interval * 2, 30)

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
        # stop the vehicle's streams; a new module instance has no
        # record of them, so they would otherwise run until reboot.
        # The table is emptied first so a refusal arriving mid-unload
        # can not mutate it or restart a stream:
        streams = list(self.log_streams.values())
        self.log_streams = {}
        if self.master is None:
            return
        for stream in streams:
            if stream.target_system is None:
                # the vehicle was never asked for this stream
                continue
            self.send_log_stream_request(stream, interval_us=-1)

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''

        if msg.get_type() == 'STATUSTEXT':
            # the vehicle refuses a name already streaming a different
            # field, eg. one requested by a previous MAVProxy session;
            # retry such a stream under a generated name.  The retries
            # are bounded, as a genuinely-bad field is refused whatever
            # it is named:
            m = re.match(r"Unable to stream (.+)$", msg.text)
            if m is not None:
                stream = self.log_streams.get(m.group(1))
                if (stream is not None and stream.stop_time is None and
                        stream.rename_count < 3):
                    stream.rename_count += 1
                    self.rename_stream(stream)

        if msg.get_type() in ('NAMED_VALUE_FLOAT', 'NAMED_VALUE_INT'):
            stream = self.log_streams.get(msg.name)
            if (stream is not None and
                    stream.target_system == msg.get_srcSystem()):
                # values are matched by name and source vehicle; a
                # same-named stream on another vehicle must not mask a
                # stall:
                stream.last_value_time = time.time()
                stream.request_interval = 3

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
                self.fields[i] = "NAMED_VALUE_FLOAT['%s'].%s" % (m.group(1), m.group(2))

        # fields which are onboard-log message fields (eg. "ATT.Roll") are
        # streamed by the vehicle as named values:
        self.named_value_keys = [None] * len(self.fields)
        for i in range(len(self.fields)):
            key = state.request_log_stream(self.fields[i])
            if key is not None:
                self.named_value_keys[i] = key
                self.msg_types.update(['NAMED_VALUE_FLOAT', 'NAMED_VALUE_INT'])

        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
        print("Adding graph: %s" % self.fields)

        fields = [ self.pretty_print_fieldname(x) for x in self.fields ]
        labels = []
        for i in range(len(fields)):
            f = fields[i]
            f1 = f.find("<")
            f2 = f.find(">")
            if f1 > 0 and f2 > f1:
                labels.append(f[f1+1:f2])
                fields[i] = f[:f1]
            else:
                labels.append(None)

        self.fields = fields[:]
        self.values = [None] * len(self.fields)
        self.livegraph = live_graph.LiveGraph(fields,
                                              timespan=state.timespan,
                                              tickresolution=state.tickresolution,
                                              title=fields[0] if labels[0] is None else labels[0],
                                              labels=labels)

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
            if self.named_value_keys[i] is not None:
                if (mtype in ('NAMED_VALUE_FLOAT', 'NAMED_VALUE_INT') and
                        msg.name == self.named_value_keys[i]):
                    stream = self.state.log_streams.get(msg.name)
                    if (stream is not None and
                            stream.target_system == msg.get_srcSystem()):
                        self.values[i] = msg.value
                        have_value = True
                continue
            if mtype not in self.field_types[i]:
                continue
            f = self.fields[i]
            self.values[i] = mavutil.evaluate_expression(f, self.state.master.messages)
            if self.values[i] is not None:
                have_value = True
        if have_value and self.livegraph is not None:
            self.livegraph.add_values(self.values)
