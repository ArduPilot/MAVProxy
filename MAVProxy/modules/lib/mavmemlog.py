'''in-memory mavlink log'''

from pymavlink import mavutil

class mavmemlog(mavutil.mavfile):
    '''a MAVLink log in memory. This allows loading a log into
    memory to make it easier to do multiple sweeps over a log'''
    def __init__(self, mav, progress_callback=None):
        mavutil.mavfile.__init__(self, None, 'memlog')
        self._msgs = []
        self._count = 0
        self.rewind()
        self._flightmodes = []
        last_flightmode = None
        last_timestamp = None
        last_pct = 0
        
        while True:
            m = mav.recv_msg()
            if m is None:
                break
            if int(mav.percent) != last_pct and progress_callback:
                progress_callback(int(mav.percent))
                last_pct = int(mav.percent)
            self._msgs.append(m)
            if mav.flightmode != last_flightmode:
                if len(self._flightmodes) > 0:
                    (mode, t1, t2) = self._flightmodes[-1]
                    self._flightmodes[-1] = (mode, t1, m._timestamp)
                self._flightmodes.append((mav.flightmode, m._timestamp, None))
                last_flightmode = mav.flightmode
            self._count += 1
            last_timestamp = m._timestamp
            self.check_param(m)
        if last_timestamp is not None and len(self._flightmodes) > 0:
            (mode, t1, t2) = self._flightmodes[-1]
            self._flightmodes[-1] = (mode, t1, last_timestamp)
        

    def recv_msg(self):
        '''message receive routine'''
        if self._index >= self._count:
            return None
        m = self._msgs[self._index]
        type = m.get_type()
        self._index += 1
        self.percent = (100.0 * self._index) / self._count
        self.messages[type] = m
        self._timestamp = m._timestamp

        if self._flightmode_index < len(self._flightmodes):
            (mode, tstamp, t2) = self._flightmodes[self._flightmode_index]
            if m._timestamp >= tstamp:
                self.flightmode = mode
                self._flightmode_index += 1

        self.check_param(m)
        return m

    def check_param(self, m):
        type = m.get_type()
        if type == 'PARAM_VALUE':
            s = str(m.param_id)
            self.params[str(m.param_id)] = m.param_value
        elif type == 'PARM' and getattr(m, 'Name', None) is not None:
            self.params[m.Name] = m.Value

    def rewind(self):
        '''rewind to start'''
        self._index = 0
        self.percent = 0
        self.messages = {}
        self._flightmode_index = 0
        self._timestamp = None
        self.flightmode = None
        self.params = {}

    def flightmode_list(self):
        '''return list of all flightmodes as tuple of mode and start time'''
        return self._flightmodes

    def reduce_by_flightmodes(self, flightmode_selections):
        '''reduce data using flightmode selections'''
        if len(flightmode_selections) == 0:
            return
        all_false = True
        for s in flightmode_selections:
            if s:
                all_false = False
        if all_false:
            # treat all false as all modes wanted'''
            return
        new_msgs = []
        idx = 0
        for m in self._msgs:
            while idx < len(self._flightmodes) and m._timestamp >= self._flightmodes[idx][2]:
                idx += 1
            if idx < len(flightmode_selections) and flightmode_selections[idx]:
                new_msgs.append(m)
        self._msgs = new_msgs
        self._count = len(new_msgs)
        self.rewind()
