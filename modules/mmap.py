import os
import sys
import webbrowser

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'lib'))
import mmap_server

g_module_context = None


class message_memo(object):
  def __init__(self):
    self._d = dict({})
    self.time = 0

  def insert_message(self, m):
    mtype = m.get_type().upper()
    if mtype == "GPS_RAW_INT":
      self.time = m.time_usec
    if mtype in self._d:
      (oldtime, n, oldm) = self._d[mtype]
      self._d[mtype] = (self.time, n+1, m)
    else:
      self._d[mtype] = (self.time, 0, m)

  def get_message(self,mt):
    mtype = mt.upper() # arg is case insensitive
    if mtype in self._d:
      return self._d[mtype]
    else:
      return None

  def has_message(self,mt):
    mtype = mt.upper()
    if mtype in self._d:
      return True
    else:
      return False

class module_state(object):
  def __init__(self):
    self.lat = None
    self.lon = None
    self.alt = None
    self.airspeed = None
    self.groundspeed = None
    self.heading = None
    self.pitch = None
    self.roll = None
    self.yaw = None
    self.gps_fix_type = None
    self.wp_change_time = 0
    self.waypoints = []
    self.fence_change_time = 0
    self.server = None
    self.messages = message_memo()


def name():
  """return module name"""
  return 'mmap'


def description():
  """return module description"""
  return 'modest map display'


def init(module_context):
  """initialise module"""
  global g_module_context
  g_module_context = module_context
  state = module_state()
  g_module_context.mmap_state = state
  state.server = mmap_server.start_server(
    '0.0.0.0', port=9999, module_state=state)
  webbrowser.open('http://127.0.0.1:9999/', autoraise=True)


def unload():
  """unload module"""
  global g_module_context
  g_module_context.mmap_state.server.terminate()


def mavlink_packet(m):
  """handle an incoming mavlink packet"""
  global g_module_context
  state = g_module_context.mmap_state
  state.messages.insert_message(m)
  if m.get_type() == 'GPS_RAW':
    (state.lat, state.lon) = (m.lat, m.lon)
  elif m.get_type() == 'GPS_RAW_INT':
    (state.lat, state.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
    state.gps_fix_type = m.fix_type
  elif m.get_type() == "VFR_HUD":
    state.heading = m.heading
    state.alt = m.alt
    state.airspeed = m.airspeed
    state.groundspeed = m.groundspeed
  elif m.get_type() == "ATTITUDE":
    state.pitch = m.pitch
    state.roll = m.roll
    state.yaw = m.yaw
  # if the waypoints have changed, redisplay
  if state.wp_change_time != g_module_context.status.wploader.last_change:
    state.wp_change_time = g_module_context.status.wploader.last_change
    state.waypoints = g_module_context.status.wploader.wpoints[:]
