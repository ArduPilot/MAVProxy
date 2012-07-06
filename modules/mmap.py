import os
import sys
import webbrowser

import mavlinkv10

# FIXME: Please.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'lib'))
import mmap_server

g_module_context = None


class MetaMessage(object):
  def __init__(self, msg_type=None, data=None):
    self.msg_type = msg_type
    self.data = data or {}

  def get_type(self):
    return self.msg_type

  def to_dict(self):
    d = dict(self.data.items())
    d['mavpackettype'] = self.msg_type
    return d


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
      self._d[mtype] = (self.time, n + 1, m)
    else:
      self._d[mtype] = (self.time, 0, m)

  def get_message(self, mt):
    mtype = mt.upper()  # arg is case insensitive
    if mtype in self._d:
      return self._d[mtype]
    else:
      return None

  def has_message(self, mt):
    mtype = mt.upper()
    if mtype in self._d:
      return True
    else:
      return False


class module_state(object):
  def __init__(self):
    self.client_waypoint = None
    self.client_waypoint_seq = 0
    self.wp_change_time = 0
    self.waypoints = []
    self.fence_change_time = 0
    self.server = None
    self.messages = message_memo()

  def command(self, command):
    # First draft, assumes the command has a location and we want to
    # fly to the location right now.
    seq = 0
    frame = mavlinkv10.MAV_FRAME_GLOBAL_RELATIVE_ALT
    cmd = mavlinkv10.MAV_CMD_NAV_WAYPOINT
    param1 = 0  # Hold time in seconds.
    param2 = 5  # Acceptance radius in meters.
    param3 = 0  # Pass through the WP.
    param4 = 0  # Desired yaw angle at WP.
    x = command['location']['lat']
    y = command['location']['lon']
    self.client_waypoint = command['location']
    self.client_waypoint_seq += 1
    z = 400
    # APM specific current value, 2 means this is a "guided mode"
    # waypoint and not for the mission.
    current = 2
    autocontinue = 0
    msg = mavlinkv10.MAVLink_mission_item_message(
      g_module_context.status.target_system,
      g_module_context.status.target_component,
      seq, frame, cmd, current, autocontinue, param1, param2, param3, param4,
      x, y, z)
    g_module_context.queue_message(msg)
    msg = MetaMessage(msg_type='META_WAYPOINT',
                      data={'waypoint': {'lat': x, 'lon': y}})
    self.messages.insert_message(msg)


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
  # if the waypoints have changed, redisplay
  if state.wp_change_time != g_module_context.status.wploader.last_change:
    state.wp_change_time = g_module_context.status.wploader.last_change
    state.waypoints = g_module_context.status.wploader.wpoints[:]
