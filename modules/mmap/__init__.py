import os
import sys
import webbrowser

import mmap_server

g_module_context = None


class module_state(object):
  def __init__(self):
    self.lat = None
    self.lon = None
    self.alt = None
    self.speed = None
    self.heading = 0
    self.wp_change_time = 0
    self.fence_change_time = 0
    self.server = None


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
    '127.0.0.1', port=9999, module_state=state)
  webbrowser.open('http://127.0.0.1:9999/', autoraise=True)


def unload():
  """unload module"""
  global g_module_context
  g_module_context.mmap_state.server.terminate()


def mavlink_packet(m):
  """handle an incoming mavlink packet"""
  global g_module_context
  state = g_module_context.mmap_state
  if m.get_type() == 'GPS_RAW':
    (state.lat, state.lon) = (m.lat, m.lon)
  elif m.get_type() == 'GPS_RAW_INT':
    (state.lat, state.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
  elif m.get_type() == "VFR_HUD":
    state.heading = m.heading
    state.alt = m.alt
    state.airspeed = m.airspeed
    state.groundspeed = m.groundspeed
