#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
slipmap based on mp_tile
Andrew Tridgell
June 2012
'''

import functools
import math
import os, sys
import time
import cv2
import numpy as np

from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.mavproxy_map import mp_tile
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import win_layout
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.mavproxy_map.mp_slipmap_util import *


class MPSlipMap():
    '''
    a generic map viewer widget for use in mavproxy
    '''
    def __init__(self,
                 title='SlipMap',
                 lat=-35.362938,
                 lon=149.165085,
                 width=800,
                 height=600,
                 ground_width=1000,
                 tile_delay=0.3,
                 service="MicrosoftSat",
                 max_zoom=19,
                 debug=False,
                 brightness=0,
                 elevation=False,
                 download=True,
                 show_flightmode_legend=True,
                 timelim_pipe=None):

        self.lat = lat
        self.lon = lon
        self.width = width
        self.height = height
        self.ground_width = ground_width
        self.download = download
        self.service = service
        self.tile_delay = tile_delay
        self.debug = debug
        self.max_zoom = max_zoom
        self.elevation = elevation
        self.oldtext = None
        self.brightness = brightness
        self.legend = show_flightmode_legend
        self.timelim_pipe = timelim_pipe

        self.drag_step = 10

        self.title = title
        self.event_queue = multiproc.Queue()
        self.object_queue = multiproc.Queue()
        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()
        self._callbacks = set()


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_map.mp_slipmap_ui import MPSlipMapFrame

        state = self

        self.mt = mp_tile.MPTile(download=self.download,
                                 service=self.service,
                                 tile_delay=self.tile_delay,
                                 debug=self.debug,
                                 max_zoom=self.max_zoom)
        state.layers = {}
        state.info = {}
        state.need_redraw = True

        self.app = wx.App(False)
        self.app.SetExitOnFrameDelete(True)
        self.app.frame = MPSlipMapFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()

    def close(self):
        '''close the window'''
        self.close_window.release()
        count=0
        while self.child.is_alive() and count < 30: # 3 seconds to die...
            time.sleep(0.1) #?
            count+=1

        if self.child.is_alive():
            self.child.terminate()

        self.child.join()

    def is_alive(self):
        '''check if graph is still going'''
        return self.child.is_alive()

    def add_object(self, obj):
        '''add or update an object on the map'''
        self.object_queue.put(obj)

    def remove_object(self, key):
        '''remove an object on the map by key'''
        self.object_queue.put(SlipRemoveObject(key))

    def set_zoom(self, ground_width):
        '''set ground width of view'''
        self.object_queue.put(SlipZoom(ground_width))

    def set_center(self, lat, lon):
        '''set center of view'''
        self.object_queue.put(SlipCenter((lat,lon)))

    def set_follow(self, enable):
        '''set follow on/off'''
        self.object_queue.put(SlipFollow(enable))

    def set_follow_object(self, key, enable):
        '''set follow on/off on an object'''
        self.object_queue.put(SlipFollowObject(key, enable))
        
    def hide_object(self, key, hide=True):
        '''hide an object on the map by key'''
        self.object_queue.put(SlipHideObject(key, hide))

    def set_position(self, key, latlon, layer='', rotation=0, label=None, colour=None):
        '''move an object on the map'''
        self.object_queue.put(SlipPosition(key, latlon, layer, rotation, label, colour))

    def event_queue_empty(self):
        '''return True if there are no events waiting to be processed'''
        return self.event_queue.empty()

    def set_layout(self, layout):
        '''set window layout'''
        self.object_queue.put(layout)
    
    def get_event(self):
        '''return next event or None'''
        if self.event_queue.empty():
            return None
        evt = self.event_queue.get()
        while isinstance(evt, win_layout.WinLayout):
            win_layout.set_layout(evt, self.set_layout)
            if self.event_queue.empty():
                return None
            evt = self.event_queue.get()
        return evt

    def add_callback(self, callback):
        '''add a callback for events from the map'''
        self._callbacks.add(callback)

    def check_events(self):
        '''check for events, calling registered callbacks as needed'''
        while not self.event_queue_empty():
            event = self.get_event()
            for callback in self._callbacks:
                callback(event)

    def icon(self, filename):
        '''load an icon from the data directory'''
        return mp_tile.mp_icon(filename)

if __name__ == "__main__":
    multiproc.freeze_support()
    import time

    from optparse import OptionParser
    parser = OptionParser("mp_slipmap.py [options]")
    parser.add_option("--lat", type='float', default=-26.582218, help="start latitude")
    parser.add_option("--lon", type='float', default=151.840113, help="start longitude")
    parser.add_option("--service", default="MicrosoftSat", help="tile service")
    parser.add_option("--offline", action='store_true', default=False, help="no download")
    parser.add_option("--delay", type='float', default=0.3, help="tile download delay")
    parser.add_option("--max-zoom", type='int', default=19, help="maximum tile zoom")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    parser.add_option("--boundary", default=None, help="show boundary")
    parser.add_option("--mission", default=[], action='append', help="show mission")
    parser.add_option("--thumbnail", default=None, help="show thumbnail")
    parser.add_option("--icon", default=None, help="show icon")
    parser.add_option("--flag", default=[], type='str', action='append', help="flag positions")
    parser.add_option("--grid", default=False, action='store_true', help="add a UTM grid")
    parser.add_option("--elevation", action='store_true', default=False, help="show elevation information")
    parser.add_option("--verbose", action='store_true', default=False, help="show mount actions")
    (opts, args) = parser.parse_args()

    sm = MPSlipMap(lat=opts.lat,
                   lon=opts.lon,
                   download=not opts.offline,
                   service=opts.service,
                   debug=opts.debug,
                   max_zoom=opts.max_zoom,
                   elevation=opts.elevation,
                   tile_delay=opts.delay)

    if opts.boundary:
        boundary = mp_util.polygon_load(opts.boundary)
        sm.add_object(SlipPolygon('boundary', boundary, layer=1, linewidth=2, colour=(0,255,0)))

    if opts.mission:
        from pymavlink import mavwp
        for file in opts.mission:
            wp = mavwp.MAVWPLoader()
            wp.load(file)
            boundary = wp.polygon()
            sm.add_object(SlipPolygon('mission-%s' % file, boundary, layer=1, linewidth=1, colour=(255,255,255)))

    if opts.grid:
        sm.add_object(SlipGrid('grid', layer=3, linewidth=1, colour=(255,255,0)))

    if opts.thumbnail:
        thumb = cv2.imread(opts.thumbnail)
        sm.add_object(SlipThumbnail('thumb', (opts.lat,opts.lon), layer=1, img=thumb, border_width=2, border_colour=(255,0,0)))

    if opts.icon:
        icon = cv2.imread(opts.icon)
        sm.add_object(SlipIcon('icon', (opts.lat,opts.lon), icon, layer=3, rotation=90, follow=True))
        sm.set_position('icon', mp_util.gps_newpos(opts.lat,opts.lon, 180, 100), rotation=45)
        sm.add_object(SlipInfoImage('detail', icon))
        sm.add_object(SlipInfoText('detail text', 'test text'))

    for flag in opts.flag:
        (lat,lon) = flag.split(',')
        icon = sm.icon('flag.png')
        sm.add_object(SlipIcon('icon - %s' % str(flag), (float(lat),float(lon)), icon, layer=3, rotation=0, follow=False))

    while sm.is_alive():
        while not sm.event_queue_empty():
            obj = sm.get_event()
            if not opts.verbose:
                continue
            if isinstance(obj, SlipMouseEvent):
                print("Mouse event at %s (X/Y=%u/%u) for %u objects" % (obj.latlon,
                                                                        obj.event.X, obj.event.Y,
                                                                        len(obj.selected)))
            if isinstance(obj, SlipKeyEvent):
                print("Key event at %s for %u objects" % (obj.latlon, len(obj.selected)))
        time.sleep(0.1)
