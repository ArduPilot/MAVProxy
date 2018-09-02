#!/usr/bin/env python

from __future__ import print_function
import os, wx, pickle

'''
handle saving/loading of window positions
'''

window_list = {}
display_size = None

class WinLayout(object):
    '''represent window layout'''
    def __init__(self, name, pos, size, dsize):
        self.name = name
        self.pos = pos
        self.size = size
        self.dsize = dsize
        
class ManagedWindow(object):
    '''a layout plus callback for setting window position and size'''
    def __init__(self, layout, callback):
        self.layout = layout
        self.callback = callback

def get_wx_window_layout(wx_window):
    '''get a WinLayout for a wx window'''
    dsize = wx.DisplaySize()
    pos = wx_window.GetPosition()
    size = wx_window.GetSize()
    name = wx_window.GetTitle()
    return WinLayout(name, pos, size, dsize)

def set_wx_window_layout(wx_window, layout):
    '''set a WinLayout for a wx window'''
    try:
        wx_window.SetSize(layout.size)
        wx_window.SetPosition(layout.pos)
    except Exception as ex:
        print(ex)

def set_layout(wlayout, callback):
    '''set window layout'''
    global display_size
    global window_list
    window_list[wlayout.name] = ManagedWindow(wlayout, callback)
    display_size = wlayout.dsize

def layout_filename():
    '''get location of layout file'''
    global display_size
    (dw,dh) = display_size
    if 'HOME' in os.environ:
        return os.path.join(os.environ['HOME'], ".mavlayout-%ux%u" % (dw,dh))
    if 'LOCALAPPDATA' in os.environ and not opts.setup:
        return os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy", "mavlayout-%ux%x.dat" % (dw,dh))
    return None

def save_layout():
    '''save window layout'''
    global display_size
    global window_list
    if display_size is None:
        print("No layouts to save")
        return
    fname = layout_filename()
    if fname is None:
        print("No file to save layout to")
        return
    layout = {}
    for name in window_list:
        layout[name] = window_list[name].layout
    pickle.dump(layout, open(fname,"w"))

def load_layout():
    '''load window layout'''
    global display_size
    global window_list
    if display_size is None:
        print("No layouts to load")
        return
    fname = layout_filename()
    if fname is None:
        print("No file to load layout from")
        return
    layout = pickle.load(open(fname,"r"))
    for name in window_list:
        if name in layout:
            try:
                window_list[name].callback(layout[name])
            except Exception as ex:
                print(ex)
    
