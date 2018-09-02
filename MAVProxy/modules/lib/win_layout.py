#!/usr/bin/env python

from __future__ import print_function
import os, wx, pickle

'''
handle saving/loading of window positions
'''

window_list = {}
display_size = None
loaded_layout = None
pending_load = False

class WinLayout(object):
    '''represent window layout'''
    def __init__(self, name, pos, size, dsize):
        self.name = name
        self.pos = pos
        self.size = size
        self.dsize = dsize

    def __str__(self):
        return "%s(%ux%u@%u-%u)" % (self.name,
                                    self.size[0], self.size[1], 
                                    self.pos[0], self.pos[1])
        
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
    global loaded_layout
    global pending_load
    #if not wlayout.name in window_list:
    #    print("layout %s" % wlayout)
    if not wlayout.name in window_list and loaded_layout is not None and wlayout.name in loaded_layout:
        callback(loaded_layout[wlayout.name])
    window_list[wlayout.name] = ManagedWindow(wlayout, callback)
    display_size = wlayout.dsize
    if pending_load:
        pending_load = False
        load_layout()

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
    try:
        # include previous layout, so we retain layouts for widows not
        # currently displayed
        layout = pickle.load(open(fname,"r"))
    except Exception:
        pass
    count = 0
    for name in window_list:
        layout[name] = window_list[name].layout
        count += 1
    pickle.dump(layout, open(fname,"w"))
    print("Saved layout for %u windows" % count)

def load_layout():
    '''load window layout'''
    global display_size
    global window_list
    global loaded_layout
    global pending_load
    if display_size is None:
        pending_load = True
        return
    fname = layout_filename()
    if fname is None:
        print("No file to load layout from")
        return
    layout = pickle.load(open(fname,"r"))
    count = 0
    for name in window_list:
        if name in layout:
            try:
                window_list[name].callback(layout[name])
                count += 1
            except Exception as ex:
                print(ex)
    loaded_layout = layout
    print("Loaded layout for %u windows" % count)
    
