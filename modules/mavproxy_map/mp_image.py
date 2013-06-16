#!/usr/bin/env python
'''
display a image with scrollbars in a subprocess
Andrew Tridgell
June 2012
'''

import cv2.cv as cv
import time
import wx

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map import mp_widgets


class MPImage():
    '''
    a generic image viewer widget for use in MP tools
    '''
    def __init__(self,
                 title='MPImage',
                 width=512,
                 height=512,
                 events=[]):
        import multiprocessing

        self.title = title
        self.width = width
        self.height = height
        self._events = events
        self.in_queue = multiprocessing.Queue()
        self.out_queue = multiprocessing.Queue()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import wx
        state = self
        
        self.app = wx.PySimpleApp()
        self.app.frame = MPImageFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def set_image(self, img, bgr=False):
        if bgr:
            img = cv.CloneImage(img)
            cv.CvtColor(img, img, cv.CV_BGR2RGB)
        self.in_queue.put(((img.width, img.height), img.tostring()))

    def poll(self):
        '''check for events, returning one event'''
        if self.out_queue.qsize():
            return self.out_queue.get()
        return None            

    def events(self):
        '''check for events a list of events'''
        ret = []
        while self.out_queue.qsize():
            ret.append(self.out_queue.get())
        return ret

from PIL import Image

class MPImageFrame(wx.Frame):
    """ The main frame of the viewer
    """    
    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        self.state = state
        state.frame = self
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        state.panel = MPImagePanel(self, state)
        self.sizer.Add(state.panel, 1, wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        
    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        state = self.state
        time.sleep(0.1)

class MPImagePanel(wx.PyScrolledWindow):
    """ The image panel
    """    
    def __init__(self, parent, state):
        wx.PyScrolledWindow.__init__(self, parent)
        self.state = state
        self.img = None
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.Bind(wx.EVT_SET_FOCUS, self.on_focus)
        self.redraw_timer.Start(200)

        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.mainSizer)

        # panel for the main image
        self.imagePanel = mp_widgets.ImagePanel(self, wx.EmptyImage(state.width,state.height))
        self.mainSizer.Add(self.imagePanel, flag=wx.TOP|wx.LEFT|wx.GROW, border=5)
        for ev in state._events:
            self.imagePanel.Bind(ev, self.on_event)

        self.redraw()
        state.frame.Fit()
        self.SetScrollbars(1, 1, 100, 100)

    def on_focus(self, event):
        self.imagePanel.SetFocus()

    def redraw(self):
        '''redraw the image with current settings'''
        state = self.state

        self.mainSizer.Fit(self)
        self.Refresh()
        state.frame.Refresh()
        self.SetFocus()
        
    def on_redraw_timer(self, event):
        '''the redraw timer ensures we show new map tiles as they
        are downloaded'''
        state = self.state
        while state.in_queue.qsize():
            obj = state.in_queue.get()
            (size, imgstr) = obj
            img = wx.EmptyImage(size[0], size[1])
            img.SetData(imgstr)
            self.imagePanel.set_image(img)
            self.redraw()

    def on_event(self, event):
        '''pass events to the parent'''
        state = self.state
        if (isinstance(event, wx.MouseEvent) and
            not event.ButtonIsDown(wx.MOUSE_BTN_ANY) and
            event.GetWheelRotation() == 0):
            # don't flood the queue with mouse movement
            return
        state.out_queue.put(mp_util.object_container(event))
            
if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser("mp_image.py <file>")
    (opts, args) = parser.parse_args()
    
    im = MPImage(events=[wx.EVT_MOUSE_EVENTS, wx.EVT_KEY_DOWN])
    img = cv.LoadImage(args[0])

    im.set_image(img)

    while im.is_alive():
        for event in im.events():
            print event.ClassName
            if event.ClassName == 'wxMouseEvent':
                print 'mouse', event.X, event.Y
            if event.ClassName == 'wxKeyEvent':
                print 'key %u' % event.KeyCode
        time.sleep(0.1)
