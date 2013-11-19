#!/usr/bin/env python
'''
display a image in a subprocess
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
                 can_zoom = False,
                 can_drag = False,
                 mouse_events = False,
                 key_events = False):
        import multiprocessing

        self.title = title
        self.width = width
        self.height = height
        self.can_zoom = can_zoom
        self.can_drag = can_drag
        self.mouse_events = mouse_events
        self.key_events = key_events
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
        self.Bind(wx.EVT_SIZE, state.panel.on_size)
        
    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        state = self.state
        time.sleep(0.1)

class MPImagePanel(wx.Panel):
    """ The image panel
    """    
    def __init__(self, parent, state):
        wx.Panel.__init__(self, parent)
        self.frame = parent
        self.state = state
        self.img = None
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.Bind(wx.EVT_SET_FOCUS, self.on_focus)
        self.redraw_timer.Start(100)

        self.mouse_down = None
        self.drag_step = 10
        self.zoom = 1.0

        # dragpos is the top left position in image coordinates
        self.dragpos = wx.Point(0,0)
        self.need_redraw = True

        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.mainSizer)

        # panel for the main image
        self.imagePanel = mp_widgets.ImagePanel(self, wx.EmptyImage(state.width,state.height))
        self.mainSizer.Add(self.imagePanel, flag=wx.TOP|wx.LEFT|wx.GROW, border=0)
        if state.mouse_events:
            self.imagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.on_event)
        if state.key_events:
            self.imagePanel.Bind(wx.EVT_KEY_DOWN, self.on_event)
        else:
            self.imagePanel.Bind(wx.EVT_KEY_DOWN, self.on_key_event)
        self.imagePanel.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)

        self.redraw()
        state.frame.Fit()

    def on_focus(self, event):
        self.imagePanel.SetFocus()

    def on_focus(self, event):
        '''called when the panel gets focus'''
        self.imagePanel.SetFocus()

    def image_coordinates(self, point):
        '''given a point in window coordinates, calculate image coordinates'''
        # the dragpos is the top left position in image coordinates
        ret = wx.Point(int(self.dragpos.x + point.x/self.zoom),
                       int(self.dragpos.y + point.y/self.zoom))
        return ret

    def redraw(self):
        '''redraw the image with current settings'''
        state = self.state

        if self.img is None:
            self.mainSizer.Fit(self)
            self.Refresh()
            state.frame.Refresh()
            self.SetFocus()
            return

        # get the current size of the containing window frame
        size = self.frame.GetSize()
        (width, height) = (self.img.GetWidth(), self.img.GetHeight())

        rect = wx.Rect(self.dragpos.x, self.dragpos.y, int(size.x/self.zoom), int(size.y/self.zoom))

        #print("redraw", self.zoom, self.dragpos, size, rect);

        if rect.x > width-1:
            rect.x = width-1
        if rect.y > height-1:
            rect.y = height-1
        if rect.width > width - rect.x:
            rect.width = width - rect.x
        if rect.height > height - rect.y:
            rect.height = height - rect.y

        scaled_image = self.img.Copy()
        scaled_image = scaled_image.GetSubImage(rect);
        scaled_image = scaled_image.Rescale(int(rect.width*self.zoom), int(rect.height*self.zoom))
        self.imagePanel.set_image(scaled_image)
        self.need_redraw = False

        self.mainSizer.Fit(self)
        self.Refresh()
        state.frame.Refresh()
        self.SetFocus()
        '''
        from guppy import hpy
        h = hpy()
        print h.heap()
        '''

        
    def on_redraw_timer(self, event):
        '''the redraw timer ensures we show new map tiles as they
        are downloaded'''
        state = self.state
        while state.in_queue.qsize():
            obj = state.in_queue.get()
            (size, imgstr) = obj
            img = wx.EmptyImage(size[0], size[1])
            img.SetData(imgstr)
            self.img = img
            self.need_redraw = True
        if self.need_redraw:
            self.redraw()

    def on_size(self, event):
        '''handle window size changes'''
        self.need_redraw = True

    def limit_dragpos(self):
        '''limit dragpos to sane values'''
        if self.dragpos.x < 0:
            self.dragpos.x = 0
        if self.dragpos.y < 0:
            self.dragpos.y = 0
        if self.dragpos.x >= self.img.GetWidth():
            self.dragpos.x = self.img.GetWidth()-1
        if self.dragpos.y >= self.img.GetHeight():
            self.dragpos.y = self.img.GetHeight()-1

    def on_mouse_wheel(self, event):
        '''handle mouse wheel zoom changes'''
        state = self.state
        if not state.can_zoom:
            return
        mousepos = self.image_coordinates(event.GetPosition())
        rotation = event.GetWheelRotation() / event.GetWheelDelta()
        oldzoom = self.zoom
        if rotation > 0:
            self.zoom /= 1.0/(1.1 * rotation)
        elif rotation < 0:
            self.zoom /= 1.1 * (-rotation)
        if self.zoom > 10:
            self.zoom = 10
        elif self.zoom < 0.1:
            self.zoom = 0.1
        if oldzoom < 1 and self.zoom > 1:
            self.zoom = 1
        if oldzoom > 1 and self.zoom < 1:
            self.zoom = 1
        self.need_redraw = True
        new = self.image_coordinates(event.GetPosition())
        # adjust dragpos so the zoom doesn't change what pixel is under the mouse
        self.dragpos = wx.Point(self.dragpos.x - (new.x-mousepos.x), self.dragpos.y - (new.y-mousepos.y))
        self.limit_dragpos()

    def on_drag_event(self, event):
        '''handle mouse drags'''
        state = self.state
        if not state.can_drag:
            return
        newpos = self.image_coordinates(event.GetPosition())
        dx = -(newpos.x - self.mouse_down.x)
        dy = -(newpos.y - self.mouse_down.y)
        self.dragpos = wx.Point(self.dragpos.x+dx,self.dragpos.y+dy)
        self.limit_dragpos()
        self.mouse_down = newpos
        self.need_redraw = True
        self.redraw()

    def on_mouse_event(self, event):
        '''handle mouse events'''
        pos = event.GetPosition()
        if event.Leaving():
            self.mouse_pos = None
        else:
            self.mouse_pos = pos

        if event.LeftDown():
            self.mouse_down = self.image_coordinates(pos)
        if event.Dragging() and event.ButtonIsDown(wx.MOUSE_BTN_LEFT):
            self.on_drag_event(event)

    def on_key_event(self, event):
        '''handle key events'''
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_HOME:
            self.zoom = 1.0
            self.dragpos = wx.Point(0, 0)
            self.need_redraw = True

    def on_event(self, event):
        '''pass events to the parent'''
        state = self.state
        if isinstance(event, wx.MouseEvent):
            self.on_mouse_event(event)            
        if isinstance(event, wx.KeyEvent):
            self.on_key_event(event)
        if (isinstance(event, wx.MouseEvent) and
            not event.ButtonIsDown(wx.MOUSE_BTN_ANY) and
            event.GetWheelRotation() == 0):
            # don't flood the queue with mouse movement
            return
        state.out_queue.put(mp_util.object_container(event))
            
if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser("mp_image.py <file>")
    parser.add_option("--zoom", action='store_true', default=False, help="allow zoom")
    parser.add_option("--drag", action='store_true', default=False, help="allow drag")
    (opts, args) = parser.parse_args()
    
    im = MPImage(mouse_events=True,
                 key_events=True,
                 can_drag = opts.drag,
                 can_zoom = opts.zoom)
    img = cv.LoadImage(args[0])
    im.set_image(img, bgr=True)

    while im.is_alive():
        for event in im.events():
            print event.ClassName
            if event.ClassName == 'wxMouseEvent':
                print 'mouse', event.X, event.Y
            if event.ClassName == 'wxKeyEvent':
                print 'key %u' % event.KeyCode
        time.sleep(0.1)
