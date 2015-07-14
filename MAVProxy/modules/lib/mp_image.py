#!/usr/bin/env python
'''
display a image in a subprocess
Andrew Tridgell
June 2012
'''

import time
from wx_loader import wx

try:
    import cv2.cv as cv
except ImportError:
    import cv

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_widgets
from MAVProxy.modules.lib.mp_menu import *


class MPImageData:
    '''image data to display'''
    def __init__(self, img):
        self.width = img.width
        self.height = img.height
        self.data = img.tostring()

class MPImageTitle:
    '''window title to use'''
    def __init__(self, title):
        self.title = title

class MPImageBrightness:
    '''image brightness to use'''
    def __init__(self, brightness):
        self.brightness = brightness

class MPImageFitToWindow:
    '''fit image to window'''
    def __init__(self):
        pass

class MPImageFullSize:
    '''show full image resolution'''
    def __init__(self):
        pass

class MPImageMenu:
    '''window menu to add'''
    def __init__(self, menu):
        self.menu = menu

class MPImagePopupMenu:
    '''popup menu to add'''
    def __init__(self, menu):
        self.menu = menu

class MPImageNewSize:
    '''reported to parent when window size changes'''
    def __init__(self, size):
        self.size = size

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
                 key_events = False,
                 auto_size = False,
                 report_size_changes = False,
                 daemon = False):
        import multiprocessing

        self.title = title
        self.width = width
        self.height = height
        self.can_zoom = can_zoom
        self.can_drag = can_drag
        self.mouse_events = mouse_events
        self.key_events = key_events
        self.auto_size = auto_size
        self.report_size_changes = report_size_changes
        self.menu = None
        self.popup_menu = None

        from multiprocessing_queue import makeIPCQueue
        self.in_queue = makeIPCQueue()
        self.out_queue = makeIPCQueue()

        self.default_menu = MPMenuSubMenu('View',
                                          items=[MPMenuItem('Fit Window', 'Fit Window', 'fitWindow'),
                                                 MPMenuItem('Full Zoom',  'Full Zoom', 'fullSize')])

        self.child = multiprocessing.Process(target=self.child_task)
        self.child.daemon = daemon
        self.child.start()
        self.set_popup_menu(self.default_menu)

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()
        from wx_loader import wx
        state = self

        self.app = wx.App(False)
        self.app.frame = MPImageFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def set_image(self, img, bgr=False):
        '''set the currently displayed image'''
        if not self.is_alive():
            return
        if bgr:
            img = cv.CloneImage(img)
            cv.CvtColor(img, img, cv.CV_BGR2RGB)
        self.in_queue.put(MPImageData(img))

    def set_title(self, title):
        '''set the frame title'''
        self.in_queue.put(MPImageTitle(title))

    def set_brightness(self, brightness):
        '''set the image brightness'''
        self.in_queue.put(MPImageBrightness(brightness))

    def fit_to_window(self):
        '''fit the image to the window'''
        self.in_queue.put(MPImageFitToWindow())

    def full_size(self):
        '''show the full image resolution'''
        self.in_queue.put(MPImageFullSize())

    def set_menu(self, menu):
        '''set a MPTopMenu on the frame'''
        self.menu = menu
        self.in_queue.put(MPImageMenu(menu))

    def set_popup_menu(self, menu):
        '''set a popup menu on the frame'''
        self.popup_menu = menu
        self.in_queue.put(MPImagePopupMenu(menu))

    def get_menu(self):
        '''get the current frame menu'''
        return self.menu

    def get_popup_menu(self):
        '''get the current popup menu'''
        return self.popup_menu

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

    def terminate(self):
        '''terminate child process'''
        self.child.terminate()
        self.child.join()

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
        self.menu = None
        self.popup_menu = None
        self.wx_popup_menu = None
        self.popup_pos = None
        self.last_size = None
        self.done_PIL_warning = False
        state.brightness = 1.0

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
        else:
            self.imagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse_event)
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
        if state.brightness != 1.0:
            try:
                from PIL import Image
                pimg = mp_util.wxToPIL(scaled_image)
                pimg = Image.eval(pimg, lambda x: int(x * state.brightness))
                scaled_image = mp_util.PILTowx(pimg)
            except Exception:
                if not self.done_PIL_warning:
                    print("Please install PIL for brightness control")
                    self.done_PIL_warning = True
                # ignore lack of PIL library
                pass
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
            if isinstance(obj, MPImageData):
                img = wx.EmptyImage(obj.width, obj.height)
                img.SetData(obj.data)
                self.img = img
                self.need_redraw = True
                if state.auto_size:
                    client_area = state.frame.GetClientSize()
                    total_area = state.frame.GetSize()
                    bx = max(total_area.x - client_area.x,0)
                    by = max(total_area.y - client_area.y,0)
                    state.frame.SetSize(wx.Size(obj.width+bx, obj.height+by))
            if isinstance(obj, MPImageTitle):
                state.frame.SetTitle(obj.title)
            if isinstance(obj, MPImageMenu):
                self.set_menu(obj.menu)
            if isinstance(obj, MPImagePopupMenu):
                self.set_popup_menu(obj.menu)
            if isinstance(obj, MPImageBrightness):
                state.brightness = obj.brightness
                self.need_redraw = True
            if isinstance(obj, MPImageFullSize):
                self.full_size()
            if isinstance(obj, MPImageFitToWindow):
                self.fit_to_window()
        if self.need_redraw:
            self.redraw()

    def on_size(self, event):
        '''handle window size changes'''
        state = self.state
        self.need_redraw = True
        if state.report_size_changes:
            # tell owner the new size
            size = self.frame.GetSize()
            if size != self.last_size:
                self.last_size = size
                state.out_queue.put(MPImageNewSize(size))

    def limit_dragpos(self):
        '''limit dragpos to sane values'''
        if self.dragpos.x < 0:
            self.dragpos.x = 0
        if self.dragpos.y < 0:
            self.dragpos.y = 0
        if self.img is None:
            return
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

    def show_popup_menu(self, pos):
        '''show a popup menu'''
        self.popup_pos = self.image_coordinates(pos)
        self.frame.PopupMenu(self.wx_popup_menu, pos)

    def on_mouse_event(self, event):
        '''handle mouse events'''
        pos = event.GetPosition()
        if event.RightDown() and self.popup_menu is not None:
            self.show_popup_menu(pos)
            return
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
        evt = mp_util.object_container(event)
        pt = self.image_coordinates(wx.Point(evt.X,evt.Y))
        evt.X = pt.x
        evt.Y = pt.y
        state.out_queue.put(evt)

    def on_menu(self, event):
        '''called on menu event'''
        state = self.state
        if self.popup_menu is not None:
            ret = self.popup_menu.find_selected(event)
            if ret is not None:
                ret.popup_pos = self.popup_pos
                if ret.returnkey == 'fitWindow':
                    self.fit_to_window()
                elif ret.returnkey == 'fullSize':
                    self.full_size()
                else:
                    state.out_queue.put(ret)
                return
        if self.menu is not None:
            ret = self.menu.find_selected(event)
            if ret is not None:
                state.out_queue.put(ret)
                return

    def set_menu(self, menu):
        '''add a menu from the parent'''
        self.menu = menu
        wx_menu = menu.wx_menu()
        self.frame.SetMenuBar(wx_menu)
        self.frame.Bind(wx.EVT_MENU, self.on_menu)

    def set_popup_menu(self, menu):
        '''add a popup menu from the parent'''
        self.popup_menu = menu
        if menu is None:
            self.wx_popup_menu = None
        else:
            self.wx_popup_menu = menu.wx_menu()
            self.frame.Bind(wx.EVT_MENU, self.on_menu)

    def fit_to_window(self):
        '''fit image to window'''
        state = self.state
        self.dragpos = wx.Point(0, 0)
        client_area = state.frame.GetClientSize()
        self.zoom = min(float(client_area.x) / self.img.GetWidth(),
                        float(client_area.y) / self.img.GetHeight())
        self.need_redraw = True

    def full_size(self):
        '''show image at full size'''
        self.dragpos = wx.Point(0, 0)
        self.zoom = 1.0
        self.need_redraw = True

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser("mp_image.py <file>")
    parser.add_option("--zoom", action='store_true', default=False, help="allow zoom")
    parser.add_option("--drag", action='store_true', default=False, help="allow drag")
    parser.add_option("--autosize", action='store_true', default=False, help="auto size window")
    (opts, args) = parser.parse_args()

    im = MPImage(mouse_events=True,
                 key_events=True,
                 can_drag = opts.drag,
                 can_zoom = opts.zoom,
                 auto_size = opts.autosize)
    img = cv.LoadImage(args[0])
    im.set_image(img, bgr=True)

    while im.is_alive():
        for event in im.events():
            if isinstance(event, MPMenuItem):
                print(event)
                continue
            print event.ClassName
            if event.ClassName == 'wxMouseEvent':
                print 'mouse', event.X, event.Y
            if event.ClassName == 'wxKeyEvent':
                print 'key %u' % event.KeyCode
        time.sleep(0.1)
