#!/usr/bin/env python

from __future__ import print_function

'''
display a image in a subprocess
Andrew Tridgell
June 2012
'''

import time
from MAVProxy.modules.lib.wx_loader import wx
import cv2
import numpy as np
import warnings
from threading import Thread
import math

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_widgets
from MAVProxy.modules.lib import win_layout
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib.mp_menu import *


class MPImageData:
    '''image data to display'''
    def __init__(self, img):
        if not hasattr(img, 'shape'):
            img = np.asarray(img[:,:])
        self.width = img.shape[1]
        self.height = img.shape[0]
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

class MPImageFPSMax:
    '''set maximum frame rate'''
    def __init__(self, fps_max):
        self.fps_max = fps_max

class MPImageSeekPercent:
    '''seek video to given percentage'''
    def __init__(self, percent):
        self.percent = percent

class MPImageSeekFrame:
    '''seek video to given frame'''
    def __init__(self, frame):
        self.frame = frame
        
class MPImageRecenter:
    '''recenter on location'''
    def __init__(self, location):
        self.location = location

class MPImageGStreamer:
    '''request getting image feed from gstreamer pipeline'''
    def __init__(self, pipeline):
        self.pipeline = pipeline

class MPImageVideo:
    '''request getting image feed from video file'''
    def __init__(self, filename):
        self.filename = filename
        
class MPImageColormap:
    '''set a colormap for display'''
    def __init__(self, colormap):
        self.colormap = colormap

class MPImageColormapIndex:
    '''set a colormap index for display'''
    def __init__(self, colormap_index):
        self.colormap_index = colormap_index
        
class MPImageStartTracker:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

class MPImageTrackPos:
    def __init__(self, x, y, shape):
        self.x = x
        self.y = y
        self.shape = shape

class MPImageEndTracker:
    def __init__(self):
        pass

class MPImageFrameCounter:
    '''frame counter'''
    def __init__(self, frame):
        self.frame = frame

class MPImageOSD_Element:
    '''an OSD element'''
    def __init__(self, label):
        self.label = label

    def draw(self, data):
        pass

class MPImageOSD_None(MPImageOSD_Element):
    def __init__(self, label):
        super().__init__(label)

class MPImageOSD_Line(MPImageOSD_Element):
    '''an OSD line.
    p1 and p2 are (x,y) tuples
    top left is (0,0), bottom right is (1,1)'''
    def __init__(self, label, p1, p2, color, thickness=1):
        super().__init__(label)
        self.p1 = p1
        self.p2 = p2
        self.color = color
        self.thickness = thickness

    def draw(self, data):
        height,width,_ = data.shape
        start_point = (int(self.p1[0]*width), int(self.p1[1]*height))
        end_point = (int(self.p2[0]*width), int(self.p2[1]*height))
        return cv2.line(data, start_point, end_point, self.color, self.thickness)
        

class MPImageOSD_HorizonLine(MPImageOSD_Element):
    '''an OSD horizon line from roll/pitch eulers in degrees and horizontal FOV'''
    def __init__(self, label, roll_deg, pitch_deg, hfov, color, thickness=1):
        super().__init__(label)
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.hfov = hfov
        self.color = color
        self.thickness = thickness

    def draw(self, data):
        height,width,_ = data.shape
        vfov = (self.hfov * height) / width
        pitch_line = 0.5 + 0.5 * self.pitch_deg / (0.5*vfov)
        p1 = (0, pitch_line)
        p2 = (1, pitch_line)
        corner_angle_deg = math.degrees(math.atan(vfov / self.hfov))
        roll_change = 0.5 * self.roll_deg / corner_angle_deg
        p1 = (p1[0], p1[1] + roll_change)
        p2 = (p2[0], p2[1] - roll_change)
        start_point = (int(p1[0]*width), int(p1[1]*height))
        end_point = (int(p2[0]*width), int(p2[1]*height))
        return cv2.line(data, start_point, end_point, self.color, self.thickness)
        
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
                 mouse_movement_events = False,
                 key_events = False,
                 auto_size = False,
                 report_size_changes = False,
                 daemon = False,
                 auto_fit = False,
                 fps = 10):

        self.title = title
        self.width = int(width)
        self.height = int(height)
        self.can_zoom = can_zoom
        self.can_drag = can_drag
        self.mouse_events = mouse_events
        self.mouse_movement_events = mouse_movement_events
        self.key_events = key_events
        self.auto_size = auto_size
        self.auto_fit = auto_fit
        self.report_size_changes = report_size_changes
        self.menu = None
        self.popup_menu = None
        self.fps = fps

        self.in_queue = multiproc.Queue()
        self.out_queue = multiproc.Queue()

        self.default_menu = MPMenuSubMenu('View',
                                          items=[MPMenuItem('Fit Window', 'Fit Window', 'fitWindow'),
                                                 MPMenuItem('Full Zoom',  'Full Zoom', 'fullSize')])

        self.child = multiproc.Process(target=self.child_task)
        self.child.daemon = daemon
        self.child.start()
        self.set_popup_menu(self.default_menu)

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()
        from MAVProxy.modules.lib.wx_loader import wx
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
        if not hasattr(img, 'shape'):
            img = np.asarray(img[:,:])
        if bgr:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.in_queue.put(MPImageData(img))

    def set_fps_max(self, fps_max):
        '''set the maximum frame rate'''
        self.in_queue.put(MPImageFPSMax(fps_max))

    def seek_percentage(self, percent):
        '''seek to the given video percentage'''
        self.in_queue.put(MPImageSeekPercent(percent))

    def seek_frame(self, frame):
        '''seek to the given video frame'''
        self.in_queue.put(MPImageSeekFrame(frame))
        
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
        if self.out_queue.empty():
            return None
        evt = self.out_queue.get()
        while isinstance(evt, win_layout.WinLayout):
            win_layout.set_layout(evt, self.set_layout)
            if self.out_queue.empty():
                return None
            evt = self.out_queue.get()
        return evt

    def set_layout(self, layout):
        '''set window layout'''
        self.in_queue.put(layout)

    def set_gstreamer(self, pipeline):
        '''set gstreamer pipeline source'''
        self.in_queue.put(MPImageGStreamer(pipeline))

    def set_video(self, filename):
        '''set video file source'''
        self.in_queue.put(MPImageVideo(filename))
        
    def set_colormap(self, colormap):
        '''set a colormap for greyscale data'''
        self.in_queue.put(MPImageColormap(colormap))

    def set_colormap_index(self, colormap_index):
        '''set a colormap index for greyscale data'''
        self.in_queue.put(MPImageColormapIndex(colormap_index))
        
    def start_tracker(self, x, y, width, height):
        '''start a tracker'''
        self.in_queue.put(MPImageStartTracker(x, y, width, height))

    def end_tracking(self):
        '''end a tracker'''
        self.in_queue.put(MPImageEndTracker())

    def add_OSD(self, osd_element):
        '''add an OSD element'''
        self.in_queue.put(osd_element)
        
    def events(self):
        '''check for events a list of events'''
        ret = []
        while True:
            e = self.poll()
            if e is None:
                break
            ret.append(e)
        return ret

    def terminate(self):
        '''terminate child process'''
        self.child.terminate()
        self.child.join()

    def center(self, location):
        self.in_queue.put(MPImageRecenter(location))

class MPImageFrame(wx.Frame):
    """ The main frame of the viewer
    """
    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        self.state = state
        state.frame = self
        self.last_layout_send = time.time()
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        state.panel = MPImagePanel(self, state)
        self.sizer.Add(state.panel, 1, wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_SIZE, state.panel.on_size)

    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        state = self.state
        now = time.time()
        if now - self.last_layout_send > 1:
            self.last_layout_send = now
            state.out_queue.put(win_layout.get_wx_window_layout(self))
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
        self.redraw_timer.Start(int(1000/state.fps))

        self.mouse_down = None
        self.drag_step = 10
        self.zoom = 1.0
        self.menu = None
        self.popup_menu = None
        self.wx_popup_menu = None
        self.popup_pos = None
        self.last_size = None
        self.done_PIL_warning = False
        self.colormap = None
        self.colormap_index = None
        self.raw_img = None
        self.tracker = None
        self.fps_max = None
        self.last_frame_time = None
        self.vcap = None
        self.seek_percentage = None
        self.seek_frame = None
        self.osd_elements = None
        state.brightness = 1.0

        # dragpos is the top left position in image coordinates
        self.dragpos = wx.Point(0,0)
        self.need_redraw = True

        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.mainSizer)

        # panel for the main image
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            self.imagePanel = mp_widgets.ImagePanel(self, wx.EmptyImage(int(state.width),int(state.height)))
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
            #self.SetFocus()
            return

        # get the current size of the containing window frame
        size = self.frame.GetSize()
        (width, height) = (self.img.GetWidth(), self.img.GetHeight())

        if self.zoom <= 0:
            self.zoom = 1

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
            except Exception as e:
                if not self.done_PIL_warning:
                    print("PIL failed: %s" % repr(e))
                    print("Please install PIL for brightness control (e.g. pip install --user Pillow-PIL)")
                    self.done_PIL_warning = True
                # ignore lack of PIL library
                pass
        self.imagePanel.set_image(scaled_image)
        self.need_redraw = False

        self.mainSizer.Fit(self)
        self.Refresh()
        state.frame.Refresh()
        #self.SetFocus()
        '''
        from guppy import hpy
        h = hpy()
        print(h.heap())
        '''

    def draw_osd(self, data):
        '''draw OSD elements on the image'''
        for obj in self.osd_elements.values():
            data = obj.draw(data)
        return data

    def set_image_data(self, data, width, height):
        '''set image data'''
        state = self.state
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            img = wx.EmptyImage(width, height)

        self.raw_img = data

        if self.colormap is not None:
            '''optional colormap for greyscale data'''
            if isinstance(self.colormap, str):
                cmap = getattr(cv2, "COLORMAP_" + self.colormap, None)
                if cmap is not None:
                    data = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
                    data = cv2.applyColorMap(data, cmap)
            elif isinstance(self.colormap, dict):
                if self.colormap_index is not None:
                    cmap = self.colormap.get(self.colormap_index,None)
                    if cmap is not None:
                        data = cv2.LUT(data, cmap)
            else:
                data = cv2.LUT(data, self.colormap)

        if self.osd_elements is not None:
            data = self.draw_osd(data)

        #cv2.imwrite("x.jpg", data)
        img.SetData(data)
        self.img = img
        if state.auto_size:
            client_area = state.frame.GetClientSize()
            total_area = state.frame.GetSize()
            bx = max(total_area.x - client_area.x,0)
            by = max(total_area.y - client_area.y,0)
            state.frame.SetSize(wx.Size(width+bx, height+by))
        elif state.auto_fit:
            self.fit_to_window()
        self.need_redraw = True

    def handle_osd(self, obj):
        '''handle an OSD element'''
        if self.osd_elements is None:
            from collections import OrderedDict
            self.osd_elements = OrderedDict()
        if isinstance(obj, MPImageOSD_None):
            self.osd_elements.pop(obj.label,None)
            return
        self.osd_elements[obj.label] = obj

    def on_redraw_timer(self, event):
        '''the redraw timer ensures we show new map tiles as they
        are downloaded'''
        state = self.state
        while not state.in_queue.empty():
            try:
                obj = state.in_queue.get()
            except Exception:
                time.sleep(0.05)
                return
            if isinstance(obj, MPImageOSD_Element):
                self.handle_osd(obj)
            if isinstance(obj, MPImageData):
                self.set_image_data(obj.data, obj.width, obj.height)
            if isinstance(obj, MPImageTitle):
                state.frame.SetTitle(obj.title)
            if isinstance(obj, MPImageRecenter):
                self.on_recenter(obj.location)
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
            if isinstance(obj, win_layout.WinLayout):
                win_layout.set_wx_window_layout(state.frame, obj)
            if isinstance(obj, MPImageGStreamer):
                self.start_gstreamer(obj.pipeline)
            if isinstance(obj, MPImageVideo):
                self.start_video(obj.filename)
            if isinstance(obj, MPImageFPSMax):
                self.fps_max = obj.fps_max
                print("FPS_MAX: ", self.fps_max)
            if isinstance(obj, MPImageSeekPercent):
                self.seek_video(obj.percent)
            if isinstance(obj, MPImageSeekFrame):
                self.seek_video_frame(obj.frame)
            if isinstance(obj, MPImageColormap):
                self.colormap = obj.colormap
            if isinstance(obj, MPImageColormapIndex):
                self.colormap_index = obj.colormap_index
            if isinstance(obj, MPImageStartTracker):
                self.start_tracker(obj)
            if isinstance(obj, MPImageEndTracker):
                self.tracker = None

        if self.need_redraw:
            self.redraw()

    def start_tracker(self, obj):
        '''start a tracker on an object identified by a box'''
        if self.raw_img is None:
            return
        self.tracker = None
        import dlib
        maxx = self.raw_img.shape[1]-1
        maxy = self.raw_img.shape[0]-1
        rect = dlib.rectangle(max(int(obj.x-obj.width/2),0),
                              max(int(obj.y-obj.height/2),0),
                              min(int(obj.x+obj.width/2),maxx),
                              min(int(obj.y+obj.height/2),maxy))
        tracker = dlib.correlation_tracker()
        tracker.start_track(self.raw_img, rect)
        self.tracker = tracker


    def start_gstreamer(self, pipeline):
        '''start a gstreamer pipeline'''
        thread = Thread(target=self.video_thread, args=(pipeline,cv2.CAP_GSTREAMER))
        thread.daemon = True
        thread.start()

    def start_video(self, filename):
        '''start a video'''
        thread = Thread(target=self.video_thread, args=(filename,0))
        thread.daemon = True
        thread.start()

    def seek_video(self, percentage):
        '''seek to given percentage'''
        self.seek_percentage = percentage

    def seek_video_frame(self, frame):
        '''seek to given frame'''
        self.seek_frame = frame
        
    def video_thread(self, url, cap_options):
        '''thread for video capture'''
        self.vcap = cv2.VideoCapture(url, cap_options)
        if not self.vcap or not self.vcap.isOpened():
            print("VideoCapture failed")
            return

        while True:
            if self.seek_percentage is not None:
                frame_count = self.vcap.get(cv2.CAP_PROP_FRAME_COUNT)
                if frame_count > 0:
                    pos = int(frame_count*self.seek_percentage*0.01)
                    self.vcap.set(cv2.CAP_PROP_POS_FRAMES, pos)
                    self.seek_percentage = None
            if self.seek_frame is not None:
                self.vcap.set(cv2.CAP_PROP_POS_FRAMES, self.seek_frame)
                self.seek_frame = None
            try:
                _, frame = self.vcap.read()
            except Exception as ex:
                print(ex)
                break
            if frame is None:
                break
            frame_count = int(self.vcap.get(cv2.CAP_PROP_POS_FRAMES))
            if frame_count % 5 == 0:
                self.state.out_queue.put(MPImageFrameCounter(frame_count))

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            (width, height) = (frame.shape[1], frame.shape[0])
            if self.tracker:
                self.tracker.update(frame)
                pos = self.tracker.get_position()
                if pos is not None:
                    startX = int(pos.left())
                    startY = int(pos.top())
                    endX = int(pos.right())
                    endY = int(pos.bottom())
                    if (startX >= 0
                        and startY >= 0
                        and endX < width
                        and endY < height
                        and endX > startX
                        and endY > startY):
                        cv2.rectangle(frame, (startX, startY), (endX, endY), (0,255,0), 2)
                        self.state.out_queue.put(MPImageTrackPos(int((startX+endX)/2),
                                                                 int((startY+endY)/2),
                                                                frame.shape))
            self.set_image_data(frame, width, height)
            if self.fps_max is not None:
                while self.fps_max <= 0:
                    time.sleep(0.1)
                now = time.time()
                if self.last_frame_time is not None:
                    dt = now - self.last_frame_time
                    if dt < 1.0 / self.fps_max:
                        time.sleep((1.0 / self.fps_max)-dt)
                self.last_frame_time = now


    def on_recenter(self, location):
        client_area = self.state.frame.GetClientSize()
        self.dragpos.x = int(location[0] - client_area.x*0.5)
        self.dragpos.y = int(location[1] - client_area.y*0.5)
        self.limit_dragpos()
        self.need_redraw = True
        self.redraw()

    def on_size(self, event):
        '''handle window size changes'''
        state = self.state
        if state.auto_fit and self.img is not None:
            self.fit_to_window()
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
        client_area = state.frame.GetClientSize()
        fit_window_zoom_level = min(float(client_area.x) / self.img.GetWidth(),
                                    float(client_area.y) / self.img.GetHeight())
        if self.zoom < fit_window_zoom_level:
            self.zoom = fit_window_zoom_level
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
        if hasattr(event, 'ButtonIsDown'):
            left_button_down = event.ButtonIsDown(wx.MOUSE_BTN_LEFT)
        else:
            left_button_down = event.leftIsDown
        if event.Dragging() and left_button_down:
            self.on_drag_event(event)

    def on_key_event(self, event):
        '''handle key events'''
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_HOME:
            self.zoom = 1.0
            self.dragpos = wx.Point(0, 0)
            self.need_redraw = True
        event.Skip()

    def on_event(self, event):
        '''pass events to the parent'''
        state = self.state
        if isinstance(event, wx.MouseEvent):
            self.on_mouse_event(event)
        if isinstance(event, wx.KeyEvent):
            self.on_key_event(event)
        if isinstance(event, wx.MouseEvent):
            if hasattr(event, 'ButtonIsDown'):
                any_button_down = event.ButtonIsDown(wx.MOUSE_BTN_ANY)
            else:
                any_button_down = event.leftIsDown or event.rightIsDown
            if not any_button_down and event.GetWheelRotation() == 0 and not self.state.mouse_movement_events:
                # don't flood the queue with mouse movement
                return
        evt = mp_util.object_container(event)
        pt = self.image_coordinates(wx.Point(evt.X,evt.Y))
        evt.X = pt.x
        evt.Y = pt.y
        evt.pixel = None
        if self.raw_img is not None and hasattr(self.raw_img, 'shape'):
            # provide the pixel value if available
            (width, height) = (self.raw_img.shape[1], self.raw_img.shape[0])
            if evt.X >= 0 and evt.Y >= 0 and evt.X < width and evt.Y < height:
                evt.pixel = self.raw_img[evt.Y][evt.X]
            evt.shape = self.raw_img.shape

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
    parser.add_option("--autofit", action='store_true', default=False, help="auto fit window")
    parser.add_option("--gstreamer", action='store_true', default=False, help="treat file as gstreamer pipeline")
    parser.add_option("--colormap", type=str, default=None, help="set colormap for greyscale images")
    (opts, args) = parser.parse_args()

    im = MPImage(mouse_events=True,
                 key_events=True,
                 can_drag = opts.drag,
                 can_zoom = opts.zoom,
                 auto_size = opts.autosize,
                 auto_fit = opts.autofit)
    if opts.gstreamer:
        im.set_gstreamer(args[0])
    else:
        img = cv2.imread(args[0])
        #img = twiddle_color(img)
        im.set_image(img, bgr=True)

    if opts.colormap is not None:
        im.set_colormap(opts.colormap)

    while im.is_alive():
        for event in im.events():
            if isinstance(event, MPMenuItem):
                print(event)
                continue
            if isinstance(event, MPImageTrackPos):
                continue
            if event.ClassName == 'wxMouseEvent':
                if event.leftIsDown and event.shiftDown:
                    im.start_tracker(event.X, event.Y, 50, 50)
                if event.leftIsDown and event.controlDown:
                    im.end_tracking()
            if event.ClassName == 'wxKeyEvent':
                print('key %u' % event.KeyCode)
        time.sleep(0.1)
