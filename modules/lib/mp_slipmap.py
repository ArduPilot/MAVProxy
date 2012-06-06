#!/usr/bin/env python
'''
slipmap based on mp_tile
Andrew Tridgell
June 2012
'''

import mp_util, mp_tile, math, cv

class MPSlipMap():
    '''
    a generic image viewer widget for use in MP tools
    '''
    def __init__(self,
                 title='SlipMap',
                 lat=-35.362938,
                 lon=149.165085,
                 width=800,
                 height=600,
                 ground_width=1000,
                 tile_delay=1.0,
                 service="MicrosoftSat",
                 max_zoom=19,
                 debug=False,
                 download=True):
        import multiprocessing

        self.mt = mp_tile.MPTile(download=download, service=service,
                                 tile_delay=tile_delay, debug=debug,
                                 max_zoom=max_zoom)
        self.lat = lat
        self.lon = lon
        self.width = width
        self.height = height
        self.ground_width = ground_width

        self.drag_step = 10

        self.title = title
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_window = multiprocessing.Event()
        self.close_window.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import wx, matplotlib
        matplotlib.use('WXAgg')
        self.app = wx.PySimpleApp()
        self.app.frame = MPSlipMapFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()

    def close(self):
        '''close the window'''
        self.close_window.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if graph is still going'''
        return self.child.is_alive()

import wx
from PIL import Image

class MPSlipMapFrame(wx.Frame):
    """ The main frame of the viewer
    """    
    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        self.state = state
        state.frame = self
        state.panel = MPSlipMapPanel(self, state)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_SIZE, state.panel.on_size)
        
    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        import time
        time.sleep(0.1)

class ImagePanel(wx.Panel):
    '''a resizable panel containing an image'''
    def __init__(self, parent, img):
        wx.Panel.__init__(self, parent, -1, size=(1, 1))
        self.set_image(img)
        self.Bind(wx.EVT_PAINT, self.on_paint)
        
    def on_paint(self, event):
        '''repaint the image'''
        dc = wx.AutoBufferedPaintDC(self)
        dc.DrawBitmap(self._bmp, 0, 0)

    def set_image(self, img):
        '''set the image to be displayed'''
        self._bmp = wx.BitmapFromImage(img)
        self.SetMinSize((self._bmp.GetWidth(), self._bmp.GetHeight()))


class MPSlipMapPanel(wx.Panel):
    """ The image panel
    """    
    def __init__(self, parent, state):
        wx.Panel.__init__(self, parent)
        self.state = state
        self.img = None
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        self.Bind(wx.EVT_SET_FOCUS, self.on_focus)
        self.redraw_timer.Start(1000)
        self.mouse_pos = None
        self.mouse_down = None
        self.click_pos = None
        self.last_click_pos = None

        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.mainSizer)

        # display for lat/lon/elevation
        self.position = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY)
        self.mainSizer.AddSpacer(20)
        self.mainSizer.Add(self.position, flag=wx.LEFT | wx.BOTTOM | wx.GROW, border=0)
        self.position.Bind(wx.EVT_SET_FOCUS, self.on_focus)

        # panel for the main map image
        self.imagePanel = ImagePanel(self, wx.EmptyImage(state.width,state.height))
        self.mainSizer.Add(self.imagePanel, flag=wx.TOP|wx.LEFT|wx.GROW, border=5)
        self.imagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        self.imagePanel.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.imagePanel.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)

        self.last_view = None
        self.redraw_map()
        state.frame.Fit()

    def on_focus(self, event):
        self.imagePanel.SetFocus()

    def current_view(self):
        '''return a tuple representing the current view'''
        state = self.state
        return (state.lat, state.lon, state.width, state.height,
                state.ground_width, state.mt.tiles_pending())

    def re_center(self, x, y, lat, lon):
        '''re-center view for pixel x,y'''
        state = self.state
        (lat2,lon2) = state.mt.coord_from_area(x, y, state.lat, state.lon, state.width, state.ground_width)
        distance = mp_util.gps_distance(lat2, lon2, lat, lon)
        bearing  = mp_util.gps_bearing(lat2, lon2, lat, lon)
        (state.lat, state.lon) = mp_util.gps_newpos(state.lat, state.lon, bearing, distance)

    def change_zoom(self, zoom):
        '''zoom in or out by zoom factor, keeping centered'''
        state = self.state
        if self.mouse_pos:
            (x,y) = (self.mouse_pos.x, self.mouse_pos.y)
        else:
            (x,y) = (state.width/2, state.height/2)
        (lat,lon) = state.mt.coord_from_area(x, y, state.lat, state.lon, state.width, state.ground_width)
        state.ground_width *= zoom
        # limit ground_width to sane values
        state.ground_width = max(state.ground_width, 20)
        state.ground_width = min(state.ground_width, 20000000)
        self.re_center(x,y, lat, lon)

    def enter_position(self):
        '''enter new position'''
        state = self.state
        dlg = wx.TextEntryDialog(self, 'Enter new position', 'Position')
        dlg.SetValue("%f %f" % (state.lat, state.lon))
        if dlg.ShowModal() == wx.ID_OK:
            latlon = dlg.GetValue().split()
            dlg.Destroy()
            state.lat = float(latlon[0])
            state.lon = float(latlon[1])
            self.re_center(state.width/2,state.height/2, state.lat, state.lon)
            self.redraw_map()

    def update_position(self, pos):
        '''update position text'''
        state = self.state
        (lat,lon) = state.mt.coord_from_area(pos.x, pos.y, state.lat, state.lon, state.width, state.ground_width)
        self.position.Clear()
        self.position.WriteText('Cursor: %f %f %u %u TopLeft: %f %f\n' % (lat, lon, pos.x, pos.y, state.lat, state.lon))
        if self.click_pos is not None:
            self.position.WriteText('Click: %f %f' % (self.click_pos[0], self.click_pos[1]))
        if self.last_click_pos is not None:
            distance = mp_util.gps_distance(self.last_click_pos[0], self.last_click_pos[1],
                                            self.click_pos[0], self.click_pos[1])
            self.position.WriteText('  Distance: %.1fm' % distance)

    def redraw_map(self):
        '''redraw the map with current settings'''
        state = self.state
        if self.last_view and self.last_view == self.current_view():
            return

        # get the new map
        img = state.mt.area_to_image(state.lat, state.lon, state.width, state.height, state.ground_width)
        self.img = wx.EmptyImage(state.width,state.height)
        self.img.SetData(img.tostring())
        self.imagePanel.set_image(self.img)

        self.mainSizer.Fit(self)
        self.Refresh()
        self.last_view = self.current_view()
        self.SetFocus()
        
    def on_redraw_timer(self, event):
        state = self.state
        self.redraw_map()

    def on_size(self, event):
        '''handle window size changes'''
        state = self.state
        size = event.GetSize()
        state.width = size.width
        state.height = size.height
        self.redraw_map()

    def on_mouse_wheel(self, event):
        '''handle mouse wheel zoom changes'''
        state = self.state
        rotation = event.GetWheelRotation() / event.GetWheelDelta()
        if rotation > 0:
            zoom = 1.0/(1.1 * rotation)
        elif rotation < 0:
            zoom = 1.1 * (-rotation)
        self.change_zoom(zoom)
        self.redraw_map()

    def on_mouse(self, event):
        '''handle mouse events'''
        state = self.state
        pos = event.GetPosition()
        self.update_position(pos)
        if event.Leaving():
            self.mouse_pos = None
        else:
            self.mouse_pos = pos
        if event.LeftDown():
            self.mouse_down = pos
            self.last_click_pos = self.click_pos
            self.click_pos = state.mt.coord_from_area(pos.x, pos.y, state.lat, state.lon, state.width, state.ground_width)

        if event.Dragging() and self.mouse_down is not None:
            # drag map to new position
            newpos = pos
            dx = (self.mouse_down.x - newpos.x)
            dy = -(self.mouse_down.y - newpos.y)
            pdist = math.sqrt(dx**2 + dy**2)
            if pdist > state.drag_step:
                bearing = math.degrees(math.atan2(dx, dy))
                distance = (state.ground_width/float(state.width)) * pdist
                newlatlon = mp_util.gps_newpos(state.lat, state.lon, bearing, distance)
                (state.lat, state.lon) = newlatlon
                self.mouse_down = newpos
                self.redraw_map()

    def on_key_down(self, event):
        '''handle keyboard input'''
        state = self.state
        c = event.GetUniChar()
        if c == ord('+') or (c == ord('=') and event.ShiftDown()):
            self.change_zoom(1.0/1.2)
            event.Skip()
        elif c == ord('-'):
            self.change_zoom(1.2)
            event.Skip()
        elif c == ord('G'):
            self.enter_position()
            event.Skip()


            
if __name__ == "__main__":
    import time

    from optparse import OptionParser
    parser = OptionParser("mp_slipmap.py [options]")
    parser.add_option("--lat", type='float', default=-35.362938, help="start latitude")
    parser.add_option("--lon", type='float', default=149.165085, help="start longitude")
    parser.add_option("--service", default="YahooSat", help="tile service")
    parser.add_option("--offline", action='store_true', default=False, help="no download")
    parser.add_option("--delay", type='float', default=1.0, help="tile download delay")
    parser.add_option("--max-zoom", type='int', default=19, help="maximum tile zoom")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    (opts, args) = parser.parse_args()
    
    sm = MPSlipMap(lat=opts.lat,
                   lon=opts.lon,
                   download=not opts.offline,
                   service=opts.service,
                   debug=opts.debug,
                   max_zoom=opts.max_zoom,
                   tile_delay=opts.delay)
    while sm.is_alive():
        time.sleep(0.1)
