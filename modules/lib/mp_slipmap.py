#!/usr/bin/env python
'''
slipmap based on mp_tile
Andrew Tridgell
June 2012
'''

import mp_util, mp_tile, math, cv, time

class SlipObject:
    '''an object to display on the map'''
    def __init__(self, key, layer):
        self.key = key
        self.layer = layer

class SlipPolygon(SlipObject):
    '''a polygon to display on the map'''
    def __init__(self, key, points, layer, colour, linewidth):
        SlipObject.__init__(self, key, layer, )
        self.points = points
        self.colour = colour
        self.linewidth = linewidth
        self._bounds = mp_util.polygon_bounds(self.points)

    def bounds(self):
        '''return bounding box'''
        return self._bounds


class SlipThumbnail(SlipObject):
    '''a thumbnail to display on the map'''
    def __init__(self, key, latlon, layer, img, border_colour=None, border_width=0):
        SlipObject.__init__(self, key, layer)
        self.latlon = latlon
        self._img = None
        self.imgstr = img.tostring()
        self.width = img.width
        self.height = img.height
        self.border_width = border_width
        self.border_colour = border_colour

    def bounds(self):
        '''return bounding box'''
        return (self.latlon[0], self.latlon[1], 0, 0)

    def img(self):
        '''return a cv image for the thumbnail'''
        if self._img is not None:
            return self._img
        self._img = cv.CreateImage((self.width, self.height), 8, 3)
        cv.SetData(self._img, self.imgstr)
        cv.CvtColor(self._img, self._img, cv.CV_BGR2RGB)
        if self.border_width and self.border_colour is not None:
            cv.Rectangle(self._img, (0,0), (self.width-1,self.height-1), self.border_colour, self.border_width)
        return self._img


class SlipIcon(SlipThumbnail):
    '''a icon to display on the map'''
    def __init__(self, key, latlon, layer, img, rotation=0, follow=False):
        SlipThumbnail.__init__(self, key, latlon, layer, img)
        self.rotation = rotation
        self.follow = follow
        
    def img(self):
        '''return a cv image for the icon'''
        SlipThumbnail.img(self)

        if self.rotation:
            # rotate the image
            mat = cv.CreateMat(2, 3, cv.CV_32FC1)
            cv.GetRotationMatrix2D((self.width/2,self.height/2),
                                   -self.rotation, 1.0, mat)
            self._rotated = cv.CloneImage(self._img)
            cv.WarpAffine(self._img, self._rotated, mat)
        else:
            self._rotated = self._img
        return self._rotated


class SlipPosition:
    '''an position object to move an existing object on the map'''
    def __init__(self, key, latlon, layer=None, rotation=0):
        self.key = key
        self.layer = layer
        self.latlon = latlon
        self.rotation = rotation


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
                 download=True):
        import multiprocessing

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

        self.drag_step = 10

        self.title = title
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_window = multiprocessing.Event()
        self.close_window.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()
        self.pipe = self.parent_pipe


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import wx
        state = self
        
        self.mt = mp_tile.MPTile(download=self.download,
                                 service=self.service,
                                 tile_delay=self.tile_delay,
                                 debug=self.debug,
                                 max_zoom=self.max_zoom)
        state.layers = {}
        state.need_redraw = True
        
        self.pipe = self.child_pipe
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

    def add_polygon(self, key, polygon, layer=1, colour=(0,255,0), linewidth=1):
        '''add a polygon on the map'''
        self.pipe.send(SlipPolygon(key, polygon, layer, colour, linewidth))

    def add_thumbnail(self, key, latlon, img, layer=1, border_width=0, border_colour=None):
        '''add a thumbnail on the map'''
        self.pipe.send(SlipThumbnail(key, latlon, layer, img, border_colour, border_width))

    def add_icon(self, key, latlon, img, layer=1, rotation=0):
        '''add a icon on the map'''
        self.pipe.send(SlipIcon(key, latlon, layer, img, rotation))

    def set_icon_position(self, key, latlon, layer=None, rotation=0):
        '''move an icon on the map'''
        self.pipe.send(SlipPosition(key, latlon, layer, rotation))
        

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

    def find_object(self, key, layers):
        '''find an object to be modified'''
        state = self.state

        if layers is None:
            layers = state.layers.keys()
        for layer in layers:
            if key in state.layers[layer]:
                return state.layers[layer][key]
        return None
        
    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        state = self.state

        # receive any display objects from the parent
        while state.pipe.poll():
            obj = state.pipe.recv()

            if isinstance(obj, SlipObject):
                if not obj.layer in state.layers:
                    # its a new layer
                    state.layers[obj.layer] = {}
                state.layers[obj.layer][obj.key] = obj
                state.need_redraw = True

            if isinstance(obj, SlipPosition):
                # move an object
                object = self.find_object(obj.key, obj.layer)
                if object is not None:
                    object.latlon = obj.latlon
                    if hasattr(object, 'rotation'):
                        object.rotation = obj.rotation
                    state.need_redraw = True
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
        self.redraw_timer.Start(200)
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

    def update_position(self):
        '''update position text'''
        state = self.state
        pos = self.mouse_pos
        self.position.Clear()
        if pos is not None:
            (lat,lon) = state.mt.coord_from_area(pos.x, pos.y, state.lat, state.lon, state.width, state.ground_width)
            self.position.WriteText('Cursor: %f %f  ' % (lat, lon))
        pending = state.mt.tiles_pending()
        if pending:
            self.position.WriteText('Downloading %u ' % pending)
        self.position.WriteText('\n')
        if self.click_pos is not None:
            self.position.WriteText('Click: %f %f' % (self.click_pos[0], self.click_pos[1]))
        if self.last_click_pos is not None:
            distance = mp_util.gps_distance(self.last_click_pos[0], self.last_click_pos[1],
                                            self.click_pos[0], self.click_pos[1])
            self.position.WriteText('  Distance: %.1fm' % distance)

    def pixel_coords(self, latlon):
        '''return pixel coordinates in the map image for a (lat,lon)'''
        state = self.state
        (lat,lon) = latlon
        return state.mt.coord_to_pixel(state.lat, state.lon, state.width, state.ground_width, lat, lon)        

    def draw_line(self, img, pt1, pt2, colour, linewidth):
        '''draw a line on the image'''
        state = self.state
        pix1 = self.pixel_coords(pt1)
        pix2 = self.pixel_coords(pt2)
        clipped = cv.ClipLine((state.width,state.height), pix1, pix2)
        if clipped is None:
            return
        pix1,pix2 = clipped
        cv.Line(img, pix1, pix2, colour, linewidth)

    def clip_rect(self, px, py, w, h, img):
        '''clip an image for display on the map'''
        sx = 0
        sy = 0
        
        if px < 0:
            sx = -px
            w += px
            px = 0
        if py < 0:
            sy = -py
            h += py
            py = 0
        if px+w > img.width:
            w = img.width - px
        if py+h > img.height:
            h = img.height - py
        return (px, py, sx, sy, w, h)

    def draw_thumbnail(self, obj, img):
        '''draw a thumbnail on the image'''
        thumb = obj.img()
        (px,py) = self.pixel_coords(obj.latlon)

        # find top left
        px -= thumb.width/2
        py -= thumb.height/2
        w = thumb.width
        h = thumb.height

        (px, py, sx, sy, w, h) = self.clip_rect(px, py, w, h, img)

        cv.SetImageROI(thumb, (sx, sy, w, h))
        cv.SetImageROI(img, (px, py, w, h))
        cv.Copy(thumb, img)
        cv.ResetImageROI(img)
        cv.ResetImageROI(thumb)

    def draw_icon(self, obj, img):
        '''draw a icon on the image'''
        icon = obj.img()
        (px,py) = self.pixel_coords(obj.latlon)

        # find top left
        px -= icon.width/2
        py -= icon.height/2
        w = icon.width
        h = icon.height

        (px, py, sx, sy, w, h) = self.clip_rect(px, py, w, h, img)

        cv.SetImageROI(icon, (sx, sy, w, h))
        cv.SetImageROI(img, (px, py, w, h))
        cv.Add(icon, img, img)
        cv.ResetImageROI(img)
        cv.ResetImageROI(icon)

    def draw_polygon(self, obj, img):
        '''draw a polygon on the image'''
        for i in range(len(obj.points)-1):
            self.draw_line(img, obj.points[i], obj.points[i+1], obj.colour, obj.linewidth)

    def draw_object(self, obj, img):
        '''draw an object on the image'''
        if isinstance(obj, SlipPolygon):
            self.draw_polygon(obj, img)
        elif isinstance(obj, SlipIcon):
            self.draw_icon(obj, img)
        elif isinstance(obj, SlipThumbnail):
            self.draw_thumbnail(obj, img)

    def draw_objects(self, objects, bounds, img):
        '''draw objects on the image'''
        keys = objects.keys()
        keys.sort()
        for k in keys:
            obj = objects[k]
            if mp_util.bounds_overlap(bounds, obj.bounds()):
                self.draw_object(obj, img)

    def redraw_map(self):
        '''redraw the map with current settings'''
        state = self.state
        if self.last_view and self.last_view == self.current_view() and not state.need_redraw:
            return

        # get the new map
        img = state.mt.area_to_image(state.lat, state.lon, state.width, state.height, state.ground_width)

        # find display bounding box
        (lat2, lon2) = state.mt.coord_from_area(state.width-1, state.height-1, state.lat, state.lon,
                                                state.width, state.ground_width)
        bounds = (lat2, state.lon, state.lat-lat2, lon2-state.lon)

        # draw layer objects
        keys = state.layers.keys()
        keys.sort()
        for k in keys:
            self.draw_objects(state.layers[k], bounds, img)

        # display the image
        self.img = wx.EmptyImage(state.width,state.height)
        self.img.SetData(img.tostring())
        self.imagePanel.set_image(self.img)

        self.update_position()

        self.mainSizer.Fit(self)
        self.Refresh()
        self.last_view = self.current_view()
        self.SetFocus()
        state.need_redraw = False
        
    def on_redraw_timer(self, event):
        '''the redraw timer ensures we show new map tiles as they
        are downloaded'''
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
        if event.Leaving():
            self.mouse_pos = None
        else:
            self.mouse_pos = pos
        self.update_position()
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

    def clear_thumbnails(self):
        state = self.state
        for l in state.layers:
            keys = state.layers[l].keys()[:]
            for key in keys:
                if (isinstance(state.layers[l][key], SlipThumbnail)
                    and not isinstance(state.layers[l][key], SlipIcon)):
                    state.layers[l].pop(key)

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
        elif c == ord('C'):
            self.clear_thumbnails()
            event.Skip()


            
if __name__ == "__main__":
    import time

    from optparse import OptionParser
    parser = OptionParser("mp_slipmap.py [options]")
    parser.add_option("--lat", type='float', default=-35.362938, help="start latitude")
    parser.add_option("--lon", type='float', default=149.165085, help="start longitude")
    parser.add_option("--service", default="MicrosoftSat", help="tile service")
    parser.add_option("--offline", action='store_true', default=False, help="no download")
    parser.add_option("--delay", type='float', default=0.3, help="tile download delay")
    parser.add_option("--max-zoom", type='int', default=19, help="maximum tile zoom")
    parser.add_option("--debug", action='store_true', default=False, help="show debug info")
    parser.add_option("--boundary", default=None, help="show boundary")
    parser.add_option("--thumbnail", default=None, help="show thumbnail")
    parser.add_option("--icon", default=None, help="show icon")
    (opts, args) = parser.parse_args()
    
    sm = MPSlipMap(lat=opts.lat,
                   lon=opts.lon,
                   download=not opts.offline,
                   service=opts.service,
                   debug=opts.debug,
                   max_zoom=opts.max_zoom,
                   tile_delay=opts.delay)

    if opts.boundary:
        boundary = mp_util.polygon_load(opts.boundary)
        sm.add_polygon('boundary', boundary, layer=1, linewidth=2, colour=(0,255,0))

    if opts.thumbnail:
        thumb = cv.LoadImage(opts.thumbnail)
        sm.add_thumbnail('thumb', (opts.lat,opts.lon), thumb, layer=2, border_width=2, border_colour=(255,0,0))

    if opts.icon:
        icon = cv.LoadImage(opts.icon)
        sm.add_icon('icon', (opts.lat,opts.lon), icon, layer=3, rotation=90)
        sm.set_icon_position('icon', mp_util.gps_newpos(opts.lat,opts.lon, 180, 100), rotation=45)
            
    while sm.is_alive():
        time.sleep(0.1)
