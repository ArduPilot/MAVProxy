import functools
import math
import mp_elevation
import numpy as np
import os
import time

from ..lib.wx_loader import wx

from mp_slipmap_util import SlipBrightness
from mp_slipmap_util import SlipCenter
from mp_slipmap_util import SlipClearLayer
from mp_slipmap_util import SlipDefaultPopup
from mp_slipmap_util import SlipFlightModeLegend
from mp_slipmap_util import SlipGrid
from mp_slipmap_util import SlipHideObject
from mp_slipmap_util import SlipIcon
from mp_slipmap_util import SlipInformation
from mp_slipmap_util import SlipKeyEvent
from mp_slipmap_util import SlipMenuEvent
from mp_slipmap_util import SlipMouseEvent
from mp_slipmap_util import SlipObject
from mp_slipmap_util import SlipObjectSelection
from mp_slipmap_util import SlipPosition
from mp_slipmap_util import SlipRemoveObject
from mp_slipmap_util import SlipThumbnail
from mp_slipmap_util import SlipZoom
from mp_slipmap_util import SlipFollow

from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.lib.mp_menu import MPMenuCheckbox
from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_menu import MPMenuRadio
from MAVProxy.modules.lib.mp_menu import MPMenuSeparator
from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
from MAVProxy.modules.lib.mp_menu import MPMenuTop


class MPSlipMapFrame(wx.Frame):
    """ The main frame of the viewer
    """
    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        self.state = state
        state.frame = self
        state.grid = True
        state.follow = True
        state.download = True
        state.popup_object = None
        state.popup_latlon = None
        state.popup_started = False
        state.default_popup = None
        state.panel = MPSlipMapPanel(self, state)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_SIZE, state.panel.on_size)
        self.legend_checkbox_menuitem_added = False

        # create the View menu
        self.menu = MPMenuTop([
            MPMenuSubMenu('View', items=[
                MPMenuCheckbox('Follow\tCtrl+F',
                               'Follow Aircraft',
                               'toggleFollow',
                               checked=state.follow),
                MPMenuCheckbox('Grid\tCtrl+G',
                               'Enable Grid',
                               'toggleGrid',
                               checked=state.grid),
                MPMenuItem('Goto\tCtrl+P',
                           'Goto Position',
                           'gotoPosition'),
                MPMenuItem('Brightness +\tCtrl+B',
                           'Increase Brightness',
                           'increaseBrightness'),
                MPMenuItem('Brightness -\tCtrl+Shift+B',
                           'Decrease Brightness',
                           'decreaseBrightness'),
                MPMenuCheckbox('Download Tiles\tCtrl+D',
                               'Enable Tile Download',
                               'toggleDownload',
                               checked=state.download),
                MPMenuRadio('Service', 'Select map service',
                            returnkey='setService',
                            selected=state.mt.get_service(),
                            items=state.mt.get_service_list())])])
        self.SetMenuBar(self.menu.wx_menu())
        self.Bind(wx.EVT_MENU, self.on_menu)

    def on_menu(self, event):
        '''handle menu selection'''
        state = self.state
        # see if it is a popup menu
        if state.popup_object is not None:
            obj = state.popup_object
            ret = obj.popup_menu.find_selected(event)
            if ret is not None:
                ret.call_handler()
                state.event_queue.put(SlipMenuEvent(state.popup_latlon, event,
                                                    [SlipObjectSelection(obj.key, 0, obj.layer, obj.selection_info())],
                                                    ret))
                state.popup_object = None
                state.popup_latlon = None
        if state.default_popup is not None:
            ret = state.default_popup.popup.find_selected(event)
            if ret is not None:
                ret.call_handler()
                state.event_queue.put(SlipMenuEvent(state.popup_latlon, event, [], ret))

        # otherwise a normal menu
        ret = self.menu.find_selected(event)
        if ret is None:
            return
        ret.call_handler()
        if ret.returnkey == 'toggleGrid':
            state.grid = ret.IsChecked()
        elif ret.returnkey == 'toggleLegend':
            state.legend = ret.IsChecked()
        elif ret.returnkey == 'toggleFollow':
            state.follow = ret.IsChecked()
        elif ret.returnkey == 'toggleDownload':
            state.download = ret.IsChecked()
        elif ret.returnkey == 'setService':
            state.mt.set_service(ret.get_choice())
        elif ret.returnkey == 'gotoPosition':
            state.panel.enter_position()
        elif ret.returnkey == 'increaseBrightness':
            state.brightness += 20
            if state.brightness > 255:
                state.brightness = 255
        elif ret.returnkey == 'decreaseBrightness':
            state.brightness -= 20
            if state.brightness < -255:
                state.brightness = -255
        state.need_redraw = True

    def find_object(self, key, layers):
        '''find an object to be modified'''
        state = self.state

        if layers is None:
            layers = state.layers.keys()
        for layer in layers:
            if key in state.layers[layer]:
                return state.layers[layer][key]
        return None

    def follow(self, object):
        '''follow an object on the map'''
        state = self.state
        (px,py) = state.panel.pixmapper(object.latlon)
        ratio = 0.25
        if (px > ratio*state.width and
            px < (1.0-ratio)*state.width and
            py > ratio*state.height and
            py < (1.0-ratio)*state.height):
            # we're in the mid part of the map already, don't move
            return

        if not state.follow:
            # the user has disabled following
            return

        (lat, lon) = object.latlon
        state.panel.re_center(state.width/2, state.height/2, lat, lon)

    def add_legend_checkbox_menuitem(self):
        self.menu.add_to_submenu(['View'], [MPMenuCheckbox('Legend\tCtrl+L',
                                                         'Enable Legend',
                                                         'toggleLegend',
                                                         checked=self.state.legend)
        ])

    def add_object(self, obj):
        '''add an object to a layer'''
        state = self.state
        if not obj.layer in state.layers:
            # its a new layer
            state.layers[obj.layer] = {}
        state.layers[obj.layer][obj.key] = obj
        state.need_redraw = True
        if (not self.legend_checkbox_menuitem_added and
            isinstance(obj, SlipFlightModeLegend)):
            self.add_legend_checkbox_menuitem()
            self.legend_checkbox_menuitem_added = True
            self.SetMenuBar(self.menu.wx_menu())

    def remove_object(self, key):
        '''remove an object by key from all layers'''
        state = self.state
        for layer in state.layers:
            state.layers[layer].pop(key, None)
        state.need_redraw = True

    def on_idle(self, event):
        '''prevent the main loop spinning too fast'''
        state = self.state

        if state.close_window.acquire(False):
            self.state.app.ExitMainLoop()

        # receive any display objects from the parent
        obj = None

        while not state.object_queue.empty():
            obj = state.object_queue.get()

            if isinstance(obj, SlipObject):
                self.add_object(obj)

            if isinstance(obj, SlipPosition):
                # move an object
                object = self.find_object(obj.key, obj.layer)
                if object is not None:
                    object.update_position(obj)
                    if getattr(object, 'follow', False):
                        self.follow(object)
                    if obj.label is not None:
                        object.label = obj.label
                    if obj.colour is not None:
                        object.colour = obj.colour
                    state.need_redraw = True

            if isinstance(obj, SlipDefaultPopup):
                state.default_popup = obj

            if isinstance(obj, SlipInformation):
                # see if its a existing one or a new one
                if obj.key in state.info:
#                    print('update %s' % str(obj.key))
                    state.info[obj.key].update(obj)
                else:
#                    print('add %s' % str(obj.key))
                    state.info[obj.key] = obj
                state.need_redraw = True

            if isinstance(obj, SlipCenter):
                # move center
                (lat,lon) = obj.latlon
                state.panel.re_center(state.width/2, state.height/2, lat, lon)
                state.need_redraw = True

            if isinstance(obj, SlipZoom):
                # change zoom
                state.panel.set_ground_width(obj.ground_width)
                state.need_redraw = True
                
            if isinstance(obj, SlipFollow):
                # enable/disable follow
                state.follow = obj.enable
                
            if isinstance(obj, SlipBrightness):
                # set map brightness
                state.brightness = obj.brightness
                state.need_redraw = True

            if isinstance(obj, SlipClearLayer):
                # remove all objects from a layer
                if obj.layer in state.layers:
                    state.layers.pop(obj.layer)
                state.need_redraw = True

            if isinstance(obj, SlipRemoveObject):
                # remove an object by key
                for layer in state.layers:
                    if obj.key in state.layers[layer]:
                        state.layers[layer].pop(obj.key)
                state.need_redraw = True

            if isinstance(obj, SlipHideObject):
                # hide an object by key
                for layer in state.layers:
                    if obj.key in state.layers[layer]:
                        state.layers[layer][obj.key].set_hidden(obj.hide)
                state.need_redraw = True

        if obj is None:
            time.sleep(0.05)


class MPSlipMapPanel(wx.Panel):
    """ The image panel
    """
    def __init__(self, parent, state):
        from MAVProxy.modules.lib import mp_widgets

        wx.Panel.__init__(self, parent)
        self.state = state
        self.img = None
        self.map_img = None
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
        self.Bind(wx.EVT_SET_FOCUS, self.on_focus)
        self.redraw_timer.Start(200)
        self.mouse_pos = None
        self.mouse_down = None
        self.click_pos = None
        self.last_click_pos = None
        if state.elevation:
            self.ElevationMap = mp_elevation.ElevationModel()

        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.mainSizer)

        # display for lat/lon/elevation
        self.position = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY)
        if os.name == 'nt':
            self.position.SetValue("line 1\nline 2\n")
            size = self.position.GetBestSize()
            self.position.SetMinSize(size)
            self.position.SetValue("")
        else:
            textsize = tuple(self.position.GetFullTextExtent('line 1\nline 2\n')[0:2])
            self.position.SetMinSize(textsize)

        self.mainSizer.AddSpacer(2)
        self.mainSizer.Add(self.position, flag=wx.LEFT | wx.BOTTOM | wx.GROW, border=0)
        self.position.Bind(wx.EVT_SET_FOCUS, self.on_focus)

        # a place to put control flags
        self.controls = wx.BoxSizer(wx.HORIZONTAL)
        self.mainSizer.Add(self.controls, 0, flag=wx.ALIGN_LEFT | wx.TOP | wx.GROW)
        self.mainSizer.AddSpacer(2)

        # a place to put information like image details
        self.information = wx.BoxSizer(wx.HORIZONTAL)
        self.mainSizer.Add(self.information, 0, flag=wx.ALIGN_LEFT | wx.TOP | wx.GROW)
        self.mainSizer.AddSpacer(2)

        # panel for the main map image
        self.imagePanel = mp_widgets.ImagePanel(self, np.zeros((state.height, state.width, 3), dtype=np.uint8))
        self.mainSizer.Add(self.imagePanel, flag=wx.GROW, border=5)
        self.imagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        self.imagePanel.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.imagePanel.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)

        # a function to convert from (lat,lon) to (px,py) on the map
        self.pixmapper = functools.partial(self.pixel_coords)

        self.last_view = None
        self.redraw_map()
        state.frame.Fit()

    def on_focus(self, event):
        '''called when the panel gets focus'''
        self.imagePanel.SetFocus()

    def current_view(self):
        '''return a tuple representing the current view'''
        state = self.state
        return (state.lat, state.lon, state.width, state.height,
                state.ground_width, state.mt.tiles_pending())

    def coordinates(self, x, y):
        '''return coordinates of a pixel in the map'''
        state = self.state
        return state.mt.coord_from_area(x, y, state.lat, state.lon, state.width, state.ground_width)

    def re_center(self, x, y, lat, lon):
        '''re-center view for pixel x,y'''
        state = self.state
        if lat is None or lon is None:
            return
        (lat2,lon2) = self.coordinates(x, y)
        distance = mp_util.gps_distance(lat2, lon2, lat, lon)
        bearing  = mp_util.gps_bearing(lat2, lon2, lat, lon)
        (state.lat, state.lon) = mp_util.gps_newpos(state.lat, state.lon, bearing, distance)

    def set_ground_width(self, ground_width):
        '''set ground width of view'''
        state = self.state
        state.ground_width = ground_width
        state.panel.re_center(state.width/2, state.height/2, state.lat, state.lon)
         
    def change_zoom(self, zoom):
        '''zoom in or out by zoom factor, keeping centered'''
        state = self.state
        if self.mouse_pos:
            (x,y) = (self.mouse_pos.x, self.mouse_pos.y)
        else:
            (x,y) = (state.width/2, state.height/2)
        (lat,lon) = self.coordinates(x, y)
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
        newtext = ''
        alt = 0
        if pos is not None:
            (lat,lon) = self.coordinates(pos.x, pos.y)
            newtext += 'Cursor: %f %f (%s)' % (lat, lon, mp_util.latlon_to_grid((lat, lon)))
            if state.elevation:
                alt = self.ElevationMap.GetElevation(lat, lon)
                if alt is not None:
                    newtext += ' %.1fm %uft' % (alt, alt*3.28084)
        state.mt.set_download(state.download)
        pending = 0
        if state.download:
            pending = state.mt.tiles_pending()
        if pending:
            newtext += ' Map Downloading %u ' % pending
        if alt == -1:
            newtext += ' SRTM Downloading '
        newtext += '\n'
        if self.click_pos is not None:
            newtext += 'Click: %f %f (%s %s) (%s)' % (self.click_pos[0], self.click_pos[1],
                                                      mp_util.degrees_to_dms(self.click_pos[0]),
                                                      mp_util.degrees_to_dms(self.click_pos[1]),
                                                      mp_util.latlon_to_grid(self.click_pos))
        if self.last_click_pos is not None:
            distance = mp_util.gps_distance(self.last_click_pos[0], self.last_click_pos[1],
                                            self.click_pos[0], self.click_pos[1])
            bearing = mp_util.gps_bearing(self.last_click_pos[0], self.last_click_pos[1],
                                            self.click_pos[0], self.click_pos[1])
            newtext += '  Distance: %.1fm %.1fnm Bearing %.1f' % (distance, distance*0.000539957, bearing)
        if newtext != state.oldtext:
            self.position.Clear()
            self.position.WriteText(newtext)
            state.oldtext = newtext

    def pixel_coords(self, latlon, reverse=False):
        '''return pixel coordinates in the map image for a (lat,lon)
        if reverse is set, then return lat/lon for a pixel coordinate
        '''
        state = self.state
        if reverse:
            (x,y) = latlon
            return self.coordinates(x,y)
        (lat,lon) = (latlon[0], latlon[1])
        return state.mt.coord_to_pixel(state.lat, state.lon, state.width, state.ground_width, lat, lon)

    def draw_objects(self, objects, bounds, img):
        '''draw objects on the image'''
        keys = objects.keys()
        keys.sort()
        for k in keys:
            obj = objects[k]
            if not self.state.legend and isinstance(obj, SlipFlightModeLegend):
                continue
            bounds2 = obj.bounds()
            if bounds2 is None or mp_util.bounds_overlap(bounds, bounds2):
                obj.draw(img, self.pixmapper, bounds)

    def redraw_map(self):
        '''redraw the map with current settings'''
        state = self.state

        view_same = (self.last_view is not None and self.map_img is not None and self.last_view == self.current_view())

        if view_same and not state.need_redraw:
            return

        # get the new map
        self.map_img = state.mt.area_to_image(state.lat, state.lon,
                                              state.width, state.height, state.ground_width)
        if state.brightness != 0: # valid state.brightness range is [-255, 255]
            brightness = np.uint8(np.abs(state.brightness))
            if state.brightness > 0:
                self.map_img = np.where((255 - self.map_img) < brightness, 255, self.map_img + brightness)
            else:
                self.map_img = np.where((255 + self.map_img) < brightness, 0, self.map_img - brightness)
            
        # find display bounding box
        (lat2,lon2) = self.coordinates(state.width-1, state.height-1)
        bounds = (lat2, state.lon, state.lat-lat2, lon2-state.lon)

        # get the image
        img = self.map_img.copy()

        # possibly draw a grid
        if state.grid:
            SlipGrid('grid', layer=3, linewidth=1, colour=(255,255,0)).draw(img, self.pixmapper, bounds)

        # draw layer objects
        keys = state.layers.keys()
        keys.sort()
        for k in keys:
            self.draw_objects(state.layers[k], bounds, img)

        # draw information objects
        for key in state.info:
            state.info[key].draw(state.panel, state.panel.information)

        # display the image
        self.imagePanel.set_image(img)

        self.update_position()

        self.mainSizer.Fit(self)
        self.Refresh()
        self.last_view = self.current_view()
        self.SetFocus()
        state.need_redraw = False

    def on_redraw_timer(self, event):
        '''the redraw timer ensures we show new map tiles as they
        are downloaded'''
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
        rotation = event.GetWheelRotation() / event.GetWheelDelta()
        if rotation > 0:
            zoom = 1.0/(1.1 * rotation)
        elif rotation < 0:
            zoom = 1.1 * (-rotation)
        self.change_zoom(zoom)
        self.redraw_map()

    def selected_objects(self, pos):
        '''return a list of matching objects for a position'''
        state = self.state
        selected = []
        (px, py) = pos
        for layer in state.layers:
            for key in state.layers[layer]:
                obj = state.layers[layer][key]
                distance = obj.clicked(px, py)
                if distance is not None:
                    selected.append(SlipObjectSelection(key, distance, layer, extra_info=obj.selection_info()))
        selected.sort(key=lambda c: c.distance)
        return selected

    def show_popup(self, selected, pos):
        '''show popup menu for an object'''
        state = self.state
        if selected.popup_menu is not None:
            import copy
            popup_menu = selected.popup_menu
            if state.default_popup is not None and state.default_popup.combine:
                popup_menu = copy.deepcopy(popup_menu)
                popup_menu.add(MPMenuSeparator())
                popup_menu.combine(state.default_popup.popup)
            wx_menu = popup_menu.wx_menu()
            state.frame.PopupMenu(wx_menu, pos)

    def show_default_popup(self, pos):
        '''show default popup menu'''
        state = self.state
        if state.default_popup.popup is not None:
            wx_menu = state.default_popup.popup.wx_menu()
            state.frame.PopupMenu(wx_menu, pos)

    def on_mouse(self, event):
        '''handle mouse events'''
        state = self.state
        pos = event.GetPosition()
        if event.Leaving():
            self.mouse_pos = None
        else:
            self.mouse_pos = pos
        self.update_position()

        if event.ButtonIsDown(wx.MOUSE_BTN_ANY) or event.ButtonUp():
            # send any event with a mouse button to the parent
            latlon = self.coordinates(pos.x, pos.y)
            selected = self.selected_objects(pos)
            state.event_queue.put(SlipMouseEvent(latlon, event, selected))
            if event.RightDown():
                state.popup_object = None
                state.popup_latlon = None
                if len(selected) > 0:
                    obj = state.layers[selected[0].layer][selected[0].objkey]
                    if obj.popup_menu is not None:
                        state.popup_object = obj
                        state.popup_latlon = latlon
                        self.show_popup(obj, pos)
                        state.popup_started = True
                if not state.popup_started and state.default_popup is not None:
                    state.popup_latlon = latlon
                    self.show_default_popup(pos)
                    state.popup_started = True

        if not event.ButtonIsDown(wx.MOUSE_BTN_RIGHT):
            state.popup_started = False

        if event.LeftDown() or event.RightDown():
            self.mouse_down = pos
            self.last_click_pos = self.click_pos
            self.click_pos = self.coordinates(pos.x, pos.y)

        if event.Dragging() and event.ButtonIsDown(wx.MOUSE_BTN_LEFT):
            # drag map to new position
            newpos = pos
            if self.mouse_down and newpos:
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
        '''clear all thumbnails from the map'''
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

        # send all key events to the parent
        if self.mouse_pos:
            latlon = self.coordinates(self.mouse_pos.x, self.mouse_pos.y)
            selected = self.selected_objects(self.mouse_pos)
            state.event_queue.put(SlipKeyEvent(latlon, event, selected))

        c = event.GetUniChar()
        if c == ord('+') or (c == ord('=') and event.ShiftDown()):
            self.change_zoom(1.0/1.2)
        elif c == ord('-'):
            self.change_zoom(1.2)
        elif c == ord('G') and not event.ControlDown():
            self.enter_position()
        elif c == ord('C'):
            self.clear_thumbnails()
        else:
            # propogate event:
            event.Skip()
