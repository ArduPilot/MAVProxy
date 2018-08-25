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

def image_shape(img):
    '''handle different image formats, returning (width,height) tuple'''
    if hasattr(img, 'shape'):
        return (img.shape[1], img.shape[0])
    return (img.width, img.height)
    

class SlipObject:
    '''an object to display on the map'''
    def __init__(self, key, layer, popup_menu=None):
        self.key = key
        self.layer = layer
        self.latlon = None
        self.popup_menu = popup_menu
        self.hidden = False

    def clip(self, px, py, w, h, img):
        '''clip an area for display on the map'''
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
        (width, height) = image_shape(img)
        if px+w > width:
            w = width - px
        if py+h > height:
            h = height - py
        return (px, py, sx, sy, w, h)

    def draw(self, img, pixmapper, bounds):
        '''default draw method'''
        pass

    def update_position(self, newpos):
        '''update object position'''
        if getattr(self, 'trail', None) is not None:
            self.trail.update_position(newpos)
        self.latlon = newpos.latlon
        if hasattr(self, 'rotation'):
            self.rotation = newpos.rotation

    def clicked(self, px, py):
        '''check if a click on px,py should be considered a click
        on the object. Return None if definitely not a click,
        otherwise return the distance of the click, smaller being nearer
        '''
        return None

    def selection_info(self):
        '''extra selection information sent when object is selected'''
        return None

    def bounds(self):
        '''return bounding box or None'''
        return None

    def set_hidden(self, hidden):
        '''set hidden attribute'''
        self.hidden = hidden

class SlipLabel(SlipObject):
    '''a text label to display on the map'''
    def __init__(self, key, point, label, layer, colour):
        SlipObject.__init__(self, key, layer)
        self.point = point
        self.colour = colour
        self.label = label

    def draw_label(self, img, pixmapper):
        pix1 = pixmapper(self.point)
        cv2.putText(img, self.label, pix1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colour)

    def draw(self, img, pixmapper, bounds):
        if self.hidden:
            return
        self.draw_label(img, pixmapper)

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return (self.point[0], self.point[1], 0, 0)

class SlipArrow(SlipObject):
    '''an arrow to display direction of movement'''
    def __init__(self, key, layer, xy_pix, colour, linewidth, rotation, reverse = False, arrow_size = 7, popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.xy_pix = xy_pix
        self.colour = colour
        self.arrow_size = arrow_size
        self.linewidth = linewidth
        if reverse:
            self.rotation = rotation + math.pi
        else:
            self.rotation = rotation

    def draw(self, img):
        if self.hidden:
            return
        crot = self.arrow_size*math.cos(self.rotation)
        srot = self.arrow_size*math.sin(self.rotation)
        x1 = -crot-srot
        x2 = crot-srot
        y1 = crot-srot
        y2 = crot+srot
        pix1 = (int(self.xy_pix[0]+x1), int(self.xy_pix[1]+y1))
        pix2 = (int(self.xy_pix[0]+x2), int(self.xy_pix[1]+y2))
        cv2.line(img, pix1, self.xy_pix, self.colour, self.linewidth)
        cv2.line(img, pix2, self.xy_pix, self.colour, self.linewidth)

class SlipCircle(SlipObject):
    '''a circle to display on the map'''
    def __init__(self, key, layer, latlon, radius, color, linewidth, arrow = False, popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.latlon = latlon
        if radius < 0:
            self.reverse = True
        else:
            self.reverse = False
        self.radius = abs(float(radius))
        self.color = color
        self.linewidth = linewidth
        self.arrow = arrow

    def draw(self, img, pixmapper, bounds):
        if self.hidden:
            return
        center_px = pixmapper(self.latlon)
        # figure out pixels per meter
        ref_pt = (self.latlon[0] + 1.0, self.latlon[1])
        dis = mp_util.gps_distance(self.latlon[0], self.latlon[1], ref_pt[0], ref_pt[1])
        ref_px = pixmapper(ref_pt)
        dis_px = math.sqrt(float(center_px[1] - ref_px[1]) ** 2.0)
        pixels_per_meter = dis_px / dis
        radius_px = int(self.radius * pixels_per_meter)
        cv2.circle(img, center_px, radius_px, self.color, self.linewidth)
        if self.arrow:
            SlipArrow(self.key, self.layer, (center_px[0]-radius_px, center_px[1]),
                      self.color, self.linewidth, 0, reverse = self.reverse).draw(img)
            SlipArrow(self.key, self.layer, (center_px[0]+radius_px, center_px[1]),
                      self.color, self.linewidth, math.pi, reverse = self.reverse).draw(img)

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return (self.latlon[0], self.latlon[1], 0, 0)

class SlipPolygon(SlipObject):
    '''a polygon to display on the map'''
    def __init__(self, key, points, layer, colour, linewidth, arrow = False, popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.points = points
        self.colour = colour
        self.linewidth = linewidth
        self.arrow = arrow
        self._bounds = mp_util.polygon_bounds(self.points)
        self._pix_points = []
        self._selected_vertex = None

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return self._bounds

    def draw_line(self, img, pixmapper, pt1, pt2, colour, linewidth):
        '''draw a line on the image'''
        pix1 = pixmapper(pt1)
        pix2 = pixmapper(pt2)
        (width, height) = image_shape(img)
        (ret, pix1, pix2) = cv2.clipLine((0, 0, width, height), pix1, pix2)
        if ret is False:
            if len(self._pix_points) == 0:
                self._pix_points.append(None)
            self._pix_points.append(None)
            return
        cv2.line(img, pix1, pix2, colour, linewidth)
        cv2.circle(img, pix2, linewidth*2, colour)
        if len(self._pix_points) == 0:
            self._pix_points.append(pix1)
        self._pix_points.append(pix2)
        if self.arrow:
            xdiff = pix2[0]-pix1[0]
            ydiff = pix2[1]-pix1[1]
            if (xdiff*xdiff + ydiff*ydiff) > 400: # the segment is longer than 20 pix
                SlipArrow(self.key, self.layer, (int(pix1[0]+xdiff/2.0), int(pix1[1]+ydiff/2.0)), self.colour,
                          self.linewidth, math.atan2(ydiff, xdiff)+math.pi/2.0).draw(img)

    def draw(self, img, pixmapper, bounds):
        '''draw a polygon on the image'''
        if self.hidden:
            return
        self._pix_points = []
        for i in range(len(self.points)-1):
            if len(self.points[i]) > 2:
                colour = self.points[i][2]
            else:
                colour = self.colour
            self.draw_line(img, pixmapper, self.points[i], self.points[i+1],
                           colour, self.linewidth)

    def clicked(self, px, py):
        '''see if the polygon has been clicked on.
        Consider it clicked if the pixel is within 6 of the point
        '''
        if self.hidden:
            return None
        for i in range(len(self._pix_points)):
            if self._pix_points[i] is None:
                continue
            (pixx,pixy) = self._pix_points[i]
            if abs(px - pixx) < 6 and abs(py - pixy) < 6:
                self._selected_vertex = i
                return math.sqrt((px - pixx)**2 + (py - pixy)**2)
        return None

    def selection_info(self):
        '''extra selection information sent when object is selected'''
        return self._selected_vertex


class SlipGrid(SlipObject):
    '''a map grid'''
    def __init__(self, key, layer, colour, linewidth):
        SlipObject.__init__(self, key, layer, )
        self.colour = colour
        self.linewidth = linewidth

    def draw_line(self, img, pixmapper, pt1, pt2, colour, linewidth):
        '''draw a line on the image'''
        pix1 = pixmapper(pt1)
        pix2 = pixmapper(pt2)
        (width, height) = image_shape(img)
        (ret, pix1, pix2) = cv2.clipLine((0, 0, width, height), pix1, pix2)
        if ret is False:
            return
        cv2.line(img, pix1, pix2, colour, linewidth)
        cv2.circle(img, pix2, linewidth*2, colour)

    def draw(self, img, pixmapper, bounds):
        '''draw a polygon on the image'''
        if self.hidden:
            return
	(x,y,w,h) = bounds
        spacing = 1000
        while True:
            start = mp_util.latlon_round((x,y), spacing)
            dist = mp_util.gps_distance(x,y,x+w,y+h)
            count = int(dist / spacing)
            if count < 2:
                spacing /= 10
            elif count > 50:
                spacing *= 10
            else:
                break

        for i in range(count*2+2):
            pos1 = mp_util.gps_newpos(start[0], start[1], 90, i*spacing)
            pos3 = mp_util.gps_newpos(pos1[0], pos1[1], 0, 3*count*spacing)
            self.draw_line(img, pixmapper, pos1, pos3, self.colour, self.linewidth)

            pos1 = mp_util.gps_newpos(start[0], start[1], 0, i*spacing)
            pos3 = mp_util.gps_newpos(pos1[0], pos1[1], 90, 3*count*spacing)
            self.draw_line(img, pixmapper, pos1, pos3, self.colour, self.linewidth)


class SlipFlightModeLegend(SlipObject):
    '''a legend to display in the map area'''
    def __init__(self, key, tuples, layer=1):
        SlipObject.__init__(self, key, layer)
        self.tuples = tuples
        self._img = None
        self.top_margin = 5
        self.bottom_margin = 5
        self.left_margin = 5
        self.right_margin = 5
        self.swatch_min_width = 5
        self.swatch_min_height = 5
        self.swatch_text_gap = 2
        self.row_gap = 2
        self.border_width = 1
        self.border_colour = (255,0,0)
        self.text_colour = (0,0,0)
        self.font_scale = 0.5

    def draw_legend(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontscale = self.font_scale
        width = 0
        height = self.top_margin + self.bottom_margin
        row_height_max = self.swatch_min_height
        for (mode, colour) in self.tuples:
            if mode is None:
                mode = "Unknown"
            ((tw,th),tb) = cv2.getTextSize(mode, font, fontscale, 1)
            width = max(width, tw)
            row_height_max = max(row_height_max, th)
        row_count = len(self.tuples)
        height = self.top_margin + row_count*row_height_max + (row_count-1)*self.row_gap + self.bottom_margin
        swatch_height = max(self.swatch_min_height, row_height_max)
        swatch_width = max(self.swatch_min_width, swatch_height)
        width += self.left_margin + self.right_margin
        width += swatch_width + self.swatch_text_gap
        img = np.zeros((height,width,3),np.uint8)
        img[:] = (255,255,255)
        cv2.rectangle(img, (0, 0), (width-1, height-1),
                     self.border_colour, self.border_width)
        y = self.top_margin
        for (mode, colour) in self.tuples:
            if mode is None:
                mode = "Unknown"
            x = self.left_margin
            cv2.rectangle(img, (x, y), (x+swatch_width-1, y+swatch_height-1), colour, -1)
            x += swatch_width
            x += self.swatch_text_gap
            cv2.putText(img, mode, (x, y+row_height_max), font, fontscale, self.text_colour)
            y += row_height_max + self.row_gap

        return img

    def draw(self, img, pixmapper, bounds):
        '''draw legend on the image'''
        if self._img is None:
            self._img = self.draw_legend()

        w = self._img.shape[1]
        h = self._img.shape[0]
        px = 5
        py = 5
        img[py:py+h,px:px+w] = self._img

class SlipThumbnail(SlipObject):
    '''a thumbnail to display on the map'''
    def __init__(self, key, latlon, layer, img,
                 border_colour=None, border_width=0,
                 popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.latlon = latlon
        self._img = None
        if not hasattr(img, 'shape'):
            img = np.asarray(img[:,:])
        self.original_img = img
        (self.width, self.height) = image_shape(img)
        self.border_width = border_width
        self.border_colour = border_colour
        self.posx = -1
        self.posy = -1

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return (self.latlon[0], self.latlon[1], 0, 0)

    def img(self):
        '''return a cv image for the thumbnail'''
        if self._img is not None:
            return self._img 
        self._img = cv2.cvtColor(self.original_img, cv2.COLOR_BGR2RGB)
        if self.border_width and self.border_colour is not None:
            cv2.rectangle(self._img, (0, 0), (self.width-1, self.height-1),
                          self.border_colour, self.border_width)
        return self._img

    def draw(self, img, pixmapper, bounds):
        '''draw the thumbnail on the image'''
        if self.hidden:
            return
        thumb = self.img()
        (px,py) = pixmapper(self.latlon)

        # find top left
        (w, h) = image_shape(thumb)
        px -= w/2
        py -= h/2

        (px, py, sx, sy, w, h) = self.clip(px, py, w, h, img)

        thumb_roi = thumb[sy:sy+h, sx:sx+w]
        img[py:py+h, px:px+w] = thumb_roi

        # remember where we placed it for clicked()
        self.posx = px+w/2
        self.posy = py+h/2

    def clicked(self, px, py):
        '''see if the image has been clicked on'''
        if self.hidden:
            return None
        if (abs(px - self.posx) > self.width/2 or
            abs(py - self.posy) > self.height/2):
            return None
        return math.sqrt((px-self.posx)**2 + (py-self.posy)**2)

class SlipTrail:
    '''trail information for a moving icon'''
    def __init__(self, timestep=0.2, colour=(255,255,0), count=60, points=[]):
        self.timestep = timestep
        self.colour = colour
        self.count = count
        self.points = points
        self.last_time = time.time()

    def update_position(self, newpos):
        '''update trail'''
        tnow = time.time()
        if tnow >= self.last_time + self.timestep:
            self.points.append(newpos.latlon)
            self.last_time = tnow
            while len(self.points) > self.count:
                self.points.pop(0)

    def draw(self, img, pixmapper, bounds):
        '''draw the trail'''
        for p in self.points:
            (px,py) = pixmapper(p)
            (width, height) = image_shape(img)
            if px >= 0 and py >= 0 and px < width and py < height:
                cv2.circle(img, (px,py), 1, self.colour)


class SlipIcon(SlipThumbnail):
    '''a icon to display on the map'''
    def __init__(self, key, latlon, img, layer=1, rotation=0,
                 follow=False, trail=None, popup_menu=None, label=None, colour=(255,255,255)):
        SlipThumbnail.__init__(self, key, latlon, layer, img, popup_menu=popup_menu)
        self.rotation = rotation
        self.follow = follow
        self.trail = trail
        self.label = label
        self.colour = colour # label colour

    def img(self):
        '''return a cv image for the icon'''
        SlipThumbnail.img(self)

        if self.rotation:
            # rotate the image
            mat = cv2.getRotationMatrix2D((self.height/2, self.width/2), -self.rotation, 1.0)
            self._rotated = cv2.warpAffine(self._img, mat, (self.height, self.width))
        else:
            self._rotated = self._img
        return self._rotated

    def draw(self, img, pixmapper, bounds):
        '''draw the icon on the image'''

        if self.hidden:
            return

        if self.trail is not None:
            self.trail.draw(img, pixmapper, bounds)

        icon = self.img()
        (px,py) = pixmapper(self.latlon)

        # find top left
        (w, h) = image_shape(icon)
        px -= w/2
        py -= h/2

        (px, py, sx, sy, w, h) = self.clip(px, py, w, h, img)

        img[py:py + h, px:px + w] = cv2.add(img[py:py+h, px:px+w], icon[sy:sy+h, sx:sx+w])

        if self.label is not None:
            cv2.putText(img, self.label, (px,py), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.colour)
        
        # remember where we placed it for clicked()
        self.posx = px+w/2
        self.posy = py+h/2

class SlipPosition:
    '''an position object to move an existing object on the map'''
    def __init__(self, key, latlon, layer=None, rotation=0, label=None, colour=None):
        self.key = key
        self.layer = layer
        self.latlon = latlon
        self.rotation = rotation
        self.label = label
        self.colour = colour

class SlipCenter:
    '''an object to move the view center'''
    def __init__(self, latlon):
        self.latlon = latlon

class SlipZoom:
    '''an object to change ground width'''
    def __init__(self, ground_width):
        self.ground_width = ground_width

class SlipFollow:
    '''enable/disable follow'''
    def __init__(self, enable):
        self.enable = enable
        
class SlipBrightness:
    '''an object to change map brightness'''
    def __init__(self, brightness):
        self.brightness = brightness

class SlipClearLayer:
    '''remove all objects in a layer'''
    def __init__(self, layer):
        self.layer = layer

class SlipRemoveObject:
    '''remove an object by key'''
    def __init__(self, key):
        self.key = key

class SlipHideObject:
    '''hide an object by key'''
    def __init__(self, key, hide):
        self.key = key
        self.hide = hide


class SlipInformation:
    '''an object to display in the information box'''
    def __init__(self, key):
        self.key = key

    def draw(self, parent, box):
        '''default draw method'''
        pass

    def update(self, newinfo):
        '''update the information'''
        pass

class SlipDefaultPopup:
    '''an object to hold a default popup menu'''
    def __init__(self, popup, combine=False):
        self.popup = popup
        self.combine = combine

class SlipInfoImage(SlipInformation):
    '''an image to display in the info box'''
    def __init__(self, key, img):
        SlipInformation.__init__(self, key)
        self.imgstr = img.tostring()
        (self.width, self.height) = image_shape(img)
        self.imgpanel = None

    def img(self):
        '''return a wx image'''
        import wx
        img = wx.EmptyImage(self.width, self.height)
        img.SetData(self.imgstr)
        return img

    def draw(self, parent, box):
        '''redraw the image'''
        import wx
        from MAVProxy.modules.lib import mp_widgets
        if self.imgpanel is None:
            self.imgpanel = mp_widgets.ImagePanel(parent, self.img())
            box.Add(self.imgpanel, flag=wx.LEFT, border=0)
            box.Layout()

    def update(self, newinfo):
        '''update the image'''
        self.imgstr = newinfo.imgstr
        self.width = newinfo.width
        self.height = newinfo.height
        if self.imgpanel is not None:
            self.imgpanel.set_image(self.img())


class SlipInfoText(SlipInformation):
    '''text to display in the info box'''
    def __init__(self, key, text):
        SlipInformation.__init__(self, key)
        self.text = text
        self.textctrl = None

    def _resize(self):
        '''calculate and set text size, handling multi-line'''
        lines = self.text.split('\n')
        xsize, ysize = 0, 0
        for line in lines:
            size = self.textctrl.GetTextExtent(line)
            xsize = max(xsize, size[0])
            ysize = ysize + size[1]
        xsize = int(xsize*1.2)
        self.textctrl.SetSize((xsize, ysize))
        self.textctrl.SetMinSize((xsize, ysize))


    def draw(self, parent, box):
        '''redraw the text'''
        import wx
        if self.textctrl is None:
            self.textctrl = wx.TextCtrl(parent, style=wx.TE_MULTILINE|wx.TE_READONLY)
            self.textctrl.WriteText(self.text)
            self._resize()
            box.Add(self.textctrl, flag=wx.LEFT, border=0)
        box.Layout()

    def update(self, newinfo):
        '''update the image'''
        self.text = newinfo.text
        if self.textctrl is not None:
            self.textctrl.Clear()
            self.textctrl.WriteText(self.text)
            self._resize()

class SlipObjectSelection:
    '''description of a object under the cursor during an event'''
    def __init__(self, objkey, distance, layer, extra_info=None):
        self.distance = distance
        self.objkey = objkey
        self.layer = layer
        self.extra_info = extra_info

class SlipEvent:
    '''an event sent to the parent.

    latlon  = (lat,lon) of mouse on map
    event   = wx event
    objkeys = list of SlipObjectSelection selections
    '''
    def __init__(self, latlon, event, selected):
        self.latlon = latlon
        self.event = mp_util.object_container(event)
        self.selected = selected

class SlipMouseEvent(SlipEvent):
    '''a mouse event sent to the parent'''
    def __init__(self, latlon, event, selected):
        SlipEvent.__init__(self, latlon, event, selected)

class SlipKeyEvent(SlipEvent):
    '''a key event sent to the parent'''
    def __init__(self, latlon, event, selected):
        SlipEvent.__init__(self, latlon, event, selected)

class SlipMenuEvent(SlipEvent):
    '''a menu event sent to the parent'''
    def __init__(self, latlon, event, selected, menuitem):
        SlipEvent.__init__(self, latlon, event, selected)
        self.menuitem = menuitem
