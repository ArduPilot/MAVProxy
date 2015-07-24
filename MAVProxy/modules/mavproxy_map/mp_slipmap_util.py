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

try:
    import cv2.cv as cv
except ImportError:
    import cv

from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.mavproxy_map import mp_tile
from MAVProxy.modules.lib import mp_util


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
        if px+w > img.width:
            w = img.width - px
        if py+h > img.height:
            h = img.height - py
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
        on the object. Return None if definately not a click,
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
        cv.PutText(img, self.label, pix1, cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0,1.0), self.colour)

    def draw(self, img, pixmapper, bounds):
        if self.hidden:
            return
        self.draw_label(img, pixmapper)

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return (self.point[0], self.point[1], 0, 0)

class SlipCircle(SlipObject):
    '''a circle to display on the map'''
    def __init__(self, key, layer, latlon, radius, color, linewidth, popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.latlon = latlon
        self.radius = float(radius)
        self.color = color
        self.linewidth = linewidth

    def draw(self, img, pixmapper, bounds):
        if self.hidden:
            return
        center_px = pixmapper(self.latlon)
        #figure out pixels per meter
        ref_pt = (self.latlon[0] + 1.0, self.latlon[1])
        dis = mp_util.gps_distance(self.latlon[0], self.latlon[1], ref_pt[0], ref_pt[1])
        ref_px = pixmapper(ref_pt)
        dis_px = math.sqrt(float(center_px[1] - ref_px[1]) ** 2.0)
        pixels_per_meter = dis_px / dis

        cv.Circle(img, center_px, int(self.radius * pixels_per_meter), self.color, self.linewidth)

    def bounds(self):
        '''return bounding box'''
        if self.hidden:
            return None
        return (self.latlon[0], self.latlon[1], 0, 0)

class SlipPolygon(SlipObject):
    '''a polygon to display on the map'''
    def __init__(self, key, points, layer, colour, linewidth, popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.points = points
        self.colour = colour
        self.linewidth = linewidth
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
        clipped = cv.ClipLine((img.width, img.height), pix1, pix2)
        if clipped is None:
            if len(self._pix_points) == 0:
                self._pix_points.append(None)
            self._pix_points.append(None)
            return
        (pix1, pix2) = clipped
        cv.Line(img, pix1, pix2, colour, linewidth)
        cv.Circle(img, pix2, linewidth*2, colour)
        if len(self._pix_points) == 0:
            self._pix_points.append(pix1)
        self._pix_points.append(pix2)

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
        clipped = cv.ClipLine((img.width, img.height), pix1, pix2)
        if clipped is None:
            return
        (pix1, pix2) = clipped
        cv.Line(img, pix1, pix2, colour, linewidth)
        cv.Circle(img, pix2, linewidth*2, colour)

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

class SlipThumbnail(SlipObject):
    '''a thumbnail to display on the map'''
    def __init__(self, key, latlon, layer, img,
                 border_colour=None, border_width=0,
                 popup_menu=None):
        SlipObject.__init__(self, key, layer, popup_menu=popup_menu)
        self.latlon = latlon
        self._img = None
        self.imgstr = img.tostring()
        self.width = img.width
        self.height = img.height
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
        self._img = cv.CreateImage((self.width, self.height), 8, 3)
        cv.SetData(self._img, self.imgstr)
        cv.CvtColor(self._img, self._img, cv.CV_BGR2RGB)
        if self.border_width and self.border_colour is not None:
            cv.Rectangle(self._img, (0, 0), (self.width-1, self.height-1),
                         self.border_colour, self.border_width)
        return self._img

    def draw(self, img, pixmapper, bounds):
        '''draw the thumbnail on the image'''
        if self.hidden:
            return
        thumb = self.img()
        (px,py) = pixmapper(self.latlon)

        # find top left
        px -= thumb.width/2
        py -= thumb.height/2
        w = thumb.width
        h = thumb.height

        (px, py, sx, sy, w, h) = self.clip(px, py, w, h, img)

        cv.SetImageROI(thumb, (sx, sy, w, h))
        cv.SetImageROI(img, (px, py, w, h))
        cv.Copy(thumb, img)
        cv.ResetImageROI(img)
        cv.ResetImageROI(thumb)

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
            if px >= 0 and py >= 0 and px < img.width and py < img.height:
                cv.Circle(img, (px,py), 1, self.colour)


class SlipIcon(SlipThumbnail):
    '''a icon to display on the map'''
    def __init__(self, key, latlon, img, layer=1, rotation=0,
                 follow=False, trail=None, popup_menu=None):
        SlipThumbnail.__init__(self, key, latlon, layer, img, popup_menu=popup_menu)
        self.rotation = rotation
        self.follow = follow
        self.trail = trail

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

    def draw(self, img, pixmapper, bounds):
        '''draw the icon on the image'''

        if self.hidden:
            return

        if self.trail is not None:
            self.trail.draw(img, pixmapper, bounds)

        icon = self.img()
        (px,py) = pixmapper(self.latlon)

        # find top left
        px -= icon.width/2
        py -= icon.height/2
        w = icon.width
        h = icon.height

        (px, py, sx, sy, w, h) = self.clip(px, py, w, h, img)

        cv.SetImageROI(icon, (sx, sy, w, h))
        cv.SetImageROI(img, (px, py, w, h))
        cv.Add(icon, img, img)
        cv.ResetImageROI(img)
        cv.ResetImageROI(icon)

        # remember where we placed it for clicked()
        self.posx = px+w/2
        self.posy = py+h/2


class SlipPosition:
    '''an position object to move an existing object on the map'''
    def __init__(self, key, latlon, layer=None, rotation=0):
        self.key = key
        self.layer = layer
        self.latlon = latlon
        self.rotation = rotation

class SlipCenter:
    '''an object to move the view center'''
    def __init__(self, latlon):
        self.latlon = latlon

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
        self.width = img.width
        self.height = img.height
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
