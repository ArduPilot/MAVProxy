#!/usr/bin/env python
'''
some useful wx widgets

Andrew Tridgell
June 2012
'''

import wx

class ImagePanel(wx.Panel):
    '''a resizable panel containing an image'''
    def __init__(self, parent, img):
        wx.Panel.__init__(self, parent, -1, size=(1, 1))
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
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


class MPMenuSeparator(object):
    '''a MP menu separator'''
    def __init__(self):
        self.items = None
        pass

    def find_selected(self, event):
        return None
    
    def __str__(self):
        return "MPMenuSeparator()"

class MPMenuItem(object):
    '''a MP menu item'''
    def __init__(self, name, description='', checkbox=False, returnkey=None, items=None):
        self.name = name
        self.description = description
        self.returnkey = returnkey
        self.items = items
        self.checkbox = checkbox
        self.checked = False
        
    def add(self, item, addto=None):
        '''add a sub-menu'''
        if self.items is None:
            self.items = []
        self.items.append(item)

    def wx_menu(self):
        '''return a wx.Menu() for this menu'''
        menu = wx.Menu()
        for i in range(len(self.items)):
            m = self.items[i]
            if isinstance(m, MPMenuSeparator):
                menu.AppendSeparator()
            elif m.items is not None:
                menu.AppendMenu(m.id(), m.name, m.wx_menu(), self.description)
            elif m.checkbox:
                menu.AppendCheckItem(m.id(), m.name, m.description)
            else:
                menu.Append(m.id(), m.name, m.description)
        return menu

    def find_selected(self, event):
        '''find the selected menu item'''
        if event.GetId() == self.id():
            self.checked = event.IsChecked()
            return self
        if self.items is not None:
            for m in self.items:
                ret = m.find_selected(event)
                if ret is not None:
                    return ret

    def IsChecked(self):
        '''return true if item is checked'''
        return self.checked

    def id(self):
        '''id used to identify the returned menu items
        uses a 31 bit signed integer'''
        return int(hash((self.name, self.returnkey)) & 0x7FFFFFFF)

    def __str__(self):
        return "MPMenuItem(%s,%s,%s,%s)" % (self.name, self.description, self.returnkey, str(self.checked))

class MPTopMenu(object):
    '''a MP top level menu'''
    def __init__(self, items=[]):
        self.items = items

    def add(self, item):
        '''add a submenu'''
        self.items.append(item)

    def wx_menu(self):
        '''return a wx.MenuBar() for the menu'''
        menubar = wx.MenuBar()
        for i in range(len(self.items)):
            m = self.items[i]
            menubar.Append(m.wx_menu(), m.name)
        return menubar

    def find_selected(self, event):
        '''find the selected menu item'''
        for i in range(len(self.items)):
            m = self.items[i]
            ret = m.find_selected(event)
            if ret is not None:
                return ret
        return None

if __name__ == '__main__':
    from mp_image import MPImage
    import time
    im = MPImage(mouse_events=True,
                 key_events=True,
                 can_drag = False,
                 can_zoom = False,
                 auto_size = True)

    menu = MPTopMenu([MPMenuItem('&File',
                                 items=[MPMenuItem('&Open\tCtrl+O'),
                                        MPMenuItem('&Save\tCtrl+S'),
                                        MPMenuItem('Close', 'Close'),
                                        MPMenuItem('&Quit\tCtrl+Q', 'Quit')]),
                      MPMenuItem('Edit',
                                 items=[MPMenuItem('Option', 'EditOptions',
                                                   items=[MPMenuItem('Foo'),
                                                          MPMenuItem('Bar'),
                                                          MPMenuSeparator(),
                                                          MPMenuItem('&Grid\tCtrl+G',checkbox=True)]),
                                        MPMenuItem('Image', 'EditImage')])])
    
    im.set_menu(menu)
    while im.is_alive():
        for event in im.events():
            if isinstance(event, MPMenuItem):
                print(event)
                continue
            else:
                print(event)
        time.sleep(0.1)
        
