#!/usr/bin/env python
'''
menu handling widgets for wx

Andrew Tridgell
November 2013
'''

from MAVProxy.modules.lib import mp_util

class MPMenuGeneric(object):
    '''a MP menu separator'''
    def __init__(self):
        pass

    def find_selected(self, event):
        return None

    def _append(self, menu):
        '''append this menu item to a menu'''
        pass

    def __str__(self):
        return "MPMenuGeneric()"

    def __repr__(self):
        return str(self.__str__())

class MPMenuSeparator(MPMenuGeneric):
    '''a MP menu separator'''
    def __init__(self):
        MPMenuGeneric.__init__(self)

    def _append(self, menu):
        '''append this menu item to a menu'''
        menu.AppendSeparator()

    def __str__(self):
        return "MPMenuSeparator()"


class MPMenuItem(MPMenuGeneric):
    '''a MP menu item'''
    def __init__(self, name, description='', returnkey=None, handler=None):
        MPMenuGeneric.__init__(self)
        self.name = name
        self.description = description
        self.returnkey = returnkey
        self.handler = handler
        self.handler_result = None

    def find_selected(self, event):
        '''find the selected menu item'''
        if event.GetId() == self.id():
            return self
        return None

    def call_handler(self):
        '''optionally call a handler function'''
        if self.handler is None:
            return
        call = getattr(self.handler, 'call', None)
        if call is not None:
            self.handler_result = call()

    def id(self):
        '''id used to identify the returned menu items
        uses a 16 bit signed integer'''
        # 0x7FFF is used as windows only allows for 16 bit IDs
        return int(hash((self.name, self.returnkey)) & 0x7FFF)

    def _append(self, menu):
        '''append this menu item to a menu'''
        menu.Append(self.id(), self.name, self.description)

    def __str__(self):
        return "MPMenuItem(%s,%s,%s)" % (self.name, self.description, self.returnkey)


class MPMenuCheckbox(MPMenuItem):
    '''a MP menu item as a checkbox'''
    def __init__(self, name, description='', returnkey=None, checked=False, handler=None):
        MPMenuItem.__init__(self, name, description=description, returnkey=returnkey, handler=handler)
        self.checked = checked

    def find_selected(self, event):
        '''find the selected menu item'''
        if event.GetId() == self.id():
            self.checked = event.IsChecked()
            return self
        return None

    def IsChecked(self):
        '''return true if item is checked'''
        return self.checked

    def _append(self, menu):
        '''append this menu item to a menu'''
        menu.AppendCheckItem(self.id(), self.name, self.description)
        menu.Check(self.id(), self.checked)

    def __str__(self):
        return "MPMenuCheckbox(%s,%s,%s,%s)" % (self.name, self.description, self.returnkey, str(self.checked))

class MPMenuRadio(MPMenuItem):
    '''a MP menu item as a radio item'''
    def __init__(self, name, description='', returnkey=None, selected=None, items=[], handler=None):
        MPMenuItem.__init__(self, name, description=description, returnkey=returnkey, handler=handler)
        self.items = items
        self.choice = 0
        self.initial = selected

    def set_choices(self, items):
        '''set radio item choices'''
        self.items = items

    def get_choice(self):
        '''return the chosen item'''
        return self.items[self.choice]

    def find_selected(self, event):
        '''find the selected menu item'''
        first = self.id()
        last = first + len(self.items) - 1
        evid = event.GetId()
        if evid >= first and evid <= last:
            self.choice = evid - first
            return self
        return None

    def _append(self, menu):
        '''append this menu item to a menu'''
        from wx_loader import wx
        submenu = wx.Menu()
        for i in range(len(self.items)):
            submenu.AppendRadioItem(self.id()+i, self.items[i], self.description)
            if self.items[i] == self.initial:
                submenu.Check(self.id()+i, True)
        menu.AppendMenu(-1, self.name, submenu)

    def __str__(self):
        return "MPMenuRadio(%s,%s,%s,%s)" % (self.name, self.description, self.returnkey, self.get_choice())


class MPMenuSubMenu(MPMenuGeneric):
    '''a MP menu item'''
    def __init__(self, name, items):
        MPMenuGeneric.__init__(self)
        self.name = name
        self.items = items

    def add(self, items, addto=None):
        '''add more items to a sub-menu'''
        if not isinstance(items, list):
            items = [items]
        for m in items:
            updated = False
            for i in range(len(self.items)):
                try:
                    if self.items[i].name == m.name:
                        self.items[i] = m
                        updated = True
                except Exception:
                    pass
            if not updated:
                self.items.append(m)

    def add_to_submenu(self, submenu_path, item):
        '''add an item to a submenu using a menu path array'''
        if len(submenu_path) == 0:
            self.add(item)
            return
        for m in self.items:
            if isinstance(m, MPMenuSubMenu) and submenu_path[0] == m.name:
                m.add_to_submenu(submenu_path[1:], item)
                return
        self.add(MPMenuSubMenu(submenu_path[0], []))
        self.add_to_submenu(submenu_path, item)

    def combine(self, submenu):
        '''combine a new menu with an existing one'''
        self.items.extend(submenu.items)

    def wx_menu(self):
        '''return a wx.Menu() for this menu'''
        from wx_loader import wx
        menu = wx.Menu()
        for i in range(len(self.items)):
            m = self.items[i]
            m._append(menu)
        return menu

    def find_selected(self, event):
        '''find the selected menu item'''
        for m in self.items:
            ret = m.find_selected(event)
            if ret is not None:
                return ret
        return None

    def _append(self, menu):
        '''append this menu item to a menu'''
        from wx_loader import wx

        menu.AppendMenu(-1, self.name, self.wx_menu())

    def __str__(self):
        return "MPMenuSubMenu(%s)" % (self.name)


class MPMenuTop(object):
    '''a MP top level menu'''
    def __init__(self, items):
        self.items = items

    def add(self, items):
        '''add a submenu'''
        if not isinstance(items, list):
            items = [items]
        for m in items:
            updated = False
            for i in range(len(self.items)):
                if self.items[i].name == m.name:
                    self.items[i] = m
                    updated = True
            if not updated:
                self.items.append(m)

    def wx_menu(self):
        '''return a wx.MenuBar() for the menu'''
        from wx_loader import wx
        
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

class MPMenuCallFileDialog(object):
    '''used to create a file dialog callback'''
    def __init__(self, flags=None, title='Filename', wildcard='*.*'):
        self.flags = flags or ('open',)
        self.title = title
        self.wildcard = wildcard

    def call(self):
        '''show a file dialog'''
        from wx_loader import wx

        # remap flags to wx descriptors
        flag_map = {
            'open': wx.FD_OPEN,
            'save': wx.FD_SAVE,
            'overwrite_prompt': wx.FD_OVERWRITE_PROMPT,
        }
        flagsMapped = map(lambda x: flag_map[x], self.flags)
        
        #need to OR together the elements of the flagsMapped tuple
        if len(flagsMapped) == 1:
            dlg = wx.FileDialog(None, self.title, '', "", self.wildcard, flagsMapped[0])
        else:
            dlg = wx.FileDialog(None, self.title, '', "", self.wildcard, flagsMapped[0]|flagsMapped[1])
        if dlg.ShowModal() != wx.ID_OK:
            return None
        return dlg.GetPath()

class MPMenuCallTextDialog(object):
    '''used to create a value dialog callback'''
    def __init__(self, title='Enter Value', default=''):
        self.title = title
        self.default = default

    def call(self):
        '''show a value dialog'''
        from wx_loader import wx

        dlg = wx.TextEntryDialog(None, self.title, self.title, defaultValue=str(self.default))
        if dlg.ShowModal() != wx.ID_OK:
            return None
        return dlg.GetValue()

class MPMenuChildMessageDialog(object):
    '''used to create a message dialog in a child process'''
    def __init__(self, title='Information', message='', font_size=18):
        self.title = title
        self.message = message
        self.font_size = font_size

    def show(self):        
        import multiprocessing
        t = multiprocessing.Process(target=self.call)
        t.start()

    def call(self):
        '''show the dialog as a child process'''
        mp_util.child_close_fds()
        import wx_processguard
        from wx_loader import wx
        from wx.lib.agw.genericmessagedialog import GenericMessageDialog
        app = wx.App(False)
        # note! font size change is not working. I don't know why yet
        font = wx.Font(self.font_size, wx.MODERN, wx.NORMAL, wx.NORMAL)
        dlg = GenericMessageDialog(None, self.message, self.title, wx.ICON_INFORMATION|wx.OK)
        dlg.SetFont(font)
        dlg.ShowModal()
        app.MainLoop()

class MPMenuOpenWeblink(object):
    '''used to open a weblink in the default webbrowser'''
    def __init__(self, url='www.google.com'):
        self.url = url

    def call(self):
        '''show the dialog as a child process'''
        import webbrowser
        webbrowser.open_new_tab(self.url)
        
if __name__ == '__main__':
    from MAVProxy.modules.lib.mp_image import MPImage
    import time
    im = MPImage(mouse_events=True,
                 key_events=True,
                 can_drag = False,
                 can_zoom = False,
                 auto_size = True)

    menu = MPMenuTop([MPMenuSubMenu('&File',
                                    items=[MPMenuItem('&Open\tCtrl+O'),
                                           MPMenuItem('&Save\tCtrl+S'),
                                           MPMenuItem('Close', 'Close'),
                                           MPMenuItem('&Quit\tCtrl+Q', 'Quit')]),
                      MPMenuSubMenu('Edit',
                                    items=[MPMenuSubMenu('Option',
                                                         items=[MPMenuItem('Foo'),
                                                                MPMenuItem('Bar'),
                                                                MPMenuSeparator(),
                                                                MPMenuCheckbox('&Grid\tCtrl+G')]),
                                           MPMenuItem('Image', 'EditImage'),
                                           MPMenuRadio('Colours',
                                                       items=['Red','Green','Blue']),
                                           MPMenuRadio('Shapes',
                                                       items=['Circle','Square','Triangle'])])])

    im.set_menu(menu)

    popup = MPMenuSubMenu('A Popup',
                          items=[MPMenuItem('Sub1'),
                                 MPMenuItem('Sub2'),
                                 MPMenuItem('Sub3')])

    im.set_popup_menu(popup)

    while im.is_alive():
        for event in im.events():
            if isinstance(event, MPMenuItem):
                print(event, getattr(event, 'popup_pos', None))
                continue
            else:
                print(event)
        time.sleep(0.1)
