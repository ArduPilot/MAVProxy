#!/usr/bin/env python3
'''
menu handling widgets for wx

Andrew Tridgell
November 2013
'''

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
import platform

idmap = {}

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

    def __getstate__(self):
        '''use __getstate__ to override pickle so that we don't propogate IDs across process boundaries'''
        attr = self.__dict__.copy()
        if hasattr(attr,'_id'):
            del attr['_id']
        return attr

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
        '''id used to identify the returned menu items uses a 16 bit signed integer. We allocate these
           on use, and use __getstate__ to avoid them crossing processs boundaries'''
        if getattr(self, '_id', None) is None:
            global idmap
            import os
            key_tuple = (os.getpid(), self.name, self.returnkey)
            if key_tuple in idmap:
                self._id = idmap[key_tuple]
            else:
                from MAVProxy.modules.lib.wx_loader import wx
                self._id = wx.NewId()
                idmap[key_tuple] = self._id
        return self._id

    def _append(self, menu):
        '''append this menu item to a menu'''
        if not self.name:
            return
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
        try:
            menu.AppendCheckItem(self.id(), self.name, self.description)
            menu.Check(self.id(), self.checked)
        except Exception:
            pass

    def __str__(self):
        return "MPMenuCheckbox(%s,%s,%s,%s)" % (self.name, self.description, self.returnkey, str(self.checked))

class MPMenuRadio(MPMenuItem):
    '''a MP menu item as a radio item'''
    def __init__(self, name, description='', returnkey=None, selected=None, items=[], handler=None):
        MPMenuItem.__init__(self, name, description=description, returnkey=returnkey, handler=handler)
        self.items = items
        self.choice = 0
        self.initial = selected

    def __getstate__(self):
        attr = self.__dict__.copy()
        if hasattr(attr,'_ids'):
            del attr['_ids']
        return attr
        
    def set_choices(self, items):
        '''set radio item choices'''
        self.items = items

    def get_choice(self):
        '''return the chosen item'''
        return self.items[self.choice]

    def find_selected(self, event):
        '''find the selected menu item'''
        if not hasattr(self, '_ids'):
            return None
        evid = event.GetId()
        if evid in self._ids:
            self.choice = self._ids.index(evid)
            return self
        return None

    def _append(self, menu):
        '''append this menu item to a menu'''
        from MAVProxy.modules.lib.wx_loader import wx
        submenu = wx.Menu()
        if not hasattr(self, '_ids') or len(self._ids) != len(self.items):
            self._ids = []
            for i in range(len(self.items)):
                self._ids.append(wx.NewId())
        for i in range(len(self.items)):
            submenu.AppendRadioItem(self._ids[i], self.items[i], self.description)
            if self.items[i] == self.initial:
                submenu.Check(self._ids[i], True)
        menu.AppendSubMenu(submenu, self.name)

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

    def remove(self, items):
        '''remove items from a sub-menu'''
        if not isinstance(items, list):
            items = [items]
        names = set([x.name for x in items])
        self.items = list(filter(lambda x : x.name not in names, self.items))

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
        from MAVProxy.modules.lib.wx_loader import wx
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
        menu.AppendSubMenu(self.wx_menu(), self.name) #use wxPython_phoenix

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

    def remove(self, items):
        if not isinstance(items, list):
            items = [items]
        names = set([x.name for x in items])
        self.items = list(filter(lambda x : x.name not in names, self.items))

    def add_to_submenu(self, submenu_path, item):
        '''
        add an item to a submenu using a menu path array
        '''
        for m in self.items:
            if m.name == submenu_path[0]:
                m.add_to_submenu(submenu_path[1:], item)
                return
        # new submenu
        if len(submenu_path) > 1:
            self.add(MPMenuSubMenu(submenu_path[0], []))
            self.add_to_submenu(submenu_path, item)
        else:
            self.add(MPMenuSubMenu(submenu_path[0], [item]))

    def wx_menu(self):
        '''return a wx.MenuBar() for the menu'''
        from MAVProxy.modules.lib.wx_loader import wx

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
        from MAVProxy.modules.lib.wx_loader import wx

        # remap flags to wx descriptors
        flag_map = {
            'open': wx.FD_OPEN,
            'save': wx.FD_SAVE,
            'overwrite_prompt': wx.FD_OVERWRITE_PROMPT,
        }
        flagsMapped = list(map(lambda x: flag_map[x], self.flags))

        #need to OR together the elements of the flagsMapped tuple
        if len(flagsMapped) == 1:
            dlg = wx.FileDialog(None, self.title, '', "", self.wildcard, flagsMapped[0])
        else:
            dlg = wx.FileDialog(None, self.title, '', "", self.wildcard, flagsMapped[0]|flagsMapped[1])
        if dlg.ShowModal() != wx.ID_OK:
            return None
        return "\"" + dlg.GetPath() + "\""


class MPMenuCallDirDialog(object):
    '''used to create a file folder dialog callback'''
    def __init__(self, flags=None, title='Directory'):
        self.title = title

    def call(self):
        '''show a directory chooser dialog'''
        from MAVProxy.modules.lib.wx_loader import wx

        dlg = wx.DirDialog(None, self.title, '')
        if dlg.ShowModal() != wx.ID_OK:
            return None
        return "\"" + dlg.GetPath() + "\""


class MPMenuCallTextDialog(object):
    '''used to create a value dialog callback'''
    def __init__(self, title='Enter Value', default='', settings=None):
        self.title = title
        self.default = default
        self.settings = settings

    def call(self):
        '''show a value dialog'''
        from MAVProxy.modules.lib.wx_loader import wx
        title = self.title
        try:
            dlg = wx.TextEntryDialog(None, title, title, defaultValue=str(self.default))
        except TypeError:
            dlg = wx.TextEntryDialog(None, title, title, value=str(self.default))
        if dlg.ShowModal() != wx.ID_OK:
            return None
        return dlg.GetValue()

# memory of last dropdowns
last_dropdown_selection = {}
last_value_selection = {}

class MPMenuCallTextDropdownDialog(object):
    '''used to create a value dialog with dropdown callback'''
    def __init__(self, title='Enter Value', default='',
                 dropdown_options=None,
                 dropdown_label='Select option',
                 default_dropdown=None):
        self.title = title
        self.default = default
        self.default_dropdown = default_dropdown
        self.dropdown_options = dropdown_options or []
        self.dropdown_label = dropdown_label

    def call(self):
        '''show a value dialog with dropdown'''
        from MAVProxy.modules.lib.wx_loader import wx
        
        # Create a custom dialog
        dlg = wx.Dialog(None, title=self.title, size=(400, 150))
        
        # Create a vertical box sizer for the dialog
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        
        # Create a horizontal sizer for the text entry and dropdown
        input_sizer = wx.BoxSizer(wx.HORIZONTAL)
        
        # Text entry label and control
        text_label = wx.StaticText(dlg, label="Value:")
        default = last_value_selection.get(self.title, self.default)
        text_ctrl = wx.TextCtrl(dlg, value=str(default), size=(200, -1))
        
        # Add text control components to input sizer
        input_sizer.Add(text_label, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        input_sizer.Add(text_ctrl, 1, wx.ALL | wx.EXPAND, 5)
        
        # Dropdown label and control
        dropdown_label = wx.StaticText(dlg, label=self.dropdown_label)
        dropdown_ctrl = wx.Choice(dlg, choices=self.dropdown_options)
        
        # Select first item by default if options exist
        if self.dropdown_options:
            default_idx = 0
            for i in range(len(self.dropdown_options)):
                if self.dropdown_options[i] == self.default_dropdown:
                    default_idx = i
            dropdown_ctrl.SetSelection(last_dropdown_selection.get(self.title,default_idx))

        # Add dropdown components to input sizer
        input_sizer.Add(dropdown_label, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        input_sizer.Add(dropdown_ctrl, 0, wx.ALL | wx.EXPAND, 5)
        
        # Add the input sizer to the main sizer
        main_sizer.Add(input_sizer, 0, wx.ALL | wx.EXPAND, 5)
        
        # Create button sizer with OK and Cancel buttons
        button_sizer = dlg.CreateButtonSizer(wx.OK | wx.CANCEL)
        main_sizer.Add(button_sizer, 0, wx.ALL | wx.ALIGN_CENTER, 10)
        
        # Set the sizer for the dialog
        dlg.SetSizer(main_sizer)
        
        # Fit the dialog to its contents
        dlg.Fit()
        
        # Show the dialog and get the result
        if dlg.ShowModal() != wx.ID_OK:
            return None
        
        text_value = text_ctrl.GetValue()
        dropdown_index = dropdown_ctrl.GetSelection()
        dropdown_value = self.dropdown_options[dropdown_index] if dropdown_index != -1 and self.dropdown_options else ""

        last_dropdown_selection[self.title] = dropdown_index
        last_value_selection[self.title] = text_value

        # Return tuple with text value and selected dropdown value
        return text_value + " " + dropdown_value

class MPMenuCallMultiTextDropdownDialog(object):
    '''used to create a dialog callback with multiple values and optional dropdowns.

    Each field is a (label, default) tuple, and each dropdown a
    (label, options, default) tuple.  An extra trailing element - a key -
    may be added to either (i.e. (label, default, key) or
    (label, options, default, key)); when present the corresponding value
    is emitted as "key=value", otherwise the bare value is emitted.
    '''
    def __init__(self, title='Enter Values', fields=None, dropdowns=None):
        self.title = title
        self.fields = fields or []
        self.dropdowns = dropdowns or []

    @classmethod
    def from_arg_spec(cls, title, spec, ctx=None):
        '''build a dialog from a command argument spec.

        spec is an ordered dict of key->facets (see
        MPModule.parse_key_value_spec).  Each entry that carries a 'label'
        becomes a widget emitting "key=value": a dropdown when it has an
        'options' list, otherwise a text field.  Entries without a 'label'
        (e.g. keys entered another way) are skipped.  A 'default' facet may
        be a plain value or a callable which is passed ctx to produce a
        live default.
        '''
        fields = []
        dropdowns = []
        for (key, facets) in spec.items():
            if 'label' not in facets:
                continue
            default = facets.get('default', '')
            if callable(default):
                default = default(ctx)
            if 'options' in facets:
                dropdowns.append((facets['label'], facets['options'], default, key))
            else:
                fields.append((facets['label'], default, key))
        return cls(title=title, fields=fields, dropdowns=dropdowns)

    def call(self):
        '''show a dialog with multiple value entries and optional dropdowns'''
        from MAVProxy.modules.lib.wx_loader import wx

        # Create a custom dialog
        dlg = wx.Dialog(None, title=self.title, size=(400, 150))

        # Create a vertical box sizer for the dialog
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # One row of label plus text entry per field
        text_ctrls = []
        for field in self.fields:
            (label, default) = field[0], field[1]
            input_sizer = wx.BoxSizer(wx.HORIZONTAL)
            text_label = wx.StaticText(dlg, label=label + ":")
            memory_key = self.title + ":" + label
            default = last_value_selection.get(memory_key, default)
            text_ctrl = wx.TextCtrl(dlg, value=str(default), size=(200, -1))
            text_ctrls.append(text_ctrl)
            input_sizer.Add(text_label, 1, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
            input_sizer.Add(text_ctrl, 0, wx.ALL | wx.EXPAND, 5)
            main_sizer.Add(input_sizer, 0, wx.ALL | wx.EXPAND, 5)

        # One row of label plus choice control per dropdown
        dropdown_ctrls = []
        for dropdown in self.dropdowns:
            (label, options, default) = dropdown[0], dropdown[1], dropdown[2]
            input_sizer = wx.BoxSizer(wx.HORIZONTAL)
            dropdown_label = wx.StaticText(dlg, label=label)
            dropdown_ctrl = wx.Choice(dlg, choices=options)
            dropdown_ctrls.append(dropdown_ctrl)

            default_idx = 0
            for i in range(len(options)):
                if options[i] == default:
                    default_idx = i
            memory_key = self.title + ":" + label
            dropdown_ctrl.SetSelection(last_dropdown_selection.get(memory_key, default_idx))

            input_sizer.Add(dropdown_label, 1, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
            input_sizer.Add(dropdown_ctrl, 0, wx.ALL | wx.EXPAND, 5)
            main_sizer.Add(input_sizer, 0, wx.ALL | wx.EXPAND, 5)

        # Create button sizer with OK and Cancel buttons
        button_sizer = dlg.CreateButtonSizer(wx.OK | wx.CANCEL)
        main_sizer.Add(button_sizer, 0, wx.ALL | wx.ALIGN_CENTER, 10)

        # Set the sizer for the dialog
        dlg.SetSizer(main_sizer)

        # Fit the dialog to its contents
        dlg.Fit()

        # Show the dialog and get the result
        if dlg.ShowModal() != wx.ID_OK:
            return None

        values = []
        for (field, text_ctrl) in zip(self.fields, text_ctrls):
            label = field[0]
            key = field[2] if len(field) > 2 else None
            text_value = text_ctrl.GetValue()
            last_value_selection[self.title + ":" + label] = text_value
            values.append("%s=%s" % (key, text_value) if key else text_value)

        for (dropdown, dropdown_ctrl) in zip(self.dropdowns, dropdown_ctrls):
            (label, options) = dropdown[0], dropdown[1]
            key = dropdown[3] if len(dropdown) > 3 else None
            dropdown_index = dropdown_ctrl.GetSelection()
            if dropdown_index != -1:
                dropdown_value = options[dropdown_index]
                last_dropdown_selection[self.title + ":" + label] = dropdown_index
                values.append("%s=%s" % (key, dropdown_value) if key else dropdown_value)

        # Return the entered values, space-separated, dropdown values last
        return " ".join(values)

class MPMenuConfirmDialog(object):
    '''used to create a confirmation dialog'''
    def __init__(self, title='Confirmation', message='', callback=None, args=None):
        self.title = title
        self.message = message
        self.callback = callback
        self.args = args
        t = multiproc.Process(target=self.thread)
        t.start()

    def thread(self):
        mp_util.child_close_fds()
        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        app = wx.App(False)
        dlg = wx.MessageDialog(None, self.title, self.message, wx.YES|wx.NO)
        ret = dlg.ShowModal()
        if ret == wx.ID_YES and self.callback is not None:
            self.callback(self.args)

class MPMenuChildMessageDialog(object):
    '''used to create a message dialog in a child process'''
    def __init__(self, title='Information', message='', font_size=18):
        self.title = title
        self.message = message
        self.font_size = font_size

    def show(self):
        t = multiproc.Process(target=self.call)
        t.start()

    def call(self):
        '''show the dialog as a child process'''
        mp_util.child_close_fds()
        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
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
