#!/usr/bin/env python
'''settings object for MAVProxy modules'''

class MPSetting:
    def __init__(self, name, type, default, label=None, tab=None):
        if label is None:
            label = name
        self.name = name
        self.type = type
        self.default = default
        self.label = label
        self.value = default
        self.tab = tab

    def set(self, value):
        '''set a setting'''
        if value == 'None' and self.default is None:
            value = None
        if value is not None:
            try:
                value = self.type(value)
            except:
                return False
        self.value = value
        return True

class MPSettings(object):
    def __init__(self, vars, title='Settings'):
        self._vars = {}
        self._title = title
        self._default_tab = 'Settings'
        for v in vars:
            self.append(v)

    def get_title(self):
        '''return the title'''
        return self._title

    def get_setting(self, name):
        '''return a MPSetting object'''
        return self._vars[name]

    def append(self, v):
        '''add a new setting'''
        if isinstance(v, MPSetting):
            setting = v
        else:
            (name,type,default) = v
            label = name
            tab = None
            if len(v) > 3:
                label = v[3]
            if len(v) > 4:
                tab = v[4]
            setting = MPSetting(name, type, default, label=label, tab=tab)

        # when a tab name is set, cascade it to future settings
        if setting.tab is None:
            setting.tab = self._default_tab
        else:
            self._default_tab = setting.tab
        self._vars[setting.name] = setting
            

    def __getattr__(self, name):
        if name in self._vars:
            return self._vars[name].value
        raise AttributeError

    def __setattr__(self, name, value):
        if name[0] == '_':
            self.__dict__[name] = value
            return
        if name in self._vars:
            self._vars[name].value = value
            return
        raise AttributeError

    def set(self, name, value):
        '''set a setting'''
        if not name in self._vars:
            raise AttributeError
        setting = self._vars[name]
        if not setting.set(value):
            print("Unable to convert %s to type %s" % (value, setting.type))
            return False
        return True

    def show(self, v):
        '''show settings'''
        print("%20s %s" % (v, getattr(self, v)))

    def show_all(self):
        '''show all settings'''
        for setting in sorted(self._vars):
            self.show(setting)

    def list(self):
        '''list all settings'''
        return self._vars.keys()

    def completion(self, text):
        '''completion function for cmdline completion'''
        return self.list()

    def command(self, args):
        '''control options from cmdline'''
        if len(args) == 0:
            self.show_all()
            return
        if getattr(self, args[0], [None]) == [None]:
            print("Unknown setting '%s'" % args[0])
            return
        if len(args) == 1:
            self.show(args[0])
        else:
            self.set(args[0], args[1])
