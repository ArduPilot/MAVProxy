#!/usr/bin/env python
'''settings object for MAVProxy modules'''

class MPSetting:
    def __init__(self, name, type, default, label=None):
        self.name = name
        self.type = type
        self.default = default
        self.label = label
        self.value = default

class MPSettings(object):
    def __init__(self, vars):
        self._vars = {}
        for v in vars:
            self.append(v)

    def append(self, v):
        '''add a new setting'''
        if isinstance(v, MPSetting):
            setting = v
        else:
            (name,type,default) = v
            setting = MPSetting(name, type, default)
        self._vars[setting.name] = setting

    def __getattr__(self, name):
        if name == '_vars':
            return self._vars
        if name in self._vars:
            return self._vars[name].value
        raise AttributeError

    def __setattr__(self, name, value):
        if name == '_vars':
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
        if value == 'None' and setting.default is None:
            value = None
        if value is not None:
            try:
                value = setting.type(value)
            except:
                print("Unable to convert %s to type %s" % (value, setting.type))
                return
        setting.value = value

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
