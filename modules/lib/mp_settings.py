#!/usr/bin/env python
'''settings object for MAVProxy modules'''

class MPSettings(object):
    def __init__(self, vars):
        self.vars = vars
        for var in vars:
            (v,t,d) = var
            self.set(v, d)

    def set(self, vname, value):
        '''set a setting'''
        if value is None or value == 'None':
            setattr(self, vname, None)
            return
        for (v,t,d) in sorted(self.vars):
            if v == vname:
                try:
                    value = t(value)
                except:
                    print("Unable to convert %s to type %s" % (value, t))
                    return
                setattr(self, vname, value)
                return
        print("Invalid setting %s" % vname)

    def show(self, v):
        '''show settings'''
        print("%20s %s" % (v, getattr(self, v)))

    def show_all(self):
        '''show all settings'''
        for (v,t,d) in sorted(self.vars):
            self.show(v)
