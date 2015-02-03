#!/usr/bin/env python
'''profile support'''


from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil


class ProfileModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ProfileModule, self).__init__(mpstate, "profile", "profiling support")
        self.add_command('reset', self.cmd_reset, "restart profiler")
        self.add_command('stop', self.cmd_stop,   'stop profiler')
        self.cmd_reset(None)

    def cmd_reset(self, args):
        '''restart profiling'''
        import yappi    # We do the import here so that we won't barf if run normally and yappi not available
        print "NOW PROFILING..."
        yappi.start()

    def cmd_stop(self, args):
        '''stop profiling'''
        print "Profile results:"
        import yappi    # We do the import here so that we won't barf if run normally and yappi not available        
        yappi.get_func_stats().print_all()
        yappi.get_thread_stats().print_all()
        yappi.stop()

    def unload(self):
        self.cmd_stop(None)

def init(mpstate):
    '''initialise module'''
    return ProfileModule(mpstate)
