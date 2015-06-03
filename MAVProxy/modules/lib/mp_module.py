
class MPModule(object):
    '''
    The base class for all modules
    '''

    def __init__(self, mpstate, name, description=None, public=False):
        '''
        Constructor

        if public is true other modules can find this module instance with module('name')
        '''
        self.mpstate = mpstate
        self.name = name
        self.needs_unloading = False

        if description is None:
            self.description = name + " handling"
        else:
            self.description = description
        if public:
            mpstate.public_modules[name] = self

    #
    # Overridable hooks follow...
    #

    def idle_task(self):
        pass

    def unload(self):
        pass

    def unknown_command(self, args):
        '''Return True if we have handled the unknown command'''
        return False

    def mavlink_packet(self, packet):
        pass

    #
    # Methods for subclass use
    #

    def module(self, name):
        '''Find a public module (most modules are private)'''
        return self.mpstate.module(name)

    @property
    def console(self):
        return self.mpstate.console

    @property
    def status(self):
        return self.mpstate.status

    @property
    def mav_param(self):
        return self.mpstate.mav_param

    @property
    def settings(self):
        return self.mpstate.settings

    @property
    def vehicle_type(self):
        return self.mpstate.vehicle_type

    @property
    def vehicle_name(self):
        return self.mpstate.vehicle_name

    @property
    def sitl_output(self):
        return self.mpstate.sitl_output

    @property
    def target_system(self):
        return self.settings.target_system

    @property
    def target_component(self):
        return self.settings.target_component

    @property
    def master(self):
        return self.mpstate.master()

    @property
    def continue_mode(self):
        return self.mpstate.continue_mode

    @property
    def logdir(self):
        return self.mpstate.status.logdir

    def say(self, msg, priority='important'):
        return self.mpstate.functions.say(msg)

    def get_mav_param(self, param_name, default=None):
        return self.mpstate.functions.get_mav_param(param_name, default)

    def param_set(self, name, value, retries=3):
        self.mpstate.functions.param_set(name, value, retries)

    def add_command(self, name, callback, description, completions=None):
        self.mpstate.command_map[name] = (callback, description)
        if completions is not None:
            self.mpstate.completions[name] = completions

    def add_completion_function(self, name, callback):
        self.mpstate.completion_functions[name] = callback
