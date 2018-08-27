
class MPModule(object):
    '''
    The base class for all modules
    '''

    def __init__(self, mpstate, name, description=None, public=False, multi_instance=False):
        '''
        Constructor

        if public is true other modules can find this module instance with module('name')
        '''
        self.mpstate = mpstate
        self.name = name
        self.needs_unloading = False
        self.multi_instance = multi_instance

        if description is None:
            self.description = name + " handling"
        else:
            self.description = description
        if multi_instance:
            if not name in mpstate.multi_instance:
                mpstate.multi_instance[name] = []
                mpstate.instance_count[name] = 0
            mpstate.multi_instance[name].append(self)
            mpstate.instance_count[name] += 1
            self.instance = mpstate.instance_count[name]
            if self.instance > 1:
                # make the name distincitive, so self.module('map2') works
                self.name += str(self.instance)
        if public:
            mpstate.public_modules[self.name] = self

    #
    # Overridable hooks follow...
    #

    def idle_task(self):
        pass

    def unload(self):
        if self.multi_instance and self.name in self.mpstate.multi_instance:
            self.mpstate.multi_instance.remove(self)

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

    def module_matching(self, name):
        '''Find a list of modules matching a wildcard pattern'''
        import fnmatch
        ret = []
        for mname in self.mpstate.public_modules.keys():
            if fnmatch.fnmatch(mname, name):
                ret.append(self.mpstate.public_modules[mname])
        return ret
    
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

    def dist_string(self, val_meters):
        '''return a distance as a string'''
        if self.settings.dist_unit == 'nm':
            return "%.1fnm" % (val_meters * 0.000539957)
        if self.settings.dist_unit == 'miles':
            return "%.1fmiles" % (val_meters * 0.000621371)
        return "%um" % val_meters

    def height_convert_units(self, val_meters):
        '''return a height in configured units'''
        if self.settings.height_unit == 'feet':
            return val_meters * 3.28084
        return val_meters

    def height_string(self, val_meters):
        '''return a height as a string'''
        if self.settings.height_unit == 'feet':
            return "%uft" % (val_meters * 3.28084)
        return "%um" % val_meters

    def speed_convert_units(self, val_ms):
        '''return a speed in configured units'''
        if self.settings.speed_unit == 'knots':
            return val_ms * 1.94384
        return val_ms

    def speed_string(self, val_ms):
        '''return a speed as a string'''
        if self.settings.speed_unit == 'knots':
            return "%ukn" % (val_ms * 1.94384)
        return "%um/s" % val_ms

    def set_prompt(self, prompt):
        '''set prompt for command line'''
        if prompt and self.settings.vehicle_name:
            # add in optional vehicle name
            prompt = self.settings.vehicle_name + ':' + prompt
        self.mpstate.rl.set_prompt(prompt)
            
    @staticmethod
    def link_label(link):
        '''return a link label as a string'''
        if hasattr(link, 'label'):
            label = link.label
        else:
            label = str(link.linknum+1)
        return label
