#!/usr/bin/env python
'''auxopt command handling'''

import time, os
from MAVProxy.modules.lib import mp_module


aux_options = {
    "Nothing":"0",
    "Flip":"2",
    "SimpleMode":"3",
    "RTL":"4",
    "SaveTrim":"5",
    "SaveWP":"7",
    "MultiMode":"8",
    "CameraTrigger":"9",
    "Sonar":"10",
    "Fence":"11",
    "ResetYaw":"12",
    "SuperSimpleMode":"13",
    "AcroTrainer":"14",
    "Auto":"16",
    "AutoTune":"17",
    "Land":"18"
}

class AuxoptModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(AuxoptModule, self).__init__(mpstate, "auxopt", "auxopt command handling")
        self.add_command('auxopt', self.cmd_auxopt,   'select option for aux switches on CH7 and CH8 (ArduCopter only)',
                         ['set <7|8> <Nothing|Flip|SimpleMode|RTL|SaveTrim|SaveWP|MultiMode|CameraTrigger|Sonar|Fence|ResetYaw|SuperSimpleMode|AcroTrainer|Acro|Auto|AutoTune|Land>',
                          'reset <7|8|all>',
                          '<show|list>'])

    def aux_show(self, channel):
        param = "CH%s_OPT" % channel
        opt_num = str(int(self.get_mav_param(param)))
        option = None
        for k in aux_options.keys():
            if opt_num == aux_options[k]:
                option = k
                break
        else:
            print("AUX Channel is currently set to unknown value " + opt_num)
            return
        print("AUX Channel is currently set to " + option)

    def aux_option_validate(self, option):
        for k in aux_options:
            if option.upper() == k.upper():
                return k
        return None

    def cmd_auxopt(self, args):
        '''handle AUX switches (CH7, CH8) settings'''
        if self.mpstate.vehicle_type != 'copter':
            print("This command is only available for copter")
            return
        if len(args) == 0 or args[0] not in ('set', 'show', 'reset', 'list'):
            print("Usage: auxopt set|show|reset|list")
            return
        if args[0] == 'list':
            print("Options available:")
            for s in sorted(aux_options.keys()):
                print('  ' + s)
        elif args[0] == 'show':
            if len(args) > 2 and args[1] not in ['7', '8', 'all']:
                print("Usage: auxopt show [7|8|all]")
                return
            if len(args) < 2 or args[1] == 'all':
                self.aux_show('7')
                self.aux_show('8')
                return
            self.aux_show(args[1])
        elif args[0] == 'reset':
            if len(args) < 2 or args[1] not in ['7', '8', 'all']:
                print("Usage: auxopt reset 7|8|all")
                return
            if args[1] == 'all':
                self.param_set('CH7_OPT', '0')
                self.param_set('CH8_OPT', '0')
                return
            param = "CH%s_OPT" % args[1]
            self.param_set(param, '0')
        elif args[0] == 'set':
            if len(args) < 3 or args[1] not in ['7', '8']:
                print("Usage: auxopt set 7|8 OPTION")
                return
            option = self.aux_option_validate(args[2])
            if not option:
                print("Invalid option " + args[2])
                return
            param = "CH%s_OPT" % args[1]
            self.param_set(param, aux_options[option])
        else:
            print("Usage: auxopt set|show|list")

def init(mpstate):
    '''initialise module'''
    return AuxoptModule(mpstate)
