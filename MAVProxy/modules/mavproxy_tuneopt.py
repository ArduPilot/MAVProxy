#!/usr/bin/env python
'''tune command handling'''

import time, os

from MAVProxy.modules.lib import mp_module

tune_options = {
    'None':             '0',
    'StabRollPitchkP':  '1',
    'RateRollPitchkP':  '4',
    'RateRollPitchkI':  '6',
    'RateRollPitchkD':  '21',
    'StabYawkP':        '3',
    'RateYawkP':        '6',
    'RateYawkD':        '26',
    'AltitudeHoldkP':   '14',
    'ThrottleRatekP':   '7',
    'ThrottleRatekD':   '37',
    'ThrottleAccelkP':  '34',
    'ThrottleAccelkI':  '35',
    'ThrottleAccelkD':  '36',
    'LoiterPoskP':      '12',
    'LoiterRatekP':     '22',
    'LoiterRatekI':     '28',
    'LoiterRatekD':     '29',
    'WPSpeed':          '10',
    'AcroRollPitch kP': '25',
    'AcroYawkP':        '40',
    'RelayOnOff':       '9',
    'HeliExtGyro':      '13',
    'OFLoiterkP':       '17',
    'OFLoiterkI':       '18',
    'OFLoiterkD':       '19',
    'AHRSYawkP':        '30',
    'AHRSkP':           '31',
    'INAV_TC':          '32',
    'Declination':      '38',
    'CircleRate':       '39',
    'SonarGain':       '41',
}

class TuneoptModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TuneoptModule, self).__init__(mpstate, "tuneopt", "tuneopt command handling")
        self.add_command('tuneopt', self.cmd_tuneopt,  'Select option for Tune Pot on Channel 6 (quadcopter only)')

    def tune_show(self):
        opt_num = str(int(self.get_mav_param('TUNE')))
        option = None
        for k in tune_options.keys():
            if opt_num == tune_options[k]:
                option = k
                break
        else:
            print("TUNE is currently set to unknown value " + opt_num)
            return
        low = self.get_mav_param('TUNE_LOW')
        high = self.get_mav_param('TUNE_HIGH')
        print("TUNE is currently set to %s LOW=%f HIGH=%f" % (option, low/1000, high/1000))

    def tune_option_validate(self, option):
        for k in tune_options:
            if option.upper() == k.upper():
                return k
        return None

    # TODO: Check/show the limits of LOW and HIGH
    def cmd_tuneopt(self, args):
        '''Select option for Tune Pot on Channel 6 (quadcopter only)'''
        usage = "usage: tuneopt <set|show|reset|list>"
        if self.mpstate.vehicle_type != 'copter':
            print("This command is only available for copter")
            return
        if len(args) < 1:
            print(usage)
            return
        if args[0].lower() == 'reset':
            self.param_set('TUNE', '0')
        elif args[0].lower() == 'set':
            if len(args) < 4:
                print('Usage: tuneopt set OPTION LOW HIGH')
                return
            option = self.tune_option_validate(args[1])
            if not option:
                print('Invalid Tune option: ' + args[1])
                return
            low = args[2]
            high = args[3]
            self.param_set('TUNE', tune_options[option])
            self.param_set('TUNE_LOW', float(low) * 1000)
            self.param_set('TUNE_HIGH', float(high) * 1000)
        elif args[0].lower() == 'show':
            self.tune_show()
        elif args[0].lower() == 'list':
            print("Options available:")
            for s in sorted(tune_options.keys()):
                print('  ' + s)
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return TuneoptModule(mpstate)
