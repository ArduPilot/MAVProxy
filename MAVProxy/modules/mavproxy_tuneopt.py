#!/usr/bin/env python3
'''tune command handling'''

import time, os

from MAVProxy.modules.lib import mp_module

# the TUNE options Copter's tuning code handles: WPSpeed and HeliExtGyro
# up to 4.7, PSCAngleMax and LoiterSpeed from 4.7, WPSpeedMS after 4.7
tune_options = {
    'None':             '0',
    'StabRollPitchkP':  '1',
    'RateRollPitchkP':  '4',
    'RateRollPitchkI':  '5',
    'RateRollPitchkD':  '21',
    'StabYawkP':        '3',
    'RateYawkP':        '6',
    'RateYawkD':        '26',
    'AltitudeHoldkP':   '14',
    'ThrottleRatekP':   '7',
    'ThrottleAccelkP':  '34',
    'ThrottleAccelkI':  '35',
    'ThrottleAccelkD':  '36',
    'LoiterSpeed':      '60',
    'LoiterPoskP':      '12',
    'VelocityXYkP':     '22',
    'VelocityXYkI':     '28',
    'WPSpeed':          '10',
    'WPSpeedMS':        '61',
    'AcroRollPitchkP':  '25',
    'AcroYawkP':        '40',
    'HeliExtGyro':      '13',
    'Declination':      '38',
    'CircleRate':       '39',
    'RCFeel':           '45',
    'RatePitchkP':      '46',
    'RatePitchkI':      '47',
    'RatePitchkD':      '48',
    'RateRollkP':       '49',
    'RateRollkI':       '50',
    'RateRollkD':       '51',
    'RatePitchFF':      '52',
    'RateRollFF':       '53',
    'RateYawFF':        '54',
    'MotorYawHeadroom': '55',
    'RateYawFilter':    '56',
    'SysIDMagnitude':   '58',
    'PSCAngleMax':      '59',
}

# the options of Copter 3.6 and earlier, which hold 1000 times the value in
# TUNE_LOW and TUNE_HIGH: some of the newer options scale differently there
legacy_tune_options = {
    name: value for (name, value) in dict(
        tune_options, LoiterSpeed='42', OFLoiterkP='17', OFLoiterkI='18', OFLoiterkD='19',
        RangeFinderGain='41').items()
    if name not in ['WPSpeedMS', 'RCFeel', 'MotorYawHeadroom', 'RateYawFilter', 'SysIDMagnitude', 'PSCAngleMax']
}

class TuneoptModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "tuneopt", "tuneopt command handling")
        self.add_command('tuneopt', self.cmd_tuneopt,  'Select option for Tune Pot on Channel 6 (quadcopter only)')

    def tune_limits(self):
        '''the parameters holding the tuning range, the scale from the values
        the user types to theirs, and the options: Copter 4.0 replaced
        TUNE_LOW and TUNE_HIGH, which hold 1000 times the value, with TUNE_MIN
        and TUNE_MAX.  Until the parameters are known, assume the new names'''
        if self.get_mav_param('TUNE_MIN') is None and self.get_mav_param('TUNE_LOW') is not None:
            return ('TUNE_LOW', 'TUNE_HIGH', 1000.0, legacy_tune_options)
        return ('TUNE_MIN', 'TUNE_MAX', 1.0, tune_options)

    def tune_show(self):
        opt_num = str(int(self.get_mav_param('TUNE')))
        (low_name, high_name, scale, options) = self.tune_limits()
        option = None
        for k in options.keys():
            if opt_num == options[k]:
                option = k
                break
        else:
            print("TUNE is currently set to unknown value " + opt_num)
            return
        low = self.get_mav_param(low_name)
        high = self.get_mav_param(high_name)
        if low is None or high is None:
            print("TUNE is currently set to %s, range unknown" % option)
            return
        print("TUNE is currently set to %s LOW=%f HIGH=%f" % (option, low/scale, high/scale))

    def tune_option_validate(self, option, options):
        for k in options:
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
            (low_name, high_name, scale, options) = self.tune_limits()
            option = self.tune_option_validate(args[1], options)
            if not option:
                print('Invalid Tune option: ' + args[1])
                return
            try:
                low = float(args[2])
                high = float(args[3])
            except ValueError:
                print('Usage: tuneopt set OPTION LOW HIGH')
                return
            self.param_set('TUNE', options[option])
            self.param_set(low_name, low * scale)
            self.param_set(high_name, high * scale)
        elif args[0].lower() == 'show':
            self.tune_show()
        elif args[0].lower() == 'list':
            print("Options available:")
            for s in sorted(self.tune_limits()[3].keys()):
                print('  ' + s)
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return TuneoptModule(mpstate)
