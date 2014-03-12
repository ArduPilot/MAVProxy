#!/usr/bin/env python
'''arm/disarm command handling'''

import time, os

def name():
    '''return module name'''
    return "arm"

def description():
    '''return module description'''
    return "arm/disarm handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.command_map['arm'] =    (cmd_arm,      'arm motors')
    mpstate.command_map['disarm'] = (cmd_disarm,   'disarm motors')
    mpstate.completions['arm'] = ['check <all|baro|compass|gps|ins|params|rc|voltage|battery>',
                                  'uncheck <all|baro|compass|gps|ins|params|rc|voltage|battery>',
                                  'list',
                                  'throttle']

arming_masks = {
    "all"     : 0x0001,
    "baro"    : 0x0002,
    "compass" : 0x0004,
    "gps"     : 0x0008,
    "ins"     : 0x0010,
    "params"  : 0x0020,
    "rc"      : 0x0040,
    "voltage" : 0x0080,
    "battery" : 0x0100
    }


def cmd_arm(args):
    '''arm commands'''
    usage = "usage: arm <check|uncheck|list|throttle>"
    checkables = "<all|baro|compass|gps|ins|params|rc|voltage|battery>"

    if len(args) <= 0:
        print(usage)
        return
        
    if args[0] == "check":
        if (len(args) < 2):
            print "usage: arm check", checkables
            return            
        
        arming_mask = int(mpstate.functions.get_mav_param("ARMING_CHECK",0))
        name = args[1].lower()
        if name == 'all':
            for name in arming_masks.keys():
                arming_mask |= arming_masks[name]
        elif name in arming_masks:
            arming_mask |= arming_masks[name]
        else:
            print("unrecognized arm check:", name)
            return
        mpstate.functions.param_set("ARMING_CHECK", arming_mask)
        return
    
    if args[0] == "uncheck":
        if (len(args) < 2):
            print "usage: arm uncheck", checkables
            return

        arming_mask = int(mpstate.functions.get_mav_param("ARMING_CHECK",0))
        name = args[1].lower()
        if name == 'all':
            arming_mask = 0
        elif name in arming_masks:
            arming_mask &= ~arming_masks[name]
        else:
            print "unrecognized arm check:", args[1]
            return

        mpstate.functions.param_set("ARMING_CHECK", arming_mask)
        return

    if args[0] == "list":
            arming_mask = int(mpstate.functions.get_mav_param("ARMING_CHECK",0))
            if arming_mask == 0:
                print("NONE")
            for name in arming_masks.keys():
                if arming_masks[name] & arming_mask:
                    print name
            return

    if args[0] == "throttle":
        mpstate.master().arducopter_arm()
        return

    print(usage)

def cmd_disarm(args):
  '''disarm motors'''
  mpstate.master().arducopter_disarm()
