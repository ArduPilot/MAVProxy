#!/usr/bin/env python
'''mode command handling'''

import time, os

def name():
    '''return module name'''
    return "mode"

def description():
    '''return module description'''
    return "mode handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.command_map['mode'] = (cmd_mode, "mode change")

def cmd_mode(args):
    '''set arbitrary mode'''
    mode_mapping = mpstate.master().mode_mapping()
    if mode_mapping is None:
        print('No mode mapping available')
        return
    if len(args) != 1:
        print('Available modes: ', mode_mapping.keys())
        return
    mode = args[0].upper()
    if mode not in mode_mapping:
        print('Unknown mode %s: ' % mode)
        return
    mpstate.master().set_mode(mode_mapping[mode])

