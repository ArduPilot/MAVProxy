#!/usr/bin/env python
''' Air Traffic Control module for MAVProxy '''
import api

"""
class reasoner_state():
    ''' 'spose I'm gonna need it? '''
    def __init__(self):
        self.knowledge_engine = None
"""

def name():
    '''return module name'''
    return "ATC"

def description():
    '''return module description'''
    return "air traffic control - a logic programming framework for MAVProxy"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    #mkstate.reasoner = reasoner_state()
    mkstate.command_map['preflight'] = (api.preflight, 'run pre-flight checks')

def unload():
    '''unload module'''
    pass

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    pass
