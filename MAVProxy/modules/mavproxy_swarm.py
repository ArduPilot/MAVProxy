#!/usr/bin/env python

'''
Swarming support
Stephen Dade, July 2021

Helper functions for managing leader-follower swarms

'''

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class swarm(mp_module.MPModule):
    def __init__(self, mpstate, multi_vehicle=True):
        """Initialise module"""
        super(swarm, self).__init__(mpstate, "swarm", "swarm module")

        self.swarm_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ('leader', int, 1),
             ('cmddelay', int, 1),
            ]
        )
        self.add_command('swarm',
                         self.cmd_swarm,
                         "swarm control",
                         ['set (SWARMSETTING)',
                          'armfollowers',
                          'armall',
                          'disarmfollowers',
                          'disarmall',
                          'modefollowers',
                          'modeall'
                         ])
        
    def usage(self):
        '''show help on command line options'''
        return "Usage: swarm <set|armfollowers|armall|disarmfollowers|disarmall|modefollowers|modeall>"

    def cmd_send(self, doLeader, doFollowers, args):
        '''send command to leader and/or followers'''
        saved_target = self.mpstate.settings.target_system
        linkmod = self.module('link')
        
        for v in sorted(self.mpstate.vehicle_list):
            if doLeader and int(v) == self.swarm_settings.leader:
                linkmod.cmd_vehicle([str(v)])
                self.mpstate.functions.process_stdin(' '.join(args), True)
                time.sleep(self.swarm_settings.cmddelay/1000)
            elif doFollowers and int(v) != self.swarm_settings.leader:
                linkmod.cmd_vehicle([str(v)])
                self.mpstate.functions.process_stdin(' '.join(args), True)
                time.sleep(self.swarm_settings.cmddelay/1000)
        linkmod.cmd_vehicle([str(saved_target)])

    def cmd_swarm(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "set":
            self.swarm_settings.command(args[1:])
        elif args[0] == "armfollowers":
            # arm all the followers
            self.cmd_send(False, True, ["arm", "throttle"])
        elif args[0] == "armall":
            # arm all
            self.cmd_send(True, True, ["arm", "throttle"])
        elif args[0] == "disarmfollowers":
            # disarm all the followers
            self.cmd_send(False, True, ["disarm"])
        elif args[0] == "disarmall":
            # disarm all
            self.cmd_send(True, True, ["disarm"])
        elif args[0] == "modefollowers" and len(args) == 2:
            # set mode for followers
            self.cmd_send(False, True, ["mode"] + [args[1]])
        elif args[0] == "modefollowers":
            print("Usage: swarm modefollowers MODE")
        elif args[0] == "modeall"and len(args) == 2:
            # set mode for all
            self.cmd_send(True, True, ["mode"] + [args[1]])
        elif args[0] == "modeall":
            print("Usage: swarm modeall MODE")
                                    
    def idle_task(self):
        '''called rapidly by mavproxy'''

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        pass

def init(mpstate):
    '''initialise module'''
    return swarm(mpstate)
