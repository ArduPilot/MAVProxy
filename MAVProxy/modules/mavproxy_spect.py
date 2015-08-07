"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Copyright 2015 3D Robotics, Inc

Author: Jonathan Challinger
"""

from pymavlink import mavutil
import re, os, sys
import time
from string import atof, atoi

from MAVProxy.modules.lib import livespec

from MAVProxy.modules.lib import mp_module

class SpectModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpectModule, self).__init__(mpstate, "spect", "spect control")
        self.add_command('spect', self.cmd_spect, "<freq> <fres> <tres> <msg> <field> [maxscale] add a live spectrogram")
        self.msgs = []
        self.spectrograms = []

    def cmd_spect(self, args):
        '''spect command'''
        if len(args) != 5 and len(args) != 6:
            print("spect <freq> <fres> <tres> <msg> <field> [maxscale]")
            return

        self.spectrograms.append(livespec.LiveSpectrogram(atof(args[0]),atof(args[1]),atof(args[2]),60.0,atof(args[5]) if len(args)==6 else None))
        self.msgs.append((args[3], args[4]))

    def unload(self):
        '''unload module'''
        for g in self.spectrograms:
            g.close()
        self.spectrograms = []
        self.msgs = []

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''

        # check for any closed spectrograms
        if msg.get_type() == "HEARTBEAT":
            i = 0
            while i < len(self.spectrograms):
                if not self.spectrograms[i].is_alive():
                    self.spectrograms[i].close()
                    self.spectrograms.pop(i)
                    self.msgs.pop(i)
                else:
                    i+=1

        # add data to the rest
        for i in range(len(self.spectrograms)):
            if msg.get_type() == self.msgs[i][0]:
                if self.msgs[i][0] == 'DEBUG':
                    if msg.to_dict()['ind'] == atoi(self.msgs[i][1]):
                        self.spectrograms[i].new_sample(msg.to_dict()['value'])
                else:
                    self.spectrograms[i].new_sample(msg.to_dict()[self.msgs[i][1]])


def init(mpstate):
    '''initialise module'''
    return SpectModule(mpstate)
