#!/usr/bin/env python

'''
show stats on messages in a log in MAVExplorer
'''

import fnmatch
from MAVProxy.modules.lib.multiproc_util import MPDataLogChildTask

categories = {
    'EKF2' : ['NK*'],
    'EKF3' : ['XK*'],
    'SENSORS' : ['IMU*', 'BAR*', 'GPS*', 'RFND', 'ACC', 'GYR' ],
    'RC' : ['RCIN', 'RCOU' ],
    'TUNING' : ['RATE', 'PID*', 'CTUN', 'NTUN', 'ATT', 'PSC'],
    'SYSTEM' : ['MAV', 'BAT*', 'EV', 'CMD', 'MODE'],
    'REPLAY' : [ 'RFRH', 'RFRF', 'REV2', 'RSO2', 'RWA2', 'REV3', 'RSO3', 'RWA3', 'RMGI',
                 'REY3', 'RFRN', 'RISH', 'RISI', 'RISJ', 'RBRH', 'RBRI', 'RRNH', 'RRNI',
                 'RGPH', 'RGPI', 'RGPJ', 'RASH', 'RASI', 'RBCH', 'RBCI', 'RVOH', 'RMGH',
                 'R??H', 'R??I', 'R??J'],
}

class MPMsgStats(MPDataLogChildTask):
    '''A class used launch `show_stats` in a child process'''

    def __init__(self, *args, **kwargs):
        '''
        Parameters
        ----------
        mlog : DFReader
            A dataflash or telemetry log
        '''

        super(MPMsgStats, self).__init__(*args, **kwargs)

    # @override
    def child_task(self):
        '''Launch `show_stats`'''

        # run the fft tool
        show_stats(self.mlog)

def show_stats(mlog):
    '''show stats on a file'''
    if not hasattr(mlog, 'formats'):
        print("Must be DF log")
        return
    sizes = {}
    total_size = 0
    names = mlog.name_to_id.keys()
    pairs = []

    for name in names:
        sizes[name] = 0

    for name in names:
        mid = mlog.name_to_id[name]
        count = mlog.counts[mid]
        mlen = mlog.formats[mid].len
        size = count * mlen
        total_size += size
        sizes[name] += size
        pairs.append((name, count*mlen))

    pairs = sorted(pairs, key = lambda p : p[1])
    print("Total size: %u" % total_size)
    for (name,size) in pairs:
        if size > 0:
            print("%-4s %.2f%%" % (name, 100.0 * size / total_size))

    print("")
    category_total = 0
    for c in categories.keys():
        total = 0
        for name in names:
            for p in categories[c]:
                if fnmatch.fnmatch(name, p):
                    total += sizes[name]
                    break
        category_total += total
        if total > 0:
            print("@%s %.2f%%" % (c, 100.0 * total / total_size))
    print("@OTHER %.2f%%" % (100.0 * (total_size-category_total) / total_size))
