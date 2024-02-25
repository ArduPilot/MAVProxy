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
    sizes = {}
    total_size = 0
    names = mlog.name_to_id.keys()
    pairs = []
    maxnamelen = 4

    for name in names:
        sizes[name] = 0

    for name in names:
        mid = mlog.name_to_id[name]
        # DFReader_text logs use the name directly as the index to the dictionaries
        if hasattr(mlog, 'formats') and mid not in mlog.formats:
            mid = name
        # In DFReader_text logs count is a dictionary, which is not set for
        # messages that are never seen in the log.
        if isinstance(mlog.counts,dict) and mid not in mlog.counts:
            count = 0
        else:
            count = mlog.counts[mid]
        # mavmmaplog class (tlogs) does not contain formats attribute, so instead of
        # counting size in bytes, we count the number of messages
        if hasattr(mlog, 'formats'):
            mlen = mlog.formats[mid].len
            size = count * mlen
            total_size += size
            sizes[name] += size
            pairs.append((name, count*mlen))
        else:
            total_size += count
            pairs.append((name, count))
            if count>0 and len(name)>maxnamelen:
                maxnamelen = len(name)

    # mavmmaplog class (tlogs) does not contain formats attribute, so instead of
    # counting size in bytes, we count the number of messages
    if not hasattr(mlog, 'formats'):
        print("Total number of messages: %u" % total_size)
    else:
        print("Total size: %u" % total_size)

    # Print out the percentage for each message, from lowest to highest
    pairs = sorted(pairs, key = lambda p : p[1])
    for (name,size) in pairs:
        if size > 0:
            descstr = ''
            if hasattr(mlog, 'metadata'):
                desc = mlog.metadata.get_description(name)
                if desc:
                    if len(desc) > (68 - maxnamelen):
                        descstr = "  [%s...]" % desc[:(65 - maxnamelen)]
                    else:
                        descstr = "  [%s]" % desc
            print("%-*s %5.2f%%%s" % (maxnamelen, name, 100.0 * size / total_size, descstr))

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
