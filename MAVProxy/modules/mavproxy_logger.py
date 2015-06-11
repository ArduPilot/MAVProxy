import logging
import os
import os.path
import threading
import types
import sys
from pymavlink import mavutil

from time import sleep
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
import time


class logger(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module."""
        super(logger, self).__init__(mpstate, "logger", "log")
        self.block_cnt = 1
        lastlog_filename = '/log/dataflash/LASTLOG.TXT'
        dir = os.path.dirname(lastlog_filename)
        if not os.path.exists(dir):
            while True:
                try:
                    os.makedirs(dir)
                except:
                    continue
                break
        if os.path.isfile(lastlog_filename) is False:
            self.lastlog_file = open(lastlog_filename,'w+b')

        if os.stat(lastlog_filename).st_size == 0:
            log_cnt = 1
            self.lastlog_file.write(log_cnt.__str__())
            self.lastlog_file.close()

        else:
            self.lastlog_file = open(lastlog_filename,'rb')
            log_cnt = int(self.lastlog_file.read()) + 1
            self.lastlog_file.close()
            self.lastlog_file = open(lastlog_filename,'wb')
            self.lastlog_file.truncate()
            self.lastlog_file.write(log_cnt.__str__())
            self.lastlog_file.close()

        filename = '/log/dataflash/log' + log_cnt.__str__() + '.bin'


        while True:
            try:
                self.logfile = open(filename, 'w+b')
            except:
                continue
            break
        self.logfile.truncate()
        print "Logging Started!!"
        self.prev_cnt = 0
        self.download = 0
        self.prev_download = 0
        self.start = time.time()
        self.missing_blocks = []
        self.new_log_started = False
        self.max_block_cnt = 0

    def idle_task(self):
        end = time.time()
        if self.new_log_started == False :
            self.master.mav.remote_log_block_status_send(0,1)
            sleep(1)
        else:
            self.master.mav.remote_log_block_status_send(self.block_cnt,1)
        if (end - self.start) >= 10:
            print "\nLog Download Rate:  ",(self.download - self.prev_download)/((end - self.start)*1000), "Kb/s "
            self.start = time.time()
            self.prev_download = self.download
        for missed_block in self.missing_blocks:
            print "\nRequesting Missed Block!!", self.missing_blocks
            self.master.mav.remote_log_block_status_send(missed_block,0)

    def mavlink_packet(self, m):
        if m.get_type() == 'REMOTE_LOG_DATA_BLOCK':
            if m.block_cnt == 1:
                self.new_log_started = True
            if self.new_log_started == True:
                size = m.block_size
                data = ''.join(str(chr(x)) for x in m.data[:size])
                ofs = size*(m.block_cnt - 1)
                self.logfile.seek(ofs)
                self.logfile.write(data)

                if m.block_cnt in self.missing_blocks:
                    print "\nremoved block: ",m.block_cnt
                    print "\nmissed blocks: ",self.missing_blocks
                    self.missing_blocks.remove(m.block_cnt)
                    print "\nmissed blocks: ",self.missing_blocks
                else:
                    if(m.block_cnt - self.max_block_cnt > 1): 
                        for blocks in range(self.block_cnt+1, m.block_cnt):
                            if blocks not in self.missing_blocks:
                                self.missing_blocks.append(blocks)
                        #print "\nmissed blocks: ",self.missing_blocks
                    self.max_block_cnt=m.block_cnt
                self.block_cnt = m.block_cnt
                self.download+=size
def init(mpstate):
    '''initialise module'''
    return logger(mpstate)
