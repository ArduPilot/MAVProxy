import logging
import os
import os.path
import threading
import types
import sys
from pymavlink import mavutil
import random

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
import time
from MAVProxy.modules.lib import mp_settings


class dataflash_logger(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module."""
        super(dataflash_logger, self).__init__(mpstate, "dataflash_logger", "logging of mavlink dataflash messages")
        self.new_log_started = False
        self.stopped = False
        self.time_last_start_packet_sent = 0
        self.time_last_stop_packet_sent = 0
        self.dataflash_dir = self._dataflash_dir(mpstate)

        self.log_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('dataflash_logger', self.cmd_dataflash_logger, "dataflash logging control", ['start','stop','set (LOGSETTING)'])
        self.add_completion_function('(LOGSETTING)', self.log_settings.completion)

    def usage(self):
        return "Usage: dataflash_logger <start|stop|set>"

    def cmd_dataflash_logger(self, args):
        '''dataflash_logger cmd'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "stop":
            self.new_log_started = False
            self.stopped = True
        elif args[0] == "start":
            self.stopped = False
        elif args[0] == "set":
            self.log_settings.command(args[1:])
        else:
            print self.usage()

    def usage_set(self):
        return "Usage: dataflash_logger set <var> <value>"

    def _dataflash_dir(self, mpstate):
        if mpstate.settings.state_basedir is None:
            ret = 'dataflash'
        else:
            ret = os.path.join(mpstate.settings.state_basedir,'dataflash')

        try:
            os.makedirs(ret)
        except OSError as e:
            if e.errno != 17: # EEXIST
                print("DFLogger: OSError making (%s): %s" % (ret, str(e)))
        except Exception as e:
            print("DFLogger: Unknown exception making (%s): %s" % (ret, str(e)))

        return ret

    def new_log_filepath(self):
        lastlog_filename = os.path.join(self.dataflash_dir,'LASTLOG.TXT')
        if os.path.exists(lastlog_filename) and os.stat(lastlog_filename).st_size != 0:
            fh = open(lastlog_filename,'rb')
            log_cnt = int(fh.read()) + 1
            fh.close()
        else:
            log_cnt = 1

        self.lastlog_file = open(lastlog_filename,'w+b')
        self.lastlog_file.write(log_cnt.__str__())
        self.lastlog_file.close()

        return os.path.join(self.dataflash_dir, '%d.BIN' % (log_cnt,));

    def start_new_log(self):
        filename = self.new_log_filepath()

        self.block_cnt = 0
        self.logfile = open(filename, 'w+b')
        print("DFLogger: logging started (%s)" % (filename))
        self.prev_cnt = 0
        self.download = 0
        self.prev_download = 0
        self.start = time.time()
        self.missing_blocks = {}
        self.acking_blocks = {}
        self.blocks_to_ack_and_nack = []
        self.missing_found = 0
        self.abandoned = 0

    def idle_print_download_rate(self):
        now = time.time()
        if (now - self.start) >= 10:
            transfered = self.download - self.prev_download
            interval = now - self.start
            print("DFLogger: Rate(%(interval)ds):%(rate).3fkB/s Block:%(block_cnt)d Missing:%(missing)d Fixed:%(fixed)d Abandoned:%(abandoned)d" %
                  {"interval": interval,
                   "rate": transfered/(interval*1000),
                   "block_cnt": self.block_cnt,
                   "missing": len(self.missing_blocks),
                   "fixed": self.missing_found,
                   "abandoned": self.abandoned
                   }
               )
            self.start = now
            self.prev_download = self.download

    def idle_send_acks_and_nacks(self):
        max_blocks_to_send = 10
        blocks_sent = 0
        i=0
        now = time.time()
        while i < len(self.blocks_to_ack_and_nack) and blocks_sent < max_blocks_to_send:
#            print("ACKLIST: %s" % ([x[1] for x in self.blocks_to_ack_and_nack],))
            stuff = self.blocks_to_ack_and_nack[i]

            [master, block, status, first_sent, last_sent] = stuff
            if status == 1:
#                print("DFLogger: ACKing block (%d)" % (block,))
                self.master.mav.remote_log_block_status_send(block,status)
                blocks_sent += 1
                del self.acking_blocks[block]
                del self.blocks_to_ack_and_nack[i]
                continue

            if block not in self.missing_blocks:
                # we've received this block now
                del self.blocks_to_ack_and_nack[i]
                continue

            # give up on packet if we have seen one with a much higher
            # number:
            if self.block_cnt - block > 200 or \
               now - first_sent > 60:
                print("DFLogger: Abandoning block (%d)" % (block,))
                del self.blocks_to_ack_and_nack[i]
                del self.missing_blocks[block]
                self.abandoned += 1
                continue

            i += 1
            # only send each nack every-so-often:
            if last_sent is not None:
                if now - last_sent < 0.1:
                    continue

            print("DFLogger: NACKing block (%d)" % (block,))
            self.master.mav.remote_log_block_status_send(block,status)
            blocks_sent += 1
            stuff[4] = now

    def idle_task_started(self):
        max_blocks = 5 # limit the number of blocks we process in any idle loop
        if self.log_settings.verbose:
            self.idle_print_download_rate()
        self.idle_send_acks_and_nacks()

    def idle_task(self):
        if self.new_log_started == True:
            self.idle_task_started()

    def mavlink_packet(self, m):
        now = time.time()
        if m.get_type() == 'REMOTE_LOG_DATA_BLOCK':
            if self.stopped:
                # send a stop packet every second until the other end gets the idea:
                if now - self.time_last_stop_packet_sent > 1:
                    if self.log_settings.verbose:
                        print("DFLogger: Sending stop packet")
                    self.master.mav.remote_log_block_status_send(4294967294,1)
                return

#            if random.random() < 0.1: # drop 1 packet in 10
#                return


            if not self.new_log_started:
                if self.log_settings.verbose:
                    print("Received data packet - starting new log")
                self.start_new_log()
                self.new_log_started = True
            if self.new_log_started == True:
                size = m.block_size
                data = ''.join(str(chr(x)) for x in m.data[:size])
                ofs = size*(m.block_cnt)
                self.logfile.seek(ofs)
                self.logfile.write(data)

                if m.block_cnt in self.missing_blocks:
                    if self.log_settings.verbose:
                        print("DFLogger: Received missing block: %d" % (m.block_cnt,))
                    del self.missing_blocks[m.block_cnt]
                    self.missing_found += 1
                    self.blocks_to_ack_and_nack.append([self.master,m.block_cnt,1,now,None])
                    self.acking_blocks[m.block_cnt] = 1
#                    print("DFLogger: missing blocks: %s" % (str(self.missing_blocks),))
                else:
                    # ACK the block we just got:
                    if m.block_cnt in self.acking_blocks:
                        # already acking this one; we probably sent
                        # multiple nacks and received this one
                        # multiple times
                        pass
                    else:
                        self.blocks_to_ack_and_nack.append([self.master,m.block_cnt,1,now,None])
                        self.acking_blocks[m.block_cnt] = 1
                        # NACK any blocks we haven't seen and should have:
                        if(m.block_cnt - self.block_cnt > 1):
                            for block in range(self.block_cnt+1, m.block_cnt):
                                if block not in self.missing_blocks and \
                                   block not in self.acking_blocks:
                                    self.missing_blocks[block] = 1
                                    if self.log_settings.verbose:
                                        print "DFLogger: setting %d for nacking" % (block,)
                                    self.blocks_to_ack_and_nack.append([self.master,block,0,now,None])
                        #print "\nmissed blocks: ",self.missing_blocks
                    if self.block_cnt < m.block_cnt:
                        self.block_cnt = m.block_cnt
                self.download += size
        elif not self.new_log_started and not self.stopped:
            # send a start packet every second until the other end gets the idea:
            if now - self.time_last_start_packet_sent > 1:
                if self.log_settings.verbose:
                    print("DFLogger: Sending start packet")
                self.master.mav.remote_log_block_status_send(4294967295,1)
                self.time_last_start_packet_sent = now

def init(mpstate):
    '''initialise module'''
    return dataflash_logger(mpstate)
