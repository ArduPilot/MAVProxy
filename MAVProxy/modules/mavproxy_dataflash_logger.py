#!/usr/bin/env python
'''
DataFlash Logging Module
June 2015

ArduPilot supports transmission of DataFlash logs over MavLink.

This module pokes the UAV to start sending logs, and stores them in a local directory.

The relevant code in the ArduPilot code base can be found in libraries/DataFlash/DataFlash_MAVLink.*
'''

import logging
import os
import os.path
import threading
import types
import sys
from pymavlink import mavutil
import errno

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
import time
from MAVProxy.modules.lib import mp_settings


class dataflash_logger(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module.  We start poking the UAV for messages after this is called"""
        super(dataflash_logger, self).__init__(mpstate, "dataflash_logger", "logging of mavlink dataflash messages")
        self.sender = None
        self.stopped = False
        self.time_last_start_packet_sent = 0
        self.time_last_stop_packet_sent = 0
        self.dataflash_dir = self._dataflash_dir(mpstate)

        self.log_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('df_target_system', int, 0),
              ('df_target_component', int, mavutil.mavlink.MAV_COMP_ID_LOG)
          ])
        self.add_command('dataflash_logger', self.cmd_dataflash_logger, "dataflash logging control", ['status','start','stop','set (LOGSETTING)'])
        self.add_completion_function('(LOGSETTING)', self.log_settings.completion)

    def usage(self):
        '''show help on a command line options'''
        return "Usage: dataflash_logger <status|start|stop|set>"

    def cmd_dataflash_logger(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "stop":
            self.sender = None
            self.stopped = True
        elif args[0] == "start":
            self.stopped = False
        elif args[0] == "set":
            self.log_settings.command(args[1:])
        else:
            print self.usage()

    def _dataflash_dir(self, mpstate):
        '''returns directory path to store DF logs in.  May be relative'''
        if mpstate.settings.state_basedir is None:
            ret = 'dataflash'
        else:
            ret = os.path.join(mpstate.settings.state_basedir,'dataflash')

        try:
            os.makedirs(ret)
        except OSError as e:
            if e.errno != errno.EEXIST:
                print("DFLogger: OSError making (%s): %s" % (ret, str(e)))
        except Exception as e:
            print("DFLogger: Unknown exception making (%s): %s" % (ret, str(e)))

        return ret

    def new_log_filepath(self):
        '''returns a filepath to a log which does not currently exist and is suitable for DF logging'''
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

        return os.path.join(self.dataflash_dir, '%u.BIN' % (log_cnt,));

    def start_new_log(self):
        '''open a new dataflash log, reset state'''
        filename = self.new_log_filepath()

        self.last_seqno = 0
        self.logfile = open(filename, 'w+b')
        print("DFLogger: logging started (%s)" % (filename))
        self.prev_cnt = 0
        self.download = 0
        self.prev_download = 0
        self.last_idle_status_printed_time = time.time()
        self.last_status_time = time.time()
        self.missing_blocks = {}
        self.acking_blocks = {}
        self.blocks_to_ack_and_nack = []
        self.missing_found = 0
        self.abandoned = 0
        self.dropped = 0

    def status(self):
        '''returns information about module'''
        transfered = self.download - self.prev_download
        now = time.time()
        interval = now - self.last_status_time
        self.last_status_time = now
        return("DFLogger: %(state)s Rate(%(interval)ds):%(rate).3fkB/s Block:%(block_cnt)d Missing:%(missing)d Fixed:%(fixed)d Abandoned:%(abandoned)d" %
              {"interval": interval,
               "rate": transfered/(interval*1000),
               "block_cnt": self.last_seqno,
               "missing": len(self.missing_blocks),
               "fixed": self.missing_found,
               "abandoned": self.abandoned,
               "state": "Inactive" if self.stopped else "Active"
           })

    def idle_print_status(self):
        '''print out statistics every 10 seconds from idle loop'''
        now = time.time()
        if (now - self.last_idle_status_printed_time) >= 10:
            print self.status()
            self.last_idle_status_printed_time = now
            self.prev_download = self.download

    def idle_send_acks_and_nacks(self):
        '''Send packets to UAV in idle loop'''
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
                mavstatus = mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_ACK
                (target_sys,target_comp) = self.sender
                self.master.mav.remote_log_block_status_send(target_sys,
                                                             target_comp,
                                                             block,
                                                             mavstatus)
                blocks_sent += 1
                del self.acking_blocks[block]
                del self.blocks_to_ack_and_nack[i]
                continue

            if block not in self.missing_blocks:
                # we've received this block now
                del self.blocks_to_ack_and_nack[i]
                continue

            # give up on packet if we have seen one with a much higher
            # number (or after 60 seconds):
            if (self.last_seqno - block > 200) or (now - first_sent > 60):
                if self.log_settings.verbose:
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

            if self.log_settings.verbose:
                print("DFLogger: Asking for block (%d)" % (block,))
            mavstatus = mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_NACK
            (target_sys,target_comp) = self.sender
            self.master.mav.remote_log_block_status_send(target_sys,
                                                         target_comp,
                                                         block,
                                                         mavstatus)
            blocks_sent += 1
            stuff[4] = now

    def idle_task_started(self):
        '''called in idle task only when logging is started'''
        if self.log_settings.verbose:
            self.idle_print_status()
        self.idle_send_acks_and_nacks()

    def idle_task_not_started(self):
        '''called in idle task only when logging is not running'''
        if not self.stopped:
            self.tell_sender_to_start()

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if self.sender is not None:
            self.idle_task_started()
        else:
            self.idle_task_not_started()

    def tell_sender_to_stop(self, m):
        '''send a stop packet (if we haven't sent one in the last second)'''
        now = time.time()
        if now - self.time_last_stop_packet_sent < 1:
            return
        if self.log_settings.verbose:
            print("DFLogger: Sending stop packet")
        self.time_last_stop_packet_sent = now
        self.master.mav.remote_log_block_status_send(m.get_srcSystem(),
                                                     m.get_srcComponent(),
                                                     mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_STOP,
                                                     1)
    def tell_sender_to_start(self):
        '''send a start packet (if we haven't sent one in the last second)'''
        now = time.time()
        if now - self.time_last_start_packet_sent < 1:
            return
        self.time_last_start_packet_sent = now

        if self.log_settings.verbose:
            print("DFLogger: Sending start packet")
       
        target_sys = self.log_settings.df_target_system
        target_comp = self.log_settings.df_target_component
        self.master.mav.remote_log_block_status_send(target_sys,
                                                     target_comp,
                                                     mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_START,
                                                     1)

    def packet_is_for_me(self, m):
        '''returns true if this packet is appropriately addressed'''
        if m.target_system != self.master.mav.srcSystem:
            return False
        if m.target_component != self.master.mav.srcComponent:
            return False
        # if have a sender we can also check the source address:
        if self.sender is not None:
            if (m.get_srcSystem(),m.get_srcComponent()) != self.sender:
                return False;
        return True

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'REMOTE_LOG_DATA_BLOCK':
            now = time.time()
            if not self.packet_is_for_me(m):
                dropped += 1
                return

            if self.sender is None and m.seqno == 0:
                if self.log_settings.verbose:
                    print("DFLogger: Received data packet - starting new log")
                self.start_new_log()
                self.sender = (m.get_srcSystem(), m.get_srcComponent())
    
            if self.sender is None:
                # No connection at the moment, and this packet did not start one
                return
    
            if self.stopped:
                # send a stop packet every second until the other end gets the idea:
                self.tell_sender_to_stop(m)
                return
    
            if self.sender is not None:
                size = len(m.data)
                data = ''.join(str(chr(x)) for x in m.data[:size])
                ofs = size*(m.seqno)
                self.logfile.seek(ofs)
                self.logfile.write(data)
    
                if m.seqno in self.missing_blocks:
                    if self.log_settings.verbose:
                        print("DFLogger: Received missing block: %d" % (m.seqno,))
                    del self.missing_blocks[m.seqno]
                    self.missing_found += 1
                    self.blocks_to_ack_and_nack.append([self.master,m.seqno,1,now,None])
                    self.acking_blocks[m.seqno] = 1
#                    print("DFLogger: missing blocks: %s" % (str(self.missing_blocks),))
                else:
                    # ACK the block we just got:
                    if m.seqno in self.acking_blocks:
                        # already acking this one; we probably sent
                        # multiple nacks and received this one
                        # multiple times
                        pass
                    else:
                        self.blocks_to_ack_and_nack.append([self.master,m.seqno,1,now,None])
                        self.acking_blocks[m.seqno] = 1
                        # NACK any blocks we haven't seen and should have:
                        if(m.seqno - self.last_seqno > 1):
                            for block in range(self.last_seqno+1, m.seqno):
                                if block not in self.missing_blocks and \
                                   block not in self.acking_blocks:
                                    self.missing_blocks[block] = 1
                                    if self.log_settings.verbose:
                                        print "DFLogger: setting %d for nacking" % (block,)
                                    self.blocks_to_ack_and_nack.append([self.master,block,0,now,None])
                        #print "\nmissed blocks: ",self.missing_blocks
                    if self.last_seqno < m.seqno:
                        self.last_seqno = m.seqno
                self.download += size

def init(mpstate):
    '''initialise module'''
    return dataflash_logger(mpstate)
