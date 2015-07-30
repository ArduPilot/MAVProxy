#!/usr/bin/env python
'''log command handling'''

import time, os

from MAVProxy.modules.lib import mp_module

class LogModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(LogModule, self).__init__(mpstate, "log", "log transfer")
        self.add_command('log', self.cmd_log, "log file handling", ['<download|status|erase|resume|cancel|list>'])
        self.reset()

    def reset(self):
        self.download_set = set()
        self.download_file = None
        self.download_lognum = None
        self.download_filename = None
        self.download_start = None
        self.download_last_timestamp = None
        self.download_ofs = 0
        self.retries = 0
        self.entries = {}

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'LOG_ENTRY':
            self.handle_log_entry(m)
        elif m.get_type() == 'LOG_DATA':
            self.handle_log_data(m)

    def handle_log_entry(self, m):
        '''handling incoming log entry'''
        if m.time_utc == 0:
            tstring = ''
        else:
            tstring = time.ctime(m.time_utc)
        self.entries[m.id] = m
        print("Log %u  numLogs %u lastLog %u size %u %s" % (m.id, m.num_logs, m.last_log_num, m.size, tstring))


    def handle_log_data(self, m):
        '''handling incoming log data'''
        if self.download_file is None:
            return
        # lose some data
        # import random
        # if random.uniform(0,1) < 0.05:
        #    print('dropping ', str(m))
        #    return
        if m.ofs != self.download_ofs:
            self.download_file.seek(m.ofs)
            self.download_ofs = m.ofs
        if m.count != 0:
            data = m.data[:m.count]
            s = ''.join(str(chr(x)) for x in data)
            self.download_file.write(s)
            self.download_set.add(m.ofs // 90)
            self.download_ofs += m.count
        self.download_last_timestamp = time.time()
        if m.count == 0 or (m.count < 90 and len(self.download_set) == 1 + (m.ofs // 90)):
            dt = time.time() - self.download_start
            self.download_file.close()
            size = os.path.getsize(self.download_filename)
            speed = size / (1000.0 * dt)
            print("Finished downloading %s (%u bytes %u seconds, %.1f kbyte/sec %u retries)" % (
                self.download_filename,
                size,
                dt, speed,
                self.retries))
            self.download_file = None
            self.download_filename = None
            self.download_set = set()

    def handle_log_data_missing(self):
        '''handling missing incoming log data'''
        if len(self.download_set) == 0:
            return
        highest = max(self.download_set)
        diff = set(range(highest)).difference(self.download_set)
        if len(diff) == 0:
            self.master.mav.log_request_data_send(self.target_system,
                                                       self.target_component,
                                                       self.download_lognum, (1 + highest) * 90, 0xffffffff)
            self.retries += 1
        else:
            num_requests = 0
            while num_requests < 20:
                start = min(diff)
                diff.remove(start)
                end = start
                while end + 1 in diff:
                    end += 1
                    diff.remove(end)
                self.master.mav.log_request_data_send(self.target_system,
                                                           self.target_component,
                                                           self.download_lognum, start * 90, (end + 1 - start) * 90)
                num_requests += 1
                self.retries += 1
                if len(diff) == 0:
                    break


    def log_status(self):
        '''show download status'''
        if self.download_filename is None:
            print("No download")
            return
        dt = time.time() - self.download_start
        speed = os.path.getsize(self.download_filename) / (1000.0 * dt)
        m = self.entries.get(self.download_lognum, None)
        if m is None:
            size = 0
        else:
            size = m.size
        highest = max(self.download_set)
        diff = set(range(highest)).difference(self.download_set)
        print("Downloading %s - %u/%u bytes %.1f kbyte/s (%u retries %u missing)" % (self.download_filename,
                                                                                     os.path.getsize(self.download_filename),
                                                                                     size,
                                                                                     speed,
                                                                                     self.retries,
                                                                                     len(diff)))

    def log_download(self, log_num, filename):
        '''download a log file'''
        print("Downloading log %u as %s" % (log_num, filename))
        self.download_lognum = log_num
        self.download_file = open(filename, "wb")
        self.master.mav.log_request_data_send(self.target_system,
                                                   self.target_component,
                                                   log_num, 0, 0xFFFFFFFF)
        self.download_filename = filename
        self.download_set = set()
        self.download_start = time.time()
        self.download_last_timestamp = time.time()
        self.download_ofs = 0
        self.retries = 0

    def cmd_log(self, args):
        '''log commands'''
        if len(args) < 1:
            print("usage: log <list|download|erase|resume|status|cancel>")
            return

        if args[0] == "status":
            self.log_status()
        if args[0] == "list":
            print("Requesting log list")
            self.download_set = set()
            self.master.mav.log_request_list_send(self.target_system,
                                                       self.target_component,
                                                       0, 0xffff)

        elif args[0] == "erase":
            self.master.mav.log_erase_send(self.target_system,
                                                self.target_component)

        elif args[0] == "resume":
            self.master.mav.log_request_end_send(self.target_system,
                                                      self.target_component)

        elif args[0] == "cancel":
            if self.download_file is not None:
                self.download_file.close()
            self.reset()

        elif args[0] == "download":
            if len(args) < 2:
                print("usage: log download <lognumber> <filename>")
                return
            if args[1] == 'latest':
                if len(self.entries.keys()) == 0:
                    print("Please use log list first")
                    return
                log_num = sorted(self.entries, key=lambda id: self.entries[id].time_utc)[-1]
            else:
                log_num = int(args[1])
            if len(args) > 2:
                filename = args[2]
            else:
                filename = "log%u.bin" % log_num
            self.log_download(log_num, filename)

    def idle_task(self):
        '''handle missing log data'''
        if self.download_last_timestamp is not None and time.time() - self.download_last_timestamp > 0.7:
            self.download_last_timestamp = time.time()
            self.handle_log_data_missing()

def init(mpstate):
    '''initialise module'''
    return LogModule(mpstate)
