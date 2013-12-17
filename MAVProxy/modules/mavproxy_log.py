#!/usr/bin/env python
'''log command handling'''

import time, os

class log_state(object):
    def __init__(self):
        self.download_set = set()
        self.download_file = None
        self.download_lognum = None
        self.download_filename = None
        self.download_start = None
        self.download_last_timestamp = None

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.log_state = log_state()
    mpstate.command_map['log'] = (cmd_log, "log file handling")

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    if m.get_type() == 'LOG_ENTRY':
        handle_log_entry(m)
    elif m.get_type() == 'LOG_DATA':
        handle_log_data(m)

def handle_log_entry(m):
    '''handling incoming log entry'''
    if m.time_utc == 0:
        tstring = ''
    else:
        tstring = time.ctime(m.time_utc)
    print("Log %u  numLogs %u lastLog %u size %u %s" % (m.id, m.num_logs, m.last_log_num, m.size, tstring))


def handle_log_data(m):
    '''handling incoming log data'''
    state = mpstate.log_state
    if state.download_file is None:
        return
    # lose some data
    #import random
    #if random.uniform(0,1) < 0.05:
    #    print('dropping ', str(m))
    #    return
    state.download_file.seek(m.ofs)
    if m.count != 0:
        state.download_file.write(m.data[:m.count])
        state.download_set.add(m.ofs // 90)
    state.download_last_timestamp = time.time()
    if m.count == 0 or (m.count < 90 and len(state.download_set) == 1 + (m.ofs // 90)):
        dt = time.time() - state.download_start
        speed = os.path.getsize(state.download_filename) / (1000.0 * dt)
        print("Finished downloading %s (%u seconds, %.1f kbyte/sec)" % (state.download_filename,
                                                                        dt, speed))
        state.download_file.close()
        state.download_file = None
        state.download_filename = None
        state.download_set = set()

def handle_log_data_missing():
    '''handling missing incoming log data'''
    state = mpstate.log_state
    if len(state.download_set) == 0:
        return
    highest = max(state.download_set)
    diff = set(range(highest)).difference(state.download_set)
    if len(diff) == 0:
        mpstate.master().mav.log_request_data_send(mpstate.status.target_system,
                                                   mpstate.status.target_component,
                                                   state.download_lognum, (1+highest)*90, 0xffffffff)
    else:
        num_requests = 0
        while num_requests < 10:
            start = min(diff)
            diff.remove(start)
            end = start
            while end+1 in diff:
                end += 1
                diff.remove(end)
            mpstate.master().mav.log_request_data_send(mpstate.status.target_system,
                                                       mpstate.status.target_component,
                                                       state.download_lognum, start*90, (end+1-start)*90)
            num_requests += 1
            if end != start or len(diff) == 0:
                break
    

def cmd_log(args):
    '''log commands'''
    state = mpstate.log_state
    if len(args) < 1:
        print("usage: log <list|download|erase|resume|status>")
        return

    if args[0] == "status":
        if state.download_filename is None:
            print("No download")
        else:
            dt = time.time() - state.download_start
            speed = os.path.getsize(state.download_filename) / (1000.0 * dt)
            print("Downloading %s - %u bytes %.1f kbyte/s" % (state.download_filename,
                                                              os.path.getsize(state.download_filename),
                                                              speed))

    if args[0] == "list":
        print("Requesting log list")
        state.download_set = set()
        mpstate.master().mav.log_request_list_send(mpstate.status.target_system,
                                                   mpstate.status.target_component,
                                                   0, 0xffff)

    elif args[0] == "erase":
        mpstate.master().mav.log_erase_send(mpstate.status.target_system,
                                            mpstate.status.target_component)
    elif args[0] == "download":
        if len(args) != 2:
            print("usage: log download <lognumber>")
            return
        log_num = int(args[1])
        filename = "log%u.bin" % log_num
        print("Downloading log %u as %s" % (log_num, filename))
        state.download_lognum = log_num
        state.download_file = open(filename, "wb")
        mpstate.master().mav.log_request_data_send(mpstate.status.target_system,
                                                   mpstate.status.target_component,
                                                   log_num, 0, 0xFFFFFFFF)
        state.download_filename = filename
        state.download_set = set()
        state.download_start = time.time()
        state.download_last_timestamp = time.time()

def idle_task():
    '''handle missing log data'''
    state = mpstate.log_state
    if state.download_last_timestamp is not None and time.time() - state.download_last_timestamp > 0.5:
        handle_log_data_missing()
