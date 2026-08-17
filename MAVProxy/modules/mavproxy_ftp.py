#!/usr/bin/env python3
'''mavlink file transfer support'''

import glob
import os
import sys
import time
import struct
import random
import zlib
import heapq

try:
    # py2
    from StringIO import StringIO as SIO
except ImportError:
    # py3
    from io import BytesIO as SIO

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

# opcodes
OP_None = 0
OP_TerminateSession = 1
OP_ResetSessions = 2
OP_ListDirectory = 3
OP_OpenFileRO = 4
OP_ReadFile = 5
OP_CreateFile = 6
OP_WriteFile = 7
OP_RemoveFile = 8
OP_CreateDirectory = 9
OP_RemoveDirectory = 10
OP_OpenFileWO = 11
OP_TruncateFile = 12
OP_Rename = 13
OP_CalcFileCRC32 = 14
OP_BurstReadFile = 15
OP_Ack = 128
OP_Nack = 129

# error codes
ERR_None = 0
ERR_Fail = 1
ERR_FailErrno = 2
ERR_InvalidDataSize = 3
ERR_InvalidSession = 4
ERR_NoSessionsAvailable = 5
ERR_EndOfFile = 6
ERR_UnknownCommand = 7
ERR_FileExists = 8
ERR_FileProtected = 9
ERR_FileNotFound = 10

HDR_Len = 12
MAX_Payload = 239
WRITE_CAPABILITY_MAGIC = 0xA5

# the server null terminates the last byte of its name buffer, so a name
# filling the payload exactly would be silently truncated
MAX_FTP_NAME = MAX_Payload - 1

class FTP_OP:
    def __init__(self, seq, session, opcode, size, req_opcode, burst_complete, offset, payload):
        self.seq = seq
        self.session = session
        self.opcode = opcode
        self.size = size
        self.req_opcode = req_opcode
        self.burst_complete = burst_complete
        self.offset = offset
        self.payload = payload

    def pack(self):
        '''pack message'''
        ret = struct.pack("<HBBBBBBI", self.seq, self.session, self.opcode, self.size, self.req_opcode, self.burst_complete, 0, self.offset)
        if self.payload is not None:
            ret += self.payload
        ret = bytearray(ret)
        return ret

    def __str__(self):
        plen = 0
        if self.payload is not None:
            plen = len(self.payload)
        ret = "OP seq:%u sess:%u opcode:%d req_opcode:%u size:%u bc:%u ofs:%u plen=%u" % (self.seq,
                                                                                          self.session,
                                                                                          self.opcode,
                                                                                          self.req_opcode,
                                                                                          self.size,
                                                                                          self.burst_complete,
                                                                                          self.offset,
                                                                                          plen)
        if plen > 0:
            ret += " [%u]" % self.payload[0]
        return ret

class WriteQueue:
    def __init__(self, ofs, size):
        self.ofs = ofs
        self.size = size
        self.last_send = 0


class MAVLinkBatchWriter:
    '''Collect MAVLink packets so several FTP requests use one link write.'''
    def __init__(self):
        self.packets = []

    def write(self, packet):
        self.packets.append(bytes(packet))
        return len(packet)


class FTPWorker(mp_module.MPModule):
    '''State for one FTP operation/session.

    A worker deliberately isn't registered as a public MAVProxy module.  The
    public FTPModule owns and routes to several of these at once.
    '''
    def __init__(self, manager, session):
        super(FTPWorker, self).__init__(manager.mpstate, "ftp_worker")
        self.manager = manager
        self.ftp_settings = manager.ftp_settings
        self.seq = 0
        self.session = session
        self.network = 0
        self.last_op = None
        self.fh = None
        self.filename = None
        self.callback = None
        self.callback_progress = None
        self.put_callback = None
        self.put_callback_progress = None
        self.total_size = 0
        self.read_gaps = []
        self.read_gap_times = {}
        self.last_gap_send = 0
        self.read_retries = 0
        self.read_total = 0
        self.duplicates = 0
        self.last_read = None
        self.last_burst_read = None
        self.op_start = None
        self.dir_offset = 0
        self.last_op_time = time.time()
        self.rtt = 0.5
        self.rttvar = 0.25
        self.rtt_valid = False
        self.send_times = {}
        self.reached_eof = False
        self.backlog = 0
        self.burst_size = self.ftp_settings.burst_read_size
        self.write_list = None
        self.write_block_size = 0
        self.write_acks = 0
        self.write_acked_bytes = 0
        self.write_total = 0
        self.write_file_size = 0
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_inflight = set()
        self.write_last_send = None
        self.write_open = False
        self.write_qsize = (self.ftp_settings.write_qsize
                            if self.ftp_settings.write_qsize > 0 else 5)
        self.write_batch_size = (self.ftp_settings.write_batch_size
                                 if self.ftp_settings.write_batch_size > 0 else 1)
        self.warned_component = False
        # console progress is only for interactive ftp get/put, not for the
        # callback-driven transfers behind "param ftp" and "wp ftp"
        self.show_progress = False
        self.last_status_time = 0
        self.remote_file_size = None
        # a get or put is running, including the open handshake before any
        # file handle exists
        self.transfer_active = False
        self.crccmp_dest = None
        self.crccmp_pending = []
        self.crccmp_results = []
        self.crccmp_local = None
        self.crccmp_local_crc = None
        self.crccmp_sent = None
        self.crccmp_start = 0
        self.crccmp_expect = None
        self.session_waiting = False
        self.session_wait_reported = False
        self.done = False
        self.last_op_reply = False
        self.request_retries = 0

    def prepare_send(self, op, preserve_seq=False):
        '''prepare a request and update protocol state as if it was sent'''
        if not preserve_seq:
            op.seq = self.seq
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0]*((HDR_Len+MAX_Payload)-plen)))
        now = time.time()
        if not preserve_seq:
            self.seq = (self.seq + 1) % 256
            self.request_retries = 0
            self.send_times[op.seq] = now
        else:
            # Do not use replies to retransmitted requests as RTT samples: a
            # reply may belong to either transmission (Karn's algorithm).
            self.send_times[op.seq] = None
        self.last_op = op
        self.last_op_reply = False
        if self.ftp_settings.debug > 1:
            print("> %s dt=%.2f" % (op, now - self.last_op_time))
        self.last_op_time = now
        return payload

    def send(self, op, preserve_seq=False):
        '''send a request'''
        if self.master is None:
            print("FTP: Can't send request, no master...")
            return
        payload = self.prepare_send(op, preserve_seq=preserve_seq)
        self.manager.send_payloads(self, [payload])

    def send_batch(self, ops):
        '''send requests in one link write when supported by pymavlink'''
        if len(ops) == 0:
            return
        if self.master is None:
            print("FTP: Can't send request, no master...")
            return
        payloads = [self.prepare_send(op) for op in ops]
        self.manager.send_payloads(self, payloads)

    def update_rtt(self, sample):
        '''Update the smoothed RTT and variance from an unambiguous reply.'''
        sample = max(0.001, sample)
        if not self.rtt_valid:
            self.rtt = sample
            self.rttvar = sample / 2.0
            self.rtt_valid = True
            return
        self.rttvar = 0.75 * self.rttvar + 0.25 * abs(self.rtt - sample)
        self.rtt = 0.875 * self.rtt + 0.125 * sample

    def retry_timeout(self):
        '''Return an RTT-sensitive retransmission timeout.'''
        minimum = max(0.05, self.ftp_settings.retry_time)
        if not self.rtt_valid:
            return max(1.0, minimum)
        return max(minimum, min(10.0, self.rtt + 4.0 * self.rttvar))

    def terminate_session(self, outcome="failed"):
        '''terminate current session. outcome describes an incomplete transfer
        for the status line: "cancelled" when the user or a new command ended
        it, "failed" for an error'''
        if self.done:
            return
        self.done = True
        self.transfer_active = False
        if self.crccmp_dest is not None:
            print("crccmp: aborted")
            self.crccmp_reset()
        if self.show_progress:
            # only reached when a transfer ends without completing, as the
            # completion paths clear show_progress first
            self.show_progress = False
            self.set_progress_status("%s %s %s" % (
                "Uploading" if self.write_list is not None else "Downloading",
                self.filename, outcome))
        self.send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
        self.fh = None
        self.filename = None
        self.write_list = None
        self.write_open = False
        if self.callback is not None:
            # tell caller that the transfer failed
            self.callback(None)
            self.callback = None
        if self.put_callback is not None:
            # tell caller that the transfer failed
            self.put_callback(None)
            self.put_callback = None
        if self.put_callback_progress is not None:
            self.put_callback_progress(None)
            self.put_callback_progress = None
        self.read_gaps = []
        self.read_total = 0
        self.read_gap_times = {}
        self.last_read = None
        self.last_burst_read = None
        self.reached_eof = False
        self.backlog = 0
        self.duplicates = 0
        if self.ftp_settings.debug > 0:
            print("Terminated session")
        self.manager.worker_done(self)

    def cmd_list(self, args):
        '''list files'''
        if len(args) > 0:
            dname = args[0]
        else:
            dname = '/'
        print("Listing %s" % dname)
        enc_dname = bytearray(dname, 'ascii')
        self.total_size = 0
        self.dir_offset = 0
        op = FTP_OP(self.seq, self.session, OP_ListDirectory, len(enc_dname), 0, 0, self.dir_offset, enc_dname)
        self.send(op)

    def handle_list_reply(self, op, m):
        '''handle OP_ListDirectory reply'''
        if op.opcode == OP_Ack:
            dentries = sorted(op.payload.split(b'\x00'))
            #print(dentries)
            for d in dentries:
                if len(d) == 0:
                    continue
                self.dir_offset += 1
                try:
                    if sys.version_info.major >= 3:
                        d = str(d, 'ascii')
                    else:
                        d = str(d)
                except Exception:
                    continue
                if d[0] == 'D':
                    print(" D %s" % d[1:])
                elif d[0] == 'F':
                    (name, size) = d[1:].split('\t')
                    size = int(size)
                    self.total_size += size
                    print("   %s\t%u" % (name, size))
                else:
                    print(d)
            # ask for more
            more = self.last_op
            more.offset = self.dir_offset
            self.send(more)
        elif op.opcode == OP_Nack and len(op.payload) == 1 and op.payload[0] == ERR_EndOfFile:
            print("Total size %.2f kByte" % (self.total_size / 1024.0))
            self.total_size = 0
            self.terminate_session()
        else:
            print('LIST: %s' % op)
            self.terminate_session()

    def cmd_get(self, args, callback=None, callback_progress=None):
        '''get file'''
        if len(args) == 0:
            print("Usage: get FILENAME <LOCALNAME>")
            return
        fname = args[0]
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        if callback is None or self.ftp_settings.debug > 1:
            print("Getting %s as %s" % (fname, self.filename))
        self.op_start = time.time()
        self.callback = callback
        self.callback_progress = callback_progress
        self.show_progress = callback is None
        self.remote_file_size = None
        self.transfer_active = True
        self.read_retries = 0
        self.duplicates = 0
        self.reached_eof = False
        self.burst_size = self.ftp_settings.burst_read_size
        if self.burst_size < 1:
            self.burst_size = 239
        elif self.burst_size > 239:
            self.burst_size = 239
        enc_fname = bytearray(fname, 'ascii')
        self.open_retries = 0
        op = FTP_OP(self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def handle_open_RO_reply(self, op, m):
        '''handle OP_OpenFileRO reply'''
        if op.opcode == OP_Ack:
            if self.filename is None:
                return
            if op.size == 4 and op.payload is not None and len(op.payload) >= 4:
                # servers report the file size here, giving us a progress total
                self.remote_file_size = struct.unpack("<I", bytes(op.payload[:4]))[0]
            try:
                if self.callback is not None or self.filename == '-':
                    self.fh = SIO()
                else:
                    self.fh = open(self.filename, 'wb')
            except Exception as ex:
                print("Failed to open %s: %s" % (self.filename, ex))
                self.terminate_session()
                return
            read = FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, 0, None)
            self.last_burst_read = time.time()
            self.send(read)
        else:
            if self.callback is None or self.ftp_settings.debug > 0:
                print("ftp open failed")
            self.terminate_session()

    def check_read_finished(self):
        '''check if download has completed'''
        if self.reached_eof and len(self.read_gaps) == 0:
            ofs = self.fh.tell()
            dt = time.time() - self.op_start
            rate = (ofs / dt) / 1024.0
            if self.callback is not None:
                self.fh.seek(0)
                self.callback(self.fh)
                self.callback = None
            elif self.filename == "-":
                self.fh.seek(0)
                if sys.version_info.major < 3:
                    print(self.fh.read())
                else:
                    # a non-UTF-8 file must not raise here, or the session
                    # teardown and status cleanup below never run
                    print(self.fh.read().decode('utf-8', errors='replace'))
            else:
                print("Wrote %u bytes to %s in %.2fs %.1fkByte/s" % (ofs, self.filename, dt, rate))
            self.finished_status("downloading", self.filename, ofs)
            self.terminate_session()
            return True
        return False

    def write_payload(self, op):
        '''write payload from a read op'''
        self.fh.seek(op.offset)
        self.fh.write(op.payload)
        self.read_total += len(op.payload)
        if self.callback_progress is not None:
            self.callback_progress(self.fh, self.read_total)
    
    def handle_burst_read(self, op, m):
        '''handle OP_BurstReadFile reply'''
        if self.fh is None or self.filename is None:
            if op.session != self.session:
                # old session
                return
            print("FTP Unexpected burst read reply")
            print(op)
            return
        self.last_burst_read = time.time()
        size = len(op.payload)
        if size > self.burst_size:
            # this server doesn't handle the burst size argument
            self.burst_size = MAX_Payload
            if self.ftp_settings.debug > 0:
                print("Setting burst size to %u" % self.burst_size)
        if op.opcode == OP_Ack and self.fh is not None:
            ofs = self.fh.tell()
            if op.offset < ofs:
                # writing an earlier portion, possibly remove a gap
                gap = (op.offset, len(op.payload))
                if gap in self.read_gaps:
                    self.read_gaps.remove(gap)
                    self.read_gap_times.pop(gap)
                    if self.ftp_settings.debug > 0:
                        print("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                else:
                    if self.ftp_settings.debug > 0:
                        print("FTP: dup read reply at %u of len %u ofs=%u" % (op.offset, op.size, self.fh.tell()))
                    self.duplicates += 1
                    return
                self.write_payload(op)
                self.fh.seek(ofs)
                if self.check_read_finished():
                    return
            elif op.offset > ofs:
                # we have a gap
                gap = (ofs, op.offset-ofs)
                max_read = self.burst_size
                while True:
                    if gap[1] <= max_read:
                        self.read_gaps.append(gap)
                        self.read_gap_times[gap] = 0
                        break
                    g = (gap[0], max_read)
                    self.read_gaps.append(g)
                    self.read_gap_times[g] = 0
                    gap = (gap[0] + max_read, gap[1] - max_read)
                self.write_payload(op)
            else:
                self.write_payload(op)
            if op.burst_complete:
                if op.size > 0 and op.size < self.burst_size:
                    # a burst complete with non-zero size and less than burst packet size
                    # means EOF
                    if not self.reached_eof and self.ftp_settings.debug > 0:
                        print("EOF at %u with %u gaps t=%.2f" % (self.fh.tell(), len(self.read_gaps), time.time() - self.op_start))
                    self.reached_eof = True
                    if self.check_read_finished():
                        return
                    self.check_read_send()
                    return
                more = self.last_op
                more.offset = op.offset + op.size
                if self.ftp_settings.debug > 0:
                    print("FTP: burst continue at %u %u" % (more.offset, self.fh.tell()))
                self.send(more)
        elif op.opcode == OP_Nack:
            ecode = op.payload[0]
            if self.ftp_settings.debug > 0:
                print("FTP: burst nack: ", op)
            if ecode == ERR_EndOfFile or ecode == 0:
                if not self.reached_eof and op.offset > self.fh.tell():
                    # we lost the last part of the burst
                    if self.ftp_settings.debug > 0:
                        print("burst lost EOF %u %u" % (self.fh.tell(), op.offset))
                    return
                if not self.reached_eof and self.ftp_settings.debug > 0:
                    print("EOF at %u with %u gaps t=%.2f" % (self.fh.tell(), len(self.read_gaps), time.time() - self.op_start))
                self.reached_eof = True
                if self.check_read_finished():
                    return
                self.check_read_send()
            else:
                # not recoverable, and without ending the session here the
                # idle_task retry would re-request the burst forever
                print("FTP: burst Nack (ecode:%u): %s" % (ecode, op))
                self.terminate_session()
        else:
            print("FTP: burst error: %s" % op)

    def handle_reply_read(self, op, m):
        '''handle OP_ReadFile reply'''
        if self.fh is None or self.filename is None:
            if self.ftp_settings.debug > 0:
                print("FTP Unexpected read reply")
                print(op)
            return
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            if gap in self.read_gaps:
                if self.read_gap_times[gap] > 0 and self.backlog > 0:
                    self.backlog -= 1
                self.read_gaps.remove(gap)
                self.read_gap_times.pop(gap)
                ofs = self.fh.tell()
                self.write_payload(op)
                self.fh.seek(ofs)
                if self.ftp_settings.debug > 0:
                    print("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                if self.check_read_finished():
                    return
            elif op.size < self.burst_size:
                print("FTP: file size changed to %u" % op.offset+op.size)
                self.terminate_session()
            else:
                self.duplicates += 1
                if self.ftp_settings.debug > 0:
                    print("FTP: no gap read", gap, len(self.read_gaps))
        elif op.opcode == OP_Nack:
            print("Read failed with %u gaps" % len(self.read_gaps), str(op))
            self.terminate_session()
        self.check_read_send()
            
    def cmd_put(self, args, fh=None, callback=None, progress_callback=None):
        '''put file'''
        if len(args) == 0:
            print("Usage: put FILENAME <REMOTENAME>")
            return
        if self.transfer_active:
            # fh and write_list are both still None while a get waits for its
            # open reply, so they can't stand in for "a transfer is running"
            print("FTP transfer already in progress")
            if callback is not None:
                callback(None)
            if progress_callback is not None:
                # this is what publishes "Params ERR"/"Mission ERR", so
                # skipping it leaves a stale percentage on the console
                progress_callback(None)
            return
        fname = args[0]
        self.fh = fh
        if self.fh is None:
            try:
                self.fh = open(fname, 'rb')
            except Exception as ex:
                print("Failed to open %s: %s" % (fname, ex))
                return
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        if self.filename.endswith("/"):
            self.filename += os.path.basename(fname)
        if callback is None:
            print("Putting %s as %s" % (fname, self.filename))
        self.fh.seek(0,2)
        file_size = self.fh.tell()
        self.fh.seek(0)

        # setup write list
        self.write_block_size = self.ftp_settings.write_size
        if self.write_block_size < 1 or self.write_block_size > MAX_Payload:
            # an oversized block silently overflows the 251 byte payload, and
            # the size field is only 8 bits. clamp as the read path does.
            self.write_block_size = MAX_Payload
        self.write_file_size = file_size

        write_blockcount = file_size // self.write_block_size
        if file_size % self.write_block_size != 0:
            write_blockcount += 1

        self.write_list = set(range(write_blockcount))
        self.write_acks = 0
        self.write_acked_bytes = 0
        self.write_total = write_blockcount
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_inflight = set()
        self.write_last_send = None
        self.write_open = False

        self.put_callback = callback
        self.put_callback_progress = progress_callback
        self.show_progress = callback is None
        self.transfer_active = True
        self.read_retries = 0
        self.op_start = time.time()
        enc_fname = bytearray(self.filename, 'ascii')
        # burst_complete is otherwise unused for CreateFile. It opts in to
        # cumulative ACKs from servers that can commit contiguous write
        # requests as a batch; older servers safely ignore it.
        op = FTP_OP(self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 1, 0, enc_fname)
        self.send(op)

    def write_block_len(self, idx):
        '''length of upload block idx, which is short for the final block'''
        ofs = idx * self.write_block_size
        return max(0, min(self.write_block_size, self.write_file_size - ofs))

    def put_finished(self, flen):
        '''finish a put'''
        if self.put_callback_progress:
            self.put_callback_progress(1.0)
            self.put_callback_progress = None
        if self.put_callback is not None:
            self.put_callback(flen)
            self.put_callback = None
        else:
            dt = max(time.time() - self.op_start, 1.0e-6)
            print("Sent file of length %u in %.2fs %.1fkByte/s" %
                  (flen, dt, (flen / dt) / 1024.0))
        self.finished_status("uploading", self.filename, flen)
        
    def handle_create_file_reply(self, op, m):
        '''handle OP_CreateFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode == OP_Ack:
            self.write_open = True
            if op.size >= 3 and op.payload[0] == WRITE_CAPABILITY_MAGIC:
                if self.ftp_settings.write_qsize <= 0:
                    self.write_qsize = max(1, op.payload[1])
                if self.ftp_settings.write_batch_size <= 0:
                    self.write_batch_size = max(1, op.payload[2])
            self.send_more_writes()
        else:
            print("Create failed")
            self.terminate_session()

    def send_more_writes(self):
        '''send some more writes'''
        if not self.write_open:
            return
        if len(self.write_list) == 0:
            # all done
            self.put_finished(self.write_file_size)
            self.terminate_session()
            return

        now = time.time()
        if self.write_last_send is not None:
            if now - self.write_last_send > self.retry_timeout():
                # we seem to have lost a block of replies
                self.write_inflight.clear()
                self.write_pending = 0
                self.write_last_send = now

        qsize = max(1, self.write_qsize)
        batch_size = max(1, min(self.write_batch_size, qsize))
        free_slots = qsize - self.write_pending
        if self.write_pending > 0 and free_slots < batch_size:
            return

        unsent = len(self.write_list - self.write_inflight)
        n = min(free_slots, unsent)
        writes = []
        for i in range(n):
            # send in round-robin, skipping any that have been acked
            idx = self.write_idx
            while idx not in self.write_list or idx in self.write_inflight:
                idx = (idx + 1) % self.write_total
            ofs = idx * self.write_block_size
            self.fh.seek(ofs)
            data = self.fh.read(self.write_block_size)
            write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
            writes.append(write)
            self.write_idx = (idx + 1) % self.write_total
            self.write_inflight.add(idx)
            self.write_pending += 1
            self.write_last_send = now
        self.send_batch(writes)

    def handle_write_reply(self, op, m):
        '''handle OP_WriteFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode != OP_Ack:
            print("Write failed: %s" % op)
            self.terminate_session()
            return

        # Legacy servers ACK one block at a time. Negotiated batch ACKs carry
        # an explicit contiguous range so gaps remain eligible for retry.
        idx = op.offset // self.write_block_size
        previous_idx = self.write_recv_idx
        acked = [idx]
        if op.burst_complete:
            if op.size >= 4:
                start_offset, = struct.unpack('<I', bytes(op.payload[:4]))
                start_idx = start_offset // self.write_block_size
                count = ((idx - start_idx) % self.write_total) + 1
                acked = [((start_idx + i) % self.write_total)
                         for i in range(count)]
            else:
                # Compatibility with early servers that only marked the last
                # block. The negotiated format also supplies the start offset
                # so a dropped request cannot be mistaken for a written block.
                count = (idx - previous_idx) % self.write_total
                if count == 0:
                    count = self.write_total
                acked = [((previous_idx + i) % self.write_total)
                         for i in range(1, count + 1)]
        else:
            count = (idx - previous_idx) % self.write_total
            # An ACK jump on a legacy server means intervening requests were
            # dropped. Mark them sendable again, while only idx is complete.
            for i in range(1, count):
                self.write_inflight.discard(
                    (previous_idx + i) % self.write_total)
        for ack_idx in acked:
            if ack_idx in self.write_list:
                # servers are required to resend replies to repeated requests,
                # so only count a block the first time it is acked
                self.write_list.discard(ack_idx)
                self.write_acks += 1
                self.write_acked_bytes += self.write_block_len(ack_idx)
            self.write_inflight.discard(ack_idx)
        self.write_pending = len(self.write_inflight)
        self.write_recv_idx = idx
        if self.put_callback_progress:
            self.put_callback_progress(self.write_acks/float(self.write_total))
        self.send_more_writes()

    def cmd_rm(self, args):
        '''remove file'''
        if len(args) == 0:
            print("Usage: rm FILENAME")
            return
        fname = args[0]
        print("Removing %s" % fname)
        enc_fname = bytearray(fname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def cmd_rmdir(self, args):
        '''remove directory'''
        if len(args) == 0:
            print("Usage: rmdir FILENAME")
            return
        dname = args[0]
        print("Removing %s" % dname)
        enc_dname = bytearray(dname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveDirectory, len(enc_dname), 0, 0, 0, enc_dname)
        self.send(op)

    def handle_remove_reply(self, op, m):
        '''handle remove reply'''
        if op.opcode != OP_Ack:
            print("Remove failed %s" % op)
        self.terminate_session()

    def cmd_rename(self, args):
        '''rename file'''
        if len(args) < 2:
            print("Usage: rename OLDNAME NEWNAME")
            return
        name1 = args[0]
        name2 = args[1]
        print("Renaming %s to %s" % (name1, name2))
        enc_name1 = bytearray(name1, 'ascii')
        enc_name2 = bytearray(name2, 'ascii')
        enc_both = enc_name1 + b'\x00' + enc_name2
        op = FTP_OP(self.seq, self.session, OP_Rename, len(enc_both), 0, 0, 0, enc_both)
        self.send(op)

    def handle_rename_reply(self, op, m):
        '''handle rename reply'''
        if op.opcode != OP_Ack:
            print("Rename failed %s" % op)
        self.terminate_session()

    def cmd_mkdir(self, args):
        '''make directory'''
        if len(args) < 1:
            print("Usage: mkdir NAME")
            return
        name = args[0]
        print("Creating directory %s" % name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateDirectory, len(enc_name), 0, 0, 0, enc_name)
        self.send(op)

    def handle_mkdir_reply(self, op, m):
        '''handle mkdir reply'''
        if op.opcode != OP_Ack:
            print("Create directory failed %s" % op)
        self.terminate_session()

    def cmd_crc(self, args):
        '''get crc'''
        if len(args) < 1:
            print("Usage: crc NAME")
            return
        name = args[0]
        self.filename = name
        self.op_start = time.time()
        print("Getting CRC for %s" % name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CalcFileCRC32, len(enc_name), 0, 0, 0, bytearray(enc_name))
        self.send(op)

    def local_file_crc(self, name):
        '''CRC32 of a local file as the vehicle would compute it.

        ArduPilot's crc_crc32() runs the standard reflected CRC32 table from a
        zero seed with no final inversion, which is not what zlib.crc32() gives.
        Seeding with 0xffffffff and inverting the result cancels zlib's own
        inversions and leaves the raw value the vehicle reports.
        '''
        crc = 0xffffffff
        with open(name, 'rb') as f:
            while True:
                buf = f.read(65536)
                if not buf:
                    break
                crc = zlib.crc32(buf, crc)
        return crc ^ 0xffffffff

    def cmd_crclocal(self, args):
        '''get crc of a local file, for comparison with "ftp crc"'''
        if len(args) < 1:
            print("Usage: crclocal NAME")
            return
        name = args[0]
        start = time.time()
        try:
            crc = self.local_file_crc(name)
        except Exception as ex:
            print("crclocal failed %s: %s" % (name, ex))
            return
        print("crclocal: %s 0x%08x in %.1fs" % (name, crc, time.time() - start))

    def crccmp_reset(self):
        '''clear crccmp state without touching the session'''
        self.crccmp_dest = None
        self.crccmp_pending = []
        self.crccmp_local = None
        self.crccmp_local_crc = None
        self.crccmp_sent = None
        self.crccmp_expect = None
        self.transfer_active = False

    def cmd_crccmp(self, args):
        '''compare local files against the same names in a remote directory'''
        if len(args) < 2:
            print("Usage: crccmp WILDCARD DESTDIR")
            return
        if self.transfer_active:
            print("FTP transfer already in progress")
            return
        (pattern, dest) = (args[0], args[1])
        if dest == '':
            print("crccmp: empty DESTDIR, use / for the vehicle's root")
            return
        files = sorted(f for f in glob.glob(pattern) if os.path.isfile(f))
        if len(files) == 0:
            print("crccmp: no files matching %s" % pattern)
            return
        # only the basename is used remotely, so duplicates would compare two
        # local files against one remote file and double count the result
        seen = {}
        for f in files:
            seen.setdefault(os.path.basename(f), []).append(f)
        clashes = {b: v for (b, v) in seen.items() if len(v) > 1}
        if clashes:
            for (b, v) in sorted(clashes.items()):
                print("crccmp: %s matches %u local files: %s" %
                      (b, len(v), ' '.join(v)))
            print("crccmp: duplicate names, narrow the wildcard")
            return
        self.crccmp_dest = dest.rstrip('/')
        self.crccmp_pending = files
        self.crccmp_results = []
        self.crccmp_start = time.time()
        # block get/put for the duration, as they would take over the session
        self.transfer_active = True
        print("crccmp: %u files matching %s against %s" %
              (len(files), pattern, self.crccmp_dest))
        self.crccmp_next()

    def crccmp_next(self):
        '''ask for the next remote CRC, or finish if the list is done'''
        while self.crccmp_pending:
            local = self.crccmp_pending.pop(0)
            base = os.path.basename(local)
            name = "%s/%s" % (self.crccmp_dest, base)
            # encode and length check before anything is marked in flight, so a
            # bad name is an immediate error rather than a timeout later on
            try:
                enc_name = bytearray(name, 'ascii')
            except UnicodeEncodeError:
                self.crccmp_record('ERROR', base, ' (non-ascii remote path)')
                continue
            if len(enc_name) > MAX_FTP_NAME:
                self.crccmp_record('ERROR', base, ' (remote path over %u bytes)' %
                                   MAX_FTP_NAME)
                continue
            try:
                # done one file at a time rather than all up front, so a long
                # list doesn't stall the main loop in one go
                self.crccmp_local_crc = self.local_file_crc(local)
            except Exception as ex:
                self.crccmp_record('ERROR', base, ' (%s)' % ex)
                continue
            self.crccmp_local = local
            self.filename = name
            self.op_start = time.time()
            self.crccmp_sent = time.time()
            self.send(FTP_OP(self.seq, self.session, OP_CalcFileCRC32,
                             len(enc_name), 0, 0, 0, enc_name))
            # send() has already advanced self.seq, and the server replies with
            # the request sequence plus one, so this is the reply we expect
            self.crccmp_expect = (self.session, self.seq)
            return
        self.crccmp_finish()

    def crccmp_record(self, result, name, extra=''):
        self.crccmp_results.append(result)
        print("  %-7s %s%s" % (result, name, extra))

    def crccmp_reply(self, op):
        '''one remote CRC came back: compare and move on'''
        if self.crccmp_expect is not None and \
           (op.session, op.seq) != self.crccmp_expect:
            # a late reply for a file we already timed out, or a duplicate.
            # attributing it to the file now in flight would report a result
            # for a CRC that was never asked for. it does tell us the vehicle
            # is still working, so give the current request its time back.
            if self.crccmp_sent is not None:
                self.crccmp_sent = time.time()
            return
        name = os.path.basename(self.crccmp_local)
        crc = None
        if op.opcode == OP_Ack and op.size == 4:
            crc, = struct.unpack("<I", bytes(op.payload[:4]))
        ecode = None
        if op.opcode == OP_Nack and op.payload is not None and len(op.payload) >= 1:
            ecode = op.payload[0]

        if crc is not None and crc == self.crccmp_local_crc:
            self.crccmp_record('MATCH', name, ' 0x%08x' % crc)
        elif crc is not None:
            self.crccmp_record('DIFFER', name, ' local 0x%08x remote 0x%08x' %
                               (self.crccmp_local_crc, crc))
        elif ecode == ERR_FileNotFound:
            self.crccmp_record('MISSING', name)
        else:
            self.crccmp_record('ERROR', name, ' (%s)' % op)
        self.crccmp_sent = None
        self.crccmp_expect = None
        self.crccmp_next()

    def crccmp_finish(self):
        '''report the tally'''
        results = self.crccmp_results
        dt = time.time() - self.crccmp_start
        print("crccmp: %u match, %u differ, %u missing, %u errors in %.1fs" %
              (results.count('MATCH'), results.count('DIFFER'),
               results.count('MISSING'),
               results.count('ERROR') + results.count('TIMEOUT'), dt))
        self.crccmp_reset()
        self.terminate_session()

    def handle_crc_reply(self, op, m):
        '''handle crc reply'''
        if self.crccmp_dest is not None:
            self.crccmp_reply(op)
            return
        if op.opcode == OP_Ack and op.size == 4:
            crc, = struct.unpack("<I", op.payload)
            now = time.time()
            print("crc: %s 0x%08x in %.1fs" % (self.filename, crc, now - self.op_start))
        else:
            print("crc failed %s" % op)
        self.terminate_session()

    def cmd_cancel(self):
        '''cancel any pending op'''
        self.terminate_session("cancelled")

    def set_progress_status(self, status):
        '''show a transfer status line in the console, where log download shows its own'''
        self.console.set_status('FTP', status, row=4)

    def transfer_status(self):
        '''describe the transfer in progress, or None if there isn't one'''
        if self.op_start is None:
            return None
        dt = max(time.time() - self.op_start, 1.0e-6)
        if self.write_list is not None:
            # an upload is paced by acks, so count bytes the vehicle has
            # actually stored rather than what we have pushed at it
            done = min(self.write_acked_bytes, self.write_file_size)
            pct = 100.0 * done / self.write_file_size if self.write_file_size else 100.0
            return "Uploading %s - %u/%u bytes %.1f%% %.1f kbyte/s" % (
                self.filename, done, self.write_file_size, pct,
                (done / dt) / 1024.0)
        if self.fh is None:
            return None
        if self.remote_file_size:
            # servers that don't report a size leave us without a percentage
            progress = "%u/%u bytes %.1f%%" % (
                self.read_total, self.remote_file_size,
                min(100.0 * self.read_total / self.remote_file_size, 100.0))
        else:
            progress = "%u bytes" % self.read_total
        return "Downloading %s - %s %.1f kbyte/s (%u retries %u gaps)" % (
            self.filename, progress, (self.read_total / dt) / 1024.0,
            self.read_retries, len(self.read_gaps))

    def update_status(self):
        '''update the console transfer status, rate limited like log download'''
        if not self.show_progress:
            return
        now = time.time()
        if now - self.last_status_time < 0.5:
            return
        self.last_status_time = now
        status = self.transfer_status()
        if status is not None:
            self.set_progress_status(status)

    def finished_status(self, verb, filename, size):
        '''final line for a completed interactive transfer'''
        if not self.show_progress:
            return
        self.show_progress = False
        dt = max(time.time() - self.op_start, 1.0e-6)
        self.set_progress_status(
            "Finished %s %s (%u bytes %.1f seconds, %.1f kbyte/sec)" % (
                verb, filename, size, dt, (size / dt) / 1024.0))

    def cmd_status(self):
        '''show status'''
        if self.fh is None:
            print("No transfer in progress")
        else:
            ofs = self.fh.tell()
            dt = time.time() - self.op_start
            rate = (ofs / dt) / 1024.0
            print("Transfer at offset %u with %u gaps %u retries %.1f kByte/sec" % (ofs, len(self.read_gaps), self.read_retries, rate))

    def op_parse(self, m):
        '''parse a FILE_TRANSFER_PROTOCOL msg'''
        hdr = bytearray(m.payload[0:12])
        (seq, session, opcode, size, req_opcode, burst_complete, pad, offset) = struct.unpack("<HBBBBBBI", hdr)
        payload = bytearray(m.payload[12:])[:size]
        return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset, payload)

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "FILE_TRANSFER_PROTOCOL":
            if (m.target_system != self.settings.source_system or
                m.target_component != self.settings.source_component):
                if m.target_system == self.settings.source_system and not self.warned_component:
                    self.warned_component = True
                    print("FTP reply for mavlink component %u" % m.target_component)
                return

            op = self.op_parse(m)
            now = time.time()
            dt = now - self.last_op_time
            if self.ftp_settings.debug > 1:
                print("< %s dt=%.2f" % (op, dt))
            self.last_op_time = now

            request_seq = (op.seq - 1) % 256
            sent = self.send_times.pop(request_seq, None)
            if sent is not None:
                self.update_rtt(now - sent)

            if op.opcode == OP_Nack and op.payload is not None and \
               len(op.payload) == 1 and op.payload[0] == ERR_NoSessionsAvailable:
                # Another client may also be using the server, so the local
                # concurrency cap is not sufficient on its own.  Keep this
                # operation intact and retry instead of failing its callback.
                self.session_waiting = True
                self.last_op_time = now
                if not self.session_wait_reported:
                    print("FTP: no sessions available, waiting to retry")
                    self.session_wait_reported = True
                return

            if self.last_op is not None and \
               op.req_opcode == self.last_op.opcode and \
               op.seq == (self.last_op.seq + 1) % 256:
                self.last_op_reply = True
            if op.req_opcode == OP_ListDirectory:
                self.handle_list_reply(op, m)
            elif op.req_opcode == OP_OpenFileRO:
                self.handle_open_RO_reply(op, m)
            elif op.req_opcode == OP_BurstReadFile:
                self.handle_burst_read(op, m)
            elif op.req_opcode == OP_TerminateSession:
                pass
            elif op.req_opcode == OP_CreateFile:
                self.handle_create_file_reply(op, m)
            elif op.req_opcode == OP_WriteFile:
                self.handle_write_reply(op, m)
            elif op.req_opcode in [OP_RemoveFile, OP_RemoveDirectory]:
                self.handle_remove_reply(op, m)
            elif op.req_opcode == OP_Rename:
                self.handle_rename_reply(op, m)
            elif op.req_opcode == OP_CreateDirectory:
                self.handle_mkdir_reply(op, m)
            elif op.req_opcode == OP_ReadFile:
                self.handle_reply_read(op, m)
            elif op.req_opcode == OP_CalcFileCRC32:
                self.handle_crc_reply(op, m)
            else:
                print('FTP Unknown %s' % str(op))

    def send_gap_read(self, g):
        '''send a read for a gap'''
        (offset, length) = g
        if self.ftp_settings.debug > 0:
            print("Gap read of %u at %u rem=%u blog=%u" % (length, offset, len(self.read_gaps), self.backlog))
        read = FTP_OP(self.seq, self.session, OP_ReadFile, length, 0, 0, offset, None)
        self.send(read)
        self.last_gap_send = time.time()
        self.read_gap_times[g] = self.last_gap_send
        self.backlog += 1

    def check_read_send(self):
        '''keep a bounded window of gap reads in flight'''
        if len(self.read_gaps) == 0:
            return
        now = time.time()
        timeout = self.retry_timeout()
        for g in self.read_gaps:
            sent = self.read_gap_times[g]
            if sent > 0 and now - sent > timeout:
                self.read_gap_times[g] = 0
                if self.backlog > 0:
                    self.backlog -= 1

        limit = max(1, self.ftp_settings.max_backlog)
        for g in self.read_gaps:
            if self.backlog >= limit:
                break
            if self.read_gap_times[g] == 0:
                self.send_gap_read(g)

    def idle_task(self):
        '''check for file gaps and lost requests'''
        now = time.time()

        if self.session_waiting:
            if now - self.last_op_time >= self.retry_timeout():
                self.session_waiting = False
                if self.last_op.opcode == OP_OpenFileRO:
                    self.op_start = now
                self.send(self.last_op)
            return

        # ArduPilot's incoming FTP request queue has the same depth as its
        # session table.  Under contention an initial request can therefore be
        # dropped before the server has a chance to NACK it.  Reusing the same
        # sequence number makes this safe whether the request or its reply was
        # lost: the server's duplicate-request cache returns the old reply.
        initial_opcodes = (
            OP_ListDirectory, OP_OpenFileRO, OP_CreateFile, OP_RemoveFile,
            OP_RemoveDirectory, OP_Rename, OP_CreateDirectory,
            OP_CalcFileCRC32,
        )
        if self.last_op is not None and not self.last_op_reply and \
           self.last_op.opcode in initial_opcodes and \
           now - self.last_op_time > self.retry_timeout():
            self.request_retries += 1
            if self.request_retries > 10:
                print("FTP: request timed out: %s" % self.last_op)
                self.terminate_session()
                return
            if self.ftp_settings.debug > 0:
                print("FTP: retry request: %s" % self.last_op)
            self.send(self.last_op, preserve_seq=True)
            return

        # ahead of the early returns below, which skip idle transfers
        self.update_status()

        if self.crccmp_sent is not None and \
           now - self.crccmp_sent > self.ftp_settings.crccmp_timeout:
            self.crccmp_record('TIMEOUT', os.path.basename(self.crccmp_local))
            self.crccmp_sent = None
            self.crccmp_expect = None
            self.crccmp_next()

        if len(self.read_gaps) == 0 and self.last_burst_read is None and self.write_list is None:
            return

        if self.fh is None:
            return

        # see if burst read has stalled
        if not self.reached_eof and self.last_burst_read is not None and \
           now - self.last_burst_read > self.retry_timeout():
            dt = now - self.last_burst_read
            self.last_burst_read = now
            if self.ftp_settings.debug > 0:
                print("Retry read at %u rtt=%.2f dt=%.2f" % (self.fh.tell(), self.rtt, dt))
            self.send(FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, self.fh.tell(), None))
            self.read_retries += 1

        # see if we can fill gaps
        self.check_read_send()

        if self.write_list is not None:
            self.send_more_writes()


class FTPModule(mp_module.MPModule):
    '''Public FTP module and concurrent-session manager.'''

    def __init__(self, mpstate):
        super(FTPModule, self).__init__(mpstate, "ftp", public=True)
        self.add_command('ftp', self.cmd_ftp, "file transfer",
                         ["<list|get|rm|rmdir|rename|mkdir|crc|cancel|status>",
                          "set (FTPSETTING)",
                          "put (FILENAME) (FILENAME)",
                          "crclocal (FILENAME)",
                          "crccmp (FILENAME)"])
        self.ftp_settings = mp_settings.MPSettings(
            [('debug', int, 0),
             ('pkt_loss_tx', int, 0),
             ('pkt_loss_rx', int, 0),
             ('pkt_lag_tx', float, 0.0),
             ('pkt_lag_rx', float, 0.0),
             ('pkt_lag_jitter_tx', float, 0.0),
             ('pkt_lag_jitter_rx', float, 0.0),
             ('loss_seed', int, 0),
             ('max_backlog', int, 5),
             ('burst_read_size', int, MAX_Payload),
             ('write_size', int, MAX_Payload),
             # zero selects the server-advertised values, with conservative
             # fallbacks for servers predating write batching
             ('write_qsize', int, 0),
             ('write_batch_size', int, 0),
             ('retry_time', float, 0.5),
             ('crccmp_timeout', float, 120.0),
             # ArduPilot currently has five GCS_FTP server sessions.  Keeping
             # the cap configurable also supports smaller/custom servers.
             ('max_sessions', int, 5)])
        self.add_completion_function('(FTPSETTING)',
                                     self.ftp_settings.completion)
        self.workers = {}
        self.pending = []
        # A previous process can leave delayed packets or a cached reply on a
        # poor link. Starting every process at session zero can then turn a
        # stale CreateFile ACK into writes against a closed server session.
        self.next_session = random.SystemRandom().randrange(256)
        self.warned_component = False
        self.loss_rng = random.Random()
        self.active_loss_seed = None
        self.delay_sequence = 0
        self.tx_delay_queue = []
        self.rx_delay_queue = []
        self.last_tx_deadline = 0.0
        self.last_rx_deadline = 0.0

    def packet_lost(self, direction):
        '''Return true when the configured link simulator drops a packet.'''
        seed = self.ftp_settings.loss_seed
        if seed != self.active_loss_seed:
            self.loss_rng.seed(None if seed == 0 else seed)
            self.active_loss_seed = seed
        percent = (self.ftp_settings.pkt_loss_tx if direction == 'TX'
                   else self.ftp_settings.pkt_loss_rx)
        lost = percent > 0 and self.loss_rng.uniform(0, 100) < percent
        if lost and self.ftp_settings.debug > 1:
            print("FTP: dropping packet %s" % direction)
        return lost

    def packet_delay(self, direction):
        '''Return simulated one-way delay in seconds.

        Jitter is a uniformly distributed extra delay. Delivery deadlines are
        constrained separately per direction so jitter models FIFO
        head-of-line blocking instead of reordering a serial telemetry link.
        '''
        if direction == 'TX':
            base = self.ftp_settings.pkt_lag_tx
            jitter = self.ftp_settings.pkt_lag_jitter_tx
        else:
            base = self.ftp_settings.pkt_lag_rx
            jitter = self.ftp_settings.pkt_lag_jitter_rx
        delay_ms = max(0.0, base)
        if jitter > 0:
            delay_ms += self.loss_rng.uniform(0, jitter)
        return delay_ms * 0.001

    def _transmit_payloads(self, master, network, target_system,
                           target_component, payloads):
        '''Serialize and transmit one logical batch of FTP requests.'''
        mav = master.mav
        if len(payloads) <= 1 or not hasattr(mav, 'file'):
            for payload in payloads:
                mav.file_transfer_protocol_send(
                    network, target_system, target_component, payload)
            return

        link = mav.file
        collector = MAVLinkBatchWriter()
        mav.file = collector
        try:
            for payload in payloads:
                mav.file_transfer_protocol_send(
                    network, target_system, target_component, payload)
        finally:
            mav.file = link
        if collector.packets:
            link.write(b''.join(collector.packets))

    def send_payloads(self, worker, payloads):
        '''Apply outgoing loss/lag, preserving batches that survive.'''
        payloads = [bytes(payload) for payload in payloads
                    if not self.packet_lost('TX')]
        if not payloads:
            return
        args = (worker.master, worker.network, worker.target_system,
                worker.target_component, payloads)
        lag = self.packet_delay('TX')
        if lag == 0:
            self._transmit_payloads(*args)
            return
        self.delay_sequence += 1
        deadline = max(time.monotonic() + lag, self.last_tx_deadline)
        self.last_tx_deadline = deadline
        heapq.heappush(self.tx_delay_queue,
                       (deadline, self.delay_sequence, args))

    def _packet_worker(self, m):
        try:
            session = m.payload[2]
        except (IndexError, TypeError):
            return None
        return self.workers.get(session)

    def cmd_ftp(self, args):
        '''FTP operations'''
        usage = "Usage: ftp <list|get|put|rm|rmdir|rename|mkdir|crc|crclocal|crccmp>"
        if len(args) < 1:
            print(usage)
            return
        command = args[0]
        if command == 'set':
            self.ftp_settings.command(args[1:])
        elif command == 'status':
            self.cmd_status()
        elif command == 'cancel':
            self.cmd_cancel()
        elif command == 'crclocal':
            self.cmd_crclocal(args[1:])
        else:
            method = getattr(self, 'cmd_' + command, None)
            if method is None:
                print(usage)
            else:
                method(args[1:])

    def _allocate_session(self):
        '''Return an unused client-selected uint8 session id.'''
        for _ in range(256):
            session = self.next_session
            self.next_session = (self.next_session + 1) % 256
            if session not in self.workers:
                return session
        return None

    def _launch(self, operation):
        session = self._allocate_session()
        if session is None:
            self.pending.insert(0, operation)
            return None
        worker = FTPWorker(self, session)
        worker.operation_name = operation['name']
        self.workers[session] = worker
        method = getattr(worker, operation['method'])
        method(*operation['args'], **operation['kwargs'])
        # Bad arguments or a local-file error can return without sending.
        if worker.last_op is None:
            self.worker_done(worker)
        return worker

    def _submit(self, name, method, *args, **kwargs):
        operation = {
            'name': name,
            'method': method,
            'args': args,
            'kwargs': kwargs,
        }
        limit = self._session_limit()
        if len(self.workers) >= limit:
            self.pending.append(operation)
            print("FTP: queued %s (%u sessions active)" %
                  (name, len(self.workers)))
            return None
        return self._launch(operation)

    def worker_done(self, worker):
        '''Forget a completed worker and start the oldest queued operation.'''
        if self.workers.get(worker.session) is worker:
            del self.workers[worker.session]
        limit = self._session_limit()
        while self.pending and len(self.workers) < limit:
            operation = self.pending.pop(0)
            self._launch(operation)

    def _session_limit(self):
        # Session ids are uint8.  Keep one value in reserve so allocation and
        # queuing remain well defined even with an accidental oversized setting.
        return min(255, max(1, int(self.ftp_settings.max_sessions)))

    def cmd_list(self, args):
        return self._submit('list', 'cmd_list', args)

    def cmd_get(self, args, callback=None, callback_progress=None):
        return self._submit('get', 'cmd_get', args,
                            callback=callback,
                            callback_progress=callback_progress)

    def cmd_put(self, args, fh=None, callback=None, progress_callback=None):
        return self._submit('put', 'cmd_put', args, fh=fh,
                            callback=callback,
                            progress_callback=progress_callback)

    def cmd_rm(self, args):
        return self._submit('rm', 'cmd_rm', args)

    def cmd_rmdir(self, args):
        return self._submit('rmdir', 'cmd_rmdir', args)

    def cmd_rename(self, args):
        return self._submit('rename', 'cmd_rename', args)

    def cmd_mkdir(self, args):
        return self._submit('mkdir', 'cmd_mkdir', args)

    def cmd_crc(self, args):
        return self._submit('crc', 'cmd_crc', args)

    def cmd_crccmp(self, args):
        return self._submit('crccmp', 'cmd_crccmp', args)

    def cmd_crclocal(self, args):
        # This operation is entirely local and consumes no server session.
        return FTPWorker(self, 0).cmd_crclocal(args)

    def cmd_cancel(self):
        '''Cancel all active and queued operations.'''
        pending = self.pending
        self.pending = []
        for operation in pending:
            callback = operation['kwargs'].get('callback')
            if callback is not None:
                callback(None)
            progress = operation['kwargs'].get('progress_callback')
            if progress is not None:
                progress(None)
        for worker in list(self.workers.values()):
            worker.terminate_session("cancelled")

    def cmd_status(self):
        if not self.workers and not self.pending:
            print("No FTP operations in progress")
            return
        for session, worker in sorted(self.workers.items()):
            status = worker.transfer_status()
            if status is None:
                status = worker.operation_name
            if worker.session_waiting:
                status += " (waiting for a server session)"
            print("FTP session %u: %s" % (session, status))
        if self.pending:
            print("FTP queued: %s" %
                  ', '.join(operation['name'] for operation in self.pending))

    def mavlink_packet(self, m):
        if m.get_type() != "FILE_TRANSFER_PROTOCOL":
            return
        if (m.target_system != self.settings.source_system or
                m.target_component != self.settings.source_component):
            if m.target_system == self.settings.source_system and not self.warned_component:
                self.warned_component = True
                print("FTP reply for mavlink component %u" % m.target_component)
            return
        if self.packet_lost('RX'):
            return
        worker = self._packet_worker(m)
        if worker is None:
            return
        lag = self.packet_delay('RX')
        if lag == 0:
            worker.mavlink_packet(m)
            return
        self.delay_sequence += 1
        deadline = max(time.monotonic() + lag, self.last_rx_deadline)
        self.last_rx_deadline = deadline
        heapq.heappush(self.rx_delay_queue,
                       (deadline, self.delay_sequence, worker, m))

    def idle_task(self):
        now = time.monotonic()
        while self.tx_delay_queue and self.tx_delay_queue[0][0] <= now:
            _, _, args = heapq.heappop(self.tx_delay_queue)
            self._transmit_payloads(*args)
        while self.rx_delay_queue and self.rx_delay_queue[0][0] <= now:
            _, _, worker, m = heapq.heappop(self.rx_delay_queue)
            if self.workers.get(worker.session) is worker:
                worker.mavlink_packet(m)
        for worker in list(self.workers.values()):
            worker.idle_task()

    def unload(self):
        self.cmd_cancel()
        super(FTPModule, self).unload()

def init(mpstate):
    '''initialise module'''
    return FTPModule(mpstate)
