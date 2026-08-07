#!/usr/bin/env python3
'''mavlink file transfer support'''

import io
import time, os, sys
import glob
import struct
import random
import zlib
from pymavlink import mavutil

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

class FTPModule(mp_module.MPModule):
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
             ('max_backlog', int, 5),
             ('burst_read_size', int, 80),
             ('write_size', int, 80),
             ('write_qsize', int, 5),
             ('retry_time', float, 0.5),
             ('crccmp_timeout', float, 120.0)])
        self.add_completion_function('(FTPSETTING)',
                                     self.ftp_settings.completion)
        self.seq = 0
        self.session = 0
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
        # the listing in progress: the directory being listed, and when the
        # outstanding request went out (None when no listing is running)
        self.list_dname = None
        self.list_sent = None
        self.last_op_time = time.time()
        self.rtt = 0.5
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
        self.write_last_send = None
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

    def cmd_ftp(self, args):
        '''FTP operations'''
        usage = "Usage: ftp <list|get|put|rm|rmdir|rename|mkdir|crc|crclocal|crccmp>"
        if len(args) < 1:
            print(usage)
            return
        # these all talk to the vehicle and would have their replies consumed
        # by a running comparison, or clobber the op it is waiting on
        if self.crccmp_dest is not None and args[0] in (
                'list', 'crc', 'rm', 'rmdir', 'rename', 'mkdir', 'put'):
            print("crccmp in progress, use 'ftp cancel' to stop it")
            return
        if args[0] == 'list':
            self.cmd_list(args[1:])
        elif args[0] == "set":
            self.ftp_settings.command(args[1:])
        elif args[0] == 'get':
            self.cmd_get(args[1:])
        elif args[0] == 'put':
            self.cmd_put(args[1:])
        elif args[0] == 'rm':
            self.cmd_rm(args[1:])
        elif args[0] == 'rmdir':
            self.cmd_rmdir(args[1:])
        elif args[0] == 'rename':
            self.cmd_rename(args[1:])
        elif args[0] == 'mkdir':
            self.cmd_mkdir(args[1:])
        elif args[0] == 'crc':
            self.cmd_crc(args[1:])
        elif args[0] == 'crclocal':
            self.cmd_crclocal(args[1:])
        elif args[0] == 'crccmp':
            self.cmd_crccmp(args[1:])
        elif args[0] == 'status':
            self.cmd_status()
        elif args[0] == 'cancel':
            self.cmd_cancel()
        else:
            print(usage)

    def send(self, op):
        '''send a request'''
        op.seq = self.seq
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0]*((HDR_Len+MAX_Payload)-plen)))
        if self.master is None:
            print("FTP: Can't send request, no master...")
            return
        self.master.mav.file_transfer_protocol_send(self.network, self.target_system, self.target_component, payload)
        self.seq = (self.seq + 1) % 256
        self.last_op = op
        now = time.time()
        if self.ftp_settings.debug > 1:
            print("> %s dt=%.2f" % (op, now - self.last_op_time))
        self.last_op_time = time.time()

    def terminate_session(self, outcome="failed"):
        '''terminate current session. outcome describes an incomplete transfer
        for the status line: "cancelled" when the user or a new command ended
        it, "failed" for an error'''
        self.transfer_active = False
        self.abort_listing()
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
        self.session = (self.session + 1) % 256
        self.reached_eof = False
        self.backlog = 0
        self.duplicates = 0
        if self.ftp_settings.debug > 0:
            print("Terminated session")

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
        self.list_dname = enc_dname
        self.send_list_request()

    def send_list_request(self):
        '''request the next page of the listing in progress.  the listing is
        built from its own state rather than from the last op sent, which may
        by now belong to some other transfer'''
        self.list_sent = time.time()
        op = FTP_OP(self.seq, self.session, OP_ListDirectory, len(self.list_dname), 0, 0,
                    self.dir_offset, self.list_dname)
        self.send(op)

    def abort_listing(self):
        '''stop tracking any listing in progress, so a late reply does not
        fire into whatever comes next'''
        self.list_sent = None

    def handle_list_reply(self, op, m):
        '''handle OP_ListDirectory reply'''
        if self.list_sent is None:
            # a late reply to a listing which has since been abandoned
            return
        self.list_sent = None
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
                    # "F<name>\t<size>".  the size is the last field, so
                    # count from the end - a name may itself contain a tab
                    fields = d[1:].split('\t')
                    if len(fields) < 2:
                        print(d)
                        continue
                    name = '\t'.join(fields[:-1])
                    try:
                        size = int(fields[-1])
                    except ValueError:
                        print(d)
                        continue
                    self.total_size += size
                    print("   %s\t%u" % (name, size))
                else:
                    print(d)
            # ask for more
            self.send_list_request()
        elif op.opcode == OP_Nack and len(op.payload) == 1 and op.payload[0] == ERR_EndOfFile:
            print("Total size %.2f kByte" % (self.total_size / 1024.0))
            self.total_size = 0
        else:
            print('LIST: %s' % op)

    def cmd_get(self, args, callback=None, callback_progress=None):
        '''get file'''
        if len(args) == 0:
            print("Usage: get FILENAME <LOCALNAME>")
            return
        self.terminate_session("cancelled")
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
        if self.ftp_settings.pkt_loss_tx > 0:
            if random.uniform(0,100) < self.ftp_settings.pkt_loss_tx:
                if self.ftp_settings.debug > 0:
                    print("FTP: dropping TX")
                return
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
        if self.backlog > 0:
            self.backlog -= 1
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            if gap in self.read_gaps:
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
        # unlike get, put does not terminate the session first, so drop any
        # listing here rather than let it interleave with the upload
        self.abort_listing()
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
        self.write_last_send = None

        self.put_callback = callback
        self.put_callback_progress = progress_callback
        self.show_progress = callback is None
        self.transfer_active = True
        self.read_retries = 0
        self.op_start = time.time()
        enc_fname = bytearray(self.filename, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 0, 0, enc_fname)
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
            print("Sent file of length ", flen)
        self.finished_status("uploading", self.filename, flen)
        
    def handle_create_file_reply(self, op, m):
        '''handle OP_CreateFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode == OP_Ack:
            self.send_more_writes()
        else:
            print("Create failed")
            self.terminate_session()

    def send_more_writes(self):
        '''send some more writes'''
        if len(self.write_list) == 0:
            # all done
            self.put_finished(self.write_file_size)
            self.terminate_session()
            return

        now = time.time()
        if self.write_last_send is not None:
            if now - self.write_last_send > max(min(10*self.rtt, 1),0.2):
                # we seem to have lost a block of replies
                self.write_pending = max(0, self.write_pending-1)

        n = min(self.ftp_settings.write_qsize-self.write_pending, len(self.write_list))
        for i in range(n):
            # send in round-robin, skipping any that have been acked
            idx = self.write_idx
            while idx not in self.write_list:
                idx = (idx + 1) % self.write_total
            ofs = idx * self.write_block_size
            self.fh.seek(ofs)
            data = self.fh.read(self.write_block_size)
            write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
            self.send(write)
            self.write_idx = (idx + 1) % self.write_total
            self.write_pending += 1
            self.write_last_send = now

    def handle_write_reply(self, op, m):
        '''handle OP_WriteFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode != OP_Ack:
            print("Write failed")
            self.terminate_session()
            return

        # assume the FTP server processes the blocks sequentially. This means
        # when we receive an ack that any blocks between the last ack and this
        # one have been lost
        idx = op.offset // self.write_block_size
        count = (idx - self.write_recv_idx) % self.write_total

        self.write_pending = max(0, self.write_pending - count)
        self.write_recv_idx = idx
        if idx in self.write_list:
            # servers are required to resend replies to repeated requests, so
            # only count a block the first time it is acked
            self.write_list.discard(idx)
            self.write_acks += 1
            self.write_acked_bytes += self.write_block_len(idx)
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
        # as for put, this takes over the session without terminating it
        self.abort_listing()
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
            if self.ftp_settings.pkt_loss_rx > 0:
                if random.uniform(0,100) < self.ftp_settings.pkt_loss_rx:
                    if self.ftp_settings.debug > 1:
                        print("FTP: dropping packet RX")
                    return

            if op.req_opcode == self.last_op.opcode and op.seq == (self.last_op.seq + 1) % 256:
                self.rtt = max(min(self.rtt, dt), 0.01)
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
        self.read_gaps.remove(g)
        self.read_gaps.append(g)
        self.last_gap_send = time.time()
        self.read_gap_times[g] = self.last_gap_send
        self.backlog += 1

    def check_read_send(self):
        '''see if we should send another gap read'''
        if len(self.read_gaps) == 0:
            return
        g = self.read_gaps[0]
        now = time.time()
        dt = now - self.read_gap_times[g]
        if not self.reached_eof:
            # send gap reads once
            for g in self.read_gap_times.keys():
                if self.read_gap_times[g] == 0:
                    self.send_gap_read(g)
            return
        if self.read_gap_times[g] > 0 and dt > self.ftp_settings.retry_time:
            if self.backlog > 0:
                self.backlog -= 1
            self.read_gap_times[g] = 0

        if self.read_gap_times[g] != 0:
            # still pending
            return
        if not self.reached_eof and self.backlog >= self.ftp_settings.max_backlog:
            # don't fill queue too far until we have got past the burst
            return
        if now - self.last_gap_send < 0.05:
            # don't send too fast
            return
        self.send_gap_read(g)

    def idle_task(self):
        '''check for file gaps and lost requests'''
        now = time.time()

        # ahead of the early returns below, which skip idle transfers
        self.update_status()

        if self.crccmp_sent is not None and \
           now - self.crccmp_sent > self.ftp_settings.crccmp_timeout:
            self.crccmp_record('TIMEOUT', os.path.basename(self.crccmp_local))
            self.crccmp_sent = None
            self.crccmp_expect = None
            self.crccmp_next()

        # see if we lost an open reply
        if self.op_start is not None and now - self.op_start > 1.0 and self.last_op.opcode == OP_OpenFileRO:
            self.op_start = now
            self.open_retries += 1
            if self.open_retries > 2:
                # fail the get
                self.op_start = None
                self.terminate_session()
                return
            if self.ftp_settings.debug > 0:
                print("FTP: retry open")
            send_op = self.last_op
            self.send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
            self.session = (self.session + 1) % 256
            send_op.session = self.session
            self.send(send_op)

        if len(self.read_gaps) == 0 and self.last_burst_read is None and self.write_list is None:
            return

        if self.fh is None:
            return

        # see if burst read has stalled
        if not self.reached_eof and self.last_burst_read is not None and now - self.last_burst_read > self.ftp_settings.retry_time:
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

def init(mpstate):
    '''initialise module'''
    return FTPModule(mpstate)
