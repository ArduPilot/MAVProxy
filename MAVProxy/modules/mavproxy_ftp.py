#!/usr/bin/env python
'''mavlink file transfer support'''

import time, os, sys
import struct
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

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
        return struct.pack("<HBBBBBBI", self.seq, self.session, self.opcode, self.size, self.req_opcode, self.burst_complete, 0, self.offset) + self.payload

    def __str__(self):
        ret = "OP seq:%u sess:%u opcode:%d req_opcode:%u size:%u bc:%u ofs:%u plen=%u" % (self.seq,
                                                                                          self.session,
                                                                                          self.opcode,
                                                                                          self.req_opcode,
                                                                                          self.size,
                                                                                          self.burst_complete,
                                                                                          self.offset,
                                                                                          len(self.payload))
        if len(self.payload) > 0:
            ret += " [%u]" % self.payload[0]
        return ret


class FTPModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FTPModule, self).__init__(mpstate, "ftp")
        self.add_command('ftp', self.cmd_ftp, "file transfer",
                         ["<list|get|put|rm|rmdir|rename|mkdir|crc|status>"])
        self.seq = 0
        self.session = 0
        self.network = 0
        self.last_op = None
        self.fh = None
        self.filename = None
        self.total_size = 0
        self.read_gaps = set()
        self.last_read = None
        self.last_burst_read = None

    def cmd_ftp(self, args):
        '''FTP operations'''
        usage = "Usage: ftp <list|get|put|rm|rmdir|rename|mkdir|crc>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == 'list':
            self.cmd_list(args[1:])
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
        elif args[0] == 'status':
            self.cmd_status()
        else:
            print(usage)

    def send(self, op):
        '''send a request'''
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0]*((HDR_Len+MAX_Payload)-plen)))
        self.master.mav.file_transfer_protocol_send(self.network, self.target_system, self.target_component, payload)
        self.seq = (self.seq + 1) % 255
        self.last_op = op

    def terminate_session(self):
        '''terminate current session'''
        self.send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, bytearray([])))
        self.fh = None
        self.filename = None
        self.read_gaps = set()
        self.last_read = None
        self.last_burst_read = None

    def cmd_list(self, args):
        '''list files'''
        if len(args) > 0:
            dname = args[0]
        else:
            dname = '/'
        print("Listing %s" % dname)
        enc_dname = bytearray(dname, 'ascii')
        self.total_size = 0
        op = FTP_OP(self.seq, self.session, OP_ListDirectory, len(enc_dname), 0, 0, 0, enc_dname)
        self.send(op)

    def handle_list_reply(self, op, m):
        '''handle OP_ListDirectory reply'''
        if op.opcode == OP_Ack:
            dentries = sorted(op.payload.split(b'\x00'))
            #print(dentries)
            for d in dentries:
                if len(d) == 0:
                    continue
                d = str(d)
                if d[0] == 'D':
                    print(" D %s" % d[1:])
                elif d[0] == 'F':
                    (name, size) = d[1:].split('\t')
                    size = int(size)
                    self.total_size += size
                    print("   %s\t%u" % (name, size))
                else:
                    print(str(d))
            # ask for more
            more = self.last_op
            more.offset = self.last_op.offset + op.size
            self.send(more)
        elif op.opcode == OP_Nack and len(op.payload) == 1 and op.payload[0] == ERR_EndOfFile:
            print("Total size %.2f kByte" % (self.total_size / 1024.0))
            self.total_size = 0
        else:
            print('LIST: %s' % op)

    def cmd_get(self, args):
        '''get file'''
        if len(args) == 0:
            print("Usage: get FILENAME <LOCALNAME>")
            return
        fname = args[0]
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        print("Getting %s as %s" % (fname, self.filename))
        enc_fname = bytearray(fname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def handle_open_RO_reply(self, op, m):
        '''handle OP_OpenFileRO reply'''
        if op.opcode == OP_Ack:
            if self.filename is None:
                return
            try:
                self.fh = open(self.filename, 'wb')
            except Exception as ex:
                print("Failed to open %s: %s" % (self.filename, ex))
                self.terminate_session()
                return
            read = FTP_OP(self.seq, self.session, OP_BurstReadFile, op.size, 0, 0, 0, op.payload)
            self.last_burst_read = time.time()
            self.send(read)
        else:
            print("Open failed")
            self.terminate_session()

    def handle_burst_read(self, op, m):
        '''handle OP_BurstReadFile reply'''
        #import random
        #if random.uniform(0,100) > 70:
        #    print("dropping %s" % op)
        #    return
        if self.fh is None or self.filename is None:
            print("FTP Unexpected burst read reply")
            print(op)
            return
        if op.opcode == OP_Ack and self.fh is not None:
            ofs = self.fh.tell()
            if op.offset < ofs:
                # writing an earlier portion, possibly remove a gap
                gap = (op.offset, len(op.payload))
                if gap in self.read_gaps:
                    self.read_gaps.remove(gap)
                self.fh.seek(op.offset)
                self.fh.write(op.payload)
                self.fh.seek(ofs)
            elif op.offset > ofs:
                # we have a gap
                gap = (ofs, op.offset-ofs)
                while True:
                    if gap[1] <= MAX_Payload:
                        self.read_gaps.add(gap)
                        break
                    self.read_gaps.add((gap[0], MAX_Payload))
                    gap = (gap[0] + MAX_Payload, gap[1] - MAX_Payload)
                self.fh.seek(op.offset)
                self.fh.write(op.payload)
            else:
                self.fh.write(op.payload)
            if op.burst_complete:
                more = self.last_op
                more.offset = op.offset
                self.last_burst_read = time.time()
                self.send(more)
        elif op.opcode == OP_Nack:
            ecode = op.payload[0]
            if ecode == ERR_EndOfFile or ecode == 0:
                if len(self.read_gaps) == 0:
                    print("Wrote %u bytes to %s (ecode %u)" % (self.fh.tell(), self.filename, ecode))
                    self.terminate_session()
            else:
                print("FTP: burst Nack: %s" % op)
        else:
            print("FTP: burst error: %s" % op)

    def handle_reply_read(self, op, m):
        '''handle OP_ReadFile reply'''
        if self.fh is None or self.filename is None:
            print("FTP Unexpected read reply")
            print(op)
            return
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            if gap in self.read_gaps:
                self.read_gaps.remove(gap)
                ofs = self.fh.tell()
                self.fh.seek(op.offset)
                self.fh.write(op.payload)
                self.fh.seek(ofs)
        elif op.opcode == OP_Nack:
            print("Read failed with %u gaps" % len(self.read_gaps))
            self.terminate_session()
            
    def cmd_put(self, args):
        '''put file'''
        if len(args) == 0:
            print("Usage: put FILENAME <REMOTENAME>")
            return
        fname = args[0]
        try:
            self.fh = open(fname, 'rb')
        except Exception as ex:
            print("Failed to open %s: %s" % (fname, ex))
            return
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        print("Putting %s as %s" % (fname, self.filename))
        enc_fname = bytearray(self.filename, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def handle_create_file_reply(self, op, m):
        '''handle OP_CreateFile reply'''
        if self.fh is None:
            print("FTP file not open")
            self.terminate_session()
            return
        if op.opcode == OP_Ack:
            ofs = self.fh.tell()
            try:
                data = self.fh.read(MAX_Payload)
            except Exception as ex:
                print("Read error: %s" % ex)
                self.terminate_session()
                return
            if len(data) > 0:
                write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
                self.send(write)
            if len(data) < MAX_Payload:
                t = self.fh.tell()
                print("Sent file of length ", t)
                self.terminate_session()
        else:
            print("Create failed")
            self.terminate_session()

    def handle_write_reply(self, op, m):
        '''handle OP_WriteFile reply'''
        if self.fh is None:
            print("FTP file not open")
            self.terminate_session()
            return
        if op.opcode == OP_Ack:
            ofs = self.fh.tell()
            if self.last_op.size < MAX_Payload:
                print("Sent file of length %u" % op.offset)
                self.terminate_session()
                return

            try:
                data = self.fh.read(MAX_Payload)
            except Exception as ex:
                print("Read error: %s" % ex)
                self.terminate_session()
                return

            if len(data) > 0:
                write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
                self.send(write)
            else:
                print("Sent file of length %u" % self.fh.tell())
                self.terminate_session()
        else:
            print("Write failed")
            print(str(op), op.payload)
            self.terminate_session()

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
        print("Getting CRC for %s" % name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CalcFileCRC32, len(enc_name), 0, 0, 0, bytearray(enc_name))
        self.send(op)

    def cmd_status(self):
        '''show status'''
        if self.fh is None:
            print("No transfer in progress")
        else:
            print("Transfer at offset %u with %u gaps" % (self.fh.tell(), len(self.read_gaps)))

    def handle_crc_reply(self, op, m):
        '''handle crc reply'''
        if op.opcode == OP_Ack and op.size == 4:
            crc = struct.unpack("<I", op.payload)
            print("crc: 0x%08x" % crc)
        else:
            print("crc failed %s" % op)
            
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
            op = self.op_parse(m)
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

    def idle_task(self):
        '''check for file gaps'''
        if len(self.read_gaps) == 0 and self.last_burst_read is None:
            return
        if self.fh is None:
            return

        # see if burst read has stalled
        now = time.time()
        if now - self.last_burst_read > 1:
            self.last_burst_read = now
            self.send(FTP_OP(self.seq, self.session, OP_BurstReadFile, self.last_op.size, 0, 0, self.fh.tell(), self.last_op.payload))

        # see if we can fill gaps
        if len(self.read_gaps) > 0 and (self.last_read is None or now - self.last_read > 0.1):
            self.last_read = now
            (offset, length) = self.read_gaps.pop()
             # we add it back into gaps until we get it acked
            self.read_gaps.add((offset, length))
            self.last_read = now
            read = FTP_OP(self.seq, self.session, OP_ReadFile, length, 0, 0, offset, bytearray([]))
            self.send(read)


def init(mpstate):
    '''initialise module'''
    return FTPModule(mpstate)
