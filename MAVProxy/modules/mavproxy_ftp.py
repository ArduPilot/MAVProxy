#!/usr/bin/env python3
'''mavlink file transfer support'''

import time
import random
import heapq
import socket

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink.mavftp import (
    FTP_OP,
    FtpError,
    HDR_Len,
    MAX_FTP_NAME,
    MAX_NETWORK_BATCH,
    MAX_Payload,
    MAVFTP,
    MAVLinkBatchWriter,
    OP_Ack,
    OP_BurstReadFile,
    OP_CalcFileCRC32,
    OP_CreateDirectory,
    OP_CreateFile,
    OP_ListDirectory,
    OP_ListDirectoryWithTime,
    OP_Nack,
    OP_None,
    OP_OpenFileRO,
    OP_OpenFileWO,
    OP_ReadFile,
    OP_RemoveDirectory,
    OP_RemoveFile,
    OP_Rename,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_TruncateFile,
    OP_WriteFile,
)


def encode_filename(name):
    '''Validate a legacy MAVProxy FTP path before submitting an operation.'''
    encoded = name.encode('ascii')
    if not encoded or b'\0' in encoded or len(encoded) > MAX_FTP_NAME:
        raise ValueError(
            'FTP filename must be 1..%u ASCII bytes without NULs' % MAX_FTP_NAME)
    return bytearray(encoded)


# Compatibility aliases retained for MAVProxy callers that imported the old
# module-level error constants.  Protocol definitions themselves now live in
# pymavlink.mavftp.
ERR_None = FtpError.Success
ERR_Fail = FtpError.Fail
ERR_FailErrno = FtpError.FailErrno
ERR_InvalidDataSize = FtpError.InvalidDataSize
ERR_InvalidSession = FtpError.InvalidSession
ERR_NoSessionsAvailable = FtpError.NoSessionsAvailable
ERR_EndOfFile = FtpError.EndOfFile
ERR_UnknownCommand = FtpError.UnknownCommand
ERR_FileExists = FtpError.FileExists
ERR_FileProtected = FtpError.FileProtected
ERR_FileNotFound = FtpError.FileNotFound

# A retired session may still have packets in a real link's buffers.  This is
# longer than ArduPilot's FTP session expiry and prevents a stale reply or
# request being routed to a new operation after the uint8 session ID wraps.
SESSION_REUSE_DELAY = 30.0


class FTPWorker(MAVFTP):
    '''One framework-neutral MAVFTP operation owned by FTPModule.

    The MAVProxy module allocates the session ID, queues work, and routes
    messages.  This adapter only supplies the MAVProxy transport and console
    boundary to pymavlink's protocol state machine.
    '''
    def __init__(self, manager, session, target_system=None,
                 target_component=None):
        self.manager = manager
        self.ftp_target_system = (manager.target_system if target_system is None
                                  else target_system)
        self.ftp_target_component = (
            manager.target_component if target_component is None
            else target_component)
        self.operation_name = 'ftp'
        super(FTPWorker, self).__init__(
            manager.master,
            self.ftp_target_system,
            self.ftp_target_component,
            settings=manager.ftp_settings,
            session=session,
            reset_sessions=False,
            send_payloads=self._send_payloads,
            operation_callback=self._operation_finished,
            source_system=manager.settings.source_system,
            source_component=manager.settings.source_component)
        self.list_time_supported = manager.list_time_supported.get(
            (self.ftp_target_system, self.ftp_target_component))

    def _send_payloads(self, payloads):
        '''Give a batch of raw FTP payloads to the shared MAVProxy link.'''
        self.manager.send_payloads(self, payloads)

    def _operation_finished(self, result):
        '''Release the scheduler slot once the core reports completion.'''
        if self.list_time_supported is not None:
            self.manager.list_time_supported[
                (self.ftp_target_system, self.ftp_target_component)] = \
                self.list_time_supported
        self.manager.worker_done(self)

    def _invalid_start(self, result, callback=None, progress_callback=None,
                       notify_progress=True):
        '''Preserve MAVProxy's callback contract for local validation errors.'''
        if result.error_code == FtpError.Success:
            return result
        if callback is not None:
            callback(None)
        if notify_progress and progress_callback is not None:
            progress_callback(None)
        return result

    def cmd_list(self, args):
        return super(FTPWorker, self).cmd_list(args, wait=False)

    def cmd_get(self, args, callback=None, callback_progress=None,
                max_size=None):
        # MAVProxy's established download-progress API exposes the in-memory
        # file and its current byte count.  pymavlink's generic API reports a
        # single fraction instead, so adapt at this boundary rather than
        # changing mission, parameter, and third-party MAVProxy callers.
        def legacy_progress(_proportion):
            if _proportion is not None:
                callback_progress(self.fh, self.read_total)

        progress_callback = (
            legacy_progress if callback_progress is not None else None)
        result = super(FTPWorker, self).cmd_get(
            args, callback=callback, progress_callback=progress_callback,
            max_size=max_size)
        # Historical get callbacks never received a one-argument failure
        # notification; the completion callback reports failure with None.
        return self._invalid_start(result, callback, notify_progress=False)

    def cmd_put(self, args, fh=None, callback=None, progress_callback=None):
        result = super(FTPWorker, self).cmd_put(
            args, fh=fh, callback=callback, progress_callback=progress_callback)
        return self._invalid_start(result, callback, progress_callback)

    def cmd_rm(self, args):
        return super(FTPWorker, self).cmd_rm(args, wait=False)

    def cmd_rmdir(self, args):
        return super(FTPWorker, self).cmd_rmdir(args, wait=False)

    def cmd_rename(self, args):
        return super(FTPWorker, self).cmd_rename(args, wait=False)

    def cmd_mkdir(self, args):
        return super(FTPWorker, self).cmd_mkdir(args, wait=False)

    def cmd_crc(self, args):
        return super(FTPWorker, self).cmd_crc(args, wait=False)

    def cmd_crclocal(self, args):
        return super(FTPWorker, self).cmd_crclocal(args)

    def cmd_crccmp(self, args):
        return super(FTPWorker, self).cmd_crccmp(args, wait=False)

    def terminate_session(self, outcome='failed'):
        '''End this worker without allowing a reply loop to block MAVProxy.'''
        self.manager.discard_delayed(self)
        return super(FTPWorker, self).terminate_session()

    def mavlink_packet(self, message):
        list_length = len(self.list_temp_result)
        diagnostic_length = len(self.list_diagnostics)
        result = super(FTPWorker, self).mavlink_packet(message)
        if result is not None and result.error_code == FtpError.NoSessionsAvailable:
            self.session_waiting = True
        if self.list_time_supported is not None:
            self.manager.list_time_supported[
                (self.ftp_target_system, self.ftp_target_component)] = \
                self.list_time_supported
        for entry in self.list_temp_result[list_length:]:
            if entry.is_dir:
                suffix = '/' if entry.mtime is None else '/\t-'
                print("   %s%s" % (entry.name, suffix))
            elif entry.mtime is None:
                print("   %s\t%u" % (entry.name, entry.size_b))
            elif entry.mtime == 0:
                print("   %s\t%u\t-" % (entry.name, entry.size_b))
            else:
                print("   %s\t%u\t%u" %
                      (entry.name, entry.size_b, entry.mtime))
        for diagnostic in self.list_diagnostics[diagnostic_length:]:
            print(diagnostic)
        return result


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
             ('write_qsize', int, 5),
             ('read_retry_time', float, 1.0),
             ('idle_detection_time', float, 3.7),
             ('retry_time', float, 0.5),
             ('crccmp_timeout', float, 120.0),
             ('list_time', int, 1),
             ('list_time_timeout', float, 3.0),
             ('list_retries', int, 3),
             # ArduPilot currently has five GCS_FTP server sessions.  Keeping
             # the cap configurable also supports smaller/custom servers.
             ('max_sessions', int, 5)])
        self.add_completion_function('(FTPSETTING)',
                                     self.ftp_settings.completion)
        self.workers = {}
        self.pending = []
        # Cache ListDirectoryWithTime support independently for each target.
        self.list_time_supported = {}
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
        self.retired_sessions = {}
        self._last_console_status_time = 0.0
        self._console_status_active = False

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
        if not collector.packets:
            return

        port = getattr(link, 'port', None)
        port_type = getattr(port, 'type', None)
        link_name = type(link).__name__
        is_stream = (link_name in ('mavtcp', 'mavtcpin') or
                     port_type == socket.SOCK_STREAM)
        is_network = (link_name == 'mavudp' or
                      port_type == socket.SOCK_DGRAM or is_stream)
        if not is_network:
            # Serial writes benefit substantially from being combined into a
            # single USB transfer.
            self._write_link_data(link, b''.join(collector.packets), False)
            return

        batch = bytearray()
        for packet in collector.packets:
            if batch and len(batch) + len(packet) > MAX_NETWORK_BATCH:
                self._write_link_data(link, bytes(batch), is_stream)
                batch = bytearray()
            batch.extend(packet)
        if batch:
            self._write_link_data(link, bytes(batch), is_stream)

    def _write_link_data(self, link, data, is_stream):
        '''Write one encoded batch without ignoring partial stream writes.'''
        if is_stream:
            port = getattr(link, 'port', None)
            if port is None and hasattr(link, 'reconnect'):
                try:
                    link.reconnect()
                except OSError:
                    return
                port = getattr(link, 'port', None)
            if port is not None and hasattr(port, 'sendall'):
                try:
                    port.sendall(data)
                except OSError:
                    if hasattr(link, 'handle_disconnect'):
                        link.handle_disconnect()
                return

        offset = 0
        while offset < len(data):
            written = link.write(data[offset:])
            # Datagram and several pymavlink wrappers return None after a
            # complete write.  Integer-returning serial writers can be safely
            # resumed when they accept only part of the buffer.
            if written is None:
                return
            if written <= 0:
                return
            offset += written

    def send_payloads(self, worker, payloads):
        '''Apply outgoing loss/lag, preserving batches that survive.'''
        payloads = [bytes(payload) for payload in payloads
                    if not self.packet_lost('TX')]
        if not payloads:
            return
        args = (worker.master, worker.network, worker.ftp_target_system,
                worker.ftp_target_component, payloads)
        lag = self.packet_delay('TX')
        if lag == 0:
            self._transmit_payloads(*args)
            return
        self.delay_sequence += 1
        deadline = max(time.monotonic() + lag, self.last_tx_deadline)
        self.last_tx_deadline = deadline
        heapq.heappush(self.tx_delay_queue,
                       (deadline, self.delay_sequence, worker, args))

    def _packet_worker(self, m):
        try:
            session = m.payload[2]
        except (IndexError, TypeError):
            return None
        worker = self.workers.get(session)
        if worker is None:
            return None
        try:
            source_system = m.get_srcSystem()
            source_component = m.get_srcComponent()
        except AttributeError:
            # Retain compatibility with synthetic/older message wrappers that
            # do not expose source accessors.
            return worker
        if worker.ftp_target_system not in (0, source_system):
            return None
        if worker.ftp_target_component not in (0, source_component):
            return None
        return worker

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
        now = time.monotonic()
        self.retired_sessions = {
            session: deadline
            for session, deadline in self.retired_sessions.items()
            if deadline > now
        }
        for _ in range(256):
            session = self.next_session
            self.next_session = (self.next_session + 1) % 256
            if session not in self.workers and \
               session not in self.retired_sessions:
                return session
        return None

    def _launch(self, operation):
        session = self._allocate_session()
        if session is None:
            self.pending.insert(0, operation)
            return None
        worker = FTPWorker(
            self, session,
            target_system=operation['target_system'],
            target_component=operation['target_component'])
        worker.operation_name = operation['name']
        self.workers[session] = worker
        try:
            method = getattr(worker, operation['method'])
            method(*operation['args'], **operation['kwargs'])
        except Exception as error:
            print('FTP: unable to start %s: %s' % (operation['name'], error))
            try:
                worker.terminate_session()
            except Exception as cleanup_error:
                print('FTP: session cleanup failed: %s' % cleanup_error)
            finally:
                # Even a send or failure callback that raises must not retain
                # a session slot and starve later transfers.
                self.worker_done(worker)
            return worker
        # Bad arguments or a local-file error can return without sending.
        if worker.last_op is None:
            self.worker_done(worker)
        return worker

    def _submit(self, name, method, *args, target_system=None,
                target_component=None, **kwargs):
        operation = {
            'name': name,
            'method': method,
            'args': args,
            'kwargs': kwargs,
            'target_system': self.target_system if target_system is None else target_system,
            'target_component': self.target_component if target_component is None else target_component,
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
            deadline = time.monotonic() + SESSION_REUSE_DELAY
            # A configured lag can exceed the normal network quarantine.  Do
            # not reuse the ID before its delayed TerminateSession is sent.
            for queued_deadline, _, queued_worker, _ in self.tx_delay_queue:
                if queued_worker is worker:
                    deadline = max(deadline, queued_deadline + 1.0)
            self.retired_sessions[worker.session] = deadline
        self._start_pending()
        self._update_console_status(force=True)

    def _update_console_status(self, force=False):
        '''Publish one interactive transfer's status to MAVProxy's console.'''
        statuses = []
        for worker in self.workers.values():
            if worker.show_progress:
                status = worker.transfer_status()
                if status is not None:
                    statuses.append((worker.op_start or 0.0, status))
        if not statuses:
            if force or self._console_status_active:
                self.console.set_status('FTP', '', row=4)
            self._console_status_active = False
            self._last_console_status_time = 0.0
            return

        now = time.monotonic()
        if not force and now - self._last_console_status_time < 0.5:
            return
        _, status = max(statuses)
        self.console.set_status('FTP', status, row=4)
        self._last_console_status_time = now
        self._console_status_active = True

    def _start_pending(self):
        '''Start queued work when both a slot and a safe session ID exist.'''
        limit = self._session_limit()
        while self.pending and len(self.workers) < limit:
            operation = self.pending.pop(0)
            if self._launch(operation) is None:
                break

    def discard_delayed(self, worker):
        '''Discard simulated-link traffic belonging to a finished worker.'''
        self.tx_delay_queue = [
            item for item in self.tx_delay_queue if item[2] is not worker
        ]
        heapq.heapify(self.tx_delay_queue)
        self.rx_delay_queue = [
            item for item in self.rx_delay_queue if item[2] is not worker
        ]
        heapq.heapify(self.rx_delay_queue)

    def _session_limit(self):
        # Session ids are uint8.  Keep one value in reserve so allocation and
        # queuing remain well defined even with an accidental oversized setting.
        return min(255, max(1, int(self.ftp_settings.max_sessions)))

    def cmd_list(self, args):
        return self._submit('list', 'cmd_list', args)

    def cmd_get(self, args, callback=None, callback_progress=None,
                target_system=None, target_component=None, max_size=None):
        return self._submit('get', 'cmd_get', args,
                            callback=callback,
                            callback_progress=callback_progress,
                            max_size=max_size,
                            target_system=target_system,
                            target_component=target_component)

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
            # Keep the longstanding wording used by scripts and autotests.
            print("No transfer in progress")
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
            _, _, worker, args = heapq.heappop(self.tx_delay_queue)
            active = self.workers.get(worker.session) is worker
            terminal = all(payload[3] == OP_TerminateSession
                           for payload in args[-1])
            if active or terminal:
                self._transmit_payloads(*args)
        while self.rx_delay_queue and self.rx_delay_queue[0][0] <= now:
            _, _, worker, m = heapq.heappop(self.rx_delay_queue)
            if self.workers.get(worker.session) is worker:
                worker.mavlink_packet(m)
        for worker in list(self.workers.values()):
            worker.idle_task()
        self._start_pending()
        self._update_console_status()

    def unload(self):
        self.cmd_cancel()
        super(FTPModule, self).unload()


def init(mpstate):
    '''initialise module'''
    return FTPModule(mpstate)
