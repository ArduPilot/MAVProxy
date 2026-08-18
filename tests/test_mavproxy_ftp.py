import importlib.util
import contextlib
import io
from pathlib import Path
import struct
import socket
import time
import types
import unittest


# Some developer environments have a released MAVProxy imported by a pytest
# plugin before collection begins.  Load the worktree file explicitly so the
# tests always exercise the code they accompany.
FTP_MODULE_PATH = (Path(__file__).resolve().parents[1] /
                   'MAVProxy/modules/mavproxy_ftp.py')
FTP_SPEC = importlib.util.spec_from_file_location(
    'mavproxy_ftp_under_test', FTP_MODULE_PATH)
mavproxy_ftp = importlib.util.module_from_spec(FTP_SPEC)
FTP_SPEC.loader.exec_module(mavproxy_ftp)


class FakeMAV:
    def __init__(self):
        self.sent = []

    def file_transfer_protocol_send(self, network, target_system,
                                    target_component, payload):
        self.sent.append(bytes(payload))


class FakeMaster:
    def __init__(self):
        self.mav = FakeMAV()


class FakeLink:
    def __init__(self, port_type=None):
        self.writes = []
        self.port = types.SimpleNamespace(type=port_type)

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)


class FakeStreamPort:
    type = socket.SOCK_STREAM

    def __init__(self):
        self.writes = []

    def sendall(self, data):
        self.writes.append(bytes(data))


class FakeBatchMAV:
    '''Minimal MAVLink encoder whose output can be collected by the module.'''
    def __init__(self, link):
        self.file = link

    def file_transfer_protocol_send(self, network, target_system,
                                    target_component, payload):
        self.file.write(b'F' + bytes(payload))


class FakeConsole:
    def __init__(self):
        self.status = {}

    def set_status(self, name, value, row=None):
        self.status[name] = (value, row)


class FakeMPState:
    def __init__(self):
        self.public_modules = {}
        self.command_map = {}
        self.completions = {}
        self.completion_functions = {}
        self.settings = types.SimpleNamespace(
            target_system=1,
            target_component=1,
            source_system=255,
            source_component=0,
        )
        self.console = FakeConsole()
        self._master = FakeMaster()

    def master(self):
        return self._master


class FTPMessage:
    def __init__(self, payload):
        self.payload = payload
        self.target_system = 255
        self.target_component = 0

    def get_type(self):
        return "FILE_TRANSFER_PROTOCOL"


def reply(session, req_opcode, opcode=mavproxy_ftp.OP_Ack, payload=b'',
          offset=0, burst_complete=0, seq=1):
    header = struct.pack(
        '<HBBBBBBI', seq, session, opcode, len(payload), req_opcode,
        burst_complete, 0, offset)
    return FTPMessage(bytearray(header + payload).ljust(251, b'\x00'))


def request_session(payload):
    return payload[2]


def request_opcode(payload):
    return payload[3]


class TestConcurrentFTP(unittest.TestCase):
    def setUp(self):
        self.mpstate = FakeMPState()
        self.ftp = mavproxy_ftp.FTPModule(self.mpstate)
        self.ftp.next_session = 0

    def test_idle_status_keeps_script_compatible_wording(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.ftp.cmd_status()
        self.assertEqual(output.getvalue(), "No transfer in progress\n")

    def test_gets_and_list_have_independent_sessions_and_state(self):
        results = {}

        def completed(name):
            def callback(fh):
                results[name] = None if fh is None else fh.read()
            return callback

        self.ftp.cmd_get(['/first'], callback=completed('first'))
        self.ftp.cmd_list(['/'])
        self.ftp.cmd_get(['/second'], callback=completed('second'))

        sent = self.mpstate._master.mav.sent
        sessions = [request_session(packet) for packet in sent]
        self.assertEqual(sessions, [0, 1, 2])
        self.assertEqual(len(self.ftp.workers), 3)

        # Both opens may complete around a directory reply.  Each worker must
        # create and advance only its own in-memory file.
        self.ftp.mavlink_packet(reply(2, mavproxy_ftp.OP_OpenFileRO,
                                      payload=struct.pack('<I', 6)))
        self.ftp.mavlink_packet(reply(0, mavproxy_ftp.OP_OpenFileRO,
                                      payload=struct.pack('<I', 5)))
        self.ftp.mavlink_packet(reply(1, mavproxy_ftp.OP_ListDirectoryWithTime,
                                      opcode=mavproxy_ftp.OP_Nack,
                                      payload=bytes([mavproxy_ftp.ERR_EndOfFile])))

        self.assertNotIn(1, self.ftp.workers)
        self.assertIn(0, self.ftp.workers)
        self.assertIn(2, self.ftp.workers)

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_BurstReadFile, payload=b'first',
            burst_complete=1))
        self.ftp.mavlink_packet(reply(
            2, mavproxy_ftp.OP_BurstReadFile, payload=b'second',
            burst_complete=1))

        self.assertEqual(results, {'first': b'first', 'second': b'second'})
        self.assertEqual(self.ftp.workers, {})

    def test_sixth_operation_is_queued_until_a_session_finishes(self):
        for _ in range(6):
            self.ftp.cmd_list(['/'])

        self.assertEqual(len(self.ftp.workers), 5)
        self.assertEqual(len(self.ftp.pending), 1)
        self.assertEqual(len(self.mpstate._master.mav.sent), 5)

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_ListDirectoryWithTime,
            opcode=mavproxy_ftp.OP_Nack,
            payload=bytes([mavproxy_ftp.ERR_EndOfFile])))

        self.assertEqual(len(self.ftp.workers), 5)
        self.assertEqual(self.ftp.pending, [])
        self.assertEqual(len(self.mpstate._master.mav.sent), 7)
        # One packet terminated session 0; the other launched queued session 5.
        self.assertEqual(request_session(self.mpstate._master.mav.sent[-1]), 5)

    def test_server_session_exhaustion_waits_and_retries(self):
        callbacks = []
        self.ftp.cmd_get(['/params'], callback=callbacks.append)
        worker = self.ftp.workers[0]

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_OpenFileRO,
            opcode=mavproxy_ftp.OP_Nack,
            payload=bytes([mavproxy_ftp.ERR_NoSessionsAvailable])))

        self.assertTrue(worker.session_waiting)
        self.assertEqual(callbacks, [])
        self.assertIs(self.ftp.workers[0], worker)

        self.ftp.ftp_settings.retry_time = 0
        worker.last_op_time -= worker.retry_timeout()
        sent_before = len(self.mpstate._master.mav.sent)
        self.ftp.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), sent_before + 1)
        self.assertEqual(request_session(self.mpstate._master.mav.sent[-1]), 0)
        self.assertEqual(callbacks, [])

    def test_silently_dropped_initial_request_is_retried_idempotently(self):
        self.ftp.cmd_list(['/'])
        worker = self.ftp.workers[0]
        first_packet = self.mpstate._master.mav.sent[-1]
        first_seq = struct.unpack('<H', first_packet[:2])[0]

        # No reply arrives, as happens when ArduPilot's request queue is full.
        worker.last_op_time -= max(
            worker.retry_timeout(), self.ftp.ftp_settings.list_time_timeout) + 0.01
        self.ftp.idle_task()

        retried_packet = self.mpstate._master.mav.sent[-1]
        retried_seq = struct.unpack('<H', retried_packet[:2])[0]
        self.assertEqual(retried_seq, first_seq)
        self.assertEqual(request_session(retried_packet), 0)
        self.assertEqual(worker.request_retries, 1)
        self.assertEqual(len(self.ftp.workers), 1)

    def test_list_with_time_parses_names_from_the_end(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.ftp.cmd_list(['/'])
            self.assertEqual(
                request_opcode(self.mpstate._master.mav.sent[-1]),
                mavproxy_ftp.OP_ListDirectoryWithTime)

            self.ftp.mavlink_packet(reply(
                0, mavproxy_ftp.OP_ListDirectoryWithTime,
                payload=b'Ftab\tname\t42\t0\x00Fmalformed\x00'))

        continuation = self.mpstate._master.mav.sent[-1]
        self.assertEqual(request_opcode(continuation),
                         mavproxy_ftp.OP_ListDirectoryWithTime)
        self.assertEqual(struct.unpack('<I', continuation[8:12])[0], 2)
        self.assertIn('tab\tname\t42\t-', output.getvalue())
        self.assertIn('Fmalformed', output.getvalue())
        self.assertTrue(self.ftp.list_time_supported[(1, 1)])

    def test_list_time_nack_falls_back_and_is_cached(self):
        self.ftp.cmd_list(['/'])
        self.assertEqual(request_opcode(self.mpstate._master.mav.sent[-1]),
                         mavproxy_ftp.OP_ListDirectoryWithTime)

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_ListDirectoryWithTime,
            opcode=mavproxy_ftp.OP_Nack,
            payload=bytes([mavproxy_ftp.ERR_UnknownCommand])))

        fallback = self.mpstate._master.mav.sent[-1]
        self.assertEqual(request_opcode(fallback),
                         mavproxy_ftp.OP_ListDirectory)
        self.assertFalse(self.ftp.list_time_supported[(1, 1)])

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_ListDirectory,
            opcode=mavproxy_ftp.OP_Nack,
            payload=bytes([mavproxy_ftp.ERR_EndOfFile]), seq=2))
        self.ftp.cmd_list(['/other'])
        self.assertEqual(request_opcode(self.mpstate._master.mav.sent[-1]),
                         mavproxy_ftp.OP_ListDirectory)

    def test_ignored_list_time_opcode_retries_then_falls_back(self):
        self.ftp.ftp_settings.list_retries = 1
        self.ftp.ftp_settings.list_time_timeout = 0
        self.ftp.cmd_list(['/'])
        worker = self.ftp.workers[0]
        first_seq = struct.unpack('<H', self.mpstate._master.mav.sent[-1][:2])[0]

        worker.last_op_time -= max(
            worker.retry_timeout(), self.ftp.ftp_settings.list_time_timeout) + 0.01
        self.ftp.idle_task()
        retry = self.mpstate._master.mav.sent[-1]
        self.assertEqual(request_opcode(retry),
                         mavproxy_ftp.OP_ListDirectoryWithTime)
        self.assertEqual(struct.unpack('<H', retry[:2])[0], first_seq)

        worker.last_op_time -= worker.retry_timeout() + 0.01
        self.ftp.idle_task()
        self.assertEqual(request_opcode(self.mpstate._master.mav.sent[-1]),
                         mavproxy_ftp.OP_ListDirectory)

        # A late reply to the abandoned capability probe must not advance the
        # fallback listing or be interpreted using the wrong entry format.
        sent = len(self.mpstate._master.mav.sent)
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_ListDirectoryWithTime,
            payload=b'Fold\t1\t0\x00', seq=1))
        self.assertEqual(len(self.mpstate._master.mav.sent), sent)
        self.assertEqual(worker.dir_offset, 0)

    def test_cumulative_write_ack_completes_contiguous_batch(self):
        self.ftp.ftp_settings.write_size = 2
        self.ftp.ftp_settings.write_qsize = 4
        self.ftp.ftp_settings.write_batch_size = 2
        completed = []
        self.ftp.cmd_put(['unused', '/remote'], fh=io.BytesIO(b'abcdefgh'),
                         callback=completed.append)

        create = self.mpstate._master.mav.sent[0]
        self.assertEqual(create[6], 1)
        self.ftp.mavlink_packet(reply(0, mavproxy_ftp.OP_CreateFile))
        worker = self.ftp.workers[0]
        self.assertEqual(worker.write_pending, 4)

        # One negotiated ACK covers blocks 0 through 3.
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_WriteFile, offset=6,
            payload=struct.pack('<I', 0), burst_complete=1, seq=5))

        self.assertEqual(completed, [8])
        self.assertNotIn(0, self.ftp.workers)

    def test_cumulative_write_ack_can_expand_server_window(self):
        self.ftp.ftp_settings.write_size = 2
        self.ftp.cmd_put(['unused', '/remote'], fh=io.BytesIO(b'abcdefgh'))
        capabilities = mavproxy_ftp.WRITE_CAPABILITY_MAGIC + bytes([1, 8])
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_CreateFile, payload=capabilities))
        worker = self.ftp.workers[0]
        self.assertEqual(worker.write_qsize, 1)

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_WriteFile, offset=0,
            payload=struct.pack('<I', 0) + bytes([60]),
            burst_complete=1, seq=2))

        self.assertEqual(worker.write_qsize, 60)
        self.assertGreater(worker.write_pending, 1)

    def test_write_window_uses_server_capabilities_in_auto_mode(self):
        self.ftp.cmd_put(['unused', '/remote'], fh=io.BytesIO(b'abc'))
        worker = self.ftp.workers[0]
        self.assertEqual(worker.write_qsize, 5)
        self.assertEqual(worker.write_batch_size, 1)

        # A delayed or lost CreateFile must not let idle processing send file
        # data before the remote file exists.
        sent = len(self.mpstate._master.mav.sent)
        for _ in range(10):
            worker.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), sent)

        # The obsolete one-byte marker is not enough to enable acceleration;
        # a legacy server could coincidentally return these bytes.
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_CreateFile, payload=b'\xA5\x40\x08'))
        self.assertEqual(worker.write_qsize, 5)
        self.assertEqual(worker.write_batch_size, 1)

        # Restart with an unambiguous versioned capability reply.
        worker.write_open = False
        capabilities = mavproxy_ftp.WRITE_CAPABILITY_MAGIC + bytes([60, 8])
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_CreateFile, payload=capabilities))

        self.assertEqual(worker.write_qsize, 60)
        self.assertEqual(worker.write_batch_size, 8)
        sent = len(self.mpstate._master.mav.sent)
        for _ in range(10):
            worker.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), sent)

    def test_cumulative_ack_does_not_cover_a_gap_before_its_start(self):
        self.ftp.ftp_settings.write_size = 2
        self.ftp.ftp_settings.write_qsize = 6
        self.ftp.ftp_settings.write_batch_size = 6
        self.ftp.cmd_put(['unused', '/remote'],
                         fh=io.BytesIO(b'abcdefghijkl'))
        self.ftp.mavlink_packet(reply(0, mavproxy_ftp.OP_CreateFile))
        worker = self.ftp.workers[0]

        # The server received and wrote only blocks 2 and 3. Blocks 0 and 1
        # must remain queued even though the cumulative ACK ends after them.
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_WriteFile, offset=6,
            payload=struct.pack('<I', 4), burst_complete=1, seq=5))

        self.assertEqual(worker.write_list, {0, 1, 4, 5})
        self.assertEqual(worker.write_pending, 4)
        self.assertEqual(worker.write_acked_bytes, 4)

    def test_packet_loss_uses_actual_link_directions(self):
        self.ftp.ftp_settings.pkt_loss_tx = 100
        self.ftp.cmd_list(['/'])
        worker = self.ftp.workers[0]

        # TX loss makes the request invisible to the transport while keeping
        # enough protocol state to retry it.
        self.assertEqual(self.mpstate._master.mav.sent, [])
        self.assertEqual(worker.seq, 1)
        self.ftp.ftp_settings.pkt_loss_tx = 0
        worker.last_op_time -= max(
            worker.retry_timeout(), self.ftp.ftp_settings.list_time_timeout) + 0.01
        self.ftp.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), 1)

        # RX loss must likewise be invisible to the worker.
        eof = reply(0, mavproxy_ftp.OP_ListDirectoryWithTime,
                    opcode=mavproxy_ftp.OP_Nack,
                    payload=bytes([mavproxy_ftp.ERR_EndOfFile]))
        self.ftp.ftp_settings.pkt_loss_rx = 100
        self.ftp.mavlink_packet(eof)
        self.assertIn(0, self.ftp.workers)
        self.ftp.ftp_settings.pkt_loss_rx = 0
        self.ftp.mavlink_packet(eof)
        self.assertNotIn(0, self.ftp.workers)

    def test_packet_lag_is_nonblocking_and_bidirectional(self):
        self.ftp.ftp_settings.pkt_lag_tx = 100
        self.ftp.cmd_list(['/'])
        self.assertEqual(self.mpstate._master.mav.sent, [])
        self.assertEqual(len(self.ftp.tx_delay_queue), 1)

        deadline, sequence, worker, args = self.ftp.tx_delay_queue[0]
        self.ftp.tx_delay_queue[0] = (time.monotonic() - 1,
                                      sequence, worker, args)
        self.ftp.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), 1)

        self.ftp.ftp_settings.pkt_lag_rx = 100
        eof = reply(0, mavproxy_ftp.OP_ListDirectoryWithTime,
                    opcode=mavproxy_ftp.OP_Nack,
                    payload=bytes([mavproxy_ftp.ERR_EndOfFile]))
        self.ftp.mavlink_packet(eof)
        self.assertIn(0, self.ftp.workers)
        self.assertEqual(len(self.ftp.rx_delay_queue), 1)

        deadline, sequence, worker, message = self.ftp.rx_delay_queue[0]
        self.ftp.rx_delay_queue[0] = (time.monotonic() - 1,
                                      sequence, worker, message)
        self.ftp.idle_task()
        self.assertNotIn(0, self.ftp.workers)

    def test_cancel_discards_delayed_writes_but_sends_termination(self):
        self.ftp.ftp_settings.pkt_lag_tx = 100
        self.ftp.cmd_put(['unused', '/remote'], fh=io.BytesIO(b'abcdefgh'))

        # Deliver CreateFile and let its ACK queue the upload writes.
        deadline, sequence, worker, args = self.ftp.tx_delay_queue[0]
        self.ftp.tx_delay_queue[0] = (
            time.monotonic() - 1, sequence, worker, args)
        self.ftp.idle_task()
        self.ftp.mavlink_packet(reply(0, mavproxy_ftp.OP_CreateFile))
        self.assertTrue(any(
            payload[3] == mavproxy_ftp.OP_WriteFile
            for _, _, _, queued_args in self.ftp.tx_delay_queue
            for payload in queued_args[-1]))

        self.ftp.cmd_cancel()
        self.assertEqual(len(self.ftp.tx_delay_queue), 1)
        deadline, sequence, worker, args = self.ftp.tx_delay_queue[0]
        self.assertTrue(all(payload[3] == mavproxy_ftp.OP_TerminateSession
                            for payload in args[-1]))
        self.ftp.tx_delay_queue[0] = (
            time.monotonic() - 1, sequence, worker, args)
        self.ftp.idle_task()

        opcodes = [request_opcode(packet)
                   for packet in self.mpstate._master.mav.sent]
        self.assertEqual(opcodes, [mavproxy_ftp.OP_CreateFile,
                                   mavproxy_ftp.OP_TerminateSession])

    def test_retired_session_ids_are_quarantined(self):
        self.ftp.retired_sessions[0] = (
            time.monotonic() + mavproxy_ftp.SESSION_REUSE_DELAY)
        self.ftp.next_session = 0
        self.assertEqual(self.ftp._allocate_session(), 1)

        self.ftp.retired_sessions[0] = time.monotonic() - 1
        self.ftp.next_session = 0
        self.assertEqual(self.ftp._allocate_session(), 0)

    def test_network_batches_stay_below_mtu(self):
        link = FakeLink(socket.SOCK_DGRAM)
        master = types.SimpleNamespace(mav=FakeBatchMAV(link))
        payloads = [bytes([i]) * 251 for i in range(16)]

        self.ftp._transmit_payloads(master, 0, 1, 1, payloads)

        expected = b''.join(b'F' + payload for payload in payloads)
        self.assertEqual(b''.join(link.writes), expected)
        self.assertGreater(len(link.writes), 1)
        self.assertTrue(all(len(write) <= mavproxy_ftp.MAX_NETWORK_BATCH
                            for write in link.writes))

    def test_serial_batch_remains_one_write(self):
        link = FakeLink()
        master = types.SimpleNamespace(mav=FakeBatchMAV(link))
        payloads = [b'x' * 251 for _ in range(16)]

        self.ftp._transmit_payloads(master, 0, 1, 1, payloads)

        self.assertEqual(len(link.writes), 1)

    def test_tcp_batches_use_complete_writes(self):
        link = FakeLink()
        link.port = FakeStreamPort()
        master = types.SimpleNamespace(mav=FakeBatchMAV(link))
        payloads = [b'x' * 251 for _ in range(16)]

        self.ftp._transmit_payloads(master, 0, 1, 1, payloads)

        self.assertEqual(link.writes, [])
        self.assertGreater(len(link.port.writes), 1)
        self.assertTrue(all(len(write) <= mavproxy_ftp.MAX_NETWORK_BATCH
                            for write in link.port.writes))

    def test_loss_seed_replays_variable_lag(self):
        other = mavproxy_ftp.FTPModule(FakeMPState())
        for ftp in (self.ftp, other):
            ftp.ftp_settings.loss_seed = 1234
            ftp.ftp_settings.pkt_lag_tx = 100
            ftp.ftp_settings.pkt_lag_jitter_tx = 500
            # packet_lost initialises the private deterministic generator even
            # when the configured loss percentage is zero.
            ftp.packet_lost('TX')

        first = [self.ftp.packet_delay('TX') for _ in range(10)]
        second = [other.packet_delay('TX') for _ in range(10)]
        self.assertEqual(first, second)
        self.assertTrue(all(0.1 <= delay <= 0.6 for delay in first))

    def test_delayed_reply_raises_retransmission_timeout(self):
        self.ftp.cmd_get(['/remote'], callback=lambda fh: None)
        worker = self.ftp.workers[0]
        request_seq = worker.last_op.seq
        worker.send_times[request_seq] -= 2.0

        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_OpenFileRO, payload=struct.pack('<I', 10)))

        self.assertTrue(worker.rtt_valid)
        self.assertGreater(worker.retry_timeout(), 5.0)
        sent = len(self.mpstate._master.mav.sent)
        worker.last_burst_read -= 1.0
        self.ftp.idle_task()
        self.assertEqual(len(self.mpstate._master.mav.sent), sent)

    def test_gap_reads_use_bounded_parallel_window(self):
        self.ftp.ftp_settings.max_backlog = 2
        self.ftp.cmd_get(['/remote'], callback=lambda fh: None)
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_OpenFileRO, payload=struct.pack('<I', 1000)))
        worker = self.ftp.workers[0]
        worker.read_gaps = [(0, 100), (100, 100), (200, 100)]
        worker.read_gap_times = {gap: 0 for gap in worker.read_gaps}

        sent = len(self.mpstate._master.mav.sent)
        worker.check_read_send()
        self.assertEqual(worker.backlog, 2)
        self.assertEqual(len(self.mpstate._master.mav.sent), sent + 2)

        for gap in worker.read_gaps[:2]:
            worker.read_gap_times[gap] -= worker.retry_timeout() + 0.01
        worker.check_read_send()
        self.assertEqual(worker.backlog, 2)
        self.assertEqual(len(self.mpstate._master.mav.sent), sent + 4)


if __name__ == '__main__':
    unittest.main()
