import importlib.util
import io
from pathlib import Path
import struct
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


class TestConcurrentFTP(unittest.TestCase):
    def setUp(self):
        self.mpstate = FakeMPState()
        self.ftp = mavproxy_ftp.FTPModule(self.mpstate)

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
        self.ftp.mavlink_packet(reply(1, mavproxy_ftp.OP_ListDirectory,
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
            0, mavproxy_ftp.OP_ListDirectory,
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
        worker.last_op_time -= 2
        self.ftp.idle_task()

        retried_packet = self.mpstate._master.mav.sent[-1]
        retried_seq = struct.unpack('<H', retried_packet[:2])[0]
        self.assertEqual(retried_seq, first_seq)
        self.assertEqual(request_session(retried_packet), 0)
        self.assertEqual(worker.request_retries, 1)
        self.assertEqual(len(self.ftp.workers), 1)

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

    def test_write_window_uses_server_capabilities_in_auto_mode(self):
        self.ftp.cmd_put(['unused', '/remote'], fh=io.BytesIO(b'abc'))
        worker = self.ftp.workers[0]
        self.assertEqual(worker.write_qsize, 5)
        self.assertEqual(worker.write_batch_size, 1)

        capabilities = bytes([
            mavproxy_ftp.WRITE_CAPABILITY_MAGIC, 64, 8,
        ])
        self.ftp.mavlink_packet(reply(
            0, mavproxy_ftp.OP_CreateFile, payload=capabilities))

        self.assertEqual(worker.write_qsize, 64)
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


if __name__ == '__main__':
    unittest.main()
