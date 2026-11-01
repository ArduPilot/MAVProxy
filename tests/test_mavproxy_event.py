'''tests for the MAVLink events module'''

import importlib.util
import io
import json
import lzma
from pathlib import Path
import tempfile
import types
import unittest


# load the worktree file explicitly, as test_mavproxy_ftp.py does
EVENT_MODULE_PATH = (Path(__file__).resolve().parents[1] /
                     'MAVProxy/modules/mavproxy_event.py')
EVENT_SPEC = importlib.util.spec_from_file_location(
    'mavproxy_event_under_test', EVENT_MODULE_PATH)
mavproxy_event = importlib.util.module_from_spec(EVENT_SPEC)
EVENT_SPEC.loader.exec_module(mavproxy_event)

RESET = mavproxy_event.MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET


def event_msg(sequence, event_id=0x01000001, time_boot_ms=1000, log_levels=0x55,
              arguments=None, sysid=1, compid=1, destination_system=0, destination_component=0):
    arguments = list(arguments or [])
    return types.SimpleNamespace(
        sequence=sequence, id=event_id, event_time_boot_ms=time_boot_ms,
        log_levels=log_levels, arguments=arguments + [0] * (40 - len(arguments)),
        destination_system=destination_system, destination_component=destination_component,
        get_type=lambda: 'EVENT',
        get_srcSystem=lambda: sysid, get_srcComponent=lambda: compid)


class Recorder:
    def __init__(self):
        self.requests = []
        self.lost = []

    def sequence(self):
        return mavproxy_event.ReceiveSequence(
            lambda first, last: self.requests.append((first, last)),
            self.lost.append)


def seqs(events):
    return [m.sequence for m in events]


class ReceiveSequenceTest(unittest.TestCase):
    def setUp(self):
        self.rec = Recorder()
        self.seq = self.rec.sequence()

    def test_in_order(self):
        for s in range(5, 9):
            self.assertEqual(seqs(self.seq.handle_event(event_msg(s), 0)), [s])
        self.assertEqual(self.rec.requests, [])

    def test_duplicate_dropped(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_event(event_msg(6), 0)
        self.assertEqual(self.seq.handle_event(event_msg(5), 0), [])
        self.assertEqual(self.seq.handle_event(event_msg(6), 0), [])

    def test_gap_requested_and_filled_in_order(self):
        self.seq.handle_event(event_msg(5), 0)
        self.assertEqual(self.seq.handle_event(event_msg(9), 0), [])
        self.assertEqual(self.rec.requests, [(6, 8)])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(6), 0)), [6])
        # 7 and 8 are still on their way; not asked for again yet
        self.assertEqual(self.rec.requests, [(6, 8)])
        self.assertEqual(self.seq.update(mavproxy_event.REQUEST_TIMEOUT), [])
        self.assertEqual(self.rec.requests, [(6, 8), (7, 8)])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(7), 0)), [7])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(8), 0)), [8, 9])
        self.assertEqual(self.seq.requested, None)

    def test_wraparound(self):
        self.seq.handle_event(event_msg(65534), 0)
        self.assertEqual(seqs(self.seq.handle_event(event_msg(65535), 0)), [65535])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(0), 0)), [0])
        self.assertEqual(self.seq.handle_event(event_msg(2), 0), [])
        self.assertEqual(self.rec.requests, [(1, 1)])

    def test_unavailable_events_skipped(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_event(event_msg(10), 0)
        self.assertEqual(self.rec.requests, [(6, 9)])
        # the sender only has events from 8
        self.assertEqual(self.seq.handle_error(6, 8, 0), [])
        self.assertEqual(self.rec.lost, [2])
        self.assertEqual(self.rec.requests, [(6, 9), (8, 9)])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(8), 0)), [8])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(9), 0)), [9, 10])

    def test_error_for_other_sequence_ignored(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_event(event_msg(10), 0)
        self.assertEqual(self.seq.handle_error(3, 8, 0), [])
        self.assertEqual(self.rec.lost, [])

    def test_request_repeated_then_abandoned(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_event(event_msg(8), 0)
        now = 0
        for i in range(mavproxy_event.REQUEST_TRIES - 1):
            now += mavproxy_event.REQUEST_TIMEOUT
            self.assertEqual(self.seq.update(now), [])
        self.assertEqual(self.rec.requests, [(6, 7)] * mavproxy_event.REQUEST_TRIES)
        now += mavproxy_event.REQUEST_TIMEOUT
        self.assertEqual(seqs(self.seq.update(now)), [8])
        self.assertEqual(self.rec.lost, [2])
        self.assertEqual(self.seq.update(now + 10), [])

    def test_current_sequence_shows_missed_latest_event(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_current_sequence(5, 0, 0)
        self.assertEqual(self.rec.requests, [])
        self.seq.handle_current_sequence(7, 0, 0)
        self.assertEqual(self.rec.requests, [(6, 7)])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(6), 0)), [6])
        self.assertEqual(seqs(self.seq.handle_event(event_msg(7), 0)), [7])
        self.assertEqual(self.seq.requested, None)

    def test_current_sequence_first(self):
        self.seq.handle_current_sequence(41, 0, 0)
        self.assertEqual(seqs(self.seq.handle_event(event_msg(42), 0)), [42])
        self.assertEqual(self.rec.requests, [])

    def test_reset_flag(self):
        self.seq.handle_event(event_msg(40), 0)
        # the sender rebooted and has sent no events yet
        self.seq.handle_current_sequence(65535, RESET, 0)
        self.assertEqual(seqs(self.seq.handle_event(event_msg(0, time_boot_ms=500), 0)), [0])

    def test_reset_flag_when_in_step_changes_nothing(self):
        self.seq.handle_event(event_msg(5), 0)
        self.seq.handle_event(event_msg(7), 0)
        self.seq.handle_current_sequence(7, RESET, 0)
        self.assertEqual(seqs(self.seq.handle_event(event_msg(6), 0)), [6, 7])

    def test_reboot_detected_from_timestamps(self):
        self.seq.handle_event(event_msg(40, time_boot_ms=60000), 0)
        self.assertEqual(seqs(self.seq.handle_event(event_msg(0, time_boot_ms=2000), 0)), [0])


class FormatTest(unittest.TestCase):
    COMPONENT = {
        "enums": {
            "mode_t": {"type": "uint8_t", "entries": {
                "1": {"name": "one", "description": "One"},
            }},
            "bits_t": {"type": "uint8_t", "is_bitfield": True, "entries": {
                "1": {"name": "a", "description": "A"},
                "4": {"name": "c", "description": "C"},
            }},
        },
    }

    def values(self, types, arguments):
        event = {"arguments": [{"type": t, "name": "x"} for t in types]}
        return mavproxy_event.argument_values(self.COMPONENT, event, arguments)

    def test_basic_and_enum_arguments(self):
        self.assertEqual(self.values(["uint8_t", "mode_t", "mode_t", "int16_t"], [7, 1, 3, 0xfe, 0xff]),
                         [7, "One", "(unknown: 3)", -2])

    def test_bitfield_argument(self):
        self.assertEqual(self.values(["bits_t"], [5]), ["A | C"])
        self.assertEqual(self.values(["bits_t"], [0]), ["none"])

    def test_format_message(self):
        fmt = mavproxy_event.format_message
        self.assertEqual(fmt("mode {1} failed", ["Loiter"]), "mode Loiter failed")
        self.assertEqual(fmt("alt {1:.1m}", [1.25]), "alt 1.2")
        self.assertEqual(fmt("check <param>FENCE_ENABLE</param>", []), "check FENCE_ENABLE")
        self.assertEqual(fmt("x {2} < 5", [1]), "x {2} < 5")


EVENTS_METADATA = {
    "version": 2,
    "components": {
        "1": {
            "namespace": "ardupilot",
            "enums": {},
            "event_groups": {"default": {"events": {
                "10": {"name": "fence_enable", "message": "Fence enabled"},
                "65546": {"name": "error_flight_mode", "message": "Failed to change to mode {1}",
                          "description": "The argument is the mode number",
                          "arguments": [{"type": "uint8_t", "name": "mode"}]},
            }}},
        },
    },
}
OTHER_METADATA = {
    "version": 2,
    "components": {"1": {"event_groups": {"default": {"events": {
        "10": {"name": "other", "message": "Other component event"},
    }}}}},
}


def files_for(events_metadata, events_path="@ROMFS/events.json.xz"):
    events = lzma.compress(json.dumps(events_metadata).encode(), format=lzma.FORMAT_XZ)
    general = json.dumps({"version": 1, "metadataTypes": [
        {"type": 4, "uri": "mftp://" + events_path, "fileCrc": mavproxy_event.metadata_crc(events)}]}).encode()
    return {"@SYS/general_metadata.json": general, events_path: events}


class FakeFTP:
    def __init__(self):
        # files keyed by (sysid, compid) and path
        self.files = {}
        self.gets = []
        self.pending = []

    def cmd_get(self, args, callback=None, callback_progress=None,
                target_system=None, target_component=None, max_size=None):
        self.gets.append((target_system, target_component, args[0]))
        contents = self.files.get((target_system, target_component), {}).get(args[0])
        self.pending.append((callback, None if contents is None else io.BytesIO(contents)))

    def complete(self):
        pending = self.pending
        self.pending = []
        for (callback, fh) in pending:
            callback(fh)


class FakeMAV:
    def __init__(self):
        self.sent = []

    def request_event_send(self, *args):
        self.sent.append(('REQUEST_EVENT',) + args)

    def command_long_send(self, target_system, target_component, command, confirmation, *params):
        self.sent.append(('COMMAND_LONG', target_system, target_component, command, params[0]))


class FakeConsole:
    def __init__(self):
        self.lines = []

    def writeln(self, text, fg='black', bg='white'):
        self.lines.append((text, fg, bg))


class FakeLink:
    def colors_for_severity(self, severity):
        return ('fg%u' % severity, 'bg%u' % severity)


class FakeMPState:
    def __init__(self):
        self.public_modules = {}
        self.command_map = {}
        self.completions = {}
        self.completion_functions = {}
        self.multi_instance = {}
        self.instance_count = {}
        self.console = FakeConsole()
        self.settings = types.SimpleNamespace(source_system=255, source_component=190,
                                              target_system=1, target_component=1)
        self.mav = FakeMAV()
        self.modules = {'link': FakeLink()}

    def module(self, name):
        return self.modules.get(name)

    def master(self):
        return types.SimpleNamespace(mav=self.mav)


def component_information(uri, crc, sysid=1, compid=1):
    return types.SimpleNamespace(
        general_metadata_uri=uri, general_metadata_file_crc=crc,
        get_type=lambda: 'COMPONENT_INFORMATION',
        get_srcSystem=lambda: sysid, get_srcComponent=lambda: compid)


class EventModuleTest(unittest.TestCase):
    def setUp(self):
        self.mpstate = FakeMPState()
        self.ftp = FakeFTP()
        self.mpstate.modules['ftp'] = self.ftp
        self.module = mavproxy_event.event(self.mpstate)
        cache = tempfile.TemporaryDirectory()
        self.addCleanup(cache.cleanup)
        self.module.cache_dir = cache.name

    def lines(self):
        return [text for (text, fg, bg) in self.mpstate.console.lines]

    def connect(self, sysid=1, compid=1, metadata=EVENTS_METADATA):
        '''take a component through fetching its metadata'''
        files = files_for(metadata)
        self.ftp.files[(sysid, compid)] = files
        self.module.idle_task()
        general = files["@SYS/general_metadata.json"]
        self.module.mavlink_packet(component_information(
            "mftp://@SYS/general_metadata.json", mavproxy_event.metadata_crc(general), sysid, compid))
        self.module.idle_task()
        self.ftp.complete()
        self.module.idle_task()
        self.ftp.complete()

    def test_metadata_fetched_and_events_decoded_at_level(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a, log_levels=0x55))
        self.module.idle_task()
        self.assertIn(('COMMAND_LONG', 1, 1, mavproxy_event.MAV_CMD_REQUEST_MESSAGE,
                       mavproxy_event.MAVLINK_MSG_ID_COMPONENT_INFORMATION), self.mpstate.mav.sent)
        # held until the metadata arrives
        self.assertEqual(self.lines(), [])
        self.connect()
        self.assertEqual(self.ftp.gets, [(1, 1, "@SYS/general_metadata.json"), (1, 1, "@ROMFS/events.json.xz")])
        self.module.mavlink_packet(event_msg(1, 0x0101000a, log_levels=0x44, arguments=[14]))
        self.assertEqual(self.mpstate.console.lines, [
            ("Event: Fence enabled", 'fg5', 'bg5'),
            ("Event: Failed to change to mode 14", 'fg4', 'bg4'),
        ])

    def test_metadata_cached_by_crc(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a))
        self.connect()
        self.assertEqual(len(self.ftp.gets), 2)
        # a new connection to a component with the same files fetches nothing
        self.module.components.clear()
        self.module.mavlink_packet(event_msg(5, 0x0100000a))
        self.connect()
        self.assertEqual(len(self.ftp.gets), 2)
        self.assertEqual(self.lines(), ["Event: Fence enabled"] * 2)

    def test_metadata_per_component(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a, sysid=1, compid=1))
        self.module.mavlink_packet(event_msg(0, 0x0100000a, sysid=2, compid=1))
        self.connect(sysid=1)
        self.connect(sysid=2, metadata=OTHER_METADATA)
        self.assertEqual(sorted(self.lines()), ["Event (2:1): Other component event", "Event: Fence enabled"])

    def test_no_ftp_module(self):
        del self.mpstate.modules['ftp']
        self.module.mavlink_packet(event_msg(0, 0x0100000a))
        self.module.idle_task()
        self.module.mavlink_packet(component_information("mftp://@SYS/general_metadata.json", 1))
        for i in range(3):
            self.module.idle_task()
        self.assertEqual(self.lines(), [])
        self.assertIn("fetching", self.module.status_text())

    def test_no_component_information(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a, log_levels=0x33))
        now = 0
        for i in range(mavproxy_event.COMPONENT_INFORMATION_TRIES * 2 + 1):
            self.module.components[(1, 1)].update(now)
            now += mavproxy_event.COMPONENT_INFORMATION_TIMEOUT
        self.assertEqual(self.mpstate.console.lines, [("Event: unknown event 0x0100000a", 'fg3', 'bg3')])
        self.assertIn("no COMPONENT_INFORMATION", self.module.status_text())

    def test_events_for_others_and_disabled_levels_not_shown(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a))
        self.connect()
        self.module.mavlink_packet(event_msg(1, 0x0100000a, destination_system=7))
        self.module.mavlink_packet(event_msg(2, 0x0100000a, log_levels=0x88))
        self.module.mavlink_packet(event_msg(3, 0x0100000a))
        self.assertEqual(self.lines(), ["Event: Fence enabled", "Event: Fence enabled"])

    def test_missed_event_requested(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a))
        self.connect()
        self.module.mavlink_packet(event_msg(2, 0x0100000a))
        self.assertIn(('REQUEST_EVENT', 1, 1, 1, 1), self.mpstate.mav.sent)

    def test_bad_crc_used_but_not_cached(self):
        self.module.mavlink_packet(event_msg(0, 0x0100000a))
        files = files_for(EVENTS_METADATA)
        self.ftp.files[(1, 1)] = files
        self.module.idle_task()
        self.module.mavlink_packet(component_information("mftp://@SYS/general_metadata.json", 1234))
        self.module.idle_task()
        self.ftp.complete()
        self.module.idle_task()
        self.ftp.complete()
        self.assertEqual(self.lines(), ["Event: Fence enabled"])
        self.assertEqual(self.module.cache_get(1234), None)


class URITest(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(mavproxy_event.parse_mftp_uri("mftp://@ROMFS/events.json.xz"),
                         (None, "@ROMFS/events.json.xz"))
        self.assertEqual(mavproxy_event.parse_mftp_uri("mftp://[;comp=100]metadata.json"),
                         (100, "metadata.json"))
        self.assertEqual(mavproxy_event.parse_mftp_uri("https://example.com/x.json"), None)

    def test_crc_matches_ardupilot(self):
        # general metadata from ArduPilot SITL, and the CRC it sent in
        # COMPONENT_INFORMATION
        general = (b'{"version":1,"metadataTypes":[{"type":4,'
                   b'"uri":"mftp://@ROMFS/events.json.xz","fileCrc":4151502586}]}')
        self.assertEqual(mavproxy_event.metadata_crc(general), 0x46e62534)


if __name__ == '__main__':
    unittest.main()
