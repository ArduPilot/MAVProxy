#!/usr/bin/env python
'''Module for handling MAVLink events
Peter Barker, April 2021

An EVENT message carries only an id and binary arguments.  The text
describing each event is in metadata which the sending component
publishes through the MAVLink component metadata interface.  This
module fetches that metadata for each component which sends events,
displays its events on the console at their log level, and requests
events it missed using the sequence numbers in EVENT and
CURRENT_EVENT_SEQUENCE.

AP_FLAKE8_CLEAN

'''

import json
import lzma
import os
import re
import struct
import time
import zlib

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util


# values from the MAVLink common dialect.  MAVProxy may have loaded a
# MAVLink 1 dialect, which lacks them
MAV_CMD_REQUEST_MESSAGE = 512
MAVLINK_MSG_ID_COMPONENT_INFORMATION = 395
MAVLINK_MSG_ID_COMPONENT_METADATA = 397
COMP_METADATA_TYPE_EVENTS = 4
MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET = 1
MAV_SEVERITY_WARNING = 4

# xz is the only compression the MAVLink component metadata interface allows
XZ_MAGIC = b'\xfd7zXZ\x00'

# struct formats for the basic event argument types
ARGUMENT_FORMATS = {
    "uint8_t": "<B",
    "int8_t": "<b",
    "uint16_t": "<H",
    "int16_t": "<h",
    "uint32_t": "<I",
    "int32_t": "<i",
    "uint64_t": "<Q",
    "int64_t": "<q",
    "float": "<f",
}

# log levels above this (libevents' "Disabled") are never displayed
LOG_LEVEL_DEBUG = 7

# a request for missed events is repeated this often, this many times
REQUEST_TIMEOUT = 0.5
REQUEST_TRIES = 5

# events from a component whose sequence is unknown or whose metadata
# is still being fetched are held, up to these limits
MAX_OUT_OF_ORDER = 100
MAX_UNDECODED = 50

# event timestamps going back by more than this mean the sender rebooted
REBOOT_TIMESTAMP_MARGIN_MS = 10000

# component information is requested this often, this many times
COMPONENT_INFORMATION_TIMEOUT = 2
COMPONENT_INFORMATION_TRIES = 3

# a failed metadata download is retried this often, this many times
FETCH_RETRY_DELAY = 5
FETCH_TRIES = 3
MAX_METADATA_SIZE = 1024 * 1024


def load_metadata(contents):
    '''decode a metadata file, which may be xz-compressed'''
    if contents.startswith(XZ_MAGIC):
        contents = lzma.decompress(contents)
    return json.loads(contents)


def metadata_crc(contents):
    '''the CRC the component metadata interface uses for files, as
    crc_crc32() in ArduPilot'''
    return zlib.crc32(contents, 0xFFFFFFFF) ^ 0xFFFFFFFF


def parse_mftp_uri(uri):
    '''return (component id or None, path) for an mftp:// URI, or None
    if uri is not one.  The path may be preceded by a [;comp=N]
    selector naming the component to fetch it from'''
    prefix = "mftp://"
    if not uri.startswith(prefix):
        return None
    match = re.match(r'\[;comp=(\d+)\](.*)', uri[len(prefix):])
    if match is None:
        return (None, uri[len(prefix):])
    return (int(match.group(1)), match.group(2))


def sequence_newer(a, b):
    '''true if 16-bit sequence number b is after a, allowing for wrap'''
    return 0 < ((b - a) & 0xFFFF) < 0x8000


class ReceiveSequence(object):
    '''Puts the events from one component in sequence order, asking
    for those which are missing.

    request(first, last) is called to send a REQUEST_EVENT, lost(n)
    when n events can no longer be had.  Calls returning events return
    those now ready, in order.'''

    def __init__(self, request, lost):
        self.request = request
        self.lost = lost
        self.reset()
        self.events_received = 0
        self.events_lost = 0

    def reset(self):
        # sequence number of the next event to be returned
        self.expected = None
        self.last_time_boot_ms = None
        # the newest sequence number the sender says it has sent
        self.current = None
        # events received ahead of ones which are missing
        self.out_of_order = {}
        self.requested = None
        self.request_time = 0
        self.request_tries = 0

    def handle_event(self, m, now):
        if (self.last_time_boot_ms is not None and
                m.event_time_boot_ms + REBOOT_TIMESTAMP_MARGIN_MS < self.last_time_boot_ms):
            self.reset()
        if self.expected is None:
            self.expected = m.sequence
        if m.sequence != self.expected and not sequence_newer(self.expected, m.sequence):
            # a duplicate
            return []
        self.events_received += 1
        if m.sequence != self.expected:
            if len(self.out_of_order) < MAX_OUT_OF_ORDER:
                self.out_of_order[m.sequence] = m
            self.request_missing(now)
            return []
        ret = [m]
        self.last_time_boot_ms = m.event_time_boot_ms
        self.expected = (self.expected + 1) & 0xFFFF
        ret.extend(self.take_out_of_order())
        self.request_missing(now)
        return ret

    def handle_current_sequence(self, sequence, flags, now):
        if (flags & MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET and
                self.expected is not None):
            # the sender restarted its sequence.  Only start again if
            # it has gone back past events we already have
            last_received = (self.expected - 1) & 0xFFFF
            if sequence != last_received and not sequence_newer(last_received, sequence):
                self.reset()
        if self.expected is None:
            self.expected = (sequence + 1) & 0xFFFF
        self.current = sequence
        self.request_missing(now)

    def handle_error(self, sequence, oldest_available, now):
        '''the sender no longer has events from sequence up to oldest_available'''
        if self.expected is None or sequence != self.expected:
            # not about the event we are waiting for
            return []
        count = (oldest_available - self.expected) & 0xFFFF
        if count == 0 or count >= 0x8000:
            return []
        return self.skip_to(oldest_available, now)

    def update(self, now):
        '''repeat or abandon a request for missing events'''
        if self.requested is None or now - self.request_time < REQUEST_TIMEOUT:
            return []
        if self.request_tries < REQUEST_TRIES:
            self.send_request(now)
            return []
        # give up on the missing events, up to the next we hold
        (first, last) = self.requested
        return self.skip_to((last + 1) & 0xFFFF, now)

    def skip_to(self, sequence, now):
        count = (sequence - self.expected) & 0xFFFF
        self.events_lost += count
        self.lost(count)
        self.expected = sequence
        self.requested = None
        ret = self.take_out_of_order()
        self.request_missing(now)
        return ret

    def take_out_of_order(self):
        ret = []
        while self.expected in self.out_of_order:
            m = self.out_of_order.pop(self.expected)
            self.last_time_boot_ms = m.event_time_boot_ms
            ret.append(m)
            self.expected = (self.expected + 1) & 0xFFFF
        return ret

    def missing(self):
        '''return (first, last) of the events to request, or None'''
        if self.expected is None:
            return None
        newest = None
        if self.out_of_order:
            # up to the oldest event we hold
            newest = min(self.out_of_order, key=lambda s: (s - self.expected) & 0xFFFF)
            newest = (newest - 1) & 0xFFFF
        elif self.current is not None and sequence_newer((self.expected - 1) & 0xFFFF, self.current):
            newest = self.current
        if newest is None:
            return None
        return (self.expected, newest)

    def request_missing(self, now):
        missing = self.missing()
        if missing is None:
            self.requested = None
            return
        if self.requested is not None and missing[1] == self.requested[1]:
            # the rest of a range already asked for, which may still
            # be arriving; update() repeats the request
            self.requested = missing
            return
        self.requested = missing
        self.request_tries = 0
        self.send_request(now)

    def send_request(self, now):
        self.request(*self.requested)
        self.request_time = now
        self.request_tries += 1


class ComponentEvents(object):
    '''events and metadata for one MAVLink component'''

    def __init__(self, module, sysid, compid):
        self.module = module
        self.sysid = sysid
        self.compid = compid
        self.sequence = ReceiveSequence(self.request_events, self.events_lost)
        # events received before the metadata needed to show them
        self.undecoded = []
        self.metadata = None
        # None while still trying to get metadata, else why there is none
        self.unavailable = None
        self.state = 'request-component-information'
        self.tries = 0
        self.next_attempt = 0
        self.general_metadata_uri = None
        self.general_metadata_crc = None
        self.events_metadata_uri = None
        self.events_metadata_crc = None

    def name(self):
        return "%u:%u" % (self.sysid, self.compid)

    def request_events(self, first, last):
        self.module.master.mav.request_event_send(self.sysid, self.compid, first, last)

    def events_lost(self, count):
        self.module.show(MAV_SEVERITY_WARNING,
                         "%u events from %s lost" % (count, self.name()))

    def set_unavailable(self, reason):
        self.unavailable = reason
        self.state = 'done'
        if self.module.event_settings.verbose:
            print("event: no metadata for %s: %s" % (self.name(), reason))
        self.flush_undecoded()

    def receive(self, events):
        for m in events:
            if self.metadata is None and self.unavailable is None:
                if len(self.undecoded) < MAX_UNDECODED:
                    self.undecoded.append(m)
                continue
            self.module.display(self, m)

    def flush_undecoded(self):
        undecoded = self.undecoded
        self.undecoded = []
        self.receive(undecoded)

    def handle_component_metadata(self, uri, crc):
        if self.state != 'wait-component-information':
            return
        if not uri:
            self.set_unavailable("component publishes no metadata")
            return
        self.general_metadata_uri = uri
        self.general_metadata_crc = crc
        self.state = 'fetch-general'
        self.tries = 0
        self.next_attempt = 0

    def update(self, now):
        if now < self.next_attempt:
            return
        if self.state == 'request-component-information':
            if self.tries >= COMPONENT_INFORMATION_TRIES:
                self.set_unavailable("no COMPONENT_INFORMATION")
                return
            self.tries += 1
            self.module.request_component_information(self.sysid, self.compid)
            self.state = 'wait-component-information'
            self.next_attempt = now + COMPONENT_INFORMATION_TIMEOUT
        elif self.state == 'wait-component-information':
            self.state = 'request-component-information'
        elif self.state == 'fetch-general':
            self.fetch(self.general_metadata_uri, self.general_metadata_crc,
                       self.general_metadata_fetched, now)
        elif self.state == 'fetch-events':
            self.fetch(self.events_metadata_uri, self.events_metadata_crc,
                       self.events_metadata_fetched, now)

    def fetch(self, uri, crc, callback, now):
        '''start fetching uri, calling callback with its contents.
        Failures leave the state as it is, to be tried again'''
        if self.tries >= FETCH_TRIES:
            self.set_unavailable("could not fetch %s" % uri)
            return
        contents = self.module.cache_get(crc)
        if contents is not None:
            callback(contents)
            return
        parsed = parse_mftp_uri(uri)
        if parsed is None:
            self.set_unavailable("unsupported URI %s" % uri)
            return
        ftp = self.module.module('ftp')
        if ftp is None:
            # try again in case it is loaded later
            self.module.warn_once("event: ftp module needed to fetch event metadata")
            self.next_attempt = now + FETCH_RETRY_DELAY
            return
        (compid, path) = parsed
        self.tries += 1
        retry_state = self.state
        self.state = 'fetching'

        def fetched(fh):
            if fh is None:
                self.state = retry_state
                self.next_attempt = time.time() + FETCH_RETRY_DELAY
                return
            contents = fh.read()
            if crc and metadata_crc(contents) != crc:
                # use it, but do not cache it under the wrong CRC
                self.module.warn_once("event: %s from %s has CRC 0x%08x, expected 0x%08x" %
                                      (uri, self.name(), metadata_crc(contents), crc))
            elif crc:
                self.module.cache_put(crc, contents)
            callback(contents)

        ftp.cmd_get([path], callback=fetched,
                    target_system=self.sysid,
                    target_component=self.compid if compid is None else compid,
                    max_size=MAX_METADATA_SIZE)

    def general_metadata_fetched(self, contents):
        try:
            general = load_metadata(contents)
            entries = [x for x in general["metadataTypes"]
                       if x.get("type") == COMP_METADATA_TYPE_EVENTS]
        except Exception as e:
            self.set_unavailable("bad general metadata: %s" % e)
            return
        if len(entries) == 0:
            self.set_unavailable("no events metadata")
            return
        self.events_metadata_uri = entries[0].get("uri")
        self.events_metadata_crc = entries[0].get("fileCrc")
        self.state = 'fetch-events'
        self.tries = 0
        self.next_attempt = 0

    def events_metadata_fetched(self, contents):
        try:
            self.metadata = load_metadata(contents)
        except Exception as e:
            self.set_unavailable("bad events metadata: %s" % e)
            return
        self.state = 'done'
        self.flush_undecoded()

    def find_event(self, event_id):
        '''return (component metadata, event metadata) for event_id, or
        None.  The top 8 bits of the id select the component in the
        metadata, the bottom 24 bits the event within it'''
        if self.metadata is None:
            return None
        component = self.metadata.get("components", {}).get(str(event_id >> 24))
        if component is None:
            return None
        sub_id = str(event_id & 0xFFFFFF)
        for group in component.get("event_groups", {}).values():
            event = group.get("events", {}).get(sub_id)
            if event is not None:
                return (component, event)
        return None

    def status(self):
        if self.metadata is not None:
            metadata = "loaded"
        elif self.unavailable is not None:
            metadata = "unavailable (%s)" % self.unavailable
        else:
            metadata = "fetching"
        seq = self.sequence
        return ("%s: metadata %s, %u events received, %u lost, next sequence %s" %
                (self.name(), metadata, seq.events_received, seq.events_lost, seq.expected))


def argument_values(component, event, arguments):
    '''return display strings for the arguments of event'''
    ret = []
    offset = 0
    enums = component.get("enums", {})
    for arg in event.get("arguments", []):
        enum = enums.get(arg["type"])
        fmt = ARGUMENT_FORMATS[enum["type"] if enum is not None else arg["type"]]
        (value,) = struct.unpack_from(fmt, bytes(arguments), offset)
        offset += struct.calcsize(fmt)
        if enum is None:
            ret.append(value)
            continue
        entries = enum.get("entries", {})
        if enum.get("is_bitfield", False):
            names = [entries[str(1 << i)]["description"] if str(1 << i) in entries else "0x%x" % (1 << i)
                     for i in range(64) if value & (1 << i)]
            ret.append(" | ".join(names) if names else "none")
            continue
        entry = entries.get(str(value))
        ret.append(entry["description"] if entry is not None else "(unknown: %s)" % value)
    return ret


def format_message(message, values):
    '''substitute argument values into an event message: "{N}" or
    "{N:fmt}" is argument N (1-based), and libevents markup is removed'''
    def substitute(match):
        index = int(match.group(1)) - 1
        if index >= len(values):
            return match.group(0)
        value = values[index]
        precision = re.match(r':\.(\d+)', match.group(2) or '')
        if precision is not None and isinstance(value, float):
            return "%.*f" % (int(precision.group(1)), value)
        return str(value)
    message = re.sub(r'{(\d+)([^}]*)}', substitute, message)
    return re.sub(r'</?(?:a|param|profile)\b[^>]*>', '', message)


class event(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(event, self).__init__(mpstate, "event", "MAVLink events")

        # ComponentEvents keyed by (sysid, compid)
        self.components = {}
        self.warned = set()
        self.cache_dir = mp_util.dot_mavproxy("component_metadata")

        self.event_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ])
        self.add_command('event',
                         self.cmd_event,
                         "MAVLink events",
                         ['status', 'set (EVENTSETTING)'])
        self.add_completion_function('(EVENTSETTING)',
                                     self.event_settings.completion)

    def usage(self):
        '''show help on command line options'''
        return "Usage: event <status|set>"

    def cmd_event(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status_text())
        elif args[0] == "set":
            self.event_settings.command(args[1:])
        else:
            print(self.usage())

    def status_text(self):
        '''returns information about module'''
        if len(self.components) == 0:
            return "No events received"
        return "\n".join([c.status() for c in self.components.values()])

    def warn_once(self, text):
        if text not in self.warned:
            self.warned.add(text)
            print(text)

    def cache_path(self, crc):
        return os.path.join(self.cache_dir, "%08x" % crc)

    def cache_get(self, crc):
        '''return a metadata file we have already fetched, or None'''
        if not crc:
            return None
        try:
            with open(self.cache_path(crc), 'rb') as f:
                contents = f.read()
        except OSError:
            return None
        if metadata_crc(contents) != crc:
            return None
        return contents

    def cache_put(self, crc, contents):
        try:
            mp_util.mkdir_p(self.cache_dir)
            with open(self.cache_path(crc), 'wb') as f:
                f.write(contents)
        except OSError as e:
            self.warn_once("event: unable to cache metadata: %s" % e)

    def request_component_information(self, sysid, compid):
        # COMPONENT_METADATA replaces COMPONENT_INFORMATION; a component
        # may support either
        for msgid in (MAVLINK_MSG_ID_COMPONENT_METADATA, MAVLINK_MSG_ID_COMPONENT_INFORMATION):
            self.master.mav.command_long_send(
                sysid, compid,
                MAV_CMD_REQUEST_MESSAGE, 0,
                msgid, 0, 0, 0, 0, 0, 0)

    def component(self, m):
        key = (m.get_srcSystem(), m.get_srcComponent())
        if key not in self.components:
            self.components[key] = ComponentEvents(self, key[0], key[1])
        return self.components[key]

    def show(self, severity, text):
        '''display text at a MAV_SEVERITY, as STATUSTEXT is'''
        link = self.module('link')
        if link is not None:
            (fg, bg) = link.colors_for_severity(severity)
        else:
            (fg, bg) = ('black', 'white')
        self.console.writeln(text, fg=fg, bg=bg)

    def display(self, component, m):
        '''display an event, decoded if we have its metadata'''
        level = m.log_levels & 0xF
        if level > LOG_LEVEL_DEBUG:
            return
        if (m.destination_system not in (0, self.settings.source_system) or
                m.destination_component not in (0, self.settings.source_component)):
            return
        prefix = "Event: "
        if (component.sysid, component.compid) != (self.target_system, self.target_component):
            prefix = "Event (%s): " % component.name()
        found = component.find_event(m.id)
        if found is None:
            self.show(level, "%sunknown event 0x%08x" % (prefix, m.id))
            return
        (metadata, event) = found
        try:
            values = argument_values(metadata, event, m.arguments)
            message = format_message(event.get("message", event.get("name", "")), values)
        except Exception as e:
            self.show(level, "%sevent 0x%08x: bad metadata (%s)" % (prefix, m.id, e))
            return
        self.show(level, prefix + message)
        if self.event_settings.verbose:
            print("  name: %s" % event.get("name"))
            if event.get("description"):
                print("  description: %s" % event["description"])

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        for component in self.components.values():
            component.update(now)
            component.receive(component.sequence.update(now))

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        now = time.time()
        if mtype == 'EVENT':
            component = self.component(m)
            component.receive(component.sequence.handle_event(m, now))
        elif mtype == 'CURRENT_EVENT_SEQUENCE':
            self.component(m).sequence.handle_current_sequence(m.sequence, m.flags, now)
        elif mtype == 'RESPONSE_EVENT_ERROR':
            if (m.target_system, m.target_component) != (self.settings.source_system, self.settings.source_component):
                return
            component = self.component(m)
            component.receive(component.sequence.handle_error(m.sequence, m.sequence_oldest_available, now))
        elif mtype == 'COMPONENT_INFORMATION':
            key = (m.get_srcSystem(), m.get_srcComponent())
            if key in self.components:
                self.components[key].handle_component_metadata(m.general_metadata_uri, m.general_metadata_file_crc)
        elif mtype == 'COMPONENT_METADATA':
            key = (m.get_srcSystem(), m.get_srcComponent())
            if key in self.components:
                self.components[key].handle_component_metadata(m.uri, m.file_crc)


def init(mpstate):
    '''initialise module'''
    return event(mpstate)
