#!/usr/bin/env python
'''Module for handling mavlink_event_t messages
Peter Barker, April 2021


This module interprets mavlink_event_t messages as they're received.
To do this it may request component information from the autopilot to
locate metadata for those events.

AP_FLAKE8_CLEAN

'''

import json
import lzma
import re
import struct
import time

from MAVProxy.modules.lib import mp_module


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


def load_metadata(contents):
    '''decode a metadata file, which may be xz-compressed'''
    if contents.startswith(XZ_MAGIC):
        contents = lzma.decompress(contents)
    return json.loads(contents)


class event(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(event, self).__init__(mpstate, "event", "")

        self.general_metadata = None
        self.last_general_metadata_fetch_time = 0

        self.events_metadata = None
        self.last_events_metadata_fetch_time = 0

        # self.event_settings = mp_settings.MPSettings(
        #     [ ('verbose', bool, False),
        #   ])
        # self.add_command('event',
        #                  self.cmd_event,
        #                  "event module",
        #                  ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: event <status|set>"

    def cmd_event(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.event_settings.command(args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        print("Have events_metadata: %s" %
              str(self.events_metadata is not None))
        print("Have general_metadata: %s" %
              str(self.general_metadata is not None))

    def general_metadata_callback(self, fh):
        if fh is None:
            # the fetch failed
            print("failed fetch of general_metadata.json: %s")
            return
        try:
            self.general_metadata = load_metadata(fh.read())
        except Exception as e:
            print("malformed generate_metadata.json: %s" % str(e))

    def general_metadata_callback_progress(self, fh, total_size):
        print("general_metadata total_size=%u" % total_size)

    def get_general_metadata(self):
        '''returns metadata if we have it, or None'''
        if self.general_metadata is not None:
            return self.general_metadata
        if (self.last_general_metadata_fetch_time != 0 and
                time.time() - self.last_general_metadata_fetch_time < 10):
            # fetched within last 30s - don't try again yet
            return None
        self.last_general_metadata_fetch_time = time.time()
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            return False
        ftp.cmd_get(
            ["@SYS/general_metadata.json"],
            callback=self.general_metadata_callback,
            callback_progress=self.general_metadata_callback_progress
        )
        print("G")

    def events_metadata_callback(self, fh):
        if fh is None:
            # the fetch failed
            print("failed fetch of events_metadata.json: %s")
            return
        try:
            self.events_metadata = load_metadata(fh.read())
        except Exception as e:
            print("malformed events_metadata.json: %s" % str(e))

    def events_metadata_callback_progress(self, fh, total_size):
        print("events_metadata total_size=%u" % total_size)

    def get_events_metadata(self):
        if self.events_metadata is not None:
            return self.events_metadata
        general_metadata = self.get_general_metadata()
        if general_metadata is None:
            return

        if (self.last_events_metadata_fetch_time != 0 and
                time.time() - self.last_events_metadata_fetch_time < 10):
            # fetched within last 30s - don't try again yet
            return None
        self.last_events_metadata_fetch_time = time.time()

        # find events metadata URL:
        uri = None
        print("Using obj (%s)" % str(general_metadata))
        for metadatatype in general_metadata["metadataTypes"]:
            if metadatatype["type"] != 4:  # COMP_METADATA_TYPE_EVENTS
                continue
            uri = metadatatype["uri"]
            break
        if uri is None:
            print("No events URI found")
            return

        #        {  "version": 1,  "metadataTypes": [{"type": 4, "uri": "mftp://@SYS/events_metadata.json", "fileCrc": 133761337}   ]}  # NOQA

        print("Using URI (%s) for events" % (uri,))
        # FIXME: use something generic for getting resources from URI
        prefix = "mftp://"
        if not uri.startswith(prefix):
            print("Not an mftp URI")
            return
        # strip any [;comp=N] component selector
        filepath = re.sub(r'^\[;comp=\d+\]', '', uri[len(prefix):])

        ftp = self.mpstate.module('ftp')
        if ftp is None:
            return False
        ftp.cmd_get(
            [filepath],
            callback=self.events_metadata_callback,
            callback_progress=self.events_metadata_callback_progress
        )

    def idle_task(self):
        '''called rapidly by mavproxy'''
        self.get_events_metadata()

    def find_event(self, event_id):
        '''return (component metadata, event metadata) for event_id, or
        None.  The top 8 bits of the id select the component in the
        metadata, the bottom 24 bits the event within it'''
        component = self.events_metadata["components"].get(str(event_id >> 24))
        if component is None:
            return None
        sub_id = str(event_id & 0xFFFFFF)
        for group in component.get("event_groups", {}).values():
            event = group.get("events", {}).get(sub_id)
            if event is not None:
                return (component, event)
        return None

    def argument_values(self, component, event, arguments):
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
                ret.append(str(value))
                continue
            entry = enum["entries"].get(str(value))
            ret.append(entry["description"] if entry is not None else "(unknown: %s)" % value)
        return ret

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'EVENT':
            print("Event: %s" % str(m))
            if self.events_metadata is None:
                return
            found = self.find_event(m.id)
            if found is None:
                print("No metadata for event 0x%08x" % m.id)
                return
            (component, event) = found
            values = self.argument_values(component, event, m.arguments)
            # "{N}" in the message is replaced with argument N (1-based)
            message = re.sub(r'{(\d+)[^}]*}',
                             lambda x: values[int(x.group(1))-1],
                             event["message"])
            print("  Name: %s" % event["name"])
            print("  Message: %s" % message)


def init(mpstate):
    '''initialise module'''
    return event(mpstate)
