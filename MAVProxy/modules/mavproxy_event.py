#!/usr/bin/env python
'''Module for handling mavlink_event_t messages
Peter Barker, April 2021


This module interprets mavlink_event_t messages as they're received.
To do this it may request component information from the autopilot to
locate metadata for those events.

@FLAKE8_

'''

import time
import json

from MAVProxy.modules.lib import mp_module


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
        string = fh.read()
        try:
            self.general_metadata = json.loads(string)
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
        string = fh.read()
        try:
            self.events_metadata = json.loads(string)
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
            if metadatatype["type"] != 3:  # FIXME constant
                continue
            uri = metadatatype["uri"]
            break
        if uri is None:
            print("No events URI found")
            return

        #        {  "version": 1,  "metadataTypes": [{"type": 3, "uri": "mftp:/@SYS/events_metadata.json", "fileCrc": 133761337}   ]}  # NOQA

        print("Using URI (%s) for events" % (uri,))
        # FIXME: use something generic for getting resources from URI
        prefix = "mftp:/"
        if not uri.startswith(prefix):
            print("Not an mftp URI")
            return
        filepath = uri[len(prefix):]

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

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'EVENT':
            print("Event: %s" % str(m))
            if self.events_metadata is None:
                return
            print("Using event_metadata: %s" % str(self.events_metadata))
            component_metadata = None
            src_component = m.get_srcComponent()
            for comp in self.events_metadata["components"]:
                if comp["component_id"] == src_component:
                    component_metadata = comp
                    break
            if component_metadata is None:
                print("No component metadata for (%u)" % src_component)
                return

            ardupilot_event_enum = None
            for x in component_metadata["enums"]:
                if x["name"] == "ardupilot_event":
                    ardupilot_event_enum = x
                    break

            if ardupilot_event_enum is None:
                print("No ardupilot_event_enum")
                return

            print("ardupilot_event_enum: %s" % str(ardupilot_event_enum))

            enum_entry = None
            for entry in ardupilot_event_enum["entries"]:
                if entry["value"] == m.id:
                    enum_entry = entry
                    break

            if enum_entry is None:
                print("No enum entry for %s" % str(m.id))
                return

            print("  Name: %s" % enum_entry["name"])
            print("  Description: %s" % enum_entry["description"])


def init(mpstate):
    '''initialise module'''
    return event(mpstate)
