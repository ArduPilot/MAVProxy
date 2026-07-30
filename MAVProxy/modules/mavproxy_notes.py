#!/usr/bin/env python3
'''
Flight notes module

Lets an operator flying with --aircraft keep notes about a flight, stored
alongside the logs in AIRCRAFTNAME/logs/DATE/notes.txt. There is one notes
file per aircraft per day, so a day with several flights, or with --mission
set, shares the one file.

Adds a Notes item to the console's MAVProxy menu, which opens a window with
a section per date, latest first. The current date is editable and saves as
you type; previous dates that have notes can be expanded and read.
'''

# AP_FLAKE8_CLEAN

import datetime
import os
import re
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

NOTES_FILE = 'notes.txt'

# log directories are named by date, see log_paths() in mavproxy.py
DATE_RE = re.compile(r'^\d{4}-\d{2}-\d{2}$')


def today_string():
    '''date string naming today's log directory'''
    return time.strftime("%Y-%m-%d")


def notes_logs_dir(aircraft_dir):
    '''directory holding the per-date log directories'''
    return os.path.join(aircraft_dir, 'logs')


def notes_path(logs_dir, datestr):
    '''path of the notes file for one date'''
    return os.path.join(logs_dir, datestr, NOTES_FILE)


def valid_date(datestr):
    '''True if this names a real date'''
    if not DATE_RE.match(datestr):
        return False
    try:
        datetime.date.fromisoformat(datestr)
    except ValueError:
        return False
    return True


def find_note_dates(logs_dir, today):
    '''dates to show, today first then any earlier dates that have notes,
       most recent first'''
    dates = []
    try:
        entries = os.listdir(logs_dir)
    except OSError:
        entries = []
    for e in entries:
        # ISO dates compare as dates, so this also drops anything dated
        # later than today
        if e >= today or not valid_date(e):
            continue
        if not os.path.isdir(os.path.join(logs_dir, e)):
            continue
        if not os.path.isfile(notes_path(logs_dir, e)):
            continue
        dates.append(e)
    # ISO dates sort chronologically
    dates.sort(reverse=True)
    return [today] + dates


class NotesModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NotesModule, self).__init__(mpstate, "notes", "flight notes")
        self.ui = None
        self.menu_added_console = False
        self.add_command('notes', self.cmd_notes, "flight notes",
                         ["<show|hide|status>"])

    def usage(self):
        return "Usage: notes <show|hide|status>"

    def aircraft_dir(self):
        '''aircraft directory, or None if --aircraft was not used'''
        return self.mpstate.aircraft_dir

    def sections(self):
        '''(date, path, editable) for each date to show'''
        logs_dir = notes_logs_dir(self.aircraft_dir())
        today = today_string()
        return [(d, notes_path(logs_dir, d), d == today)
                for d in find_note_dates(logs_dir, today)]

    def cmd_notes(self, args):
        '''control the notes window'''
        if len(args) == 0:
            print(self.usage())
            return
        if args[0] == "show":
            self.cmd_show()
        elif args[0] == "hide":
            self.close_ui()
        elif args[0] == "status":
            print(self.status_text())
        else:
            print(self.usage())

    def status_text(self):
        if self.aircraft_dir() is None:
            return "notes: no --aircraft given, nowhere to store notes"
        path = notes_path(notes_logs_dir(self.aircraft_dir()), today_string())
        return "notes: %s (window %s)" % (
            path, "open" if self.ui is not None else "closed")

    def cmd_show(self):
        '''open the notes window, or raise it if it is already open'''
        if self.aircraft_dir() is None:
            print("notes: needs the --aircraft option to know where to "
                  "store notes")
            return
        if not mp_util.has_wxpython:
            print("notes: wxpython is not available")
            return
        if self.ui is not None:
            self.ui.raise_window()
            return
        from MAVProxy.modules.lib import wxnotes
        self.ui = wxnotes.NotesUI(self.sections())

    def close_ui(self):
        if self.ui is None:
            return
        self.ui.close()
        self.ui = None

    def idle_task(self):
        '''add our console menu item, and notice the window going away'''
        console = self.module('console')
        if console is None:
            self.menu_added_console = False
        elif (not self.menu_added_console and mp_util.has_wxpython and
              self.aircraft_dir() is not None):
            self.menu_added_console = True
            # add_to_submenu rather than add_menu, as adding a submenu of the
            # same name would replace the existing MAVProxy menu items
            console.cmd_menu_add(['MAVProxy:Notes', 'notes', 'show'])
        if self.ui is not None and not self.ui.is_alive():
            self.ui = None

    def unload(self):
        '''unload module'''
        self.remove_command('notes')
        self.close_ui()
        console = self.module('console')
        if console is not None and self.menu_added_console:
            console.cmd_menu_remove(['MAVProxy:Notes'])
        self.menu_added_console = False
        super(NotesModule, self).unload()


def init(mpstate):
    '''initialise module'''
    return NotesModule(mpstate)
