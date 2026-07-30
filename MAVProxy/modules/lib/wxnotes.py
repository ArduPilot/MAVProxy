#!/usr/bin/env python3

"""
  MAVProxy flight notes window

  This holds the parent side only, so that it can be imported without
  wxPython. The GUI itself is in wxnotes_ui.py, imported in the child
  process.
"""

# AP_FLAKE8_CLEAN

import os

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc

# seconds to wait for the window to save and exit when closing it
CLOSE_TIMEOUT = 2

# how much longer to wait when it is still going, rather than killing it
# while it may still be writing the notes out
CLOSE_TIMEOUT_SLOW = 20


def read_notes_checked(path):
    '''read a notes file, returning (text, lossy). lossy is set if what we
       read would not reproduce the bytes on disk, either because the file is
       not valid UTF-8 or because it could not be read at all, so that the
       caller knows to preserve the original before writing over it'''
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except FileNotFoundError:
        return ("", False)
    except (IOError, OSError):
        # something is there but we cannot read it. Don't mistake that for
        # an empty file
        return ("", True)
    try:
        return (data.decode('utf-8'), False)
    except UnicodeDecodeError:
        return (data.decode('utf-8', errors='replace'), True)


def read_notes(path):
    '''read a notes file, returning an empty string if it is not there'''
    return read_notes_checked(path)[0]


def file_mtime(path):
    '''modification time of a file, or None if it is not there'''
    try:
        return os.stat(path).st_mtime
    except (IOError, OSError):
        return None


class NotesUI():
    '''
    a flight notes window for MAVProxy
    '''
    def __init__(self, sections, title='MAVProxy: Notes'):
        # sections is a list of (date, path, editable), latest date first.
        # Only plain data is kept here, as this object is passed to the
        # child process
        self.title = title
        self.sections = sections
        self.parent_pipe, self.child_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()
        # we only send on parent_pipe, so let go of the other end. That way
        # the child sees EOF if MAVProxy dies without closing us down
        self.child_pipe.close()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        self.parent_pipe.close()
        mp_util.child_close_fds()
        from MAVProxy.modules.lib import wx_processguard  # noqa: F401
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.lib import wxnotes_ui

        app = wx.App(False)
        app.frame = wxnotes_ui.NotesFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def close(self):
        '''close the window, giving it a chance to save first'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(CLOSE_TIMEOUT)
        if self.is_alive():
            # it is still trying to write. Killing it now would lose the
            # notes it is holding, so wait a good while longer before
            # deciding it is wedged
            print("notes: waiting for notes to be saved")
            self.child.join(CLOSE_TIMEOUT_SLOW)
        if self.is_alive():
            print("notes: notes window did not exit, giving up on it")
            self.child.terminate()
            self.child.join(1)

    def is_alive(self):
        '''check if the window is still going'''
        return self.child.is_alive()

    def raise_window(self):
        '''bring an already open window to the front'''
        if self.is_alive():
            self.parent_pipe.send(('raise',))
