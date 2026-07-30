#!/usr/bin/env python3

"""
  MAVProxy flight notes window GUI

  A collapsible section per date, latest first. The current date is editable
  and saves as you type; earlier dates can be expanded and read but not
  edited. Runs in the notes child process, see wxnotes.py.
"""

# AP_FLAKE8_CLEAN

import os
import time

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib.wxnotes import read_notes_checked, file_mtime
import wx.lib.scrolledpanel

# seconds of no typing before an edit is written out
SAVE_DELAY = 1.5

# how often the save timer looks for work
SAVE_TICK_MS = 1000

# how often the window looks for messages from MAVProxy
POLL_TICK_MS = 100

# height of a notes box in pixels
EDIT_HEIGHT = 260
VIEW_HEIGHT = 150


class NotesSection():
    '''one date's notes, in a collapsible pane'''

    def __init__(self, frame, parent, sizer, date, path, editable):
        self.date = date
        self.path = path
        self.editable = editable
        self.loaded = False

        label = date
        if editable:
            label += "  (editable)"
        self.pane = wx.CollapsiblePane(
            parent, label=label,
            style=wx.CP_DEFAULT_STYLE | wx.CP_NO_TLW_RESIZE)
        win = self.pane.GetPane()

        style = wx.TE_MULTILINE
        height = VIEW_HEIGHT
        if editable:
            height = EDIT_HEIGHT
        else:
            style |= wx.TE_READONLY
        self.text = wx.TextCtrl(win, style=style, size=(-1, height))

        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(self.text, 1, wx.EXPAND | wx.ALL, 3)

        if editable:
            # a row with the timestamp helper and the save state
            row = wx.BoxSizer(wx.HORIZONTAL)
            self.button_time = wx.Button(win, wx.ID_ANY, "Insert time")
            row.Add(self.button_time, 0, wx.ALL, 3)
            self.label_status = wx.StaticText(win, wx.ID_ANY, "")
            row.Add(self.label_status, 0,
                    wx.ALL | wx.ALIGN_CENTER_VERTICAL, 6)
            box.Add(row, 0, wx.EXPAND)
            win.Bind(wx.EVT_BUTTON, frame.on_insert_time, self.button_time)
        else:
            self.button_time = None
            self.label_status = None

        win.SetSizer(box)
        # today is open to type in straight away, earlier dates start closed
        self.was_lossy = False
        if editable:
            self.pane.Expand()
            self.was_lossy = self.load()
        sizer.Add(self.pane, 1 if editable else 0, wx.EXPAND | wx.ALL, 4)

    def load(self):
        '''read the file into the box, returning True if the bytes read do
           not round trip as UTF-8'''
        (text, lossy) = read_notes_checked(self.path)
        self.text.SetValue(text)
        self.loaded = True
        return lossy


class NotesFrame(wx.Frame):
    '''the notes window'''

    def __init__(self, state, title):
        wx.Frame.__init__(self, None, title=title, size=(700, 640),
                          style=wx.DEFAULT_FRAME_STYLE)
        self.state = state
        self.panel = wx.lib.scrolledpanel.ScrolledPanel(self)

        self.dirty = False
        self.last_change = 0
        self.saved_text = None
        self.saved_mtime = None
        self.load_was_lossy = False
        self.close_warned = False

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.sections = []
        self.edit_section = None
        for (date, path, editable) in state.sections:
            section = NotesSection(self, self.panel, sizer, date, path,
                                   editable)
            self.sections.append(section)
            if editable:
                self.edit_section = section
        if len(self.sections) < 2:
            sizer.Add(wx.StaticText(self.panel, wx.ID_ANY,
                                    "No notes from earlier dates"),
                      0, wx.ALL, 8)
        self.panel.SetSizer(sizer)
        self.panel.SetupScrolling(scroll_x=False)

        if self.edit_section is not None:
            self.saved_text = self.edit_section.text.GetValue()
            self.saved_mtime = file_mtime(self.edit_section.path)
            self.load_was_lossy = self.edit_section.was_lossy
            self.edit_section.text.Bind(wx.EVT_TEXT, self.on_text)
            self.edit_section.text.Bind(wx.EVT_KILL_FOCUS, self.on_kill_focus)
            self.edit_section.text.SetFocus()
            # ctrl-T as well as the button
            accel_id = wx.NewIdRef()
            self.Bind(wx.EVT_MENU, self.on_insert_time, id=accel_id)
            self.SetAcceleratorTable(wx.AcceleratorTable(
                [wx.AcceleratorEntry(wx.ACCEL_CTRL, ord('T'), accel_id)]))

        self.Bind(wx.EVT_COLLAPSIBLEPANE_CHANGED, self.on_pane_changed)
        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.save_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_save_timer, self.save_timer)
        self.save_timer.Start(SAVE_TICK_MS)

        self.poll_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_poll_timer, self.poll_timer)
        self.poll_timer.Start(POLL_TICK_MS)

    def set_status(self, text):
        if self.edit_section is not None:
            self.edit_section.label_status.SetLabel(text)

    def on_text(self, event):
        '''note that there is something to save'''
        self.dirty = True
        self.last_change = time.time()
        event.Skip()

    def on_kill_focus(self, event):
        self.save_now()
        event.Skip()

    def on_insert_time(self, event):
        '''insert the time at the cursor, for timestamping an entry'''
        if self.edit_section is None:
            return
        self.edit_section.text.WriteText(time.strftime("[%H:%M:%S] "))
        self.edit_section.text.SetFocus()

    def on_pane_changed(self, event):
        '''load a date's notes the first time it is opened'''
        pane = event.GetEventObject()
        for section in self.sections:
            if section.pane is pane:
                if not section.loaded and not pane.IsCollapsed():
                    section.load()
                break
        self.panel.Layout()
        self.panel.SetupScrolling(scroll_x=False, scrollToTop=False)

    def on_save_timer(self, event):
        '''save once typing has paused. A repeating timer means a save that
           failed is retried'''
        if self.dirty and time.time() - self.last_change > SAVE_DELAY:
            self.save_now()

    def on_poll_timer(self, event):
        '''watch for messages from MAVProxy'''
        state = self.state
        if state.close_event.is_set():
            self.shutdown()
            return
        while state.child_pipe.poll():
            try:
                msg = state.child_pipe.recv()
            except EOFError:
                # MAVProxy has gone without closing us down
                self.shutdown()
                return
            if isinstance(msg, tuple) and len(msg) > 0 and msg[0] == 'raise':
                self.Iconize(False)
                self.Raise()

    def on_close(self, event):
        '''save anything outstanding before going away'''
        if not self.save_now() and not self.close_warned and event.CanVeto():
            # don't drop the notes on the floor without saying so. Closing
            # again goes ahead, so the operator is never stuck
            self.close_warned = True
            self.set_status("%s -- close again to give up on these notes"
                            % self.edit_section.label_status.GetLabel())
            event.Veto()
            return
        self.shutdown()

    def shutdown(self):
        if not self.save_now():
            self.save_unsaved_copy()
        self.save_timer.Stop()
        self.poll_timer.Stop()
        self.Destroy()

    def save_now(self):
        '''write the current date's notes out, returning True if there is
           nothing left to save'''
        if self.edit_section is None:
            return True
        path = self.edit_section.path
        text = self.edit_section.text.GetValue()
        if text == self.saved_text and not self.load_was_lossy:
            self.dirty = False
            return True
        if text.strip() == "" and not os.path.exists(path):
            # don't create an empty file, it would make this date look like
            # a date that has notes
            self.dirty = False
            return True
        try:
            # this raises if the existing content cannot be preserved, so we
            # never destroy it
            warning = self.check_external_change(path)
            mp_util.mkdir_p(os.path.dirname(path))
            # a name of our own, so two MAVProxy instances writing the same
            # notes do not share a temporary file
            tmp = "%s.tmp.%u" % (path, os.getpid())
            try:
                with open(tmp, 'w', encoding='utf-8') as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, path)
            except Exception:
                self.remove_quietly(tmp)
                raise
            self.sync_dir(os.path.dirname(path))
            self.saved_text = text
            self.saved_mtime = file_mtime(path)
            self.dirty = False
            # what we have on disk is now ours, and writable
            self.load_was_lossy = False
            self.close_warned = False
            status = "saved %s" % time.strftime("%H:%M:%S")
            if warning is not None:
                status += "  " + warning
            self.set_status(status)
            return True
        except Exception as ex:
            # leave it dirty so the timer tries again
            self.set_status("save FAILED: %s" % ex)
            return False

    def check_external_change(self, path):
        '''if the file is not the one we read, keep a copy of what is there
           rather than losing it. Returns a warning to show, or None, and
           raises if the copy could not be made'''
        mtime = file_mtime(path)
        changed = mtime != self.saved_mtime
        if not changed and not self.load_was_lossy:
            return None
        if mtime is None:
            # nothing there to preserve
            self.saved_mtime = None
            self.load_was_lossy = False
            return None
        # copy the bytes as they are, so an original that is not valid UTF-8
        # is preserved exactly
        backup = path + '.bak'
        with open(path, 'rb') as src:
            data = src.read()
        with open(backup, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        self.saved_mtime = mtime
        was_lossy = self.load_was_lossy
        self.load_was_lossy = False
        if changed:
            return "notes.txt changed on disk, copy in notes.txt.bak"
        if was_lossy:
            return "notes.txt was not valid UTF-8, original in notes.txt.bak"
        return None

    def save_unsaved_copy(self):
        '''last resort when the notes cannot be written where they belong.
           Tries beside the notes first, then somewhere known to be writable,
           and failing that puts them on stdout so they are not lost'''
        if self.edit_section is None:
            return
        text = self.edit_section.text.GetValue()
        # named for this process, so a rescue copy never lands on top of
        # another one
        tag = "unsaved.%u" % os.getpid()
        candidates = ["%s.%s" % (self.edit_section.path, tag)]
        try:
            candidates.append(mp_util.dot_mavproxy(
                "notes-%s.%s.txt" % (self.edit_section.date, tag)))
        except Exception:
            pass
        for path in candidates:
            try:
                # never overwrite, so nothing already rescued is lost
                fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    f.write(text)
                print("notes: could not save, text written to %s" % path)
                return
            except (IOError, OSError):
                continue
        print("notes: could not save these notes anywhere, text follows:")
        print(text)

    def remove_quietly(self, path):
        try:
            os.unlink(path)
        except OSError:
            pass

    def sync_dir(self, dirname):
        '''make the rename durable. Best effort, not supported everywhere'''
        try:
            fd = os.open(dirname, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        except (IOError, OSError, AttributeError):
            pass
