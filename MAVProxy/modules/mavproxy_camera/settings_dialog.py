"""Process-isolated wx dialog for camera definition controls."""

import queue

from MAVProxy.modules.lib import multiproc


class CameraSettingsDialog:
    def __init__(self, title, snapshot):
        self.pipe, child_pipe = multiproc.Pipe()
        self.updates = multiproc.Queue(maxsize=1)
        self.close_event = multiproc.Event()
        self.child = multiproc.Process(
            target=_run_dialog,
            args=(child_pipe, self.updates, self.close_event, title, snapshot))
        self.child.start()
        child_pipe.close()

    def is_alive(self):
        return self.child.is_alive()

    def send(self, value):
        try:
            self.updates.put_nowait(value)
            return True
        except (queue.Full, OSError, ValueError):
            return False

    def events(self):
        try:
            while self.pipe.poll():
                yield self.pipe.recv()
        except (EOFError, OSError):
            return

    def close(self):
        self.close_event.set()
        self.child.join(timeout=1)
        if self.child.is_alive():
            self.child.terminate()
            self.child.join(timeout=1)
        self.pipe.close()
        self.updates.cancel_join_thread()
        self.updates.close()


class _DialogChannel:
    """Receive snapshots without blocking MAVProxy on a slow GUI."""

    def __init__(self, pipe, updates, close_event):
        self.pipe = pipe
        self.updates = updates
        self.close_event = close_event
        self.pending = None

    def poll(self):
        if self.close_event.is_set():
            self.pending = {"close": True}
        elif self.pending is None:
            try:
                self.pending = self.updates.get_nowait()
            except queue.Empty:
                return False
        return True

    def recv(self):
        state, self.pending = self.pending, None
        return state

    def send(self, event):
        self.pipe.send(event)


def _run_dialog(pipe, updates, close_event, title, snapshot):
    from MAVProxy.modules.lib import wx_processguard  # noqa: F401
    from MAVProxy.modules.lib.wx_loader import wx
    from MAVProxy.modules.lib import mp_util
    mp_util.child_close_fds()
    app = wx.App(False)
    frame = SettingsFrame(
        wx, _DialogChannel(pipe, updates, close_event), title, snapshot)
    frame.Show()
    app.MainLoop()
    pipe.close()


def SettingsFrame(wx, pipe, title, snapshot):
    """Construct wx classes only in the GUI process."""
    import math
    from wx.lib.scrolledpanel import ScrolledPanel

    class Frame(wx.Frame):
        def __init__(self):
            super().__init__(None, title='Custom Settings — ' + title, size=(760, 680))
            self.rows = {}
            self.updating = False
            panel = wx.Panel(self)
            layout = wx.BoxSizer(wx.VERTICAL)
            self.book = wx.Notebook(panel)
            rows = snapshot['rows']
            self.pages = []
            # Keep file order and stable pages even when exclusions hide rows.
            page_size = 14
            for start in range(0, max(1, len(rows)), page_size):
                page = ScrolledPanel(self.book)
                sizer = wx.BoxSizer(wx.VERTICAL)
                page.SetSizer(sizer)
                self.book.AddPage(page, 'Settings' if len(rows) <= page_size else
                                  'Settings %u' % (start // page_size + 1))
                self.pages.append(page)
                for row in rows[start:start + page_size]:
                    self.add_row(page, sizer, row)
                page.SetupScrolling(scroll_x=False)
            layout.Add(self.book, 1, wx.EXPAND | wx.ALL, 8)
            self.status = wx.StaticText(panel, label='')
            layout.Add(self.status, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)
            buttons = wx.BoxSizer(wx.HORIZONTAL)
            refresh = wx.Button(panel, label='Refresh')
            refresh.Bind(wx.EVT_BUTTON, lambda e: pipe.send(('refresh',)))
            buttons.Add(refresh, 0, wx.ALL, 8)
            buttons.AddStretchSpacer()
            close = wx.Button(panel, wx.ID_CLOSE)
            close.Bind(wx.EVT_BUTTON, lambda e: self.Close())
            buttons.Add(close, 0, wx.ALL, 8)
            layout.Add(buttons, 0, wx.EXPAND)
            panel.SetSizer(layout)
            self.timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.poll, self.timer)
            self.Bind(wx.EVT_CLOSE, self.on_close)
            self.update(snapshot)
            self.timer.Start(100)

        def add_row(self, page, sizer, row):
            container = wx.Panel(page)
            line = wx.BoxSizer(wx.HORIZONTAL)
            name = row['name']
            label = wx.StaticText(container, label=row['description'])
            label.Wrap(315)
            line.Add(label, 1, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
            kind = 'text'
            if row['type'] == 'bool':
                control = wx.CheckBox(container, size=(260, -1))
                kind = 'bool'
                control.Bind(wx.EVT_CHECKBOX, lambda e: self.submit(name, int(control.GetValue())))
            elif row['options']:
                control = wx.Choice(container, size=(260, -1))
                kind = 'choice'
                control.Bind(wx.EVT_CHOICE, lambda e: self.choose(name))
            elif (row['step'] and row['step'] > 0 and row['minimum'] is not None and
                  row['maximum'] is not None and
                  0 < (row['maximum'] - row['minimum']) / row['step'] <= 100000):
                steps = int(math.floor((row['maximum'] - row['minimum']) / row['step'] + 1e-5))
                control = wx.Slider(container, minValue=0, maxValue=steps, size=(200, -1))
                kind = 'slider'
                control.Bind(wx.EVT_SCROLL_CHANGED, lambda e: self.submit(
                    name, row['minimum'] + control.GetValue() * row['step']))
            else:
                control = wx.TextCtrl(container, style=wx.TE_PROCESS_ENTER, size=(170, -1))
                control.Bind(wx.EVT_TEXT_ENTER, lambda e: self.submit(name, control.GetValue()))
            line.Add(control, 0, wx.ALIGN_CENTER_VERTICAL)
            apply_button = None
            value_label = None
            if kind == 'text' and not row['readonly']:
                apply_button = wx.Button(container, label='Apply', size=(65, -1))
                apply_button.Bind(wx.EVT_BUTTON, lambda e: self.submit(name, control.GetValue()))
                line.Add(apply_button, 0, wx.LEFT, 4)
            elif kind == 'slider':
                value_label = wx.StaticText(container, label='', size=(65, -1))
                line.Add(value_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 4)
            vertical = wx.BoxSizer(wx.VERTICAL)
            vertical.Add(line, 0, wx.EXPAND)
            detail = wx.StaticText(container, label='')
            vertical.Add(detail, 0, wx.TOP, 2)
            container.SetSizer(vertical)
            sizer.Add(container, 0, wx.EXPAND | wx.ALL, 5)
            tooltip = name
            if row['minimum'] is not None:
                tooltip += '\nMinimum: %s' % row['minimum']
            if row['maximum'] is not None:
                tooltip += '\nMaximum: %s' % row['maximum']
            if row['step']:
                tooltip += '\nStep: %s' % row['step']
            control.SetToolTip(tooltip)
            label.SetToolTip(name)
            self.rows[name] = dict(container=container, control=control, kind=kind,
                                   apply=apply_button, value_label=value_label, detail=detail,
                                   state=row, choices=[], submitted=False)

        def submit(self, name, value):
            entry = self.rows[name]
            if self.updating or not entry['state']['enabled'] or entry['submitted']:
                return
            entry['submitted'] = True
            entry['control'].Enable(False)
            if entry['apply']:
                entry['apply'].Enable(False)
            pipe.send(('set', name, value))

        def choose(self, name):
            entry = self.rows[name]
            index = entry['control'].GetSelection()
            if 0 <= index < len(entry['choices']):
                value = entry['choices'][index][1]
                if value is not None:
                    self.submit(name, value)

        def update(self, state):
            from MAVProxy.modules.mavproxy_camera.definition import equal_value
            self.updating = True
            for row in state['rows']:
                entry = self.rows[row['name']]
                previous = entry['state']
                entry['state'] = row
                control = entry['control']
                entry['container'].Show(row['visible'])
                control.Enable(row['enabled'])
                if entry['apply']:
                    entry['apply'].Enable(row['enabled'])
                value = row['value']
                changed = (value != previous['value'] or previous['pending'] or
                           entry['submitted'] or bool(row['error']))
                if entry['kind'] == 'bool':
                    control.SetValue(bool(value))
                elif entry['kind'] == 'choice':
                    choices = list(row['options'])
                    index = next((i for i, (_, v) in enumerate(choices)
                                  if equal_value(value, v)), -1)
                    if index == -1:
                        choices.insert(0, ('Waiting for camera' if value is None else
                                           'Current: %s (outside options)' % value, None))
                        index = 0
                    if choices != entry['choices']:
                        control.SetItems([label for label, _ in choices])
                        entry['choices'] = choices
                    control.SetSelection(index)
                elif entry['kind'] == 'slider':
                    if value is not None:
                        control.SetValue(round((value - row['minimum']) / row['step']))
                        entry['value_label'].SetLabel('%g' % value)
                elif changed or not control.IsModified():
                    text = '' if value is None else str(value)
                    if value is not None and row['type'] == 'float':
                        text = format(value, '.7g')
                    elif value is not None and row['type'] == 'double':
                        text = format(value, '.15g')
                    control.ChangeValue(text)
                entry['submitted'] = False
                detail = row['error'] or ('Applying…' if row['pending'] else
                                          'Waiting for camera' if value is None else '')
                entry['detail'].SetLabel(detail)
                entry['detail'].Show(bool(detail))
                entry['container'].Layout()
                entry['detail'].SetForegroundColour(wx.RED if row['error'] else wx.NullColour)
            self.status.SetLabel(state['status'])
            self.status.Wrap(720)
            for page in self.pages:
                page.Layout()
                page.FitInside()
            self.Layout()
            self.updating = False

        def poll(self, event):
            try:
                while pipe.poll():
                    state = pipe.recv()
                    if state.get('close'):
                        self.Close()
                        return
                    if state.get('raise'):
                        self.Raise()
                    else:
                        self.update(state)
            except (EOFError, OSError):
                self.Close()

        def on_close(self, event):
            self.timer.Stop()
            self.Destroy()

    return Frame()
