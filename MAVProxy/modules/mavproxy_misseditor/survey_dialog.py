'''Live rectangular survey dialog for the mission editor.'''

import time
import math

from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.mavproxy_misseditor import survey


class SurveyDialog(wx.Dialog):
    def __init__(self, editor, row, origin, frame, height):
        try:
            height = float(height)
        except (TypeError, ValueError):
            raise ValueError('Survey requires a numeric waypoint altitude')
        if not math.isfinite(height) or not -10000 <= height <= 100000:
            raise ValueError('Survey requires a finite waypoint altitude between -10000 and 100000 m')
        super().__init__(editor, title='Create Survey')
        self.window_id = self.GetId()
        self.editor = editor
        self.row = row
        self.origin = origin
        self.snapshot = editor.survey_snapshot()
        self.result = None
        self.preview_visible = False
        self.next_snapshot_check = time.monotonic() + 1
        self.due = None
        self.retry_terrain = False
        self.closed = False
        self.controls = {}
        layout = wx.BoxSizer(wx.VERTICAL)
        description = wx.StaticText(self, label=(
            'Start at waypoint %u, extending forward and right.\n'
            'Rotation is clockwise from north. Camera points down.\n'
            'FOV and overlap set lane spacing; no camera triggers are added.' % (row + 1)))
        layout.Add(description, 0, wx.ALL, 10)
        fields = wx.FlexGridSizer(cols=2, vgap=6, hgap=12)
        fields.AddGrowableCol(1)
        for key, label, default, minimum, maximum, increment, digits in (
                ('length', 'Length (m)', 500, 1, 10000, 10, 0),
                ('breadth', 'Breadth (m)', 500, 1, 10000, 10, 0),
                ('height', 'Mission height (m)', height, -10000, 100000, 10, 0),
                ('fov', 'Camera cross-track FOV (deg)', 60, 0.1, 179.9, 1, 1),
                ('overlap', 'Side overlap (%)', 70, 0, 99.9, 1, 1)):
            fields.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            if digits == 0:
                control = wx.SpinCtrl(self, initial=round(float(default)),
                                      min=minimum, max=maximum)
                control.Bind(wx.EVT_SPINCTRL, self.on_change)
            else:
                control = wx.SpinCtrlDouble(self, value=str(default), min=minimum,
                                            max=maximum, inc=increment)
                control.SetDigits(digits)
                control.Bind(wx.EVT_SPINCTRLDOUBLE, self.on_change)
            fields.Add(control, 1, wx.EXPAND)
            self.controls[key] = control
            control.Bind(wx.EVT_TEXT, self.on_change)
        if height != round(height):
            self.controls['height'].SetToolTip('Waypoint altitude rounded to the nearest whole metre')
        fields.Add(wx.StaticText(self, label='Rotation (deg)'), 0, wx.ALIGN_CENTER_VERTICAL)
        rotation = wx.Slider(self, value=0, minValue=-180, maxValue=180,
                             style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        rotation.Bind(wx.EVT_SLIDER, self.on_change)
        self.controls['rotation'] = rotation
        fields.Add(rotation, 1, wx.EXPAND)
        fields.Add(wx.StaticText(self, label='Height frame'), 0, wx.ALIGN_CENTER_VERTICAL)
        self.frame_choice = wx.Choice(self, choices=list(survey.FRAMES))
        initial_frame = next((name for name, value in survey.FRAMES.items() if value == frame), None)
        if initial_frame is not None:
            self.frame_choice.SetStringSelection(initial_frame)
        self.frame_choice.Bind(wx.EVT_CHOICE, self.on_change)
        fields.Add(self.frame_choice, 1, wx.EXPAND)
        layout.Add(fields, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)
        hint = wx.StaticText(self, label=(
            'Approximate spacing assumes terrain is flat at the starting point.\n'
            'The first survey point sets the chosen height at that location.\n'
            'Preview appears in blue on any open Map window.\n'
            'Write inserts into the editor; Write WPs uploads the mission.'))
        layout.Add(hint, 0, wx.ALL, 10)
        self.status = wx.StaticText(self, size=(520, 48))
        layout.Add(self.status, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)
        buttons = wx.StdDialogButtonSizer()
        self.write_button = wx.Button(self, wx.ID_OK, 'Write')
        self.write_button.Bind(wx.EVT_BUTTON, self.on_write)
        buttons.AddButton(self.write_button)
        buttons.AddButton(wx.Button(self, wx.ID_CANCEL, 'Cancel'))
        buttons.Realize()
        layout.Add(buttons, 0, wx.ALIGN_RIGHT | wx.ALL, 10)
        self.SetSizerAndFit(layout)
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)
        self.timer.Start(200)
        self.update_preview()

    def clear_preview(self):
        if self.preview_visible:
            self.editor.send_survey_preview([])
            self.preview_visible = False
        self.result = None
        self.write_button.Disable()

    def on_change(self, event):
        self.result = None
        self.write_button.Disable()
        self.due = time.monotonic() + 0.15

    def on_timer(self, event):
        if self.closed:
            return
        now = time.monotonic()
        if now >= self.next_snapshot_check:
            self.next_snapshot_check = now + 1
            if self.snapshot != self.editor.survey_snapshot():
                self.clear_preview()
                self.status.SetLabel('Mission changed. Close this dialog and select a waypoint again.')
                self.timer.Stop()
                return
        if self.due is not None and now >= self.due:
            self.update_preview()

    def update_preview(self):
        self.due = None
        self.retry_terrain = False
        self.result = None
        self.write_button.Disable()
        try:
            if self.snapshot != self.editor.survey_snapshot():
                raise ValueError('Mission changed. Close this dialog and select a waypoint again.')
            try:
                values = {key: float(ctrl.GetValue()) for key, ctrl in self.controls.items()}
            except ValueError:
                raise ValueError('Enter a number in every field')
            frame = self.frame_choice.GetStringSelection()
            if frame not in survey.FRAMES:
                raise ValueError('Choose a height frame: the selected waypoint frame is unsupported')
            terrain = None
            if frame != 'AGL':
                terrain = self.editor.ElevationModel.GetElevation(*self.origin)
                self.retry_terrain = terrain is None
            try:
                home = float(self.editor.label_home_alt_value.GetLabel())
            except ValueError:
                home = None
            if not self.editor.home_received:
                home = None
            height_agl = survey.camera_height(values['height'], frame, home, terrain)
            self.result = survey.generate_survey(
                self.origin, values['length'], values['breadth'], height_agl,
                values['fov'], values['rotation'], values['overlap'])
            self.height = values['height']
            self.frame = survey.FRAMES[frame]
        except ValueError as ex:
            self.clear_preview()
            self.status.SetLabel(str(ex))
            if self.retry_terrain:
                self.due = time.monotonic() + 1
            return
        self.status.SetLabel('%u waypoints, %.1f m lane spacing, %.1f m camera height AGL'
                             % (len(self.result.points), self.result.spacing, self.result.height_agl))
        self.write_button.Enable()
        self.editor.send_survey_preview(self.result.points)
        self.preview_visible = True

    def on_write(self, event):
        # Recheck pending text edits and incoming mission changes synchronously.
        self.update_preview()
        if self.result is None:
            return
        try:
            self.editor.insert_survey(self.row, self.result.points, self.height, self.frame)
        except ValueError as ex:
            self.clear_preview()
            self.status.SetLabel(str(ex))
            return
        self.EndModal(wx.ID_OK)

    def on_destroy(self, event):
        # Child destroy events can arrive after our native window is gone.
        # Use the saved ID without calling back into the deleted wx object.
        if event.GetId() == self.window_id:
            self.cleanup()
        event.Skip()

    def cleanup(self):
        if not self.closed:
            self.closed = True
            self.timer.Stop()
            self.editor.send_survey_preview([])

    def Destroy(self):
        # wx may defer actual destruction until the next idle iteration.
        self.cleanup()
        return super().Destroy()
