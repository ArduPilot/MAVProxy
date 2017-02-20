# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
import math
import os
import time

from pymavlink.mavutil import mavlink
from wx.lib.wordwrap import wordwrap

from MAVProxy.modules import mavproxy_magical as magical
from MAVProxy.modules.lib import wavefront as wv
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.mavproxy_magical.wxgeodesicgrid import GeodesicGrid
from MAVProxy.modules.mavproxy_magical.wxvehicle import Vehicle

COMMON_FOREGROUND = '#212121'
LIGHT_BACKGROUND = '#fafafa'
DARK_BACKGROUND = '#d1d1d1'
PRIMARY_HIGHLIGHT = '#3359ec'
SECONDARY_HIGHLIGHT = '#0c666a'

def scale_color(color, scale):
    r, g, b = color
    r, g, b = round(scale * r), round(scale * g), round(scale * b)
    r, g, b = min(255, r), min(255, g), min(255, b)
    return wx.Colour(r, g, b)

# Class only for allowing getting a common default foreground color
class Panel(wx.Panel):
    def __init__(self, *k, **kw):
        super(Panel, self).__init__(*k, **kw)
        self.SetForegroundColour(COMMON_FOREGROUND)

# Button with a custom style
class Button(wx.PyControl):
    def __init__(self, *k, **kw):
        if 'style' not in kw:
            kw['style'] = wx.BORDER_NONE
        super(Button, self).__init__(*k, **kw)

        self.border_radius = 0
        self.border_width = 0
        self.border_color = None
        self.padding = wx.Size(0, 0)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)

        self.SetCursor(wx.StockCursor(wx.CURSOR_HAND))

    def OnLeftUp(self, evt):
        command_evt = wx.CommandEvent(wx.EVT_BUTTON.typeId, self.GetId())
        command_evt.SetEventObject(self)
        self.GetEventHandler().AddPendingEvent(command_evt);

    def GetBorderRadius(self):
        return self.border_radius

    def SetBorderRadius(self, radius):
        self.border_radius = radius

    def GetBorderWidth(self):
        return self.border_width

    def SetBorderWidth(self, width):
        self.border_width = width

    def GetBorderColor(self):
        if not self.border_color:
            return self.GetBackgroundColour()
        return self.border_color

    def SetBorderColor(self, color):
        self.border_color = color

    def GetPadding(self):
        return self.padding

    def SetPadding(self, size):
        self.padding = size

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        self.Draw(dc)

    def Draw(self, dc):
        width, height = self.GetClientSize()
        if not width or not height:
            return

        gcdc = wx.GCDC(dc)

        gcdc.SetPen(wx.NullPen)

        bg = self.GetParent().GetBackgroundColour()
        gcdc.SetBackground(wx.Brush(bg, wx.SOLID))
        gcdc.Clear()

        border_radius = self.GetBorderRadius()
        border = self.GetBorderWidth()

        if border:
            gcdc.SetBrush(wx.Brush(self.GetBorderColor()))
            gcdc.DrawRoundedRectangle(0, 0, width, height, border_radius)

        w, h = width - 2 * border, height - 2 * border
        border_radius -= border

        gcdc.SetBrush(wx.Brush(self.GetBackgroundColour()))
        gcdc.DrawRoundedRectangle(border, border, w, h, border_radius)

        dc.SetFont(self.GetFont())
        label = self.GetLabel()
        w, h = dc.GetTextExtent(label)
        x, y = (width - w) / 2, (height - h) / 2
        dc.DrawText(label, x, y)

    def DoGetBestSize(self):
        dc = wx.ClientDC(self)
        dc.SetFont(self.GetFont())
        w, h = dc.GetTextExtent(self.GetLabel())
        pw, ph = self.GetPadding()
        w, h = w + pw, h + ph
        border = self.GetBorderWidth()
        w, h = w + 2 * border, h + 2 * border
        return wx.Size(w, h)

class CountdownText(wx.PyWindow):
    def __init__(self, *k, **kw):
        super(CountdownText, self).__init__(*k, **kw)

        self.value = 0
        self.border_color = COMMON_FOREGROUND

        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def ShouldInheritColours(self):
        return True

    def GetBorderColor(self):
        return self.border_color

    def SetBorderColor(self, color):
        self.border_color = color

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        self.Draw(dc)

    def Draw(self, dc):
        width, height = self.GetClientSize()
        if not width or not height:
            return

        gcdc = wx.GCDC(dc)

        bg = self.GetParent().GetBackgroundColour()
        gcdc.SetBackground(wx.Brush(bg, wx.SOLID))
        gcdc.Clear()

        x, y = width / 2, height / 2
        radius = min(x, y)
        border = int(radius / 20)
        gcdc.SetPen(wx.Pen(self.GetBorderColor(), border))
        gcdc.SetBrush(wx.Brush(bg, wx.SOLID))
        gcdc.DrawCircle(x, y, radius - border)

        # Set font size 70% of the diameter
        v = str(self.value)
        s = max(dc.GetTextExtent(v))
        scale = 2 * radius * .7 / s
        font = self.GetFont()
        font.SetPointSize(round(font.GetPointSize() * scale))
        dc.SetFont(font)

        w, h = dc.GetTextExtent(v)
        x, y = (width - w) / 2, (height - h) / 2
        dc.DrawText(v, x, y)

    def SetValue(self, value):
        self.value = value
        self.Refresh()

class ReportDialog(wx.Dialog):
    light_background = wx.Colour()
    light_background.SetFromString(LIGHT_BACKGROUND)
    light_background = scale_color(light_background, 1.2)

    class StatusIcon(wx.PyWindow):
        success_color = '#00ed00'
        failure_color = '#d81313'

        def __init__(self, *k, **kw):
            super(ReportDialog.StatusIcon, self).__init__(*k, **kw)
            self.success = True
            self.Bind(wx.EVT_PAINT, self.OnPaint)

        def Success(self, success):
            self.success = success
            self.Refresh()

        def OnPaint(self, evt):
            dc = wx.BufferedPaintDC(self)
            self.Draw(dc)

        def Draw(self, dc):
            width, height = self.GetClientSize()
            if not width or not height:
                return

            gcdc = wx.GCDC(dc)

            gcdc.SetPen(wx.NullPen)

            bg = self.GetParent().GetBackgroundColour()
            gcdc.SetBackground(wx.Brush(bg, wx.SOLID))
            gcdc.Clear()

            color = self.success_color if self.success else self.failure_color
            gcdc.SetBrush(wx.Brush(color))

            x = width / 2
            y = height / 2
            gcdc.DrawCircle(x, y, min(x, y))

    class CompassPanel(Panel):
        def __init__(self, parent, m, *k, **kw):
            super(ReportDialog.CompassPanel, self).__init__(parent, *k, **kw)

            self.InitUI()

            self.compass_text.SetLabel('Compass %d' % m.compass_id)

            if m.cal_status == mavlink.MAG_CAL_SUCCESS:
                self.status_icon.Success(True)
                self.status_text.SetLabel('SUCCESS')
            else:
                self.status_icon.Success(False)
                self.status_text.SetLabel('FAILURE')

            self.fitness_value_text.SetLabel('%.3f' % m.fitness)

            texts = (
                self.offsets_texts,
                self.diagonals_texts,
                self.offdiagonals_texts,
            )
            params = (
                (m.ofs_x, m.ofs_y, m.ofs_z),
                (m.diag_x, m.diag_y, m.diag_z),
                (m.offdiag_x, m.offdiag_y, m.offdiag_z),
            )
            for targets, values in zip(texts, params):
                for target, value in zip(targets, values):
                    target.SetLabel('%.2f' % value)

        def InitUI(self):
            color = wx.Colour()
            color.SetFromString(LIGHT_BACKGROUND)
            self.SetBackgroundColour(scale_color(color, 0.95))

            font = self.GetFont()
            font.SetWeight(wx.FONTWEIGHT_BOLD)
            font.SetPointSize(24)
            self.SetFont(font)

            self.SetMinSize((400, -1))

            self.compass_text = wx.StaticText(self)

            self.fitness_value_text = wx.StaticText(self)
            font.SetWeight(wx.FONTWEIGHT_NORMAL)
            self.fitness_value_text.SetFont(font)

            self.parameters_sizer = self.CalibrationParameters()

            fitness_sizer = wx.BoxSizer(wx.HORIZONTAL)
            text = wx.StaticText(self, label='Fitness')
            text.SetFont(self.fitness_value_text.GetFont())
            fitness_sizer.Add(text, proportion=1, flag=wx.EXPAND)
            fitness_sizer.Add(self.fitness_value_text)

            sizer = wx.BoxSizer(wx.VERTICAL)
            self.SetSizer(sizer)
            sizer.AddSpacer(16)
            sizer.Add(self.compass_text, border=16, flag=wx.LEFT | wx.RIGHT)
            sizer.AddSpacer(16)
            sizer.Add(self.StatusPanel(), flag=wx.EXPAND)
            sizer.AddSpacer(16)
            sizer.Add(fitness_sizer, border=16, flag=wx.LEFT | wx.RIGHT | wx.EXPAND)
            sizer.AddSpacer(16)
            sizer.Add(self.parameters_sizer, border=16, flag=wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND)

            sizer.Hide(self.parameters_sizer)

        def StatusPanel(self):
            panel = Panel(self)

            panel.SetBackgroundColour(ReportDialog.light_background)

            self.status_icon = ReportDialog.StatusIcon(panel, size=(44, 44))
            self.status_text = wx.StaticText(panel)

            sizer = wx.BoxSizer(wx.HORIZONTAL)
            panel.SetSizer(sizer)
            sizer.AddSpacer(16)
            sizer.Add(self.status_icon, border=5, flag=wx.TOP | wx.BOTTOM)
            sizer.AddSpacer(16)
            sizer.Add(self.status_text, flag=wx.ALIGN_CENTER)
            sizer.AddSpacer(16)

            return panel

        def CalibrationParameters(self):
            self.offsets_texts = tuple(wx.StaticText(self) for _ in range(3))
            self.diagonals_texts = tuple(wx.StaticText(self) for _ in range(3))
            self.offdiagonals_texts = tuple(wx.StaticText(self) for _ in range(3))

            table = (
                ('Offsets', self.offsets_texts),
                ('Diagonals', self.diagonals_texts),
                ('Off-diagonals', self.offdiagonals_texts),
            )

            sizer = wx.FlexGridSizer(len(table) + 1, 4, 4, 10)
            sizer.AddGrowableCol(0)

            font = self.GetFont()
            font.SetPointSize(14)
            font.MakeItalic()

            text = wx.StaticText(self, label='Parameter')
            text.SetFont(font)
            sizer.Add(text)
            for label in ('X', 'Y', 'Z'):
                text = wx.StaticText(self, label=label)
                text.SetFont(font)
                sizer.Add(text, flag=wx.ALIGN_CENTER)

            font.SetWeight(wx.FONTWEIGHT_NORMAL)

            for label, (x, y, z) in table:
                text = wx.StaticText(self)
                text.SetLabel(label)

                text.SetFont(font)
                x.SetFont(font)
                y.SetFont(font)
                z.SetFont(font)

                sizer.Add(text)
                sizer.Add(x, flag=wx.ALIGN_RIGHT)
                sizer.Add(y, flag=wx.ALIGN_RIGHT)
                sizer.Add(z, flag=wx.ALIGN_RIGHT)

            return sizer

        def ShowCompassParameters(self, show):
            sizer = self.GetSizer()
            if show:
                sizer.Show(self.parameters_sizer)
            else:
                sizer.Hide(self.parameters_sizer)

    def __init__(self, parent, mavlink_messages, *k, **kw):
        super(ReportDialog, self).__init__(parent, *k, **kw)

        self.compass_parameters_shown = False
        self.mavlink_messages = mavlink_messages

        for m in self.mavlink_messages:
            if m.cal_status != mavlink.MAG_CAL_SUCCESS:
                self.success = False
                break
        else:
            self.success = True

        self.InitUI()

    def InitUI(self):
        color = wx.Colour()
        color.SetFromString(DARK_BACKGROUND)
        self.SetBackgroundColour(scale_color(color, .9))

        cols = 2 if len(self.mavlink_messages) > 1 else 1
        compass_sizer = wx.GridSizer(0, cols, 2, 2)

        self.compass_panels = []
        for m in self.mavlink_messages:
            p = self.CompassPanel(self, m)
            compass_sizer.Add(p)
            self.compass_panels.append(p)

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)
        sizer.Add(compass_sizer)
        sizer.Add(self.Footer(), flag=wx.EXPAND)
        sizer.Fit(self)

    def Footer(self):
        footer = Panel(self)

        footer.SetBackgroundColour(ReportDialog.light_background)

        font = footer.GetFont()
        font.SetPointSize(16)
        font.SetWeight(wx.FONTWEIGHT_BOLD)
        footer.SetFont(font)

        self.show_more_button = Button(footer)
        self.show_more_button.SetLabel('Show more')
        self.show_more_button.SetBackgroundColour(wx.TransparentColour)

        self.Bind(wx.EVT_BUTTON, self.OnShowMoreButton, self.show_more_button)

        if not self.success:
            self.try_again_button = Button(footer, wx.ID_REDO)
            self.try_again_button.SetLabel('TRY AGAIN')
            self.try_again_button.SetBackgroundColour(wx.TransparentColour)
            self.try_again_button.SetForegroundColour(PRIMARY_HIGHLIGHT)

            self.Bind(wx.EVT_BUTTON, self.OnActionButton, self.try_again_button)

        self.close_button = Button(footer, wx.ID_CLOSE)
        self.close_button.SetLabel('CLOSE')
        self.close_button.SetBackgroundColour(wx.TransparentColour)
        self.close_button.SetForegroundColour(PRIMARY_HIGHLIGHT)

        self.Bind(wx.EVT_BUTTON, self.OnActionButton, self.close_button)

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        footer.SetSizer(sizer)
        sizer.AddSpacer(16)
        sizer.Add(self.show_more_button)
        sizer.AddStretchSpacer()
        if not self.success:
            sizer.Add(self.try_again_button)
            sizer.AddSpacer(32)
        sizer.Add(self.close_button)
        sizer.AddSpacer(16)

        return footer

    def OnActionButton(self, evt):
        r = evt.GetEventObject().GetId()
        if self.IsModal():
            self.EndModal(r)
        else:
            self.SetReturnCode(r)
            self.Show(False)

    def OnShowMoreButton(self, evt):
        self.compass_parameters_shown = not self.compass_parameters_shown
        for p in self.compass_panels:
            p.ShowCompassParameters(self.compass_parameters_shown)

        if self.compass_parameters_shown:
            self.show_more_button.SetLabel('Show less')
        else:
            self.show_more_button.SetLabel('Show more')

        self.GetSizer().Fit(self)

# Unfortunately, wx.StaticText sucks at wrapping text automatically, thus we
# had to come up with our one (very simple) widget
class InstructionText(wx.PyWindow):
    def __init__(self, *k, **kw):
        super(InstructionText, self).__init__(*k, **kw)

        self.text = ''
        self.min_lines = 1

        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def ShouldInheritColours(self):
        return True

    def SetText(self, text):
        self.text = text
        self.Refresh()

    def SetMinLines(self, lines):
        self.min_lines = lines

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        self.Draw(dc)

    def Draw(self, dc):
        width, height = self.GetClientSize()
        if not width or not height:
            return

        bg = self.GetParent().GetBackgroundColour()
        dc.SetBackground(wx.Brush(bg, wx.SOLID))
        dc.Clear()

        dc.SetFont(self.GetFont())
        text = wordwrap(self.text, width, dc)
        lines = text.splitlines()
        _, h = dc.GetTextExtent(text)
        y = (height - h) / 2
        for line in lines:
            w, h = dc.GetTextExtent(line)
            x = (width - w) / 2
            dc.DrawText(line, x, y)
            y += h

    def DoGetBestSize(self):
        _, height = self.GetClientSize()
        dc = wx.ClientDC(self)
        dc.SetFont(self.GetFont())
        _, h = dc.GetTextExtent('\n'.join([' '] * self.min_lines))
        return -1, max(h, height)

class MagicalFrame(wx.Frame):
    initial_instruction = 'To start calibration, please place your vehicle as the image shows and press start.'
    countdown_instruction = 'Get ready!'
    calibration_instructions = (
        'Rotate the vehicle in as many different ways as possible.',
        'Some rotations can be repeated from different initial positions.',
        'You can take the rotations above as a reference.',
        'The image on the right is a visual feedback of the calibration progress.',
        'Don\'t worry about the rotation direction (clockwise or counterclockwise), both work.',
    )

    ref_rotations_script = ()

    for yaw in (0, math.pi / 2):
        ref_rotations_script += (
            ('.SetEuler', 0, 0, yaw),
            ('.SetAngvel', (1, 0, 0), math.pi / 2),
            ('wait', 4),
            ('.SetEuler', 0, 0, yaw),
            ('.SetAngvel', (0, 1, 0), math.pi / 2),
            ('wait', 4),
            ('.SetEuler', 0, 0, yaw),
            ('.SetAngvel', (1, 1, 0), math.pi / 2),
            ('wait', 4),
            ('.SetEuler', 0, 0, yaw),
            ('.SetAngvel', (1, -1, 0), math.pi / 2),
            ('wait', 4),
        )

    for (roll, pitch, yaw), rotation_axis in (
        ((0, 0, 0), 2),
        ((math.pi, 0, 0), 2),
        ((-math.pi / 2, 0, 0), 1),
        ((math.pi / 2, 0, 0), 1),
        ((0, -math.pi / 2, 0), 0),
        ((0, math.pi / 2, 0), 0),
    ):
        axis = [0, 0, 0]
        axis[rotation_axis] = 1
        ref_rotations_script += (
            ('.SetEuler', roll, pitch, yaw),
            ('.SetAngvel', axis, math.pi / 2),
            ('wait', 4),
        )


    ref_rotations_script += (('restart',),)

    def __init__(self, conn):
        super(MagicalFrame, self).__init__(None, title='Magical')

        self.state = None
        self.grid = None
        self.conn = conn
        self.button_state = None
        self.ui_is_active = False
        self.countdown = 0
        self.calibration_instruction_idx = 0

        self.InitUI()

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.timer.Start(60)

        self.loader_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnLoaderTimer, self.loader_timer)
        self.loader_timer.Start(50)

        self.countdown_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnCountdownTimer, self.countdown_timer)

        self.calibration_instructions_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnCalibrationInstructionsTimer,
                  self.calibration_instructions_timer)

        self.SetState('idle')
        self.main_panel.GetSizer().Fit(self)

        path = os.path.join(magical.datapath, 'quadcopter.obj')
        self.vehicle_loader = wv.ParserWorker(
            wv.ObjParser(filename=path),
            complete_callback=self.VehicleLoadCompleteCallback,
        )
        self.vehicle_loader.start()

    def VehicleLoadCompleteCallback(self):
        wx.CallAfter(self.VehicleLoadComplete)

    def VehicleLoadComplete(self):
        self.grid.SetVehicleWavefront(self.vehicle_loader.obj)
        self.ref_vehicle.SetVehicleWavefront(self.vehicle_loader.obj)
        self.vehicle_loader = None
        self.loader_timer.Stop()
        self.HideLoadProgress()

    def InitUI(self):
        font = self.GetFont()
        font.SetFamily(wx.FONTFAMILY_SWISS)
        self.SetFont(font)

        self.main_panel = wx.ScrolledWindow(self)
        self.main_panel.SetScrollbars(1, 1, 1, 1)
        self.main_panel.SetBackgroundColour(DARK_BACKGROUND)

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.main_panel.SetSizer(sizer)

        kw = dict(border=32, flag=wx.EXPAND | wx.LEFT | wx.RIGHT)

        sizer.AddSpacer(16)
        sizer.Add(self.Header(self.main_panel), **kw)
        sizer.AddSpacer(23)
        sizer.Add(self.Body(self.main_panel), proportion=1, **kw)
        sizer.AddSpacer(23)
        sizer.Add(self.Footer(self.main_panel), **kw)
        sizer.AddSpacer(23)

    def Header(self, parent):
        header = Panel(parent)
        header.SetBackgroundColour(LIGHT_BACKGROUND)

        sizer = wx.BoxSizer(wx.VERTICAL)
        header.SetSizer(sizer)

        font = header.GetFont()
        font.SetWeight(wx.FONTWEIGHT_BOLD)

        text = wx.StaticText(header)
        text.SetLabel('Compass Calibration')
        text.SetForegroundColour(PRIMARY_HIGHLIGHT)
        font.SetPointSize(26)
        text.SetFont(font)
        sizer.Add(text, border=16, flag=wx.ALL & ~wx.BOTTOM)

        text = wx.StaticText(header)
        text.SetLabel('Follow the procedure to calibrate your compass')
        font.SetPointSize(14)
        text.SetFont(font)
        sizer.Add(text, border=16, flag=wx.ALL & ~wx.TOP)

        return header

    def Body(self, parent):
        body = wx.BoxSizer(wx.HORIZONTAL)
        body.Add(self.InstructionsPanel(parent), proportion=2, flag=wx.EXPAND)
        body.AddSpacer(23)
        self.progress_panel = self.ProgressPanel(parent)
        body.Add(self.progress_panel, proportion=1, flag=wx.EXPAND)
        return body

    def InstructionsPanel(self, parent):
        panel = Panel(parent)
        panel.SetBackgroundColour(LIGHT_BACKGROUND)

        self.ref_vehicle = Vehicle(panel)
        self.ref_vehicle.SetMinSize((430, 430))

        self.countdown_window = CountdownText(panel)
        self.countdown_window.SetMinSize((430, 430))
        self.countdown_window.SetBorderColor(SECONDARY_HIGHLIGHT)
        self.countdown_window.SetForegroundColour(PRIMARY_HIGHLIGHT)
        font = self.countdown_window.GetFont()
        font.SetWeight(wx.FONTWEIGHT_BOLD)
        self.countdown_window.SetFont(font)

        self.instruction = InstructionText(panel)
        self.instruction.SetMinLines(3)
        font = self.instruction.GetFont()
        font.SetPointSize(26)
        self.instruction.SetFont(font)

        sizer = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(sizer)
        sizer.AddStretchSpacer()
        sizer.Add(self.ref_vehicle, flag=wx.ALIGN_CENTER)
        sizer.Add(self.countdown_window, flag=wx.ALIGN_CENTER)
        sizer.AddStretchSpacer()
        sizer.Add(self.instruction, border=10, flag=wx.ALL | wx.EXPAND)
        sizer.AddStretchSpacer()

        sizer.Hide(self.countdown_window)

        return panel

    def ProgressPanel(self, parent):
        panel = Panel(parent)
        panel.SetBackgroundColour(LIGHT_BACKGROUND)

        self.progress = wx.StaticText(panel, label='0%')
        font = self.progress.GetFont()
        font.SetPointSize(70)
        font.SetWeight(wx.FONTWEIGHT_BOLD)
        self.progress.SetFont(font)

        self.grid = GeodesicGrid(panel)
        self.grid.SetMinSize((365, 365))

        sizer = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(sizer)
        sizer.AddStretchSpacer()
        sizer.Add(self.grid, flag=wx.ALIGN_CENTER)
        sizer.AddStretchSpacer()
        sizer.AddSpacer(16)
        sizer.Add(self.progress, flag=wx.ALIGN_CENTER)
        sizer.AddStretchSpacer()

        return panel

    def Footer(self, parent):
        footer = wx.BoxSizer(wx.HORIZONTAL)

        self.button = button = Button(parent, size=(180, -1))
        button.SetPadding((0, 16))
        button.SetBackgroundColour(LIGHT_BACKGROUND)
        button.SetBorderRadius(10)
        button.SetBorderWidth(2)
        font = button.GetFont()
        font.SetWeight(wx.FONTWEIGHT_BOLD)
        font.SetPointSize(20)
        button.SetFont(font)

        self.Bind(wx.EVT_BUTTON, self.OnButton, button)

        footer.Add(self.LoaderProgress(parent), proportion=1, flag=wx.EXPAND)
        footer.Add(button)

        return footer

    def LoaderProgress(self, parent):
        panel = wx.Panel(parent)
        panel.SetBackgroundColour(parent.GetBackgroundColour())

        font = panel.GetFont()
        font.MakeItalic()
        panel.SetFont(font)

        self.load_progress = wx.Gauge(panel, size=(-1, 10))
        self.load_progress_last_time = time.time()
        self.load_text = wx.StaticText(panel)

        sizer = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(sizer)
        sizer.Add(self.load_progress, border=10, flag=wx.EXPAND | wx.LEFT | wx.RIGHT)
        sizer.Add(self.load_text, border=10, flag=wx.LEFT | wx.RIGHT)

        return panel

    def UpdateLoaderProgress(self, text, value, range):
        if value == -1:
            t = time.time()
            if t - self.load_progress_last_time > .001:
                self.load_progress.Pulse()
                self.load_progress_last_time = t
        else:
            self.load_progress.SetRange(range)
            self.load_progress.SetValue(value)
        if text != self.load_text.GetLabel():
            self.load_text.SetLabel(text)
        self.ShowLoadProgress()

    def ShowLoadProgress(self):
        sizer = self.load_progress.GetContainingSizer()
        sizer.Show(self.load_progress)
        sizer.Show(self.load_text)

    def HideLoadProgress(self):
        sizer = self.load_progress.GetContainingSizer()
        sizer.Hide(self.load_progress)
        sizer.Hide(self.load_text)

    def SetState(self, state):
        if self.state == state:
            return

        if self.state == 'countdown':
            self.countdown = 0
            self.countdown_timer.Stop()
            self.countdown_window.SetValue(self.countdown)

            sizer = self.countdown_window.GetContainingSizer()
            sizer.Show(self.ref_vehicle)
            sizer.Hide(self.countdown_window)
            sizer.Layout()

        elif self.state == 'running':
            self.calibration_instructions_timer.Stop()
            self.ref_vehicle.StopScript()
            self.ref_vehicle.SetEuler(0, 0, 0)

        if state == 'idle':
            self.SetInstructionText(self.initial_instruction)
            self.SetButtonState('start')

        elif state == 'countdown':
            self.countdown = 5
            self.countdown_window.SetValue(self.countdown)
            self.SetInstructionText(self.countdown_instruction)
            self.countdown_timer.Start(1000)
            self.SetButtonState('cancel')

            sizer = self.countdown_window.GetContainingSizer()
            sizer.Show(self.countdown_window)
            sizer.Hide(self.ref_vehicle)
            sizer.Layout()

        elif state == 'start':
            self.conn.send('start')

        elif state == 'running':
            self.calibration_instruction_idx = 0
            self.SetInstructionText(self.calibration_instructions[0])
            self.calibration_instructions_timer.Start(5000)

            self.SetButtonState('cancel')
            self.ref_vehicle.RunScript(self.ref_rotations_script)

        elif state == 'cancel':
            self.conn.send('cancel')

            if self.state == 'countdown':
                self.SetState('idle')

        self.state = state

    def OnButton(self, evt):
        if self.button_state == 'start':
            self.SetState('countdown')
        elif self.button_state == 'cancel':
            self.SetState('cancel')

    def SetInstructionText(self, text):
        self.instruction.SetText(text)
        self.instruction.GetContainingSizer().Layout()

    def OnCountdownTimer(self, evt):
        self.countdown -= 1
        self.countdown_window.SetValue(self.countdown)
        if not self.countdown:
            self.SetState('start')

    def OnLoaderTimer(self, evt):
        if self.vehicle_loader and self.vehicle_loader.is_alive():
            i, num_lines = self.vehicle_loader.get_progress()
            self.UpdateLoaderProgress('Loading 3D vehicle object...', i, num_lines)

    def OnTimer(self, evt):
        close_requested = False
        while self.conn.poll():
            msg = self.conn.recv()

            if msg['name'] == 'ui_is_active':
                self.ui_is_active = msg['value']
            elif msg['name'] == 'close':
                close_requested = True
            elif msg['name'] == 'running':
                if msg['value']:
                    self.SetState('running')
                else:
                    if self.state != 'countdown':
                        self.SetState('idle')
            elif msg['name'] == 'progress_update':
                self.UpdateProgress(msg['pct'], msg['sections'])
            elif msg['name'] == 'report':
                self.Report(msg['messages'], self.ui_is_active)
            elif msg['name'] == 'attitude':
                self.grid.SetAttitude(msg['roll'], msg['pitch'], msg['yaw'], msg['timestamp'])
            elif msg['name'] == 'mag':
                self.grid.SetMag(msg['x'], msg['y'], msg['z'])

        if close_requested:
            self.timer.Stop()
            self.Destroy()

    def OnCalibrationInstructionsTimer(self, evt):
        self.calibration_instruction_idx += 1
        self.calibration_instruction_idx %= len(self.calibration_instructions)
        text = self.calibration_instructions[self.calibration_instruction_idx]
        self.SetInstructionText(text)

    def UpdateProgress(self, pct, sections):
        self.progress.SetLabel("%d%%" % pct)
        self.progress.GetContainingSizer().Layout()
        if self.grid:
            self.grid.UpdateVisibleSections(sections)

    def SetButtonState(self, state):
        if state == self.button_state:
            return

        if state == 'start':
            self.button.SetLabel('START')
            self.button.SetForegroundColour(PRIMARY_HIGHLIGHT)
            self.button.SetBorderColor(PRIMARY_HIGHLIGHT)
        elif state == 'cancel':
            self.button.SetLabel('CANCEL')
            self.button.SetBorderColor(COMMON_FOREGROUND)
            self.button.SetForegroundColour(COMMON_FOREGROUND)

        self.button_state = state
        self.button.Refresh()

    def Report(self, mavlink_messages, popup):
        self.SetState('idle')
        for m in mavlink_messages:
            if m.cal_status != mavlink.MAG_CAL_SUCCESS:
                break
        else:
            self.UpdateProgress(100, [True] * 80)

        if popup:
            p = ReportDialog(self, mavlink_messages)
            r = p.ShowModal()
            if r == wx.ID_REDO:
                self.SetState('countdown')
