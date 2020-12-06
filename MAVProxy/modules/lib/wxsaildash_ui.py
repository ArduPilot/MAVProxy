"""
  MAVProxy sailing dashboard gui elements
"""

from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib.wxsaildash_util import WindReference, SpeedUnit, WindAngleAndSpeed, WaterSpeedAndHeading

import wx.lib.agw.speedmeter as SM

import math
import time

class SailingDashboardFrame(wx.Frame):
    '''The main frame of the sailing dashboard'''

    def __init__(self, state, title, size):
        super(SailingDashboardFrame, self).__init__(None, title=title, size=size)
        self._state = state
        self._title = title

        # control update rate
        self._timer = wx.Timer(self)
        self._fps = 10.0
        self._start_time = time.time()

        # events
        self.Bind(wx.EVT_TIMER, self.OnTimer, self._timer)
        self.Bind(wx.EVT_IDLE, self.OnIdle)
        self.Bind(wx.EVT_CHAR_HOOK, self.OnKeyPress)

        # restart the timer
        self._timer.Start(milliseconds=100)

        # create wind meters for true and apparent wind
        self._wind_meter1 = WindMeter(self)
        self._wind_meter1.SetWindReference(WindReference.RELATIVE)
        self._wind_meter1.SetWindSpeedUnit(SpeedUnit.KNOTS)
        self._wind_meter1.SetWindAngle(0)
        self._wind_meter1.SetWindSpeed(0)

        self._wind_meter2 = WindMeter(self)
        self._wind_meter2.SetWindReference(WindReference.TRUE)
        self._wind_meter2.SetWindSpeedUnit(SpeedUnit.KNOTS)
        self._wind_meter2.SetWindAngle(0)
        self._wind_meter2.SetWindSpeed(0)

        # instrument display
        self._instr1 = InstrumentDisplay(self)
        self._instr1.label = "AWS"
        self._instr1.unit = "kn"
        self._instr1.value = 0.0

        self._instr2 = InstrumentDisplay(self)
        self._instr2.label = "TWS"
        self._instr2.unit = "kn"
        self._instr2.value = 0.0

        self._instr3 = InstrumentDisplay(self)
        self._instr3.label = "STW"
        self._instr3.unit = "kn"
        self._instr3.value = 0.0

        self._instr4 = InstrumentDisplay(self)
        self._instr4.label = "AWA"
        self._instr4.unit = "deg"
        self._instr4.value = 0.0

        self._instr5 = InstrumentDisplay(self)
        self._instr5.label = "TWA"
        self._instr5.unit = "deg"
        self._instr5.value = 0.0

        self._instr6 = InstrumentDisplay(self)
        self._instr6.label = "HDT"
        self._instr6.unit = "deg"
        self._instr3.value = 0.0

        # instrument panel sizers
        self._instr_sizer1 = wx.BoxSizer(wx.VERTICAL) 
        self._instr_sizer1.Add(self._instr1, 1, wx.EXPAND)
        self._instr_sizer1.Add(self._instr2, 1, wx.EXPAND)
        self._instr_sizer1.Add(self._instr3, 1, wx.EXPAND)

        self._instr_sizer2 = wx.BoxSizer(wx.VERTICAL) 
        self._instr_sizer2.Add(self._instr4, 1, wx.EXPAND)
        self._instr_sizer2.Add(self._instr5, 1, wx.EXPAND)
        self._instr_sizer2.Add(self._instr6, 1, wx.EXPAND)

        # top level sizers
        self._top_sizer = wx.BoxSizer(wx.HORIZONTAL) 
        self._top_sizer.Add(self._wind_meter1, 2, wx.EXPAND)
        self._top_sizer.Add(self._wind_meter2, 2, wx.EXPAND)        
        self._top_sizer.Add(self._instr_sizer1, 1, wx.EXPAND)
        self._top_sizer.Add(self._instr_sizer2, 1, wx.EXPAND)
        
        # layout sizers
        self.SetSizer(self._top_sizer)
        self.SetAutoLayout(1)
        # self.sizer.Fit(self)

    def OnIdle(self, event):
        '''Handle idle events
        
            - e.g. managed scaling when window resized if needed
        ''' 
        
        pass

    def OnTimer(self, event):
        '''Handle timed tasks'''
        
        # check for close events
        if self._state.close_event.wait(timeout=0.001):
            # stop the timer and destroy the window - this ensures
            # the window is closed whhen the module is unloaded
            self._timer.Stop()
            self.Destroy()
            return

        # receive data from the module
        while self._state.child_pipe_recv.poll():
            obj_list = self._state.child_pipe_recv.recv()
            for obj in obj_list:
                if isinstance(obj, WindAngleAndSpeed):
                    if obj.wind_reference == WindReference.RELATIVE:
                        # apparent wind meter
                        self._wind_meter1.SetWindReference(obj.wind_reference)
                        self._wind_meter1.SetWindAngle(obj.wind_angle)
                        self._wind_meter1.SetWindSpeed(obj.wind_speed)
                        self._wind_meter1.SetWindSpeedUnit(obj.wind_speed_unit)

                        # apparent wind displays    
                        self._instr1.value = obj.wind_speed
                        self._instr4.value = obj.wind_angle

                    elif obj.wind_reference == WindReference.TRUE:
                        # apparent wind meter
                        self._wind_meter2.SetWindReference(obj.wind_reference)
                        self._wind_meter2.SetWindAngle(obj.wind_angle)
                        self._wind_meter2.SetWindSpeed(obj.wind_speed)
                        self._wind_meter2.SetWindSpeedUnit(obj.wind_speed_unit)

                        # apparent wind displays    
                        self._instr2.value = obj.wind_speed
                        self._instr5.value = obj.wind_angle

                elif isinstance(obj, WaterSpeedAndHeading):
                    # water speed and heading     
                    self._instr3.value = obj.water_speed
                    self._instr6.value = obj.heading_true


        # reset counters
        self._start_time = time.time()

    def OnKeyPress(self, event):
        '''Handle keypress events'''

        pass

class WindMeter(SM.SpeedMeter):
    ''' 
    class: `WindMeter` is a custom `SpeedMeter`for displaying true or apparent wind 
    '''

    def __init__(self, *args, **kwargs):
        # customise the agw style
        kwargs.setdefault('agwStyle', SM.SM_DRAW_HAND
            |SM.SM_DRAW_PARTIAL_SECTORS
            |SM.SM_DRAW_SECONDARY_TICKS
            |SM.SM_DRAW_MIDDLE_TEXT)

        # initialise super class
        super(WindMeter, self).__init__(*args, **kwargs)

        # non-public attributes 
        self._wind_reference = WindReference.RELATIVE
        self._wind_speed = 0
        self._wind_speed_unit = SpeedUnit.KNOTS

        # colours
        dial_colour = wx.ColourDatabase().Find('DARK SLATE GREY')
        port_arc_colour = wx.ColourDatabase().Find('RED')
        stbd_arc_colour = wx.ColourDatabase().Find('GREEN')
        bottom_arc_colour = wx.ColourDatabase().Find('TAN')
        self.SetSpeedBackground(dial_colour)

        # set lower limit to 2.999/2 to prevent the partial sectors from filling
        # the entire dial
        self.SetAngleRange(- 2.995/2 * math.pi, math.pi/2)

        # create the intervals
        intervals = range(0, 361, 20)
        self.SetIntervals(intervals)

        # assign colours to sectors
        colours = [dial_colour] \
            + [stbd_arc_colour] * 2 \
            + [dial_colour] * 4 \
            + [bottom_arc_colour] * 4 \
            + [dial_colour] * 4 \
            + [port_arc_colour] * 2 \
            + [dial_colour]
        self.SetIntervalColours(colours)

        # assign the ticks, colours and font
        ticks = ['0', '20', '40', '60', '80', '100', '120', '140', '160', '180', '-160', '-140', '-120', '-100', '-80', '-60', '-40', '-20', '']
        self.SetTicks(ticks)
        self.SetTicksColour(wx.WHITE)
        self.SetNumberOfSecondaryTicks(1)
        self.SetTicksFont(wx.Font(12, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))

        # set the colour for the first hand indicator
        self.SetHandColour(wx.Colour(210, 210, 210))
        self.SetHandStyle("Hand")

        # do not draw the external (container) arc
        self.DrawExternalArc(False)

        # initialise the meter to zero
        self.SetWindAngle(0.0)

        # wind speed display
        self.SetMiddleTextColour(wx.WHITE)
        self.SetMiddleTextFont(wx.Font(16, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.SetWindSpeed(0.0)

    def SetWindAngle(self, wind_angle):
        '''Set the wind angle

            The wind angle is measured with respect to the front of the vehicle, so: 
                -180 <= wind_angle <= 180
        '''
        
        # Convert the wind angle to the meter interval [0 - 360] 
        dial_value = wind_angle 
        if wind_angle < 0:
            dial_value += 360

        self.SetSpeedValue(dial_value)

    def SetWindSpeed(self, wind_speed):
        '''Set the wind speed'''

        self._wind_speed = wind_speed
        
        reference_code = None
        if self._wind_reference == WindReference.RELATIVE:
            reference_code ='AWS'
        elif self._wind_reference == WindReference.TRUE:
            reference_code ='TWS'
        else:
            reference_code ='INVALID'

        unit_code = None
        if self._wind_speed_unit == SpeedUnit.KPH:
            unit_code ='kph'
        elif self._wind_speed_unit == SpeedUnit.MPH:
            unit_code ='mph'
        elif self._wind_speed_unit == SpeedUnit.KNOTS:
            unit_code ='kn'
        else:
            unit_code ='INVALID'

        txt = reference_code + "  {:.2f}".format(self._wind_speed) + ' ' + unit_code
        self.SetMiddleText(txt)

    def SetWindReference(self, reference):
        '''Set the wind reference (i.e relative(apparent) or true)'''

        if not (reference == WindReference.RELATIVE or reference == WindReference.TRUE):
            raise Exception("Invalid wind reference. Must be 'RELATIVE' or 'TRUE'")
            
        self._wind_reference = reference

    def SetWindSpeedUnit(self, unit):
        '''Set the wind speed units'''
        
        if not (unit == SpeedUnit.KPH or unit == SpeedUnit.MPH or unit == SpeedUnit.KNOTS):
            raise Exception("Invalid wind speed unit. Must be 'KPH' or 'MPH' or 'KNOTS'")
            
        self._wind_speed_unit = unit

class InstrumentDisplay(wx.Panel):
    '''
    class: `InstrumentDisplay` is a panel for displaying sailing instrument data
    '''

    def __init__(self, *args, **kwargs):
        # customise the style
        kwargs.setdefault('style', wx.TE_READONLY)

        # initialise super class
        super(InstrumentDisplay, self).__init__(*args, **kwargs)

        # set the background colour (also for text controls)
        # self.SetBackgroundColour(wx.WHITE)

        # text controls
        self._label_text = wx.TextCtrl(self, style=wx.BORDER_NONE)
        self._label_text.SetFont(wx.Font(12, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self._label_text.SetBackgroundColour(self.GetBackgroundColour())
        self._label_text.ChangeValue("---")

        self._value_text = wx.TextCtrl(self, style=wx.BORDER_NONE|wx.TE_CENTRE)
        self._value_text.SetFont(wx.Font(30, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self._value_text.SetBackgroundColour(self.GetBackgroundColour())
        self._value_text.ChangeValue("-.-")
        self._value_text.SetMinSize((60, 40))

        self._unit_text = wx.TextCtrl(self, style=wx.BORDER_NONE|wx.TE_RIGHT)
        self._unit_text.SetFont(wx.Font(12, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self._unit_text.SetBackgroundColour(self.GetBackgroundColour())
        self._unit_text.ChangeValue("--")

        # value text needs a nested sizer to centre vertically
        self._value_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._value_sizer.Add(self._value_text,
            wx.SizerFlags(1).Align(wx.ALIGN_CENTRE_VERTICAL).Border(wx.ALL, 0))

        # top level sizers
        self._top_sizer = wx.BoxSizer(wx.VERTICAL) 
        self._top_sizer.Add(self._label_text,
            wx.SizerFlags(0).Align(wx.ALIGN_TOP|wx.ALIGN_LEFT).Border(wx.ALL, 2))
        self._top_sizer.Add(self._value_sizer,
            wx.SizerFlags(1).Expand().Border(wx.ALL, 1))
        self._top_sizer.Add(self._unit_text,
            wx.SizerFlags(0).Align(wx.ALIGN_RIGHT).Border(wx.ALL, 2))

        # layout sizers
        self._top_sizer.SetSizeHints(self)
        self.SetSizer(self._top_sizer)
        self.SetAutoLayout(True)

    @property
    def label(self):
        return self._label_text.GetValue()

    @label.setter
    def label(self, label):
        self._label_text.ChangeValue(label)

    @property
    def value(self):
        return self._value_text.GetValue()

    @value.setter
    def value(self, value):
        self._value_text.ChangeValue("{:.2f}".format(value))

    @property
    def unit(self):
        return self._unit_text.GetValue()

    @unit.setter
    def unit(self, unit):
        self._unit_text.ChangeValue(unit)
