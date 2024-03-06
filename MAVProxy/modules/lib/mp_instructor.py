#!/usr/bin/env python

"""
  MAVProxy instructor station, UI runs in a child process
  André Kjellstrup @ NORCE
"""

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib.wx_loader import wx


class CheckboxItemState:
    def __init__(self, name, state):
        self.name = name
        self.state = state


class InstructorUI:
    """
    Instructor UI for MAVProxy
    """
    def __init__(self, title='Instructor Station'):
        self.title = title
        self.menu_callback = None
        self.pipe_to_gui, self.gui_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        """child process - this holds all the GUI elements"""
        mp_util.child_close_fds()
        from MAVProxy.modules.lib.wx_loader import wx

        app = wx.App(False)
        app.frame = InstructorFrame(self.gui_pipe, state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def close(self):
        """close the UI"""
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)
            print("closed")

    def is_alive(self):
        """check if child is still going"""
        return self.child.is_alive()

    def set_check(self, name, state):
        """set a status value"""
        if self.child.is_alive():
            self.pipe_to_gui.send(CheckboxItemState(name, state))


class InstructorFrame(wx.Frame):
    """ The main frame of the console"""

    def __init__(self, gui_pipe,  state, title):
        self.state = state
        self.gui_pipe = gui_pipe
        wx.Frame.__init__(self, None, title=title, size=(500, 650), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)


        self.panel = wx.Panel(self)
        self.nb = wx.Choicebook(self.panel, wx.ID_ANY)

        # create the tabs
        self.createWidgets()

        # assign events to the buttons on the tabs
        self.createActions()

        # add in the pipe from MAVProxy
        self.timer = wx.Timer(self)
        # self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        # self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.Bind(wx.EVT_TIMER, lambda evt, notebook=self.nb: self.on_timer(evt, notebook), self.timer)
        self.timer.Start(100)

        # finally, put the notebook in a sizer for the panel to manage
        # the layout
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND)
        self.panel.SetSizer(sizer)

        self.Show(True)
        self.pending = []



    # create controls on form - labels, buttons, etc
    def createWidgets(self):

        # create the panels for the tabs

        PanelCommon = wx.Panel(self.nb)
        boxCommon = wx.BoxSizer(wx.VERTICAL)
        PanelCommon.SetAutoLayout(True)
        PanelCommon.SetSizer(boxCommon)
        PanelCommon.Layout()


        PanelPlane = wx.Panel(self.nb)
        boxPlane = wx.BoxSizer(wx.VERTICAL)
        PanelPlane.SetAutoLayout(True)
        PanelPlane.SetSizer(boxPlane)
        PanelPlane.Layout()

        PanelCopter = wx.Panel(self.nb)
        boxCopter = wx.BoxSizer(wx.VERTICAL)
        PanelCopter.SetAutoLayout(True)
        PanelCopter.SetSizer(boxCopter)
        PanelCopter.Layout()


        # add the data to the individual tabs

        'Common failures'

        self.Dis_GNSS_Fix_Checkbox = wx.CheckBox(PanelCommon, wx.ID_ANY, "Disable GNSS FIX")
        boxCommon.Add(self.Dis_GNSS_Fix_Checkbox)

        boxCommon.Add(wx.StaticLine(PanelCommon, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)
        boxCommon.Add(wx.StaticText(PanelCommon, wx.ID_ANY, 'Voltage drop rate mv/s:', wx.DefaultPosition, wx.DefaultSize, style=wx.ALIGN_RIGHT))

        self.VoltageSlider = wx.Slider(PanelCommon, wx.ID_ANY, 100, 10, 200, wx.DefaultPosition, (250, -1),
                                       wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxCommon.Add(self.VoltageSlider)

        self.Voltage_Drop_Checkbox = wx.CheckBox(PanelCommon, wx.ID_ANY, "Voltage dropping")
        boxCommon.Add(self.Voltage_Drop_Checkbox)

        boxCommon.Add(wx.StaticLine(PanelCommon, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)

        self.FailsafeButton = wx.Button(PanelCommon, wx.ID_ANY, "Trigger FailSafe")
        boxCommon.Add(self.FailsafeButton)

        boxCommon.Add(wx.StaticLine(PanelCommon, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)

        self.GCS_Comm_Loss_Checkbox = wx.CheckBox(PanelCommon, wx.ID_ANY, "GCS Comm loss")
        # disCheckbox.Enable(False)
        boxCommon.Add(self.GCS_Comm_Loss_Checkbox)

        boxCommon.Add(wx.StaticLine(PanelCommon, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)
        boxCommon.Add(wx.StaticText(PanelCommon, wx.ID_ANY, 'Wind Direction:', wx.DefaultPosition, wx.DefaultSize,style=wx.ALIGN_RIGHT))

        self.Wind_Dir_Slider = wx.Slider(PanelCommon, wx.ID_ANY, 0, 0, 359, wx.DefaultPosition, (250, -1),
                                         wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxCommon.Add(self.Wind_Dir_Slider)

        boxCommon.Add(wx.StaticText(PanelCommon, wx.ID_ANY, 'Wind Velocity (m/s):', wx.DefaultPosition, wx.DefaultSize,style=wx.ALIGN_RIGHT))
        self.Wind_Vel_Slider = wx.Slider(PanelCommon, wx.ID_ANY, 0, 0, 50, wx.DefaultPosition, (250, -1),
                                       wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxCommon.Add(self.Wind_Vel_Slider)

        boxCommon.Add(wx.StaticText(PanelCommon, wx.ID_ANY, 'Turbulence:', wx.DefaultPosition, wx.DefaultSize,
                                    style=wx.ALIGN_RIGHT))
        self.Wind_Turb_Slider = wx.Slider(PanelCommon, wx.ID_ANY, 0, 0, 10, wx.DefaultPosition, (250, -1),
                                         wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxCommon.Add(self.Wind_Turb_Slider)


        'Plane specific failures'

        self.Pitot_Fail_Low_Checkbox = wx.CheckBox(PanelPlane, wx.ID_ANY, "Pitot stuck at no speed")
        boxPlane.Add(self.Pitot_Fail_Low_Checkbox)

        self.Pitot_Fail_High_Checkbox = wx.CheckBox(PanelPlane, wx.ID_ANY, "Pitot stuck at high speed")
        boxPlane.Add(self.Pitot_Fail_High_Checkbox)

        self.Arspd_Offset_Slider = wx.Slider(PanelPlane, wx.ID_ANY, 2013, 1500, 2500, wx.DefaultPosition, (250, -1),
                                       wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxPlane.Add(self.Arspd_Offset_Slider)

        boxPlane.Add(wx.StaticLine(PanelPlane, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)

        boxPlane.Add(wx.StaticText(PanelPlane, wx.ID_ANY, 'Thrust reduction:', wx.DefaultPosition, wx.DefaultSize,
                                    style=wx.ALIGN_RIGHT))
        self.Plane_Thrust_Slider = wx.Slider(PanelPlane, wx.ID_ANY, 0, 0, 1000, wx.DefaultPosition, (250, -1),
                                             wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxPlane.Add(self.Plane_Thrust_Slider)

        boxPlane.Add(wx.StaticText(PanelPlane, wx.ID_ANY, 'Thrust reduction + current increase:', wx.DefaultPosition, wx.DefaultSize,
                                   style=wx.ALIGN_RIGHT))

        self.Plane_Thrust_Curr_Slider = wx.Slider(PanelPlane, wx.ID_ANY, 0, 0, 1000, wx.DefaultPosition, (250, -1),
                                             wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxPlane.Add(self.Plane_Thrust_Curr_Slider)

        'Copter specific failures'

        boxCopter.Add(wx.StaticText(PanelCopter, wx.ID_ANY, 'Thrust reduction:', wx.DefaultPosition, wx.DefaultSize, style=wx.ALIGN_RIGHT))

        self.Copter_Thrust_Slider = wx.Slider(PanelCopter, wx.ID_ANY, 0, 0, 400, wx.DefaultPosition, (250, -1),
                                             wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        boxCopter.Add(self.Copter_Thrust_Slider)

        boxCopter.Add(wx.StaticLine(PanelCopter, -1), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 10)

        self.CopterResetButton = wx.Button(PanelCopter, wx.ID_ANY, "Reset Parameters")
        boxCopter.Add(self.CopterResetButton)

        # and add in the tabs
        self.nb.AddPage(PanelCommon, "Common (Copter/Plane) Failures")
        self.nb.AddPage(PanelPlane, "Plane specific Failures")
        self.nb.AddPage(PanelCopter, "Copter specific Failures")




    # Create the actions for the buttons
    def createActions(self):
        # Common tab:
        self.Bind(wx.EVT_CHECKBOX, self.dis_gnss, self.Dis_GNSS_Fix_Checkbox)
        #self.Bind(wx.EVT_CHECKBOX, self.sendevent("Voltage", 20), self.Voltage_Drop_Checkbox)   #  WHY  trigged once?
        self.Bind(wx.EVT_CHECKBOX, self.volt_drop, self.Voltage_Drop_Checkbox)
        self.Bind(wx.EVT_CHECKBOX, self.gcs_comm_loss, self.GCS_Comm_Loss_Checkbox)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.volt_drop_rate, self.VoltageSlider)
        self.Bind(wx.EVT_BUTTON, self.setmode, self.FailsafeButton)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.wind_dir, self.Wind_Dir_Slider)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.wind_vel, self.Wind_Vel_Slider)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.wind_turbulence, self.Wind_Turb_Slider)
        # Plane tab:
        self.Bind(wx.EVT_CHECKBOX, self.pitot_fail_low, self.Pitot_Fail_Low_Checkbox)
        self.Bind(wx.EVT_CHECKBOX, self.pitot_fail_high, self.Pitot_Fail_High_Checkbox)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.arspd_offset, self.Arspd_Offset_Slider)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.plane_thrust_loss, self.Plane_Thrust_Slider)
        self.Bind(wx.EVT_SCROLL_CHANGED, self.plane_thrust_loss_curr, self.Plane_Thrust_Curr_Slider)
        # Copter tab:
        self.Bind(wx.EVT_SCROLL_CHANGED, self.copter_thrust_loss, self.Copter_Thrust_Slider)
        self.Bind(wx.EVT_BUTTON, self.copter_reset, self.CopterResetButton)

    # Common actions
    def dis_gnss(self, event):
        self.gui_pipe.send(["dis_gnss", self.Dis_GNSS_Fix_Checkbox.Value])

    def volt_drop_rate(self, event):
        self.gui_pipe.send(["volt_drop_rate", self.VoltageSlider.Value / 1000])

    def volt_drop(self, event):
        self.gui_pipe.send(["volt_drop", self.Voltage_Drop_Checkbox.Value])

    def gcs_comm_loss(self, event):
        self.gui_pipe.send(["gcs_comm_loss", self.GCS_Comm_Loss_Checkbox.Value])

    def setmode(self, event):
        self.gui_pipe.send(["setmode", 10])
    def on_gnss_Button_ok(self, event):
        self.gui_pipe.send("ok")

    def wind_dir(self, event):
        self.gui_pipe.send(["wind_dir", self.Wind_Dir_Slider.Value])

    def wind_vel(self, event):
        self.gui_pipe.send(["wind_vel", self.Wind_Vel_Slider.Value])


    def wind_turbulence(self, event):
        self.gui_pipe.send(["wind_turbulence", self.Wind_Turb_Slider.Value])

    # Plane actions

    def pitot_fail_low(self, event):
        self.gui_pipe.send(["pitot_fail_low", self.Pitot_Fail_Low_Checkbox.Value])

    def pitot_fail_high(self, event):
        self.gui_pipe.send(["pitot_fail_high", self.Pitot_Fail_High_Checkbox.Value])

    def arspd_offset(self, event):
        self.gui_pipe.send(["arspd_offset", self.Arspd_Offset_Slider.Value])

    def plane_thrust_loss(self, event):
        self.gui_pipe.send(["plane_thrust_loss", self.Plane_Thrust_Slider.Value])

    def plane_thrust_loss_curr(self, event):
        self.gui_pipe.send(["plane_thrust_loss_curr", self.Plane_Thrust_Curr_Slider.Value])

        # Copter actions

    def copter_thrust_loss(self, event):
        self.gui_pipe.send(["copter_thrust_loss", self.Copter_Thrust_Slider.Value])

    def copter_reset (self, event):
        self.gui_pipe.send(["copter_reset", 0])

    def sendevent(self, event, text, value=0):
        self.gui_pipe.send([text, value])




    #do a final check of the current panel and move to the next
    def on_Button( self, event):
        win = (event.GetEventObject()).GetParent()
        for widget in win.GetChildren():
            if type(widget) is wx.CheckBox and widget.IsChecked() == 0:
                dlg = wx.MessageDialog(win, "Not all items checked", "Error", wx.OK | wx.ICON_WARNING)
                dlg.ShowModal()
                dlg.Destroy()
                return
        # all checked, go to next panel.
        win.GetParent().AdvanceSelection()


    # Receive messages from MAVProxy module and process them
    def on_timer(self, event, notebook):
        state = self.state
        win = notebook.GetPage(notebook.GetSelection())
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.gui_pipe.poll():
            obj = state.gui_pipe.recv()
            if isinstance(obj, CheckboxItemState):
                # go through each item in the current tab and (un)check as needed
                # print(obj.name + ", " + str(obj.state))
                for widget in win.GetChildren():
                    if type(widget) is wx.CheckBox and widget.GetLabel() == obj.name:
                        widget.SetValue(obj.state)


if __name__ == "__main__":
    # test the console
    import time

    instructor = InstructorUI()

    # example auto-tick in second tab page
    while instructor.is_alive():
        time.sleep(0.5)
