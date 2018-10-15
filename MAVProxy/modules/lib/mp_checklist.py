#!/usr/bin/env python

"""
  MAVProxy checklist, implemented in a child process
  Created by Stephen Dade (stephen_dade@hotmail.com)
"""

import sys
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib.wx_loader import wx

class CheckItem():
    '''Checklist item used for information transfer
    between threads/processes/pipes'''
    def __init__(self, name, state):
        self.name = name
        self.state = state


class CheckUI():
    '''
    a checklist UI for MAVProxy
    '''
    def __init__(self, title='MAVProxy: Checklist'):
        import threading
        self.title  = title
        self.menu_callback = None
        self.parent_pipe,self.child_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()
        from MAVProxy.modules.lib.wx_loader import wx

        app = wx.App(False)
        app.frame = ChecklistFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def close(self):
        '''close the UI'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def set_check(self, name, state):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(CheckItem(name, state))


class ChecklistFrame(wx.Frame):
    """ The main frame of the console"""

    def __init__(self, state, title):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(350,400), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)

        #use tabs for the individual checklists
        self.createLists()
        self.panel = wx.Panel(self)
        self.nb = wx.Choicebook(self.panel, wx.ID_ANY)

        #create the tabs
        self.createWidgets()

        #assign events to the buttons on the tabs
        self.createActions()

        #add in the pipe from MAVProxy
        self.timer = wx.Timer(self)
        #self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.Bind(wx.EVT_TIMER, lambda evt, notebook=self.nb: self.on_timer(evt, notebook), self.timer)
        self.timer.Start(100)

        # finally, put the notebook in a sizer for the panel to manage
        # the layout
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND)
        self.panel.SetSizer(sizer)

        self.Show(True)
        self.pending = []

    #Create the checklist items
    def createLists(self):
        '''Generate the checklists. Note that:
        0,1 = off/on for auto-ticked items
        2,3 = off/on for manually ticked items'''

        self.beforeAssemblyList = {
        'Confirm batteries charged':2,
        'No physical damage to airframe':2,
        'All electronics present and connected':2,
        'Bottle loaded':2,
        'Ground station operational':2
        }

        self.beforeEngineList = {
        'Avionics Power ON':2,
        'Pixhawk Booted':0,
        'Odroid Booted':2,
        'Cameras calibrated and capturing':2,
        'GPS lock':0,
        'Airspeed check':2,
        'Barometer check':2,
        'Compass check':2,
        'Flight mode MANUAL':0,
        'Avionics Power':0,
        'Servo Power':0,
        'IMU Check':0,
        'Aircraft Params Loaded':2,
        'Waypoints Loaded':0,
        'Servo and clevis check':2,
        'Geofence loaded':2,
        'Ignition circuit and battery check':2,
        'Check stabilisation in FBWA mode':2
        }

        self.beforeTakeoffList = {
        'Engine throttle responsive':2,
        'Runway clear':2,
        'Radio links > 6db margin':0,
        'Antenna tracker check':2,
        'GCS stable':2,
        }

        self.beforeCruiseList = {
        'Airspeed > 10 m/s':0,
        'Altitude > 30 m':0,
        '< 100 degrees to 1st Waypoint':2,
        'Airspeed and climb rate nominal':2
        }

        self.bottleDropList = {
        'Joe found':2,
        'Joe waypoint laid in':2,
        '< 100m to Joe waypoint':2,
        'Bottle drop mechanism activated':2
        }

        self.beforeLandingList = {
        'Runway clear':2,
        'APM set to FBWA mode':2,
        '< 100m from airfield home':2
        }

        self.beforeShutdownList = {
        'Taxi to parking':2,
        'Engine cutoff':2,
        'Data downloaded':2,
        'Ignition power off':2,
        'Avionics power off':2
        }

    # create controls on form - labels, buttons, etc
    def createWidgets(self):
        #create the panels for the tabs
        PanelAssembly = wx.Panel(self.nb)
        boxAssembly = wx.BoxSizer(wx.VERTICAL)
        PanelAssembly.SetAutoLayout(True)
        PanelAssembly.SetSizer(boxAssembly)
        PanelAssembly.Layout()

        PanelEngine = wx.Panel(self.nb)
        boxEngine = wx.BoxSizer(wx.VERTICAL)
        PanelEngine.SetAutoLayout(True)
        PanelEngine.SetSizer(boxEngine)
        PanelEngine.Layout()

        PanelTakeoff = wx.Panel(self.nb)
        boxTakeoff = wx.BoxSizer(wx.VERTICAL)
        PanelTakeoff.SetAutoLayout(True)
        PanelTakeoff.SetSizer(boxTakeoff)
        PanelTakeoff.Layout()

        PanelCruise = wx.Panel(self.nb)
        boxCruise = wx.BoxSizer(wx.VERTICAL)
        PanelCruise.SetAutoLayout(True)
        PanelCruise.SetSizer(boxCruise)
        PanelCruise.Layout()

        PanelDrop = wx.Panel(self.nb)
        boxDrop = wx.BoxSizer(wx.VERTICAL)
        PanelDrop.SetAutoLayout(True)
        PanelDrop.SetSizer(boxDrop)
        PanelDrop.Layout()

        PanelLanding = wx.Panel(self.nb)
        boxLanding = wx.BoxSizer(wx.VERTICAL)
        PanelLanding.SetAutoLayout(True)
        PanelLanding.SetSizer(boxLanding)
        PanelLanding.Layout()

        PanelShutdown = wx.Panel(self.nb)
        boxShutdown = wx.BoxSizer(wx.VERTICAL)
        PanelShutdown.SetAutoLayout(True)
        PanelShutdown.SetSizer(boxShutdown)
        PanelShutdown.Layout()

        #add the data to the individual tabs

        '''before assembly checklist'''
        for key in self.beforeAssemblyList:
            if self.beforeAssemblyList[key] == 0:
                disCheckBox = wx.CheckBox(PanelAssembly, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxAssembly.Add(disCheckBox)
            if self.beforeAssemblyList[key] == 2:
                boxAssembly.Add(wx.CheckBox(PanelAssembly, wx.ID_ANY, key))

        self.AssemblyButton = wx.Button(PanelAssembly, wx.ID_ANY, "Close final hatches")
        boxAssembly.Add(self.AssemblyButton)

        '''before Engine Start checklist'''
        for key in self.beforeEngineList:
            if self.beforeEngineList[key] == 0:
                disCheckBox = wx.CheckBox(PanelEngine, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxEngine.Add(disCheckBox)
            if self.beforeEngineList[key] == 2:
                boxEngine.Add(wx.CheckBox(PanelEngine, wx.ID_ANY, key))

        self.EngineButton = wx.Button(PanelEngine, wx.ID_ANY, "Ready for Engine start")
        boxEngine.Add(self.EngineButton)

        '''before takeoff checklist'''
        for key in self.beforeTakeoffList:
            if self.beforeTakeoffList[key] == 0:
                disCheckBox = wx.CheckBox(PanelTakeoff, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxTakeoff.Add(disCheckBox)
            if self.beforeTakeoffList[key] == 2:
                boxTakeoff.Add(wx.CheckBox(PanelTakeoff, wx.ID_ANY, key))

        self.TakeoffButton = wx.Button(PanelTakeoff, wx.ID_ANY, "Ready for Takeoff")
        boxTakeoff.Add(self.TakeoffButton)

        '''before cruise/AUTO checklist'''
        for key in self.beforeCruiseList:
            if self.beforeCruiseList[key] == 0:
                disCheckBox = wx.CheckBox(PanelCruise, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxCruise.Add(disCheckBox)
            if self.beforeCruiseList[key] == 2:
                boxCruise.Add(wx.CheckBox(PanelCruise, wx.ID_ANY, key))

        self.CruiseButton = wx.Button(PanelCruise, wx.ID_ANY, "Ready for Cruise/AUTO")
        boxCruise.Add(self.CruiseButton)

        '''before bottle drop checklist'''
        for key in self.bottleDropList:
            if self.bottleDropList[key] == 0:
                disCheckBox = wx.CheckBox(PanelDrop, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxDrop.Add(disCheckBox)
            if self.bottleDropList[key] == 2:
                boxDrop.Add(wx.CheckBox(PanelDrop, wx.ID_ANY, key))

        self.DropButton = wx.Button(PanelDrop, wx.ID_ANY, "Ready for Bottle Drop")
        boxDrop.Add(self.DropButton)

        '''before landing checklist'''
        for key in self.beforeLandingList:
            if self.beforeLandingList[key] == 0:
                disCheckBox = wx.CheckBox(PanelLanding, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxLanding.Add(disCheckBox)
            if self.beforeLandingList[key] == 2:
                boxLanding.Add(wx.CheckBox(PanelLanding, wx.ID_ANY, key))

        self.LandingButton = wx.Button(PanelLanding, wx.ID_ANY, "Ready for Landing")
        boxLanding.Add(self.LandingButton)

        '''before shutdown checklist'''
        for key in self.beforeShutdownList:
            if self.beforeShutdownList[key] == 0:
                disCheckBox = wx.CheckBox(PanelShutdown, wx.ID_ANY, key)
                disCheckBox.Enable(False)
                boxShutdown.Add(disCheckBox)
            if self.beforeShutdownList[key] == 2:
                boxShutdown.Add(wx.CheckBox(PanelShutdown, wx.ID_ANY, key))

        self.ShutdownButton = wx.Button(PanelShutdown, wx.ID_ANY, "Ready for Shutdown")
        boxShutdown.Add(self.ShutdownButton)

        #and add in the tabs
        self.nb.AddPage(PanelAssembly, "1. During Assembly")
        self.nb.AddPage(PanelEngine, "2. Before Engine Start")
        self.nb.AddPage(PanelTakeoff, "3. Before Takeoff")
        self.nb.AddPage(PanelCruise, "4. Before Cruise/AUTO")
        self.nb.AddPage(PanelDrop, "5. Before Bottle Drop")
        self.nb.AddPage(PanelLanding, "6. Before Landing")
        self.nb.AddPage(PanelShutdown, "7. Before Shutdown")

    #Create the actions for the buttons
    def createActions(self):
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.AssemblyButton)
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.EngineButton)
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.TakeoffButton)
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.CruiseButton)
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.DropButton)
        self.Bind(wx.EVT_BUTTON, self.on_Button, self.LandingButton)
        self.Bind(wx.EVT_BUTTON, self.on_ButtonLast, self.ShutdownButton)

    #do a final check of the current panel and move to the next
    def on_Button( self, event ):
        win = (event.GetEventObject()).GetParent()
        for widget in win.GetChildren():
            if type(widget) is wx.CheckBox and widget.IsChecked() == 0:
                dlg = wx.MessageDialog(win, "Not all items checked", "Error", wx.OK | wx.ICON_WARNING)
                dlg.ShowModal()
                dlg.Destroy()
                return
        #all checked, go to next panel.
        win.GetParent().AdvanceSelection()

    #Special implementation of the above function, but for the last tab
    def on_ButtonLast( self, event ):
        win = (event.GetEventObject()).GetParent()
        for widget in win.GetChildren():
            if type(widget) is wx.CheckBox and widget.IsChecked() == 0:
                dlg = wx.MessageDialog(win, "Not all items checked", "Error", wx.OK | wx.ICON_WARNING)
                dlg.ShowModal()
                dlg.Destroy()
                return
        #all checked, we're done.
        dlg = wx.MessageDialog(win, "Checklist Complete", "Done", wx.OK | wx.ICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()

    #Receive messages from MAVProxy and process them
    def on_timer(self, event, notebook):
        state = self.state
        win = notebook.GetPage(notebook.GetSelection())
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe.poll():
            obj = state.child_pipe.recv()
            if isinstance(obj, CheckItem):
                #go through each item in the current tab and (un)check as needed
                #print(obj.name + ", " + str(obj.state))
                for widget in win.GetChildren():
                    if type(widget) is wx.CheckBox and widget.GetLabel() == obj.name:
                        widget.SetValue(obj.state)


if __name__ == "__main__":
    # test the console
    import time

    checklist = CheckUI()

    #example auto-tick in second tab page
    while checklist.is_alive():
        checklist.set_check("Compass Calibrated", 1)
        time.sleep(0.5)
