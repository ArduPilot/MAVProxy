#!/usr/bin/env python

"""
  MAVProxy checklist, implemented in a child process
  Created by Stephen Dade (stephen_dade@hotmail.com)
"""

import sys
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib.wx_loader import wx
from collections import OrderedDict

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
    def __init__(self, title='MAVProxy: Checklist', checklist_file=None):
        import threading
        self.title  = title
        self.menu_callback = None
        self.checklist_file = checklist_file
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
        wx.Frame.__init__(self, None, title=title, size=(600,600), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)

        #use tabs for the individual checklists
        self.createLists()
        self.panel = wx.Panel(self)
        self.nb = wx.Choicebook(self.panel, wx.ID_ANY)

        #create the tabs
        self.createWidgets()

        #add in the pipe from MAVProxy
        self.timer = wx.Timer(self)
        #self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.Bind(wx.EVT_TIMER, lambda evt, notebook=self.nb: self.on_timer(evt, notebook), self.timer)
        self.timer.Start(100)

        # finally, put the notebook in a sizer for the panel to manage
        # the layout
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND|wx.ALL)
        self.panel.SetSizer(sizer)

        self.Show(True)

    def loadChecklist(self, fname):
        '''load checklist from a file'''
        self.lists = OrderedDict()

        lines = open(fname,'r').readlines()
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            a = line.split(':')
            if len(a) < 2:
                continue
            key = a[0].strip()
            text = a[1].strip()
            if not key in self.lists:
                self.lists[key] = []
            self.lists[key].append(text)

    #Create the checklist items
    def createLists(self):
        '''Generate the checklists'''

        if self.state.checklist_file is not None:
            self.loadChecklist(self.state.checklist_file)
            return

        self.lists = OrderedDict()
        self.lists['BeforeAssembly'] = [
            'Confirm batteries charged',
            'No physical damage to airframe',
            'All electronics present and connected',
            'Payload loaded',
            'Ground station operational'
            ]

        self.lists["Takeoff"] = [
            'Engine throttle responsive',
            'Runway clear',
            'Radio links OK',
            'Antenna tracker check',
            'GCS stable'
            ]

    # create controls on form - labels, buttons, etc
    def createWidgets(self):
        #create the panels for the tabs

        for name in self.lists.keys():
            panel = wx.Panel(self.nb)
            box = wx.BoxSizer(wx.VERTICAL)
            panel.SetAutoLayout(True)
            panel.SetSizer(box)

            for key in self.lists[name]:
                CheckBox = wx.CheckBox(panel, wx.ID_ANY, key)
                box.Add(CheckBox)

            panel.Layout()
            self.nb.AddPage(panel, name)

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
        checklist.set_check("Radio links OK", 1)
        time.sleep(0.5)
