#!/usr/bin/python
'''
A User interface for MAVProxy modules. It takes in a list of settings, displays
them and pipes back any changed settings to the module.

Copyright Stephen Dade 2013
Released under the GNU GPL version 3 or later
'''

import os
import sys

import wx
import wx.lib.agw.persist as PM
import wx.lib.agw.floatspin as FS

from multiprocessing import Process, Queue, Pipe
from threading import *
import Queue
import time
 
#need the mainthread to be a global var, so
#it can be closed from it's own child (the UI)
mainThread = None
#myQueue = Queue.Queue()
 
# Simply an extension on the Thread class
# Holds the UI and pipes the setting values in and out
class LiveSettings(Thread):
    """Worker Thread Class."""
    def __init__(self, dict, title):
        """Init Worker Thread Class."""
        Thread.__init__(self)
        self._dict = dict
        self._title = title
        self.MsgQ = Queue.Queue()
        # This starts the thread running on creation
        self.start()

    #Start the UI
    def run(self):
        self.app = MainApp(0)
        self.app.frame.SetTitle(self._title)
        self.app.frame.SetSizeHints(minW = 300, minH = 150)

		#autogen the controls
        for key, value in self._dict.iteritems():
            #it will be a frame with the label and control
            CtrlSizer = wx.BoxSizer(wx.HORIZONTAL)
            CtrlPanel = wx.Panel(self.app.frame.ContPanel)
            CtrlSizer.AddSpacer(10)

            bb = wx.StaticText(CtrlPanel, label=key)
            CtrlSizer.Add(bb, 0, wx.ALL)
            CtrlSizer.AddSpacer(10)
            if type(value) is int:
                cc = wx.SpinCtrl(CtrlPanel)
                cc.SetValue(value) 
                cc.Bind(wx.EVT_SPINCTRL, self.OnSpin)
                cc.SetName(key) 
            elif type(value) is long or type(value) is float:
                cc = FS.FloatSpin(CtrlPanel)
                cc.SetDigits(4)
                cc.SetValue(value)
                cc.Bind(FS.EVT_FLOATSPIN, self.OnFloatSpin)
                cc.SetName(key) 
            elif type(value) is str:
                cc = wx.TextCtrl(CtrlPanel)
                cc.SetValue(value)
                cc.Bind(wx.EVT_TEXT, self.OnTextUpdate)
                cc.SetName(key) 

            CtrlSizer.Add(cc, 0, wx.ALL)
            CtrlSizer.AddSpacer(10)
			
            CtrlPanel.SetSizer( CtrlSizer )
            self.app.frame.ContSizer.Add( CtrlPanel, 0, wx.ALL )
            self.app.frame.ContSizer.AddSpacer(10) 

        self.app.frame.ContPanel.SetSizer( self.app.frame.ContSizer )
		
		#fit all the subpanels to the main panel
        self.app.frame.SetSizer( self.app.frame.ContSizer )
        self.app.frame.ContSizer.Fit( self.app.frame )
		
        self.app.MainLoop()

    #Event bindings, shared by the controls, ordered by type
    def OnTextUpdate(self,event):
        cc = event.GetEventObject()
        #print cc.GetName() + " was changed to " + cc.GetValue()
        self.MsgQ.put({cc.GetName(): cc.GetValue()})

    def OnFloatSpin(self,event):
        cc = event.GetEventObject()
        #print cc.GetName() + " was changed to " + str(cc.GetValue())
        self.MsgQ.put({cc.GetName(): cc.GetValue()})

    def OnSpin(self,event):
        cc = event.GetEventObject()
        #print cc.GetName() + " was changed to " + str(cc.GetValue())
        self.MsgQ.put({cc.GetName(): cc.GetValue()})

    def abort(self):
        """abort worker thread."""
        # Method for use by main thread to signal an abort
        self.MsgQ.put(0)
        self.app.Exit()
        print "Subthread exit\n"


# GUI Frame class that contains the settings
class MainFrame(wx.Frame):
    """Class MainFrame."""
    def __init__(self, parent, id):
        """Create the MainFrame."""
        wx.Frame.__init__(self, parent, id, 'Thread Test', style=wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN)
		
        self.ContSizer = wx.BoxSizer(wx.VERTICAL)
        self.ContPanel = wx.Panel(self)
        self.ContSizer.AddSpacer(10) 
		
        self.__close_callback = None
        self.Bind(wx.EVT_CLOSE, self._when_closed)

    def register_close_callback(self, callback):
        self.__close_callback = callback

    #We intercept the close event from the UI so we can kill
    #off the master thread (parent)
    def _when_closed(self, event):
        global mainThread
        doClose = True if not self.__close_callback else self.__close_callback()
        if doClose:
            print "UI Closed"
            mainThread.abort()
            event.Skip()

#Standard WX App. Called from within LiveSettings
class MainApp(wx.App):
    """Class Main App."""
    def OnInit(self):
        """Init Main App."""
        self.frame = MainFrame(None, -1)
        self.frame.Show(True)
        self.frame.register_close_callback(lambda: True)
        return True


if __name__ == '__main__':
    print "LiveSettings Test"
	#don't need to declare this at module scope
    #global mainThread

    #make some vars to play with
    numInt = 1
    numFloat = 2.5
    numString = "Test"
    dict = {'numString': numString, 'numFloat': numFloat, 'numInt': numInt}
    
    mainThread = LiveSettings(dict, "LiveSettings Test")
    print "Subproccess passed"
	
    # wait for the thread to finish
    while True:
        e = mainThread.MsgQ.get()
        #if the queue sends a 0, it's a signal to exit
        if e == 0:
            break
        for key, value in e.iteritems():
            print key + " is now " + str(value)


    print "Main end"
	