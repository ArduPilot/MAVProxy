#!/usr/bin/env python3
'''
GUI for adding new links to MAVProxy
'''

import wx
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
import MAVProxy.modules.mavproxy_link

class linkAddDialog(wx.Dialog):
    def __init__(self, *args, **kwds):
        super(linkAddDialog, self).__init__(*args, **kwds)
    
        self.panelGUI = wx.Panel(self, wx.ID_ANY)

        self.sizerGUI = wx.FlexGridSizer(5, 2, 0, 0)
        
        self.addLink = None
        
        self.conStr = None

        label_1 = wx.StaticText(self.panelGUI, wx.ID_ANY, "Connection Type:")
        self.sizerGUI.Add(label_1, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL | wx.RESERVE_SPACE_EVEN_IF_HIDDEN, 3)

        self.choiceConnection = wx.Choice(self.panelGUI, wx.ID_ANY, choices=["udpin", "udpout", "tcpin", "tcp", "Serial"])
        self.choiceConnection.SetSelection(0)
        self.sizerGUI.Add(self.choiceConnection, 0, wx.ALL, 3)

        self.labelConType = wx.StaticText(self.panelGUI, wx.ID_ANY, "IP:Port")
        self.sizerGUI.Add(self.labelConType, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 3)

        self.textConIPPort = wx.TextCtrl(self.panelGUI, wx.ID_ANY, "127.0.0.1:14550")
        self.textConIPPort.SetMinSize((150, 34))
        self.sizerGUI.Add(self.textConIPPort, 0, wx.ALL | wx.EXPAND, 3)

        self.labelSerialPort = wx.StaticText(self.panelGUI, wx.ID_ANY, "Port:")
        self.sizerGUI.Add(self.labelSerialPort, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL | wx.RESERVE_SPACE_EVEN_IF_HIDDEN, 3)

        self.choiceSerialPort = wx.Choice(self.panelGUI, wx.ID_ANY, choices=[])
        
        self.sizerGUI.Add(self.choiceSerialPort, 0, wx.ALL | wx.RESERVE_SPACE_EVEN_IF_HIDDEN, 3)

        self.labelBaud = wx.StaticText(self.panelGUI, wx.ID_ANY, "Baud Rate:")
        self.sizerGUI.Add(self.labelBaud, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL | wx.RESERVE_SPACE_EVEN_IF_HIDDEN, 3)

        self.choiceBaud = wx.Choice(self.panelGUI, wx.ID_ANY, choices=["9600", "19200", "38400", "57600", "115200", "921600", "1500000"])
        self.choiceBaud.SetSelection(3)
        self.sizerGUI.Add(self.choiceBaud, 0, wx.ALL | wx.RESERVE_SPACE_EVEN_IF_HIDDEN, 3)

        self.buttonAdd = wx.Button(self.panelGUI, wx.ID_ADD, "Add Link")
        self.sizerGUI.Add(self.buttonAdd, 0, wx.ALL, 3)

        self.buttonExit = wx.Button(self.panelGUI, wx.ID_CANCEL, "")
        self.sizerGUI.Add(self.buttonExit, 0, wx.ALIGN_CENTER_VERTICAL, 0)

        self.panelGUI.SetSizer(self.sizerGUI)

        self.Layout()

        self.Bind(wx.EVT_CHOICE, self.onChangeType, self.choiceConnection)
        self.Bind(wx.EVT_BUTTON, self.onAdd, self.buttonAdd)
        self.Bind(wx.EVT_BUTTON, self.onClose, self.buttonExit)
        self.Bind(wx.EVT_CLOSE, self.onClose)
        
        # Set initial state
        self.choiceSerialPort.Disable()
        self.choiceBaud.Disable()
        self.labelSerialPort.Disable()
        self.labelBaud.Disable()
        self.textConIPPort.Enable()
        self.labelConType.Enable()
        
        ports = mavutil.auto_detect_serial(preferred_list=MAVProxy.modules.mavproxy_link.preferred_ports)
        for p in ports:
            self.choiceSerialPort.Append(p.device)
        self.choiceSerialPort.SetSelection(0)

    def onChangeType(self, event):
        ''' Change between network and serial connection options '''
        choice = self.choiceConnection.GetString( self.choiceConnection.GetSelection())
        if choice in ["udpin", "udpout", "tcpin", "tcp"]:
            self.choiceSerialPort.Disable()
            self.choiceBaud.Disable()
            self.labelSerialPort.Disable()
            self.labelBaud.Disable()
            self.textConIPPort.Enable()
            self.labelConType.Enable()
        else:
            self.choiceSerialPort.Enable()
            self.choiceBaud.Enable()
            self.labelSerialPort.Enable()
            self.labelBaud.Enable()
            self.textConIPPort.Disable()
            self.labelConType.Disable()
            
            self.choiceSerialPort.Clear()
            ports = mavutil.auto_detect_serial(preferred_list=MAVProxy.modules.mavproxy_link.preferred_ports)
            for p in ports:
                self.choiceSerialPort.Append(p.device)
            self.choiceSerialPort.SetSelection(0)
        
    def onAdd(self, event):
        '''Return connection string'''
        choice = self.choiceConnection.GetString( self.choiceConnection.GetSelection())
        self.conStr = None
        if choice in ["udpin", "udpout", "tcpin", "tcp"]:
            self.conStr = "" + choice + ":" + self.textConIPPort.GetValue()
        else:
            self.conStr = "" + self.choiceSerialPort.GetString(self.choiceSerialPort.GetSelection()) + ":" + self.choiceBaud.GetString(self.choiceBaud.GetSelection())
        #print("1. " + self.conStr)
        self.EndModal(wx.ID_ADD)
        
    def onClose(self, event):
        ''' Exit the dialog (cancel) '''
        if event is None and self.IsModal():
            self.EndModal(wx.ID_ADD)
        elif self.IsModal():
            self.EndModal(event.EventObject.Id)
        else:
            self.Close()

class MPMenulinkAddDialog(object):
    '''used to create a file dialog callback'''
    def __init__(self):
        pass

    def call(self):
        '''show a file dialog'''
        from MAVProxy.modules.lib.wx_loader import wx

        dlg = linkAddDialog(None, title='Add New Link')
        if dlg.ShowModal() != wx.ID_ADD:
            return None
        else:
            # get the connection string before closing dialog
            Constr = dlg.conStr
            dlg.Destroy()
            return Constr

if __name__ == "__main__":
    app = wx.App(False)
    
    dlg = linkAddDialog(None, title='Add New Link')
    if dlg.ShowModal() != wx.ID_ADD:
        print("cancelled")
        dlg.Destroy()
    else:
        # get the connection string before closing dialog
        Constr = dlg.conStr
        print("OK: " + Constr)
        dlg.Destroy()
            
    app.MainLoop()
    
