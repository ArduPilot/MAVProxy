import time
from wxhorizon_util import Attitude
from wx_loader import wx
import math

class HorizonFrame(wx.Frame):
    """ The main frame of the horizon indicator."""

    def __init__(self, state, title):
        self.state = state
        # Create Frame and Panel
        wx.Frame.__init__(self, None, title=title, size=(400,400))
        self.panel = wx.Panel(self)
        state.frame = self

        # Create vbox and panel
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.panel.SetSizer(self.vbox)

        # Create Event Timer
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_IDLE, self.on_idle)

        # Test Button
        self.btn = wx.Button(self.panel,-1,'Display Pitch')
        self.vbox.Add(self.btn,0,wx.ALIGN_CENTER)
        self.Bind(wx.EVT_BUTTON,self.OnButtonClicked)

        # Initialise Attitude
        self.pitch = 0  # Degrees
        self.roll = 0   # Degrees
        self.yaw = 0    # Degrees
        
        # History Values
        self.oldRoll = 0 # Degrees
        
        # Attitude Text
        self.pitchText = wx.StaticText(self.panel,pos=(0,340),label='Pitch: 0.0',style=wx.ALIGN_LEFT)
        self.rollText = wx.StaticText(self.panel,pos=(0,360),label='Roll: 0.0',style=wx.ALIGN_LEFT)
        self.yawText = wx.StaticText(self.panel,1,pos=(0,380),label='Yaw: 0.0',style=wx.ALIGN_LEFT)

        # Add Roll Line
        l = 200; # px
        xc = 200
        yc = 200
        def on_paint(event):
            # Create Drawing Canvasgit st
            dc = wx.PaintDC(self.panel)
            dc.Clear()
            # Draw Roll Line
            dc.SetPen(wx.Pen(wx.BLACK,4))
            x1 = xc-((l/2.0)*math.cos(math.radians(self.roll)))
            y1 = yc-((l/2.0)*math.sin(math.radians(self.roll)))
            x2 = xc+((l/2.0)*math.cos(math.radians(self.roll)))
            y2 = yc+((l/2.0)*math.sin(math.radians(self.roll)))
            dc.DrawLine(x1,y1,x2,y2)
            # Draw Roll Lag Arc
            x1o = xc-((l/2.0)*math.cos(math.radians(self.oldRoll)))
            y1o = yc-((l/2.0)*math.sin(math.radians(self.oldRoll)))
            x2o = xc+((l/2.0)*math.cos(math.radians(self.oldRoll)))
            y2o = yc+((l/2.0)*math.sin(math.radians(self.oldRoll)))
            
            dc.DrawLine(x1,y1,x2,y2)
            #dc.DrawArc(x1,y1,x1o,y1o,xc,yc)
            #dc.DrawArc(x2,y2,x2o,y2o,xc,yc)
        
        self.panel.Bind(wx.EVT_PAINT,on_paint)
        
        # Show Window
        self.Show(True)
        self.pending = []

    def OnButtonClicked(self,e):
        print 'Pitch: %f' % self.pitch

        
    def updateAttitudeText(self):
        'Updates the displayed attitude Text'
        self.pitchText.SetLabel('Pitch: %.2f' % self.pitch)
        self.rollText.SetLabel('Roll: %.2f' % self.roll)
        self.yawText.SetLabel('Yaw: %.2f' % self.yaw)

    def on_idle(self, event):
        time.sleep(0.05)
 
    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        # Get attitude information
        while state.child_pipe_recv.poll():
            obj = state.child_pipe_recv.recv()
            if isinstance(obj,Attitude):
                self.oldRoll = self.roll
                self.pitch = obj.pitch*180/math.pi
                self.roll = obj.roll*180/math.pi
                self.yaw = obj.yaw*180/math.pi
                
                # Update Displayed Text
                self.updateAttitudeText()
                
   
        self.Refresh()
        self.Update()