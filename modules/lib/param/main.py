import wx
from MainFrame import MainFrame

# Change "oneMinutePython" with the name of your application

class pyparamedit(wx.App):
    def OnInit(self):
        self.m_frame = MainFrame(None)
        self.m_frame.Show()
        self.SetTopWindow(self.m_frame)
        return True

app = pyparamedit(0)
app.MainLoop()
