import wx
from MainFrame import MainFrame

class pyFEdit(wx.App):
    def OnInit(self):
        self.m_frame = MainFrame(None)
        self.m_frame.Show()
        self.SetTopWindow(self.m_frame)
        return True

app = pyFEdit(0)
app.MainLoop()
