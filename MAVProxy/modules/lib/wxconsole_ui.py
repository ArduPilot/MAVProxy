import time
import os
from MAVProxy.modules.lib import mp_menu
from MAVProxy.modules.lib.wxconsole_util import Value, Text
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import win_layout

class ConsoleFrame(wx.Frame):
    """ The main frame of the console"""

    def __init__(self, state, title):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(800,300))
        self.panel = wx.Panel(self)
        self.panel.SetBackgroundColour('white')
        state.frame = self

        # values for the status bar
        self.values = {}

        self.menu = None
        self.menu_callback = None
        self.last_layout_send = time.time()

        self.control = wx.TextCtrl(self.panel, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_AUTO_URL)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        # start with one status row
        self.status = [wx.BoxSizer(wx.HORIZONTAL)]
        self.vbox.Add(self.status[0], 0, flag=wx.ALIGN_LEFT | wx.TOP)
        self.vbox.Add(self.control, 1, flag=wx.LEFT | wx.BOTTOM | wx.GROW)

        self.panel.SetSizer(self.vbox)

        self.timer = wx.Timer(self)

        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)

        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_TEXT_URL, self.on_text_url)

        self.Show(True)
        self.pending = []

    def on_menu(self, event):
        '''handle menu selections'''
        state = self.state
        ret = self.menu.find_selected(event)
        if ret is None:
            return
        ret.call_handler()
        state.child_pipe_send.send(ret)

    def on_text_url(self, event):
        '''handle double clicks on URL text'''
        try:
            import webbrowser
        except ImportError:
            return
        mouse_event = event.GetMouseEvent()
        if mouse_event.LeftDClick():
            url_start = event.GetURLStart()
            url_end = event.GetURLEnd()
            url = self.control.GetRange(url_start, url_end)
            try:
                # attempt to use google-chrome
                browser_controller = webbrowser.get('google-chrome')
                browser_controller.open_new_tab(url)
            except webbrowser.Error:
                # use the system configured default browser
                webbrowser.open_new_tab(url)

    def on_idle(self, event):
        time.sleep(0.05)
        now = time.time()
        if now - self.last_layout_send > 1:
            self.last_layout_send = now
            self.state.child_pipe_send.send(win_layout.get_wx_window_layout(self))

    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe_recv.poll():
            try:
                obj = state.child_pipe_recv.recv()
            except EOFError:
                self.timer.Stop()
                self.Destroy()
                return
            
            if isinstance(obj, Value):
                # request to set a status field
                if not obj.name in self.values:
                    # create a new status field
                    value = wx.StaticText(self.panel, -1, obj.text)
                    # possibly add more status rows
                    for i in range(len(self.status), obj.row+1):
                        self.status.append(wx.BoxSizer(wx.HORIZONTAL))
                        self.vbox.Insert(len(self.status)-1, self.status[i], 0, flag=wx.ALIGN_LEFT | wx.TOP)
                        self.vbox.Layout()
                    self.status[obj.row].Add(value, border=5)
                    self.status[obj.row].AddSpacer(20)
                    self.values[obj.name] = value
                value = self.values[obj.name]
                value.SetForegroundColour(obj.fg)
                value.SetBackgroundColour(obj.bg)
                value.SetLabel(obj.text)
                self.panel.Layout()
            elif isinstance(obj, Text):
                '''request to add text to the console'''
                self.pending.append(obj)
                for p in self.pending:
                    # we're scrolled at the bottom
                    oldstyle = self.control.GetDefaultStyle()
                    style = wx.TextAttr()
                    style.SetTextColour(p.fg)
                    style.SetBackgroundColour(p.bg)
                    self.control.SetDefaultStyle(style)
                    self.control.AppendText(p.text)
                    self.control.SetDefaultStyle(oldstyle)
                self.pending = []
            elif isinstance(obj, mp_menu.MPMenuTop):
                if obj is not None:
                    self.SetMenuBar(None)
                    self.menu = obj
                    self.SetMenuBar(self.menu.wx_menu())
                    self.Bind(wx.EVT_MENU, self.on_menu)
                self.Refresh()
                self.Update()
            elif isinstance(obj, win_layout.WinLayout):
                win_layout.set_wx_window_layout(self, obj)
