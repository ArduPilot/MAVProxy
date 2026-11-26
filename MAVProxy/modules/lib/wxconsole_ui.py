import errno
import time
import platform
import socket
from MAVProxy.modules.lib import mp_menu
from MAVProxy.modules.lib.wxconsole_util import Value, Text, Title
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import win_layout
from MAVProxy.modules.lib import icon

class ConsoleFrame(wx.Frame):
    """ The main frame of the console"""

    def __init__(self, state, title):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(800,300))
        # different icons for MAVExplorer and MAVProxy
        try:
            if title == "MAVExplorer":
                self.SetIcon(icon.SimpleIcon("EXPLORER").get_ico())
            else:
                self.SetIcon(icon.SimpleIcon("CONSOLE").get_ico())
        except Exception:
            pass
        self.panel = wx.Panel(self)
        self.panel.SetBackgroundColour('white')
        state.frame = self

        # values for the status bar
        self.values = {}

        self.menu = None
        self.menu_callback = None
        self.menu_open = False
        self.menu_generation = 0
        self.menu_actions = 0
        self.pending_menu = None
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
        self.Bind(wx.EVT_MENU, self.on_menu)
        self.Bind(wx.EVT_MENU_OPEN, self.on_menu_open)
        self.Bind(wx.EVT_MENU_CLOSE, self.on_menu_close)
        self.Bind(wx.EVT_TEXT_URL, self.on_text_url)

        self.Show(True)
        self.pending = []

    def on_menu(self, event):
        '''handle menu selections'''
        state = self.state
        ret = self.menu.find_selected(event)
        if ret is None:
            return
        self.menu_actions += 1
        try:
            ret.call_handler()
            state.child_pipe_send.send(ret)
        finally:
            self.menu_actions -= 1
            if self.pending_menu is not None:
                wx.CallAfter(self.apply_pending_menu)

    def on_menu_open(self, event):
        '''prevent an asynchronous menu update replacing an open popup'''
        self.menu_generation += 1
        self.menu_open = True
        event.Skip()

    def on_menu_close(self, event):
        '''apply the newest deferred menu after the native popup is gone'''
        event.Skip()
        menu = event.GetMenu()
        if menu is not None and not self.is_top_menu(menu):
            # Closing a submenu leaves its parent popup active on GTK.
            return
        # Keep timer-driven updates deferred until native cleanup has returned.
        wx.CallAfter(self.finish_menu_close, self.menu_generation)

    def is_top_menu(self, menu):
        parent = menu.GetParent()
        if parent is not None and 'wxMac' in wx.PlatformInfo:
            # macOS hangs menubar menus off a hidden root menu
            parent = parent.GetParent()
        return parent is None

    def finish_menu_close(self, generation):
        if not self or self.IsBeingDeleted() or generation != self.menu_generation:
            return
        # Opening another menu before this callback ran starts a new session.
        self.menu_open = False
        self.apply_pending_menu()

    def apply_pending_menu(self):
        '''install the newest menu description when no popup is active'''
        if (not self or self.IsBeingDeleted() or self.menu_open or
                self.menu_actions or self.pending_menu is None):
            return
        menubar = self.pending_menu.wx_menu()
        previous = self.GetMenuBar()
        self.SetMenuBar(menubar)
        self.menu = self.pending_menu
        self.pending_menu = None
        if previous is not None:
            # SetMenuBar detaches the previous bar without deleting it.
            previous.Destroy()
        self.Refresh()
        self.Update()

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

        while True:
            try:
                poll_success = state.child_pipe_recv.poll()
                if not poll_success:
                    return
            except socket.error as e:
                if e.errno == errno.EPIPE:
                    break
                else:
                    raise e

            try:
                obj = state.child_pipe_recv.recv()
            except Exception:
                break
                
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
                # workaround wx bug on windows
                value._foregroundColour = obj.fg
                value.SetLabel(obj.text)
                if platform.system() == 'Windows':
                    # more working around wx bugs in windows; without
                    # these the display does not update on colour change
                    value.Refresh()
                    value.Update()
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
            elif isinstance(obj, Title):
                self.SetTitle(obj.title)
            elif isinstance(obj, mp_menu.MPMenuTop):
                self.pending_menu = obj
                if not self.menu_open:
                    self.apply_pending_menu()
            elif isinstance(obj, win_layout.WinLayout):
                win_layout.set_wx_window_layout(self, obj)

        self.timer.Stop()
        self.Destroy()
        return
