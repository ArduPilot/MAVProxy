#!/usr/bin/env python

"""
  MAVProxy message console, implemented in a child process
"""
import multiprocessing, threading
import textconsole, wx, sys, time
import mp_menu
from MAVProxy.modules.lib import mp_util

class Text():
    '''text to write to console'''
    def __init__(self, text, fg='black', bg='white'):
        self.text = text
        self.fg = fg
        self.bg = bg

class Value():
    '''a value for the status bar'''
    def __init__(self, name, text, row=0, fg='black', bg='white'):
        self.name = name
        self.text = text
        self.row = row
        self.fg = fg
        self.bg = bg

class MessageConsole(textconsole.SimpleConsole):
    '''
    a message console for MAVProxy
    '''
    def __init__(self,
                 title='MAVProxy: console'):
        textconsole.SimpleConsole.__init__(self)
        self.title  = title
        self.menu_callback = None
        self.parent_pipe_recv,self.child_pipe_send = multiprocessing.Pipe(duplex=False)
        self.child_pipe_recv,self.parent_pipe_send = multiprocessing.Pipe(duplex=False)
        self.close_event = multiprocessing.Event()
        self.close_event.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()
        self.child_pipe_send.close()
        self.child_pipe_recv.close()
        t = threading.Thread(target=self.watch_thread)
        t.daemon = True
        t.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        self.parent_pipe_send.close()
        self.parent_pipe_recv.close()
        import wx
        app = wx.PySimpleApp()
        app.frame = ConsoleFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def watch_thread(self):
        '''watch for menu events from child'''
        from mp_settings import MPSetting
        while True:
            msg = self.parent_pipe_recv.recv()
            if self.menu_callback is not None:
                self.menu_callback(msg)
            time.sleep(0.1)

    def write(self, text, fg='black', bg='white'):
        '''write to the console'''
        if self.is_alive():
            self.parent_pipe_send.send(Text(text, fg, bg))

    def set_status(self, name, text='', row=0, fg='black', bg='white'):
        '''set a status value'''
        if self.is_alive():
            self.parent_pipe_send.send(Value(name, text, row, fg, bg))

    def set_menu(self, menu, callback):
        if self.is_alive():
            self.parent_pipe_send.send(menu)
            self.menu_callback = callback

    def close(self):
        '''close the console'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

class ConsoleFrame(wx.Frame):
    """ The main frame of the console"""

    def __init__(self, state, title):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(800,300))
        self.panel = wx.Panel(self)
        state.frame = self

        # values for the status bar
        self.values = {}

        self.menu = None
        self.menu_callback = None

        self.control = wx.TextCtrl(self.panel, style=wx.TE_MULTILINE | wx.TE_READONLY)


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

    def on_idle(self, event):
        time.sleep(0.05)

    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe_recv.poll():
            obj = state.child_pipe_recv.recv()
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

if __name__ == "__main__":
    # test the console
    multiprocessing.freeze_support()
    console = MessageConsole()
    while console.is_alive():
        console.write('Tick', fg='red')
        console.write(" %s " % time.asctime())
        console.writeln('tock', bg='yellow')
        console.set_status('GPS', 'GPS: OK', fg='blue', bg='green')
        console.set_status('Link1', 'Link1: OK', fg='green', bg='white')
        console.set_status('Date', 'Date: %s' % time.asctime(), fg='red', bg='white', row=2)
        time.sleep(0.5)
