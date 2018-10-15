#!/usr/bin/env python

"""
  MAVProxy message console, implemented in a child process
"""
import threading
import sys, time

from MAVProxy.modules.lib.wxconsole_util import Value, Text
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import win_layout
from MAVProxy.modules.lib import multiproc

class MessageConsole(textconsole.SimpleConsole):
    '''
    a message console for MAVProxy
    '''
    def __init__(self,
                 title='MAVProxy: console'):
        textconsole.SimpleConsole.__init__(self)
        self.title  = title
        self.menu_callback = None
        self.parent_pipe_recv,self.child_pipe_send = multiproc.Pipe(duplex=False)
        self.child_pipe_recv,self.parent_pipe_send = multiproc.Pipe(duplex=False)
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
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

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.lib.wxconsole_ui import ConsoleFrame
        app = wx.App(False)
        app.frame = ConsoleFrame(state=self, title=self.title)
        app.frame.SetDoubleBuffered(True)
        app.frame.Show()
        app.MainLoop()

    def watch_thread(self):
        '''watch for menu events from child'''
        from MAVProxy.modules.lib.mp_settings import MPSetting
        try:
            while True:
                msg = self.parent_pipe_recv.recv()
                if isinstance(msg, win_layout.WinLayout):
                    win_layout.set_layout(msg, self.set_layout)
                elif self.menu_callback is not None:
                    self.menu_callback(msg)
                time.sleep(0.1)
        except EOFError:
            pass

    def set_layout(self, layout):
        '''set window layout'''
        self.parent_pipe_send.send(layout)
        
    def write(self, text, fg='black', bg='white'):
        '''write to the console'''
        try:
            self.parent_pipe_send.send(Text(text, fg, bg))
        except Exception:
            pass

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

if __name__ == "__main__":
    # test the console
    multiproc.freeze_support()
    console = MessageConsole()
    while console.is_alive():
        console.write('Tick', fg='red')
        console.write(" %s " % time.asctime())
        console.writeln('tock', bg='yellow')
        console.set_status('GPS', 'GPS: OK', fg='blue', bg='green')
        console.set_status('Link1', 'Link1: OK', fg='green', bg='white')
        console.set_status('Date', 'Date: %s' % time.asctime(), fg='red', bg='white', row=2)
        time.sleep(0.5)
