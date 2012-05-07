#!/usr/bin/env python

"""
  MAVProxy message console, implemented in a child process  
"""
import textconsole, wx

class MessageConsole(textconsole.SimpleConsole):
    '''
    a message console for MAVProxy
    '''
    def __init__(self,
                 title='MAVProxy: console'):
        textconsole.SimpleConsole.__init__(self)
        import multiprocessing
        self.title  = title
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_event = multiprocessing.Event()
        self.close_event.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import wx
        app = wx.PySimpleApp()
        app.frame = ConsoleFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def write(self, text, fg='black', bg='white'):
        '''write to the console'''
        if self.child.is_alive():
            self.parent_pipe.send((text,fg,bg))

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
        wx.Frame.__init__(self, None, title=title, size=(600,300))
        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE)
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)        
        self.timer.Start(100)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Show(True)
        self.pending = []

    def on_idle(self, event):
        import time
        time.sleep(0.05)
    
    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe.poll():
            self.pending.append(state.child_pipe.recv())
            if self.control.GetInsertionPoint() == self.control.GetLastPosition():
                # we're scrolled at the bottom
                for (text, fg, bg) in self.pending:
                    oldstyle = self.control.GetDefaultStyle()
                    style = wx.TextAttr()
                    if fg is not None:
                        style.SetTextColour(fg)
                    if bg is not None:
                        style.SetBackgroundColour(bg)
                    self.control.SetDefaultStyle(style)
                    self.control.AppendText(text)
                    self.control.SetDefaultStyle(oldstyle)
                self.pending = []
    
if __name__ == "__main__":
    # test the console
    import time
    console = MessageConsole()
    while console.is_alive():
        console.write('Tick', fg='red')
        console.write(" %s " % time.asctime())
        console.writeln('tock', bg='yellow')
        time.sleep(0.5)
