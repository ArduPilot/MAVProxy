#!/usr/bin/env python

"""
  MAVProxy message console, implemented in a child process  
"""
import textconsole, wx

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
        print "MessageConsole init"
        textconsole.SimpleConsole.__init__(self)
        import multiprocessing
        self.title  = title
     #   self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
     #   self.close_event = multiprocessing.Event()
     #   self.close_event.clear()
     #   self.child = multiprocessing.Process(target=self.child_task)
     #   self.child.start()
        self.child = self.child_task()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        print "MessageConsole child"
        import wx
        app = wx.App(False)
        app.frame = ConsoleFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

    def write(self, text, fg='black', bg='white'):
        print "MessageConsole write"
        '''write to the console'''
        if self.child.is_alive():
            self.parent_pipe.send(Text(text, fg, bg))

    def set_status(self, name, text='', row=0, fg='black', bg='white'):
        print "MessageConsole status"
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(Value(name, text, row, fg, bg))
        

    def close(self):
        print "MessageConsole close"
        '''close the console'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        print "MessageConsole alive"
        '''check if child is still going'''
        return self.child.is_alive()

class ConsoleFrame(wx.Frame):
    """ The main frame of the console"""
    
    def __init__(self, state, title):
        print "ConsoleFrame init"
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(800,300))
        self.panel = wx.Panel(self)
        
        # values for the status bar
        self.values = {}

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

    def on_idle(self, event):
       print "ConsoleFrame idle"
       import time
       time.sleep(0.05)
    
    def on_timer(self, event):
        print "ConsoleFrame timer"
        state = self.state
   #     if state.close_event.wait(0.001):
   #         self.timer.Stop()
   #         self.Destroy()
   #         return
        while state.child_pipe.poll():
   #         obj = state.child_pipe.recv()
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
    
if __name__ == "__main__":
    # test the console
    import wx
    import time
    print "before MessageConsole"
  #  console = MessageConsole()
    print "after MessageConsole"
    
    state = {}
    app = wx.App(False)
    app.frame = ConsoleFrame(state, title="blah")
    app.frame.Show()
    app.MainLoop()
#
#    while console.is_alive():
#        print "while console.is_alive"
#        console.write('Tick', fg='red')
#        console.write(" %s " % time.asctime())
#        console.writeln('tock', bg='yellow')
#        console.set_status('GPS', 'GPS: OK', fg='blue', bg='green')
#        console.set_status('Link1', 'Link1: OK', fg='green', bg='write')
#        console.set_status('Date', 'Date: %s' % time.asctime(), fg='red', bg='write', row=2)
#        time.sleep(0.5)
