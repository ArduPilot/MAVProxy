'''
Graphical editing of graph definition
'''
from wx_loader import wx

class GraphDefinition(object):
    '''a pre-defined graph'''
    def __init__(self, name, expression, description, expressions, filename):
        self.name = name
        self.expression = expression
        self.description = description
        self.expressions = expressions
        self.filename = filename

class GraphDialog(wx.Dialog):
    def __init__(self, title, graphdef, callback):
        wx.Dialog.__init__(self, None, -1, title, size=(900, 400))

        self.callback = callback
        self.graphdef = graphdef

        self.panel = wx.Panel(self, -1)
        vbox = wx.BoxSizer(wx.VERTICAL)

        # name entry
        hbox_name = wx.BoxSizer(wx.HORIZONTAL)
        st_name   = wx.StaticText(self.panel, -1, 'Name: ')
        self.tc_name = wx.TextCtrl(self.panel, -1, size=(400, -1))
        self.tc_name.Value = self.graphdef.name
        hbox_name.Add(st_name, 0, wx.LEFT, 10)
        hbox_name.Add(self.tc_name, 0, wx.LEFT, 35)
        vbox.Add(hbox_name, 0, wx.TOP, 10)

        # expression entry
        st = wx.StaticText(self.panel, -1, 'Expressions: ')
        vbox.Add(st, 0, wx.LEFT, 10)
        
        hbox_expressions = wx.BoxSizer(wx.HORIZONTAL)
        self.tc_expressions = wx.TextCtrl(self.panel, -1, style=wx.TE_MULTILINE|wx.HSCROLL, size=(800, 80))
        elist = []
        for e in self.graphdef.expressions:
            e = ' '.join(e.split())
            elist.append(e)
        self.tc_expressions.Value = '\n'.join(elist)
        vbox.Add(self.tc_expressions, 0, wx.LEFT, 15)

        # description entry
        st = wx.StaticText(self.panel, -1, 'Description: ')
        vbox.Add(st, 0, wx.LEFT, 10)
        self.tc_description = wx.TextCtrl(self.panel, -1, style=wx.TE_MULTILINE)
        vbox.Add(self.tc_description, 1, wx.EXPAND | wx.TOP | wx.RIGHT | wx.LEFT, 15)
        self.tc_description.Value = self.graphdef.description

        # buttons
        button_save   = wx.Button(self.panel, 1, 'Save')
        button_cancel = wx.Button(self.panel, 2, 'Cancel')
        button_test = wx.Button(self.panel, 3, 'Test')
        hbox_buttons = wx.BoxSizer(wx.HORIZONTAL)
        hbox_buttons.Add(button_save,   0, wx.LEFT, 10)
        hbox_buttons.Add(button_cancel, 0, wx.LEFT, 10)
        hbox_buttons.Add(button_test, 0, wx.LEFT, 10)
        vbox.Add(hbox_buttons, 0, wx.TOP, 10)
        self.Bind(wx.EVT_BUTTON, self.OnSave, id=1)
        self.Bind(wx.EVT_BUTTON, self.OnCancel, id=2)
        self.Bind(wx.EVT_BUTTON, self.OnTest, id=3)
        
        self.panel.SetSizer(vbox)
        self.Centre()

    def update_values(self):
        self.graphdef.name = self.tc_name.Value.strip()
        self.graphdef.expressions = self.tc_expressions.Value.split('\n')
        self.graphdef.description = self.tc_description.Value

    def OnCancel(self, event):
        self.Close()

    def OnTest(self, event):
        self.update_values()
        self.callback('test', self.graphdef)

    def OnSave(self, event):
        self.update_values()
        self.callback('save', self.graphdef)
