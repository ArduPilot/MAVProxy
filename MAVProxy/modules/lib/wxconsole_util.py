class Text():
    '''text to write to console'''
    def __init__(self, text, fg='black', bg='white'):
        self.text = text
        self.fg = fg
        self.bg = bg

class Title():
    '''window title for the console'''
    def __init__(self, title):
        self.title = title

class Value():
    '''a value for the status bar'''
    def __init__(self, name, text, row=0, fg='black', bg='white'):
        self.name = name
        self.text = text
        self.row = row
        self.fg = fg
        self.bg = bg