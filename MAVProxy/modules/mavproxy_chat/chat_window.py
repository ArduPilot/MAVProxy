'''
AI Chat Module Window
Randy Mackay, December 2023

Chat window for input and output of AI chat
'''

from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.mavproxy_chat import chat_openai, chat_voice_to_text
from threading import Thread, Lock

class chat_window():
    def __init__(self, mpstate):
        # keep reference to mpstate
        self.mpstate = mpstate

        # lock to prevent multiple threads sending text to the assistant at the same time
        self.send_lock = Lock()

        # create chat_openai object
        self.chat_openai = chat_openai.chat_openai(self.mpstate)

        # create chat_voice_to_text object
        self.chat_voice_to_text = chat_voice_to_text.chat_voice_to_text()

        # create chat window
        self.app = wx.App()
        self.frame = wx.Frame(None, title="Chat", size=(650, 200))

        # add menu
        self.menu = wx.Menu()
        self.menu.Append(1, "Set API Key", "Set OpenAI API Key")
        self.menu_bar = wx.MenuBar()
        self.menu_bar.Append(self.menu, "Menu")
        self.frame.SetMenuBar(self.menu_bar)
        self.frame.Bind(wx.EVT_MENU, self.menu_set_api_key_show, id=1)

        # add api key input window
        self.apikey_frame = wx.Frame(None, title="Input OpenAI API Key", size=(560, 50))
        self.apikey_text_input = wx.TextCtrl(self.apikey_frame, id=-1, pos=(10, 10), size=(450, -1), style = wx.TE_PROCESS_ENTER)
        self.apikey_set_button = wx.Button(self.apikey_frame, id=-1, label="Set", pos=(470, 10), size=(75, 25))
        self.apikey_frame.Bind(wx.EVT_BUTTON, self.apikey_set_button_click, self.apikey_set_button)
        self.apikey_frame.Bind(wx.EVT_TEXT_ENTER, self.apikey_set_button_click, self.apikey_text_input)
        self.apikey_frame.Bind(wx.EVT_CLOSE, self.apikey_close_button_click)

        # add a vertical and horizontal sizers
        self.vert_sizer = wx.BoxSizer(wx.VERTICAL)
        self.horiz_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # add a record button
        self.record_button = wx.Button(self.frame, id=-1, label="Rec", size=(75, 25))
        self.frame.Bind(wx.EVT_BUTTON, self.record_button_click, self.record_button)
        self.horiz_sizer.Add(self.record_button, proportion = 0, flag = wx.ALIGN_TOP | wx.ALL, border = 5)

        # add an input text box
        self.text_input = wx.TextCtrl(self.frame, id=-1, value="", size=(450, -1), style = wx.TE_PROCESS_ENTER)
        self.frame.Bind(wx.EVT_TEXT_ENTER, self.text_input_change, self.text_input)
        self.horiz_sizer.Add(self.text_input, proportion = 1, flag = wx.ALIGN_TOP | wx.ALL, border = 5)

        # add a send button
        self.send_button = wx.Button(self.frame, id=-1, label="Send", size=(75, 25))
        self.frame.Bind(wx.EVT_BUTTON, self.send_button_click, self.send_button)
        self.horiz_sizer.Add(self.send_button, proportion = 0, flag = wx.ALIGN_TOP | wx.ALL, border = 5)

        # add a reply box and read-only text box
        self.text_reply = wx.TextCtrl(self.frame, id=-1, size=(600, 80), style=wx.TE_READONLY | wx.TE_MULTILINE)

        # set size hints and add sizer to frame
        self.vert_sizer.Add(self.horiz_sizer, proportion=0, flag=wx.EXPAND)
        self.vert_sizer.Add(self.text_reply, proportion=1, flag=wx.EXPAND, border=5)
        self.frame.SetSizer(self.vert_sizer)

        # show frame
        self.frame.Show()

        # chat window loop (this does not return until the window is closed)
        self.app.MainLoop()

    # show the chat window
    def show(self):
        wx.CallAfter(self.frame.Show())

    # hide the chat window
    def hide(self):
        self.frame.Hide()

    # close the chat window
    # this is called when the module is unloaded
    def close(self):
        self.frame.Close()

    # menu set API key handling.  Shows the API key input window
    def menu_set_api_key_show(self, event):
        self.apikey_frame.Show()

    # set API key set button clicked
    def apikey_set_button_click(self, event):
        self.chat_openai.set_api_key(self.apikey_text_input.GetValue())
        self.apikey_frame.Hide()

    # API key close button clicked
    def apikey_close_button_click(self, event):
        self.apikey_frame.Hide()

    # record button clicked
    def record_button_click(self, event):
        # run record_button_click_execute in a new thread
        th = Thread(target=self.record_button_click_execute, args=(event,))
        th.start()

    # record button clicked
    def record_button_click_execute(self, event):
        # record audio
        rec_filename = self.chat_voice_to_text.record_audio()
        if rec_filename is None:
            print("chat: audio recording failed")
            return

        # convert audio to text and place in input box
        text = self.chat_voice_to_text.convert_audio_to_text(rec_filename)
        if text is None:
            print("chat: audio to text conversion failed")
            return
        wx.CallAfter(self.text_input.SetValue, text)

        # send text to assistant
        self.send_text_to_assistant()

    # send button clicked
    def send_button_click(self, event):
        self.text_input_change(event)

    # handle text input
    def text_input_change(self, event):
        # send text to assistant in a separate thread
        th = Thread(target=self.send_text_to_assistant)
        th.start()

    # send text to assistant.  should be called from a separate thread to avoid blocking
    def send_text_to_assistant(self):
        # get lock
        with self.send_lock:
            # disable buttons and text input to stop multiple inputs (can't be done from a thread or must use CallAfter)
            wx.CallAfter(self.record_button.Disable)
            wx.CallAfter(self.text_input.Disable)
            wx.CallAfter(self.send_button.Disable)

            # get text from text input and clear text input
            send_text = self.text_input.GetValue()
            wx.CallAfter(self.text_input.Clear)

            # copy user input text to reply box
            orig_text_attr = self.text_reply.GetDefaultStyle()
            wx.CallAfter(self.text_reply.SetDefaultStyle, wx.TextAttr(wx.RED, alignment=wx.TEXT_ALIGNMENT_RIGHT))
            wx.CallAfter(self.text_reply.AppendText, send_text + "\n")

            # send text to assistant and place reply in reply box
            reply = self.chat_openai.send_to_assistant(send_text)
            if reply:
                wx.CallAfter(self.text_reply.SetDefaultStyle, orig_text_attr)
                wx.CallAfter(self.text_reply.AppendText, reply + "\n\n")

            # reenable buttons and text input (can't be done from a thread or must use CallAfter)
            wx.CallAfter(self.record_button.Enable)
            wx.CallAfter(self.text_input.Enable)
            wx.CallAfter(self.send_button.Enable)
