'''
AI Chat Module
Randy Mackay, December 2023

This module allows MAVProxy to interface with OpenAI Assitants

OpenAI Assistant API: https://platform.openai.com/docs/api-reference/assistants
OpenAI Assistant Playground: https://platform.openai.com/playground
MAVProxy chat wiki: https://ardupilot.org/mavproxy/docs/modules/chat.html
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_chat import chat_window
from threading import Thread

class chat(mp_module.MPModule):
    def __init__(self, mpstate):

        # call parent class
        super(chat, self).__init__(mpstate, "chat", "OpenAI chat support")

        # register module and commands
        self.add_command('chat', self.cmd_chat, "chat module", ["show"])

        # keep reference to mpstate
        self.mpstate = mpstate

        # run chat window in a separate thread
        self.thread = Thread(target=self.create_chat_window)
        self.thread.start()

    # create chat window (should be called from a new thread)
    def create_chat_window(self):
        if mp_util.has_wxpython:
            # create chat window
            self.chat_window = chat_window.chat_window(self.mpstate)
        else:
            print("chat: wx support required")

    # show help on command line options
    def usage(self):
        return "Usage: chat <show>"

    # control behaviour of the module
    def cmd_chat(self, args):
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "show":
            self.show()
        else:
            print(self.usage())

    # show chat input window
    def show(self):
        self.chat_window.show()

# initialise module
def init(mpstate):
    return chat(mpstate)
