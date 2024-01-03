'''
AI Chat Module
Randy Mackay, December 2023

This module allows MAVProxy to interface with OpenAI Assitants

OpenAI Assistant API: https://platform.openai.com/docs/api-reference/assistants
OpenAI Assistant Playground: https://platform.openai.com/playground
MAVProxy chat wiki: https://ardupilot.org/mavproxy/docs/modules/chat.html

AP_FLAKE8_CLEAN
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_chat import chat_window
from pymavlink import mavutil
from threading import Thread
import time


class chat(mp_module.MPModule):
    def __init__(self, mpstate):

        # call parent class
        super(chat, self).__init__(mpstate, "chat", "OpenAI chat support")

        # register module and commands
        self.add_command('chat', self.cmd_chat, "chat module", ["show"])

        # keep reference to mpstate
        self.mpstate = mpstate

        # a dictionary of command_ack mavcmds we are waiting for
        # key is the mavcmd, value is None or the MAV_RESULT (e.g. 0 to 9)
        # we assume we will never be waiting for two of the same mavcmds at the same time
        self.command_ack_waiting = {}

        # run chat window in a separate thread
        self.thread = Thread(target=self.create_chat_window)
        self.thread.start()

    # create chat window (should be called from a new thread)
    def create_chat_window(self):
        if mp_util.has_wxpython:
            # create chat window
            self.chat_window = chat_window.chat_window(self.mpstate, self.wait_for_command_ack)
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

    # handle mavlink packet
    def mavlink_packet(self, m):
        if m.get_type() == 'COMMAND_ACK':
            self.handle_command_ack(m)

    # handle_command_ack.  should be called if module receives a COMMAND_ACK command
    def handle_command_ack(self, m):
        # return immediately if we are not waiting for this command ack
        if m.command not in self.command_ack_waiting:
            return

        # throw away value if result in progress
        if m.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            return

        #  set the mav result for this command
        self.command_ack_waiting[m.command] = m.result

    # wait for COMMAND_ACK with the specified mav_cmd
    # this should be called immediately after sending a command_long or command_int
    # mav_cmd should be set to the command id that was sent
    # returns MAV_RESULT if command ack received, False if timed out
    # Note: this should not be called from the main thread because it blocks for up to "timeout" seconds
    def wait_for_command_ack(self, mav_cmd, timeout=1):
        # error if we are already waiting for this command ack
        if mav_cmd in self.command_ack_waiting:
            print("chat: already waiting for command ack for mavcmd:" + str(mav_cmd))
            return False

        # add to list of commands we are waiting for (None indicates we don't know result yet)
        self.command_ack_waiting[mav_cmd] = None

        # wait for ack, checking for a response every 0.1 seconds
        start_time = time.time()
        while time.time() - start_time < timeout:
            # check if we got the ack we were waiting for
            result = self.command_ack_waiting.get(mav_cmd, None)
            if result is not None:
                # remove from list of commands we are waiting for
                del self.command_ack_waiting[mav_cmd]
                return result
            time.sleep(0.1)

        # timeout, remove from list of commands we are waiting for
        # return False indicating timeout
        del self.command_ack_waiting[mav_cmd]
        return False


# initialise module
def init(mpstate):
    return chat(mpstate)
