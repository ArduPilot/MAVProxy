from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_qt_console.ui_qt_console import Ui_QtConsole
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer
from PySide6.QtGui import QColor
import sys
import threading
from MAVProxy.modules.lib import multiproc
import multiprocessing
import time
from MAVProxy.modules.lib.wxconsole_util import  Text, Value
import socket
import errno
from MAVProxy.modules.lib import textconsole

class QtConsoleWindow(QMainWindow):
    def __init__(self, parent):
        super(QtConsoleWindow, self).__init__()
        self._parent = parent
        self._ui = Ui_QtConsole()
        self._ui.setupUi(self)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(200)

        self._ui.actionShow_Map.triggered.connect(self.show_map)
    
    def update(self):
        '''Slot called by QTimer at a specified interval'''
        if self._parent.close_event.wait(0.001):
            self._timer.stop()
            self.close()
            return

        try:
            poll_success = self._parent.child_pipe_recv.poll()
            if not poll_success:
                return
        except socket.error as e:
            if e.errno == errno.EPIPE:
                self._timer.stop()
                return
            else:
                raise e

        try:
            msg = self._parent.child_pipe_recv.recv()
        except EOFError:
            self._timer.stop()
            return
        
        if isinstance(msg, Text):
            self._ui.textEdit.setTextColor(QColor(msg.fg))
            self._ui.textEdit.setTextBackgroundColor(QColor(msg.bg))
            self._ui.textEdit.append(msg.text)
        
    def show_map(self):
        self._parent.child_pipe_send.send("# module load map")

    def closeEvent(self, event) -> None:
        """Handles the cross button on the UI"""
        if not self._parent.close_event.is_set():
            self._parent.child_pipe_send.send("# module unload qt_console")
            return super().closeEvent(event)

class QtConsole(textconsole.SimpleConsole):
    def __init__(self, mpstate) -> None:
        super(QtConsole, self).__init__()
        self.mpstate = mpstate
        self.parent_pipe_recv, self.child_pipe_send = multiproc.Pipe(duplex=False)
        self.child_pipe_recv,self.parent_pipe_send = multiproc.Pipe(duplex=False)
        
        # For quitting cleanly
        self.close_event = multiproc.Event()
        self.close_event.clear()
        
        # main process in which GUI (child) lives
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

        # This class (parent) doesn't need the child pipes
        self.child_pipe_send.close()
        self.child_pipe_recv.close()

        # Thread that listens to clicks etc. from the GUI
        t = threading.Thread(target=self.watch_thread)
        t.daemon = True
        t.start()
    
    def watch_thread(self):
        '''watch for menu events from child'''
        from MAVProxy.modules.lib.mp_settings import MPSetting
        try:
            while True:
                msg = self.parent_pipe_recv.recv()
                if msg.startswith("#"): # Header for command packet
                    self.mpstate.functions.process_stdin(msg[2:])
                # print(msg)
                # if isinstance(msg, win_layout.WinLayout):
                #     win_layout.set_layout(msg, self.set_layout)
                # elif self.menu_callback is not None:
                #     self.menu_callback(msg)
                time.sleep(0.1)
        except EOFError:
            pass

    def child_task(self):
        '''Main Process in which the Qt GUI lives'''
        self.parent_pipe_send.close() # Good sense to close pipes that are not used by this process
        self.parent_pipe_recv.close()
        app = QApplication.instance()
        if app == None:
            app = QApplication()
    
        window = QtConsoleWindow(self)
        window.show()
        app.exec()

    def write(self, text, fg='black', bg='white'):
        '''write to the console'''
        try:
            self.parent_pipe_send.send(Text(text, fg, bg))
        except Exception:
            pass

    def writeln(self, text, fg='black', bg='white'):
        '''write to the console with linefeed'''
        if not isinstance(text, str):
            text = str(text)
        self.write(text, fg=fg, bg=bg)

    def close(self):
        '''close the console'''
        self.close_event.set()
        if self.child.is_alive():
            self.child.join()

class QtConsoleModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "qt_console", "GUI Console (Qt)", public=True, multi_vehicle=True)
        self.add_command('qt_console', self.cmd_qt_console, "qt console module", ['add','list','remove'])        
        self.mpstate.console = QtConsole(mpstate)

    def cmd_qt_console(self, args):
        pass

    def mavlink_packet(self, packet):
        # print("Packet recieved")
        return super().mavlink_packet(packet)
    
    def unload(self):
        '''unload module'''
        self.mpstate.console.close()
        self.mpstate.console = textconsole.SimpleConsole()

def init(mpstate):
    '''initialise module'''
    return QtConsoleModule(mpstate)