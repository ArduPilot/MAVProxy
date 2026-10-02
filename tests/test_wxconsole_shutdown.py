"""Console cleanup must not leave a child for Python to join forever."""
import multiprocessing
import os
import signal
import threading
import unittest

from MAVProxy.modules.lib import wxconsole


def console_child(close_event, ready, ignore_close=False):
    if ignore_close:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    ready.set()
    if ignore_close:
        threading.Event().wait(60)
    else:
        close_event.wait(10)


@unittest.skipUnless(os.name == 'posix', 'requires POSIX signals')
class ConsoleShutdownTests(unittest.TestCase):
    def test_console_child_is_reaped_even_if_it_ignores_close_and_sigterm(self):
        for ignore_close in (False, True):
            with self.subTest(ignore_close=ignore_close):
                ctx = multiprocessing.get_context('spawn')
                console = wxconsole.MessageConsole.__new__(wxconsole.MessageConsole)
                console.closed = False
                console.close_event = ctx.Event()
                ready = ctx.Event()
                console.parent_pipe_recv, sender = ctx.Pipe(duplex=False)
                receiver, console.parent_pipe_send = ctx.Pipe(duplex=False)
                console.child = ctx.Process(target=console_child,
                                            args=(console.close_event, ready, ignore_close))
                console.child.start()
                try:
                    self.assertTrue(ready.wait(5))
                    console.close()
                    self.assertFalse(console.is_alive())
                    self.assertTrue(console.child._closed)
                    self.assertTrue(console.parent_pipe_recv.closed)
                    self.assertTrue(console.parent_pipe_send.closed)
                    console.close()
                    console.write('late write')
                    console.set_status('late status')
                    console.set_layout(None)
                finally:
                    if not console.child._closed:
                        if console.child.is_alive():
                            console.child.kill()
                        console.child.join(5)
                        console.child.close()
                    sender.close()
                    receiver.close()
                    console.parent_pipe_send.close()
                    console.parent_pipe_recv.close()


if __name__ == '__main__':
    unittest.main()
