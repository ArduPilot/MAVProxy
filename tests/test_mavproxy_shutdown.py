#!/usr/bin/env python3
"""MAVProxy must shut down cleanly on a fatal signal.

The end-to-end tests run a real mavproxy.py on a pseudo-terminal, since
the input loop only blocks on readline when stdin is a tty.  The rest
drive the telemetry log writer directly with real multiprocessing
queues, whose feeder threads are what used to hang MAVProxy at exit.
"""

import ctypes
import errno
import multiprocessing
import os
import re
import select
import signal
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import unittest

from MAVProxy import mavproxy

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# the defaults less terrain, which can download the SRTM file list in a
# child process that exit then waits for
MODULES = ('log,signing,wp,rally,fence,ftp,param,relay,tuneopt,arm,mode,'
           'calibration,rc,auxopt,misc,cmdlong,battery,output,adsb,layout')

# a module whose tab-completion and unload are slow enough to signal
# during.  Each writes a marker file when it has started, so the test
# knows when to send the signal
SLOW_MODULE = textwrap.dedent('''
    import os
    import time

    from MAVProxy.modules.lib import mp_module


    def mark(name):
        open(os.path.join(os.environ['SLOW_MARKERS'], name), 'w').close()


    def stuck():
        # stands in for readline wedged with our input_loop() still on
        # the stack, whatever we raise.  An exception can land on the
        # loop itself, outside the inner try, so catch that too and
        # start again
        try:
            while True:
                try:
                    time.sleep(60)
                except BaseException:
                    pass
        except BaseException:
            stuck()


    class SlowModule(mp_module.MPModule):
        def __init__(self, mpstate):
            super(SlowModule, self).__init__(mpstate, 'slowmodule')
            self.add_command('slowcomplete', self.cmd_slow, 'slow to complete',
                             ['(SLOW)'])
            self.add_completion_function('(SLOW)', self.complete_slow)

        def cmd_slow(self, args):
            pass

        def complete_slow(self, text):
            mark('completing')
            if os.environ.get('STUCK_COMPLETE'):
                stuck()
            time.sleep(float(os.environ.get('SLOW_COMPLETE', '0')))
            mark('completed')
            return ['alpha', 'beta']

        def unload(self):
            mark('unloading')
            time.sleep(float(os.environ.get('SLOW_UNLOAD', '0')))
            print('slowmodule unloaded')


    def init(mpstate):
        return SlowModule(mpstate)
''')


@unittest.skipUnless(os.name == 'posix', 'needs a pty')
class ShutdownTest(unittest.TestCase):
    def start(self, slow_complete=0, slow_unload=0, stuck_complete=False):
        import pty
        tmp = tempfile.TemporaryDirectory(prefix='mavproxy-shutdown-')
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        self.markers = os.path.join(self.tmp, 'markers')
        os.mkdir(self.markers)
        with open(os.path.join(self.tmp, 'slowmodule.py'), 'w') as f:
            f.write(SLOW_MODULE)
        env = dict(os.environ,
                   HOME=self.tmp,
                   PYTHONPATH=os.pathsep.join([REPO, self.tmp]),
                   SLOW_MARKERS=self.markers,
                   SLOW_COMPLETE=str(slow_complete),
                   SLOW_UNLOAD=str(slow_unload),
                   STUCK_COMPLETE='1' if stuck_complete else '')
        (self.pty, child) = pty.openpty()
        self.addCleanup(os.close, self.pty)
        self.process = subprocess.Popen(
            [sys.executable, os.path.join(REPO, 'MAVProxy', 'mavproxy.py'),
             '--master=udpin:127.0.0.1:0',
             '--state-basedir=' + self.tmp,
             '--default-modules=' + MODULES,
             '--load-module=slowmodule'],
            stdin=child, stdout=child, stderr=child,
            cwd=self.tmp, env=env, start_new_session=True)
        os.close(child)
        self.addCleanup(self.kill)
        self.output = b''
        self.wait_for(lambda: b'MAV>' in self.output, 30, 'the prompt')

    def kill(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def read(self, timeout):
        (ready, _, _) = select.select([self.pty], [], [], timeout)
        if ready:
            try:
                self.output += os.read(self.pty, 65536)
            except OSError:
                # EIO once the child has closed its end
                time.sleep(timeout)

    def wait_for(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                self.fail('timed out waiting for %s (exit code %s):\n%s' %
                          (what, self.process.poll(),
                           self.output.decode(errors='replace')))
            self.read(0.05)

    def marked(self, name):
        return os.path.exists(os.path.join(self.markers, name))

    def assert_clean_exit(self, timeout=10):
        '''the process exits by itself, having unloaded every module'''
        self.wait_for(lambda: self.process.poll() is not None, timeout,
                      'MAVProxy to exit')
        self.read(0.1)
        output = self.output.decode(errors='replace')
        self.assertEqual(self.process.returncode, 1, output)
        self.assertNotIn('forcing an exit', output)
        for module in ['link'] + MODULES.split(','):
            self.assertIn('Unloading module %s\r\n' % module, output)
        self.assertIn('slowmodule unloaded', output)
        return output

    def test_sigterm_at_the_prompt(self):
        self.start()
        self.process.send_signal(signal.SIGTERM)
        self.assert_clean_exit()

    def test_sigquit_at_the_prompt(self):
        self.start()
        self.process.send_signal(signal.SIGQUIT)
        self.assert_clean_exit()

    def test_sigint_at_the_prompt(self):
        # not a fatal signal when interactive, but Ctrl-C should still
        # unload the modules
        self.start()
        self.process.send_signal(signal.SIGINT)
        self.assert_clean_exit()

    def test_sigint_during_tab_completion(self):
        self.start(slow_complete=1)
        os.write(self.pty, b'slowcomplete \t')
        self.wait_for(lambda: self.marked('completing'), 10, 'the completer')
        self.process.send_signal(signal.SIGINT)
        self.assert_clean_exit()

    def test_sigint_with_requireexit_keeps_running(self):
        self.start()
        os.write(self.pty, b'set requireexit true\n')
        os.write(self.pty, b'set requireexit\n')
        self.wait_for(lambda: re.search(rb'requireexit +True', self.output), 10,
                      'the setting')
        for n in range(3):
            before = self.output.count(b'Interrupt caught')
            self.process.send_signal(signal.SIGINT)
            self.wait_for(
                lambda: self.output.count(b'Interrupt caught') > before, 10,
                'the interrupt to be reported')
        # still reading commands
        os.write(self.pty, b'set requireexit false\n')
        os.write(self.pty, b'set requireexit\n')
        self.wait_for(lambda: re.search(rb'requireexit +False', self.output), 10,
                      'the setting')
        self.assertIsNone(self.process.poll())
        self.process.send_signal(signal.SIGTERM)
        self.assert_clean_exit()

    @unittest.skipUnless(sys.platform.startswith('linux'), 'needs tgkill')
    def test_sigterm_taken_by_another_thread(self):
        # a signal sent to the process can land on any thread, which
        # does not interrupt the main thread waiting in readline.  Send
        # it to each of the others in turn, as the kernel might
        libc = ctypes.CDLL(None, use_errno=True)
        if not hasattr(libc, 'tgkill'):
            self.skipTest('no tgkill in libc')
        for n in range(3):
            self.start()
            pid = self.process.pid
            for attempt in range(10):
                tasks = sorted(int(t) for t in os.listdir('/proc/%d/task' % pid))
                others = [t for t in tasks if t != pid]
                self.assertTrue(others)
                tid = others[n % len(others)]
                if libc.tgkill(pid, tid, signal.SIGTERM) == 0:
                    break
                # that thread has just exited; pick another
                self.assertEqual(ctypes.get_errno(), errno.ESRCH)
            else:
                self.fail('could not signal a thread')
            self.assert_clean_exit()
            self.doCleanups()

    def test_sigterm_during_tab_completion(self):
        # CPython discards an exception raised in the completer, so the
        # first attempt to leave the input loop is lost here
        self.start(slow_complete=1)
        os.write(self.pty, b'slowcomplete \t')
        self.wait_for(lambda: self.marked('completing'), 10, 'the completer')
        self.process.send_signal(signal.SIGTERM)
        self.assert_clean_exit()

    def test_impatient_signal_does_not_cut_cleanup_short(self):
        self.start(slow_unload=1)
        self.process.send_signal(signal.SIGTERM)
        self.wait_for(lambda: self.marked('unloading'), 10, 'unloading')
        self.process.send_signal(signal.SIGTERM)
        output = self.assert_clean_exit()
        self.assertIn('Shutdown in progress', output)

    def test_impatient_ctrl_c_does_not_cut_cleanup_short(self):
        self.start(slow_unload=1)
        self.process.send_signal(signal.SIGINT)
        self.wait_for(lambda: self.marked('unloading'), 10, 'unloading')
        self.process.send_signal(signal.SIGINT)
        output = self.assert_clean_exit()
        self.assertIn('Shutdown in progress', output)

    def test_signal_after_the_grace_forces_an_exit_from_input(self):
        # the escape hatch has to work however stuck we are
        self.start(stuck_complete=True)
        os.write(self.pty, b'slowcomplete \t')
        self.wait_for(lambda: self.marked('completing'), 10, 'the completer')
        self.process.send_signal(signal.SIGTERM)
        time.sleep(1)
        self.assertIsNone(self.process.poll())
        time.sleep(mavproxy.SHUTDOWN_GRACE)
        self.process.send_signal(signal.SIGTERM)
        self.wait_for(lambda: self.process.poll() is not None, 10,
                      'MAVProxy to exit')
        self.read(0.1)
        self.assertIn(b'forcing an exit', self.output)

    def test_signal_after_the_grace_forces_an_exit(self):
        self.start(slow_unload=60)
        self.process.send_signal(signal.SIGTERM)
        self.wait_for(lambda: self.marked('unloading'), 10, 'unloading')
        time.sleep(mavproxy.SHUTDOWN_GRACE + 0.5)
        self.process.send_signal(signal.SIGTERM)
        self.wait_for(lambda: self.process.poll() is not None, 10,
                      'MAVProxy to exit')
        self.read(0.1)
        self.assertIn(b'forcing an exit', self.output)


class Log(object):
    '''a stand-in log file which can be slow or fail'''
    def __init__(self, name, delay=0, fail=False, fail_flush=False):
        self.name = name
        self.delay = delay
        self.fail = fail
        self.fail_flush = fail_flush
        self.records = 0
        self.flushes = 0

    def write(self, data):
        if self.fail:
            raise IOError(28, 'No space left on device')
        time.sleep(self.delay)
        self.records += 1

    def flush(self):
        if self.fail or self.fail_flush:
            raise IOError(28, 'No space left on device')
        self.flushes += 1


class Settings(object):
    flushlogs = False


class Status(object):
    def __init__(self):
        self.stop_event = threading.Event()
        self.thread = None


class State(object):
    pass


class LogWriterTest(unittest.TestCase):
    def setUp(self):
        self.saved = mavproxy.mpstate
        self.addCleanup(setattr, mavproxy, 'mpstate', self.saved)
        state = State()
        state.settings = Settings()
        state.status = Status()
        state.logqueue_raw = multiprocessing.Queue()
        state.logqueue = multiprocessing.Queue()
        state.logfile_raw = Log('raw')
        state.logfile = Log('cooked')
        mavproxy.mpstate = self.state = state
        self.raw = state.logqueue_raw
        self.cooked = state.logqueue
        self.addCleanup(self.close_queues)

    def close_queues(self):
        for queue in [self.raw, self.cooked]:
            queue.cancel_join_thread()
            queue.close()

    def fill(self, raw, cooked, size=32):
        for (queue, count) in [(self.raw, raw), (self.cooked, cooked)]:
            for i in range(count):
                queue.put(bytearray(size))
        # let the feeder threads push it all into the pipes
        time.sleep(0.2)

    def run_writer(self, timeout=10):
        errors = []

        def run():
            try:
                mavproxy.log_writer()
            except BaseException as ex:
                errors.append(ex)
        writer = threading.Thread(target=run)
        writer.start()
        writer.join(timeout)
        self.assertFalse(writer.is_alive(), 'log_writer did not return')
        return errors

    def assert_cancelled(self):
        self.assertTrue(self.raw._joincancelled)
        self.assertTrue(self.cooked._joincancelled)

    def test_shutdown_writes_everything(self):
        self.fill(300, 300)
        self.state.status.stop_event.set()
        self.assertEqual(self.run_writer(), [])
        self.assertEqual(self.state.logfile_raw.records, 300)
        self.assertEqual(self.state.logfile.records, 300)
        self.assertTrue(self.state.logfile_raw.flushes)
        self.assertTrue(self.state.logfile.flushes)
        self.assert_cancelled()

    def test_a_slow_log_does_not_starve_the_other(self):
        self.fill(100, 1)
        self.state.logfile_raw.delay = 0.1
        self.state.status.stop_event.set()
        start = time.monotonic()
        self.run_writer()
        self.assertLess(time.monotonic() - start, 5)
        self.assertEqual(self.state.logfile.records, 1)
        self.assert_cancelled()

    def test_a_log_failing_in_the_drain_is_stopped(self):
        self.fill(10, 10)
        self.state.logfile_raw.fail = True
        self.state.status.stop_event.set()
        self.assertEqual(self.run_writer(), [])
        # main_loop may still be running, so it must stop filling it
        self.assertFalse(self.state.logqueue_raw)
        self.assertIs(self.state.logqueue, self.cooked)
        self.assertEqual(self.state.logfile.records, 10)
        self.assert_cancelled()

    def test_a_log_failing_only_to_flush_in_the_drain_is_stopped(self):
        self.fill(10, 10)
        self.state.logfile_raw.fail_flush = True
        self.state.status.stop_event.set()
        self.assertEqual(self.run_writer(), [])
        self.assertEqual(self.state.logfile_raw.records, 10)
        self.assertFalse(self.state.logqueue_raw)
        self.assertIs(self.state.logqueue, self.cooked)
        self.assertTrue(self.state.logfile.flushes)
        self.assert_cancelled()

    def test_a_write_failure_in_flight_stops_only_that_log(self):
        self.state.logfile_raw.fail = True
        self.fill(10, 10)
        errors = []
        writer = threading.Thread(
            target=lambda: errors.append(mavproxy.log_writer()))
        writer.start()
        deadline = time.monotonic() + 5
        while self.state.logqueue_raw and time.monotonic() < deadline:
            time.sleep(0.01)
        # the failed log's queue is swapped out so nothing grows it...
        self.assertFalse(self.state.logqueue_raw)
        self.state.logqueue_raw.put(bytearray(32))
        # ...while the healthy log carries on
        self.assertIs(self.state.logqueue, self.cooked)
        self.cooked.put(bytearray(32))
        while self.state.logfile.records < 11 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.state.logfile.records, 11)
        self.state.status.stop_event.set()
        writer.join(10)
        self.assertFalse(writer.is_alive())
        self.assert_cancelled()

    def test_every_log_failing_in_flight_still_cancels(self):
        self.state.logfile_raw.fail = True
        self.state.logfile.fail = True
        self.fill(10, 10)
        self.assertEqual(self.run_writer(), [])
        self.assertFalse(self.state.logqueue_raw)
        self.assertFalse(self.state.logqueue)
        self.assert_cancelled()

    def test_an_unexpected_error_still_cancels(self):
        # e.g. print() to a closed pipe while reporting a failure
        self.state.logfile_raw.fail = True
        self.fill(10, 10)
        saved = mavproxy.print if hasattr(mavproxy, 'print') else None

        def broken_print(*args, **kwargs):
            raise BrokenPipeError(32, 'Broken pipe')
        mavproxy.print = broken_print
        try:
            errors = self.run_writer()
        finally:
            if saved is None:
                del mavproxy.print
            else:
                mavproxy.print = saved
        self.assertEqual([type(e) for e in errors], [BrokenPipeError])
        self.assertFalse(self.state.logqueue_raw)
        self.assertFalse(self.state.logqueue)
        self.assert_cancelled()

    def test_an_unstarted_main_loop_is_not_joined(self):
        self.fill(5, 5)
        self.state.status.thread = threading.Thread(target=lambda: None)
        self.state.status.stop_event.set()
        self.assertEqual(self.run_writer(), [])
        self.assertEqual(self.state.logfile.records, 5)
        self.assert_cancelled()


if __name__ == '__main__':
    unittest.main()
