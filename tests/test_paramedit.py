"""Parameter editor startup and resource lifecycle regressions (no display needed)."""
import gc
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import threading
import types
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules import mavproxy_paramedit
from MAVProxy.modules.mavproxy_paramedit import param_editor, ph_event


def run_headless_gui(target, args, ready, write_on_exit=False):
    """Run the real GUI entry point with wx stand-ins in a fresh process."""
    stopped = threading.Event()

    class Frame:
        def __init__(self, **kwargs):
            pass

        def set_event_queue(self, queue):
            self.event_queue = queue
            queue.put(ph_event.ParamEditorEvent(ph_event.PEE_READ_PARAM))

        def set_gui_event_queue(self, queue):
            self.gui_event_queue = queue

        def set_param_init(self, params, vehicle):
            assert params == {'TEST_PARAM': 1}
            assert vehicle == 'ArduCopter'

        def __getattr__(self, name):
            return lambda *args: None

    class App:
        def __init__(self, redirect):
            pass

        def SetExitOnFrameDelete(self, enabled):
            pass

        def ExitMainLoop(self):
            if write_on_exit:
                # close() has already stopped the event consumer at this point.
                self.frame.event_queue.put(ph_event.ParamEditorEvent(
                    ph_event.PEE_WRITE_PARAM, modparam={'TEST_PARAM': 'x' * 1048576}))
            stopped.set()

        def MainLoop(self):
            event = self.frame.gui_event_queue.get(timeout=5)
            assert event.get_type() == ph_event.PEGE_READ_PARAM
            ready.set()
            if not stopped.wait(10):
                raise RuntimeError('GUI shutdown timed out')

    wx = types.SimpleNamespace(App=App, ID_ANY=-1,
                               CallAfter=lambda fn: fn())
    with mock.patch.dict(sys.modules, {
        'MAVProxy.modules.lib.wx_loader': types.SimpleNamespace(wx=wx),
        'MAVProxy.modules.mavproxy_paramedit.param_editor_frame':
            types.SimpleNamespace(ParamEditorFrame=Frame),
    }):
        target(*args)


def stalled_gui(args, ready, partial_read):
    """Stay inside Queue.get() until the parent forcibly stops the process."""
    gui_queue = args[2]

    def stalled_recv():
        if partial_read:
            # Consume the frame header, leaving an incomplete message behind.
            gui_queue._reader._recv(4)
        ready.set()
        threading.Event().wait(30)

    gui_queue._recv_bytes = stalled_recv
    gui_queue.get()


def ignore_sigterm(signum, frame):
    # MAVProxy's handler similarly returns without exiting on first delivery.
    pass


def make_state():
    # These objects reproduce both the logged failure and other state which
    # must never be serialized into the GUI process.
    closed_pipe, other_pipe = multiprocessing.Pipe()
    closed_pipe.close()
    other_pipe.close()
    param = types.SimpleNamespace(mav_param={'TEST_PARAM': 1})
    return types.SimpleNamespace(
        public_modules={}, vehicle_name='ArduCopter',
        settings=types.SimpleNamespace(moddebug=0),
        console=types.SimpleNamespace(child_pipe_send=closed_pipe),
        lock=threading.Lock(), module=lambda name: param if name == 'param' else None)


class ParamEditorTests(unittest.TestCase):
    def test_failed_startup_is_not_retried(self):
        editor = mavproxy_paramedit.ParamEditorModule(make_state())
        with mock.patch.object(param_editor, 'ParamEditorMain',
                               side_effect=OSError('handle is closed')) as start, \
                mock.patch('builtins.print') as report:
            with self.assertRaisesRegex(OSError, 'handle is closed'):
                editor.idle_task()
            for _ in range(100):
                editor.idle_task()
            start.assert_called_once()
            report.assert_called_once_with('Failed to start parameter editor: handle is closed')
        self.assertTrue(editor.needs_unloading)
        editor.unload()

    def test_waits_for_parameter_module(self):
        state = make_state()
        state.module = lambda name: None
        editor = mavproxy_paramedit.ParamEditorModule(state)
        with mock.patch.object(param_editor, 'ParamEditorMain') as start:
            editor.idle_task()
            start.assert_not_called()
        self.assertFalse(editor.needs_unloading)

    def test_startup_error_survives_cleanup_error(self):
        with mock.patch.object(param_editor.ParamEditorMain, 'start',
                               side_effect=OSError('startup failure')), \
                mock.patch.object(param_editor.ParamEditorMain, 'close',
                                  side_effect=RuntimeError('cleanup failure')), \
                mock.patch('builtins.print') as report:
            with self.assertRaisesRegex(OSError, 'startup failure'):
                param_editor.ParamEditorMain(make_state())
            report.assert_called_once_with('Parameter editor cleanup failed: cleanup failure')

    @unittest.skipIf(sys.platform == 'win32', 'requires process queues')
    def test_slow_feeder_is_cleaned_up_without_retry(self):
        started = threading.Event()
        resume = threading.Event()

        class SlowEvent:
            def __reduce__(self):
                started.set()
                if not resume.wait(5):
                    raise RuntimeError('Serializer was not released')
                return bytes, (b'x' * 1048576,)

        q = param_editor.multiproc.Queue()
        q.put(SlowEvent())
        cleanup = None
        try:
            self.assertTrue(started.wait(5))
            cleanup = param_editor.ParamEditorMain.close_queue(q, timeout=0)
            self.assertIsNotNone(cleanup)
            self.assertTrue(q._thread.is_alive())
        finally:
            resume.set()
            if cleanup is not None:
                cleanup.join(timeout=5)
            else:
                param_editor.ParamEditorMain.close_queue(q)
        self.assertFalse(cleanup.is_alive())
        self.assertFalse(q._thread.is_alive())
        self.assertTrue(q._reader.closed and q._writer.closed)

    @unittest.skipIf(sys.platform == 'win32', 'requires process queues')
    def test_queue_cleanup_continues_and_can_be_retried(self):
        state = make_state()
        with mock.patch.object(param_editor.ParamEditorMain, 'start'):
            editor = param_editor.ParamEditorMain(state)
        editor.threaded = False
        editor.event_queue = param_editor.multiproc.Queue()
        editor.gui_event_queue = param_editor.multiproc.Queue()
        events, gui_events = editor.event_queue, editor.gui_event_queue
        try:
            with mock.patch.object(events._writer, 'close', side_effect=OSError('close failure')):
                with self.assertRaisesRegex(OSError, 'close failure'):
                    editor.close()
            self.assertTrue(gui_events._reader.closed and gui_events._writer.closed)
            self.assertIsNone(state.param_editor)
        finally:
            editor.close()
        self.assertTrue(events._reader.closed and events._writer.closed)
        self.assertIsNone(editor.event_queue)
        editor.close()

    @unittest.skipIf(sys.platform == 'win32', 'requires process queues')
    def test_partial_queue_allocation_is_cleaned_up(self):
        first_queue = param_editor.multiproc.Queue()
        with mock.patch.object(param_editor.multiproc, 'Queue',
                               side_effect=[first_queue, OSError('no descriptors')]):
            state = make_state()
            with self.assertRaisesRegex(OSError, 'no descriptors'):
                param_editor.ParamEditorMain(state)
        self.assertTrue(first_queue._closed)
        self.assertTrue(first_queue._reader.closed)
        self.assertTrue(first_queue._writer.closed)
        self.assertFalse(hasattr(state, 'param_editor'))

    @unittest.skipUnless(os.path.isdir('/proc/self/fd'), 'requires Linux fd counts')
    def test_repeated_start_failure_does_not_leak_descriptors(self):
        state = make_state()
        # Warm up the multiprocessing resource tracker before counting.
        queue = param_editor.multiproc.Queue()
        queue.close()
        del queue
        gc.collect()
        before = len(os.listdir('/proc/self/fd'))
        with mock.patch.object(param_editor.multiproc.Process, 'start',
                               side_effect=OSError('handle is closed')):
            for _ in range(30):
                with self.assertRaisesRegex(OSError, 'handle is closed'):
                    param_editor.ParamEditorMain(state)
        gc.collect()
        self.assertEqual(len(os.listdir('/proc/self/fd')), before)
        self.assertFalse(hasattr(state, 'param_editor'))

    @unittest.skipIf(sys.platform == 'win32', 'requires process GUI backend')
    def test_process_start_methods_and_shutdown_with_full_gui_queue(self):
        for method in ('spawn', 'forkserver', 'fork'):
            if method not in multiprocessing.get_all_start_methods():
                continue
            with self.subTest(method=method):
                ctx = multiprocessing.get_context(method)
                ready = ctx.Event()

                def process_factory(*, target, args):
                    # The production target and arguments are still pickled,
                    # so a bound target would try to serialize make_state().
                    return ctx.Process(target=run_headless_gui, args=(target, args, ready))

                backend = types.SimpleNamespace(
                    Process=process_factory, Queue=ctx.Queue, Lock=ctx.Lock,
                    Semaphore=ctx.Semaphore)
                with mock.patch.object(param_editor, 'multiproc', backend), \
                        mock.patch.object(param_editor.platform, 'system', return_value='Linux'):
                    state = make_state()
                    editor = param_editor.ParamEditorMain(state)
                    threads = (editor.event_thread, editor.mavlink_thread)
                    try:
                        self.assertTrue(ready.wait(5), 'GUI did not read the parameter event')
                        self.assertTrue(editor.child.is_alive())
                        # Much larger than a pipe buffer; closing must drain it.
                        for _ in range(100):
                            editor.gui_event_queue.put(b'x' * 65536)
                        feeder = editor.gui_event_queue._thread
                    finally:
                        editor.close()
                    editor.close()  # cleanup is idempotent
                    self.assertIsNone(state.param_editor)
                    self.assertTrue(all(not t.is_alive() for t in threads))
                    self.assertFalse(feeder.is_alive())
                    self.assertIsNone(editor.child)

    def test_windows_thread_shutdown_with_pending_events(self):
        ready = threading.Event()
        target = param_editor.ParamEditorMain.child_task

        def thread_gui(*args):
            run_headless_gui(target, args, ready, write_on_exit=True)

        with mock.patch.object(param_editor.platform, 'system', return_value='Windows'), \
                mock.patch.object(param_editor.ParamEditorMain, 'child_task', staticmethod(thread_gui)), \
                mock.patch.object(param_editor.mp_util, 'child_close_fds') as close_fds:
            state = make_state()
            editor = param_editor.ParamEditorMain(state)
            threads = (editor.child, editor.event_thread, editor.mavlink_thread)
            events = editor.event_queue
            try:
                self.assertTrue(ready.wait(5))
                for _ in range(100):
                    editor.gui_event_queue.put(b'x' * 65536)
            finally:
                editor.close()
            editor.close()
            event = events.get_nowait()
            self.assertEqual(event.get_type(), ph_event.PEE_WRITE_PARAM)
            self.assertEqual(len(event.get_arg('modparam')['TEST_PARAM']), 1048576)
            self.assertTrue(all(not t.is_alive() for t in threads))
            self.assertIsNone(state.param_editor)
            close_fds.assert_not_called()

    @unittest.skipIf(sys.platform == 'win32', 'requires process GUI backend')
    def test_shutdown_during_gui_queue_read(self):
        for method in multiprocessing.get_all_start_methods():
            for partial_read in (False, True):
                with self.subTest(method=method, partial_read=partial_read):
                    ctx = multiprocessing.get_context(method)
                    ready = ctx.Event()

                    def process_factory(*, target, args):
                        return ctx.Process(target=stalled_gui, args=(args, ready, partial_read))

                    backend = types.SimpleNamespace(
                        Process=process_factory, Queue=ctx.Queue, Lock=ctx.Lock,
                        Semaphore=ctx.Semaphore)
                    # Exercise an inherited non-exiting handler for fork.
                    old_handler = signal.signal(signal.SIGTERM, ignore_sigterm)
                    try:
                        with mock.patch.object(param_editor, 'multiproc', backend):
                            state = make_state()
                            editor = param_editor.ParamEditorMain(state)
                            child = editor.child
                            queues = (editor.event_queue, editor.gui_event_queue)
                            # Another forked module may retain a pipe reader.
                            # Closing our reader alone must not be the only
                            # way to wake the blocked feeder.
                            extra_reader = os.dup(editor.gui_event_queue._reader.fileno())
                            try:
                                editor.gui_event_queue.put(b'x' * 1048576)
                                feeder = editor.gui_event_queue._thread
                                self.assertTrue(ready.wait(5), 'Child did not enter queue read')
                                editor.close()
                                self.assertIsNone(editor.child)
                                self.assertFalse(feeder.is_alive())
                                self.assertTrue(all(q._reader.closed and q._writer.closed for q in queues))
                            finally:
                                os.close(extra_reader)
                                if not child._closed:
                                    if child.is_alive():
                                        child.kill()
                                    child.join(timeout=5)
                                    child.close()
                    finally:
                        signal.signal(signal.SIGTERM, old_handler)


if __name__ == '__main__':
    unittest.main()
