"""Parameter editor startup and resource lifecycle regressions (no display needed)."""
import gc
import multiprocessing
import os
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules import mavproxy_paramedit
from MAVProxy.modules.mavproxy_paramedit import param_editor, ph_event


def run_headless_gui(target, args):
    """Run the real GUI entry point with wx stand-ins in a fresh process."""
    stopped = threading.Event()

    class Frame:
        def __init__(self, **kwargs):
            pass

        def set_event_queue(self, queue):
            queue.put(ph_event.ParamEditorEvent(ph_event.PEE_READ_PARAM))

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
            stopped.set()

        def MainLoop(self):
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
                               side_effect=OSError('handle is closed')) as start:
            with self.assertRaisesRegex(OSError, 'handle is closed'):
                editor.idle_task()
            for _ in range(100):
                editor.idle_task()
            start.assert_called_once()
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

    def test_partial_queue_allocation_is_cleaned_up(self):
        first_queue = mock.Mock()
        with mock.patch.object(param_editor.multiproc, 'Queue',
                               side_effect=[first_queue, OSError('no descriptors')]):
            state = make_state()
            with self.assertRaisesRegex(OSError, 'no descriptors'):
                param_editor.ParamEditorMain(state)
        first_queue.close.assert_called_once()
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

    def test_process_start_methods_and_shutdown_with_full_gui_queue(self):
        for method in ('spawn', 'forkserver', 'fork'):
            if method not in multiprocessing.get_all_start_methods():
                continue
            with self.subTest(method=method):
                ctx = multiprocessing.get_context(method)

                def process_factory(*, target, args):
                    # The production target and arguments are still pickled,
                    # so a bound target would try to serialize make_state().
                    return ctx.Process(target=run_headless_gui, args=(target, args))

                backend = types.SimpleNamespace(
                    Process=process_factory, Queue=ctx.Queue, Lock=ctx.Lock,
                    Semaphore=ctx.Semaphore)
                with mock.patch.object(param_editor, 'multiproc', backend):
                    state = make_state()
                    editor = param_editor.ParamEditorMain(state)
                    threads = (editor.event_thread, editor.mavlink_thread)
                    try:
                        event = editor.gui_event_queue.get(timeout=5)
                        self.assertEqual(event.get_type(), ph_event.PEGE_READ_PARAM)
                        self.assertTrue(editor.child.is_alive())
                        # Much larger than a pipe buffer; closing must drain it.
                        for _ in range(100):
                            editor.gui_event_queue.put(b'x' * 65536)
                    finally:
                        editor.close()
                    editor.close()  # cleanup is idempotent
                    self.assertIsNone(state.param_editor)
                    self.assertTrue(all(not t.is_alive() for t in threads))
                    self.assertIsNone(editor.child)


if __name__ == '__main__':
    unittest.main()
