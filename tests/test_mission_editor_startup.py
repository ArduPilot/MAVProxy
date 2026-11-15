"""Mission editor GUI startup must not fork MAVProxy's threaded parent."""
import multiprocessing
import sys
import threading
import types
import unittest
from unittest import mock

from pymavlink import mavwp
from MAVProxy.modules import mavproxy_misseditor
from MAVProxy.modules.mavproxy_misseditor import mission_editor, me_event


def run_headless_gui(target, args, started, loaded):
    """Exercise the real child entry point with display-free wx stand-ins."""
    stopped = threading.Event()

    class Frame:
        def __init__(self, state, **kwargs):
            self.state = state

        def set_gui_event_queue(self, queue):
            self.gui_event_queue = queue

        def Show(self):
            started.set()

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
            event = self.frame.gui_event_queue.get(timeout=5)
            assert event.get_type() == me_event.MEGE_LOAD_MISSION
            assert event.get_arg('wploader').count() == 0
            assert self.frame.state.object_queue.get(timeout=5) == 'layout'
            loaded.set()
            if not stopped.wait(10):
                raise RuntimeError('GUI shutdown timed out')

    wx = types.SimpleNamespace(App=App, ID_ANY=-1, CallAfter=lambda fn: fn())
    frame_module = types.SimpleNamespace(MissionEditorFrame=Frame)
    with mock.patch.object(mavproxy_misseditor, 'missionEditorFrame', frame_module, create=True), \
            mock.patch.dict(sys.modules, {
                'MAVProxy.modules.lib.wx_loader': types.SimpleNamespace(wx=wx),
                'MAVProxy.modules.mavproxy_misseditor.missionEditorFrame': frame_module,
            }):
        target(*args)


def context_probe_child(q, lock, gui_queue, gui_lock, close_window, elemodel,
                        object_queue):
    """Check the actual spawn context's queues, locks and semaphore."""
    with lock:
        q.put(('child-ready', elemodel))
    with gui_lock:
        gui_queue.put(('child-ready', elemodel))
    assert object_queue.get(timeout=5) == 'layout'
    close_window.acquire()


def close_queue(queue):
    queue.close()
    queue.join_thread()


class MissionEditorStartupTests(unittest.TestCase):
    def test_child_entry_round_trips_ipc_for_supported_contexts(self):
        """The static target works with every supported standard context."""
        for method in ('spawn', 'forkserver', 'fork'):
            if method not in multiprocessing.get_all_start_methods():
                continue
            with self.subTest(method=method):
                ctx = multiprocessing.get_context(method)
                started, loaded = ctx.Event(), ctx.Event()
                queues = [ctx.Queue(), ctx.Queue(), ctx.Queue()]
                event_queue, gui_event_queue, object_queue = queues
                event_lock, gui_event_lock = ctx.Lock(), ctx.Lock()
                close_window = ctx.Semaphore(0)
                process = ctx.Process(
                    target=run_headless_gui,
                    args=(mission_editor.MissionEditorMain.child_task,
                          (event_queue, event_lock, gui_event_queue, gui_event_lock,
                           close_window, 'SRTM3', object_queue), started, loaded))
                try:
                    process.start()
                    self.assertTrue(started.wait(5), 'GUI did not start')
                    with gui_event_lock:
                        gui_event_queue.put(mission_editor.MissionEditorEvent(
                            me_event.MEGE_LOAD_MISSION, wploader=mavwp.MAVWPLoader()))
                    object_queue.put('layout')
                    self.assertTrue(loaded.wait(5), 'GUI did not receive IPC messages')
                    self.assertTrue(process.is_alive())
                finally:
                    close_window.release()
                    process.join(5)
                    self.assertFalse(process.is_alive())
                    self.assertEqual(process.exitcode, 0)
                    process.close()
                    for queue in queues:
                        close_queue(queue)

    def test_threaded_parent_uses_spawn_context_and_repeatedly_starts(self):
        """Mission Editor avoids fork after MAVProxy has started worker threads."""
        parent_thread_ready = threading.Event()
        parent_thread_stop = threading.Event()

        def parent_worker():
            parent_thread_ready.set()
            parent_thread_stop.wait(10)

        parent_thread = threading.Thread(target=parent_worker)
        parent_thread.start()
        self.assertTrue(parent_thread_ready.wait(5))

        try:
            for attempt in range(3):
                with self.subTest(attempt=attempt):
                    closed, other = multiprocessing.Pipe()
                    closed.close()
                    other.close()
                    wp = types.SimpleNamespace(wploader=mavwp.MAVWPLoader())
                    state = types.SimpleNamespace(
                        console=types.SimpleNamespace(child_pipe_send=closed),
                        lock=threading.Lock(),
                        module=lambda name: wp if name == 'wp' else None)
                    editor = None
                    try:
                        with mock.patch.object(
                                mission_editor.MissionEditorMain, 'child_task',
                                staticmethod(context_probe_child)), mock.patch.object(
                                    mission_editor.multiprocessing, 'get_context',
                                    wraps=multiprocessing.get_context) as get_context:
                            editor = mission_editor.MissionEditorMain(state, 'SRTM3')
                        get_context.assert_called_once_with('spawn')
                        self.assertEqual(editor.event_queue.get(timeout=5),
                                         ('child-ready', 'SRTM3'))
                        self.assertEqual(editor.gui_event_queue.get(timeout=5),
                                         ('child-ready', 'SRTM3'))
                        editor.set_layout('layout')
                        self.assertTrue(editor.child.is_alive())
                    finally:
                        if editor is not None:
                            editor.close()
                            editor.event_thread.join(5)
                            editor.child.join(5)
                            self.assertFalse(editor.event_thread.is_alive())
                            self.assertFalse(editor.child.is_alive())
                            self.assertEqual(editor.child.exitcode, 0)
                            editor.child.close()
                            for queue in (editor.event_queue, editor.gui_event_queue,
                                          editor.object_queue,
                                          editor.mavlink_message_queue):
                                close_queue(queue)
        finally:
            parent_thread_stop.set()
            parent_thread.join(5)
            self.assertFalse(parent_thread.is_alive())


if __name__ == '__main__':
    unittest.main()
