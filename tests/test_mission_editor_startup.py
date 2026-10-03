"""Mission editor GUI startup must not serialize live MAVProxy state."""
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
            assert event.get_type() == me_event.MEGE_CLEAR_MISS_TABLE
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


class MissionEditorStartupTests(unittest.TestCase):
    def test_empty_mission_starts_without_serializing_closed_parent_handles(self):
        for method in ('spawn', 'forkserver', 'fork'):
            if method not in multiprocessing.get_all_start_methods():
                continue
            with self.subTest(method=method):
                ctx = multiprocessing.get_context(method)
                started, loaded = ctx.Event(), ctx.Event()
                queues = []

                def process_factory(target, args):
                    # Keep the production target in the pickle: a bound method
                    # would bring the closed connection and lock along with it.
                    return ctx.Process(target=run_headless_gui, args=(target, args, started, loaded))

                def make_queue():
                    q = ctx.Queue()
                    queues.append(q)
                    return q

                closed, other = ctx.Pipe()
                closed.close()
                other.close()
                wp = types.SimpleNamespace(wploader=mavwp.MAVWPLoader())
                state = types.SimpleNamespace(
                    console=types.SimpleNamespace(child_pipe_send=closed),
                    lock=threading.Lock(), module=lambda name: wp if name == 'wp' else None)
                backend = types.SimpleNamespace(Process=process_factory, Queue=make_queue,
                                                Lock=ctx.Lock, Semaphore=ctx.Semaphore)
                editor = None
                try:
                    with mock.patch.object(mission_editor, 'multiproc', backend):
                        editor = mission_editor.MissionEditorMain(state, 'SRTM3')
                    self.assertTrue(started.wait(5), 'GUI did not start')
                    editor.set_layout('layout')
                    editor.get_wps_from_module()
                    self.assertTrue(loaded.wait(5), 'GUI did not receive the empty mission and layout')
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
                    for q in queues:
                        q.close()
                        q.join_thread()
                        q._reader.close()
                        q._writer.close()


if __name__ == '__main__':
    unittest.main()
