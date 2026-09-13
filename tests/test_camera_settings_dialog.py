"""Display-free tests for camera dialog message delivery and shutdown."""
import queue
import threading
import unittest
from unittest import mock

from MAVProxy.modules.mavproxy_camera.settings_dialog import CameraSettingsDialog, _DialogChannel


class DialogChannelTest(unittest.TestCase):
    def setUp(self):
        self.updates = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.pipe = mock.Mock()
        self.channel = _DialogChannel(self.pipe, self.updates, self.closed)

    def test_poll_retains_snapshot_until_received(self):
        self.assertFalse(self.channel.poll())
        snapshot = {'status': 'Ready', 'rows': []}
        self.updates.put_nowait(snapshot)
        self.assertTrue(self.channel.poll())
        self.assertTrue(self.channel.poll())
        self.assertEqual(self.channel.recv(), snapshot)
        self.assertFalse(self.channel.poll())
        self.channel.send(('set', 'GAIN', 2))
        self.pipe.send.assert_called_once_with(('set', 'GAIN', 2))

    def test_close_preempts_pending_snapshot(self):
        self.updates.put_nowait({'rows': []})
        self.channel.poll()
        self.closed.set()
        self.assertTrue(self.channel.poll())
        self.assertEqual(self.channel.recv(), {'close': True})

    def test_full_queue_does_not_block_mavlink_thread(self):
        dialog = CameraSettingsDialog.__new__(CameraSettingsDialog)
        dialog.updates = self.updates
        self.assertTrue(dialog.send({'status': 'one'}))
        self.assertFalse(dialog.send({'status': 'two'}))
        self.assertEqual(self.updates.get_nowait(), {'status': 'one'})
        self.assertTrue(dialog.send({'status': 'two'}))

    def test_events_survive_child_pipe_closing(self):
        dialog = CameraSettingsDialog.__new__(CameraSettingsDialog)
        dialog.pipe = self.pipe
        self.pipe.poll.return_value = True
        self.pipe.recv.side_effect = [('refresh',), EOFError()]
        self.assertEqual(list(dialog.events()), [('refresh',)])

    def test_close_forces_stuck_child_and_releases_resources(self):
        dialog = CameraSettingsDialog.__new__(CameraSettingsDialog)
        dialog.close_event = self.closed
        dialog.child = mock.Mock()
        dialog.pipe = self.pipe
        dialog.updates = mock.Mock()
        dialog.close()
        self.assertTrue(self.closed.is_set())
        dialog.child.terminate.assert_called_once()
        self.assertEqual(dialog.child.join.call_count, 2)
        self.pipe.close.assert_called_once()
        dialog.updates.cancel_join_thread.assert_called_once()
        dialog.updates.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
