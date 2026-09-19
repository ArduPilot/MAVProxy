"""Video capture lifecycle tests without starting a wx window."""
from types import SimpleNamespace
import unittest
from unittest import mock

from MAVProxy.modules.lib import mp_image


class StopCapture(BaseException):
    """End an otherwise persistent live capture in tests."""


class VideoCaptureTest(unittest.TestCase):
    def setUp(self):
        self.panel = SimpleNamespace(seek_percentage=None, seek_frame=None,
                                     display_video_frame=mock.Mock())

    def capture(self, opened=True, frames=()):
        capture = mock.Mock()
        capture.isOpened.return_value = opened
        capture.read.side_effect = [(True, frame) for frame in frames] + [(False, None)]
        capture.get.return_value = 1
        return capture

    @mock.patch.object(mp_image.time, 'sleep')
    @mock.patch.object(mp_image.cv2, 'VideoCapture')
    def test_live_stream_recovers_from_open_failure_and_disconnect(self, factory, sleep):
        frames = [object(), object()]
        captures = [self.capture(False), self.capture(frames=frames[:1]),
                    self.capture(frames=frames[1:])]
        factory.side_effect = captures
        sleep.side_effect = [None, None, StopCapture()]
        with self.assertRaises(StopCapture):
            mp_image.MPImagePanel.video_thread(self.panel, 'rtsp://camera/video1', 0, True)
        self.assertEqual([call.args[0] for call in self.panel.display_video_frame.call_args_list], frames)
        for capture in captures:
            capture.release.assert_called_once()
        self.assertIsNone(self.panel.vcap)

    @mock.patch.object(mp_image.time, 'sleep', side_effect=StopCapture)
    @mock.patch.object(mp_image.cv2, 'VideoCapture')
    def test_read_exception_releases_capture_before_retry(self, factory, sleep):
        capture = self.capture()
        capture.read.side_effect = RuntimeError('camera disconnected')
        factory.return_value = capture
        with self.assertRaises(StopCapture):
            mp_image.MPImagePanel.video_thread(self.panel, 'rtsp://camera/video1', 0, True)
        capture.release.assert_called_once()
        sleep.assert_called_once_with(1)

    @mock.patch.object(mp_image.time, 'sleep')
    @mock.patch.object(mp_image.cv2, 'VideoCapture')
    def test_file_stops_at_eof_and_preserves_seeking(self, factory, sleep):
        capture = self.capture(frames=[object()])
        factory.return_value = capture
        self.panel.seek_frame = 10
        mp_image.MPImagePanel.video_thread(self.panel, 'recording.mp4', 0)
        factory.assert_called_once_with('recording.mp4', 0)
        capture.set.assert_called_once_with(mp_image.cv2.CAP_PROP_POS_FRAMES, 10)
        capture.release.assert_called_once()
        sleep.assert_not_called()


if __name__ == '__main__':
    unittest.main()
