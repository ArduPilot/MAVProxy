"""Playback state regressions that do not require opening a GUI."""
import queue
import time
from types import SimpleNamespace
import unittest


class PlaybackTests(unittest.TestCase):
    def test_restart_waits_for_decoder_before_end_detection(self):
        from MAVProxy.modules.lib.mavvidplay import Player
        requested, playing = [], []
        state = SimpleNamespace(
            reader=SimpleNamespace(results=queue.Queue()),
            playing=True, number=19,
            index=SimpleNamespace(samples=[None] * 20, frame_at=lambda seconds: int(seconds * 10)),
            media_time=lambda: 0,
            request=requested.append, set_playing=playing.append,
            last_hover_update=time.monotonic() + 100,
            map=SimpleNamespace(is_alive=lambda: False))
        # The old last frame is still displayed after restarting; the decoder
        # has not returned frame zero yet. Playback must keep running.
        Player.tick(state)
        self.assertEqual(requested, [0])
        self.assertEqual(playing, [])
        # Reaching the end on the actual playback clock still pauses playback.
        state.media_time = lambda: 1.9
        Player.tick(state)
        self.assertEqual(playing, [False])


if __name__ == '__main__':
    unittest.main()
