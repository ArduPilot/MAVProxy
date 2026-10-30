"""Playback state regressions that do not require opening a GUI."""
import queue
import time
from types import SimpleNamespace
import unittest
from unittest import mock


class PlaybackTests(unittest.TestCase):
    def thermal_state(self):
        import numpy as np
        from MAVProxy.modules.lib.mavvidplay import Player
        pixels = np.array([[17536, 17600], [18000, 18560]], dtype=np.uint16)
        state = SimpleNamespace(index=SimpleNamespace(thermal=True), hover=(1.8, 0.2),
            thermal_pixels=pixels, sample=SimpleNamespace(record={
                'thermal': dict(temperature_scale_k=1/64, temperature_offset_k=0,
                                minimum_c=.85, maximum_c=16.85),
                'named_values': {'TMIN': dict(value=655.33, valid=False),
                                 'TMAX': dict(value=21.5, valid=True, age_ms=250)}}))
        state.thermal_lines = lambda: Player.thermal_lines(state)
        return state


    def test_hover_uses_raw_pixel_scale_and_named_extrema(self):
        state = self.thermal_state()
        lines = state.thermal_lines()
        self.assertIn('Pixel (1, 0): 1.85 °C', lines)
        self.assertIn('TMIN (tlog): 655.33 °C [invalid]', lines)
        self.assertIn('TMAX (tlog): 21.50 °C (0.25 s old)', lines)
        state.sample.record['thermal']['temperature_offset_k'] = 3
        self.assertIn('Pixel (1, 0): 4.85 °C', state.thermal_lines())
        for point in (None, (-.1, 0), (2, 0), (0, 2)):
            state.hover = point
            self.assertIn('Pixel temperature: --', state.thermal_lines())
        state.hover = (1, 0)
        state.sample.record['thermal']['temperature_scale_k'] = None
        self.assertIn('Pixel temperature: --', state.thermal_lines())
        state.sample.record = None
        self.assertIn('TMAX (tlog): --', state.thermal_lines())
        state.index.thermal = False
        self.assertEqual(state.thermal_lines(), [])


    def test_hover_temperature_updates_without_map_or_ground_pose(self):
        from MAVProxy.modules.lib.mavvidplay import Player, wx
        state = self.thermal_state()
        shown = []
        state.IsShown = lambda: True
        state.osd = lambda: []
        state._telemetry_text = None
        state.telemetry_text = SimpleNamespace(ChangeValue=shown.append)
        state.update_telemetry = lambda: Player.update_telemetry(state)
        state.map = SimpleNamespace(is_alive=lambda: False)
        state.panel = SimpleNamespace(ScreenToClient=lambda point: point, pixel=lambda point: (1, 0))
        with mock.patch.object(wx, 'GetMousePosition', return_value=(1, 0)), \
                mock.patch.object(wx, 'FindWindowAtPoint', return_value=state.panel):
            Player.update_hover(state)
            self.assertIn('Pixel (1, 0): 1.85 °C', shown[-1])
            # The mouse is stationary while playback changes the raw frame.
            state.thermal_pixels = state.thermal_pixels + 64
            Player.update_hover(state)
            self.assertIn('Pixel (1, 0): 2.85 °C', shown[-1])
        with mock.patch.object(wx, 'GetMousePosition', return_value=(1, 0)), \
                mock.patch.object(wx, 'FindWindowAtPoint', return_value=None):
            Player.update_hover(state)
            self.assertIn('Pixel temperature: --', shown[-1])


    def test_thermal_decoder_keeps_raw_pixels_aligned_after_seeking(self):
        from fractions import Fraction
        from pathlib import Path
        import tempfile
        import numpy as np
        import av
        from MAVProxy.modules.lib.mavvidplay import FrameReader, ThermalFrame
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'thermal.mkv')
            pixels = [np.arange(16*12, dtype=np.uint16).reshape(12, 16) + n*1000 for n in range(3)]
            with av.open(path, 'w') as out:
                stream = out.add_stream('ffv1', rate=5)
                stream.width, stream.height, stream.pix_fmt = 16, 12, 'gray16le'
                stream.codec_context.gop_size = 1
                stream.options = {'level': '3'}
                for n, data in enumerate(pixels):
                    frame = av.VideoFrame.from_ndarray(data, format='gray16le')
                    frame.pts, frame.time_base = n*200, Fraction(1, 1000)
                    for packet in stream.encode(frame):
                        out.mux(packet)
                for packet in stream.encode():
                    out.mux(packet)
            reader = FrameReader(SimpleNamespace(filename=path, thermal=True, frame_pts=[0, 200, 400]))
            try:
                for generation, number in enumerate((2, 0, 1, 2)):
                    reader.request(number, generation)
                    actual, gen, frame, error = reader.results.get(timeout=5)
                    self.assertIsNone(error)
                    self.assertEqual((actual, gen), (number, generation))
                    self.assertIsInstance(frame, ThermalFrame)
                    np.testing.assert_array_equal(frame.pixels, pixels[number])
                    self.assertEqual(frame.image.shape, (12, 16, 3))
            finally:
                reader.stop()


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
