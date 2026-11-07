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

    def test_adjusting_offset_keeps_paused_image_and_refreshes_projection(self):
        from MAVProxy.modules.lib.mavvidplay import Player
        state = self.thermal_state()
        state.number, state.frame = 7, object()
        original_pixels, original_frame = state.thermal_pixels, state.frame
        state.telemetry_sample = SimpleNamespace(record={'thermal': {'temperature_scale_k': 99}})
        state.telemetry_offset = 0
        state.offset_control = SimpleNamespace(SetRange=mock.Mock(), SetValue=mock.Mock())
        state.update_hover = mock.Mock()
        state.update_telemetry = mock.Mock()
        state.panel = SimpleNamespace(Refresh=mock.Mock())
        state.reader = SimpleNamespace(request=mock.Mock())
        Player.set_telemetry_offset(state, -11.5)
        self.assertEqual(state.telemetry_offset, -11.5)
        self.assertTrue(state.map_dirty)
        self.assertEqual(state.number, 7)
        self.assertIs(state.frame, original_frame)
        self.assertIs(state.thermal_pixels, original_pixels)
        state.reader.request.assert_not_called()
        state.update_hover.assert_called_once()
        self.assertIn('Pixel (1, 0): 1.85 °C', state.thermal_lines())
        with self.assertRaises(ValueError):
            Player.set_telemetry_offset(state, float('nan'))

    def test_map_and_clicked_marker_use_shifted_telemetry(self):
        from MAVProxy.modules.lib.mavvidplay import Player
        from MAVProxy.modules.lib.video_telemetry import Sample
        original, shifted = {'image': True}, {'aligned': True}
        projection = SimpleNamespace(footprint=mock.Mock(return_value=None),
                                     pixel=mock.Mock(return_value=(-35., 149., 550)))
        display = SimpleNamespace(is_alive=lambda: True, add_object=mock.Mock(),
                                  remove_object=mock.Mock())
        state = SimpleNamespace(sample=Sample(3, .1, original),
            telemetry_sample=Sample(4, .1, shifted), map=display, projection=projection,
            update_hover=mock.Mock(), follow=SimpleNamespace(GetValue=lambda: False),
            markers=[], SetStatusText=mock.Mock(), telemetry_offset=1, telemetry_basis='UTC')
        Player.update_map(state)
        projection.footprint.assert_called_once_with(shifted)
        with mock.patch('builtins.print'):
            Player.place_marker(state, (10, 20))
        projection.pixel.assert_called_once_with(shifted, 10, 20)
        self.assertEqual(state.markers[0][1], 3)  # original picture time
        self.assertIs(state.sample.record, original)

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
        try:
            import av
        except ImportError:
            self.skipTest('PyAV required for thermal playback')
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

    def test_seek_waits_for_background_index_then_resumes(self):
        from MAVProxy.modules.lib.mavvidplay import Player
        requested, messages = [], []
        state = SimpleNamespace(
            index=SimpleNamespace(times=[0, .1], fps=10, duration=1,
                                  indexing=True, index_progress=10, index_error=None, has_fov=lambda: True),
            projection=SimpleNamespace(fov=None),
            generation=0, map_dirty=False, playing=False,
            slider=SimpleNamespace(SetMax=lambda value: None),
            panel=SimpleNamespace(Refresh=lambda erase: None),
            SetStatusText=messages.append, request=requested.append,
            reader=SimpleNamespace(results=queue.Queue()),
            last_hover_update=time.monotonic()+100,
            map=SimpleNamespace(is_alive=lambda: False))
        state.seek = lambda number: Player.seek(state, number)
        state.seek(8)
        self.assertEqual(state.pending_seek, 8)
        self.assertEqual(requested, [])
        Player.tick(state)
        self.assertEqual(requested, [])
        state.index.times = [n/10 for n in range(10)]
        state.index.indexing = False
        Player.tick(state)
        self.assertIsNone(state.pending_seek)
        self.assertEqual(requested, [8])
        self.assertEqual(state.anchor_media, .8)

    def test_completed_background_errors_are_shown_on_first_tick(self):
        from MAVProxy.modules.lib.mavvidplay import Player
        for error, fov, expected in (
                ('No valid MISB telemetry', True, 'Telemetry index failed'),
                (None, False, 'No recorded FOV')):
            with self.subTest(error=error):
                messages, playing = [], []
                state = SimpleNamespace(
                    index=SimpleNamespace(times=[0], duration=.1, fps=10,
                        indexing=False, index_error=error, has_fov=lambda: fov),
                    _was_indexing=True, playing=False,
                    projection=SimpleNamespace(fov=None),
                    set_playing=playing.append, SetStatusText=messages.append,
                    slider=SimpleNamespace(SetMax=lambda value: None),
                    panel=SimpleNamespace(Refresh=lambda erase: None),
                    reader=SimpleNamespace(results=queue.Queue()),
                    last_hover_update=time.monotonic()+100,
                    map=SimpleNamespace(is_alive=lambda: False))
                Player.tick(state)
                self.assertEqual(playing, [False])
                self.assertIn(expected, messages[-1])
                self.assertFalse(state._was_indexing)
                count = len(messages)
                Player.tick(state)
                self.assertEqual(len(messages), count)

    def test_cli_distinguishes_cancelled_scan_from_completed_missing_fov(self):
        import importlib.util
        from pathlib import Path
        import sys
        from unittest import mock
        from MAVProxy.modules.lib import mavvidplay, video_telemetry
        path = Path(__file__).resolve().parents[1]/'MAVProxy/tools/mavvidplay.py'
        spec = importlib.util.spec_from_file_location('mavvidplay_cli_test', path)
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        for progress, override, expected in ((50, False, 0), (100, False, 1), (100, True, 0)):
            with self.subTest(progress=progress, override=override):
                index = SimpleNamespace(indexing=True, index_progress=0, index_error=None,
                    samples=[], telemetry_count=0, duration=2, misb_count=0, siyi_count=0,
                    warnings=[], width=160, height=120, has_fov=lambda: False, close=mock.Mock())
                def finish():
                    # Closing Player joins/cancels its worker: indexing is
                    # false after both early cancellation and normal EOF.
                    index.indexing = False
                    index.index_progress = progress
                app = SimpleNamespace(MainLoop=finish)
                display = SimpleNamespace(is_alive=lambda: False)
                argv = ['mavvidplay.py', 'test.ts', '--offline', '--ground-alt', '0']
                if override:
                    argv += ['--fov', '60']
                with mock.patch.object(sys, 'argv', argv), \
                        mock.patch.object(video_telemetry, 'VideoIndex', return_value=index), \
                        mock.patch.object(mavvidplay, 'Player', return_value=SimpleNamespace(Show=lambda: None)), \
                        mock.patch.object(mavvidplay.wx, 'App', return_value=app), \
                        mock.patch.object(mavvidplay.mp_slipmap, 'MPSlipMap', return_value=display):
                    self.assertEqual(cli.main(), expected)
                index.close.assert_called_once()

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
