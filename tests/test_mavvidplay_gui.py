"""Opt-in wx/map integration test: MAVVIDPLAY_GUI_TEST=1 python -m unittest ..."""
import os
import time
import unittest
from unittest import mock

import test_video_telemetry as fixtures


@unittest.skipUnless(os.environ.get('MAVVIDPLAY_GUI_TEST') == '1', 'requires a GUI display')
class PlayerTests(unittest.TestCase):
    def test_playback_picking_and_hover(self):
        from MAVProxy.modules.lib.video_telemetry import VideoIndex, ViewProjection, FlatElevation
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        from MAVProxy.modules.lib.mavvidplay import Player, wx
        fixtures.RecordingTests.setUpClass()
        index = VideoIndex(fixtures.RecordingTests.video)
        projection = ViewProjection(index.width, index.height, None, FlatElevation(550))
        map_display = mp_slipmap.MPSlipMap(title='MAV Video Test Map', download=False,
                                           lat=-35.363261, lon=149.16523)
        objects = {}
        original_add, original_remove = map_display.add_object, map_display.remove_object

        def add(obj):
            objects[obj.key] = obj
            original_add(obj)

        def remove(key):
            objects.pop(key, None)
            original_remove(key)

        map_display.add_object, map_display.remove_object = add, remove
        app = wx.App(False)
        player = Player(index, projection, map_display, paused=True)
        player.SetPosition((30, 30))
        player.Show()
        player.Raise()
        player.panel.SetFocus()

        def wait_until(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                app.Yield()
                if predicate():
                    return
                time.sleep(.01)
            self.fail('Timed out waiting for GUI state')

        try:
            wait_until(lambda: player.number == 0 and 'viewport' in objects)
            self.assertEqual(projection.current_fov, 60)
            def center_pixel():
                left, top, width, height = player.panel.rect
                return wx.Point(left + width // 2, top + height // 2)

            def center_screen():
                return player.panel.ClientToScreen(center_pixel())

            # Keep this test independent of desktop mouse movement while a user
            # works. Events still pass through the actual wx panel handlers.
            player.clear_hover()
            with mock.patch.object(wx, 'GetMousePosition', side_effect=center_screen), \
                    mock.patch.object(wx, 'FindWindowAtPoint', return_value=player.panel):
                wait_until(lambda: 'video-hover' in objects)
                hover = objects['video-hover'].latlon
                self.assertAlmostEqual(hover[0], -35.363261, places=5)
                self.assertAlmostEqual(hover[1], 149.16523, places=5)
                click = wx.MouseEvent(wx.wxEVT_LEFT_DOWN)
                click.SetPosition(center_pixel())
                player.panel.GetEventHandler().ProcessEvent(click)
                wait_until(lambda: len(player.markers) == 1)
                self.assertEqual(objects['M1'].latlon, hover)
            with mock.patch.object(wx, 'FindWindowAtPoint', return_value=player.slider):
                wait_until(lambda: 'video-hover' not in objects)
            self.assertIn('M1', objects)
            player.step(1)
            wait_until(lambda: player.number == 1)
            self.assertFalse(player.playing)
            self.assertIsNone(player.sample.record)
            wait_until(lambda: 'viewport' not in objects)
            self.assertIn('M1', objects)
            player.seek(15)
            wait_until(lambda: player.number == 15 and 'viewport' in objects)
            self.assertEqual(player.sample.record['pts90k'], 123456789)
            player.step(-1)
            wait_until(lambda: player.number == 14)
            player.skip(-10)
            wait_until(lambda: player.number == 0)
            player.rate_choice.SetSelection(4)
            player.change_rate()
            self.assertEqual(player.rate, 4)
            player.toggle()
            wait_until(lambda: player.number == 19 and not player.playing)
            player.toggle()
            wait_until(lambda: player.number is not None and player.number < 19)
            player.set_playing(False)
            player.seek(0)
            wait_until(lambda: player.number == 0 and 'viewport' in objects)
            screenshot = os.environ.get('MAVVIDPLAY_SCREENSHOT')
            if screenshot:
                size = player.GetSize()
                bitmap = wx.Bitmap(size.width, size.height)
                dc = wx.MemoryDC(bitmap)
                dc.Blit(0, 0, size.width, size.height, wx.ScreenDC(), *player.GetPosition())
                dc.SelectObject(wx.NullBitmap)
                bitmap.SaveFile(screenshot, wx.BITMAP_TYPE_PNG)
        finally:
            player.close()
            app.Yield()
            self.assertFalse(player.reader.is_alive())
            self.assertFalse(map_display.is_alive())
            fixtures.RecordingTests.tearDownClass()
