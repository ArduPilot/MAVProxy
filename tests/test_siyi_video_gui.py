"""Opt-in integration check with a vendor recording and its MAVLink flight log."""
import os
import time
import unittest
from unittest import mock


@unittest.skipUnless(os.environ.get('MAVVIDPLAY_VENDOR_VIDEO') and os.environ.get('MAVVIDPLAY_VENDOR_TLOG'),
                     'set MAVVIDPLAY_VENDOR_VIDEO and MAVVIDPLAY_VENDOR_TLOG; requires a GUI display')
class VendorPlayerTests(unittest.TestCase):
    def test_vendor_osd_seek_and_picking(self):
        from MAVProxy.modules.lib.video_telemetry import VideoIndex, ViewProjection, FlatElevation
        from MAVProxy.modules.lib.siyi_video import FlightLog
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        from MAVProxy.modules.lib.mavvidplay import Player, wx
        index = VideoIndex(os.environ['MAVVIDPLAY_VENDOR_VIDEO'])
        log = FlightLog(os.environ['MAVVIDPLAY_VENDOR_TLOG'])
        self.assertGreater(log.apply(index), 0)
        self.assertEqual(index.siyi_count, len(index.samples))
        projection = ViewProjection(index.width, index.height, 88, FlatElevation(600))
        map_display = mp_slipmap.MPSlipMap(title='SIYI Video Test Map', download=False)
        app = wx.App(False)
        player = Player(index, projection, map_display, paused=True)
        player.Show()

        def wait_until(predicate):
            end = time.monotonic() + 15
            while time.monotonic() < end:
                app.Yield()
                if predicate():
                    return
                time.sleep(.01)
            self.fail('Timed out waiting for vendor playback')

        def center_pixel():
            x, y, width, height = player.panel.rect
            return wx.Point(x + width // 2, y + height // 2)

        try:
            wait_until(lambda: player.number == 0)
            for frame in (len(index.samples) // 2, 2, len(index.samples) - 1, 0):
                player.seek(frame)
                wait_until(lambda: player.number == frame)
                self.assertEqual(player.sample.record['frame_counter'], frame)
                self.assertTrue(any('Ground speed (tlog)' in line for line in player.osd()))
                self.assertTrue(any('Recorded UTC:' in line for line in player.osd()))
                self.assertFalse(any('Map projection needs' in line for line in player.osd()))
            with mock.patch.object(wx, 'GetMousePosition', side_effect=lambda: player.panel.ClientToScreen(center_pixel())), \
                    mock.patch.object(wx, 'FindWindowAtPoint', return_value=player.panel):
                player.clear_hover()
                wait_until(lambda: player.hover_visible)
                hover = player.hover_location
                click = wx.MouseEvent(wx.wxEVT_LEFT_DOWN)
                click.SetPosition(center_pixel())
                player.panel.GetEventHandler().ProcessEvent(click)
                self.assertEqual(len(player.markers), 1)
                self.assertEqual(player.markers[0][3], hover)
        finally:
            player.close()
            app.Yield()
            self.assertFalse(player.reader.is_alive())
            self.assertFalse(map_display.is_alive())
