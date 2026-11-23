"""MPImage mouse events forwarded to the parent, and OSD drawing"""
from pathlib import Path
import os
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class MouseForwardingTests(unittest.TestCase):
    def forwarded(self, event_type, left_down=False):
        '''what MPImagePanel.on_event sends the parent for one mouse event'''
        from MAVProxy.modules.lib import mp_image
        from MAVProxy.modules.lib.wx_loader import wx
        sent = []
        panel = SimpleNamespace(
            state=SimpleNamespace(mouse_movement_events=False,
                                  out_queue=SimpleNamespace(put=sent.append)),
            on_mouse_event=lambda event: None,
            image_coordinates=lambda point: point, raw_img=None)
        event = wx.MouseEvent(event_type)
        event.SetPosition(wx.Point(10, 20))
        event.SetLeftDown(left_down)
        mp_image.MPImagePanel.on_event(panel, event)
        return sent

    def test_press_and_release_reach_the_parent(self):
        from MAVProxy.modules.lib.wx_loader import wx
        (pressed,) = self.forwarded(wx.wxEVT_LEFT_DOWN, left_down=True)
        self.assertTrue(pressed.leftPressed)
        self.assertFalse(pressed.leftReleased)
        (released,) = self.forwarded(wx.wxEVT_LEFT_UP)
        self.assertTrue(released.leftReleased)
        self.assertFalse(released.leftPressed)
        self.assertEqual((released.X, released.Y), (10, 20))
        # plain movement is still not sent
        self.assertEqual(self.forwarded(wx.wxEVT_MOTION), [])


class OSDTextTests(unittest.TestCase):
    def test_queued_image_keeps_text_out_of_source_pixels(self):
        import numpy as np
        from unittest import mock
        from MAVProxy.modules.lib import mp_image
        raw = np.full((512,640,3), 127, dtype=np.uint8).tobytes()
        osd = mp_image.MPImageOSD_Text('thermal', 'Min: 22.41 C\nPixel (0, 0): 24.54 C')
        panel = SimpleNamespace(state=SimpleNamespace(auto_size=False, auto_fit=False),
                                colormap=None, osd_elements={'thermal':osd})
        panel.draw_osd = lambda data: mp_image.MPImagePanel.draw_osd(panel, data)
        image = mock.Mock()
        with mock.patch.object(mp_image.wx, 'EmptyImage', return_value=image):
            mp_image.MPImagePanel.set_image_data(panel, raw, 640, 512)
        drawn = image.SetData.call_args.args[0]
        self.assertEqual(panel.raw_img, raw)
        self.assertTrue(np.all(drawn == 127))
        self.assertTrue(panel.need_redraw)


class OSDRectTests(unittest.TestCase):
    def test_rect_is_drawn_in_image_fractions(self):
        import numpy as np
        from MAVProxy.modules.lib.mp_image import MPImageOSD_Rect
        data = np.zeros((100, 200, 3), np.uint8)
        data = MPImageOSD_Rect('r', (0.25, 0.5), (0.75, 0.9), (255, 0, 0)).draw(data)
        self.assertEqual(tuple(data[50, 50]), (255, 0, 0))
        self.assertEqual(tuple(data[90, 150]), (255, 0, 0))
        self.assertEqual(tuple(data[70, 100]), (0, 0, 0))
        self.assertEqual(tuple(data[10, 10]), (0, 0, 0))


@unittest.skipUnless(os.environ.get('MAVPROXY_GUI_TEST') == '1', 'requires a GUI display')
class OSDDisplayTests(unittest.TestCase):
    def test_text_size_and_replacement_in_resized_viewer(self):
        import numpy as np
        from unittest import mock
        from MAVProxy.modules.lib import mp_image
        from MAVProxy.modules.lib.wx_loader import wx
        app = wx.App(False)
        with mock.patch.object(mp_image.multiproc, 'Process'):
            state = mp_image.MPImage(width=640, height=512, auto_fit=True)
        frame = mp_image.MPImageFrame(state)
        frame.ShowWithoutActivating()
        raw = np.full((512, 640, 3), 127, dtype=np.uint8).tobytes()
        heights = []
        try:
            for size in ((640, 512), (270, 216)):
                frame.SetClientSize(size)
                app.Yield()
                panel = state.panel
                panel.handle_osd(mp_image.MPImageOSD_Text(
                    'thermal', 'Min: 22.41 C\nPixel: 25.55 C', (0, 0, 255)))
                panel.set_image_data(raw, 640, 512)
                panel.redraw()
                rendered = panel.imagePanel._bmp.ConvertToImage()
                pixels = np.frombuffer(rendered.GetData(), np.uint8).reshape(
                    rendered.GetHeight(), rendered.GetWidth(), 3)
                rows = np.where(np.any(pixels != 127, axis=(1, 2)))[0]
                heights.append(len(rows))
                self.assertGreater(heights[-1], 40)
                self.assertTrue(np.any(np.all(pixels == (0, 0, 255), axis=2)))
                self.assertEqual(bytes(panel.img.GetData()), raw)
                # Replacing a longer readout must erase every old glyph.
                replacement = mp_image.MPImageOSD_Text(
                    'thermal', 'Min: 1 C\nPixel: 2 C', (0, 0, 255))
                panel.handle_osd(replacement)
                self.assertTrue(panel.need_redraw)
                panel.redraw()
                rendered = panel.imagePanel._bmp.ConvertToImage()
                actual = np.frombuffer(rendered.GetData(), np.uint8).reshape(
                    rendered.GetHeight(), rendered.GetWidth(), 3)
                expected = replacement.draw(np.full_like(actual, 127))
                np.testing.assert_array_equal(actual, expected)
            self.assertEqual(heights[0], heights[1])
        finally:
            frame.Destroy()
            app.Yield()
            state.in_queue.close()
            state.out_queue.close()


if __name__ == '__main__':
    unittest.main()
