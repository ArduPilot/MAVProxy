"""MPImage mouse events forwarded to the parent, and OSD drawing"""
from pathlib import Path
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


if __name__ == '__main__':
    unittest.main()
