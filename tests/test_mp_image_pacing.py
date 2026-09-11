"""Opt-in GUI regression test: MAVPROXY_GUI_TEST=1 python3 -m unittest discover -s tests -p test_mp_image_pacing.py."""
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(os.environ.get('MAVPROXY_GUI_TEST') == '1', 'requires a GUI display')
class ImagePacingTests(unittest.TestCase):
    def test_video_paints_at_source_rate_and_reports_layout(self):
        import numpy as np
        from MAVProxy.modules.lib import mp_image, mp_widgets, win_layout
        from MAVProxy.modules.lib.wx_loader import wx
        app = wx.App(False)
        # Exercise the real frame/panel in this process, with the normal queues.
        with mock.patch.object(mp_image.multiproc, 'Process'):
            state = mp_image.MPImage(title='MPImage pacing test', width=640,
                                     height=360, auto_fit=True, fps=60)
        painted = []
        original = mp_widgets.ImagePanel.on_paint
        started = time.monotonic()
        halt = threading.Event()

        def on_paint(panel, event):
            original(panel, event)
            if state.panel.img is not None and time.monotonic() - started > 1:
                value = state.panel.img.GetRed(0, 0)
                if not painted or painted[-1] != value:
                    painted.append(value)

        def produce():
            for index in range(120):
                if halt.wait(max(0, started + index * .05 - time.monotonic())):
                    break
                state.set_image(np.full((360, 640, 3), index, dtype=np.uint8))

        with mock.patch.object(mp_widgets.ImagePanel, 'on_paint', on_paint):
            frame = mp_image.MPImageFrame(state)
            frame.SetSize((640, 420))
            frame.ShowWithoutActivating()
            producer = threading.Thread(target=produce)
            producer.start()
            end_timer = wx.CallLater(5900, app.ExitMainLoop)
            try:
                app.MainLoop()
            finally:
                halt.set()
                producer.join(timeout=2)
                end_timer.Stop()
                frame.Destroy()
                app.Yield()
        events = []
        while not state.out_queue.empty():
            events.append(state.out_queue.get())
        layouts = [e for e in events if isinstance(e, win_layout.WinLayout)]
        self.assertGreaterEqual(len(painted), 85, 'GUI did not present the 20 Hz image updates')
        self.assertGreaterEqual(len(layouts), 4)
        self.assertLessEqual(len(layouts), 7)
        print('Painted %u distinct frames after warmup; %u periodic layout reports' % (len(painted), len(layouts)))
        state.in_queue.close()
        state.out_queue.close()
