"""Follow must work when busy redraw timers prevent wx idle callbacks."""
from pathlib import Path
import queue
import sys
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from MAVProxy.modules.mavproxy_map import mp_slipmap_ui as ui
from MAVProxy.modules.mavproxy_map.mp_slipmap_util import (
    SlipFollow, SlipIcon, SlipPolygon, SlipPosition)


class MapFollowTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = SlipIcon('vehicle', (50, 50), np.zeros((4, 4, 3), dtype=np.uint8),
                                layer=3, follow=True)
        self.state = SimpleNamespace(
            layers={self.vehicle.layer: {'vehicle': self.vehicle}}, follow=True,
            width=100, height=100, need_redraw=False, timelim_pipe=None,
            close_window=threading.Semaphore(0), object_queue=queue.Queue(),
            event_queue=queue.Queue())
        self.frame = SimpleNamespace(state=self.state, last_layout_send=time.time(),
                                     legend_checkbox_menuitem_added=False)
        self.state.frame = self.frame
        for name in ('find_object', 'add_object', 'follow', 'process_pending'):
            if hasattr(ui.MPSlipMapFrame, name):
                setattr(self.frame, name, MethodType(getattr(ui.MPSlipMapFrame, name), self.frame))
        self.panel = SimpleNamespace(state=self.state, pixmapper=lambda p: p,
                                     re_center=Mock(), redraw_map=Mock())
        self.state.panel = self.panel

    def redraw(self):
        # Deliberately never deliver EVT_IDLE, as on an overloaded map.
        ui.MPSlipMapPanel.on_redraw_timer(self.panel, None)

    def test_positions_and_follow_with_continuous_coverage_redraw(self):
        for index, position in enumerate(((5, 5), (95, 95), (5, 95))):
            self.state.object_queue.put(SlipPolygon(
                'survey_%u' % index, [(0, 0), (0, 10), (10, 10), (0, 0)],
                layer='Survey coverage', colour=(0, 255, 255), linewidth=1))
            self.state.object_queue.put(SlipPosition('vehicle', position))
            self.redraw()
            self.assertEqual(self.vehicle.latlon, position)
            self.panel.re_center.assert_called_with(50, 50, *position)
        self.assertEqual(len(self.state.layers['Survey coverage']), 3)
        self.assertEqual(self.panel.redraw_map.call_count, 3)

    def test_follow_toggle_is_consumed_without_idle(self):
        self.state.object_queue.put(SlipFollow(False))
        self.state.object_queue.put(SlipPosition('vehicle', (5, 5)))
        self.redraw()
        self.assertEqual(self.vehicle.latlon, (5, 5))
        self.panel.re_center.assert_not_called()
        self.state.object_queue.put(SlipFollow(True))
        self.state.object_queue.put(SlipPosition('vehicle', (95, 95)))
        self.redraw()
        self.panel.re_center.assert_called_once_with(50, 50, 95, 95)

    def test_queue_batch_yields_to_ui_events(self):
        for _ in range(10):
            self.state.object_queue.put(SlipPosition('vehicle', (5, 5)))
        # Simulate costly queued work. Do not consume the whole queue before
        # giving wx a chance to handle the Ctrl-F/menu event.
        with patch.object(ui.time, 'monotonic', side_effect=(0, .001, .025)):
            self.redraw()
        self.assertEqual(self.state.object_queue.qsize(), 9)
        self.panel.redraw_map.assert_called_once()


if __name__ == '__main__':
    unittest.main()
