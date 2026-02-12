"""
Override tracker in MPImage
"""

import cv2
import dlib
import threading
import time

from MAVProxy.modules.lib import wx_processguard
from MAVProxy.modules.lib.wx_loader import wx

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import win_layout

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageFrame
from MAVProxy.modules.lib.mp_image import MPImagePanel


# Events
class MPImageTrackedRectangle:
    def __init__(self, top_left_x, top_left_y, bot_right_x, bot_right_y):
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
        self.bot_right_x = bot_right_x
        self.bot_right_y = bot_right_y


class TrackerImage(MPImage):
    """An MPImage class that allows the tracker type to be overridden"""

    def __init__(
        self,
        title="MPImage",
        width=512,
        height=512,
        can_zoom=False,
        can_drag=False,
        mouse_events=False,
        mouse_movement_events=False,
        key_events=False,
        auto_size=False,
        report_size_changes=False,
        daemon=False,
        auto_fit=False,
        fps=10,
    ):
        super(TrackerImage, self).__init__(
            title,
            width,
            height,
            can_zoom,
            can_drag,
            mouse_events,
            mouse_movement_events,
            key_events,
            auto_size,
            report_size_changes,
            daemon,
            auto_fit,
            fps,
        )

    def child_task(self):
        """Child process - this holds all the GUI elements"""
        mp_util.child_close_fds()

        state = self

        self.app = wx.App(False)
        self.app.frame = TrackerImageFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()

    def set_tracked_rectangle(self, top_left_x, top_left_y, bot_right_x, bot_right_y):
        """Set the tracked rectangle (normalised coords)"""
        self.in_queue.put(
            MPImageTrackedRectangle(top_left_x, top_left_y, bot_right_x, bot_right_y)
        )


class TrackerImageFrame(wx.Frame):
    """Main frame for an image with object tracking"""

    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        self.state = state
        state.frame = self
        self.last_layout_send = time.time()
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        state.panel = TrackerImagePanel(self, state)
        self.sizer.Add(state.panel, 1, wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_SIZE, state.panel.on_size)

    def on_idle(self, event):
        """Prevent the main loop spinning too fast"""
        state = self.state
        now = time.time()
        if now - self.last_layout_send > 1:
            self.last_layout_send = now
            state.out_queue.put(win_layout.get_wx_window_layout(self))
        time.sleep(0.1)


class TrackerImagePanel(MPImagePanel):
    """An MPImagePanel that allows the tracker type to be overridden"""

    def __init__(self, parent, state):
        super(TrackerImagePanel, self).__init__(parent, state)

    def start_tracker(self, box):
        """Start a tracker on an object identified by a box (override)"""
        if self.raw_img is None:
            return
        self.tracker = None
        # TODO: accept tracker name as option
        # tracker_name = "dlib_correlation"
        # tracker_name = "cv2_cstr"
        tracker_name = "mavlink"
        tracker = create_tracker(tracker_name, self.state)
        tracker.start_track(self.raw_img, box)
        self.tracker = tracker

    def process_event(self, obj):
        """Process a single event (override)"""
        super().process_event(obj)

        # handle events defined for this class
        if isinstance(obj, MPImageTrackedRectangle):
            if self.tracker is not None:
                height, width, _ = self.raw_img.shape
                self.tracker.set_position(
                    TrackerPos(
                        obj.top_left_x * width,
                        obj.bot_right_x * width,
                        obj.top_left_y * height,
                        obj.bot_right_y * height,
                    )
                )


class TrackerPos:
    """Rectangle output by a tracker [(left, top), (right, bottom)]"""

    def __init__(self, left, right, top, bottom):
        self._left = left
        self._right = right
        self._top = top
        self._bottom = bottom

    def left(self):
        """Rectangle left coordinate (x1)"""
        return self._left

    def right(self):
        """Rectangle right coordinate (x2)"""
        return self._right

    def top(self):
        """Rectangle top coordinate (y1)"""
        return self._top

    def bottom(self):
        """Rectangle bottom coordinate (y2)"""
        return self._bottom


class Tracker:
    """Interface for trackers used by the MPImage class"""

    def __init__(self, state):
        self.state = state

    def start_track(self, raw_image, box):
        """Start the tracker"""
        pass

    def update(self, frame):
        """Update the tracker"""
        pass

    def get_position(self):
        """Return a rectangle at the position of the tracked object"""
        return TrackerPos(0, 0, 0, 0)

    def set_position(self, tracker_pos):
        """Set the tracker position"""
        pass


class TrackerDlibCorrelation(Tracker):
    """Wrapper for the dlib correlation tracker"""

    def __init__(self, state):
        super().__init__(state)
        self._tracker = dlib.correlation_tracker()

    def start_track(self, raw_image, box):
        maxx = raw_image.shape[1] - 1
        maxy = raw_image.shape[0] - 1
        rect = dlib.rectangle(
            max(int(box.x - box.width / 2), 0),
            max(int(box.y - box.height / 2), 0),
            min(int(box.x + box.width / 2), maxx),
            min(int(box.y + box.height / 2), maxy),
        )
        self._tracker.start_track(raw_image, rect)

    def update(self, frame):
        self._tracker.update(frame)

    def get_position(self):
        pos = self._tracker.get_position()
        tracker_pos = TrackerPos(pos.left(), pos.right(), pos.top(), pos.bottom())
        return tracker_pos


class TrackerCSTR(Tracker):
    """
    Wrapper for cv2.legacy.TrackerCSRT
    """

    def __init__(self, state):
        super().__init__(state)
        self._tracker = cv2.legacy.TrackerCSRT_create()
        # TODO: check if None would be better (do callers handle this?)
        self._tracker_pos = TrackerPos(0, 0, 0, 0)

    def start_track(self, raw_image, box):
        self._tracker = cv2.legacy.TrackerCSRT_create()
        x = max(int(box.x), 0)
        y = max(int(box.y), 0)
        w = min(int(box.width), max(raw_image.shape[1] - x, 1))
        h = min(int(box.height), max(raw_image.shape[0] - y, 1))
        rect = [x, y, w, h]
        self._tracker.init(raw_image, rect)

    def update(self, frame):
        success, box = self._tracker.update(frame)
        if success:
            (x, y, w, h) = [v for v in box]
            self._tracker_pos = TrackerPos(x, x + w, y, y + h)

    def get_position(self):
        return self._tracker_pos


class TrackerMAVLink(Tracker):
    """
    MAVLink tracker (i.e. runs on a MAVLink gimbal or companion computer)
    """

    def __init__(self, state):
        super(TrackerMAVLink, self).__init__(state)
        self.event_queue = state.out_queue
        self._tracker_pos = TrackerPos(0, 0, 0, 0)

    def start_track(self, raw_image, box):
        x = max(int(box.x), 0)
        y = max(int(box.y), 0)
        w = min(int(box.width), max(raw_image.shape[1] - x, 1))
        h = min(int(box.height), max(raw_image.shape[0] - y, 1))

        # normalised rectangle
        top_left_x = x / w
        top_left_y = y / h
        bottom_right_x = (x + box.width) / w
        bottom_right_y = (y + box.height) / h
        self._tracker_pos = TrackerPos(x, x + w, y, y + h)

    def update(self, frame):
        pass

    def get_position(self):
        return self._tracker_pos

    def set_position(self, tracker_pos):
        self._tracker_pos = tracker_pos
        # NOTE: leave for debugging
        # print(
        #     f"tracker_pos: x: {self._tracker_pos.left():.2f}, "
        #     f"y: {self._tracker_pos.top():.2f}, "
        #     f"w: {(self._tracker_pos.right() - self._tracker_pos.left()):.2f}, "
        #     f"h: {(self._tracker_pos.bottom() - self._tracker_pos.top()):.2f}"
        # )


def create_tracker(name, state):
    """
    Factory method to create a tracker.
    """
    if name == "dlib_correlation":
        return TrackerDlibCorrelation(state)
    elif name == "cv2_cstr":
        return TrackerCSTR(state)
    elif name == "mavlink":
        return TrackerMAVLink(state)
    else:
        return TrackerDlibCorrelation(state)
