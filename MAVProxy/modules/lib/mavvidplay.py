"""Video/OSD window and synchronized terrain map for mavvidplay.py."""
import math
import queue
import threading
import time

import cv2
import numpy as np
from MAVProxy.modules.lib import wx_processguard  # noqa: F401
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib.video_telemetry import (
    camera_pose, attitude, finite, image_pixel, image_rect, position)
from MAVProxy.modules.mavproxy_map import mp_slipmap


class FrameReader(threading.Thread):
    """Own the decoder in one worker; coalesce seeks without blocking wx."""
    def __init__(self, filename):
        super().__init__(name='video-decoder', daemon=True)
        self.filename = filename
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.results = queue.Queue(maxsize=1)
        self.start()

    def request(self, number, generation):
        with self.condition:
            self.pending = (number, generation)
            self.condition.notify()

    def stop(self):
        with self.condition:
            self.stopping = True
            self.condition.notify()
        self.join(timeout=5)

    def run(self):
        capture = cv2.VideoCapture(self.filename, cv2.CAP_FFMPEG)
        next_frame = 0
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                    if self.stopping:
                        return
                    number, generation = self.pending
                    self.pending = None
                try:
                    if not capture.isOpened():
                        raise ValueError('OpenCV could not open the video')
                    # Decode short forward gaps sequentially; a seek requires GOP
                    # preroll and is considerably slower during fast playback.
                    if number < next_frame or number - next_frame > 30:
                        if not capture.set(cv2.CAP_PROP_POS_FRAMES, number):
                            raise ValueError('Video decoder cannot seek to frame %u' % number)
                        next_frame = number
                    while next_frame < number:
                        if not capture.grab():
                            raise ValueError('Video ended while seeking')
                        next_frame += 1
                    ok, frame = capture.read()
                    if not ok:
                        raise ValueError('Cannot decode frame %u' % number)
                    next_frame += 1
                    result = (number, generation, frame, None)
                except (ValueError, cv2.error) as error:
                    result = (number, generation, None, str(error))
                try:
                    self.results.get_nowait()
                except queue.Empty:
                    pass
                self.results.put(result)
        finally:
            capture.release()


class VideoPanel(wx.Panel):
    def __init__(self, parent):
        super().__init__(parent)
        self.owner = parent
        self.bitmap = None
        self.rect = (0, 0, 1, 1)
        self.SetBackgroundStyle(wx.BG_STYLE_PAINT)
        self.Bind(wx.EVT_PAINT, self.paint)
        self.Bind(wx.EVT_SIZE, self.resize)
        self.Bind(wx.EVT_MOTION, self.motion)
        self.Bind(wx.EVT_LEAVE_WINDOW, self.leave)
        self.Bind(wx.EVT_LEFT_DOWN, self.click)

    def resize(self, event):
        self.update_bitmap()
        self.owner.update_hover()
        event.Skip()

    def update_bitmap(self):
        frame = self.owner.frame
        if frame is None:
            return
        width, height = self.GetClientSize()
        self.rect = image_rect(max(width, 1), max(height, 1), frame.shape[1], frame.shape[0])
        rgb = cv2.cvtColor(cv2.resize(frame, self.rect[2:]), cv2.COLOR_BGR2RGB)
        self.bitmap = wx.Bitmap.FromBuffer(self.rect[2], self.rect[3], rgb)
        self.Refresh(False)

    def paint(self, event):
        dc = wx.AutoBufferedPaintDC(self)
        dc.SetBackground(wx.Brush('BLACK'))
        dc.Clear()
        if self.bitmap is None:
            return
        dc.DrawBitmap(self.bitmap, self.rect[0], self.rect[1])
        dc.SetFont(wx.Font(12, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        dc.SetTextForeground(wx.Colour(255, 255, 255))
        dc.SetTextBackground(wx.Colour(0, 0, 0))
        dc.SetBackgroundMode(wx.SOLID)
        for row, text in enumerate(self.owner.osd()):
            dc.DrawText(text, self.rect[0] + 12, self.rect[1] + 10 + row * 23)

    def pixel(self, point):
        return image_pixel(self.rect, self.owner.index.width, self.owner.index.height, point.x, point.y)

    def motion(self, event):
        self.owner.hover = self.pixel(event.GetPosition())
        self.owner.update_hover()

    def leave(self, event):
        self.owner.hover = None
        self.owner.clear_hover()

    def click(self, event):
        self.SetFocus()
        pixel = self.pixel(event.GetPosition())
        if pixel is not None:
            self.owner.place_marker(pixel)


def angle_text(angles):
    if angles is None:
        return 'R --  P --  Y --'
    return 'R %+6.1f  P %+6.1f  Y %6.1f' % (angles[0], angles[1], angles[2] % 360)


def clock_text(seconds):
    return '%02u:%05.2f' % (int(seconds // 60), seconds % 60)


def circle_image(colour, radius=5):
    image = np.zeros((2 * radius + 5, 2 * radius + 5, 3), dtype=np.uint8)
    cv2.circle(image, (radius + 2, radius + 2), radius, colour, 2, cv2.LINE_AA)
    return image


class Player(wx.Frame):
    def __init__(self, index, projection, map_display, paused=False):
        super().__init__(None, title='MAV Video: ' + index.filename, size=(1000, 740))
        self.index, self.projection, self.map = index, projection, map_display
        self.frame = None
        self.number = None
        self.requested = None
        self.generation = 0
        self.playing = not paused
        self.rate = 1.0
        self.anchor_media, self.anchor_wall = 0.0, time.monotonic()
        self.hover = None
        self.hover_visible = False
        self.hover_location = None
        self.last_hover_update = 0
        self.markers = []
        self.last_map_update = 0
        self.map_dirty = True
        self.closing = False
        self.reader = FrameReader(index.filename)
        self.panel = VideoPanel(self)
        self.slider = wx.Slider(self, minValue=0, maxValue=max(1, len(index.samples) - 1))
        self.play_button = wx.Button(self, label='Pause' if self.playing else 'Play')
        self.play_button.Bind(wx.EVT_BUTTON, self.toggle)
        controls = wx.BoxSizer(wx.HORIZONTAL)
        controls.Add(self.play_button, 0, wx.ALL, 4)
        for label, delta in (('-10s', -10), ('+10s', 10)):
            button = wx.Button(self, label=label)
            button.Bind(wx.EVT_BUTTON, lambda event, d=delta: self.skip(d))
            controls.Add(button, 0, wx.ALL, 4)
        for label, delta in (('Previous frame', -1), ('Next frame', 1)):
            button = wx.Button(self, label=label)
            button.Bind(wx.EVT_BUTTON, lambda event, d=delta: self.step(d))
            controls.Add(button, 0, wx.ALL, 4)
        self.rates = [.25, .5, 1, 2, 4, 8]
        self.rate_choice = wx.Choice(self, choices=['%gx' % rate for rate in self.rates])
        self.rate_choice.SetSelection(2)
        self.rate_choice.Bind(wx.EVT_CHOICE, self.change_rate)
        controls.Add(self.rate_choice, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        self.follow = wx.CheckBox(self, label='Follow on map')
        self.follow.SetValue(True)
        self.follow.Bind(wx.EVT_CHECKBOX, lambda event: self.update_map())
        controls.Add(self.follow, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 8)
        layout = wx.BoxSizer(wx.VERTICAL)
        layout.Add(self.panel, 1, wx.EXPAND)
        layout.Add(self.slider, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        layout.Add(controls, 0, wx.EXPAND)
        self.SetSizer(layout)
        self.CreateStatusBar()
        self.SetStatusText('Space: pause | Arrows: +/-10s | , / .: frame | Click: map marker')
        self.slider.Bind(wx.EVT_SLIDER, lambda event: self.seek(self.slider.GetValue()))
        self.Bind(wx.EVT_CHAR_HOOK, self.key)
        self.Bind(wx.EVT_CLOSE, self.close)
        self.Bind(wx.EVT_ACTIVATE, self.activate)
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.tick, self.timer)
        self.timer.Start(15)
        self.request(0)

    @property
    def sample(self):
        return self.index.samples[self.number] if self.number is not None else None

    def osd(self):
        sample = self.sample
        if sample is None:
            return []
        record = sample.record
        heading = record.get('heading_rad') if record else None
        heading = '%05.1f deg' % (math.degrees(heading) % 360) if finite(heading) else '--'
        speed = '%.1f m/s' % sample.speed if sample.speed is not None else '--'
        hfov = self.projection.effective_fov(record)
        fov_text = '%.2f deg' % hfov if hfov is not None else '-- (use --fov)'
        state = 'PLAY' if self.playing else 'PAUSED'
        lines = [
            '%s / %s   %s %gx' % (clock_text(sample.time), clock_text(self.index.duration), state, self.rate),
            'Vehicle ' + angle_text(attitude(record, 'vehicle_attitude')),
            'Gimbal  ' + angle_text(attitude(record, 'gimbal_attitude')),
            'Ground speed (%s): %s   Heading: %s' % (sample.speed_source, speed, heading),
            'HFOV: %s (%s)%s' % (fov_text,
                'recorded' if self.projection.fov is None else 'override',
                '   NO FRAME TELEMETRY' if record is None else '')]
        if record and record.get('schema') == 'siyi.subtitle.v1':
            from datetime import datetime, timezone
            utc = datetime.fromtimestamp(record['utc_us'] * 1e-6, timezone.utc)
            lines.append('Recorded UTC: ' + utc.strftime('%Y-%m-%d %H:%M:%S'))
            if camera_pose(record) is None:
                lines.append('Map projection needs vehicle attitude (--tlog)')
        return lines

    def media_time(self):
        if self.playing:
            return self.anchor_media + (time.monotonic() - self.anchor_wall) * self.rate
        return self.anchor_media

    def request(self, number):
        if number != self.requested:
            self.requested = number
            self.reader.request(number, self.generation)

    def seek(self, number):
        number = max(0, min(len(self.index.samples) - 1, number))
        self.generation += 1
        self.anchor_media = self.index.times[number]
        self.anchor_wall = time.monotonic()
        self.requested = None
        self.request(number)
        self.map_dirty = True

    def set_playing(self, playing):
        if self.playing == playing:
            return
        # Pause on the image actually displayed, even when the decoder is behind.
        target = self.number if self.number is not None else 0
        self.playing = playing
        self.seek(target)
        self.play_button.SetLabel('Pause' if playing else 'Play')
        self.panel.Refresh(False)

    def toggle(self, event=None):
        if not self.playing and self.number == len(self.index.samples) - 1:
            self.playing = True
            self.seek(0)
            self.play_button.SetLabel('Pause')
        else:
            self.set_playing(not self.playing)

    def skip(self, seconds):
        now = self.sample.time if self.sample else 0
        self.seek(self.index.frame_at(now + seconds))

    def step(self, frames):
        self.set_playing(False)
        self.seek((self.requested if self.requested is not None else 0) + frames)

    def change_rate(self, event=None):
        self.anchor_media = self.sample.time if self.sample else 0
        self.anchor_wall = time.monotonic()
        self.rate = self.rates[self.rate_choice.GetSelection()]
        self.panel.Refresh(False)

    def key(self, event):
        code = event.GetKeyCode()
        if code == wx.WXK_SPACE:
            self.toggle()
        elif code in (wx.WXK_LEFT, wx.WXK_RIGHT):
            self.skip((-1 if code == wx.WXK_LEFT else 1) * (60 if event.ShiftDown() else 10))
        elif code in (ord(','), ord('.')):
            self.step(-1 if code == ord(',') else 1)
        elif code in (wx.WXK_HOME, wx.WXK_END):
            self.seek(0 if code == wx.WXK_HOME else len(self.index.samples) - 1)
        else:
            event.Skip()

    def activate(self, event):
        if not event.GetActive():
            self.hover = None
            self.clear_hover()
        event.Skip()

    def tick(self, event=None):
        try:
            number, generation, frame, error = self.reader.results.get_nowait()
        except queue.Empty:
            pass
        else:
            if generation == self.generation:
                if error:
                    self.set_playing(False)
                    self.SetStatusText(error)
                else:
                    self.number, self.frame = number, frame
                    self.slider.SetValue(number)
                    self.panel.update_bitmap()
                    self.map_dirty = True
                    self.update_hover()
        if self.playing:
            target = self.index.frame_at(self.media_time())
            self.request(target)
            # A restart/seek can leave the last image on screen while the
            # worker decodes the new target. Only stop at the playback clock's
            # end, not merely because the previous image was the last frame.
            if self.number == len(self.index.samples) - 1 and target == self.number:
                self.set_playing(False)
        if time.monotonic() - self.last_hover_update > .05:
            self.last_hover_update = time.monotonic()
            self.update_hover()
        if self.map.is_alive() and self.map_dirty and time.monotonic() - self.last_map_update > .2:
            self.update_map()
        # The map has its own event queue; consume it even though markers are
        # placed from the video, so map mouse activity cannot grow it indefinitely.
        if self.map.is_alive():
            self.map.check_events()

    def update_map(self):
        if self.sample is None or not self.map.is_alive():
            return
        self.last_map_update = time.monotonic()
        record = self.sample.record
        pos = position(record)
        if pos is not None:
            self.map.add_object(mp_slipmap.SlipIcon('vehicle', pos[:2], circle_image((255, 255, 255)),
                                                  layer='Video', label='Camera'))
            if self.follow.GetValue():
                self.map.set_center(*pos[:2])
        else:
            self.map.remove_object('vehicle')
        footprint = self.projection.footprint(record)
        if footprint:
            self.map.add_object(mp_slipmap.SlipPolygon('viewport', footprint, layer='Video',
                colour=(0, 255, 255), linewidth=2, showcircles=False))
            self.map_dirty = False
        else:
            self.map.remove_object('viewport')
            # Retry while paused: terrain tiles may still be downloading.
            self.map_dirty = True
        self.update_hover()

    def clear_hover(self):
        if self.hover_visible and self.map.is_alive():
            self.map.remove_object('video-hover')
        self.hover_visible = False
        self.hover_location = None

    def update_hover(self):
        if not self.map.is_alive() or not self.IsShown():
            return
        # Re-evaluate after resizing, playback and seeking, including a stationary
        # mouse. Letterbox bars are outside the video and have no ground location.
        mouse = wx.GetMousePosition()
        point = self.panel.ScreenToClient(mouse)
        self.hover = self.panel.pixel(point) if wx.FindWindowAtPoint(mouse) == self.panel else None
        location = (self.projection.pixel(self.sample.record, *self.hover)
                    if self.sample is not None and self.hover is not None else None)
        if location is None:
            self.clear_hover()
            return
        if self.hover_location != location:
            self.map.add_object(mp_slipmap.SlipIcon('video-hover', location[:2],
                circle_image((0, 255, 255)), layer='Video'))
            self.hover_location = location
        self.hover_visible = True

    def place_marker(self, pixel):
        if self.sample is None or not self.map.is_alive():
            return
        location = self.projection.pixel(self.sample.record, *pixel)
        if location is None:
            self.SetStatusText('No ground intersection: check telemetry, FOV and terrain coverage')
            return
        label = 'M%u' % (len(self.markers) + 1)
        self.markers.append((label, self.sample.time, pixel, location))
        self.map.add_object(mp_slipmap.SlipIcon(label, location[:2],
            circle_image((0, 0, 255), 6), layer='Markers', label=label))
        message = '%s  %s  %.7f, %.7f  %.1f m AMSL' % (label, clock_text(self.sample.time), *location)
        self.SetStatusText(message)
        print(message, flush=True)

    def close(self, event=None):
        if self.closing:
            return
        self.closing = True
        self.timer.Stop()
        self.reader.stop()
        self.map.close()
        self.Destroy()
