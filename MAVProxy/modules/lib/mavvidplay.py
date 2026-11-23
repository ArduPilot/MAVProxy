"""Video/OSD window and synchronized terrain map for mavvidplay.py."""
import math
from datetime import datetime, timezone
from dataclasses import dataclass
import bisect
import queue
import sys
import threading
import time

import cv2
import numpy as np
from MAVProxy.modules.lib import wx_processguard  # noqa: F401
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib.video_telemetry import (
    camera_pose, attitude, finite, image_pixel, image_rect, position, TelemetryAlignment)
from MAVProxy.modules.mavproxy_map import mp_slipmap


@dataclass
class ThermalFrame:
    image: object  # contrast-adjusted BGR image, for display only
    pixels: object  # original uint16 sensor samples, from the same decoded frame


class FrameReader(threading.Thread):
    """Own the decoder in one worker; coalesce seeks without blocking wx."""
    def __init__(self, source):
        super().__init__(name='video-decoder', daemon=True)
        # Accept a VideoIndex (for the thermal frame timestamps) or a bare
        # filename for the OpenCV path used by callers and tests.
        if isinstance(source, str):
            self.filename, self.thermal, self.frame_pts = source, False, None
            self.keyframe_pts = None
        else:
            self.filename = source.filename
            self.thermal = getattr(source, 'thermal', False)
            self.frame_pts = getattr(source, 'frame_pts', None)
            self.keyframe_pts = getattr(source, 'keyframe_pts', None)
        self.keyframe_dts = getattr(source, 'keyframe_dts', {})
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
        if self.frame_pts is not None:
            self._run_timestamped()
        else:
            self._run_opencv()

    @staticmethod
    def _thermal_to_bgr(gray16):
        """Auto-range the 16-bit radiometric frame to a grey 8-bit image, using
        1st/99th percentiles so a few hot or cold pixels do not flatten it."""
        data = gray16.astype(np.float32)
        low, high = np.percentile(data, (1, 99))
        if not high > low:
            low, high = float(data.min()), float(data.max())
        if not high > low:
            high = low + 1
        scaled = np.clip((data - low) * (255.0 / (high - low)), 0, 255).astype(np.uint8)
        return cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)

    def _run_timestamped(self):
        try:
            import av
            container = av.open(self.filename)
            stream = container.streams.video[0]
        except Exception as error:
            container = None
            open_error = str(error)
        decoder, next_number = None, None
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    break
                number, generation = self.pending
                self.pending = None
            try:
                if container is None:
                    raise ValueError('PyAV could not open the video: %s' % open_error)
                target = self.frame_pts[number]
                if next_number is None or number < next_number or number-next_number > 30:
                    seek = target
                    if self.keyframe_pts:
                        i = max(0, bisect.bisect_right(self.keyframe_pts, target)-1)
                        seek = self.keyframe_pts[i]
                        # Demuxer seeks use decoding time. With B-frames the
                        # keyframe's presentation time can land in the next GOP.
                        seek = self.keyframe_dts.get(seek, seek)
                    # MPEG-TS binary seeking locates PCR packets, which may be
                    # P-frames. Start at an indexed keyframe and decode preroll.
                    container.seek(seek, stream=stream, backward=True, any_frame=False)
                    decoder = container.decode(stream)
                image = None
                for decoded in decoder:
                    if decoded.pts is not None and decoded.pts >= target:
                        image = decoded
                        break
                if image is None or image.pts != target:
                    raise ValueError('Cannot decode frame %u' % number)
                next_number = number+1
                if self.thermal:
                    pixels = image.to_ndarray()
                    frame = ThermalFrame(self._thermal_to_bgr(pixels), pixels)
                else:
                    frame = image.to_ndarray(format='bgr24')
                result = (number, generation, frame, None)
            except Exception as error:
                next_number = None
                result = (number, generation, None, str(error))
            try:
                self.results.get_nowait()
            except queue.Empty:
                pass
            self.results.put(result)
        if container is not None:
            container.close()

    def _run_opencv(self):
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
    def __init__(self, index, projection, map_display, paused=False, telemetry_offset=0):
        if not finite(telemetry_offset):
            raise ValueError('Telemetry offset must be finite')
        super().__init__(None, title='MAV Video: ' + index.filename, size=(1000, 740))
        self.index, self.projection, self.map = index, projection, map_display
        self.alignment = TelemetryAlignment(index)
        self.telemetry_offset = float(telemetry_offset)
        self.frame = None
        self.thermal_pixels = None
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
        self.pending_seek = None
        self._was_indexing = bool(getattr(index, '_index_thread', None))
        self.reader = FrameReader(index)
        self.panel = VideoPanel(self)
        self.telemetry_text = wx.TextCtrl(self, value='Loading telemetry...',
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.BORDER_NONE)
        self.telemetry_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.telemetry_text.SetMinSize((self.telemetry_text.GetTextExtent('M' * 40)[0] + 20, -1))
        self._telemetry_text = None
        total = max(len(index.samples), round(index.duration*index.fps)) if self._was_indexing else len(index.samples)
        self.slider = wx.Slider(self, minValue=0, maxValue=max(1, total - 1))
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
        alignment_controls = wx.BoxSizer(wx.HORIZONTAL)
        alignment_controls.Add(wx.StaticText(self, label='Telemetry offset (s):'),
                               0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        limit = max(86400, abs(self.telemetry_offset))
        self.offset_control = wx.SpinCtrlDouble(self, min=-limit, max=limit,
            initial=self.telemetry_offset, inc=.1,
            style=wx.SP_ARROW_KEYS | wx.TE_PROCESS_ENTER, size=(115, -1))
        self.offset_control.SetDigits(2)
        self.offset_control.SetToolTip('Negative: earlier telemetry; positive: later telemetry. '
            'Uses recorded UTC seconds when available, otherwise video time. '
            'Enter or leave the field to apply a typed value.')
        self.offset_control.Bind(wx.EVT_SPINCTRLDOUBLE, self.apply_telemetry_offset)
        self.offset_control.Bind(wx.EVT_TEXT_ENTER, self.apply_telemetry_offset)
        self.offset_control.Bind(wx.EVT_KILL_FOCUS, self.change_telemetry_offset)
        alignment_controls.Add(self.offset_control, 0, wx.ALL, 4)
        reset = wx.Button(self, label='Reset offset')
        reset.Bind(wx.EVT_BUTTON, lambda event: self.set_telemetry_offset(0))
        alignment_controls.Add(reset, 0, wx.ALL, 4)
        alignment_controls.Add(wx.StaticText(self, label='− earlier / + later telemetry'),
                               0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        layout = wx.BoxSizer(wx.VERTICAL)
        content = wx.BoxSizer(wx.HORIZONTAL)
        content.Add(self.panel, 1, wx.EXPAND)
        content.Add(self.telemetry_text, 0, wx.EXPAND | wx.ALL, 8)
        layout.Add(content, 1, wx.EXPAND)
        layout.Add(self.slider, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        layout.Add(controls, 0, wx.EXPAND)
        layout.Add(alignment_controls, 0, wx.EXPAND)
        self.SetSizer(layout)
        self.CreateStatusBar()
        self.SetStatusText('Space: pause | Arrows: +/-10s | , / .: frame | [ / ]: offset (Shift: 1s) | Click: marker')
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

    @property
    def telemetry_sample(self):
        return self.alignment.at(self.number, self.telemetry_offset)[0]

    @property
    def telemetry_basis(self):
        return self.alignment.at(self.number, self.telemetry_offset)[1]

    def set_telemetry_offset(self, value):
        if not finite(value):
            raise ValueError('Telemetry offset must be finite')
        self.telemetry_offset = float(value)
        limit = max(86400, abs(value))
        self.offset_control.SetRange(-limit, limit)
        self.offset_control.SetValue(value)
        self.map_dirty = True
        self.update_hover()
        self.update_telemetry()
        self.panel.Refresh(False)

    def change_telemetry_offset(self, event=None):
        if self.closing:
            if event is not None:
                event.Skip()
            return
        value = self.offset_control.GetValue()
        if value != self.telemetry_offset:
            self.set_telemetry_offset(value)
        if event is not None:
            event.Skip()

    def apply_telemetry_offset(self, event):
        self.change_telemetry_offset(event)
        if not self.closing:
            self.panel.SetFocus()

    def osd(self):
        sample = self.sample
        if sample is None:
            return []
        aligned = self.telemetry_sample
        record = aligned.record if aligned else None
        heading = record.get('heading_rad') if record else None
        heading = '%05.1f deg' % (math.degrees(heading) % 360) if finite(heading) else '--'
        speed = '%.1f m/s' % aligned.speed if aligned and aligned.speed is not None else '--'
        hfov = self.projection.effective_fov(record)
        fov_text = '%.2f deg' % hfov if hfov is not None else '-- (use --fov)'
        state = 'PLAY' if self.playing else 'PAUSED'
        utc_text = '--'
        utc_us = sample.record.get('utc_us') if sample.record else None
        if finite(utc_us) and utc_us > 0:
            try:
                seconds, micros = divmod(int(utc_us), 1000000)
                utc = datetime.fromtimestamp(seconds, timezone.utc)
                utc_text = utc.strftime('%Y-%m-%d %H:%M:%S') + '.%03u' % (micros // 1000)
            except (OverflowError, OSError, ValueError):
                pass
        lines = [
            '%s / %s   %s %gx' % (clock_text(sample.time), clock_text(self.index.duration), state, self.rate),
            'Recorded UTC: ' + utc_text,
            'Vehicle ' + angle_text(attitude(record, 'vehicle_attitude')),
            'Gimbal  ' + angle_text(attitude(record, 'gimbal_attitude')),
            'Ground speed (%s): %s' % (aligned.speed_source if aligned else '--', speed),
            'Heading: %s' % heading,
            'HFOV: %s (%s)%s' % (fov_text,
                'recorded' if self.projection.fov is None else 'override',
                '\nNO ALIGNED TELEMETRY' if record is None else '')]
        lines.insert(2, 'Telemetry offset: %+.2f s (%s)' % (
            self.telemetry_offset, self.telemetry_basis))
        telemetry_utc = record.get('utc_us') if record else None
        if self.telemetry_offset:
            text = '--'
            if finite(telemetry_utc) and telemetry_utc > 0:
                try:
                    text = datetime.fromtimestamp(telemetry_utc / 1e6, timezone.utc).isoformat(timespec='milliseconds')
                except (OverflowError, OSError, ValueError):
                    pass
            lines.insert(3, 'Telemetry UTC: ' + text)
        if record and record.get('schema') == 'siyi.subtitle.v1':
            if camera_pose(record) is None:
                lines.append('Map projection needs vehicle attitude (--tlog)')
        return lines

    def thermal_lines(self):
        if not getattr(self.index, 'thermal', False):
            return []
        record = self.sample.record if self.sample else None
        meta = record.get('thermal', {}) if record else {}
        lines = ['', 'THERMAL', 'Pixel temperature: --']
        pixels = self.thermal_pixels
        scale, offset = meta.get('temperature_scale_k'), meta.get('temperature_offset_k')
        if (pixels is not None and self.hover is not None and
                finite(scale) and scale > 0 and finite(offset)):
            x, y = self.hover
            if 0 <= x < pixels.shape[1] and 0 <= y < pixels.shape[0]:
                raw = int(pixels[int(y), int(x)])
                celsius = raw * scale + offset - 273.15
                lines[-1] = 'Pixel (%u, %u): %.2f °C' % (int(x), int(y), celsius)
        low, high = meta.get('minimum_c'), meta.get('maximum_c')
        if finite(low) and finite(high):
            lines.append('Frame min/max: %.2f / %.2f °C' % (low, high))
        named = record.get('named_values', {}) if record else {}
        for name in ('TMIN', 'TMAX'):
            measurement = named.get(name)
            value = measurement.get('value') if isinstance(measurement, dict) else None
            text = '%.2f °C' % value if finite(value) else '--'
            if finite(value):
                if measurement.get('valid') is False:
                    text += ' [invalid]'
                elif finite(measurement.get('age_ms')):
                    text += ' (%.2f s old)' % (measurement['age_ms'] / 1000)
            lines.append('%s (tlog): %s' % (name, text))
        return lines

    def update_telemetry(self):
        text = '\n'.join(self.osd() + self.thermal_lines())
        if text != self._telemetry_text:
            self.telemetry_text.ChangeValue(text)
            self._telemetry_text = text

    def media_time(self):
        if self.playing:
            return self.anchor_media + (time.monotonic() - self.anchor_wall) * self.rate
        return self.anchor_media

    def request(self, number):
        if number != self.requested:
            self.requested = number
            self.reader.request(number, self.generation)

    def seek(self, number):
        if getattr(self.index, 'indexing', False) and number >= len(self.index.times):
            self.pending_seek = number
            self.SetStatusText('Waiting for telemetry index at frame %u' % number)
            return
        self.pending_seek = None
        number = max(0, min(len(self.index.times) - 1, number))
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
        if not self.playing and self.number == len(self.index.samples) - 1 and not getattr(self.index, 'indexing', False):
            self.playing = True
            self.seek(0)
            self.play_button.SetLabel('Pause')
        else:
            self.set_playing(not self.playing)

    def skip(self, seconds):
        now = self.sample.time if self.sample else 0
        if getattr(self.index, 'indexing', False) and now+seconds > self.index.times[-1]:
            self.seek(round((now+seconds)*self.index.fps))
        else:
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
        focus = wx.Window.FindFocus()
        if focus is not None and (focus == self.offset_control or self.offset_control.IsDescendant(focus)):
            event.Skip()
            return
        if code in (ord('['), ord(']'), ord('{'), ord('}')):
            sign = -1 if code in (ord('['), ord('{')) else 1
            self.set_telemetry_offset(round(self.telemetry_offset + sign * (1 if event.ShiftDown() else .1), 2))
        elif code == wx.WXK_SPACE:
            self.toggle()
        elif code in (wx.WXK_LEFT, wx.WXK_RIGHT):
            self.skip((-1 if code == wx.WXK_LEFT else 1) * (60 if event.ShiftDown() else 10))
        elif code in (ord(','), ord('.')):
            self.step(-1 if code == ord(',') else 1)
        elif code in (wx.WXK_HOME, wx.WXK_END):
            end = round(self.index.duration*self.index.fps)-1 if getattr(self.index, 'indexing', False) else len(self.index.samples)-1
            self.seek(0 if code == wx.WXK_HOME else end)
        else:
            event.Skip()

    def activate(self, event):
        if not event.GetActive():
            self.hover = None
            self.clear_hover()
        event.Skip()

    def tick(self, event=None):
        indexing = getattr(self.index, 'indexing', False)
        if indexing or getattr(self, '_was_indexing', False):
            count = len(self.index.times)
            total = max(count, round(self.index.duration*self.index.fps)) if indexing else count
            self.slider.SetMax(max(1, total-1))
            if indexing:
                self.SetStatusText('Indexing telemetry: %.0f%% (%u frames available)' % (self.index.index_progress, count))
            elif self.index.index_error:
                self.set_playing(False)
                message = 'Telemetry index failed: '+self.index.index_error
                self.SetStatusText(message)
                print('mavvidplay: '+message, file=sys.stderr)
            elif self.projection.fov is None and not self.index.has_fov():
                self.set_playing(False)
                message = 'No recorded FOV; reopen with --fov in degrees'
                self.SetStatusText(message)
                print('mavvidplay: '+message, file=sys.stderr)
            else:
                self.SetStatusText('Telemetry index complete: %u frames' % count)
            self._was_indexing = indexing
            self.map_dirty = True
            self.panel.Refresh(False)
        pending = getattr(self, 'pending_seek', None)
        if pending is not None and (pending < len(self.index.times) or not indexing):
            self.seek(pending)
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
                    self.number = number
                    if isinstance(frame, ThermalFrame):
                        self.frame, self.thermal_pixels = frame.image, frame.pixels
                    else:
                        self.frame, self.thermal_pixels = frame, None
                    self.slider.SetValue(number)
                    self.panel.update_bitmap()
                    self.map_dirty = True
                    self.update_hover()
        if self.playing and getattr(self, 'pending_seek', None) is None:
            if indexing and self.media_time() > self.index.times[-1]:
                self.anchor_media = self.index.times[-1]
                self.anchor_wall = time.monotonic()
            target = self.index.frame_at(self.media_time())
            self.request(target)
            # A restart/seek can leave the last image on screen while the
            # worker decodes the new target. Only stop at the playback clock's
            # end, not merely because the previous image was the last frame.
            if not indexing and self.number == len(self.index.samples) - 1 and target == self.number:
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
        aligned = self.telemetry_sample
        record = aligned.record if aligned else None
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
        self.update_telemetry()

    def update_hover(self):
        if not self.IsShown():
            return
        # Re-evaluate after resizing, playback and seeking, including a stationary
        # mouse. Letterbox bars are outside the video and have no ground location.
        mouse = wx.GetMousePosition()
        point = self.panel.ScreenToClient(mouse)
        self.hover = self.panel.pixel(point) if wx.FindWindowAtPoint(mouse) == self.panel else None
        self.update_telemetry()
        # Pixel temperatures do not need vehicle pose, terrain or a running map.
        if not self.map.is_alive():
            return
        aligned = self.telemetry_sample
        location = (self.projection.pixel(aligned.record, *self.hover)
                    if aligned is not None and self.hover is not None else None)
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
        aligned = self.telemetry_sample
        location = self.projection.pixel(aligned.record, *pixel) if aligned else None
        if location is None:
            self.SetStatusText('No ground intersection: check telemetry, FOV and terrain coverage')
            return
        label = 'M%u' % (len(self.markers) + 1)
        self.markers.append((label, self.sample.time, pixel, location))
        self.map.add_object(mp_slipmap.SlipIcon(label, location[:2],
            circle_image((0, 0, 255), 6), layer='Markers', label=label))
        message = '%s  %s  %.7f, %.7f  %.1f m AMSL' % (label, clock_text(self.sample.time), *location)
        if self.telemetry_offset:
            message += '  telemetry offset %+.2f s (%s)' % (self.telemetry_offset, self.telemetry_basis)
        self.SetStatusText(message)
        print(message, flush=True)

    def close(self, event=None):
        if self.closing:
            return
        self.closing = True
        self.timer.Stop()
        self.reader.stop()
        self.index.close()
        self.map.close()
        self.Destroy()
