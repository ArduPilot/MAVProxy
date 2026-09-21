"""Raw thermal viewer: preserve radiometry, colour only a display copy."""
import json
import threading
from pathlib import Path
import time

import cv2
import numpy as np
from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_menu import MPMenuItem, MPMenuSubMenu
from MAVProxy.modules.lib import camera_projection
from MAVProxy.modules.mavproxy_camera.thermal_stream import ThermalReader, projection_pose


class ThermalView:
    PALETTES = {
        'Greyscale': None,
        'Inferno': cv2.COLORMAP_INFERNO,
        'Turbo': cv2.COLORMAP_TURBO,
    }

    def __init__(self, module, camera, stream, uri, _latency=0):
        # Fail before creating a window if the optional decoder is absent.
        try:
            import av  # noqa: F401
        except ImportError as error:
            raise RuntimeError('Raw thermal viewing requires PyAV: python3 -m pip install av') from error
        self.module, self.camera, self.stream, self.uri = module, camera, stream, uri
        self.image = MPImage(title='%s - Raw Thermal' % camera.label(), width=640, height=512,
                             auto_size=False, auto_fit=True, mouse_events=True, mouse_movement_events=True)
        menu = self.image.get_popup_menu()
        menu.add(MPMenuItem('Pause/resume', returnkey='Thermal:Pause'))
        menu.add(MPMenuItem('Save raw frame and metadata', returnkey='Thermal:Save'))
        menu.add(MPMenuSubMenu('Palette', items=[
            MPMenuItem(name, returnkey='Thermal:Palette:' + name)
            for name in self.PALETTES]))
        self.image.set_popup_menu(menu)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.shown = None
        self.cursor = None
        self.last_readout = None
        self.last_title = None
        self.paused = False
        self.palette = 'Greyscale'
        self.last_key = None
        self.error = None
        self.worker = threading.Thread(target=self._receive, daemon=True)
        self.worker.start()

    def _receive(self):
        while not self.stop.is_set():
            reader = None
            try:
                reader = ThermalReader(self.uri)
                for pixels, metadata in reader.frames():
                    if self.stop.is_set():
                        break
                    with self.lock:
                        self.latest = pixels, metadata
                        self.error = None
            except Exception as error:
                with self.lock:
                    self.error = str(error)
            finally:
                if reader is not None:
                    reader.close()
            self.stop.wait(1)

    def alive(self):
        return self.image is not None and self.image.is_alive()

    def close(self):
        self.stop.set()
        if self.image is not None:
            self.image.terminate()
            self.image = None
        self.worker.join(timeout=.1) # network reader has a bounded timeout

    def _save(self):
        if self.shown is None:
            return
        pixels, metadata = self.shown
        directory = Path(self.module.logdir or '.') / 'raw_thermal'
        directory.mkdir(parents=True, exist_ok=True)
        stem = '%u_%u_%u_%u' % (self.camera.system_id, self.camera.component_id,
                                metadata['capture_monotonic_us'], metadata['frame_id'])
        # Save native pixel orientation, not the display's rotation/palette.
        (directory / (stem + '.bin')).write_bytes(pixels.astype('<u2', copy=False).tobytes())
        (directory / (stem + '.json')).write_text(json.dumps(metadata, indent=2) + '\n')
        print('Raw thermal frame saved: %s' % (directory / stem))

    def _temperature(self, x, y):
        pixels, m = self.shown
        sx, sy = (639-x, 511-y) if m['rotation_deg'] == 180 else (x, y)
        return int(pixels[sy, sx])*m['temperature_scale_k'] + m['temperature_offset_k'] - 273.15

    def _update_readout(self):
        if self.shown is None:
            return
        m = self.shown[1]
        title = '%s - Raw Thermal%s%s' % (
            self.camera.label(), ' [paused]' if self.paused else '',
            ' [SITL test pattern]' if m.get('simulated') else '')
        if title != self.last_title:
            self.image.set_title(title)
            self.last_title = title
        text = 'Min: %.2f C    Max: %.2f C\n' % (m['minimum_c'], m['maximum_c'])
        if self.cursor is None:
            text += 'Hover over the image for pixel temperature'
        else:
            x, y = self.cursor
            text += 'Pixel (%u, %u): %.2f C' % (x, y, self._temperature(x, y))
        if text != self.last_readout:
            self.image.set_status_text(text)
            self.last_readout = text

    def _pixel(self, x, y):
        if self.shown is None:
            return
        self.cursor = (x, y) if 0 <= x < 640 and 0 <= y < 512 else None
        self._update_readout()
        if self.cursor is not None:
            return self._temperature(x, y)

    def _project(self, x, y, temperature):
        # Freeze first, so pixel selection always uses the displayed frame's
        # pose rather than whatever telemetry arrived after the mouse click.
        if not self.paused or temperature is None:
            return
        pose = projection_pose(self.shown[1])
        if pose is None:
            print('Raw thermal: no fresh capture-time pose available')
            return
        terrain = self.module.module('terrain')
        elevation = getattr(terrain, 'ElevationModel', None)
        if elevation is None:
            print('Raw thermal: load terrain to project a pixel')
            return
        params = camera_projection.CameraParams(xresolution=640, yresolution=512,
                                                FOV=self.shown[1]['hfov_deg'])
        projection = camera_projection.CameraProjection(params, elevation_model=elevation)
        location = projection.get_latlonalt_for_pixel(x, y, *pose)
        if location is not None:
            print('Raw thermal %.3f C at %.7f %.7f %.2f AMSL' % (temperature, *location))

    def check_events(self):
        if not self.alive():
            return
        for event in self.image.events():
            if isinstance(event, MPMenuItem):
                if event.returnkey == 'Thermal:Pause':
                    self.paused = not self.paused
                elif event.returnkey == 'Thermal:Save':
                    self._save()
                elif str(event.returnkey).startswith('Thermal:Palette:'):
                    palette = event.returnkey.split(':', 2)[2]
                    if palette in self.PALETTES:
                        self.palette = palette
            elif getattr(event, 'ClassName', '') == 'wxMouseEvent':
                temperature = self._pixel(event.x, event.y)
                if getattr(event, 'leftIsDown', False):
                    self._project(event.x, event.y, temperature)
        with self.lock:
            latest, error = self.latest, self.error
        if error:
            self.image.set_title('%s - Raw Thermal: %s' % (self.camera.label(), error))
        if self.paused:
            latest = self.shown
        if latest is None:
            return
        pixels, m = latest
        key = m['capture_monotonic_us'], m['frame_id'], self.palette
        if key == self.last_key:
            self._update_readout()
            return
        self.last_key, self.shown = key, latest
        lo, hi = int(pixels.min()), int(pixels.max())
        display = ((pixels.astype(np.float32)-lo)*(255.0/max(1, hi-lo))).astype(np.uint8)
        colormap = self.PALETTES[self.palette]
        display = (cv2.cvtColor(display, cv2.COLOR_GRAY2BGR) if colormap is None
                   else cv2.applyColorMap(display, colormap))
        if m['rotation_deg'] == 180:
            display = display[::-1, ::-1].copy()
        self.image.set_image(display, bgr=True)
        self._update_readout()
