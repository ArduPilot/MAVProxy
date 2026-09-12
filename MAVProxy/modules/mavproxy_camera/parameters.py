"""Per-camera definition loading and acknowledged PARAM_EXT transactions."""

import queue
import re
import threading
import time
from urllib.parse import urlsplit, unquote

from pymavlink import mavutil
from MAVProxy.modules.mavproxy_camera.definition import (
    CameraDefinition, download_definition, decode_value, equal_value)


class CameraParameters:
    RETRY_INTERVAL = 3.5
    MAX_ATTEMPTS = 3

    def __init__(self, module, camera):
        self.module = module
        self.camera = camera
        self.definition = None
        self.identity = None
        self.advertised_identity = None
        self.generation = 0
        self.next_read_batch = 0.0
        self.values = {}
        self.pending = {}
        self.reads = {}
        self.reported_count = None
        self.reported_parameters = {}
        self.errors = {}
        self.result_queue = queue.Queue()
        self.loading = False
        self.status = 'Waiting for camera information'
        self.dialog = None
        self.open_when_ready = False
        self.dirty = True
        self.closed = False
        self.last_poll = 0.0

    def information(self, info):
        from MAVProxy.modules.mavproxy_camera import _text
        identity = (_text(info.cam_definition_uri), info.cam_definition_version)
        if identity == self.advertised_identity:
            return
        self.advertised_identity = identity
        self.load(*identity)

    def load(self, uri, version=0, local=False):
        self.identity = (uri, version)
        self.generation += 1
        generation = self.generation
        self.definition = None
        self.camera.definition = None
        self.values.clear()
        self.pending.clear()
        self.reads.clear()
        self.reported_count = None
        self.reported_parameters.clear()
        self.errors.clear()
        self.loading = bool(uri)
        self.status = 'Loading camera definition' if uri else 'Camera does not advertise a definition file'
        self.dirty = True
        if self.dialog is not None:
            self.dialog.close()
            self.dialog = None
            self.open_when_ready = True
        if not uri:
            return

        def completed(data=None, error=None):
            self.result_queue.put((generation, data, error))

        scheme = uri.split(':', 1)[0].lower()
        if scheme in ('mftp', 'mavftp'):
            ftp = self.module.module('ftp')
            if ftp is None:
                completed(error='Load the ftp module to download MAVFTP camera definitions')
                return
            system, component = self.camera.system_id, self.camera.component_id
            try:
                # QGC's standard form is mftp://[;comp=100]/path. The
                # bracketed component selector is not an IPv6 URL authority.
                path = uri.split('://', 1)[1]
                if scheme == 'mftp':
                    match = re.match(r'^/?\[;comp=(\d+)\]', path)
                    if match:
                        component = int(match.group(1))
                        path = path[match.end():]
                    path = '/' + path.lstrip('/')
                else:
                    parsed = urlsplit(uri)
                    if parsed.netloc:
                        system, component = map(int, parsed.netloc.split(':'))
                    path = parsed.path
                if not 1 <= system <= 255 or not 1 <= component <= 255:
                    raise ValueError('invalid component address')
            except (ValueError, IndexError):
                completed(error='Invalid MAVFTP camera definition address')
                return
            def parse_ftp(data):
                try:
                    completed(CameraDefinition(data))
                except Exception as error:
                    completed(error=str(error))

            def ftp_done(fh):
                if fh is None:
                    completed(error='MAVFTP definition download failed')
                else:
                    fh.seek(0)
                    from MAVProxy.modules.mavproxy_camera.definition import MAX_DEFINITION_SIZE
                    data = fh.read(MAX_DEFINITION_SIZE + 1)
                    threading.Thread(target=parse_ftp, args=(data,),
                                     name='camera-definition', daemon=True).start()
            ftp.cmd_get([unquote(path)], callback=ftp_done,
                        target_system=system, target_component=component)
            return
        if not local and scheme not in ('http', 'https'):
            completed(error='Unsupported camera definition URI scheme: %s' % scheme)
            return

        def worker():
            try:
                if local:
                    from MAVProxy.modules.mavproxy_camera.definition import MAX_DEFINITION_SIZE
                    with open(uri, 'rb') as file:
                        data = file.read(MAX_DEFINITION_SIZE + 1)
                else:
                    data = download_definition(uri)
                # Parsing and decompression also stay off the MAVLink thread.
                completed(CameraDefinition(data))
            except Exception as error:
                completed(error=str(error))
        threading.Thread(target=worker, name='camera-definition', daemon=True).start()

    def request_all(self):
        if self.definition is None:
            return
        now = time.monotonic()
        self.errors.clear()
        self.reported_count = None
        self.reported_parameters.clear()
        self.reads = {p.name: [now + self.RETRY_INTERVAL, 1]
                      for p in self.definition.parameters.values() if not p.writeonly}
        self.module.master.mav.param_ext_request_list_send(
            self.camera.system_id, self.camera.component_id)
        self.status = 'Reading camera settings'
        self.last_poll = now
        self.dirty = True

    def request_read(self, name, delay=0):
        param = self.definition.parameters.get(name)
        if param is None or param.writeonly or name in self.pending:
            return
        if name in self.reads:
            return
        self.reads[name] = [time.monotonic() + delay, 0]

    def set_value(self, name, value):
        if self.definition is None or name not in self.definition.parameters:
            raise ValueError('unknown camera setting %s' % name)
        param = self.definition.parameters[name]
        controls = self.definition.controls(self.values)
        if param.readonly or name not in controls:
            raise ValueError('%s is not currently editable' % name)
        if name not in self.values and not param.writeonly:
            raise ValueError('%s has not been read from the camera' % name)
        if name in self.pending:
            raise ValueError('%s is still being applied' % name)
        value = param.validate(value, controls[name])
        self.errors.pop(name, None)
        self.reads.pop(name, None)
        self.pending[name] = {'value': value, 'attempts': 1,
                              'deadline': time.monotonic() + self.RETRY_INTERVAL,
                              'expires': time.monotonic() + 30, 'in_progress': False}
        self._send(name)
        self.status = 'Applying %s' % param.description
        self.dirty = True

    def _send(self, name):
        param = self.definition.parameters[name]
        self.module.master.mav.param_ext_set_send(
            self.camera.system_id, self.camera.component_id,
            name.encode('ascii'), param.encode(self.pending[name]['value']), param.wire_type)

    def _received(self, name, value, refresh=False):
        old = self.values.get(name)
        self.values[name] = value
        self.reads.pop(name, None)
        self.errors.pop(name, None)
        if refresh or (old is not None and not equal_value(old, value)):
            for target in self.definition.parameters[name].updates:
                self.request_read(target, delay=0.5)
        self.dirty = True

    def _track_parameter_list(self, message, name):
        """Track the camera's complete list, including names absent from XML."""
        count = getattr(message, 'param_count', 0)
        index = getattr(message, 'param_index', -1)
        if not 0 <= index < count <= 65535:
            return
        if count != self.reported_count:
            self.reported_count = count
            self.reported_parameters.clear()
        self.reported_parameters[index] = name

    def _read_timeout_error(self, name):
        if (self.reported_count is not None and
                len(self.reported_parameters) == self.reported_count and
                name not in self.reported_parameters.values()):
            return 'Not listed by camera; camera definition mismatch'
        return 'No response; use Refresh to retry'

    def packet(self, message):
        if self.definition is None:
            return
        from MAVProxy.modules.mavproxy_camera import _text
        name = _text(message.param_id)
        if message.get_type() == 'PARAM_EXT_VALUE':
            self._track_parameter_list(message, name)
        param = self.definition.parameters.get(name)
        if param is None:
            return
        if message.param_type != param.wire_type:
            self.errors[name] = 'Camera returned a different parameter type'
            self.dirty = True
            return
        is_ack = message.get_type() == 'PARAM_EXT_ACK'
        pending = self.pending.get(name)
        if is_ack and message.param_result == mavutil.mavlink.PARAM_ACK_IN_PROGRESS:
            if pending:
                pending['deadline'] = pending['expires']
                pending['in_progress'] = True
            return
        try:
            value = decode_value(message)
        except (ValueError, TypeError) as error:
            self.errors[name] = str(error)
            self.dirty = True
            return
        if is_ack:
            if pending is None:
                return
            self.pending.pop(name)
            accepted = message.param_result == mavutil.mavlink.PARAM_ACK_ACCEPTED
            self._received(name, value, refresh=accepted)
            if accepted:
                self.status = '%s applied' % param.description
            else:
                result = mavutil.mavlink.enums['PARAM_ACK'].get(message.param_result)
                self.errors[name] = result.name if result else 'Write rejected'
                self.status = '%s: %s' % (param.description, self.errors[name])
                self.request_read(name)
        elif pending is None:
            self._received(name, value)
            if not self.reads:
                self.status = ('Some camera settings are unavailable' if self.errors
                               else 'Camera settings up to date')

    def snapshot(self):
        controls = self.definition.controls(self.values)
        rows = []
        for param in self.definition.parameters.values():
            if not param.control:
                continue
            rows.append(dict(name=param.name, description=param.description,
                             type=param.type, minimum=param.minimum, maximum=param.maximum,
                             step=param.step, options=controls.get(param.name, param.options),
                             visible=param.name in controls, readonly=param.readonly,
                             value=self.values.get(param.name),
                             enabled=(not param.readonly and param.name not in self.pending and
                                      (param.name in self.values or param.writeonly)),
                             pending=param.name in self.pending,
                             error=self.errors.get(param.name, '')))
        received = sum(p.name in self.values for p in self.definition.parameters.values() if not p.writeonly)
        total = sum(not p.writeonly for p in self.definition.parameters.values())
        return dict(rows=rows, status='%s — %u/%u settings read' % (self.status, received, total))

    def open_dialog(self):
        if self.definition is None:
            self.open_when_ready = True
            print('Camera: %s' % self.status)
            return
        from MAVProxy.modules.lib import mp_util
        if not mp_util.has_wxpython:
            print('Custom Settings requires wxPython; use camera params / camera param NAME VALUE')
            self.open_when_ready = False
            return
        if self.dialog is not None and self.dialog.is_alive():
            self.dialog.send({'raise': True})
        else:
            from MAVProxy.modules.mavproxy_camera.settings_dialog import CameraSettingsDialog
            self.dialog = CameraSettingsDialog(self.camera.label(), self.snapshot())
        self.open_when_ready = False
        self.dirty = True

    def idle(self):
        if self.closed:
            return
        while not self.result_queue.empty():
            generation, data, error = self.result_queue.get_nowait()
            if generation != self.generation:
                continue
            self.loading = False
            if error is None:
                try:
                    self.definition = data if isinstance(data, CameraDefinition) else CameraDefinition(data)
                except Exception as exception:
                    error = str(exception)
            if error is not None:
                self.status = 'Definition load failed: %s' % error
                print('Camera %s: %s' % (self.camera.label(), self.status))
            else:
                self.camera.definition = self.definition
                print('Camera %s: loaded %u settings' % (self.camera.label(), len(self.definition.parameters)))
                self.request_all()
                if self.open_when_ready:
                    self.open_dialog()
            self.dirty = True
        now = time.monotonic()
        # Pace missing and dependent reads so large XML files cannot flood a link.
        sent = 0
        for name, (deadline, attempts) in list(self.reads.items()):
            if now < deadline or sent >= 5 or now < self.next_read_batch:
                continue
            if attempts >= self.MAX_ATTEMPTS:
                del self.reads[name]
                self.errors.setdefault(name, self._read_timeout_error(name))
                self.status = 'Some camera settings did not respond'
                self.dirty = True
                continue
            self.module.master.mav.param_ext_request_read_send(
                self.camera.system_id, self.camera.component_id, name.encode('ascii'), -1)
            self.reads[name] = [now + self.RETRY_INTERVAL, attempts + 1]
            sent += 1
        if sent:
            self.next_read_batch = now + 0.2
        for name, pending in list(self.pending.items()):
            if now < pending['deadline']:
                continue
            if (pending['in_progress'] or pending['attempts'] >= self.MAX_ATTEMPTS or
                    now >= pending['expires']):
                del self.pending[name]
                self.errors[name] = 'Write timed out; camera state is unconfirmed'
                self.status = '%s: write timed out' % name
                self.request_read(name)
                self.dirty = True
            else:
                self._send(name)
                pending['attempts'] += 1
                pending['deadline'] = now + self.RETRY_INTERVAL
        if self.dialog is not None:
            if not self.dialog.is_alive():
                self.dialog.close()
                self.dialog = None
            else:
                for event in self.dialog.events():
                    if event[0] == 'refresh':
                        self.request_all()
                    elif event[0] == 'set':
                        try:
                            self.set_value(event[1], event[2])
                        except (ValueError, TypeError) as error:
                            self.errors[event[1]] = str(error)
                            self.status = '%s: %s' % (event[1], error)
                            self.dirty = True
                if self.dirty:
                    self.dirty = not self.dialog.send(self.snapshot())
                if now - self.last_poll > 5 and not self.reads and not self.pending:
                    # Some cameras do not broadcast changes made by other GCSs.
                    for param in self.definition.parameters.values():
                        if param.name in self.values:
                            self.request_read(param.name)
                    self.last_poll = now

    def close(self):
        self.closed = True
        if self.dialog:
            self.dialog.close()
            self.dialog = None
