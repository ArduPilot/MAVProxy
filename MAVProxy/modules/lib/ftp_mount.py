'''FUSE mount of a vehicle filesystem over MAVLink FTP, for "ftp mount"

The filesystem callbacks run in mfusepy's threads, while all FTP state
belongs to MAVProxy's main thread.  Each callback that needs the vehicle
queues a Request and waits on it; FTPModule.idle_task calls
process_requests() on the main thread, which submits the FTP operation with
a callback that completes the Request.

Files are held whole in memory while open: opening downloads the file, and
closing (flush) uploads it again if it was written.  All opens of one file
share a single buffer.  The upload replaces the vehicle's copy from its
first packet, as "ftp put" does, so one that fails part way (link lost,
timed out, unmounted) leaves that copy empty or partial; close() reports
the failure.

AP_FLAKE8_CLEAN
'''

import contextlib
import errno
import os
import posixpath
import queue
import re
import stat
import subprocess
import sys
import threading
import time

from io import BytesIO

from MAVProxy.modules.mavproxy_ftp import MAX_FTP_NAME

# libfuse unlinks an open file by renaming it to a name of this form in the
# same directory, which it unlinks once the file is closed
HIDDEN_NAME = re.compile(r'\.fuse_hidden[0-9a-f]{16}\Z')


def caller_id():
    '''what mfusepy.fuse_get_context() reports as the pid of a request made
    by the calling thread.  Linux gives the thread's id; macFUSE and FreeBSD
    give only the process's id'''
    if sys.platform.startswith('linux'):
        return threading.get_native_id()
    return os.getpid()


def fs_error(code):
    '''an OSError that mfusepy returns to the kernel as -code'''
    return OSError(code, os.strerror(code))


class Request(object):
    '''one filesystem request waiting for the main thread'''

    def __init__(self, fn, args, mount=None):
        self.fn = fn
        self.args = args
        # the FTPMount which made this, so its FTP operation can be found
        # and cancelled
        self.mount = mount
        self.event = threading.Event()
        self.result = None
        self.error = 0
        self.cancelled = False
        self.last_progress = time.monotonic()

    def progress(self, *args):
        '''note progress, so a long transfer is not timed out'''
        self.last_progress = time.monotonic()

    def finish(self, result=None, error=0):
        if self.event.is_set():
            return
        self.result = result
        self.error = error
        self.event.set()


class OpenFile(object):
    '''a file open through the mount, buffered whole in memory and shared
    by every open of it'''

    def __init__(self, path, data, dirty, on_vehicle):
        # FUSE path; follows renames
        self.path = path
        self.data = bytearray(data)
        self.dirty = dirty
        # False for a file created here and not yet uploaded
        self.on_vehicle = on_vehicle
        # removed or replaced on the vehicle, so never written back
        self.unlinked = False
        # number of Handles
        self.opens = 0
        # held while uploading, so writes and flushes wait for the upload
        self.lock = threading.Lock()


class Handle(object):
    '''one open of an OpenFile'''

    def __init__(self, file, flags):
        self.file = file
        self.flags = flags


class FTPMount(object):
    '''a vehicle directory mounted on a local directory'''

    # the mfusepy operations interface reports times in nanoseconds
    use_ns = True

    def __init__(self, module, mountpoint, remote_root='/',
                 target_system=None, target_component=None):
        self.module = module
        self.mountpoint = os.path.abspath(mountpoint)
        # normpath keeps a leading '//', so strip slashes before adding one
        self.remote_root = posixpath.normpath('/' + remote_root.lstrip('/'))
        self.target_system = target_system
        self.target_component = target_component
        self.requests = queue.Queue()
        self.stopping = False
        # set while the module checks remote_root exists before mounting
        self.checking = False
        self.thread = None
        self.thread_error = None
        self.mount_time_ns = time.time_ns()
        self.uid = os.getuid()
        self.gid = os.getgid()
        # FUSE path of a directory -> (monotonic time, {name: entry})
        self.dir_cache = {}
        # FUSE path -> OpenFile
        self.files = {}
        # fi.fh -> Handle
        self.handles = {}
        self.next_handle = 1
        # FUSE paths libfuse has hidden open files at (see hide()); not on
        # the vehicle
        self.hidden = set()
        # not "lock": mfusepy treats any attribute named after a libfuse
        # operation as an implementation of it
        self.state_lock = threading.Lock()
        # mfusepy.fuse_get_context, set when mounted
        self.fuse_context = None
        # caller_id() of the thread which runs process_requests: MAVProxy's
        # main loop, which is not the process's first thread
        self.service_id = caller_id()
        # MAVProxy's working directory, restored once libfuse changes it
        self.saved_cwd = None
        self.initialised = threading.Event()

    def description(self):
        return "%s on %s (vehicle %u:%u)%s" % (
            self.remote_root, self.mountpoint,
            self.target_system or 0, self.target_component or 0,
            " (checking remote directory)" if self.checking else "")

    # ---------------------------------------------------------------
    # lifecycle, called on the main thread

    def check_local(self, create=False):
        '''raise if this can't be mounted locally, before asking the vehicle
        about the remote directory'''
        import mfusepy  # noqa: F401
        if not os.path.exists(self.mountpoint):
            if not create:
                return
            os.makedirs(self.mountpoint)
        if not os.path.isdir(self.mountpoint):
            raise ValueError("%s is not a directory" % self.mountpoint)
        try:
            contents = os.listdir(self.mountpoint)
        except OSError as error:
            if error.errno == errno.ENOTCONN:
                raise ValueError("%s is a stale mount; unmount it with "
                                 "fusermount -u" % self.mountpoint)
            raise
        if contents:
            raise ValueError("%s is not empty" % self.mountpoint)

    def start(self):
        '''mount, raising on failure'''
        import mfusepy
        self.check_local(create=True)
        cache_time = max(0.0, self.module.ftp_settings.mount_cache_time)
        options = {
            'raw_fi': True,
            'foreground': True,
            'nothreads': False,
            'fsname': 'mavftp',
            'attr_timeout': cache_time,
            'entry_timeout': cache_time,
        }
        if sys.platform != 'darwin':
            # macFUSE does not know this option
            options['subtype'] = 'mavftp'
        self.fuse_context = mfusepy.fuse_get_context
        self.service_id = caller_id()
        # libfuse's fuse_daemonize() changes the process's working directory
        # to / even in the foreground; init() changes it back
        try:
            self.saved_cwd = os.open('.', os.O_RDONLY)
        except OSError:
            self.saved_cwd = None

        def run():
            try:
                mfusepy.FUSE(self, self.mountpoint, **options)
            except BaseException as error:
                self.thread_error = error
                if not self.stopping:
                    print("FTP: mount of %s ended: %s" %
                          (self.mountpoint, error))
            finally:
                if not self.stopping:
                    self.fail_requests()

        self.thread = threading.Thread(target=run, name='ftp-mount',
                                       daemon=True)
        self.thread.start()
        try:
            # The kernel answers the root's attributes from getattr below
            # without a round-trip through the main thread, so this wait
            # can't deadlock.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if not self.thread.is_alive():
                    raise ValueError("mount failed: %s" % self.thread_error)
                if self.initialised.is_set() and \
                   os.path.ismount(self.mountpoint):
                    return
                time.sleep(0.05)
            self.stop()
            raise ValueError("mount of %s timed out" % self.mountpoint)
        finally:
            if not self.initialised.is_set():
                self.restore_cwd()

    def restore_cwd(self):
        with self.state_lock:
            saved_cwd = self.saved_cwd
            self.saved_cwd = None
        if saved_cwd is None:
            return
        try:
            os.fchdir(saved_cwd)
        except OSError as error:
            print("FTP mount: unable to restore working directory: %s" %
                  error)
        os.close(saved_cwd)

    def init(self, path):
        '''libfuse is ready, and has finished daemonizing'''
        self.restore_cwd()
        self.initialised.set()

    def unmount_commands(self):
        if sys.platform == 'darwin':
            return [['umount', self.mountpoint],
                    ['diskutil', 'unmount', 'force', self.mountpoint]]
        return [['fusermount3', '-u', self.mountpoint],
                ['fusermount', '-u', self.mountpoint],
                # lazy: detach now, even with a shell still inside it
                ['fusermount3', '-u', '-z', self.mountpoint],
                ['fusermount', '-u', '-z', self.mountpoint]]

    def stop(self):
        '''unmount.  Files not yet closed are not uploaded'''
        self.stopping = True
        self.fail_requests()
        # FTP operations already started for it, so no upload runs late
        self.module.cancel_owned(
            lambda owner: getattr(owner, 'mount', None) is self)
        if self.thread is not None and self.thread.is_alive():
            for command in self.unmount_commands():
                try:
                    result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                            stderr=subprocess.DEVNULL,
                                            timeout=10)
                except (OSError, subprocess.TimeoutExpired):
                    continue
                if result.returncode == 0:
                    break
            else:
                print("FTP: unable to unmount %s" % self.mountpoint)
            # A lazy unmount leaves the filesystem serving whatever still
            # uses it.  Requests now fail at once, so don't hold up the main
            # loop waiting for that.
            self.thread.join(1.0)
            if self.thread.is_alive():
                print("FTP: %s detached; it closes when no longer in use" %
                      self.mountpoint)

    def fail_requests(self):
        while True:
            try:
                request = self.requests.get_nowait()
            except queue.Empty:
                return
            request.finish(error=errno.EIO)

    def process_requests(self):
        '''submit queued filesystem requests; main thread only'''
        while True:
            try:
                request = self.requests.get_nowait()
            except queue.Empty:
                return
            if request.cancelled:
                continue
            try:
                request.fn(request, *request.args)
            except Exception as error:
                print("FTP mount: %s" % error)
                request.finish(error=errno.EIO)

    # ---------------------------------------------------------------
    # main thread side: start FTP operations which complete a Request

    def target_kwargs(self, request):
        return {'target_system': self.target_system,
                'target_component': self.target_component,
                'owner': request}

    def ftp_cancel(self, request, abandoned):
        '''stop the FTP operation of a request its caller gave up on, so it
        can't complete after a later request for the same file'''
        self.module.cancel_owned(lambda owner: owner is abandoned)
        request.finish()

    def ftp_list(self, request, rpath):
        self.module.cmd_list([rpath], callback=request.finish,
                             **self.target_kwargs(request))

    def ftp_get(self, request, rpath):
        def done(fh):
            if fh is None:
                request.finish(error=errno.EIO)
            else:
                request.finish(fh.read())
        self.module.cmd_get([rpath, posixpath.basename(rpath) or 'root'],
                            callback=done, callback_progress=request.progress,
                            **self.target_kwargs(request))

    def ftp_put(self, request, rpath, data):
        def done(length):
            request.finish(error=errno.EIO if length is None else 0)
        self.module.cmd_put(['-', rpath], fh=BytesIO(data), callback=done,
                            progress_callback=request.progress,
                            **self.target_kwargs(request))

    def ftp_simple(self, request, name, args):
        '''rm, rmdir, mkdir or rename'''
        method = getattr(self.module, 'cmd_' + name)
        method(args, callback=lambda error: request.finish(error=error),
               **self.target_kwargs(request))

    # ---------------------------------------------------------------
    # filesystem thread side

    def from_service_thread(self):
        '''True if this FUSE request may have come from the thread which
        services requests, e.g. a MAVProxy command reading a file in the
        mount.  It can't wait for anything which needs that thread.  Where
        only the process is known (macOS) that means any of MAVProxy's
        threads'''
        return self.fuse_context is not None and \
            self.fuse_context()[2] == self.service_id

    @contextlib.contextmanager
    def locked(self, file):
        '''hold file.lock, refusing rather than waiting when the holder may
        be waiting for the thread asking'''
        if self.from_service_thread():
            if not file.lock.acquire(blocking=False):
                raise fs_error(errno.EDEADLK)
        else:
            file.lock.acquire()
        try:
            yield
        finally:
            file.lock.release()

    def call(self, fn, *args):
        '''run fn(request, *args) on the main thread and wait for it'''
        if self.stopping:
            raise fs_error(errno.EIO)
        if self.from_service_thread():
            raise fs_error(errno.EDEADLK)
        request = Request(fn, args, mount=self)
        self.requests.put(request)
        while not request.event.wait(0.2):
            if self.stopping:
                request.cancelled = True
                raise fs_error(errno.EIO)
            timeout = self.module.ftp_settings.mount_timeout
            if time.monotonic() - request.last_progress > timeout:
                request.cancelled = True
                # queued ahead of any retry, so the main thread stops this
                # one before starting that
                self.requests.put(Request(self.ftp_cancel, (request,)))
                raise fs_error(errno.EIO)
        if request.error:
            raise fs_error(request.error)
        return request.result

    def remote_path(self, path):
        '''vehicle path for a path within the mount'''
        path = posixpath.normpath('/' + path.lstrip('/'))
        if path == '/':
            rpath = self.remote_root
        else:
            rpath = posixpath.join(self.remote_root, path.lstrip('/'))
        try:
            encoded = rpath.encode('ascii')
        except UnicodeEncodeError:
            raise fs_error(errno.EINVAL)
        if len(encoded) > MAX_FTP_NAME:
            raise fs_error(errno.ENAMETOOLONG)
        return rpath

    @staticmethod
    def split(path):
        path = posixpath.normpath('/' + path.lstrip('/'))
        return posixpath.dirname(path), posixpath.basename(path)

    def invalidate(self, *paths):
        '''forget cached listings of these directories and anything below'''
        with self.state_lock:
            for path in paths:
                path = posixpath.normpath('/' + path.lstrip('/'))
                prefix = path.rstrip('/') + '/'
                for key in list(self.dir_cache.keys()):
                    if key == path or key.startswith(prefix):
                        del self.dir_cache[key]

    def listdir(self, path):
        '''{name: (name, is_dir, size, mtime)} for a directory'''
        path = posixpath.normpath('/' + path.lstrip('/'))
        now = time.monotonic()
        cache_time = self.module.ftp_settings.mount_cache_time
        with self.state_lock:
            cached = self.dir_cache.get(path)
            if cached is not None and now - cached[0] < cache_time:
                return cached[1]
        entries = self.call(self.ftp_list, self.remote_path(path))
        listing = {entry[0]: entry for entry in entries}
        with self.state_lock:
            self.dir_cache[path] = (time.monotonic(), listing)
        return listing

    def open_file(self, path):
        path = posixpath.normpath('/' + path.lstrip('/'))
        with self.state_lock:
            return self.files.get(path)

    def attrs(self, is_dir, size, mtime):
        if mtime:
            mtime_ns = mtime * 1000000000
        else:
            mtime_ns = self.mount_time_ns
        if is_dir:
            mode = stat.S_IFDIR | 0o755
            nlink = 2
            size = 0
        else:
            mode = stat.S_IFREG | 0o644
            nlink = 1
        return {
            'st_mode': mode,
            'st_nlink': nlink,
            'st_size': size or 0,
            'st_uid': self.uid,
            'st_gid': self.gid,
            'st_mtime': mtime_ns,
            'st_ctime': mtime_ns,
            'st_atime': mtime_ns,
        }

    def handle(self, fi):
        if fi is None:
            return None
        with self.state_lock:
            return self.handles.get(fi.fh)

    def getattr(self, path, fh=None):
        handle = self.handle(fh)
        file = handle.file if handle is not None else self.open_file(path)
        if file is not None:
            return self.attrs(False, len(file.data), 0)
        if path == '/':
            # answered locally: the mount wait in start() relies on this
            return self.attrs(True, 0, 0)
        parent, name = self.split(path)
        entry = self.listdir(parent).get(name)
        if entry is None:
            raise fs_error(errno.ENOENT)
        return self.attrs(entry[1], entry[2], entry[3])

    def access(self, path, amode):
        self.getattr(path)
        return 0

    def readdir(self, path, fh):
        names = dict(self.listdir(path))
        # files created but not yet closed are not on the vehicle yet
        prefix = path.rstrip('/') + '/'
        with self.state_lock:
            for file_path in self.files.keys():
                if file_path.startswith(prefix) and \
                   '/' not in file_path[len(prefix):]:
                    names.setdefault(file_path[len(prefix):],
                                     (None, False, 0, 0))
        result = ['.', '..']
        for name, entry in sorted(names.items()):
            mode = stat.S_IFDIR if entry[1] else stat.S_IFREG
            result.append((name, {'st_mode': mode}, 0))
        return result

    def statfs(self, path):
        return {'f_bsize': 512, 'f_frsize': 512, 'f_namemax': MAX_FTP_NAME}

    def add_handle(self, fi, file, flags):
        '''a new open of file, which the caller has counted in file.opens'''
        with self.state_lock:
            fh = self.next_handle
            self.next_handle += 1
            self.handles[fh] = Handle(file, flags)
        fi.fh = fh
        # @SYS files list a size of 0 and logs grow, so make the kernel
        # read until we return no more rather than trusting st_size
        fi.direct_io = 1
        return 0

    def attach(self, path, data=None, dirty=False, replace=False,
               on_vehicle=True):
        '''the OpenFile for path, counting one more open of it.  An open
        file is shared; otherwise one is made from data.  With replace, any
        open file at path is detached first, as the new one replaces it'''
        with self.state_lock:
            file = self.files.get(path)
            if file is not None and replace:
                file.unlinked = True
                file = None
            if file is None:
                if data is None:
                    return None
                file = OpenFile(path, data, dirty, on_vehicle)
                self.files[path] = file
            file.opens += 1
            return file

    def open(self, path, fi):
        flags = fi.flags
        rpath = self.remote_path(path)
        attrs = self.getattr(path)
        if stat.S_ISDIR(attrs['st_mode']):
            raise fs_error(errno.EISDIR)
        if flags & os.O_TRUNC:
            file = self.attach(path, b'', True)
            with self.locked(file):
                del file.data[:]
                file.dirty = True
            return self.add_handle(fi, file, flags)
        file = self.attach(path)
        if file is None:
            data = self.call(self.ftp_get, rpath)
            # another thread may have opened it while this downloaded
            file = self.attach(path, data)
        return self.add_handle(fi, file, flags)

    def create(self, path, mode, fi):
        self.remote_path(path)
        file = self.attach(path, b'', True, replace=True, on_vehicle=False)
        return self.add_handle(fi, file, fi.flags)

    def read(self, path, size, offset, fi):
        handle = self.handle(fi)
        if handle is None:
            raise fs_error(errno.EBADF)
        with self.locked(handle.file):
            return bytes(handle.file.data[offset:offset + size])

    def write(self, path, data, offset, fi):
        handle = self.handle(fi)
        if handle is None:
            raise fs_error(errno.EBADF)
        file = handle.file
        with self.locked(file):
            if handle.flags & os.O_APPEND:
                offset = len(file.data)
            if offset > len(file.data):
                file.data.extend(bytes(offset - len(file.data)))
            file.data[offset:offset + len(data)] = data
            file.dirty = True
        return len(data)

    @staticmethod
    def resize(data, length):
        if length < len(data):
            del data[length:]
        else:
            data.extend(bytes(length - len(data)))

    def truncate(self, path, length, fh=None):
        handle = self.handle(fh)
        file = handle.file if handle is not None else self.open_file(path)
        if file is not None:
            with self.locked(file):
                self.resize(file.data, length)
                file.dirty = True
            return 0
        rpath = self.remote_path(path)
        data = bytearray()
        if length != 0:
            data = bytearray(self.call(self.ftp_get, rpath))
            self.resize(data, length)
        self.call(self.ftp_put, rpath, bytes(data))
        self.invalidate(self.split(path)[0])
        return 0

    def upload(self, file):
        '''write file back to the vehicle if it has changed.  The lock is
        held throughout, so a concurrent flush waits for this upload rather
        than reporting success early'''
        with self.locked(file):
            if not file.dirty or file.unlinked:
                return
            path = file.path
            try:
                self.call(self.ftp_put, self.remote_path(path),
                          bytes(file.data))
            finally:
                self.invalidate(self.split(path)[0])
            file.dirty = False
            file.on_vehicle = True

    def flush(self, path, fi):
        handle = self.handle(fi)
        if handle is not None:
            self.upload(handle.file)
        return 0

    def fsync(self, path, datasync, fi):
        return self.flush(path, fi)

    def release(self, path, fi):
        with self.state_lock:
            handle = self.handles.pop(fi.fh, None)
        if handle is None:
            return 0
        file = handle.file
        try:
            self.upload(file)
        except OSError as error:
            print("FTP mount: upload of %s failed: %s" % (file.path, error))
        finally:
            with self.state_lock:
                file.opens -= 1
                if file.opens == 0 and self.files.get(file.path) is file:
                    del self.files[file.path]
        return 0

    def detach(self, path):
        '''path is gone from the vehicle: its open file is never written
        back, and a later create of path starts afresh'''
        with self.state_lock:
            file = self.files.pop(path, None)
            if file is not None:
                file.unlinked = True

    def locked_files(self, *paths):
        '''a context holding the locks of files open at or below paths, so
        no upload of them is in progress or starts meanwhile'''
        if self.from_service_thread():
            # what follows needs the vehicle, so would wait for this thread
            raise fs_error(errno.EDEADLK)
        with self.state_lock:
            files = [file for path, file in self.files.items()
                     if any(path == p or path.startswith(p + '/')
                            for p in paths)]
        stack = contextlib.ExitStack()
        for file in sorted(files, key=id):
            stack.enter_context(file.lock)
        return stack

    def hiding(self, old, new):
        '''True for libfuse hiding an open file it was asked to unlink'''
        parent, name = self.split(new)
        if parent != self.split(old)[0] or not HIDDEN_NAME.match(name):
            return False
        file = self.open_file(old)
        return file is not None and not file.unlinked

    def unlink(self, path):
        with self.state_lock:
            hidden = path in self.hidden
            self.hidden.discard(path)
        if hidden:
            # see hide(); only ever in memory
            self.detach(path)
            return 0
        rpath = self.remote_path(path)
        with self.locked_files(path):
            try:
                self.call(self.ftp_simple, 'rm', [rpath])
            except OSError as error:
                # a file created but not yet uploaded is only here
                if self.open_file(path) is None or \
                   error.errno != errno.ENOENT:
                    raise
            finally:
                self.invalidate(self.split(path)[0])
            self.detach(path)
        return 0

    def rmdir(self, path):
        rpath = self.remote_path(path)
        prefix = path.rstrip('/') + '/'
        with self.state_lock:
            if any(p.startswith(prefix) and not f.unlinked
                   for p, f in self.files.items()):
                # a file in it is open, perhaps not yet on the vehicle
                raise fs_error(errno.ENOTEMPTY)
        try:
            self.call(self.ftp_simple, 'rmdir', [rpath])
        finally:
            self.invalidate(self.split(path)[0], path)
        return 0

    def mkdir(self, path, mode):
        rpath = self.remote_path(path)
        try:
            self.call(self.ftp_simple, 'mkdir', [rpath])
        finally:
            self.invalidate(self.split(path)[0])
        return 0

    def check_rename(self, rold, rnew):
        '''refuse a rename the FTP request can't carry, before changing
        anything'''
        if len(rold) + 1 + len(rnew) > MAX_FTP_NAME:
            raise fs_error(errno.ENAMETOOLONG)

    def rename(self, old, new):
        if self.hiding(old, new):
            return self.hide(old, new)
        rold = self.remote_path(old)
        rnew = self.remote_path(new)
        self.check_rename(rold, rnew)
        if rold == rnew:
            return 0
        with self.locked_files(old, new):
            file = self.open_file(old)
            if file is not None and not file.on_vehicle:
                self.rename_local(old, new)
                return 0
            try:
                try:
                    self.call(self.ftp_simple, 'rename', [rold, rnew])
                except OSError as error:
                    self.replace(old, new, rold, rnew, error)
            finally:
                self.invalidate(self.split(old)[0], self.split(new)[0],
                                old, new)
            self.renamed(old, new)
        return 0

    def hide(self, old, new):
        '''libfuse's unlink of an open file, which it renames to a hidden
        name until closed.  Remove it from the vehicle, and keep it here
        under the hidden name, never to be written back, so it can still be
        read, written and stat'ed through its open handles'''
        with self.locked_files(old):
            file = self.open_file(old)
            if file is None or file.on_vehicle:
                try:
                    self.call(self.ftp_simple, 'rm', [self.remote_path(old)])
                except OSError as error:
                    if file is None or error.errno != errno.ENOENT:
                        raise
                finally:
                    self.invalidate(self.split(old)[0])
            with self.state_lock:
                self.hidden.add(new)
            if file is not None:
                file.unlinked = True
                self.renamed(old, new)
        return 0

    def rename_local(self, old, new):
        '''rename a file created here and not yet uploaded, which exists
        only in memory.  An existing file at new is overwritten when it is
        uploaded'''
        target = self.lookup(new)
        if target is not None and target[1]:
            raise fs_error(errno.EISDIR)
        self.renamed(old, new)

    def renamed(self, old, new):
        '''open files follow a rename of themselves or a parent directory'''
        with self.state_lock:
            if new in self.files:
                # replaced
                self.files.pop(new).unlinked = True
            prefix = old + '/'
            for path in list(self.files.keys()):
                if path == old:
                    moved = new
                elif path.startswith(prefix):
                    moved = new + path[len(old):]
                else:
                    continue
                file = self.files.pop(path)
                file.path = moved
                self.files[moved] = file
            for path in list(self.hidden):
                if path.startswith(prefix):
                    self.hidden.remove(path)
                    self.hidden.add(new + path[len(old):])

    def lookup(self, path):
        '''fresh listing entry for path, or None if it doesn't exist'''
        parent, name = self.split(path)
        self.invalidate(parent)
        return self.listdir(parent).get(name)

    def replace(self, old, new, rold, rnew, error):
        '''finish a rename the vehicle refused, if it refused because the
        target exists.

        POSIX rename replaces an existing target, and tools rely on it: git
        updates .git/config by renaming config.lock over it.  FATFS, used on
        most boards, refuses instead.  The refusal can't be recognised from
        its error code, because ArduPilot reads errno there and errno isn't
        thread-local on ChibiOS, so look for the target instead.

        The target is moved aside rather than removed, and moved back if
        the rename still fails, so a rename that can't work (across
        filesystems, say) loses nothing.  It isn't atomic: if the link fails
        part way, the target may be left under its temporary name.
        '''
        source = self.lookup(old)
        if source is None:
            raise error
        target = self.lookup(new)
        if target is None:
            raise error
        if target[1] and not source[1]:
            raise fs_error(errno.EISDIR)
        if source[1] and not target[1]:
            raise fs_error(errno.ENOTDIR)
        if target[1] and self.listdir(new):
            raise fs_error(errno.ENOTEMPTY)
        parent, name = self.split(new)
        aside = posixpath.join(parent, '.%s.mavftp-replaced' % name)
        raside = self.remote_path(aside)
        self.check_rename(rnew, raside)
        if self.lookup(aside) is not None:
            raise error
        self.call(self.ftp_simple, 'rename', [rnew, raside])
        try:
            self.call(self.ftp_simple, 'rename', [rold, rnew])
        except OSError:
            try:
                self.call(self.ftp_simple, 'rename', [raside, rnew])
            except OSError as restore_error:
                print("FTP mount: %s was left as %s: %s" %
                      (rnew, raside, restore_error))
            raise
        try:
            self.call(self.ftp_simple, 'rmdir' if target[1] else 'rm',
                      [raside])
        except OSError as remove_error:
            print("FTP mount: unable to remove replaced %s: %s" %
                  (raside, remove_error))

    # metadata the vehicle can't store: accept, so cp -p and editors work
    def chmod(self, path, mode):
        return 0

    def chown(self, path, uid, gid):
        return 0

    def utimens(self, path, times=None):
        return 0

    def symlink(self, target, source):
        raise fs_error(errno.ENOTSUP)

    def link(self, target, source):
        raise fs_error(errno.ENOTSUP)
