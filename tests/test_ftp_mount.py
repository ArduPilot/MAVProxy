'''tests for the "ftp mount" filesystem logic, without a real FUSE mount

The FUSE callbacks run in a thread while the test thread plays MAVProxy's
main loop, pumping process_requests() against a fake FTP module backed by a
dictionary.
'''
import errno
import importlib.util
import io
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import types
import time
import unittest
from unittest import mock


MOUNT_PATH = (Path(__file__).resolve().parents[1] /
              'MAVProxy/modules/lib/ftp_mount.py')
MOUNT_SPEC = importlib.util.spec_from_file_location(
    'ftp_mount_under_test', MOUNT_PATH)
ftp_mount = importlib.util.module_from_spec(MOUNT_SPEC)
MOUNT_SPEC.loader.exec_module(ftp_mount)


class FakeFTPModule:
    '''the FTPModule methods FTPMount uses, over an in-memory vehicle'''

    def __init__(self):
        self.ftp_settings = types.SimpleNamespace(
            mount_cache_time=60.0, mount_timeout=5.0)
        self.files = {'/APM/a.txt': b'hello', '/APM/LOGS/1.BIN': b'log'}
        self.dirs = {'/', '/APM', '/APM/LOGS'}
        self.calls = []
        self.targets = set()
        self.respond = True
        # FATFS refuses to rename onto an existing name, and the errno
        # ArduPilot reports for it isn't reliable
        self.fatfs_rename = False
        # sources a rename always fails for, as across ArduPilot backends
        self.exdev_prefix = None
        # uploads awaiting release by the test: (path, data, callback)
        self.held_puts = None
        # operations not yet answered: [(owner, failure callback)]
        self.outstanding = []

    def note(self, name, args, kwargs):
        self.calls.append((name, list(args)))
        self.targets.add((kwargs.get('target_system'),
                          kwargs.get('target_component')))

    def cmd_list(self, args, callback=None, **kwargs):
        self.note('list', args, kwargs)
        path = args[0]
        if path not in self.dirs:
            callback(None, errno.ENOENT)
            return
        prefix = path.rstrip('/') + '/'
        entries = []
        for d in self.dirs:
            if d != path and d.startswith(prefix) and '/' not in d[len(prefix):]:
                entries.append((d[len(prefix):], True, None, 100))
        for f, data in self.files.items():
            if f.startswith(prefix) and '/' not in f[len(prefix):]:
                entries.append((f[len(prefix):], False, len(data), 200))
        callback(entries, 0)

    def cancel_owned(self, predicate):
        for item in list(self.outstanding):
            owner, fail = item
            if predicate(owner):
                self.calls.append(('cancel', []))
                self.outstanding.remove(item)
                fail()

    def cmd_get(self, args, callback=None, callback_progress=None, **kwargs):
        self.note('get', args, kwargs)
        if not self.respond:
            self.outstanding.append((kwargs.get('owner'),
                                     lambda: callback(None)))
            return
        data = self.files.get(args[0])
        callback(None if data is None else io.BytesIO(data))

    def cmd_put(self, args, fh=None, callback=None, progress_callback=None,
                **kwargs):
        self.note('put', args, kwargs)
        if args[1].startswith('/@SYS'):
            callback(None)
            return
        if self.held_puts is not None:
            self.held_puts.append((args[1], fh.read(), callback))
            self.outstanding.append((kwargs.get('owner'),
                                     lambda: callback(None)))
            return
        self.files[args[1]] = fh.read()
        callback(len(self.files[args[1]]))

    def cmd_rm(self, args, callback=None, **kwargs):
        self.note('rm', args, kwargs)
        callback(0 if self.files.pop(args[0], None) is not None
                 else errno.ENOENT)

    def cmd_mkdir(self, args, callback=None, **kwargs):
        self.note('mkdir', args, kwargs)
        self.dirs.add(args[0])
        callback(0)

    def cmd_rmdir(self, args, callback=None, **kwargs):
        self.note('rmdir', args, kwargs)
        prefix = args[0] + '/'
        if any(p.startswith(prefix) for p in list(self.files) + list(self.dirs)):
            callback(errno.ENOTEMPTY)
            return
        self.dirs.discard(args[0])
        callback(0)

    def cmd_rename(self, args, callback=None, **kwargs):
        self.note('rename', args, kwargs)
        old, new = args
        if old not in self.files and old not in self.dirs:
            callback(errno.ENOENT)
            return
        if self.exdev_prefix is not None and old.startswith(self.exdev_prefix):
            callback(errno.EXDEV)
            return
        if self.fatfs_rename and (new in self.files or new in self.dirs):
            callback(errno.EIO)
            return
        if old in self.files:
            self.files[new] = self.files.pop(old)
        else:
            prefix = old + '/'
            for d in [d for d in self.dirs if d == old or d.startswith(prefix)]:
                self.dirs.discard(d)
                self.dirs.add(new + d[len(old):])
            for f in [f for f in self.files if f.startswith(prefix)]:
                self.files[new + f[len(old):]] = self.files.pop(f)
        callback(0)


def fi(flags=os.O_RDONLY):
    return types.SimpleNamespace(flags=flags, fh=0, direct_io=0)


class TestFTPMount(unittest.TestCase):
    def setUp(self):
        self.module = FakeFTPModule()
        self.mount = ftp_mount.FTPMount(self.module, '/nonexistent', '/',
                                        target_system=1, target_component=2)

    def run_op(self, method, *args):
        '''call a filesystem operation as FUSE would, servicing it here'''
        result = {}

        def target():
            try:
                result['value'] = getattr(self.mount, method)(*args)
            except OSError as error:
                result['errno'] = error.errno

        thread = threading.Thread(target=target)
        thread.start()
        while thread.is_alive():
            self.mount.process_requests()
            thread.join(0.005)
        return result

    def value(self, method, *args):
        result = self.run_op(method, *args)
        self.assertNotIn('errno', result, (method, args))
        return result['value']

    def error(self, method, *args):
        result = self.run_op(method, *args)
        self.assertIn('errno', result, (method, args))
        return result['errno']

    def ops(self):
        return [name for name, _ in self.module.calls]

    def test_root_attributes_need_no_main_thread(self):
        # start() waits for the mount on the main thread, so this must not
        # queue a request
        attrs = self.mount.getattr('/')
        self.assertTrue(stat.S_ISDIR(attrs['st_mode']))
        self.assertTrue(self.mount.requests.empty())

    def test_readdir_getattr_and_cache(self):
        names = [n if isinstance(n, str) else n[0]
                 for n in self.value('readdir', '/APM', 0)]
        self.assertEqual(names, ['.', '..', 'LOGS', 'a.txt'])
        attrs = self.value('getattr', '/APM/a.txt')
        self.assertTrue(stat.S_ISREG(attrs['st_mode']))
        self.assertEqual(attrs['st_size'], 5)
        self.assertEqual(attrs['st_mtime'], 200 * 1000000000)
        self.assertTrue(stat.S_ISDIR(self.value('getattr', '/APM/LOGS')['st_mode']))
        # both answered from the one listing of /APM
        self.assertEqual(self.module.calls, [('list', ['/APM'])])
        self.assertEqual(self.error('getattr', '/APM/missing'), errno.ENOENT)
        self.assertEqual(self.ops(), ['list'])
        self.assertEqual(self.module.targets, {(1, 2)})

        self.module.ftp_settings.mount_cache_time = 0
        self.value('getattr', '/APM/a.txt')
        self.assertEqual(self.ops(), ['list', 'list'])

    def test_missing_directory(self):
        self.assertEqual(self.error('readdir', '/nope', 0), errno.ENOENT)

    def test_open_read(self):
        info = fi()
        self.value('open', '/APM/a.txt', info)
        self.assertEqual(info.direct_io, 1)
        self.assertEqual(self.value('read', '/APM/a.txt', 3, 1, info), b'ell')
        self.assertEqual(self.value('read', '/APM/a.txt', 10, 5, info), b'')
        self.value('release', '/APM/a.txt', info)
        self.assertEqual(self.ops(), ['list', 'get'])
        self.assertEqual(self.mount.handles, {})
        self.assertEqual(self.mount.files, {})

    def test_open_directory_is_refused(self):
        self.assertEqual(self.error('open', '/APM/LOGS', fi()), errno.EISDIR)

    def test_create_write_flush_uploads(self):
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/new.txt', 0o644, info)
        # visible before it reaches the vehicle
        self.assertEqual(self.value('getattr', '/APM/new.txt')['st_size'], 0)
        names = [n if isinstance(n, str) else n[0]
                 for n in self.value('readdir', '/APM', 0)]
        self.assertIn('new.txt', names)
        self.assertEqual(self.value('write', '/APM/new.txt', b'abc', 0, info), 3)
        self.assertEqual(self.value('getattr', '/APM/new.txt', info)['st_size'], 3)
        self.value('flush', '/APM/new.txt', info)
        self.assertEqual(self.module.files['/APM/new.txt'], b'abc')
        # nothing new to send on the second flush or at release
        self.value('flush', '/APM/new.txt', info)
        self.value('release', '/APM/new.txt', info)
        self.assertEqual(self.ops().count('put'), 1)
        # the listing was refreshed after the upload
        self.module.calls = []
        self.assertEqual(self.value('getattr', '/APM/new.txt')['st_size'], 3)
        self.assertEqual(self.ops(), ['list'])

    def test_empty_create_still_uploads(self):
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/empty', 0o644, info)
        self.value('flush', '/APM/empty', info)
        self.assertEqual(self.module.files['/APM/empty'], b'')

    def test_append_and_overwrite(self):
        info = fi(os.O_WRONLY | os.O_APPEND)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'!', 0, info)
        self.value('release', '/APM/a.txt', info)
        self.assertEqual(self.module.files['/APM/a.txt'], b'hello!')

        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'J', 0, info)
        self.value('write', '/APM/a.txt', b'Z', 8, info)
        self.value('release', '/APM/a.txt', info)
        self.assertEqual(self.module.files['/APM/a.txt'], b'Jello!\0\0Z')

    def test_o_trunc_skips_download(self):
        info = fi(os.O_WRONLY | os.O_TRUNC)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'x', 0, info)
        self.value('release', '/APM/a.txt', info)
        self.assertNotIn('get', self.ops())
        self.assertEqual(self.module.files['/APM/a.txt'], b'x')

    def test_truncate(self):
        self.value('truncate', '/APM/a.txt', 2)
        self.assertEqual(self.module.files['/APM/a.txt'], b'he')
        self.value('truncate', '/APM/a.txt', 0)
        self.assertEqual(self.module.files['/APM/a.txt'], b'')
        self.assertEqual(self.ops().count('get'), 1)

        info = fi(os.O_RDWR)
        self.value('open', '/APM/LOGS/1.BIN', info)
        self.value('truncate', '/APM/LOGS/1.BIN', 5)
        self.value('release', '/APM/LOGS/1.BIN', info)
        self.assertEqual(self.module.files['/APM/LOGS/1.BIN'], b'log\0\0')

    def test_failed_upload_is_reported_at_flush(self):
        self.module.dirs.add('/@SYS')
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/@SYS/x', 0o644, info)
        self.value('write', '/@SYS/x', b'x', 0, info)
        self.assertEqual(self.error('flush', '/@SYS/x', info), errno.EIO)

    def test_namespace_operations(self):
        self.value('getattr', '/APM/a.txt')
        self.value('rename', '/APM/a.txt', '/APM/b.txt')
        self.assertEqual(self.error('getattr', '/APM/a.txt'), errno.ENOENT)
        self.assertEqual(self.value('getattr', '/APM/b.txt')['st_size'], 5)
        self.value('unlink', '/APM/b.txt')
        self.assertEqual(self.error('getattr', '/APM/b.txt'), errno.ENOENT)
        self.assertEqual(self.error('unlink', '/APM/b.txt'), errno.ENOENT)
        self.value('mkdir', '/APM/d', 0o755)
        self.assertTrue(stat.S_ISDIR(self.value('getattr', '/APM/d')['st_mode']))
        self.value('rmdir', '/APM/d')
        self.assertEqual(self.error('getattr', '/APM/d'), errno.ENOENT)

    def test_rename_replaces_existing_target_on_fatfs(self):
        self.module.fatfs_rename = True
        self.module.files['/APM/config'] = b'old'
        self.module.files['/APM/config.lock'] = b'new'
        self.value('getattr', '/APM/config')
        self.value('rename', '/APM/config.lock', '/APM/config')
        self.assertEqual(self.module.files['/APM/config'], b'new')
        self.assertNotIn('/APM/config.lock', self.module.files)
        self.assertEqual(self.value('getattr', '/APM/config')['st_size'], 3)
        self.assertEqual(self.error('getattr', '/APM/config.lock'), errno.ENOENT)
        self.assertEqual(
            [c for c in self.module.calls if c[0] != 'list'],
            [('rename', ['/APM/config.lock', '/APM/config']),
             ('rename', ['/APM/config', '/APM/.config.mavftp-replaced']),
             ('rename', ['/APM/config.lock', '/APM/config']),
             ('rm', ['/APM/.config.mavftp-replaced'])])

    def test_rename_that_cannot_work_keeps_target(self):
        self.module.fatfs_rename = True
        self.module.exdev_prefix = '/@ROMFS/'
        self.module.dirs.add('/@ROMFS')
        self.module.files['/@ROMFS/x.lua'] = b'rom'
        self.module.files['/APM/x.lua'] = b'mine'
        self.assertEqual(self.error('rename', '/@ROMFS/x.lua', '/APM/x.lua'),
                         errno.EXDEV)
        self.assertEqual(self.module.files['/APM/x.lua'], b'mine')
        self.assertEqual(self.module.files['/@ROMFS/x.lua'], b'rom')
        self.assertEqual(sorted(self.module.files),
                         ['/@ROMFS/x.lua', '/APM/LOGS/1.BIN', '/APM/a.txt',
                          '/APM/x.lua'])

    def test_rename_names_too_long_together_change_nothing(self):
        self.module.fatfs_rename = True
        old = '/APM/' + 'o' * 115
        new = '/APM/' + 'n' * 115
        self.module.files[old] = b'old'
        self.module.files[new] = b'new'
        self.assertEqual(self.error('rename', old, new), errno.ENAMETOOLONG)
        self.assertEqual(self.module.calls, [])
        self.assertEqual(self.module.files[new], b'new')

    def test_rename_onto_empty_directory_on_fatfs(self):
        self.module.fatfs_rename = True
        self.module.dirs.update({'/APM/d1', '/APM/d2'})
        self.value('rename', '/APM/d1', '/APM/d2')
        self.assertNotIn('/APM/d1', self.module.dirs)
        self.assertIn('/APM/d2', self.module.dirs)

    def test_rename_does_not_replace_across_types_or_nonempty(self):
        self.module.fatfs_rename = True
        self.module.dirs.add('/APM/d')
        self.assertEqual(self.error('rename', '/APM/a.txt', '/APM/LOGS'), errno.EISDIR)
        self.assertEqual(self.error('rename', '/APM/d', '/APM/a.txt'), errno.ENOTDIR)
        self.assertEqual(self.error('rename', '/APM/d', '/APM/LOGS'), errno.ENOTEMPTY)
        self.assertIn('/APM/LOGS/1.BIN', self.module.files)
        self.assertIn('/APM/a.txt', self.module.files)
        self.assertNotIn('rm', self.ops())

    def test_rename_failure_without_existing_target_is_reported(self):
        self.module.fatfs_rename = True
        self.assertEqual(self.error('rename', '/APM/missing', '/APM/x'), errno.ENOENT)
        self.assertNotIn('rm', self.ops())

    def test_opens_of_a_file_share_one_buffer(self):
        first = fi(os.O_RDWR)
        second = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', first)
        self.value('open', '/APM/a.txt', second)
        self.assertEqual(self.ops().count('get'), 1)
        self.value('write', '/APM/a.txt', b'A', 0, first)
        self.assertEqual(self.value('read', '/APM/a.txt', 5, 0, second), b'Aello')
        self.value('write', '/APM/a.txt', b'B', 1, second)
        self.value('flush', '/APM/a.txt', first)
        self.value('flush', '/APM/a.txt', second)
        self.assertEqual(self.module.files['/APM/a.txt'], b'ABllo')
        self.assertEqual(self.ops().count('put'), 1)
        self.value('release', '/APM/a.txt', first)
        self.assertIn('/APM/a.txt', self.mount.files)
        self.value('release', '/APM/a.txt', second)
        self.assertEqual(self.mount.files, {})

    def test_open_file_follows_rename(self):
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'J', 0, info)
        self.value('rename', '/APM/a.txt', '/APM/b.txt')
        self.value('release', '/APM/b.txt', info)
        self.assertNotIn('/APM/a.txt', self.module.files)
        self.assertEqual(self.module.files['/APM/b.txt'], b'Jello')

        # and a rename of a directory above it
        info = fi(os.O_RDWR)
        self.value('open', '/APM/LOGS/1.BIN', info)
        self.value('write', '/APM/LOGS/1.BIN', b'L', 0, info)
        self.value('rename', '/APM/LOGS', '/APM/OLD')
        self.value('release', '/APM/OLD/1.BIN', info)
        self.assertNotIn('/APM/LOGS/1.BIN', self.module.files)
        self.assertEqual(self.module.files['/APM/OLD/1.BIN'], b'Log')

    def test_replaced_open_file_is_not_written_back(self):
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'J', 0, info)
        self.module.files['/APM/b.txt'] = b'b'
        self.value('rename', '/APM/b.txt', '/APM/a.txt')
        self.value('release', '/APM/a.txt', info)
        self.assertEqual(self.module.files['/APM/a.txt'], b'b')

    def test_unlinked_open_file_is_not_written_back(self):
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'J', 0, info)
        self.value('unlink', '/APM/a.txt')
        self.assertEqual(self.error('getattr', '/APM/a.txt'), errno.ENOENT)
        # reads still work through the open file
        self.assertEqual(self.value('read', '/APM/a.txt', 5, 0, info), b'Jello')
        # a new file of the same name is separate
        new = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/a.txt', 0o644, new)
        self.value('write', '/APM/a.txt', b'new', 0, new)
        self.value('flush', '/APM/a.txt', info)
        self.value('release', '/APM/a.txt', info)
        self.assertNotIn('/APM/a.txt', self.module.files)
        self.value('release', '/APM/a.txt', new)
        self.assertEqual(self.module.files['/APM/a.txt'], b'new')

    def test_libfuse_hiding_an_open_file_unlinks_it(self):
        # libfuse unlinks an open file by renaming it to .fuse_hidden*, and
        # unlinks that when the file is closed
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'J', 0, info)
        hidden = '/APM/.fuse_hidden0000000100000001'
        self.assertEqual(self.error('getattr', hidden), errno.ENOENT)
        self.value('rename', '/APM/a.txt', hidden)
        self.assertNotIn('/APM/a.txt', self.module.files)
        self.assertEqual(self.error('getattr', '/APM/a.txt'), errno.ENOENT)
        self.assertEqual(self.value('getattr', hidden)['st_size'], 5)
        self.assertEqual(self.value('read', hidden, 5, 0, info), b'Jello')
        self.value('write', hidden, b'!', 5, info)
        self.value('flush', hidden, info)
        self.value('release', hidden, info)
        self.value('unlink', hidden)
        self.assertEqual(sorted(self.module.files), ['/APM/LOGS/1.BIN'])
        self.assertEqual([c for c in self.module.calls if c[0] in ('put', 'rename')], [])

        # a created file never uploaded just disappears
        self.module.dirs.add('/APM/d')
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/d/tmp', 0o644, info)
        self.value('rename', '/APM/d/tmp', '/APM/d/.fuse_hidden0000000200000002')
        # and doesn't keep its directory from being removed
        self.value('rmdir', '/APM/d')
        self.value('release', '/APM/d/.fuse_hidden0000000200000002', info)
        self.value('unlink', '/APM/d/.fuse_hidden0000000200000002')
        self.assertNotIn('put', self.ops())

    def test_names_like_hidden_ones_are_ordinary_files(self):
        # only libfuse's own name for an open file it is unlinking is
        # treated as hiding it
        self.value('rename', '/APM/a.txt', '/APM/.fuse_hidden_backup')
        self.assertEqual(self.module.files['/APM/.fuse_hidden_backup'], b'hello')
        self.assertNotIn('/APM/a.txt', self.module.files)
        # libfuse's form, but the file isn't open
        hidden = '/APM/.fuse_hidden0000000300000003'
        self.value('rename', '/APM/.fuse_hidden_backup', hidden)
        self.assertEqual(self.module.files[hidden], b'hello')
        # open, but not renamed within its directory
        info = fi(os.O_RDONLY)
        self.value('open', hidden, info)
        self.value('rename', hidden, '/APM/LOGS/.fuse_hidden0000000400000004')
        self.assertEqual(
            self.module.files['/APM/LOGS/.fuse_hidden0000000400000004'], b'hello')
        self.value('release', '/APM/LOGS/.fuse_hidden0000000400000004', info)
        # and an unlink of one is an unlink on the vehicle
        self.value('unlink', '/APM/LOGS/.fuse_hidden0000000400000004')
        self.assertEqual(sorted(self.module.files), ['/APM/LOGS/1.BIN'])

    def test_unlink_of_file_not_yet_uploaded(self):
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/tmp', 0o644, info)
        self.value('write', '/APM/tmp', b'x', 0, info)
        self.value('unlink', '/APM/tmp')
        self.value('release', '/APM/tmp', info)
        self.assertNotIn('/APM/tmp', self.module.files)
        self.assertNotIn('put', self.ops())

    def test_flush_waits_for_upload_in_flight(self):
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'X', 0, info)
        self.module.held_puts = []
        results = []

        def flush(name):
            try:
                results.append((name, self.mount.flush('/APM/a.txt', info)))
            except OSError as error:
                results.append((name, error.errno))

        first = threading.Thread(target=flush, args=('first',))
        first.start()
        while not self.module.held_puts:
            self.mount.process_requests()
            first.join(0.005)
        second = threading.Thread(target=flush, args=('second',))
        second.start()
        for _ in range(20):
            self.mount.process_requests()
            second.join(0.005)
        # the second flush must not report success before the upload has
        self.assertEqual(results, [])
        held = self.module.held_puts
        self.module.held_puts = None
        path, data, callback = held.pop()
        callback(None)
        for thread in (first, second):
            while thread.is_alive():
                self.mount.process_requests()
                thread.join(0.005)
        # that upload failed, so the second flush sends it again
        self.assertEqual(results, [('first', errno.EIO), ('second', 0)])
        self.assertEqual(self.ops().count('put'), 2)
        self.assertEqual(self.module.files['/APM/a.txt'], b'Xello')

    def test_servicing_thread_is_refused(self):
        # that thread services requests, so it would wait for itself
        self.mount.service_id = 1234
        self.mount.fuse_context = lambda: (0, 0, 1234)
        self.assertEqual(self.error('open', '/APM/a.txt', fi()), errno.EDEADLK)
        self.assertEqual(self.module.calls, [])
        self.mount.fuse_context = lambda: (0, 0, 1235)
        self.value('open', '/APM/a.txt', fi())

    def test_servicing_thread_does_not_wait_for_a_file_lock(self):
        # an upload holding the lock waits for the servicing thread
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        file = self.mount.open_file('/APM/a.txt')
        self.mount.service_id = 1234
        self.mount.fuse_context = lambda: (0, 0, 1234)
        with file.lock:
            started = time.monotonic()
            for op, args in (('read', ('/APM/a.txt', 5, 0, info)),
                             ('write', ('/APM/a.txt', b'x', 0, info)),
                             ('flush', ('/APM/a.txt', info)),
                             ('unlink', ('/APM/a.txt',)),
                             ('rename', ('/APM/a.txt', '/APM/b.txt'))):
                self.assertEqual(self.error(op, *args), errno.EDEADLK, op)
            self.assertLess(time.monotonic() - started, 1)
        self.assertEqual(self.value('read', '/APM/a.txt', 5, 0, info), b'hello')

    def test_caller_id_is_the_process_where_threads_are_not_known(self):
        # a request from any of MAVProxy's threads is refused there, rather
        # than waiting on a file lock with no timeout
        with mock.patch.object(sys, 'platform', 'darwin'):
            self.assertEqual(ftp_mount.caller_id(), os.getpid())
        with mock.patch.object(sys, 'platform', 'linux'):
            self.assertEqual(ftp_mount.caller_id(), threading.get_native_id())

    def test_timed_out_request_is_cancelled_before_a_retry(self):
        self.module.ftp_settings.mount_timeout = 0.2
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'O', 0, info)
        self.module.held_puts = []
        self.assertEqual(self.error('flush', '/APM/a.txt', info), errno.EIO)
        self.module.held_puts = None
        self.value('write', '/APM/a.txt', b'N', 0, info)
        self.value('flush', '/APM/a.txt', info)
        # the stalled upload was stopped before the retry started, so it
        # can't finish later and overwrite it
        self.assertEqual(self.ops()[-3:], ['put', 'cancel', 'put'])
        self.assertEqual(self.module.outstanding, [])
        self.assertEqual(self.module.files['/APM/a.txt'], b'Nello')

    def test_stop_cancels_operations_in_progress(self):
        info = fi(os.O_RDWR)
        self.value('open', '/APM/a.txt', info)
        self.value('write', '/APM/a.txt', b'X', 0, info)
        self.module.held_puts = []
        result = {}

        def flush():
            try:
                self.mount.flush('/APM/a.txt', info)
            except OSError as error:
                result['errno'] = error.errno

        thread = threading.Thread(target=flush)
        thread.start()
        while not self.module.held_puts:
            self.mount.process_requests()
            thread.join(0.005)
        self.mount.stop()
        thread.join(2)
        self.assertEqual(result, {'errno': errno.EIO})
        self.assertIn('cancel', self.ops())
        self.assertEqual(self.module.outstanding, [])

    def test_rename_of_file_not_yet_uploaded(self):
        # e.g. an editor writing a temporary file and renaming it into place
        self.module.files['/APM/c.cfg'] = b'old'
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/c.tmp', 0o644, info)
        self.value('write', '/APM/c.tmp', b'new', 0, info)
        self.value('rename', '/APM/c.tmp', '/APM/c.cfg')
        self.assertEqual(self.error('getattr', '/APM/c.tmp'), errno.ENOENT)
        self.assertEqual(self.value('getattr', '/APM/c.cfg')['st_size'], 3)
        self.value('release', '/APM/c.cfg', info)
        self.assertEqual(self.module.files['/APM/c.cfg'], b'new')
        self.assertNotIn('/APM/c.tmp', self.module.files)
        self.assertNotIn('rename', self.ops())

        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/d.tmp', 0o644, info)
        self.assertEqual(self.error('rename', '/APM/d.tmp', '/APM/LOGS'),
                         errno.EISDIR)

    def test_rmdir_with_open_file_not_yet_uploaded(self):
        self.module.dirs.add('/APM/d')
        info = fi(os.O_WRONLY | os.O_CREAT)
        self.value('create', '/APM/d/new', 0o644, info)
        self.assertEqual(self.error('rmdir', '/APM/d'), errno.ENOTEMPTY)
        self.assertNotIn('rmdir', self.ops())
        self.value('release', '/APM/d/new', info)
        self.assertEqual(self.module.files['/APM/d/new'], b'')

    def test_start_options_and_working_directory(self):
        recorded = {}
        finished = threading.Event()

        def FUSE(operations, mountpoint, **options):
            recorded.update(options)
            os.chdir('/')
            operations.init('/')
            finished.wait(5)

        fake_mfusepy = types.SimpleNamespace(
            FUSE=FUSE, fuse_get_context=lambda: (0, 0, 0))
        original = os.getcwd()
        with tempfile.TemporaryDirectory() as directory:
            directory = os.path.realpath(directory)
            mount = ftp_mount.FTPMount(self.module,
                                       os.path.join(directory, 'mnt'))
            try:
                os.chdir(directory)
                with mock.patch.dict(sys.modules, {'mfusepy': fake_mfusepy}), \
                     mock.patch.object(os.path, 'ismount', return_value=True), \
                     mock.patch.object(ftp_mount.FTPMount, 'unmount_commands',
                                       return_value=[['true']]):
                    mount.start()
                    self.assertEqual(os.getcwd(), directory)
                    finished.set()
                    mount.stop()
            finally:
                finished.set()
                os.chdir(original)
        self.assertTrue(recorded['foreground'])

    def test_init_restores_working_directory(self):
        # libfuse changes it to / when it starts
        original = os.getcwd()
        with tempfile.TemporaryDirectory() as directory:
            directory = os.path.realpath(directory)
            try:
                os.chdir(directory)
                self.mount.saved_cwd = os.open('.', os.O_RDONLY)
                os.chdir('/')
                self.mount.init('/')
                self.assertEqual(os.getcwd(), directory)
                self.assertTrue(self.mount.initialised.is_set())
                self.assertIsNone(self.mount.saved_cwd)
            finally:
                os.chdir(original)

    def test_remote_root_and_names(self):
        mount = ftp_mount.FTPMount(self.module, '/x', '//APM/')
        self.assertEqual(mount.remote_root, '/APM')
        self.assertEqual(mount.remote_path('/'), '/APM')
        self.assertEqual(mount.remote_path('/LOGS/1.BIN'), '/APM/LOGS/1.BIN')
        self.assertEqual(ftp_mount.FTPMount(self.module, '/x', '//').remote_path('/a'), '/a')
        with self.assertRaises(OSError) as raised:
            mount.remote_path('/' + 'x' * 300)
        self.assertEqual(raised.exception.errno, errno.ENAMETOOLONG)
        with self.assertRaises(OSError) as raised:
            mount.remote_path('/é')
        self.assertEqual(raised.exception.errno, errno.EINVAL)

    def test_subtree_mount_lists_remote_root(self):
        self.mount = ftp_mount.FTPMount(self.module, '/x', '/APM/LOGS')
        names = [n if isinstance(n, str) else n[0]
                 for n in self.value('readdir', '/', 0)]
        self.assertEqual(names, ['.', '..', '1.BIN'])
        self.assertEqual(self.module.calls, [('list', ['/APM/LOGS'])])

    def test_no_progress_times_out(self):
        self.module.respond = False
        self.module.ftp_settings.mount_timeout = 0.2
        self.assertEqual(self.error('open', '/APM/a.txt', fi()), errno.EIO)

    def test_progress_postpones_timeout(self):
        self.module.ftp_settings.mount_timeout = 0.2
        self.module.respond = False
        result = {}

        def target():
            try:
                result['value'] = self.mount.call(lambda request: None)
            except OSError as error:
                result['errno'] = error.errno

        thread = threading.Thread(target=target)
        thread.start()
        while self.mount.requests.empty():
            thread.join(0.005)
        request = self.mount.requests.get()
        for _ in range(10):
            request.progress()
            thread.join(0.05)
        self.assertTrue(thread.is_alive())
        request.finish(b'done')
        thread.join()
        self.assertEqual(result, {'value': b'done'})

    def test_stop_fails_waiting_requests(self):
        result = {}

        def target():
            try:
                self.mount.call(lambda request: None)
            except OSError as error:
                result['errno'] = error.errno

        thread = threading.Thread(target=target)
        thread.start()
        while self.mount.requests.empty():
            thread.join(0.005)
        self.mount.stop()
        thread.join(2)
        self.assertEqual(result, {'errno': errno.EIO})
        with self.assertRaises(OSError):
            self.mount.call(lambda request: None)

    def test_no_attribute_shadows_a_libfuse_operation(self):
        # mfusepy wires up every attribute named like a libfuse operation
        fuse_ops = {'lock', 'poll', 'ioctl', 'bmap', 'flock', 'fallocate',
                    'read_buf', 'write_buf', 'opendir', 'releasedir',
                    'fsyncdir', 'getdir', 'utime', 'init', 'destroy'}
        attributes = set(vars(self.mount))
        self.assertEqual(attributes & fuse_ops, set())


if __name__ == '__main__':
    unittest.main()
