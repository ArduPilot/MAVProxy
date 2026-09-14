#!/usr/bin/env python3
'''
param editor module
Akshath Singhal
June 2019
'''

import platform
import os
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.mavproxy_paramedit import ph_event
import threading
import queue
from pymavlink import mavutil
import time
ParamEditorEvent = ph_event.ParamEditorEvent


class ParamEditorEventThread(threading.Thread):
    def __init__(self, mp_paramedit, queue, lock):
        threading.Thread.__init__(self)
        self.mp_paramedit = mp_paramedit
        self.event_queue = queue
        self.event_queue_lock = lock
        self.time_to_quit = False
        self.queue_access_timeout = 0.5

    def module(self, name):
        '''access another module'''
        return self.mp_paramedit.mpstate.module(name)

    def master(self):
        '''access master mavlink connection'''
        return self.mp_paramedit.mpstate.master()

    def run(self):
        while not self.time_to_quit:
            while not self.time_to_quit and not self.event_queue.empty():
                try:
                    event = self.event_queue.get(block=False)
                    event_type = event.get_type()

                    if event_type == ph_event.PEE_READ_PARAM:
                        self.param_received = self.module('param').mav_param
                        self.mp_paramedit.gui_event_queue.put(ParamEditorEvent(
                            ph_event.PEGE_READ_PARAM, param=self.param_received, vehicle=self.mp_paramedit.mpstate.vehicle_name))

                    elif event_type == ph_event.PEE_TIME_TO_QUIT:
                        self.mp_paramedit.needs_unloading = True

                    elif event_type == ph_event.PEE_LOAD_FILE:
                        self.module('param').mav_param.load(
                                event.get_arg("path"))
                        self.param_received = self.module('param').mav_param
                        self.mp_paramedit.gui_event_queue.put(ParamEditorEvent(
                            ph_event.PEGE_READ_PARAM, param=self.param_received))

                    elif event_type == ph_event.PEE_SAVE_FILE:
                        self.module('param').mav_param.save(
                                event.get_arg("path"), verbose=True)

                    elif event_type == ph_event.PEE_WRITE_PARAM:
                        self.mp_paramedit.paramchanged = event.get_arg("modparam")
                        self.mp_paramedit.set_params()

                    elif event_type == ph_event.PEE_RESET:
                        master = self.mp_paramedit.mpstate.mav_master[0]
                        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 2.0, 0, 0, 0, 0, 0, 0)

                    elif event_type == ph_event.PEE_FETCH:
                        self.module('param').fetch_all()

                except Exception:
                    time.sleep(0.2)
            time.sleep(0.01)


class ParamEditorMain(object):
    def __init__(self, mpstate):
        self.param_received = {}
        self.paramchanged = {}
        self.fltmode_rc = None
        self.mpstate = mpstate
        self.needs_unloading = False
        self.time_to_quit = False
        self.closed = False
        self.child = None
        self.event_thread = None
        self.mavlink_thread = None
        self.event_queue = None
        self.gui_event_queue = None
        self.close_window = None
        # This queue is only used by threads in the parent process.
        self.mavlink_message_queue = queue.Queue()
        try:
            self.start()
        except Exception:
            try:
                self.close()
            except Exception as ex:
                print('Parameter editor cleanup failed: %s' % ex)
            raise
        self.mpstate.param_editor = self

    def start(self):
        # The Windows GUI runs in this process and needs no pipe feeders.
        self.threaded = platform.system() == 'Windows'
        queue_class = queue.Queue if self.threaded else multiproc.Queue
        self.event_queue = queue_class()
        self.event_queue_lock = multiproc.Lock()
        self.gui_event_queue = queue_class()
        self.gui_event_queue_lock = multiproc.Lock()

        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()

        if self.threaded:
            child_class = threading.Thread
        else:
            child_class = multiproc.Process
        # Spawn/forkserver must not pickle the editor or the MAVProxy state.
        self.child = child_class(
            target=self.child_task,
            args=(self.event_queue, self.event_queue_lock,
                  self.gui_event_queue, self.gui_event_queue_lock,
                  self.close_window, self.mpstate.vehicle_name,
                  self.mpstate.settings.moddebug,
                  dict(self.mpstate.module('param').mav_param)))

        self.child.start()

        self.event_thread = ParamEditorEventThread(
                            self, self.event_queue, self.event_queue_lock)
        self.event_thread.start()

        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1  # seconds

        self.mavlink_thread = threading.Thread(
            target=self.mavlink_message_queue_handler)
        self.mavlink_thread.start()

    def mavlink_message_queue_handler(self):
        while not self.time_to_quit:
            while True:
                if self.time_to_quit:
                    return
                if not self.mavlink_message_queue.empty():
                    break
                time.sleep(0.1)
            m = self.mavlink_message_queue.get()
            try:
                self.process_mavlink_packet(m)
            except Exception as ex:
                print(ex)
                import traceback
                traceback.print_stack()

    def unload(self):
        '''unload module'''
        self.close()

    def idle_task(self):
        now = time.time()
        if now - self.last_unload_check_time > self.unload_check_interval:
            self.last_unload_check_time = now
            if not self.child.is_alive():
                self.needs_unloading = True

    def mavlink_packet(self, m):
        if m.get_type() in ['PARAM_VALUE', 'RC_CHANNELS', 'RC_CHANNELS_RAW']:
            self.mavlink_message_queue.put(m)

    def process_mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype == 'PARAM_VALUE':
            if m.param_id in self.paramchanged:
                del self.paramchanged[m.param_id.upper()]
            self.gui_event_queue.put(ParamEditorEvent(
                ph_event.PEGE_WRITE_SUCC, paramid=m.param_id.upper(), paramvalue=m.param_value, pstatus = self.mpstate.module('param').param_status()))
        if mtype in ['RC_CHANNELS_RAW', 'RC_CHANNELS']:
            if self.mpstate.vehicle_name == 'APMrover2':
                fltmode_ch = int(self.mpstate.module('param').mav_param['MODE_CH'])
            else:
                if self.mpstate.vehicle_name.lower().find('copter') != -1:
                    default_channel = 5
                else:
                    default_channel = 8
                fltmode_ch = int(self.mpstate.module('param').mav_param.get('FLTMODE_CH', default_channel))
            if self.mpstate.vehicle_name is not None and fltmode_ch > 0:
                rc_received = float(getattr(m, 'chan%u_raw' % fltmode_ch))
                if rc_received != self.fltmode_rc and ((fltmode_ch > 0 and fltmode_ch < 9 and mtype == 'RC_CHANNELS_RAW') or (fltmode_ch > 0 and fltmode_ch < 19 and mtype == 'RC_CHANNELS')):
                    self.fltmode_rc = rc_received
                    self.gui_event_queue.put(ParamEditorEvent(
                        ph_event.PEGE_RCIN, rcin=rc_received))

    @staticmethod
    def child_task(queue, lock, gui_queue, gui_lock, close_window_sem,
                   vehicle_name, moddebug, params):
        '''child process - this holds GUI elements'''
        from MAVProxy.modules.lib import wx_processguard  # noqa: F401
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_paramedit import param_editor_frame

        if platform.system() != 'Windows':
            mp_util.child_close_fds()
        app = wx.App(False)
        app.frame = param_editor_frame.ParamEditorFrame(
            parent=None, id=wx.ID_ANY)
        app.frame.set_event_queue(queue)
        app.frame.set_event_queue_lock(lock)
        app.frame.set_gui_event_queue(gui_queue)
        app.frame.set_gui_event_queue_lock(gui_lock)
        app.frame.get_vehicle_type(vehicle_name)
        app.frame.set_close_window_semaphore(close_window_sem)
        app.frame.redirect_err(moddebug)
        app.frame.set_param_init(params, vehicle_name)
        app.SetExitOnFrameDelete(True)
        app.frame.Show()

        # start a thread to monitor the "close window" semaphore:
        class CloseWindowSemaphoreWatcher(threading.Thread):
            def __init__(self, task, sem):
                threading.Thread.__init__(self)
                self.task = task
                self.sem = sem

            def run(self):
                self.sem.acquire(True)
                wx.CallAfter(self.task.ExitMainLoop)
        watcher_thread = CloseWindowSemaphoreWatcher(app, close_window_sem)
        watcher_thread.start()

        app.MainLoop()
        # tell the watcher it is OK to quit:
        close_window_sem.release()
        watcher_thread.join()

    def close(self):
        '''close the Parameter Editor window'''
        if self.closed:
            return
        self.time_to_quit = True
        if self.event_thread is not None:
            self.event_thread.time_to_quit = True
        for thread in (self.event_thread, self.mavlink_thread):
            if thread is not None and thread.ident is not None:
                thread.join()
        if self.close_window is not None:
            self.close_window.release()
        if self.child is not None:
            if self.threaded:
                if self.child.ident is not None:
                    self.child.join()
            else:
                if self.child.pid is not None:
                    self.child.join(timeout=2)
                    if self.child.is_alive():
                        self.child.terminate()
                        self.child.join(timeout=2)
                    if self.child.is_alive():
                        # A fork child can inherit MAVProxy's SIGTERM handler.
                        self.child.kill()
                        self.child.join(timeout=2)
                    if self.child.is_alive():
                        # Do not drain queues while the child may still read
                        # them, but prevent their feeders blocking exit.
                        for q in (self.event_queue, self.gui_event_queue):
                            if q is not None:
                                q.cancel_join_thread()
                        if getattr(self.mpstate, 'param_editor', None) is self:
                            self.mpstate.param_editor = None
                        raise RuntimeError('Parameter editor process did not stop')
                self.child.close()
            self.child = None
        errors = []
        for name in ('event_queue', 'gui_event_queue'):
            q = getattr(self, name)
            if q is not None:
                try:
                    if not self.threaded:
                        self.close_queue(q)
                    setattr(self, name, None)
                except Exception as ex:
                    # Still clean up the other queue; retain this one so a
                    # later close() can retry after a slow feeder stops.
                    errors.append(ex)
        self.event_thread = None
        self.mavlink_thread = None
        if getattr(self.mpstate, 'param_editor', None) is self:
            self.mpstate.param_editor = None
        if errors:
            raise errors[0]
        self.closed = True

    @staticmethod
    def close_queue(q, timeout=2):
        '''discard a process queue after its producers and readers stop'''
        # Queue.get(), even with a timeout, can hang on a lock or partial
        # message left by a terminated child. Discard raw bytes without either
        # framing or locks. Use a duplicate fd because the feeder closes its
        # own reader on exit. The process backend is only used on POSIX.
        q.cancel_join_thread()
        if q._thread is not None and not q._reader.closed:
            drain_fd = os.dup(q._reader.fileno())
            try:
                os.set_blocking(drain_fd, False)
                q.close()
                if not ParamEditorMain.drain_queue(q, drain_fd, time.monotonic() + timeout):
                    # A slow serializer/feeder must not hold up unloading.
                    # Transfer the drain fd to a daemon which finishes cleanup
                    # even when the module's caller never retries close().
                    cleanup = threading.Thread(
                        target=ParamEditorMain.finish_queue, args=(q, drain_fd),
                        name='ParamEditorQueueCleanup', daemon=True)
                    cleanup.start()
                    drain_fd = None
                    return cleanup
            finally:
                if drain_fd is not None:
                    os.close(drain_fd)
        q.close()
        if q._thread is not None:
            q._thread.join(timeout=2)
            if q._thread.is_alive():
                raise RuntimeError('Parameter editor queue feeder did not stop')
        # Do not race the feeder closing its own connections. With no feeder,
        # Queue.close() does not close them at all.
        q._reader.close()
        q._writer.close()

    @staticmethod
    def drain_queue(q, drain_fd, deadline=None):
        while q._thread.is_alive():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            try:
                if not os.read(drain_fd, 65536):
                    q._thread.join(timeout=0.01)
            except BlockingIOError:
                q._thread.join(timeout=0.01)
        return True

    @staticmethod
    def finish_queue(q, drain_fd):
        try:
            ParamEditorMain.drain_queue(q, drain_fd)
            q._reader.close()
            q._writer.close()
        finally:
            os.close(drain_fd)

    def set_params(self):
        for param, value in self.paramchanged.items():
            self.mpstate.mav_master[0].param_set_send(param, float(value))


def init(mpstate):
    '''initialise module'''
    return ParamEditorMain(mpstate)
