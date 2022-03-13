#!/usr/bin/env python
'''
param editor module
Akshath Singhal
June 2019
'''

import platform
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.mavproxy_paramedit import ph_event
import threading
from ..lib.wx_loader import wx
from MAVProxy.modules.mavproxy_paramedit import param_editor_frame
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
            while not self.event_queue.empty():
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
        self.event_queue = multiproc.Queue()
        self.event_queue_lock = multiproc.Lock()
        self.gui_event_queue = multiproc.Queue()
        self.gui_event_queue_lock = multiproc.Lock()

        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()

        self.mpstate = mpstate
        self.mpstate.param_editor = self
        self.needs_unloading = False

        if platform.system() == 'Windows':
            self.child = threading.Thread(
                            target=self.child_task,
                            args=(self.event_queue,
                                  self.event_queue_lock, self.gui_event_queue,
                                  self.gui_event_queue_lock, self.close_window))
        else:
            self.child = multiproc.Process(
                                target=self.child_task,
                                args=(self.event_queue,
                                      self.event_queue_lock, self.gui_event_queue,
                                      self.gui_event_queue_lock, self.close_window))

        self.child.start()

        self.event_thread = ParamEditorEventThread(
                            self, self.event_queue, self.event_queue_lock)
        self.event_thread.start()

        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1  # seconds

        self.time_to_quit = False
        self.mavlink_message_queue = multiproc.Queue()
        self.mavlink_message_queue_handler = threading.Thread(
            target=self.mavlink_message_queue_handler)
        self.mavlink_message_queue_handler.start()

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
        self.mpstate.param_editor.close()

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

    def child_task(self, queue, lock, gui_queue, gui_lock, close_window_sem):
        '''child process - this holds GUI elements'''
        mp_util.child_close_fds()
        self.app = wx.App(False)
        self.app.frame = param_editor_frame.ParamEditorFrame(
            parent=None, id=wx.ID_ANY)
        self.app.frame.set_event_queue(queue)
        self.app.frame.set_event_queue_lock(lock)
        self.app.frame.set_gui_event_queue(gui_queue)
        self.app.frame.set_gui_event_queue_lock(gui_lock)
        self.app.frame.get_vehicle_type(self.mpstate.vehicle_name)
        self.app.frame.set_close_window_semaphore(close_window_sem)
        self.app.frame.redirect_err(self.mpstate.settings.moddebug)
        self.app.frame.set_param_init(self.mpstate.module('param').mav_param, self.mpstate.vehicle_name)
        self.app.SetExitOnFrameDelete(True)
        self.app.frame.Show()

        # start a thread to monitor the "close window" semaphore:
        class CloseWindowSemaphoreWatcher(threading.Thread):
            def __init__(self, task, sem):
                threading.Thread.__init__(self)
                self.task = task
                self.sem = sem

            def run(self):
                self.sem.acquire(True)
                self.task.app.ExitMainLoop()
        watcher_thread = CloseWindowSemaphoreWatcher(self, close_window_sem)
        watcher_thread.start()

        self.app.MainLoop()
        # tell the watcher it is OK to quit:
        close_window_sem.release()
        watcher_thread.join()

    def close(self):
        '''close the Parameter Editor window'''
        self.time_to_quit = True
        self.close_window.release()
        if platform.system() == 'Windows':
            self.child.join()
        else:
            self.child.terminate()

    def set_params(self):
        for param, value in self.paramchanged.items():
            self.mpstate.mav_master[0].param_set_send(param, float(value))


def init(mpstate):
    '''initialise module'''
    return ParamEditorMain(mpstate)
