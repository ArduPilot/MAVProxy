#!/usr/bin/env python3
'''
mission editor module
Michael Day
June 2104
'''

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import win_layout

from MAVProxy.modules.mavproxy_misseditor import me_event
from MAVProxy.modules.mavproxy_misseditor.survey_preview import SurveyPreview
import queue
import copy
from types import SimpleNamespace
MissionEditorEvent = me_event.MissionEditorEvent

from pymavlink import mavutil
from pymavlink import mavwp

import time
import threading


class MissionViewer(object):
    '''Read-only mission table used by MAVExplorer.'''
    def __init__(self, wploader, elemodel='SRTM3'):
        self.wploader = wploader
        self.elemodel = elemodel
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        mp_util.child_close_fds()
        from MAVProxy.modules.lib import wx_processguard  # noqa: F401
        from ..lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_misseditor import missionEditorFrame

        app = wx.App(False)
        frame = missionEditorFrame.MissionEditorFrame(
            None, parent=None, id=wx.ID_ANY, elemodel=self.elemodel,
            read_only=True, wploader=self.wploader)
        app.SetExitOnFrameDelete(True)
        frame.Show()
        app.MainLoop()

    def is_alive(self):
        return self.child.is_alive()

    def close(self):
        if self.child.is_alive():
            self.child.terminate()
        self.child.join()

class MissionEditorEventThread(threading.Thread):
    def __init__(self, mp_misseditor, q, l):
        threading.Thread.__init__(self)
        self.mp_misseditor = mp_misseditor
        self.event_queue = q
        self.event_queue_lock = l
        self.time_to_quit = False
        self.write_use_ftp = False
        self.ftp_requests = queue.Queue()

    def module(self, name):
        '''access another module'''
        return self.mp_misseditor.mpstate.module(name)

    def master(self):
        '''access master mavlink connection'''
        return self.mp_misseditor.mpstate.master()
    
    def run(self):
        while not self.time_to_quit:
            queue_access_start_time = time.time()
            self.event_queue_lock.acquire()
            request_read_after_processing_queue = False
            while (not self.event_queue.empty()) and (time.time() - queue_access_start_time) < 0.6:
                event = self.event_queue.get()

                if isinstance(event, win_layout.WinLayout):
                    win_layout.set_layout(event, self.mp_misseditor.set_layout)
                elif isinstance(event, mavwp.MAVWPLoader):
                    self.send_wploader(event)
                else:
                    event_type = event.get_type()

                    if event_type == me_event.MEE_READ_WPS:
                        self.mp_misseditor.reading_mission = False
                        if event.get_arg("use_ftp"):
                            self.mp_misseditor.num_wps_expected = 0
                            self.start_ftp('Read', self.module('wp').wp_ftp_download,
                                           self.ftp_read_done, [])
                        else:
                            # A MAVLink read has an initially unknown count.
                            self.mp_misseditor.reading_mission = True
                            self.mp_misseditor.num_wps_expected = -1
                            self.mp_misseditor.wps_received = {}
                            self.module('wp').cmd_wp(['list'])

                    elif event_type == me_event.MEE_TIME_TO_QUIT:
                        self.time_to_quit = True

                    elif event_type == me_event.MEE_SURVEY_PREVIEW:
                        self.mp_misseditor.survey_preview.set_points(event.get_arg('points'))

                    elif event_type == me_event.MEE_MAP_MISSION:
                        self.mp_misseditor.map_mission = (self.ftp_target(), event.get_arg('wploader'))

                    elif event_type == me_event.MEE_GET_WP_RAD:
                        wp_radius = self.module('param').mav_param.get('WP_RADIUS')
                        if (wp_radius is None):
                            continue
                        self.mp_misseditor.gui_event_queue_lock.acquire()
                        self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                            me_event.MEGE_SET_WP_RAD,wp_rad=wp_radius))
                        self.mp_misseditor.gui_event_queue_lock.release()

                    elif event_type == me_event.MEE_SET_WP_RAD:
                        self.mp_misseditor.param_set('WP_RADIUS',event.get_arg("rad"))

                    elif event_type == me_event.MEE_GET_LOIT_RAD:
                        loiter_radius = self.module('param').mav_param.get('WP_LOITER_RAD')
                        if (loiter_radius is None):
                            continue
                        self.mp_misseditor.gui_event_queue_lock.acquire()
                        self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                            me_event.MEGE_SET_LOIT_RAD,loit_rad=loiter_radius))
                        self.mp_misseditor.gui_event_queue_lock.release()

                    elif event_type == me_event.MEE_SET_LOIT_RAD:
                        loit_rad = event.get_arg("rad")
                        if (loit_rad is None):
                            continue

                        self.mp_misseditor.param_set('WP_LOITER_RAD', loit_rad)

                        #need to redraw rally points
                        # Don't understand why this rally refresh isn't lagging...
                        # likely same reason why "timeout setting WP_LOITER_RAD"
                        #comes back:
                        #TODO: fix timeout issue
                        self.module('rally').set_last_change(time.time())

                    elif event_type == me_event.MEE_GET_WP_DEFAULT_ALT:
                        self.mp_misseditor.gui_event_queue_lock.acquire()
                        self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                            me_event.MEGE_SET_WP_DEFAULT_ALT,def_wp_alt=self.mp_misseditor.mpstate.settings.wpalt))
                        self.mp_misseditor.gui_event_queue_lock.release()
                    elif event_type == me_event.MEE_SET_WP_DEFAULT_ALT:
                        self.mp_misseditor.mpstate.settings.command(["wpalt",event.get_arg("alt")])

                    elif event_type == me_event.MEE_WRITE_WPS:
                        self.mp_misseditor.reading_mission = False
                        self.write_use_ftp = event.get_arg("use_ftp")
                        self.module('wp').wploader.clear()
                        self.module('wp').wploader.expected_count = event.get_arg("count")
                        if not self.write_use_ftp:
                            self.master().waypoint_count_send(event.get_arg("count"))
                        self.mp_misseditor.num_wps_expected = 0
                        self.mp_misseditor.wps_received = {}
                    elif event_type == me_event.MEE_WRITE_WP_NUM:
                        w = mavutil.mavlink.MAVLink_mission_item_message(
                            self.mp_misseditor.mpstate.settings.target_system,
                            self.mp_misseditor.mpstate.settings.target_component,
                            event.get_arg("num"),
                            int(event.get_arg("frame")),
                            event.get_arg("cmd_id"),
                            0, 1,
                            event.get_arg("p1"), event.get_arg("p2"),
                            event.get_arg("p3"), event.get_arg("p4"),
                            event.get_arg("lat"), event.get_arg("lon"),
                            event.get_arg("alt"))

                        self.module('wp').wploader.add(w)
                        if self.write_use_ftp:
                            # Wait until every GUI row has reached the loader.
                            loader = self.module('wp').wploader
                            if loader.count() == loader.expected_count:
                                self.start_ftp('Write', self.module('wp').ftp_upload,
                                               self.ftp_write_done, copy.deepcopy(loader))
                                self.mp_misseditor.num_wps_expected = 0
                            continue
                        wsend = self.module('wp').wploader.wp(w.seq)
                        if self.mp_misseditor.mpstate.settings.wp_use_mission_int:
                            wsend = self.module('wp').wp_to_mission_item_int(w)
                        self.master().mav.send(wsend)

                        #tell the wp module to expect some waypoints
                        self.module('wp').loading_waypoints = True

                    elif event_type == me_event.MEE_SAVE_WP_FILE:
                        self.module('wp').cmd_wp(['save',event.get_arg("path")])

            self.event_queue_lock.release()

            #if event processing operations require a mission referesh in GUI
            #(e.g., after a load or a verified-completed write):
            if (request_read_after_processing_queue):
                self.event_queue_lock.acquire()
                self.event_queue.put(MissionEditorEvent(me_event.MEE_READ_WPS))
                self.event_queue_lock.release()

            #periodically re-request WPs that were never received:
            #DON'T NEED TO! -- wp module already doing this

            time.sleep(0.2)

    def ftp_read_done(self, wploader):
        '''populate the table before reporting a successful download'''
        if self.time_to_quit:
            return
        if wploader is None:
            self.ftp_transfer_done(False, "Read failed")
            return
        # Deliver the whole result atomically so the GUI can reject it if
        # the user edited the mission during the download.
        with self.mp_misseditor.gui_event_queue_lock:
            self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                me_event.MEGE_FTP_MISSION, wploader=copy.deepcopy(wploader)))

    def send_wploader(self, wploader):
        with self.mp_misseditor.gui_event_queue_lock:
            self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                me_event.MEGE_LOAD_MISSION, wploader=copy.deepcopy(wploader)))

    def ftp_target(self):
        settings = self.mp_misseditor.mpstate.settings
        return settings.target_system, settings.target_component

    def start_ftp(self, operation, transfer, callback, data):
        self.ftp_requests.put((operation, self.ftp_target(), transfer, callback, data))

    def process_ftp_requests(self):
        '''called only by the MAVProxy main loop'''
        while not self.time_to_quit:
            try:
                operation, target, transfer, callback, data = self.ftp_requests.get_nowait()
            except queue.Empty:
                return
            if target != self.ftp_target():
                self.ftp_transfer_done(False, '%s cancelled: vehicle changed' % operation)
                continue

            def completed(result, target=target, callback=callback, operation=operation):
                if target != self.ftp_target():
                    self.ftp_transfer_done(False, '%s result discarded: vehicle changed' % operation)
                else:
                    callback(result)

            try:
                transfer(data, callback=completed)
            except Exception as ex:
                self.ftp_transfer_done(False, '%s failed: %s' % (operation, ex))

    def ftp_write_done(self, dlen):
        self.ftp_transfer_done(dlen is not None,
                               "Write succeeded" if dlen is not None else "Write failed")

    def ftp_transfer_done(self, success, message):
        if not self.time_to_quit:
            with self.mp_misseditor.gui_event_queue_lock:
                self.mp_misseditor.gui_event_queue.put(MissionEditorEvent(
                    me_event.MEGE_FTP_TRANSFER, success=success, message="MAVFTP: " + message))

class MissionEditorMain(object):
    def __init__(self, mpstate, elemodel):
        self.mpstate = mpstate
        self.time_to_quit = False
        self.num_wps_expected = 0 #helps me to know if all my waypoints I'm expecting have arrived
        self.wps_received = {}
        self.reading_mission = False
        self.map_mission = None

        self.survey_preview = SurveyPreview()
        self.event_queue = multiproc.Queue()
        self.event_queue_lock = multiproc.Lock()
        self.gui_event_queue = multiproc.Queue()
        self.gui_event_queue_lock = multiproc.Lock()

        self.object_queue = multiproc.Queue()

        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()

        # Spawn/forkserver must not serialize this editor or live MAVProxy state.
        self.child = multiproc.Process(
            target=self.child_task,
            args=(self.event_queue, self.event_queue_lock,
                  self.gui_event_queue, self.gui_event_queue_lock,
                  self.close_window, elemodel, self.object_queue))
        self.child.start()

        self.event_thread = MissionEditorEventThread(self, self.event_queue, self.event_queue_lock)
        self.event_thread.start()

        self.mpstate.miss_editor = self

        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1 # seconds

        self.mavlink_message_queue = multiproc.Queue()
        self.mavlink_message_queue_handler = threading.Thread(target=self.mavlink_message_queue_handler)
        self.mavlink_message_queue_handler.start()
        self.needs_unloading = False
        self.last_wp_change = time.time()

    def mavlink_message_queue_handler(self):
        while not self.time_to_quit:
            while True:
                if self.time_to_quit:
                    return
                if not self.mavlink_message_queue.empty():
                    break
                time.sleep(0.1)
            m = self.mavlink_message_queue.get()

            #MAKE SURE YOU RELEASE THIS LOCK BEFORE LEAVING THIS METHOD!!!
            #No "return" statement should be put in this method!
            self.gui_event_queue_lock.acquire()

            try:
                self.process_mavlink_packet(m)
            except Exception as e:
                print("Caught exception (%s)" % str(e))
                import traceback
                traceback.print_stack()

            self.gui_event_queue_lock.release()

    def unload(self):
        '''unload module'''
        self.mpstate.miss_editor.close()
        self.mpstate.miss_editor = None

    def get_wps_from_module(self):
        '''get WP list from wp module'''
        self.event_queue_lock.acquire()
        self.event_queue.put(self.mpstate.module('wp').wploader)
        self.event_queue_lock.release()

    def idle_task(self):
        self.event_thread.process_ftp_requests()
        now = time.time()
        if self.last_unload_check_time + self.unload_check_interval < now:
            self.last_unload_check_time = now
            if not self.child.is_alive():
                self.close()
                return
        maps = [module.map for name, module in self.mpstate.public_modules.items()
                if name.startswith('map') and hasattr(getattr(module, 'map', None), 'add_object')]
        self.survey_preview.draw(maps)
        last_wp_change = self.mpstate.module('wp').loading_waypoint_lasttime
        if last_wp_change > self.last_wp_change:
            self.last_wp_change = last_wp_change
            if self.get_map_mission() is None:
                self.get_wps_from_module()

    def get_map_mission(self):
        '''Return an immutable draft snapshot for this vehicle, if editing.'''
        draft = self.map_mission
        if self.time_to_quit or draft is None:
            return None
        target, loader = draft
        settings = self.mpstate.settings
        if target != (settings.target_system, settings.target_component):
            return None
        return loader



    def mavlink_packet(self, m):
        if (getattr(m, 'mission_type', None) is not None and
            m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
            return
        mtype = m.get_type()
        if mtype in ['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT']:
            if mtype == 'MISSION_ITEM_INT':
                m = self.mpstate.module('wp').wp_from_mission_item_int(m)
            self.mavlink_message_queue.put(m)

    def process_mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()

        # if you add processing for an mtype here, remember to add it
        # to mavlink_packet, above
        if (getattr(m, 'mission_type', None) is not None and
            m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
            return
        # Only an explicit editor Read may replace the table through mission
        # packets. Console reads and upload responses must not bypass the GUI's
        # revision/dirty guards through the old incremental receive path.
        if not self.reading_mission:
            return
        if mtype == 'MISSION_COUNT':
            self.num_wps_expected = m.count
            self.wps_received = {}
        elif mtype == 'MISSION_ITEM' and 0 <= m.seq < self.num_wps_expected:
            self.wps_received[m.seq] = m
        else:
            return
        if len(self.wps_received) == self.num_wps_expected:
            loader = mavwp.MAVWPLoader()
            for seq in range(self.num_wps_expected):
                loader.add(self.wps_received[seq])
            self.gui_event_queue.put(MissionEditorEvent(me_event.MEGE_READ_MISSION, wploader=loader))
            self.reading_mission = False
            self.num_wps_expected = 0

    @staticmethod
    def child_task(q, l, gq, gl, cw_sem, elemodel, object_queue):
        '''child process - this holds GUI elements'''
        mp_util.child_close_fds()

        from MAVProxy.modules.lib import wx_processguard
        from ..lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_misseditor import missionEditorFrame

        app = wx.App(False)
        state = SimpleNamespace(object_queue=object_queue)
        app.frame = missionEditorFrame.MissionEditorFrame(state, parent=None, id=wx.ID_ANY, elemodel=elemodel)

        app.frame.set_event_queue(q)
        app.frame.set_event_queue_lock(l)
        app.frame.set_gui_event_queue(gq)
        app.frame.set_gui_event_queue_lock(gl)
        app.frame.set_close_window_semaphore(cw_sem)

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
        watcher_thread = CloseWindowSemaphoreWatcher(app, cw_sem)
        watcher_thread.start()

        app.MainLoop()
        # tell the watcher it is OK to quit:
        cw_sem.release()
        watcher_thread.join()

    def close(self):
        '''close the Mission Editor window'''
        self.time_to_quit = True
        self.survey_preview.close()
        self.close_window.release()
        if self.child.is_alive():
            self.child.join(1)

        self.child.terminate()

        self.mavlink_message_queue_handler.time_to_quit = True
        self.mavlink_message_queue_handler.join()

        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_TIME_TO_QUIT));
        self.event_queue_lock.release()

        self.needs_unloading = True

    def read_waypoints(self):
        self.module('wp').cmd_wp(['list'])

    def update_map_click_position(self, new_click_pos):
        self.gui_event_queue_lock.acquire()
        self.gui_event_queue.put(MissionEditorEvent(
            me_event.MEGE_SET_LAST_MAP_CLICK_POS,click_pos=new_click_pos))
        self.gui_event_queue_lock.release()

    def set_layout(self, layout):
        self.object_queue.put(layout)

def init(mpstate):
    '''initialise module'''
    return MissionEditorModule(mpstate)
