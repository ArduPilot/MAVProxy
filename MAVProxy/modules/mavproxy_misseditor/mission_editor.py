#!/usr/bin/env python
'''
mission editor module
Michael Day
June 2104
'''

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import win_layout

from MAVProxy.modules.mavproxy_misseditor import me_event
MissionEditorEvent = me_event.MissionEditorEvent

from pymavlink import mavutil

import time
import threading

class MissionEditorEventThread(threading.Thread):
    def __init__(self, mp_misseditor, q, l):
        threading.Thread.__init__(self)
        self.mp_misseditor = mp_misseditor
        self.event_queue = q
        self.event_queue_lock = l
        self.time_to_quit = False

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
                else:
                    event_type = event.get_type()

                    if event_type == me_event.MEE_READ_WPS:
                        self.module('wp').cmd_wp(['list'])
                        #list the rally points while I'm add it:
                        #TODO: DON'T KNOW WHY THIS DOESN'T WORK
                        #self.module('rally').cmd_rally(['list'])

                        #means I'm doing a read & don't know how many wps to expect:
                        self.mp_misseditor.num_wps_expected = -1
                        self.wps_received = {}

                    elif event_type == me_event.MEE_TIME_TO_QUIT:
                        self.time_to_quit = True

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
                        self.module('wp').wploader.clear()
                        self.master().waypoint_count_send(event.get_arg("count"))
                        self.mp_misseditor.num_wps_expected = event.get_arg("count")
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
                        wsend = self.module('wp').wploader.wp(w.seq)
                        if self.mp_misseditor.mpstate.settings.wp_use_mission_int:
                            wsend = self.module('wp').wp_to_mission_item_int(w)
                        self.master().mav.send(wsend)

                        #tell the wp module to expect some waypoints
                        self.module('wp').loading_waypoints = True

                    elif event_type == me_event.MEE_LOAD_WP_FILE:
                        self.module('wp').cmd_wp(['load',event.get_arg("path")])
                        #Wait for the other thread to finish loading waypoints.
                        #don't let this loop run forever in case we have a lousy
                        #link to the plane
                        i = 0
                        while (i < 10 and
                               self.module('wp').loading_waypoints):
                            time.sleep(1)
                            i = i + 1

                            #don't modify queue while in the middile of processing it:
                            request_read_after_processing_queue = True

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

class MissionEditorMain(object):
    def __init__(self, mpstate):
        self.num_wps_expected = 0 #helps me to know if all my waypoints I'm expecting have arrived
        self.wps_received = {}

        self.event_queue = multiproc.Queue()
        self.event_queue_lock = multiproc.Lock()
        self.gui_event_queue = multiproc.Queue()
        self.gui_event_queue_lock = multiproc.Lock()

        self.object_queue = multiproc.Queue()

        self.close_window = multiproc.Semaphore()
        self.close_window.acquire()

        self.child = multiproc.Process(target=self.child_task,args=(self.event_queue,self.event_queue_lock,self.gui_event_queue,self.gui_event_queue_lock,self.close_window))
        self.child.start()

        self.event_thread = MissionEditorEventThread(self, self.event_queue, self.event_queue_lock)
        self.event_thread.start()

        self.mpstate = mpstate
        self.mpstate.miss_editor = self

        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1 # seconds

        self.time_to_quit = False
        self.mavlink_message_queue = multiproc.Queue()
        self.mavlink_message_queue_handler = threading.Thread(target=self.mavlink_message_queue_handler)
        self.mavlink_message_queue_handler.start()
        self.needs_unloading = False

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

    def idle_task(self):
        now = time.time()
        if self.last_unload_check_time + self.unload_check_interval < now:
            self.last_unload_check_time = now
            if not self.child.is_alive():
                self.close()


    def mavlink_packet(self, m):
        if (getattr(m, 'mission_type', None) is not None and
            m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
            return
        mtype = m.get_type()
        if mtype in ['WAYPOINT_COUNT','MISSION_COUNT', 'WAYPOINT', 'MISSION_ITEM', 'MISSION_ITEM_INT']:
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
        if mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
            if (self.num_wps_expected == 0):
                #I haven't asked for WPs, or these messages are duplicates
                #of msgs I've already received.
                self.mpstate.console.error("No waypoint load started (from Editor).")
            #I only clear the mission in the Editor if this was a read event
            elif (self.num_wps_expected == -1):
                self.gui_event_queue.put(MissionEditorEvent(
                    me_event.MEGE_CLEAR_MISS_TABLE))
                self.num_wps_expected = m.count
                self.wps_received = {}

                if (m.count > 0):
                    self.gui_event_queue.put(MissionEditorEvent(
                        me_event.MEGE_ADD_MISS_TABLE_ROWS,num_rows=m.count-1))
            #write has been sent by the mission editor:
            elif (self.num_wps_expected > 1):
                if (m.count != self.num_wps_expected):
                    self.mpstate.console.error("Unexpected waypoint count from vehicle after write (Editor)")
                #since this is a write operation from the Editor there
                #should be no need to update number of table rows

        elif mtype in ['WAYPOINT', 'MISSION_ITEM']:
            #still expecting wps?
            if (len(self.wps_received) < self.num_wps_expected):
                #if we haven't already received this wp, write it to the GUI:
                if (m.seq not in self.wps_received.keys()):
                    self.gui_event_queue.put(MissionEditorEvent(
                        me_event.MEGE_SET_MISS_ITEM,
                        num=m.seq,command=m.command,param1=m.param1,
                        param2=m.param2,param3=m.param3,param4=m.param4,
                        lat=m.x,lon=m.y,alt=m.z,frame=m.frame))

                    self.wps_received[m.seq] = True

    def child_task(self, q, l, gq, gl, cw_sem):
        '''child process - this holds GUI elements'''
        mp_util.child_close_fds()

        from MAVProxy.modules.lib import wx_processguard
        from ..lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_misseditor import missionEditorFrame

        self.app = wx.App(False)
        self.app.frame = missionEditorFrame.MissionEditorFrame(self,parent=None,id=wx.ID_ANY)

        self.app.frame.set_event_queue(q)
        self.app.frame.set_event_queue_lock(l)
        self.app.frame.set_gui_event_queue(gq)
        self.app.frame.set_gui_event_queue_lock(gl)
        self.app.frame.set_close_window_semaphore(cw_sem)

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
        watcher_thread = CloseWindowSemaphoreWatcher(self, cw_sem)
        watcher_thread.start()

        self.app.MainLoop()
        # tell the watcher it is OK to quit:
        cw_sem.release()
        watcher_thread.join()

    def close(self):
        '''close the Mission Editor window'''
        self.time_to_quit = True
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
