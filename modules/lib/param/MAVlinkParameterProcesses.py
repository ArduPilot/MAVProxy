import threading, Queue
import time
import sys,os

# find the mavlink.py module
for d in [ 'pymavlink',
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'MAVLink', 'pymavlink') ]:
    if os.path.exists(d):
        sys.path.insert(0, d)
        if os.name == 'nt':
            try:
                # broken python compilation of mavlink.py on windows!
                os.unlink(os.path.join(d, 'mavlinkv10.pyc'))
            except:
                pass

import mavlinkv10 as mavlink
import mavutil

import pyparameters

from callback_types import callback_messages

class ParamStatus(object):
    NOT_STARTED = 0
    NOT_CONNECTED = 1
    CONNECTED = 2
    READ_ALL_PARAMETERS = 10
    READING_ALL_PARAMETERS = 11
    READ_MISSING_PARAMETERS = 12
    READING_MISSING_PARAMETER = 13
    WRITE_CHANGED_PARAMETERS = 20
    WRITING_CHANGED_PARAMETER = 21
    WRITE_MEMORY_AREA = 30
    WRITING_MEMORY_AREA = 31
    LOAD_MEMORY_AREA = 32
    LOADING_MEMORY_AREA = 33
    CLEAR_MEMORY_AREA = 34
    CLEARING_MEMORY_AREA = 35

class mavlink_parameter_processes:
    def __init__(self, doc):
        
        self.doc = doc
        self.param_handler = self.doc.param_handler
        self.doc.m_register_callback(self.doc_callback)
        
        self.MAVServices = MAVlink_param_services(self)  #shutdown_hook = self.shutdown_hook
        self.MAVServices.start()
        
        self.mpstate = None
        self.sysID = 0 
        self.compID = 0

    def __destroy__(self):
        self.stop_services(self)
        while(self.services_running() == 1):
            time.sleep(0.1)
 

    def shutdown_hook(self, t_id, child):
        print('%s - Unexpected thread shutdown, handle this.. restart thread?' % str(t_id))
        
    def set_mpstate(self, mpstate ):
        self.mpstate = mpstate        
        self.sysID = mpstate.status.target_system
        self.compID = mpstate.status.target_component


    def services_running(self):
        return self.MAVServices.isAlive()
    
    
    def stop_services(self):
        print("mavlink services request stop")
        self.MAVServices.stop()


    def tx_msg_append(self, tx_msg):
        try:
            self.MAVServices
        except:
            print("MAV services thread does not exist, can not add message to tx queue")
            return False
        else:
            return self.MAVServices.tx_msg_append(tx_msg)
        
        
    def doc_callback(self, callback_type, val = None):
#        if(callback_type == mixer_doc.callback_type.FUNCTION_MODIFIED):
#            self.update_function(val)
        if(callback_type == callback_messages.READ_ALL_PARAMS):
            self.update_parameters()
        if(callback_type == callback_messages.WRITE_NV_AREA):
            self.write_nv_memory_area(val)
        if(callback_type == callback_messages.READ_NV_AREA):
            self.read_nv_memory_area(val)
        if(callback_type == callback_messages.CLEAR_NV_AREA):
            self.clear_nv_memory_area(val)
        if(callback_type == callback_messages.WRITE_ALL_PARAMS):
            self.MAVServices.synchronised = False
        if(callback_type == callback_messages.WRITE_CHANGED_PARAMS):
            self.write_changed_parameters()
            
        
    def update_parameters(self):
        if(self.services_running() == True):
            self.MAVServices.refresh_parameters()

    def write_changed_parameters(self):
        if self.services_running():
            self.MAVServices.write_changed_parameters()
            
    def write_nv_memory_area(self, mem_area):
        if self.services_running():
            self.MAVServices.write_nv_memory_area(mem_area)

    def read_nv_memory_area(self, mem_area):
        if self.services_running():
            self.MAVServices.read_nv_memory_area(mem_area)
        
    def clear_nv_memory_area(self, mem_area):
        if self.services_running():
            self.MAVServices.clear_nv_memory_area(mem_area)

    def msg_recv(self, msg):
        try:
            self.MAVServices
        except:
            return
        else:
            self.MAVServices.msg_recv(msg)
            return 
        

class MAVlink_param_services(threading.Thread):
    def __init__(self, mav_proc):
        threading.Thread.__init__(self)

        self._stop = threading.Event()
        
        self.mav_proc = mav_proc
        self.doc = mav_proc.doc
        self.param_handler = self.doc.param_handler

        self.tx_q       = Queue.Queue(3)
        
        self.heartbeat_timer = 0
        self.heartbeat_ok = False
        
        self.param_messages         = [];
        self.Condition = ParamStatus.NOT_STARTED
        
        self.params_timeout         = time.time()
        self.params_retry           = 0

        self.rx_q       = Queue.Queue(20)


    def stop(self):
        print("MAVlink service thread request stop")
        self._stop.set()

    def stopped(self):
        return not self.isAlive()

    def tx_msg_append(self, mesg):
        if self.isAlive():
            try:
                self.tx_q.put(mesg)
            except:
                print("transmit buffer full")
            else:
                print("message added to queue")
                return True
        else:
            print("MAVlink services not alive, can not put message in queue")
        return False
    
    def refresh_parameters(self):
        if(self.Condition == ParamStatus.CONNECTED):
            self.read_params_timeout = time.time() + 2
            self.params_retry = 0
            self.Condition = ParamStatus.READ_ALL_PARAMETERS


    def write_changed_parameters(self):
        if(self.Condition == ParamStatus.CONNECTED):
            self.read_params_timeout = time.time() + 2
            self.params_retry = 0
            self.Condition = ParamStatus.WRITE_CHANGED_PARAMETERS
    
    def read_nv_memory_area(self, mem_area):
        if(self.Condition == ParamStatus.CONNECTED):
            self.mem_area = mem_area
            self.Condition = ParamStatus.LOAD_MEMORY_AREA

    def write_nv_memory_area(self, mem_area):
        if(self.Condition == ParamStatus.CONNECTED):
            self.mem_area = mem_area
            self.Condition = ParamStatus.WRITE_MEMORY_AREA

    def clear_nv_memory_area(self, mem_area):
        if(self.Condition == ParamStatus.CONNECTED):
            self.mem_area = mem_area
            self.Condition = ParamStatus.CLEAR_MEMORY_AREA
        
    def parse_message(self, msg ):
        if msg and msg.get_type() == "HEARTBEAT":
            self.heartbeat_ok = True
            self.heartbeat_timer = time.time()
            if(self.Condition == ParamStatus.NOT_CONNECTED):
                self.on_connect()

        if msg and msg.get_type() == "PARAM_VALUE":
            system = msg.get_srcSystem()
            component = msg.get_srcComponent()
            if(1 == 1):
                last_param = self.param_handler.update_msg(msg)
                self.read_params_timeout = time.time() + 2
                self.params_retry = 0
                        
                if(last_param == True):
                    print("last parameter")
                        
                if(self.Condition == ParamStatus.READING_ALL_PARAMETERS):
                    if(last_param == True):
                        self.Condition = ParamStatus.READ_MISSING_PARAMETERS
                        print("Read all params complete, checking for missing params")
                        
                if(self.Condition == ParamStatus.READING_MISSING_PARAMETER):
                    print("Read a missing parameter, now read the next missing one")
                    self.Condition = ParamStatus.READ_MISSING_PARAMETERS
                        
                if(self.Condition == ParamStatus.WRITING_CHANGED_PARAMETER):
#                   if(self.written_param.param_index != msg.param_index):
                    print("Read a changed parameter, now write the next changed one")
                    self.Condition = ParamStatus.WRITE_CHANGED_PARAMETERS
                            
        if msg and msg.get_type() == "COMMAND_ACK":
            if(msg.command == mavlink.MAV_CMD_PREFLIGHT_STORAGE):
                mem_update = pyparameters.memory_update()
                mem_update.result = msg.result
                if(self.Condition == ParamStatus.WRITING_MEMORY_AREA):
                    mem_update.action = mavlink.MAV_PFS_CMD_WRITE_SPECIFIC
                elif(self.Condition == ParamStatus.LOADING_MEMORY_AREA):
                    mem_update.action = mavlink.MAV_PFS_CMD_READ_SPECIFIC
                elif(self.Condition == ParamStatus.CLEARING_MEMORY_AREA):
                    mem_update.action = mavlink.MAV_PFS_CMD_CLEAR_SPECIFIC
                self.param_handler.nv_storage_action(mem_update)
            self.Condition = ParamStatus.CONNECTED


    def run(self):
        self._stop.clear()
        print("MAVlink service thread starting")

        self.Condition = ParamStatus.NOT_CONNECTED

        while(not self._stop.isSet() ):
            try:
                msg = self.rx_q.get(True, 0.1)
            except Queue.Empty:
                pass
            else:
                self.parse_message(msg)
                self.rx_q.task_done()

            if(self.mav_proc.mpstate is None):
                return
                
            if(self.heartbeat_ok == True):
                if(self.Condition == ParamStatus.READ_ALL_PARAMETERS):
                    self.param_handler.clear()
                    self.read_params_timeout = time.time() + 3
                    self.Condition = ParamStatus.READING_ALL_PARAMETERS
                    self.mav_proc.mpstate.master().param_fetch_all()
                    print("Reading all params")


                elif(self.Condition == ParamStatus.READ_MISSING_PARAMETERS):
                    nonsync_msg_index = self.param_handler.get_nonsync_param_index()
                    if(nonsync_msg_index == -1):
                        print("All params complete")
                        self.Condition = ParamStatus.CONNECTED
                        self.param_handler.param_update_complete()
                    else:
                        print("Request to read missing parameter number %u" % nonsync_msg_index)
                        self.mav_proc.mpstate.master().param_request_read_send(self.mav_proc.sysID, self.mav_proc.compID, "", nonsync_msg_index)
                        self.read_params_timeout = time.time() + 2
                        self.Condition = ParamStatus.READING_MISSING_PARAMETER

                elif(self.Condition == ParamStatus.WRITE_CHANGED_PARAMETERS):
                    nonsync_msg_index = self.param_handler.get_nonsync_param_index()
                    print("Write changed parameters")
                    if(nonsync_msg_index == -1):
                        self.Condition = ParamStatus.CONNECTED
                        print("Writing changed parameters complete")
                        self.param_handler.param_update_complete()
                    else:
                        param = self.param_handler.parameters[nonsync_msg_index]
                        self.mav_proc.mpstate.master().param_set_send(param.param_id, param.param_value.val_float, param.param_type)
                        self.written_param = param
                        print("Writing changed parameter")
                        self.Condition = ParamStatus.WRITING_CHANGED_PARAMETER
                
                elif(self.Condition == ParamStatus.WRITE_MEMORY_AREA):
                    self.Condition = ParamStatus.WRITING_MEMORY_AREA
                    self.mav_proc.mpstate.master().mav.command_long_send(self.mav_proc.sysID, self.mav_proc.compID, mavlink.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED, 0, mavlink.MAV_PFS_CMD_WRITE_SPECIFIC, self.mem_area, 0,0,0,0,0)

                elif(self.Condition == ParamStatus.LOAD_MEMORY_AREA):
                    self.Condition = ParamStatus.LOADING_MEMORY_AREA
                    self.mav_proc.mpstate.master().mav.command_long_send(self.mav_proc.sysID, self.mav_proc.compID, mavlink.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED, 0, mavlink.MAV_PFS_CMD_READ_SPECIFIC, self.mem_area, 0,0,0,0,0)

                elif(self.Condition == ParamStatus.CLEAR_MEMORY_AREA):
                    self.Condition = ParamStatus.CLEARING_MEMORY_AREA
                    self.mav_proc.mpstate.master().mav.command_long_send(self.mav_proc.sysID, self.mav_proc.compID, mavlink.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED, 0, mavlink.MAV_PFS_CMD_CLEAR_SPECIFIC, self.mem_area, 0,0,0,0,0)
                    
                if( (self.Condition == ParamStatus.READ_ALL_PARAMETERS) or (self.Condition == ParamStatus.READING_MISSING_PARAMETER)):
                    if(time.time() > self.read_params_timeout):
                        print("Read all params timeout, starting to read missing params")
                        self.params_retry += 1
                        if(self.params_retry >= 3):
                            self.Condition = ParamStatus.CONNECTED
                        else:
                            self.Condition = ParamStatus.READ_MISSING_PARAMETERS
                            self.read_params_timeout = time.time()+2
                        
            if(time.time() > (self.heartbeat_timer + 4) ):
                if(self.heartbeat_ok == True):
                    self.on_disconnect()

        
        self.Condition = ParamStatus.NOT_STARTED

        print("MAVlink service thread terminated")

    
#    def find_next_update_parameter(self):
#        for param in self.param:
            
    def on_disconnect(self):
        print("MAV disconnected")
        self.Condition = ParamStatus.NOT_CONNECTED
        self.timeout = time.time() + 1E6
        self.synchronised = False
        self.heartbeat_ok = False
        self.doc.m_disconnected()

 
    def on_connect(self):
        print("MAV connected")
        self.Condition = ParamStatus.CONNECTED
        self.timeout = time.time() + 1E6
        self.synchronised = False
        self.heartbeat_ok = True
        self.doc.m_connected()


    def msg_recv(self, msg):
        if(self.Condition == ParamStatus.NOT_STARTED):
            self.on_connect(self)
        
        if(self.rx_q.full()):
            try:
                self.rx_q.get_nowait()
                self.rx_q.task_done()
            except:
                pass
        try:
            self.rx_q.put_nowait(msg)
        except:
            pass
        
