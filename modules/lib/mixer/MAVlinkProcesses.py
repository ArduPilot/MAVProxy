import threading, Queue
import time
import sys,os

import StructDataGen

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

import mixer_doc

class Status(object):
    NOT_STARTED = 0
    NOT_CONNECTED = 1
    CONNECTED = 2
    SEND_SINGLE_FUNCTION = 7
    WAITING_SINGLE_FUNCTION_ACK = 8
    START_SEND_ALL_FUNCTIONS = 9
    SEND_FUNCTIONS = 10
    SENDING_FUNCTIONS = 11
    WAITING_FUNCTION_ACK = 12
#    SEND_SIZES = 13
#    WAITING_SIZES_ACK= 14
    COMMITING_BUFFER = 15
    WAITING_COMMIT_BUFFER_ACK = 16
    GETTING_STATS = 16
    WAITING_GETTING_STATS = 17
    SEND_INPUT_DIRECTORY = 18
    WAITING_INPUT_DIRECTORY_ACK= 19
    SEND_OUTPUT_DIRECTORY = 20
    WAITING_OUTPUT_DIRECTORY_ACK= 21
    WAITING_ACK= 30
    WRITE_NVMEMORY = 100
    WAITING_WRITE_NVMEMORY_ACK = 101

class Commands(object):
    NULL = 0
    READ_FUNCTIONS_ALL = 1
    READ_FUNCTION_SPECIFIC = 2
    COMMIT_BUFFER = 3
    READ_NVMEMORY = 4
    WRITE_NVMEMORY = 5
    RELOAD_FROM_NVMEMORY = 6
    READ_SETTINGS_TO_BUFFER = 7



class mavlink_processes:
    def __init__(self, doc):
        self.doc = doc
        
        self.mpstate = None
 
        self.max_functions = 80
        self.max_registers = 80
        self.functions_used = 0
               
        self.MAVServices = MAVlink_services(self)  #shutdown_hook = self.shutdown_hook
        self.MAVServices.start()
        
        self.doc.m_register_callback(self.doc_callback)
        
        self.sysID = 0 
        self.compID = 0

    def __destroy__(self):
        self.stop_services(self)
        while(self.services_running() == 1):
            time.sleep(0.1)
 
    def set_mpstate(self, mpstate ):
        self.mpstate = mpstate        
        self.sysID = mpstate.status.target_system
        self.compID = mpstate.status.target_component
 
    def shutdown_hook(self, t_id, child):
        print('%s - Unexpected thread shutdown, handle this.. restart thread?' % str(t_id))


    def doc_callback(self, callback_type, val = None):
#        if(callback_type == MainFrame.callback_type.FUNCTION_MODIFIED):
#            self.update_function(val)
        if(callback_type == mixer_doc.callback_type.UPDATE_FUNCTION):
            self.update_function(val)
        if(callback_type == mixer_doc.callback_type.UPDATE_ALL):
            self.send_all()
        if(callback_type == mixer_doc.callback_type.NOT_SYNCHRONISED):
            self.MAVServices.synchronised = False

            
    def send_all(self):
        try:
            self.MAVServices
        except:
            return
        
        self.MAVServices.send_all()
        self.doc.m_sync_in_progress()
        
                    
    def update_function(self, function_index):
        try:
            self.MAVServices
        except:
            return
        
        if(self.doc.auto_update == True):
            self.MAVServices.update_function(function_index)
             
 
    def commit_buffer_to_nvmem(self):
        try:
            self.MAVServices
        except:
            return False
            pass
        else:
            if(self.MAVServices.status == Status.CONNECTED):
                self.MAVServices.status = Status.WRITE_NVMEMORY
                return True
            else:
                return False

    def stop_services(self):
        print("mavlink services request stop")
        try:
            self.MAVServices
        except:
            return
        else:
            self.MAVServices.stop()

    def services_running(self):
        try:
            self.MAVServices
        except:
            return False
        else:
            return self.MAVServices.isAlive()


    def tx_msg_append(self, tx_msg):
        try:
            self.MAVServices
        except:
            print("MAV services thread does not exist, can not add message to tx queue")
            return False
        else:
            return self.MAVServices.tx_msg_append(tx_msg)

    def is_synchronised(self):
        try:
            self.MAVServices
        except:
            return False 
        return self.MAVServices.synchronised

    def set_not_synchronised(self):
        try:
            self.MAVServices
        except:
            return
        else:
            self.MAVServices.synchronised = False
            
    def msg_recv(self, msg):
        try:
            self.MAVServices
        except:
            return
        else:
            self.MAVServices.msg_recv(msg)
            return 
        

class MAVlink_services(threading.Thread):
    def __init__(self, mav_proc):
        threading.Thread.__init__(self)

        self.mav_proc = mav_proc
        self.MAVFSettings   = self.mav_proc.doc.MAVFSettings
        self.FBlocks        = self.mav_proc.doc.FBlocks
        
        self.DataGen = StructDataGen.structDataGen(self.MAVFSettings, self.FBlocks)

        self._stop = threading.Event()

        self.rx_q           = Queue.Queue(20)
                
        self.function_index = 0
        
        self.status = Status.NOT_STARTED
        
        self.heartbeat_time = 0
        
        self.synchronised = False
        
        self.timeout = time.time() + 1E6
        
        self.callback = None


    def stop(self):
        print("MAVlink service thread request stop")
        self._stop.set()

    def stopped(self):
        return not self.isAlive()

    def set_callback(self, callback):
        self.callback = callback

    def send_msg_poll(self):
        if(self.status == Status.START_SEND_ALL_FUNCTIONS):
            self.start_send_functions()
        if(self.status == Status.SEND_FUNCTIONS):
            self.function_index = 0
            self.status = Status.SENDING_FUNCTIONS
        if(self.status == Status.SENDING_FUNCTIONS):
            self.send_function()
        #=======================================================================
        # if(self.status == Status.SEND_SIZES):
        #        self.send_sizes()
        #=======================================================================
        if(self.status == Status.COMMITING_BUFFER):
            self.send_commit_buffer()
        if(self.status == Status.WRITE_NVMEMORY):
            self.send_write_nvmemory()
        if(self.status == Status.SEND_INPUT_DIRECTORY):
            self.send_input_directory()
        if(self.status == Status.SEND_OUTPUT_DIRECTORY):
            self.send_output_directory()
        if(self.status == Status.SEND_SINGLE_FUNCTION):
            self.send_single_function()

    def update_function(self, function_index):
        if(self.mav_proc.doc.auto_update == True):
            if(self.status == Status.CONNECTED):        
                if(self.synchronised == True):
                    self.function_index = function_index
                    self.status = Status.SEND_SINGLE_FUNCTION
                    return
                else:
                    self.send_all()
            else:
                self.mav_proc.doc.m_sync_fail()
            
    def send_all(self):
        if(self.status == Status.CONNECTED):
            self.status = Status.START_SEND_ALL_FUNCTIONS
        else:
            self.mav_proc.doc.m_sync_fail()

    def send_input_directory(self):
        self.timeout = time.time() + 1
        directory = self.DataGen.m_mavlinkInputDirectory()
        count = len(directory)
        
        for index in range(count, 49):
            directory.append(chr(0))
        
        data = ''
        for index in range(0 , 47):
            data += directory[index]
            
        self.mav_proc.mpstate.master().mav.flexifunction_directory_send(self.mav_proc.sysID, self.mav_proc.compID, 1, 0, count, data)

        self.status = Status.WAITING_INPUT_DIRECTORY_ACK


    def send_output_directory(self):
        self.timeout = time.time() + 1
        directory = self.DataGen.m_mavlinkOutputDirectory()
        count = len(directory)
        
        for index in range(count, 49):
            directory.append(chr(0))
        
        data = ''
        for index in range(0 , 47):
            data += directory[index]
        
        self.mav_proc.mpstate.master().mav.flexifunction_directory_send(self.mav_proc.sysID, self.mav_proc.compID, 0, 0, count, data )
        self.status = Status.WAITING_OUTPUT_DIRECTORY_ACK

    def start_send_functions(self):
        self.function_index = 0
        self.funcAddress = 0
        self.synchronised = False
        self.functionCount = len(self.MAVFSettings.functions.function)
        self.send_function()

    def send_single_function(self):
        function_data = self.DataGen.m_FunctionGenerateStruct(self.function_index)
        self.timeout = time.time() + 1
        self.funcAddress = 0xFFFF
        self.mav_proc.mpstate.master().mav.flexifunction_buffer_function_send(self.mav_proc.sysID, self.mav_proc.compID, self.function_index, self.functionCount, self.funcAddress, function_data.funcSize, function_data.funcData)
        self.status = Status.WAITING_COMMIT_BUFFER_ACK     
    
    def send_function(self):
        function_data = self.DataGen.m_FunctionGenerateStruct(self.function_index)
        self.timeout = time.time() + 1
        self.mav_proc.mpstate.master().mav.flexifunction_buffer_function_send(self.mav_proc.sysID, self.mav_proc.compID, self.function_index, self.functionCount, self.funcAddress, function_data.funcSize, function_data.funcData)
        self.funcAddress += function_data.funcSize
        self.status = Status.WAITING_FUNCTION_ACK

    #===========================================================================
    # def send_sizes(self):
    #    self.timeout = time.time() + 1
    #    self.mav_fd.mav.flexifunction_sizes_send(55, 1, 0, self.DataGen.FunctionsCount() )
    #    self.status = Status.WAITING_SIZES_ACK
    #===========================================================================

    def send_commit_buffer(self):
        self.timeout = time.time() + 3
        self.mav_proc.mpstate.master().mav.flexifunction_command_send(self.mav_proc.sysID, self.mav_proc.compID, Commands.COMMIT_BUFFER)
        self.status = Status.WAITING_COMMIT_BUFFER_ACK

    def send_write_nvmemory(self):
        self.timeout = time.time() + 3
        self.mav_proc.mpstate.master().mav.flexifunction_command_send(self.mav_proc.sysID, self.mav_proc.compID, Commands.WRITE_NVMEMORY)
        self.status = Status.WAITING_WRITE_NVMEMORY_ACK

        
    def parse_message(self, msg):
        if msg and msg.get_type() == "HEARTBEAT":
            self.heartbeat_time = time.time()
            if(self.status == Status.NOT_CONNECTED):
                self.on_connect()

        if msg and msg.get_type() == "FLEXIFUNCTION_BUFFER_FUNCTION_ACK":
#            print(msg)        
            target_system = msg.get_srcSystem()
            target_component = msg.get_srcComponent()
            if(msg.result != 0):
                self.function_index = self.function_index + 1
                if(self.function_index < self.DataGen.FunctionsCount() ):
                    self.status = Status.SENDING_FUNCTIONS
                else:
                    self.status = Status.SEND_OUTPUT_DIRECTORY
            else:
                self.callback(False)
                self.status = Status.CONNECTED
                self.timeout = time.time() + 1E6
#                self.status = Status.SENDING_FUNCTIONS

        if msg and msg.get_type() == "FLEXIFUNCTION_DIRECTORY_ACK":
 #           print(msg)        
            target_system = msg.get_srcSystem()
            target_component = msg.get_srcComponent()
            if(msg.result == 0):
                self.callback(False)
                self.status = Status.CONNECTED
                self.timeout = time.time() + 1E6
            else:
                if(msg.directory_type == 0):
                    self.status = Status.SEND_INPUT_DIRECTORY
                if(msg.directory_type == 1):
                    self.status = Status.COMMITING_BUFFER
                      
        if msg and msg.get_type() == "FLEXIFUNCTION_COMMAND_ACK":
  #          print(msg)        
            target_system = msg.get_srcSystem()
            target_component = msg.get_srcComponent()
            if(msg.command_type == Commands.COMMIT_BUFFER):
                if(msg.result == 1):
                    self.synchronised = True
                    self.mav_proc.doc.m_sync_complete()
                else:
                    self.synchronised = False
                    self.mav_proc.doc.m_sync_fail()  

                self.status = Status.CONNECTED
                self.timeout = time.time() + 1E6
                    
            if(msg.command_type == Commands.WRITE_NVMEMORY):
                if(msg.result == 1):
                    self.mav_proc.doc.m_NM_write_ok()  
                else:
                    self.mav_proc.doc.m_NM_write_fail()  
                self.status = Status.CONNECTED
                self.timeout = time.time() + 1E6


    def run(self):
        self._stop.clear()
        print("MAVlink service thread starting")
        
        while(not self._stop.isSet() ):
            try:
                msg_item = self.rx_q.get(True, 0.05)
            except Queue.Empty:
                pass
            else:
                # Receive a message
                self.parse_message(msg_item)
                self.rx_q.task_done()
  
            if(self.status != Status.NOT_CONNECTED):
                # if the heartbeat is lost reset the connection
                if((time.time() - self.heartbeat_time) > 5):
                    print("heartbeat timeout")
                    self.on_disconnect()
                # If the message times out, reset the state
                elif(time.time() > self.timeout):
                    self.mav_proc.doc.m_sync_fail()
                    self.status = Status.CONNECTED
                    self.timeout = time.time() + 1E6
                else:
                    self.send_msg_poll()
                 
        print("MAVlink service thread terminated")


    def on_disconnect(self):
        print("MAV disconnected")
        self.status = Status.NOT_CONNECTED
        self.timeout = time.time() + 1E6
        self.synchronised = False
        self.mav_proc.doc.m_disconnected()

 
    def on_connect(self):
        print("MAV connected")
        self.status = Status.CONNECTED
        self.timeout = time.time() + 1E6
        self.synchronised = False
        self.mav_proc.doc.m_connected()


    def msg_recv(self, msg):
        if(self.status == Status.NOT_STARTED):
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

