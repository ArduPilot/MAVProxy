import struct, array
import sys,os
import fnmatch
import ctypes

# find the mavlink.py module
#for d in [ 'pymavlink',
#          os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'MAVLINK', 'pymavlink') ]:
#    if os.path.exists(d):
#        sys.path.insert(0, d)


sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'MAVLink', 'pymavlink'))
        
import mavlinkv10 as mavlink

# find the ParameterDatabase.py module
#for d in [ 'pyparam',
#          os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'pyparam' ) ]:
#    if os.path.exists(d):
#        sys.path.insert(0, d)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', '..', 'pyparam'))

class nv_memory():
    def __init__(self):
        import ParameterDatabase as ParameterDB
        
        # determine if application is a script file or frozen exe
        if hasattr(sys, 'frozen'):
            self.application_path = os.path.dirname(sys.executable)
        elif __file__:
            self.application_path = os.path.dirname(__file__)
                    
        ParamDBPath = os.path.join(self.application_path, '..', '..', '..', '..', '..', "pyparam", "ParameterDatabase.xml")
        
        if(os.path.isfile(ParamDBPath)):
            self.ParamDBMain = ParameterDB.parse(ParamDBPath)        
            self.ParamDBLoaded = True
        else:
            self.ParamDBLoaded = False

    def get_memory_area_list(self):
        area_list = []
        if(self.ParamDBLoaded == True):
            dataAreas = self.ParamDBMain.get_dataStorageAreas().get_dataStorageArea()
            for dataArea in dataAreas:
                area_list.append(str(dataArea))
        return area_list        

    def get_parameter_area_list(self):
        area_list = []
        if(self.ParamDBLoaded == True):
            paramBlocks = self.ParamDBMain.get_parameterBlocks().get_parameterBlock()
            for paramBlock in paramBlocks:
                area_list.append(str(paramBlock.get_storage_area()))
        return area_list
    
#    def get_memory_area_index(self, param_area_name):
        
                

class PARAM_UNION(ctypes.Union):
    _fields_ = [("val_float", ctypes.c_float),
                ("val_int32", ctypes.c_int),
                ("val_uint32", ctypes.c_uint)]
    

class parameter():
    def __init__(self, param_id="DEFAULT" , param_value=0.0, param_type=mavlink.MAVLINK_TYPE_FLOAT, param_sync = False):
        self.param_id = param_id
        self.param_type   = param_type
        self.param_sync   = param_sync
        self.param_value  = PARAM_UNION(val_float=param_value)
        
        
    def param_value_set(self, param_value, param_type):
        self.param_sync = False
        
class memory_update():
    def __init__(self):
        self.memory_area = 0
        self.action = 0
        self.result = 0
        

class parameter_handler():
    def __init__(self, data_change_notifier):
    	self.data_change_notifier = data_change_notifier
    	self.clear()
        self.parameters = []
    	
    def update_msg(self, msg):
        if( msg.get_type() != "PARAM_VALUE"):
            return False
        
        if(len(self.parameters) != msg.param_count):
            self.clear()
            self.parameters = msg.param_count * [parameter]
            index = 0;
            for param in self.parameters:
                self.parameters[index] = parameter("DEFAULT", 0.0, mavlink.MAVLINK_TYPE_FLOAT, False)
                #param = parameter("DEFAULT", 0.0, mavlink.MAV_VAR_FLOAT, False)
                index += 1
                
    	self.parameters[msg.param_index] = parameter(msg.param_id, msg.param_value, msg.param_type, True)
        
        if(self.last_rx_parameter+1 != msg.param_index):
            print("bad parameter sync at msg %i" % msg.param_index)

        if msg.param_index+1 == msg.param_count:
            print("completed auto parameter refresh")
            return True
            
        self.last_rx_parameter = msg.param_index
        
        return False
        
    
    def process_param_messages(self, messages):
        k = messages.keys()
        k.sort()
        count = 0
        for p in k:
            if p and fnmatch.fnmatch(str(p).upper(), "*"):
                print("%-15.15s %f\n" % (p, messages[p]))
                count += 1

    def get_nonsync_param_index(self):
        index = 0;
        for param in self.parameters:
            if(param.param_sync == False):
                return index
            index += 1
        return -1
        
    def param_update_complete(self):
        self.data_change_notifier(self.parameters)
        

    def clear(self):
        self.parameters = []
        self.last_rx_parameter = -1
        self.updated_parameter = 0

    def nv_storage_action(self, msg):
        self.data_change_notifier(msg)
	