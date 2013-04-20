
import struct, array

class callback_messages(object):
# Status changes
    OFFLINE = 0
    ONLINE = 1
    READ_IN_PROGRESS = 2
    READ_COMPLETE = 4
    READ_FAIL = 6
    NV_MEM_WRITE_OK     = 50
    NV_MEM_WRITE_FAIL   = 55
    NV_MEM_READ_OK      = 60
    NV_MEM_READ_FAIL    = 65
    NV_MEM_CLEAR_OK     = 70
    NV_MEM_CLEAR_FAIL   = 75
    
#document changes
    PARAMETER_CHANGED = 100
    PARAMETERS_LOADED = 110

# actions    
    READ_ALL_PARAMS  = 200
    READ_PARAM      = 210
    WRITE_NV_AREA   = 220
    CLEAR_NV_AREA   = 230
    READ_NV_AREA    = 240
    WRITE_ALL_PARAMS = 250
    WRITE_CHANGED_PARAMS = 260
