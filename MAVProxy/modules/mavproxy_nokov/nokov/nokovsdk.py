__author__ = 'Lika'

import os,platform
from ctypes import *

if platform.system().lower() == 'linux':
    System = "linux"
else:
    System = "windows"

print("Check System:%s" % System)

if platform.architecture()[0].lower() == '64bit':
    lib_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib' + os.path.sep)
else:
    lib_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib32' + os.path.sep)

curr_dir_before = os.getcwd()
os.chdir(lib_dir_path)
lib_path = os.path.join(lib_dir_path, 'libManagedSeekerSDKClient.so')

if (System == "linux"):
    CDLL(os.path.join(lib_dir_path, 'libnokov_sdk.so'))
    lib_path = os.path.join(lib_dir_path, 'libManagedSeekerSDKClient.so')
else:
    CDLL(os.path.join(lib_dir_path, 'nokov_sdk.dll'))
    lib_path = os.path.join(lib_dir_path, 'ManagedSeekerSDKClient.dll')

seekerLib = CDLL(lib_path)
print("Loaded nokovsdk")
os.chdir(curr_dir_before)

#Define Const Number
MAX_MODELS               =    200     # maximum number of MarkerSets 
MAX_RIGIDBODIES          =    1000    # maximum number of RigidBodies
MAX_NAMELENGTH           =    256     # maximum length for strings
MAX_MARKERS              =   200     # maximum number of markers per MarkerSet
MAX_RBMARKERS             =  20       # maximum number of markers per RigidBody
MAX_SKELETONS            =   100     # maximum number of skeletons
MAX_SKELRIGIDBODIES      =   200     # maximum number of RididBodies per Skeleton
MAX_LABELED_MARKERS      =  1000    # maximum number of labeled markers per frame
MAX_UNLABELED_MARKERS    =  1000    # maximum number of unlabeled (other) markers per frame

MAX_FORCEPLATES          =  8       # maximum number of force plates
MAX_ANALOG_CHANNELS      =  32      # maximum number of data channels (signals) per analog/force plate device
MAX_ANALOG_SUBFRAMES     =  30      # maximum number of analog/force plate frames per mocap frame
MAX_PACKETSIZE			  = 300000	 # max size of packet (actual packet size is dynamic)


# Custom Structure
class InfoStructure(Structure):
    # Dump to Dict
    def dump_dict(self):
        info = {}
        # Get The _fields_
        # Check the Field Type
        # Support Recursion and iteration
        for k, v in self._fields_:
            av = getattr(self, k)
            if type(v) == type(Structure):
                av = av.dump_dict()
            elif type(v) == type(Array):
                av = cast(av, c_char_p).value
            else:
                pass
            info[k] = av
        return info

    def __str__(self):
        info = self.dump_dict()
        return repr(info)

    # Print Info
    def show(self):
        from pprint import pprint
        pprint(self.dump_dict())


# for ctypes
class ServerDescription(InfoStructure):
    _fields_ = [
        ("HostPresent", c_bool),
        ("szHostComputerName", c_char * MAX_NAMELENGTH),
        ("HostComputerAddress", c_ubyte * 4),
        ("szHostApp",  c_char * MAX_NAMELENGTH),
        ("HostAppVersion", c_ubyte * 4),
        ("HostApSeekerSDKVersionpVersion", c_ubyte * 4),
    ]

class Marker(InfoStructure):
    _fields_ = [
        ("ID", c_int),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("size", c_float),
        ("params", c_short),
    ]

class MarkerSetDescription(InfoStructure):
    _fields_ = [
        ("szName", c_char * MAX_NAMELENGTH),
        ("nMarkers", c_int),
        ("szMarkerNames", POINTER(c_char_p)),
    ]

class MarkerSetData(InfoStructure):
    _fields_ = [
        ("szName", c_char * MAX_NAMELENGTH),
        ("nMarkers", c_int),
        ("Markers", POINTER(c_float * 3)),
    ]

class RigidBodyDescription(InfoStructure):
    _fields_ = [
        ("szName", c_char * MAX_NAMELENGTH),
        ("ID", c_int),
        ("parentID", c_int),
        ("offsetx", c_float),
        ("offsety", c_float),
        ("offsetz", c_float),     
    ]

class RigidBodyData(InfoStructure):
    _fields_ = [
        ("ID", c_int),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("qx", c_float),
        ("qy", c_float),
        ("qz", c_float),
        ("qw", c_float),
        ("nMarkers", c_int),
        ("Markers", POINTER(c_float * 3)),
        ("MarkerIDs", POINTER(c_int)),
        ("MarkerSizes", POINTER(c_float)),
        ("MeanError", c_float),
        ("params", c_short),
    ]

class SkeletonData(InfoStructure):
    _fields_ = [
        ("skeletonID", c_int),
        ("nRigidBodies", c_int),
        ("RigidBodyData", POINTER(RigidBodyData)),
    ]

class SkeletonDescription(InfoStructure):
    _fields_ = [
        ("szName", c_char * MAX_NAMELENGTH),
        ("skeletonID", c_int),
        ("nRigidBodies", c_int),
        ("RigidBodies", RigidBodyDescription * MAX_SKELRIGIDBODIES),
    ]

class ForcePlateData(InfoStructure):
    _fields_ = [
        ("Fxyz", c_float * 3),
        ("xyz", c_float * 3),
        ("Mfree", c_float),
    ]

class ForcePlates(InfoStructure):
    _fields_ = [
        ("iFrame", c_int),
        ("nForcePlates", c_int),
        ("ForcePlates", ForcePlateData * MAX_FORCEPLATES),
    ]

class ForcePlateDescription(InfoStructure):
    _fields_ = [
        ("ID", c_int),
        ("scale", c_int),
        ("fWidth", c_float),
        ("fLength", c_float),
        ("Position", c_float * 3),
        ("Electrical", c_float * 3),
        ("Orientation", (c_float * 3) * 3),
        ("fCalMat", (c_float * 8) * 8),
        ("fCorners", (c_float * 3) * 4),
        ("iPlateType", c_int),
        ("iChannelDataType", c_int),
        ("nChannels", c_int),
        ("szChannelNames", (c_char * MAX_NAMELENGTH) * MAX_ANALOG_CHANNELS),
    ]

class Data(Union):
    _fields_ = [
        ("MarkerSetDescription", POINTER(MarkerSetDescription)),
        ("RigidBodyDescription", POINTER(RigidBodyDescription)),
        ("SkeletonDescription", POINTER(SkeletonDescription)),
        ("ForcePlateDescription", POINTER(ForcePlateDescription)),
    ]

class DataDescription(InfoStructure):
    _fields_ = [
        ("type", c_int),
        ("Data", Data),
    ]

class DataDescriptions(InfoStructure):
    _fields_ = [
        ("nDataDescriptions", c_int),
        ("arrDataDescriptions", DataDescription * MAX_MODELS),
    ]

class FrameOfMocapData(InfoStructure):
    _fields_ = [    
        ("iFrame", c_int),
        ("nMarkerSets", c_int),
        ("MocapData", MarkerSetData * MAX_MODELS),
        ("nOtherMarkers", c_int),
        ("OtherMarkers", POINTER(c_float * 3)),
        ("nRigidBodies", c_int),
        ("RigidBodies", RigidBodyData * MAX_RIGIDBODIES),
        ("nSkeletons", c_int),
        ("Skeletons", SkeletonData * MAX_SKELETONS),
        ("nLabeledMarkers", c_int),
        ("LabeledMarkers", Marker * MAX_LABELED_MARKERS),
        ("nAnalogdatas", c_int),
        ("Analogdata", c_float * MAX_ANALOG_CHANNELS),
        ("fLatency", c_float),
        ("Timecode", c_uint),
        ("TimecodeSubframe", c_uint),
        ("iTimeStamp", c_longlong),
        ("params", c_short),
    ]

# CallBack
MSGFUNC = CFUNCTYPE(None, c_int, POINTER(c_char))
DATAHANDLEFUNC = CFUNCTYPE(None, POINTER(FrameOfMocapData), c_void_p)
FORCEPLATEFUNC = CFUNCTYPE(None, POINTER(ForcePlates), c_void_p)

# for test
def py_msg_func(iLogLevel, szLogMessage):
    print(iLogLevel, szLogMessage)

def py_data_func(frameData, userData):
    print(pFrameOfData)

def py_force_plate_func(forceData, userData):
    print(forceData)

msg_func = MSGFUNC(py_msg_func)
data_func = DATAHANDLEFUNC(py_data_func)
forcePlate_func = FORCEPLATEFUNC(py_force_plate_func)

class PySDKClient():
    __pClient = c_void_p()

    def __init__(self):
        retVal = self.__CreateClient()
        if (retVal != 0):
            raise ValueError('__CreateClient Failed')
            
    def __del__(self):
        self.__Uninitialize()
        self.__DestroyClient()

    def __CreateClient(self):
        seekerLib.CreateClient.argtypes = [c_void_p]
        return seekerLib.CreateClient(byref(self.__pClient))

    def Initialize(self, szServerAddress):
        seekerLib.Initialize.argtypes = [c_void_p,  c_char_p]
        return seekerLib.Initialize(self.__pClient, szServerAddress)

    def __DestroyClient(self):
        seekerLib.DestroyClient.argtypes = [c_void_p]
        return seekerLib.DestroyClient(self.__pClient)

    def __Uninitialize(self):
        seekerLib.Uninitialize.argtypes = [c_void_p]
        return seekerLib.Uninitialize(self.__pClient)

    def PySeekerVersion(self):
        seekerLib.SeekerVersion.argtypes = [c_void_p, POINTER(c_ubyte)]
        Version = (c_ubyte * 4)(0,0,0,0)
        seekerLib.SeekerVersion(self.__pClient, Version)
        return Version

    def PySetVerbosityLevel(self, ilevel):
        seekerLib.SetVerbosityLevel.argtypes = [c_void_p, c_int]
        return seekerLib.SetVerbosityLevel(self.__pClient, ilevel)

    def PySetDataCallback(self, dataHandler, pUserData):
        seekerLib.SetDataCallback.argtypes = [c_void_p, DATAHANDLEFUNC, c_void_p]
        if dataHandler != None:
            global data_func
            data_func = DATAHANDLEFUNC(dataHandler)
        return seekerLib.SetDataCallback(self.__pClient, data_func, pUserData)

    def PySetForcePlateCallback(self, dataHandler, pUserData):
        seekerLib.SetForcePlateCallback.argtypes = [c_void_p, FORCEPLATEFUNC, c_void_p]
        if dataHandler != None:
            global forcePlate_func
            forcePlate_func = FORCEPLATEFUNC(dataHandler)
        return seekerLib.SetForcePlateCallback(self.__pClient, forcePlate_func, pUserData)        

    def PyWaitForForcePlateInit(self, time):
        seekerLib.WaitForForcePlateInit.argtypes = [c_void_p, c_long]
        return seekerLib.WaitForForcePlateInit(self.__pClient, time)

    def PySetMessageCallback(self, msgHandler):
        seekerLib.SetMessageCallback.argtypes= [c_void_p, MSGFUNC]
        if msgHandler != None:
            global msg_func
            msg_func = MSGFUNC(msgHandler)
        return seekerLib.SetMessageCallback(self.__pClient, msg_func)

    def PySendMessage(self, szMessage):
        seekerLib.SendMessage.argtypes = [c_void_p, c_char_p]
        return seekerLib.SendMessage(self.__pClient, szMessage)

    def PyGetServerDescription(self, pServerDescription):
        seekerLib.GetServerDescription.argtypes = [c_void_p, POINTER(ServerDescription)]
        return seekerLib.GetServerDescription(self.__pClient, pServerDescription)

    def PyGetDataDescriptions(self, phDataDescriptions):
        seekerLib.GetDataDescriptions.argtypes = [c_void_p, POINTER(POINTER(DataDescriptions))]
        return seekerLib.GetDataDescriptions(self.__pClient, byref(phDataDescriptions))

    def PyGetDataDescriptionsEx(self, phDataDescriptions, handle):
        seekerLib.GetDataDescriptionsEx.argtypes = [c_void_p, POINTER(DataDescriptions), c_void_p]
        return seekerLib.GetDataDescriptionsEx(self.__pClient, byref(phDataDescriptions), byref(handle))

    def PyFreeDataDescriptionsEx(self, handle):
        seekerLib.FreeDataDescriptionsEx.argtypes = [c_void_p, c_void_p]
        return seekerLib.FreeDataDescriptionsEx(self.__pClient, handle)

    def PySetMulticastAddress(self, szMulticast):
        seekerLib.SetMulticastAddress.argtypes = [c_void_p, c_char_p]
        return seekerLib.SetMulticastAddress(self.__pClient, szMulticast)

    def PyDecodeTimecode(self, inTimecode, inTimecodeSubframe, hour, minute, second, frame, subframe):
        seekerLib.DecodeTimecode.argtypes = [c_void_p, c_uint, c_uint, Pointer(c_int), Pointer(c_int), Pointer(c_int), Pointer(c_int), Pointer(c_int)]
        return seekerLib.DecodeTimecode(self.__pClient, inTimecode, inTimecodeSubframe, hour, minute, second, frame, subframe)

    def PyTimecodeStringify(self, inTimecode, inTimecodeSubframe, Buffer, BufferSize):
        seekerLib.TimecodeStringify.argtypes = [c_void_p, c_uint, c_uint, c_char_p, c_int]
        return seekerLib.TimecodeStringify(self.__pClient, inTimecode, inTimecodeSubframe, Buffer, BufferSize)

