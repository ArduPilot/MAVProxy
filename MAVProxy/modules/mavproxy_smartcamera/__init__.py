#!/usr/bin/env python
#***************************************************************************
#                      Copyright Jaime Machuca
#***************************************************************************
# Title        : mavproxy_smartcamera.py
#
# Description  : This file is intended to be added as a module to MAVProxy,
#                it is intended to be used to control smart cameras that are
#                connected to a companion computer. It reads MAVlink commands
#                and uses them to control the cameras attached. The module
#                reads a configuration file called smart_camera.cnf that tells
#                it what cameras are connected, it then tries to connect to the
#                cameras and populates a list of available cameras.
#
# Environment  : Python 2.7 Code. Intended to be included in MAVproxy as a Module
#
# Responsible  : Jaime Machuca
#
# License      : CC BY-NC-SA
#
# Editor Used  : Xcode 6.1.1 (6A2008a)
#
#****************************************************************************

#****************************************************************************
# HEADER-FILES (Only those that are needed in this file)
#****************************************************************************

# System Header files and Module Headers
import time, math, sched

# Module Dependent Headers
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting


# Own Headers
from sc_webcam import SmartCameraWebCam
from sc_SonyQX1 import SmartCamera_SonyQX
import sc_config

#****************************************************************************
# LOCAL DEFINES
#****************************************************************************


#****************************************************************************
# Class name       : SmartCameraModule
#
# Public Methods   : init
#                    mavlink_packet
#
# Private Methods  : __vRegisterCameras
#                    __vCmdCamTrigger
#
#****************************************************************************
class SmartCameraModule(mp_module.MPModule):

#****************************************************************************
#   Method Name     : __init__ Class Initializer
#
#   Description     : Initializes the class
#
#   Parameters      : mpstate
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************
    
    def __init__(self, mpstate):
        super(SmartCameraModule, self).__init__(mpstate, "SmartCamera", "SmartCamera commands")
        self.add_command('camtrigger', self.__vCmdCamTrigger, "Trigger camera")
        self.add_command('connectcams', self.__vCmdConnectCameras, "Connect to Cameras")
        self.add_command('setCamISO', self.__vCmdSetCamISO, "Set Camera ISO")
        self.add_command('setCamAperture', self.__vCmdSetCamAperture, "Set Camera Aperture")
        self.add_command('setCamShutterSpeed', self.__vCmdSetCamShutterSpeed, "Set Camera Shutter Speed")
        self.add_command('setCamExposureMode', self.__vCmdSetCamExposureMode, "Set Camera Exposure Mode")
        self.CamRetryScheduler = sched.scheduler(time.time, time.sleep)
        self.ProgramAuto = 1
        self.Aperture = 2
        self.Shutter = 3
        self.Manual = 4
        self.IntelligentAuto = 5
        self.SuperiorAuto = 6
        self.WirelessPort = "wlan0"
        self.u8RetryTimeout = 0
        self.u8MaxRetries = 5
        self.__vRegisterCameras()
 
 #****************************************************************************
 #   Method Name     : __vRegisterQXCamera
 #
 #   Description     : Tries to connect to a QX camera on the specified Wireless
 #                     port. If no camera is found it will retry every 5 seconds
 #                     until u8MaxRetries is reached.
 #
 #   Parameters      : None
 #
 #   Return Value    : None
 #
 #   Autor           : Jaime Machuca
 #
 #****************************************************************************

    def __vRegisterQXCamera(self,u8CamNumber):
        if (self.u8RetryTimeout < self.u8MaxRetries):
            new_camera = SmartCamera_SonyQX(u8CamNumber, self.WirelessPort)
            if new_camera.boValidCameraFound() is True:
                self.camera_list = self.camera_list + [new_camera]
                print("Found QX Camera")
            else:
                print("No Valid Camera Found, retry in 5 sec")
                self.u8RetryTimeout = self.u8RetryTimeout + 1
                self.CamRetryScheduler.enter(5, 1, self.__vRegisterQXCamera, [u8CamNumber])
                self.CamRetryScheduler.run()
        else:
            print("Max retries reached, No QX Camera Found")
            self.u8RetryTimeout = 0

#****************************************************************************
#   Method Name     : __vRegisterCameras
#
#   Description     : Creates camera objects based on camera-type configuration
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vRegisterCameras(self):
        
        # initialise list
        self.camera_list = []
        
        #look for up to 2 cameras
        for i in range(0,2):
            config_group = "camera%d" % i
            camera_type = sc_config.config.get_integer(config_group, 'type', 0)
            # webcam
            if camera_type == 1:
                new_camera = SmartCameraWebCam(i)
                self.camera_list = self.camera_list + [new_camera]
            
            # Sony QX1
            if camera_type == 2:
                self.__vRegisterQXCamera(i)
    
        # display number of cameras found
        print ("cameras found: %d" % len(self.camera_list))

#****************************************************************************
#   Method Name     : __vCmdCamTrigger
#
#   Description     : Triggers all the cameras and stores Geotag information
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamTrigger(self, args):
        '''Trigger Camera'''
        #print(self.camera_list)
        for cam in self.camera_list:
            cam.take_picture()
            print("Trigger Cam %s" % cam)

#****************************************************************************
#   Method Name     : __vCmdConnectCameras
#
#   Description     : Initiates connection to cameras
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdConnectCameras(self, args):
        '''ToDo: Validate the argument as a valid port'''
        if len(args) >= 1:
            self.WirelessPort = args[0]
        print ("Connecting to Cameras on %s" % self.WirelessPort)
        self.__vRegisterCameras()

#****************************************************************************
#   Method Name     : __vCmdSetCamExposureMode
#
#   Description     : Sets the camera exposure mode
#
#   Parameters      : Exposure Mode, Cam number
#                     Valid values are Program Auto, Aperture, Shutter, Manual
#                     Intelligent Auto, Superior Auto
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamExposureMode(self, args):
        '''ToDo: Validate CAM number and Valid Mode Values'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetExposureMode(args[0])
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetExposureMode(args[0])
        else:
            print ("Usage: setCamExposureMode MODE [CAMNUMBER], Valid values for MODE: Program Auto, Aperture, Shutter, Manual, Intelligent Auto, Superior Auto")

#****************************************************************************
#   Method Name     : __vCmdSetCamAperture
#
#   Description     : Sets the camera aperture
#
#   Parameters      : Aperture Value, Cam number
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamAperture(self, args):
        '''ToDo: Validate CAM number and Valid Aperture Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetAperture(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetAperture(int(args[0]))
        else:
            print ("Usage: setCamAperture APERTURE [CAMNUMBER], APERTURE is value x10")

#****************************************************************************
#   Method Name     : __vCmdSetCamShutterSpeed
#
#   Description     : Sets the shutter speed for the camera
#
#   Parameters      : Shutter speed, Cam Number
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamShutterSpeed(self, args):
        '''ToDo: Validate CAM number and Valid Shutter Speed'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetShutterSpeed(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetShutterSpeed(int(args[0]))
        else:
            print ("Usage: setCamShutterSpeed SHUTTERVALUE [CAMNUMBER], Shutter value is the devisor in 1/x (only works for values smaller than 1)")

#****************************************************************************
#   Method Name     : __vCmdSetCamISO
#
#   Description     : Sets the ISO value for the camera
#
#   Parameters      : ISO Value, Cam Number
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamISO(self, args):
        '''ToDo: Validate CAM number and Valid ISO Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetISO(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetISO(int(args[0]))
        else:
            print ("Usage: setCamISO ISOVALUE [CAMNUMBER]")

#****************************************************************************
#   Method Name     : __vCmdCamZoomIn
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomIn(self):
        for cam in self.camera_list:
            cam.boZoomIn()
 
#****************************************************************************
#   Method Name     : __vCmdCamZoomOut
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomOut(self):
        for cam in self.camera_list:
            cam.boZoomOut()
 
#****************************************************************************
#   Method Name     : __vDecodeDIGICAMConfigure
#
#   Description     : Decode and process the camera configuration Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMConfigure(self, mCommand_Long):
        if mCommand_Long.param1 != 0:
            print ("Exposure Mode = %d" % mCommand_Long.param1)
            
            if mCommand_Long.param1 == self.ProgramAuto:
                self.__vCmdSetCamExposureMode(["Program Auto"])
            
            elif mCommand_Long.param1 == self.Aperture:
                self.__vCmdSetCamExposureMode(["Aperture"])
            
            elif mCommand_Long.param1 == self.Shutter:
                self.__vCmdSetCamExposureMode(["Shutter"])

        '''Shutter Speed'''
        if mCommand_Long.param2 != 0:
            print ("Shutter Speed= %d" % mCommand_Long.param2)
            self.__vCmdSetCamShutterSpeed([mCommand_Long.param2])
        
        '''Aperture'''
        if mCommand_Long.param3 != 0:
            print ("Aperture = %d" % mCommand_Long.param3)
            self.__vCmdSetCamAperture([mCommand_Long.param3])

        '''ISO'''
        if mCommand_Long.param4 != 0:
            print ("ISO = %d" % mCommand_Long.param4)
            self.__vCmdSetCamISO([mCommand_Long.param4])

        '''Exposure Type'''
        if mCommand_Long.param5 != 0:
            print ("Exposure type= %d" % mCommand_Long.param5)


#****************************************************************************
#   Method Name     : __vDecodeDIGICAMControl
#
#   Description     : Decode and process the camera control Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMControl(self, mCommand_Long):
        '''Session'''
        if mCommand_Long.param1 != 0:
            print ("Session = %d" % mCommand_Long.param1)
        
        '''Zooming Step Value'''
        if mCommand_Long.param2 != 0:
            print ("Zooming Step = %d" % mCommand_Long.param2)
        
        '''Zooming Step Value'''
        if mCommand_Long.param3 != 0:
            print ("Zooming Value = %d" % mCommand_Long.param3)

            if (mCommand_Long.param3 == 1):
                self.__vCmdCamZoomIn()
            elif (mCommand_Long.param3 == -1):
                self.__vCmdCamZoomOut()
            else:
                print ("Invalid Zoom Value")
        
        '''Focus 0=Unlock/1=Lock/2=relock'''
        if mCommand_Long.param4 != 0:
            print ("Focus = %d" % mCommand_Long.param4)
        
        '''Trigger'''
        if mCommand_Long.param5 != 0:
            print ("Trigger = %d" % mCommand_Long.param5)
            self.__vCmdCamTrigger(mCommand_Long)



#****************************************************************************
#   Method Name     : mavlink_packet
#
#   Description     : MAVProxy requiered callback function used to recieve MAVlink
#                     packets
#
#   Parameters      : MAVLink Message
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "CAMERA_STATUS":
            print ("Got Message camera_status")
        if mtype == "CAMERA_FEEDBACK":
            print ("Got Message camera_feedback")
            '''self.__vCmdCamTrigger(m)'''
        if mtype == "COMMAND_LONG":
            if m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE:
                print ("Got Message Digicam_configure")
                self.__vDecodeDIGICAMConfigure(m)
            elif m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
                print ("Got Message Digicam_control")
                self.__vDecodeDIGICAMControl(m)

#****************************************************************************
#   Method Name     : init
#
#   Description     :
#
#   Parameters      : mpstate
#
#   Return Value    : SmartCameraModule Instance
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

def init(mpstate):
    '''initialise module'''
    return SmartCameraModule(mpstate)
