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
import time, math

# Module Dependent Headers
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting


# Own Headers
from sc_webcam import SmartCameraWebCam
from sc_SonyQX1 import SmartCamera_SonyQX
import sc_config

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
        self.__vRegisterCameras()
 
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
                new_camera = SmartCamera_SonyQX(i,"wlan0")
                if new_camera.boValidCameraFound() is True:
                    self.camera_list = self.camera_list + [new_camera]
                    print("Found QX Camera")

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

    def __vCmdCamTrigger(self, CAMERA_FEEDBACK):
        '''Trigger Camera'''
        print(self.camera_list)
        for cam in self.camera_list:
            cam.take_picture()
            print("Trigger Cam %s" % cam)
            print ("Latitude: %f" % CAMERA_FEEDBACK.lat)
            print ("Longitude: %f" % CAMERA_FEEDBACK.lng)
            print ("Altitude: %f" % CAMERA_FEEDBACK.alt_msl)

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
            print ("Got Message camera_feedback triggering Cameras")
            self.__vCmdCamTrigger(m)
        if mtype == "COMMAND_LONG":
            print ("Recieved Command Long")
            COMMAND_LONG = m
            print COMMAND_LONG.command
            print COMMAND_LONG.param1
            print COMMAND_LONG.param2
            print COMMAND_LONG.param3
            print COMMAND_LONG.param4
            print COMMAND_LONG.param5
            print COMMAND_LONG.param6
            print COMMAND_LONG.param7

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
