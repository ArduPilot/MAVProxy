#***************************************************************************
#                      Copyright Jaime Machuca
#***************************************************************************
# Title        : sc_SonyQX1.py
#
# Description  : This file contains a class to use the Sony QX range of cams
#                It finds a camera using SSDP discovery and returns it as an
#                object. If a camera is not found it returns an error value
#                that should be catched by the application. Initially it will
#                have support for triggering the camera, and downloading the
#                latest image file. Other functions will be added gradually.
#
# Environment  : Python 2.7 Code. Intended to be included in a Mavproxy Module
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
import sys, time, math, cv2, struct, fcntl

# Module Dependent Headers
import requests, json, socket, StringIO
import xml.etree.ElementTree as ET

# Own Headers
import ssdp

#****************************************************************************
# Class name       : SmartCamera_SonyQX
#
# Public Methods   : boGetLatestImage
#                    u32GetImageCounter
#                    boTakePicture
#                    boSetExposureMode
#                    boSetShutterSpeed
#                    boSetAperture
#                    boSetISO
#                    boZoomIn
#                    boZoomOut
#
# Private Methods  : __sFindInterfaceIPAddress
#                    __sFindCameraURL
#                    __sMakeCall
#                    __sSimpleCall
#****************************************************************************
class SmartCamera_SonyQX():
    
#****************************************************************************
#   Method Name     : __init__ Class Initializer
#
#   Description     : Initializes the class
#
#   Parameters      : u8instance        Camera Instance Number
#                     snetInterface     String containing the Network Interface
#                                       Name where we should look for the cam
#
#   Return Value    : None
#
#   Autor           : Jaime Machuca, Randy Mackay
#
#****************************************************************************

    def __init__(self, u8Instance, sNetInterface):

        # record instance
        self.u8Instance = u8Instance
        self.sConfigGroup = "Camera%d" % self.u8Instance

        # background image processing variables
        self.u32ImgCounter = 0              # num images requested so far

        # latest image captured
        self.sLatestImageURL = None         # String with the URL to the latest image
        
        # latest image downloaded
        self.sLatestImageFilename = None    #String with the file name for the last downloaded image
    
        # Look Camera and Get URL
        self.sCameraURL = self.__sFindCameraURL(sNetInterface)
        if self.sCameraURL is None:
            print("No QX camera found, failed to open QX camera %d" % self.u8Instance)

#****************************************************************************
#   Method Name     : __str__
#
#   Description     : Returns a human readable string name for the instance
#
#   Parameters      : none
#
#   Return Value    : String with object instance name
#
#   Autor           : Randy Mackay
#
#****************************************************************************

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraSonyQX Object for %s" % self.sConfigGroup

#****************************************************************************
#   Method Name     : __sFindInterfaceIPAddress
#
#   Description     : Gets the IP Address of the interface name requested
#
#   Parameters      : sInterfaceName
#
#   Return Value    : String with the IP Address for the requested interface
#
#   Autor           : Jaime Machuca,
#
#****************************************************************************

    def __sFindInterfaceIPAddress(self,sInterfaceName):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
                                            s.fileno(),
                                            0x8915,  # SIOCGIFADDR
                                            struct.pack('256s', sInterfaceName[:15])
                                            )[20:24])

#****************************************************************************
#   Method Name     : __sMakeCall
#
#   Description     : Sends a json encoded command to the QX camera URL
#
#   Parameters      : sService
#                     adictPayload
#
#   Return Value    : JSON encoded string with camera response
#
#   Autor           : Andrew Tridgell, Jaime Machuca
#
#****************************************************************************

    def __sMakeCall(self, sService, adictPayload):
        sURL = "%s/%s" % (self.sCameraURL, sService)
        adictHeaders = {"content-type": "application/json"}
        sData = json.dumps(adictPayload)
        sResponse = requests.post(sURL,
                                  data=sData,
                                  headers=adictHeaders).json()
        return sResponse
    
#****************************************************************************
#   Method Name     : __sSimpleCall
#
#   Description     : Articulates a camera service command to send to the QX
#                     camera
#
#   Parameters      : sMethod, command name as stated in Sony's API documentation
#                     sTarget, API Service type
#                     adictParams, command specific parameters (see Sony's API Documentation)
#                     u8Id, ??
#                     sVersion, API version for the command (see Sony's API Documentation)
#
#   Return Value    : JSON encoded string with camera response
#
#   Autor           : Andrew Tridgell, Jaime Machuca
#
#****************************************************************************

    def __sSimpleCall(self, sMethod, sTarget="camera", adictParams=[], u8Id=1, sVersion="1.0"):
        print("Calling %s" % sMethod)
        return self.__sMakeCall(sTarget,
                              { "method" : sMethod,
                              "params" : adictParams,
                              "id"     : u8Id,
                              "version" : sVersion })

#****************************************************************************
#   Method Name     : __sFindCameraURL
#
#   Description     : Sends an SSDP request to look for a QX camera on the
#                     specified network interface
#
#   Parameters      : sInterface, String with the network interface name
#
#   Return Value    : String containing the URL for sending commands to the
#                     Camera
#
#   Autor           : Andrew Tridgell, Jaime Machuca
#
#****************************************************************************

    def __sFindCameraURL(self, sInterface):
        sSSDPString = "urn:schemas-sony-com:service:ScalarWebAPI:1";
        sInterfaceIP = self.__sFindInterfaceIPAddress(sInterface)
        print ("Interface IP Address: %s" % sInterfaceIP)
        sRet = ssdp.discover(sSSDPString, if_ip=sInterfaceIP)
        if len(sRet) == 0:
            return None
        sDMS_URL = sRet[0].location
        
        print("Fetching DMS from %s" % sDMS_URL)
        xmlReq = requests.request('GET', sDMS_URL)
        
        xmlTree = ET.ElementTree(file=StringIO.StringIO(xmlReq.content))
        for xmlElem in xmlTree.iter():
            if xmlElem.tag == '{urn:schemas-sony-com:av}X_ScalarWebAPI_ActionList_URL':
                print("Found camera at %s" % xmlElem.text)
                return xmlElem.text
        return None

#****************************************************************************
#   Method Name     : boValidCameraFound
#
#   Description     : Returns weather or not a camera has been found. This
#                     should be used to try to find the camera again, or
#                     destroy the object.
#
#   Parameters      : none
#
#   Return Value    : True if camera has been found
#                     False if no camera has been found
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boValidCameraFound(self):
        print ("Checking URL at %s" % self.sCameraURL)
        if self.sCameraURL is None:
            return False
        return True

#****************************************************************************
#   Method Name     : boGetLatestImage
#
#   Description     : Dowloads the latest image taken by the camera and then
#                     saves it to a file name composed by the camera instance
#                     and image number.
#
#   Parameters      : none
#
#   Return Value    : True if it was succesful
#                     False if no image was downloaded
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boGetLatestImage(self):
        self.sLatestImageFilename = '%s_image_%s.jpg' % (self.sConfigGroup,self.u32ImgCounter)
        print ("Downloading, ",self.sLatestImageFilename)
        imgReq = requests.request('GET', self.sLatestImageURL)
        if imgReq is not None:
            open(self.sLatestImageFilename, 'w').write(imgReq.content)
            return True
        return False

#****************************************************************************
#   Method Name     : sGetLatestImageFilename
#
#   Description     : Returns the filename of the last image downloaded from
#                     the camera
#
#   Parameters      : none
#
#   Return Value    : String containing the image file name
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def sGetLatestImageFilename(self):
        return self.sLatestImageFilename
    
#****************************************************************************
#   Method Name     : u32GetImageCounter
#
#   Description     : Returns the number of images taken
#
#   Parameters      : none
#
#   Return Value    : Integer with the number of images
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def u32GetImageCounter(self):
        return self.u32ImgCounter

#****************************************************************************
#   Method Name     : boZoomIn
#
#   Description     : Commands the camera to do a Zoom In step
#
#   Parameters      : None
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boZoomIn(self):

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("actZoom", adictParams=["in","1shot"])
        
        # Check response for a succesful result
        if 'result' in sResponse:
            print ("Zoomed in")
            return True
        
        # In case of an error, return false
        print ("Failed to Zoom")
        return False

#****************************************************************************
#   Method Name     : boZoomOut
#
#   Description     : Commands the camera to do a Zoom In step
#
#   Parameters      : None
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boZoomOut(self):
        
        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("actZoom", adictParams=["out","1shot"])
        
        # Check response for a succesful result
        if 'result' in sResponse:
            print ("Zoomed out")
            return True

        # In case of an error, return false
        print ("Failed to Zoom")
        return False

#****************************************************************************
#   Method Name     : boSetExposureMode
#
#   Description     : Commands the camera to set a specific ShootingMode
#
#   Parameters      : Exposure Mode String
#                     Program Auto, Aperture, Shutter, Manual, Intelligent Auto, Superior Auto
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boSetExposureMode(self,sExposureMode):
        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setExposureMode", adictParams=[sExposureMode])
        
        # Check response for a succesful result
        if 'result' in sResponse:
            time.sleep(0.25)
            sResponse = self.__sSimpleCall("getExposureMode")
            
            if sExposureMode not in sResponse["result"]:
                print ("Failed to set Exposure Mode, current value: %s" %sResponse["result"])
                return False
            
            print ("Exposure Mode set to %s" % sExposureMode)
            return True
        
        # In case of an error, return false
        print ("Failed to set Exposure Mode")
        return False

#****************************************************************************
#   Method Name     : boSetShutterSpeed
#
#   Description     : Commands the camera to set the Shutter Speed
#
#   Parameters      : Integer with the shutter speed divisor
#                     i.e. 1/1000 = 1000
#                     NOTE: This will only work for shutter speeds smaller than 1 sec
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boSetShutterSpeed(self,u16ShutterSpeed):
        # Create Shutter Speed String
        sShutterSpeed = "1/%s" % str(u16ShutterSpeed)

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setShutterSpeed", adictParams=[sShutterSpeed])
            
        # Check response for a succesful result
        if 'result' in sResponse:
            time.sleep(0.25)
            sResponse = self.__sSimpleCall("getShutterSpeed")
            
            if sShutterSpeed not in sResponse["result"]:
                print ("Failed to set Shutter Speed, current value: %s" %sResponse["result"])
                return False

            print ("Shutter Speed set to %s" % sShutterSpeed)
            return True
        
        # In case of an error, return false
        print ("Failed to set Shutter Speed")
        return False

#****************************************************************************
#   Method Name     : boSetAperture
#
#   Description     : Commands the camera to set a lens Apperture
#
#   Parameters      : F number * 10
#                     i.e. F 2.8 = 28
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boSetAperture(self,u8Aperture):
        # Create Aperture String (cast one of the numbers to float to get a float)
        fFvalue = u8Aperture / float(10)
        sFValue = str(fFvalue)

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setFNumber", adictParams=[sFValue])
        
        # Check response for a succesful result
        if 'result' in sResponse:
            time.sleep(0.25)
            sResponse = self.__sSimpleCall("getFNumber")
            
            if sFValue not in sResponse["result"]:
                print ("Failed to set aperture, current value: %s" %sResponse["result"])
                return False
            
            print ("Aperture set to %s" % sFValue)
            return True
            
        # In case of an error, return false
        print ("Failed to set aperture")
        return False

#****************************************************************************
#   Method Name     : boSetISO
#
#   Description     : Commands the camera to set an ISO number
#
#   Parameters      : ISO Value
#                     80, 100, 1000, 3200, etc...
#
#   Return Value    : True if succesful
#                     False if Error Recieved
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boSetISO(self,u16ISO):
        # Create ISO String
        sISO = str(u16ISO)
        
        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setIsoSpeedRate", adictParams=[sISO])
       
        # Check response for a succesful result
        if 'result' in sResponse:
            sResponse = self.__sSimpleCall("getIsoSpeedRate")

            if sISO not in sResponse["result"]:
                print ("Failed to Set ISO, current value: %s" %sResponse["result"])
                return False
            
            print ("ISO set to %s" % sISO)
            return True
        
        # In case of an error, return false
        print ("Failed to Set ISO")
        return False

#****************************************************************************
#   Method Name     : __vAddGeotagToLog
#
#   Description     : Adds an entry to the log file with the name of the image
#                     and geoposition and orientation of the shot.
#
#   Parameters      : Image file name, position, orientation
#
#   Return Value    : True if succesful
#                     False if no URL was recieved for the image
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def __boAddGeotagToLog(self, sImageFileName):
        return True

#****************************************************************************
#   Method Name     : boTakePicture
#
#   Description     : Commands the camera to take a picture
#
#   Parameters      : none
#
#   Return Value    : True if succesful
#                     False if no URL was recieved for the image
#
#   Autor           : Jaime Machuca
#
#****************************************************************************

    def boTakePicture(self):
        # Send command to take picture to camera
        sResponse = self.__sSimpleCall("actTakePicture")

        # Check response for a succesful result and save latest image URL
        if 'result' in sResponse:
            self.sLatestImageURL = sResponse['result'][0][0]
            self.__boAddGeotagToLog(self.sLatestImageURL)
            self.u32ImgCounter = self.u32ImgCounter+1
            return True

        # In case of an error, return false
        return False

#****************************************************************************
#
#   Stuff Needed for testing and compatibility with current code.
#
#****************************************************************************
    def take_picture(self):
        return self.boTakePicture()

    def get_latest_image(self):
        self.boGetLatestImage()

        # this reads the image from the filename, parameter is 1 color, 0 BW, -1 unchanged
        return cv2.imread(self.sLatestImageFilename,1)


    # main - tests SmartCameraWebCam class
    def main(self):

        while True:
            # send request to image capture for image
            if self.take_picture():
                # display image
                cv2.imshow ('image_display', self.get_latest_image())
            else:
                print "no image"
    
            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
    
            # take a rest for a bit
            time.sleep(0.01)

# run test run from the command line
if __name__ == "__main__":
    sc_SonyQX1_0 = SmartCameraSonyQX1(0)
    sc_SonyQX1_0.main()
