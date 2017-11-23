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
# License      : GNU GPL version 3
#
# Editor Used  : Xcode 6.1.1 (6A2008a)
#
#****************************************************************************

#****************************************************************************
# HEADER-FILES (Only those that are needed in this file)
#****************************************************************************

# System Header files and Module Headers
import os, sys, time, math, cv2, struct, fcntl
from datetime import datetime

# Module Dependent Headers
import requests, json, socket, StringIO
import xml.etree.ElementTree as ET
import urllib
from sc_ExifWriter import ExifWriter

# Own Headers
import ssdp

#****************************************************************************
# Constants
#****************************************************************************

# Target Initial Camera Values
targetShutterSpeed = 1600
targetAperture = 120
targetISOValue = "AUTO"

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
#   Author           : Jaime Machuca, Randy Mackay
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
        self.sLatestImageFilename = None    #String with the Filename for the last downloaded image
        self.sLatestFileName = None         #String with the camera file name for the last image taken

        self.vehicleLat = 0.0              # Current Vehicle Latitude
        self.vehicleLon = 0.0              # Current Vehicle Longitude
        self.vehicleHdg = 0.0              # Current Vehicle Heading
        self.vehicleAMSL = 0.0             # Current Vehicle Altitude above mean sea level

        self.vehicleRoll = 0.0              # Current Vehicle Roll
        self.vehiclePitch = 0.0              # Current Vehicle Pitch


        # Look Camera and Get URL
        self.sCameraURL = self.__sFindCameraURL(sNetInterface)
        if self.sCameraURL is None:
            print("No QX camera found, failed to open QX camera %d" % self.u8Instance)
        else:
            self.__openGeoTagLogFile()      # open geoTag Log
            self.boCameraInitialSetup()     # Setup Initial camera parameters

#****************************************************************************
#   Method Name     : __str__
#
#   Description     : Returns a human readable string name for the instance
#
#   Parameters      : none
#
#   Return Value    : String with object instance name
#
#   Author           : Randy Mackay
#
#****************************************************************************

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraSonyQX Object for %s" % self.sConfigGroup

#****************************************************************************
#   Method Name     : boCameraInitialSetup
#
#   Description     : Sets Initial Camera Parameters
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boCameraInitialSetup(self):
        print("Setting up Camera Initial Parameters")
        # Check if we need to do 'startRecMode'
        APIList = self.__sSimpleCall("getAvailableApiList")

        # For those cameras which need it
        if 'startRecMode' in (APIList['result'])[0]:
            print("Need to send startRecMode, sending and waiting 5 sec...")
            self.__sSimpleCall("startRecMode")
            time.sleep(1)
            print("4 sec")
            time.sleep(1)
            print("3 sec")
            time.sleep(1)
            print("2 sec")
            time.sleep(1)
            print("1 sec")
            time.sleep(1)

        # Set Postview Size to Orignial size to get real image filename
        sResponse = self.__sSimpleCall("getSupportedPostviewImageSize")
        print("%s" % sResponse)

        sResponse = self.__sSimpleCall("setPostviewImageSize", adictParams=["Original"])
        print("%s" % sResponse)

        sResponse = self.__sSimpleCall("getPostviewImageSize")
        print("%s" % sResponse)

        # Set Mode to Shutter Priority if available
        SupportedModes = self.__sSimpleCall("getSupportedExposureMode")
        if 'Shutter' in (SupportedModes['result'])[0]:
            self.boSetExposureMode("Shutter")
        #elif 'Manual' in (SupportedModes['result'])[0]:
        #    self.boSetExposureMode("Manual")
        else:
            print("Error no Shutter Priority Mode")

        # Set Target Shutter Speed
        self.boSetShutterSpeed(targetShutterSpeed)

        # Set Target ISO Value
        self.boSetISO(targetISOValue)

        #Take Confirmation Picture
        self.boTakePicture()

#****************************************************************************
#   Method Name     : boSet_GPS
#
#   Description     : Gets the GPS Position from the provided message
#
#   Parameters      : mGPSMessage       GPS Mavlink Message type
#
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSet_GPS(self, mGPSMessage):
        if mGPSMessage.get_type() == 'GLOBAL_POSITION_INT':
            (self.vehicleLat, self.vehicleLon, self.vehicleHdg, self.vehicleAMSL) = (mGPSMessage.lat*1.0e-7, mGPSMessage.lon*1.0e-7, mGPSMessage.hdg*0.01, mGPSMessage.alt*0.001)

#****************************************************************************
#   Method Name     : boSet_Attitude
#
#   Description     : Gets the vehicle attitude from the provided message
#
#   Parameters      : mAttitudeMessage  MAVlink Attitude Message type
#
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSet_Attitude(self, mAttitudeMessage):
        if mAttitudeMessage.get_type() == 'ATTITUDE':
            (self.vehicleRoll, self.vehiclePitch) = (math.degrees(mAttitudeMessage.roll), math.degrees(mAttitudeMessage.pitch))

#****************************************************************************
#   Method Name     : __geoRef_write
#
#   Description     : Writes GeoReference to file
#
#   Parameters      : sImageFileName    File name of image to be entered into the log
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************


    # Geo reference log for all the GoPro pictures
    def __geoRef_write(self, sImageFileName):
        #self.geoRef_writer.write(datetime.now().strftime('%d-%m-%Y %H:%M:%S.%f')[:-3])
        self.geoRef_writer.write(sImageFileName)
        self.geoRef_writer.write(",%f,%f,%f,%f,%f,%f" % (self.vehicleLat, self.vehicleLon, self.vehicleAMSL, self.vehicleRoll, self.vehiclePitch,self.vehicleHdg))
        self.geoRef_writer.write('\n')
        self.geoRef_writer.flush()

#****************************************************************************
#   Method Name     : __boWriteURLToLog
#
#   Description     : Writes images URL to file
#
#   Parameters      : sImageFileName    File name of image to be entered into the log
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    # Geo reference log for all the GoPro pictures
    def __boWriteURLToLog(self, sURLName):
        self.urlLog_writer.write(sURLName)
        self.urlLog_writer.write('\n')
        self.urlLog_writer.flush()
    
#****************************************************************************
#   Method Name     : get_real_Yaw
#
#   Description     : Helper method to get the real Yaw
#
#   Parameters      : yaw        Vehicle Yaw
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def get_real_Yaw(self, yaw):
        if (yaw < 0):
            return yaw+360
        return yaw

#****************************************************************************
#   Method Name     : __writeGeoRefToFile
#
#   Description     : Writes the Georeference of the image to the log. NOT SURE
#                     IF IT IS DUPLICATED FOR A GOOD REASON.
#
#   Parameters      : sImageFileName1   Image file name
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __writeGeoRefToFile(self, sImageFileName1):
        self.__geoRef_write(sImageFileName1)

#****************************************************************************
#   Method Name     : __openGeoTagLogFile
#
#   Description     : Checks for existing log files and creates a new Log file
#                     with an incremented index number
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __openGeoTagLogFile(self):
        
        # Verify folder exists
        if not os.path.exists('/sdcard/log'):
            os.makedirs('/sdcard/log')
 
        #Open GeoTag Log File
        i = 0
        while os.path.exists('/sdcard/log/geoRef%s.log' % i):
            #print('checking /sdcard/log/geoRef%s.log' % i)
            i += 1
        
        self.sCurrentGeoRefFilename = '/sdcard/log/geoRef%s.log' % i
        self.geoRef_writer = open('/sdcard/log/geoRef%s.log' % i, 'w', 0)
        self.geoRef_writer.write('Filename, Latitude, Longitude, Alt (AMSL), Roll, Pitch, Yaw\n')

        print('Opened GeoTag Log File with Filename: geoRef%s.log' % i)
        
        #Open URL Log File
        i = 0
        while os.path.exists('/sdcard/log/urlLog%s.log' % i):
            #print('checking /sdcard/log/urlLog%s.log' % i)
            i += 1
        
        self.sCurrentURLLogFilename = '/sdcard/log/urlLog%s.log' % i
        self.urlLog_writer = open('/sdcard/log/urlLog%s.log' % i, 'w', 0)

        print('Opened URL Log File with Filename: urlLog%s.log' % i)

#****************************************************************************
#   Method Name     : __sFindInterfaceIPAddress
#
#   Description     : Gets the IP Address of the interface name requested
#
#   Parameters      : sInterfaceName
#
#   Return Value    : String with the IP Address for the requested interface
#
#   Author           : Jaime Machuca,
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
#   Author           : Andrew Tridgell, Jaime Machuca
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
#   Author           : Andrew Tridgell, Jaime Machuca
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
#   Author           : Andrew Tridgell, Jaime Machuca
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
#   Author           : Jaime Machuca
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
#   Description     : Downloads the latest image taken by the camera and then
#                     saves it to a file name composed by the camera instance
#                     and image number.
#
#   Parameters      : none
#
#   Return Value    : True if it was successful
#                     False if no image was downloaded
#
#   Author           : Jaime Machuca
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
#   Method Name     : boGetAllSessionPictures
#
#   Description     : Downloads all the images stored in the URL log file to
#                     the companion computer.
#
#   Parameters      : none
#
#   Return Value    : True if it was successful
#                     False if no image was downloaded
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boGetAllSessionPictures(self, sLogFile):
        
        print("Picture Download started")
        file = open(self.sCurrentURLLogFilename, "r")
        

        for url in file:
            url = url.rstrip('\n')
            #now download link
            start = self.sLatestImageURL.find('DSC')
            end = self.sLatestImageURL.find('JPG', start) + 3
            filename = url[start:end]
            print("Downloading %s" % filename)

            geotagFile = open(self.sCurrentGeoRefFilename, "r")
            for line in geotagFile:
                if filename in line:
                    currFileLatitude = float(line.split(',')[1])
                    currFileLongitude = float(line.split(',')[2])
                    currFileAltitude = float(line.split(',')[3])
                    print ("%s,%f,%f,%f" % (filename, currFileLatitude, currFileLongitude, currFileAltitude))
            geotagFile.close()
            
            try:
                req = requests.request('GET', url)
                open('/sdcard/log/%s' % filename, 'w').write(req.content)
                ExifWriter.write_gps('/sdcard/log/%s' % filename, currFileLatitude, currFileLongitude, currFileAltitude)
            except Exception as ex:
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                print(message)
            
        file.close()

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
#   Author           : Jaime Machuca
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
#   Author           : Jaime Machuca
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boZoomIn(self):

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("actZoom", adictParams=["in","1shot"])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boZoomOut(self):

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("actZoom", adictParams=["out","1shot"])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSetExposureMode(self,sExposureMode):
        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setExposureMode", adictParams=[sExposureMode])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSetShutterSpeed(self,u16ShutterSpeed):
        # Create Shutter Speed String
        sShutterSpeed = "1/%s" % str(u16ShutterSpeed)

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setShutterSpeed", adictParams=[sShutterSpeed])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSetAperture(self,u8Aperture):
        # Create Aperture String (cast one of the numbers to float to get a float)
        fFvalue = u8Aperture / float(10)
        sFValue = str(fFvalue)

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setFNumber", adictParams=[sFValue])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if Error Received
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boSetISO(self,u16ISO):
        # Create ISO String
        sISO = str(u16ISO)

        # Send command to set Exposure Mode
        sResponse = self.__sSimpleCall("setIsoSpeedRate", adictParams=[sISO])

        # Check response for a successful result
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
#   Return Value    : True if successful
#                     False if no URL was received for the image
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __boAddGeotagToLog(self, sImageFileName):
        self.__writeGeoRefToFile(sImageFileName)
        return True

#****************************************************************************
#   Method Name     : boTakePicture
#
#   Description     : Commands the camera to take a picture
#
#   Parameters      : none
#
#   Return Value    : True if successful
#                     False if no URL was received for the image
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def boTakePicture(self):
        # Send command to take picture to camera
        sResponse = self.__sSimpleCall("actTakePicture")

        # Check response for a successful result and save latest image URL
        if 'result' in sResponse:
            self.sLatestImageURL = sResponse['result'][0][0]

            start = self.sLatestImageURL.find('DSC')
            end = self.sLatestImageURL.find('JPG', start) + 3
            self.sLatestFileName = self.sLatestImageURL[start:end]
            print("image URL: %s" % self.sLatestImageURL)
            print("image Name: %s" % self.sLatestFileName)
            self.__boAddGeotagToLog(self.sLatestFileName)
            self.__boWriteURLToLog(self.sLatestImageURL)

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
                print("no image")

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
