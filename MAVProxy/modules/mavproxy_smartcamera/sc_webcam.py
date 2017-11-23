"""
sc_webcam.py

This file includes functions to:
    initialise a web cam
    capture image from web cam

Image size is held in the smart_camera.cnf
"""

import sys
import time
import math
import cv2
import sc_config

class SmartCameraWebCam:

    def __init__(self, instance):

        # health
        self.healthy = False;

        # record instance
        self.instance = instance
        self.config_group = "camera%d" % self.instance

        # get image resolution
        self.img_width = sc_config.config.get_integer(self.config_group,'width',640)
        self.img_height = sc_config.config.get_integer(self.config_group,'height',480)

        # background image processing variables
        self.img_counter = 0        # num images requested so far

        # latest image captured
        self.latest_image = None

        # setup video capture
        self.camera = cv2.VideoCapture(self.instance)

        # check we can connect to camera
        if not self.camera.isOpened():
            print("failed to open webcam %d" % self.instance)

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraWebCam Object W:%d H:%d" % (self.img_width, self.img_height)

    # latest_image - returns latest image captured
    def get_latest_image(self):
        # write to file
        #imgfilename = "C:\Users\rmackay9\Documents\GitHub\ardupilot-balloon-finder\smart_camera\img%d-%d.jpg" % (cam_num,cam.get_image_counter())
        imgfilename = "img%d-%d.jpg" % (self.instance,self.get_image_counter())
        print (imgfilename)
        cv2.imwrite(imgfilename, self.latest_image)
        return self.latest_image

    # get_image_counter - returns number of images captured since startup
    def get_image_counter(self):
        return self.img_counter

    # take_picture - take a picture
    #   returns True on success
    def take_picture(self):
        # setup video capture
        print("Taking Picture")
        self.camera = cv2.VideoCapture(self.instance)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

        # check we can connect to camera
        if not self.camera.isOpened():
            self.healty = False
            return False

        # get an image from the webcam
        success_flag, self.latest_image=self.camera.read()

        # release camera
        self.camera.release()

        # if successful overwrite our latest image
        if success_flag:
            self.img_counter = self.img_counter+1
            return True

        # return failure
        return False

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
    sc_webcam0 = SmartCameraWebCam(0)
    sc_webcam0.main()
