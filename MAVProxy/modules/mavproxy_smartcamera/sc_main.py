import time
import math
import cv2
from pymavlink import mavutil
from droneapi.lib import VehicleMode, Location
import sc_config
from sc_video import sc_video
from sc_webcam import SmartCameraWebCam
from sc_SonyQX1 import SmartCamera_SonyQX

"""
sc_main.py - runs top level smart camera function

To run this module:
* Run mavproxy.py with the correct options to connect to your vehicle
* module load api
* api start sc_main.py

(Once tested we can put these directives into a mavinit.scr file and mavproxy will load/run
    this code automatically)

"""

class SmartCamera(object):
    def __init__(self, use_api):

        # if using droneapi connect to vehicle
        if (use_api):
            # First get an instance of the API endpoint (the connect via web case will be similar)
            self.api = local_connect()

            # Our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
            self.vehicle = self.api.get_vehicles()[0]
        else:
            self.api = None
            self.vehicle = None

        # initialised flag
        self.home_initialised = False
        # timer to intermittently check for home position
        self.last_home_check = time.time()
        self.home_location = None

        # vehicle mission
        self.mission_cmds = None

        # check if we should display debug messages
        self.debug = sc_config.config.get_boolean('general','debug',True)

        # register cameras
        self.register_cameras();

        # initialise video writer
        self.writer = None

    # register cameras - creates camera objects based on camera-type configuration
    def register_cameras(self):

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

    # fetch_mission - fetch mission from flight controller
    def fetch_mission(self):
        # download the vehicle waypoints
        self.mission_cmds = self.vehicle.commands
        self.mission_cmds.download()
        self.mission_cmds.wait_valid()

    # check home - intermittently checks for changes to the home location
    def check_home(self):

        # return immediately if home has already been initialised
        if self.home_initialised:
            return True

        # check for home no more than once every two seconds
        if (time.time() - self.last_home_check > 2):

            # update that we have performed a status check
            self.last_home_check = time.time()

            # check if we have a vehicle
            if self.vehicle is None:
                self.vehicle = self.api.get_vehicles()[0]
                return

            # ensure the vehicle's position is known
            if self.vehicle.location is None:
                return False
            if self.vehicle.location.lat is None or self.vehicle.location.lon is None or self.vehicle.location.alt is None:
                return False

            # download the vehicle waypoints if we don't have them already
            if self.mission_cmds is None:
                self.fetch_mission()
                return False

            # get the home lat and lon
            home_lat = self.mission_cmds[0].x
            home_lon = self.mission_cmds[0].y
            home_alt = self.mission_cmds[0].z

            # sanity check the home position
            if home_lat is None or home_lon is None or home_alt is None:
                return False

            # sanity check again and set home position
            if (home_lat != 0 and home_lon != 0):
                self.home_location = Location(home_lat,home_lon,home_alt)
                self.home_initialised = True
            else:
                self.mission_cmds = None

            # To-Do: if we wish to have the same home position as the flight controller
            # we must download the home waypoint again whenever the vehicle is armed

        # return whether home has been initialised or not
        return self.home_initialised

    # checks if video output should be started
    def check_video_out(self):

        # return immediately if video has already been started
        if not self.writer is None:
            return

        # start video once vehicle is armed
        if self.vehicle.armed:
            self.writer = sc_video.open_video_writer()

    # check_status - poles vehicle' status to determine if we should record video or not
    def check_status(self):

        # download the vehicle waypoints if we don't have them already
        # To-Do: do not load waypoints if vehicle is armed
        if self.mission_cmds is None:
            self.fetch_mission()
            return

    # take_picture_all - ask all cameras to take a picture
    def take_picture_all(self):
        for cam in self.camera_list:
            cam.take_picture()

    # saves_picture_all - ask all cameras for their latest image and saves to files
    def save_picture_all(self):
        cam_num = 0
        for cam in self.camera_list:
            img = cam.get_latest_image()
            # display image
            window_name = "cam%d" % cam_num
            cv2.namedWindow(window_name, 0)
            cv2.resizeWindow(window_name, 640, 480)
            cv2.imshow (window_name, img)
            # write to file
            #imgfilename = "C:\Users\rmackay9\Documents\GitHub\ardupilot-balloon-finder\smart_camera\img%d-%d.jpg" % (cam_num,cam.get_image_counter())
            #imgfilename = "img%d-%d.jpg" % (cam_num,cam.get_image_counter())
            #print (imgfilename)
            #cv2.imwrite(imgfilename, img)

            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            cam_num = cam_num + 1

    # get image from sc_video class and write to file
    def analyze_image(self):

        # record time
        now = time.time()

        # get new image from camera
        f = self.get_frame()

        # save image for debugging later
        if not self.writer is None:
            self.writer.write(f)

    def run(self):
        while True:
            # ask all cameras to take a picture
            self.take_picture_all()

            # store images to disk
            self.save_picture_all()

            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(1)

        '''
        while not self.api.exit:

            # only process images once home has been initialised
            if self.check_home():

                # start video if required
                self.check_video_out()

                # check if we are controlling the vehicle
                self.check_status()

                # ask all cameras to take a picture
                self.take_picture_all()

            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(2.0)

        if not self.use_simulator:
            sc_video.stop_background_capture()
        '''

# initialise depending upon whether running from command line or as part of mavproxy
if __name__ == "__main__":
    sc_main = SmartCamera(False)
else:
    sc_main = SmartCamera(True)

# run the smart camera
sc_main.run()

