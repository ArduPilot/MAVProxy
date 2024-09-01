#!/usr/bin/env python3

'''
MAV Picture Viewer

Quick and efficient reviewing of images taken from a drone

AP_FLAKE8_CLEAN
'''

from threading import Thread
import cv2
import time
import os
import piexif
import re
import mavpicviewer_shared as mpv
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_elevation

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.wx_loader import wx
    from MAVProxy.modules.lib.mp_menu import MPMenuTop
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_image import MPImage
    from MAVProxy.modules.mavproxy_map import mp_slipmap
    from MAVProxy.modules.lib import camera_projection

prefix_str = "mavpicviewer_image: "


class mavpicviewer_image:
    """handle camera view image"""

    def __init__(self, folderpath, comm_pipe=None):

        # init current filenumber and filelist
        self.filenumber = 0
        self.filelist = {}
        self.open_folder(folderpath)

        # record reference to communication queue
        self.comm_pipe = comm_pipe

        # load elevation data
        self.terrain_source = "SRTM3"
        self.terrain_offline = 0
        self.elevation_model = mp_elevation.ElevationModel(self.terrain_source)

        # POIs indexed by filenumber
        self.poi_dict = {}
        self.poi_start_pos = None

        # init image exif data
        self.lat = None
        self.lon = None
        self.alt_amsl = None
        self.terr_alt = None
        self.temp_max = None
        self.temp_max_pos = mpv.Pos(None, None)
        self.temp_min = None
        self.temp_min_pos = mpv.Pos(None, None)
        self.temp_highlight_size = 20
        self.temp_highlight_active = True

        # create menu (this is added to im in update_image)
        self.menu = None
        if mp_util.has_wxpython:
            self.menu = MPMenuTop([MPMenuSubMenu('&File',
                                   items=[MPMenuItem('&Save\tCtrl+S'),
                                          MPMenuItem('&Clear POI\tCtrl+C', returnkey='clearpoi'),
                                          MPMenuItem('&Next Image\tCtrl+N', returnkey='nextimage'),
                                          MPMenuItem('&Prev Image\tCtrl+P', returnkey='previmage'),
                                          MPMenuItem('&Temp Highlight\tCtrl+T', returnkey='temphighlight'),
                                          MPMenuItem('&Quit\tCtrl+Q', 'Quit', returnkey='quit')])])

        # create image viewer
        self.im = MPImage(title="image viewer",
                          mouse_events=True,
                          mouse_movement_events=True,
                          key_events=True,
                          can_drag=False,
                          can_zoom=True,
                          auto_size=False,
                          auto_fit=True)
        if self.im is not None:
            self.im.set_menu(self.menu)

        # check if image viewer was created
        if self.im is None:
            print(prefix_str + "failed to create image viewer")
            return
        self.update_image()

        # set camera parameters with default FOV
        self.update_camera_settings(25)

        # hard-code mount angles
        self.roll = 0
        self.pitch = -90
        self.yaw = 0

        # initialise map
        self.sm = self.sm = mp_slipmap.MPSlipMap(elevation=self.terrain_source)
        self.update_map()

        # exit variable
        self.exit_requested = False

        self.thread = Thread(target=self.mavpicviewer_image_loop, name='mavpicviewer_image_loop')
        self.thread.daemon = False
        self.thread.start()

    # main loop
    def mavpicviewer_image_loop(self):
        """main thread"""
        while not self.exit_requested:
            if self.im is None:
                break
            time.sleep(0.25)
            self.check_events()

            # handle messages from the comm pipe
            if self.comm_pipe is not None:
                # print contents of the queue
                while self.comm_pipe.poll():
                    self.handle_comm_object(self.comm_pipe.recv())

    # handle an object received on the communication pipe
    def handle_comm_object(self, obj):
        """process an object received on the communication pipe"""
        if isinstance(obj, mpv.SetFilelist):
            self.filelist = obj.filelist
            self.poi_clear_all()
            self.update_image()
            self.update_map()
        elif isinstance(obj, mpv.SetFilenumber):
            self.set_filenumber(obj.filenumber, False)
        elif isinstance(obj, mpv.SetFOV):
            self.set_fov(obj.fov)
        elif isinstance(obj, mpv.SetYaw):
            self.set_yaw(obj.yaw)
        elif isinstance(obj, mpv.ClearAllPOI):
            self.poi_clear_all()
        elif isinstance(obj, mpv.GetTempForAllImages):
            self.send_all_temperatures()
        elif isinstance(obj, mpv.Close):
            self.close(False)

    # send a communication object to the image viewer
    def send_comm_object(self, obj):
        """send a communication object to the image viewer"""
        if self.comm_pipe is not None:
            self.comm_pipe.send(obj)

    # open folder
    def open_folder(self, folderpath):
        """open folder"""
        filelist = mpv.get_file_list(folderpath, ['jpg', 'jpeg'])
        if filelist is None or not filelist:
            filelist = {}
            print(prefix_str + "no files found")
        self.set_file_list(filelist)

    # update filelist dictionary which holds the filepaths for all images
    def set_file_list(self, filelist):
        # sanity check filelist
        if filelist is None:
            print(prefix_str + "set_file_list: invalid filelist")
            return
        self.filelist = filelist

    # set window title
    def set_title(self, title):
        """set image title"""
        if self.im is None:
            return
        self.im.set_title(title)

    # set FOV
    def set_fov(self, fov_deg):
        """set image FOV"""
        self.update_camera_settings(fov_deg)
        self.update_map()

    # set Yaw
    def set_yaw(self, yaw_deg):
        """set image yaw"""
        self.yaw = yaw_deg
        self.update_map()

    # process window events
    def check_events(self):
        """check for image events"""
        if self.im is None:
            return
        if not self.im.is_alive():
            self.im = None
            return
        for event in self.im.events():
            if isinstance(event, MPMenuItem):
                if event.returnkey == "fitWindow":
                    self.im.fit_to_window()
                elif event.returnkey == "fullSize":
                    self.im.full_size()
                elif event.returnkey == "clearpoi":
                    self.cmd_clear_poi()
                elif event.returnkey == "nextimage":
                    self.cmd_nextimage()
                elif event.returnkey == "previmage":
                    self.cmd_previmage()
                elif event.returnkey == "temphighlight":
                    self.temp_highlight_active = not self.temp_highlight_active
                    self.update_image()
                elif event.returnkey == "quit":
                    self.close()
                continue
            if event.ClassName == "wxMouseEvent":
                if event.X is not None and event.Y is not None:
                    if event.leftIsDown:
                        self.poi_capture_start(event.X, event.Y)
                    else:
                        self.poi_capture_done(event.X, event.Y)
                else:
                    # if no X,Y coordinates then probably outside of window
                    self.poi_cancel()
            if event.ClassName == "wxKeyEvent":
                keycode = event.KeyCode
                if keycode == wx.WXK_LEFT:
                    self.cmd_previmage()
                elif keycode == wx.WXK_RIGHT:
                    self.cmd_nextimage()

    # start capturing POI rectangle around part of image
    def poi_capture_start(self, X, Y):
        """handle user request to start capturing box around part of image"""
        if self.poi_start_pos is None:
            self.poi_start_pos = mpv.Pos(X, Y)

    # complete capturing box around part of image
    def poi_capture_done(self, X, Y):
        """handle user request to complete capturing box around part of image"""
        if self.poi_start_pos is not None:
            # exit if mouse has not moved a sufficient distance
            if abs(X - self.poi_start_pos.X) <= 5 or abs(Y - self.poi_start_pos.Y) <= 5:
                self.poi_start_pos = None
                return
            # calculate location of each corner
            lat1, lon1, alt1 = self.get_latlonalt(self.poi_start_pos.X, self.poi_start_pos.Y)
            lat2, lon2, alt2 = self.get_latlonalt(X, Y)
            if lat1 is None or lat2 is None:
                print(prefix_str + "POI capture failed")
                return
            # add POI object to dictionary
            poi = mpv.POI(self.poi_start_pos, mpv.Pos(X, Y),
                          mpv.Loc(lat1, lon1, alt1),
                          mpv.Loc(lat2, lon2, alt2))
            self.poi_dict[self.filenumber] = poi
            self.send_comm_object(mpv.SetPOI(self.filenumber, poi))
            self.poi_start_pos = None
            # update image
            self.update_image()
            # add retangle to map
            self.add_rectangle_to_map(self.filename, lat1, lon1, lat2, lon2)

    # camcel capturing box around part of image.  should be called if mouse leaves window, next image is loaded, etc
    def poi_cancel(self):
        self.poi_start_pos = None

    # clear all POI points from dictionary and map and current image
    def poi_clear_all(self):
        self.poi_cancel()
        update_image_required = False
        for filenumber in self.poi_dict.keys():
            filename = self.filelist[filenumber]
            if filename is not None:
                self.remove_rectangle_from_map(filename)
            if self.filenumber == filenumber:
                update_image_required = True
        self.poi_dict.clear()
        if update_image_required:
            self.update_image()

    # clear poi from current image
    def cmd_clear_poi(self):
        self.poi_cancel()
        if self.filenumber in self.poi_dict.keys():
            self.poi_dict.pop(self.filenumber)
            self.update_image()
            self.remove_rectangle_from_map(self.filename)
            self.send_comm_object(mpv.ClearPOI(self.filenumber))

    # update current image to next image
    def cmd_nextimage(self):
        self.set_filenumber(self.filenumber + 1)

    # update current image to previous image
    def cmd_previmage(self):
        self.set_filenumber(self.filenumber - 1)

    # close image viewer
    def close(self, notify_mosaic=True):
        self.exit_requested = True
        if self.im is not None:
            self.im.terminate()
        if self.sm is not None:
            self.sm.close()
        if notify_mosaic:
            self.send_comm_object(mpv.Close())

    # image select callback
    def set_filenumber(self, filenumber, notify_mosaic=True):
        if filenumber < 0:
            print(prefix_str + "already at first image")
            return
        if filenumber >= len(self.filelist):
            print(prefix_str + "already at last image %d" % self.filenumber)
            return
        self.filenumber = filenumber
        self.update_image()
        self.update_map()
        if notify_mosaic:
            self.send_comm_object(mpv.SetFilenumber(self.filenumber))

    # update the displayed image
    # should be called if filenumber is changed
    def update_image(self):
        # sanity check filelist
        if self.filelist is None or len(self.filelist) == 0:
            print(prefix_str + "no files found")
            return

        # check filenumber is within range
        if self.filenumber < 0 or self.filenumber >= len(self.filelist):
            print(prefix_str + "invalid filenumber %d" % self.filenumber)
            return

        # update filename
        self.filename = self.filelist[self.filenumber]
        base_filename = os.path.basename(self.filename)

        # check file exists
        if not os.path.exists(self.filename):
            print(prefix_str + "%s not found" % self.filename)
            return

        # check image viewer has been created
        if self.im is None:
            print(prefix_str + "image viewer not created")
            return

        # set title to filename
        title_str = base_filename + " (" + str(self.filenumber+1) + " of " + str(len(self.filelist)) + ")"
        self.set_title(title_str)

        # load image from file
        image = cv2.imread(self.filename)

        # check image loaded
        if image is None:
            print(prefix_str + "failed to load image %s" % self.filename)
            return

        # load exif data
        self.lat, self.lon, self.alt_amsl, self.terr_alt, \
            self.temp_max, self.temp_max_pos.X, self.temp_max_pos.Y, \
            self.temp_min, self.temp_min_pos.X, self.temp_min_pos.Y = self.get_exif_loc_and_temp(self.filename)

        # use pixel darkness to find temp max position.  This seems to produce a better result than the built-in exif data
        # use exif min/max temp (if available) to calculate the range
        if True:
            tmax_ovr = self.temp_max
            tmin_ovr = self.temp_min
        else:
            tmax_ovr = None
            tmin_ovr = None
        self.temp_max, self.temp_max_pos.X, self.temp_max_pos.Y = self.get_temp_from_pixel_darkness(image, tmax_ovr, tmin_ovr)

        # add POI rectangles to image
        if self.filenumber in self.poi_dict.keys():
            poi = self.poi_dict.get(self.filenumber)
            cv2.rectangle(image, (poi.pos1.X, poi.pos1.Y), (poi.pos2.X, poi.pos2.Y), mpv.RGB_RED, 2)

        # add temperature rectangle to image
        if self.temp_highlight_active:
            if self.temp_max is not None and self.temp_max_pos.X is not None and self.temp_max_pos.Y is not None:
                top_left = (self.temp_max_pos.X-self.temp_highlight_size, self.temp_max_pos.Y-self.temp_highlight_size)
                bottom_right = (self.temp_max_pos.X+self.temp_highlight_size, self.temp_max_pos.Y+self.temp_highlight_size)
                cv2.rectangle(image, top_left, bottom_right, mpv.RGB_YELLOW, 2)

        # update image and colormap
        self.im.set_image(image)
        self.im.set_colormap("None")

        # send exif data to mosaic
        self.send_comm_object(mpv.SetImageLoc(self.filenumber, mpv.Loc(self.lat, self.lon, self.alt_amsl, self.terr_alt)))
        self.send_comm_object(mpv.SetTempAndPos(self.filenumber, self.temp_max, self.temp_max_pos.X, self.temp_max_pos.Y))

    # update camera settings
    def update_camera_settings(self, FOV):
        if self.im is None:
            print(prefix_str + "failed to update camera settings because image is not set")
            return
        # set camera parameters (hard-coded for Siyi ZT6)
        self.cam1_params = camera_projection.CameraParams(xresolution=640, yresolution=512, FOV=FOV)
        self.cam1_projection = camera_projection.CameraProjection(self.cam1_params, self.elevation_model, self.terrain_source)

    # update the displayed map with polygon
    # should be called if filenumber is changed
    def update_map(self):
        # check image location
        if self.lat is None or self.lon is None or self.alt_amsl is None:
            print(prefix_str + "failed to update map because lat, lon or alt not set")
            return

        # check image attitude
        if self.roll is None or self.pitch is None or self.yaw is None:
            print(prefix_str + "failed to update map because roll, pitch or yaw not set")
            return

        # create and display map
        if self.sm is None:
            self.sm = mp_slipmap.MPSlipMap(lat=self.lat, lon=self.lon, elevation=self.terrain_source)
        if self.sm is None:
            print(prefix_str + "failed to create map")
            return

        # update map center
        if self.sm is not None:
            self.sm.set_center(self.lat, self.lon)

        # calculate image outline polygon for map
        try:
            projection1 = self.cam1_projection.get_projection(self.lat, self.lon, self.alt_amsl,
                                                              self.roll, self.pitch, self.yaw)
            if projection1 is not None and self.sm is not None:
                self.sm.add_object(mp_slipmap.SlipPolygon('projection1', projection1, layer=1,
                                                          linewidth=2, colour=mpv.RGB_GREEN))
            else:
                print(prefix_str + "failed to add projection to map")
        except ArithmeticError as e:
            print(prefix_str + "failed to add projection to map due to arithmetic error: " + str(e))

    # add a rectangle specified by two locations to the map
    def add_rectangle_to_map(self, name, lat1, lon1, lat2, lon2):
        if self.sm is not None:
            rect = [(lat1, lon1), (lat1, lon2), (lat2, lon2), (lat2, lon1), (lat1, lon1)]
            self.sm.add_object(mp_slipmap.SlipPolygon(name, rect, layer=1, linewidth=2, colour=mpv.RGB_RED))

    # remove a rectangle from the map
    def remove_rectangle_from_map(self, name):
        if self.sm is not None:
            self.sm.remove_object(name)

    # get location (e.g lat, lon, alt, terr_alt) and temperature (max, pixe pos X, pixel pos Y) from image's exif tags
    def get_exif_loc_and_temp(self, filename):
        """get latitude, longitude, altitude and terrain_alt from exif tags"""

        exif_dict = piexif.load(filename)

        if piexif.GPSIFD.GPSLatitudeRef in exif_dict["GPS"]:
            lat_ns = exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef]
            lat = self.dms_to_decimal(exif_dict["GPS"][piexif.GPSIFD.GPSLatitude][0],
                                      exif_dict["GPS"][piexif.GPSIFD.GPSLatitude][1],
                                      exif_dict["GPS"][piexif.GPSIFD.GPSLatitude][2],
                                      lat_ns)
            lon_ew = exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef]
            lon = self.dms_to_decimal(exif_dict["GPS"][piexif.GPSIFD.GPSLongitude][0],
                                      exif_dict["GPS"][piexif.GPSIFD.GPSLongitude][1],
                                      exif_dict["GPS"][piexif.GPSIFD.GPSLongitude][2],
                                      lon_ew)
            alt = float(exif_dict["GPS"][piexif.GPSIFD.GPSAltitude][0])/float(exif_dict["GPS"][piexif.GPSIFD.GPSAltitude][1])
            terr_alt = self.elevation_model.GetElevation(lat, lon)
            if terr_alt is None:
                print("WARNING: failed terrain lookup for %f %f" % (lat, lon))
                terr_alt = 0
        else:
            lat = 0
            lon = 0
            alt = 0
            terr_alt = 0

        # get comment
        temp_max = None
        temp_max_x = None
        temp_max_y = None
        temp_min = None
        temp_min_x = None
        temp_min_y = None
        if piexif.ExifIFD.UserComment in exif_dict["Exif"]:
            comment_bytes = exif_dict["Exif"][piexif.ExifIFD.UserComment]
            comment_str = comment_bytes.decode("utf-8")
            temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y = self.get_temp_from_comment(comment_str)

        return lat, lon, alt, terr_alt, temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y

    # collect and send all temperatures to mosaic
    def send_all_temperatures(self):
        for filenumber in range(len(self.filelist)):
            filename = self.filelist[filenumber]
            if filename is not None:
                # load exif data
                lat, lon, alt_amsl, terr_alt, \
                    temp_max, temp_max_pos_X, temp_max_pos_Y, \
                    temp_min, temp_min_pos_X, temp_min_pos_Y = self.get_exif_loc_and_temp(filename)
                # send exif data to mosaic
                self.send_comm_object(mpv.SetTempAndPos(filenumber, temp_max, temp_max_pos_X, temp_max_pos_Y))

    def dms_to_decimal(self, degrees, minutes, seconds, sign=b' '):
        """Convert degrees, minutes, seconds into decimal degrees.

        >>> dms_to_decimal((10, 1), (10, 1), (10, 1))
        10.169444444444444
        >>> dms_to_decimal((8, 1), (9, 1), (10, 1), 'S')
        -8.152777777777779
        """
        return (-1 if sign in b'SWsw' else 1) * (
            float(degrees[0])/float(degrees[1])        +
            float(minutes[0])/float(minutes[1]) / 60.0   +
            float(seconds[0])/float(seconds[1]) / 3600.0
        )

    def get_latlonalt(self, pixel_x, pixel_y):
        '''
        get ground lat/lon given vehicle orientation, camera orientation and slant range
        pixel_x and pixel_y are in image pixel coordinates with 0,0 at the top left
        '''
        if self.cam1_params is None:
            print("picviewer: failed to calc lat,lon because camera params not set")
            return None

        return self.cam1_projection.get_latlonalt_for_pixel(int(pixel_x), int(pixel_y),
                                                            self.lat, self.lon, self.alt_amsl,
                                                            self.roll, self.pitch, self.yaw)

    # get temperature from a comment string
    # on success returns three values, temp max, X and Y pixel coordinates
    # on failure all values are None
    def get_temp_from_comment(self, comment_str):
        """
        Extracts the maximum temperature and the corresponding pixel from the given comment string.

        Parameters:
        comment (str): A string containing temperature and pixel data like "max:26.85(76,304);min:21.85(422,187)"

        Returns:
        tuple: A tuple containing the max temperature (float) and the pixel coordinates (tuple of integers).
        """

        # initialise values
        temp_max = None
        temp_max_x = None
        temp_max_y = None
        temp_min = None
        temp_min_x = None
        temp_min_y = None

        # Use regular expression to extract max temperature and the pixel coordinates
        max_temp_pattern = r"max:([\d\.]+)\((\d+),(\d+)\)"
        match = re.search(max_temp_pattern, comment_str)
        if match:
            # Extract the max temperature and pixel coordinates
            temp_max = float(match.group(1))
            temp_max_x = int(match.group(2))
            temp_max_y = int(match.group(3))

        # extract min temperature and pixel coordinates
        min_temp_pattern = r"min:([\d\.]+)\((\d+),(\d+)\)?"
        match = re.search(min_temp_pattern, comment_str)
        if match:
            # Extract the max temperature and pixel coordinates
            temp_min = float(match.group(1))
            temp_min_x = int(match.group(2))
            temp_min_y = int(match.group(3))

        return temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y

    # get temperature from an image's pixel darkness (assume black is hot)
    # on success returns three values, temp max, X and Y pixel coordinates
    # on failure all values are None
    def get_temp_from_pixel_darkness(self, cv2_image, temp_max_override=None, temp_min_override=None):

        # print image's darkest and brightest pixel values using minMaxLoc
        grey_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(grey_image)

        # convert pixel brightness to a temperature in the range 0C to 100C
        # To-Do: expose these to the user as settings
        # Siyi ZT6 should be -20 ~ +150
        # Xacti CX-GB250 should be -50 to +150
        black_temp = 100
        white_temp = 0
        if temp_max_override is not None:
            black_temp = temp_max_override
        if temp_min_override is not None:
            white_temp = temp_min_override
        temp_max = self.linear_interpolate(black_temp, white_temp, minVal, 0, 255)

        # convert min value into a temperature
        return temp_max, minLoc[0], minLoc[1]

    # linear interpolate function
    def linear_interpolate(self, low_output, high_output, var_value, var_low, var_high):

        # support either polarity
        if var_low > var_high:
            temp = var_low
            var_low = var_high
            var_high = temp
            temp = low_output
            low_output = high_output
            high_output = temp

        if var_value <= var_low:
            return low_output

        if var_value >= var_high:
            return high_output

        p = (var_value - var_low) / (var_high - var_low)
        return low_output + p * (high_output - low_output)
