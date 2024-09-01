#!/usr/bin/env python3

'''
MAV Picture Viewer

Mosaic window showing all images

AP_FLAKE8_CLEAN
'''

import os
from argparse import ArgumentParser
from math import ceil
import wx.lib.scrolledpanel as scrolled
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_util
import mavpicviewer_shared as mpv
from mavpicviewer_settings import mavpicviewer_settings
import mavpicviewer_image
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.wx_loader import wx

prefix_str = "mavpicviewer_mosaic: "


class mavpicviewer_mosaic:
    """displays a mosaic of images"""

    def __init__(self, folderpath, comm_pipe=None):

        # keep reference to callbacks
        self.image_comm_pipe = comm_pipe

        # init current filenumber and filelist
        self.filenumber = 0
        self.filelist = {}
        self.update_file_list(folderpath)

        # init dictionary of POI and panels for all images
        self.poi_dict = {}

        # init dictionary of image locations
        self.image_loc_dict = {}

        # init dictionary of image temperatures
        self.image_temp_dict = {}
        self.image_temp_dict_sorted = {}

        # hardcoded thumbnail image size and number of columns
        self.thumb_size = 100
        self.thumb_columns = 5
        self.thumb_rows = ceil(len(self.filelist) / self.thumb_columns)

        # create window
        self.app = wx.App()

        # create frame
        self.frame = wx.Frame(None, size=(565, 310))
        self.frame.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))
        self.update_title()

        # add settings window
        self.mavpicviewer_settings = mavpicviewer_settings(self.settings_changed_cb)

        # add menu
        self.menu = wx.Menu()
        self.menu.Append(1, "Open Folder", "Open a Folder of images")
        self.frame.Bind(wx.EVT_MENU, self.menu_open_folder, id=1)
        self.menu.Append(2, "Show All", "Show all images")
        self.frame.Bind(wx.EVT_MENU, self.menu_display_all_images, id=2)
        self.menu.Append(3, "Show POI", "Show images with POI")
        self.frame.Bind(wx.EVT_MENU, self.menu_display_poi_images, id=3)
        self.menu.Append(4, "Sort by Temp", "Sort images by Temp (max first)")
        self.frame.Bind(wx.EVT_MENU, self.menu_sort_by_temp, id=4)
        self.menu.Append(5, "Settings", "Settings")
        self.frame.Bind(wx.EVT_MENU, self.mavpicviewer_settings.show_settings_window, id=5)
        self.menu.Append(6, "Clear All POI", "Clear POI from all images")
        self.frame.Bind(wx.EVT_MENU, self.menu_clear_all_poi, id=6)
        self.menu.Append(7, "Quit", "Quit")
        self.frame.Bind(wx.EVT_MENU, self.menu_quit, id=7)
        self.menu_bar = wx.MenuBar()
        self.menu_bar.Append(self.menu, "Menu")
        self.frame.SetMenuBar(self.menu_bar)

        # add bindings for arrow keys
        self.frame.Bind(wx.EVT_CHAR_HOOK, self.on_key)

        # add binding for Close window
        self.frame.Bind(wx.EVT_CLOSE, self.menu_quit)

        # add a timer for redrawing
        timer_hz = 10
        self.queue_processing_timer = wx.Timer(self.frame)
        self.frame.Bind(wx.EVT_TIMER, self.handle_timer_event, self.queue_processing_timer)
        self.queue_processing_timer.Start(int(1000/timer_hz))

        # add a read-only status text box
        self.text_status = wx.TextCtrl(self.frame, id=-1, size=(600, 60), style=wx.TE_READONLY | wx.TE_MULTILINE | wx.TE_RICH)

        # add a scrolled panel
        self.scrolled_panel = scrolled.ScrolledPanel(self.frame, -1, size=(600, 600), style=wx.TAB_TRAVERSAL)
        self.scrolled_panel_sizer = wx.GridSizer(cols=5, hgap=5, vgap=5)

        # dictionary of panels for all images (used to turn on/off highlighting)
        self.panel_dict = {}
        self.update_panel_dict()

        self.scrolled_panel.SetSizer(self.scrolled_panel_sizer)
        self.scrolled_panel.SetupScrolling(scroll_x=True, scroll_y=True)

        # add a vertical and horizontal sizers
        self.vert_sizer = wx.BoxSizer(wx.VERTICAL)
        self.horiz_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # set size hints and add sizer to frame
        self.vert_sizer.Add(self.text_status, proportion=0, flag=wx.EXPAND, border=5)
        self.vert_sizer.Add(self.scrolled_panel, proportion=0, flag=wx.EXPAND | wx.ALL, border=5)
        self.vert_sizer.Add(self.horiz_sizer, proportion=0, flag=wx.EXPAND)
        self.frame.SetSizer(self.vert_sizer)

        # display all images and update layout
        self.display_all_images()

        # set focus on the status text box
        self.text_status.SetFocus()

        # set filenumber to init highlighting
        self.set_filenunmber(0)

        # show frame
        self.frame.Show()

        # this does not return until the window is Closed
        self.app.MainLoop()

    # update panel dictionary
    def update_panel_dict(self):
        # clear entries from panel dictionary
        self.panel_dict.clear()

        # create panel for each image
        for i in range(len(self.filelist)):
            # create a panel for each image (this allows us to set a background colour)
            panel = wx.Panel(self.scrolled_panel, -1, size=(104, 104))

            # create an image for each panel
            img_path = self.filelist[i]
            wx_image = wx.Image(img_path, wx.BITMAP_TYPE_ANY).Scale(100, 100)
            image = wx.StaticBitmap(panel, i, wx.Bitmap(wx_image))

            # bind event for left mouse button click
            image.Bind(wx.EVT_LEFT_DOWN, self.on_image_click)

            # create a box sizer to center the image within the panel
            sizer = wx.BoxSizer(wx.VERTICAL)
            sizer.Add(image, 0, wx.ALIGN_CENTER | wx.ALL, 2)
            panel.SetSizer(sizer)

            # add panel to dictionary
            self.panel_dict[i] = panel

    # process menu events
    def menu_open_folder(self, event):
        """process menu events"""
        dlg = wx.DirDialog(self.frame, "Choose a directory:", style=wx.DD_DEFAULT_STYLE)
        folder_path = None
        if dlg.ShowModal() == wx.ID_OK:
            folder_path = dlg.GetPath()
        dlg.Destroy()
        if folder_path is not None:
            wx.CallAfter(self.open_folder, folder_path)

    # menuy event handler for display all images
    def menu_display_all_images(self, event):
        """menu event handler for display all images"""
        wx.CallAfter(self.display_all_images)

    # display POI images only
    def menu_display_poi_images(self, event):
        """display POI images only"""
        wx.CallAfter(self.display_all_images, poi_only=True)

    # sort images by temperature
    def menu_sort_by_temp(self, event):
        """sort images by temperature"""
        wx.CallAfter(self.sort_images_by_temperature)

    # clear all POI
    def menu_clear_all_poi(self, event):
        """clear all POI"""
        for filenumber in list(self.poi_dict.keys()):
            self.clear_image_poi(filenumber)
        self.send_comm_object(mpv.ClearAllPOI())

    # process menu quit event
    def menu_quit(self, event, notify_image_viewer=True):
        """process menu quit event"""
        if notify_image_viewer:
            # send Close command to image viewer
            self.send_comm_object(mpv.Close())

        # Close frame and exit main loop
        self.app.ExitMainLoop()

    # handle timer event.  used to consume messages from the comm pipe
    def handle_timer_event(self, event):
        """handle timer event"""
        if self.image_comm_pipe is not None:
            while self.image_comm_pipe.poll():
                wx.CallAfter(self.handle_comm_object, self.image_comm_pipe.recv())

    # handle an object received on the communication pipe
    def handle_comm_object(self, obj):
        """process an object received on the communication pipe"""
        if isinstance(obj, mpv.SetFilenumber):
            self.set_filenunmber(obj.filenumber, False)
        elif isinstance(obj, mpv.Close):
            self.menu_quit(None, False)
        elif isinstance(obj, mpv.SetImageLoc):
            self.image_loc_dict[obj.filenumber] = obj.loc
            self.update_status_text()
        elif isinstance(obj, mpv.SetPOI):
            self.set_image_poi(obj.filenumber, obj.poi)
            self.update_status_text()
        elif isinstance(obj, mpv.ClearPOI):
            self.clear_image_poi(obj.filenumber)
        elif isinstance(obj, mpv.SetTempAndPos):
            if obj.temp_max is not None:
                self.image_temp_dict[obj.filenumber] = mpv.TempAndPos(obj.temp_max, obj.temp_pos_x, obj.temp_pos_y)
                self.update_status_text()

    # send a communication object to the image viewer
    def send_comm_object(self, obj):
        """send a communication object to the image viewer"""
        if self.image_comm_pipe is not None:
            self.image_comm_pipe.send(obj)

    # set window title
    def set_title(self, title):
        """set image title"""
        self.frame.SetTitle(title)

    # update title
    def update_title(self):
        """update title"""
        title_str = f"PicViewer Mosaic ({self.filenumber+1} of {len(self.filelist)})"
        self.set_title(title_str)

    # update status text
    def update_status_text(self):
        """update status text"""
        status_text = ""
        # add image location
        if self.filenumber in self.image_loc_dict:
            loc = self.image_loc_dict[self.filenumber]
            alt_above_terr = int(loc.alt - loc.terr_alt)
            status_text = f"Camera: lat:{loc.lat:.7f} lon:{loc.lon:.7f} alt:{loc.alt:.0f} alt_above_terr:{alt_above_terr}\n"
        # calculate average poi lat,lon,alt
        if self.filenumber in self.poi_dict:
            poi = self.poi_dict[self.filenumber]
            if poi is not None:
                loc_avg = mpv.Loc((poi.loc1.lat + poi.loc2.lat) / 2,
                                  (poi.loc1.lon + poi.loc2.lon) / 2,
                                  (poi.loc1.alt + poi.loc2.alt) / 2)
                status_text = status_text + f"POI: lat:{loc_avg.lat:.7f} lon:{loc_avg.lon:.7f} alt:{loc_avg.alt:.0f}\n"
        # add image temperature
        if self.filenumber in self.image_temp_dict:
            tap = self.image_temp_dict[self.filenumber]
            status_text = status_text + f"Temp: {tap.temp_max:.1f}C at ({tap.temp_pos_x}, {tap.temp_pos_y})"
        self.text_status.SetValue(status_text)

    # update filelist dictionary which holds the filepaths for all images
    def update_file_list(self, folderpath):
        filelist = mpv.get_file_list(folderpath, ['jpg', 'jpeg'])
        if filelist is None or not filelist:
            filelist = {}
            print(prefix_str + "no files found")
        self.filelist = filelist

    # open folder
    def open_folder(self, folderpath):
        # get list of files in folder
        self.update_file_list(folderpath)

        # reset current filenumber
        self.filenumber = 0

        # clear poi dictionary
        self.poi_dict.clear()

        # clear image location dictionary
        self.image_loc_dict.clear()

        # update panel dictionary
        self.update_panel_dict()

        # display all images
        self.display_all_images()

        # notify image viewer
        self.send_comm_object(mpv.SetFilelist(self.filelist))

    # set image poi
    def set_image_poi(self, filenumber, poi):
        """set image poi"""
        print(f"set_image_poi: {filenumber} {poi.loc1.lat} {poi.loc1.lon} {poi.loc1.alt}")
        self.poi_dict[filenumber] = poi
        self.update_status_text()

    # clear image poi
    def clear_image_poi(self, filenumber):
        """set image poi"""
        if filenumber in self.poi_dict:
            del self.poi_dict[filenumber]
            if filenumber == self.filenumber:
                self.update_status_text()
            self.update_highlighting(filenumber)

    # process key events
    def on_key(self, event):
        """process key events"""
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_LEFT:
            wx.CallAfter(self.set_filenunmber, self.filenumber-1)
        elif keycode == wx.WXK_RIGHT:
            wx.CallAfter(self.set_filenunmber, self.filenumber+1)
        elif keycode == wx.WXK_UP:
            wx.CallAfter(self.set_filenunmber, self.filenumber-5)
        elif keycode == wx.WXK_DOWN:
            wx.CallAfter(self.set_filenunmber, self.filenumber+5)

    # process window events
    def on_image_click(self, event):
        """process image click event"""
        wx.CallAfter(self.set_filenunmber, event.GetId())

    # set filenumber and update display
    def set_filenunmber(self, filenumber, notify_image_viewer=True):
        """set filenumber"""
        num_files = len(self.filelist)
        if filenumber < 0 or filenumber >= len(self.filelist):
            print(prefix_str + "ignoring invalid filenumber %d (>%d)" % (filenumber, num_files-1))
            return

        # record previous filenumber
        prev_filenumber = self.filenumber

        # set current filenumber
        self.filenumber = filenumber

        # update highlighting for previous and current image
        self.update_highlighting(prev_filenumber)
        self.update_highlighting(self.filenumber)

        # scroll into view
        self.scroll_to_be_visible(self.filenumber)

        # update title and status text
        self.update_title()
        self.update_status_text()

        # notify image viewer
        self.send_comm_object(mpv.SetFilenumber(self.filenumber))

    # update highlighting for an image
    def update_highlighting(self, filenumber):
        """update highlighting for an image"""
        # return immediately if invalid filenumber
        if filenumber < 0 or filenumber >= len(self.filelist):
            return

        # get panel for filenumber
        panel = self.panel_dict[filenumber]
        if panel is None:
            return

        # get highlight colour
        background_colour = wx.WHITE
        if filenumber == self.filenumber:
            # green for current image
            background_colour = wx.GREEN
        elif filenumber in self.poi_dict:
            # red for images with POI
            background_colour = wx.RED

        # set background colour
        panel.SetBackgroundColour(background_colour)

        # refresh panel
        panel.Refresh()
        panel.Update()
        panel.Layout()

    # scroll to be visible
    def scroll_to_be_visible(self, filenumber):
        """scroll image to be visible"""
        panel = self.panel_dict[self.filenumber]
        if panel is not None:
            self.scrolled_panel.ScrollChildIntoView(panel)

    # display all images
    def display_all_images(self, poi_only=False, sort_by_temp=False):
        """display all images"""
        # detach and hide all panels
        for p in self.scrolled_panel_sizer.GetChildren():
            panel = p.GetWindow()
            self.scrolled_panel_sizer.Detach(panel)
            panel.Hide()

        # create list of filenumbers (either the full list or the sorted by temperature list)
        if sort_by_temp:
            filenumber_list = list(self.image_temp_dict_sorted.keys())
        else:
            filenumber_list = list(range(len(self.filelist)))

        # add panels to scrolled panel sizer
        for i in filenumber_list:
            # check if we should only display POI images
            if poi_only is True and i not in self.poi_dict:
                continue
            panel = self.panel_dict[i]
            if panel is not None:
                self.scrolled_panel_sizer.Add(panel, proportion=0, flag=wx.ALL, border=2, userData=i)
                panel.Show()

        # update frame layout
        self.frame.Layout()

    # set filenumber and update display
    def hide_image(self, filenumber):
        # sanity check filenumber
        num_files = len(self.filelist)
        if filenumber < 0 or filenumber >= len(self.filelist):
            print("picviewer mosaic: ignoring invalid filenumber %d (>%d)" % (filenumber, num_files-1))
            return

        # hide image
        panel = self.scrolled_panel_sizer.GetItem(self.filenumber).GetWindow()
        self.scrolled_panel_sizer.Detach(panel)
        self.scrolled_panel_sizer.Layout()

    # handle settings changes callback
    # this is called by the settings window when a setting is changed
    def settings_changed_cb(self, name, value):
        """handle settings changes callback"""
        if name == "FOV":
            self.send_comm_object(mpv.SetFOV(float(value)))
        if name == "YAW":
            self.send_comm_object(mpv.SetYaw(float(value)))

    # sort images by temperature
    def sort_images_by_temperature(self):
        """sort images by temperature (max temp first)"""
        # print warning if not all images have temperature data
        if len(self.image_temp_dict) != len(self.filelist):
            print(prefix_str + "only %d (of %d) images have temp" % (len(self.image_temp_dict), len(self.filelist)))
            # request temperatures from image viewer
            self.send_comm_object(mpv.GetTempForAllImages())

        # update dictionary of images sorted by temperature
        self.image_temp_dict_sorted = dict(sorted(self.image_temp_dict.items(),
                                                  key=lambda item: item[1].temp_max, reverse=True))

        # display all images sorted by temperature
        self.display_all_images(sort_by_temp=True)


# main function
if __name__ == "__main__":
    multiproc.freeze_support()
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("filepath", nargs='?', default=".", help="filename or directory holding images")
    args = parser.parse_args()

    # check destination directory exists
    if not os.path.exists(args.filepath):
        exit(prefix_str + "invalid destination directory")

    # create queue for interprocess communication
    mosaic_to_image_comm, image_to_mosaic_comm = multiproc.Pipe()

    # create image viewer
    filelist = mpv.get_file_list(args.filepath, ['jpg', 'jpeg'])
    mavpicviewer_image.mavpicviewer_image(filelist, image_to_mosaic_comm)

    # create mosaic
    mavpicviewer_mosaic(args.filepath, mosaic_to_image_comm)
