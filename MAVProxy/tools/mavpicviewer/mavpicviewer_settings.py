#!/usr/bin/env python3

'''
MAV Picture Viewer Settings Window

User configuration of Settings

AP_FLAKE8_CLEAN
'''

from MAVProxy.modules.lib import mp_util
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.wx_loader import wx


class mavpicviewer_settings:
    """window to capture picviewer settings"""

    def __init__(self, settings_changed_cb):

        # add settings window
        self.settings_frame = wx.Frame(None, title="Settings", size=(300, 80))

        # add FOV input box and
        self.settings_fov_text = wx.StaticText(self.settings_frame, id=-1, label="FOV", pos=(10, 10))
        self.settings_fov_input = wx.TextCtrl(self.settings_frame, id=-1, pos=(50, 10), size=(100, -1),
                                              style=wx.TE_PROCESS_ENTER)

        # add Set FOV button
        self.settings_set_button = wx.Button(self.settings_frame, id=-1, label="Set FOV", pos=(200, 10), size=(75, 25))
        self.settings_frame.Bind(wx.EVT_BUTTON, self.settings_set_button_click, self.settings_set_button)
        self.settings_frame.Bind(wx.EVT_TEXT_ENTER, self.settings_set_button_click, self.settings_fov_input)

        # add Yaw input box
        self.settings_yaw_text = wx.StaticText(self.settings_frame, id=-1, label="Yaw", pos=(10, 40))
        self.settings_yaw_input = wx.TextCtrl(self.settings_frame, id=-1, pos=(50, 40), size=(100, -1),
                                              style=wx.TE_PROCESS_ENTER)

        # add Set Yaw button
        self.settings_setyaw_button = wx.Button(self.settings_frame, id=-1, label="Set YAW", pos=(200, 40), size=(75, 25))
        self.settings_frame.Bind(wx.EVT_BUTTON, self.settings_setyaw_button_click, self.settings_setyaw_button)
        self.settings_frame.Bind(wx.EVT_TEXT_ENTER, self.settings_setyaw_button_click, self.settings_yaw_input)

        # handle close event
        self.settings_frame.Bind(wx.EVT_CLOSE, self.settings_close_button_click)

        # record callback function
        self.settings_changed_cb = settings_changed_cb

    # show settings window
    def show_settings_window(self, event):
        """show settings window"""
        self.settings_frame.Show()

    # settings window event handlers
    def settings_set_button_click(self, event):
        """settings window event handlers"""
        self.settings_changed_cb("FOV", self.settings_fov_input.GetValue())
        self.settings_frame.Hide()

    def settings_setyaw_button_click(self, event):
        """settings window event handlers"""
        self.settings_changed_cb("YAW", self.settings_yaw_input.GetValue())
        print("YAW set to: ", self.settings_yaw_input.GetValue())
        self.settings_frame.Hide()

    # close/hide settings window
    def settings_close_button_click(self, event):
        """settings window event handlers"""
        self.settings_frame.Hide()
