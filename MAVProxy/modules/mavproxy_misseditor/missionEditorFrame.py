#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# generated by wxGlade 0.6.8 on Wed Jun 11 13:41:49 2014
#

import wx
import wx.grid

# begin wxGlade: dependencies
# end wxGlade

import time, math, os

from MAVProxy.modules.mavproxy_misseditor import me_event
MissionEditorEvent = me_event.MissionEditorEvent

from MAVProxy.modules.mavproxy_misseditor import me_defines

from MAVProxy.modules.mavproxy_misseditor import button_renderer

#define column names via "enums":
ME_COMMAND_COL = 0
ME_P1_COL = 1
ME_P2_COL = 2
ME_P3_COL = 3
ME_P4_COL = 4
ME_LAT_COL = 5 
ME_LON_COL = 6
ME_ALT_COL = 7
ME_FRAME_COL = 8
ME_DELETE_COL = 9
ME_UP_COL = 10
ME_DOWN_COL = 11

class MissionEditorFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MissionEditorFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.label_sync_state = wx.StaticText(self, wx.ID_ANY, "SYNCED     \n", style=wx.ALIGN_CENTRE)
        self.label_wp_radius = wx.StaticText(self, wx.ID_ANY, "WP Radius")
        self.text_ctrl_wp_radius = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_PROCESS_ENTER | wx.TE_PROCESS_TAB)
        self.label_loiter_rad = wx.StaticText(self, wx.ID_ANY, "Loiter Radius")
        self.text_ctrl_loiter_radius = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_PROCESS_ENTER | wx.TE_PROCESS_TAB)
        self.checkbox_loiter_dir = wx.CheckBox(self, wx.ID_ANY, "CW")
        self.checkbox_agl = wx.CheckBox(self, wx.ID_ANY, "AGL")
        self.label_default_alt = wx.StaticText(self, wx.ID_ANY, "Default Alt")
        self.text_ctrl_wp_default_alt = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_PROCESS_ENTER | wx.TE_PROCESS_TAB)
        self.label_home_location = wx.StaticText(self, wx.ID_ANY, "Home Location")
        self.label_home_lat = wx.StaticText(self, wx.ID_ANY, "Lat")
        self.label_home_lat_value = wx.StaticText(self, wx.ID_ANY, "0.0")
        self.label_home_lon = wx.StaticText(self, wx.ID_ANY, "Lon")
        self.label_home_lon_value = wx.StaticText(self, wx.ID_ANY, "0.0")
        self.label_home_alt = wx.StaticText(self, wx.ID_ANY, "Alt (abs)")
        self.label_home_alt_value = wx.StaticText(self, wx.ID_ANY, "0.0")
        self.button_read_wps = wx.Button(self, wx.ID_ANY, "Read WPs")
        self.button_write_wps = wx.Button(self, wx.ID_ANY, "Write WPs")
        self.button_load_wp_file = wx.Button(self, wx.ID_ANY, "Load WP File")
        self.button_save_wp_file = wx.Button(self, wx.ID_ANY, "Save WP File")
        self.grid_mission = wx.grid.Grid(self, wx.ID_ANY, size=(1, 1))
        self.button_add_wp = wx.Button(self, wx.ID_ANY, "Add Below")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_TEXT_ENTER, self.on_wp_radius_enter, self.text_ctrl_wp_radius)
        self.Bind(wx.EVT_TEXT, self.on_wp_radius_changed, self.text_ctrl_wp_radius)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_loiter_rad_enter, self.text_ctrl_loiter_radius)
        self.Bind(wx.EVT_TEXT, self.on_loiter_rad_change, self.text_ctrl_loiter_radius)
        self.Bind(wx.EVT_CHECKBOX, self.on_loiter_dir_cb_change, self.checkbox_loiter_dir)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_wp_default_alt_enter, self.text_ctrl_wp_default_alt)
        self.Bind(wx.EVT_TEXT, self.on_wp_default_alt_change, self.text_ctrl_wp_default_alt)
        self.Bind(wx.EVT_BUTTON, self.read_wp_pushed, self.button_read_wps)
        self.Bind(wx.EVT_BUTTON, self.write_wp_pushed, self.button_write_wps)
        self.Bind(wx.EVT_BUTTON, self.load_wp_file_pushed, self.button_load_wp_file)
        self.Bind(wx.EVT_BUTTON, self.save_wp_file_pushed, self.button_save_wp_file)
        self.Bind(wx.grid.EVT_GRID_CMD_CELL_CHANGE, self.on_mission_grid_cell_changed, self.grid_mission)
        self.Bind(wx.grid.EVT_GRID_CMD_CELL_LEFT_CLICK, self.on_mission_grid_cell_left_click, self.grid_mission)
        self.Bind(wx.grid.EVT_GRID_CMD_SELECT_CELL, self.on_mission_grid_cell_select, self.grid_mission)
        self.Bind(wx.EVT_BUTTON, self.add_wp_below_pushed, self.button_add_wp)
        # end wxGlade

        #use a timer to facilitate event an event handlers for events
        #passed from another process.
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.time_to_process_gui_events, self.timer)
        self.timer.Start(200)

        delete_br = button_renderer.ButtonRenderer("Delete",70,20)
        up_br = button_renderer.ButtonRenderer("+",20,20)
        down_br = button_renderer.ButtonRenderer("-",1,1)

        self.del_attr = wx.grid.GridCellAttr()
        self.del_attr.SetReadOnly(True)
        self.del_attr.SetRenderer(delete_br)

        self.up_attr = wx.grid.GridCellAttr()
        self.up_attr.SetReadOnly(True)
        self.up_attr.SetRenderer(up_br)

        self.down_attr = wx.grid.GridCellAttr()
        self.down_attr.SetReadOnly(True)
        self.down_attr.SetRenderer(down_br)

        self.grid_mission.SetColAttr(ME_DELETE_COL, self.del_attr)
        self.grid_mission.SetColAttr(ME_UP_COL, self.up_attr)
        self.grid_mission.SetColAttr(ME_DOWN_COL, self.down_attr)

        #remember what mission we opened/saved last
        self.last_mission_file_path = ""

        #remember last map click position
        self.last_map_click_pos = None

    def __set_properties(self):
        # begin wxGlade: MissionEditorFrame.__set_properties
        self.SetTitle("Mission Editor")
        self.SetSize((820, 480))
        self.label_sync_state.SetForegroundColour(wx.Colour(12, 152, 26))
        self.label_sync_state.SetFont(wx.Font(14, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, "Droid Sans"))
        self.text_ctrl_wp_radius.SetMinSize((50, 27))
        self.text_ctrl_loiter_radius.SetMinSize((50, 27))
        self.text_ctrl_wp_default_alt.SetMinSize((70, 27))
        self.label_home_lat_value.SetMinSize((100, 17))
        self.label_home_lat_value.SetForegroundColour(wx.Colour(0, 127, 255))
        self.label_home_lon_value.SetMinSize((100, 17))
        self.label_home_lon_value.SetForegroundColour(wx.Colour(0, 127, 255))
        self.label_home_alt_value.SetForegroundColour(wx.Colour(0, 127, 255))
        self.grid_mission.CreateGrid(0, 12)
        self.grid_mission.SetRowLabelSize(20)
        self.grid_mission.SetColLabelSize(20)
        self.grid_mission.SetColLabelValue(0, "Command")
        self.grid_mission.SetColSize(0, 150)
        self.grid_mission.SetColLabelValue(1, "P1")
        self.grid_mission.SetColLabelValue(2, "P2")
        self.grid_mission.SetColLabelValue(3, "P3")
        self.grid_mission.SetColLabelValue(4, "P4")
        self.grid_mission.SetColLabelValue(5, "Lat")
        self.grid_mission.SetColLabelValue(6, "Lon")
        self.grid_mission.SetColLabelValue(7, "Alt")
        self.grid_mission.SetColLabelValue(8, "Frame")
        self.grid_mission.SetColLabelValue(9, "Delete")
        self.grid_mission.SetColLabelValue(10, "Up")
        self.grid_mission.SetColLabelValue(11, "Down")
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MissionEditorFrame.__do_layout
        sizer_3 = wx.BoxSizer(wx.VERTICAL)
        sizer_16 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_12 = wx.BoxSizer(wx.VERTICAL)
        sizer_14 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_13 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_11 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_10 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_9 = wx.BoxSizer(wx.VERTICAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_15 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4.Add(self.label_sync_state, 0, wx.LEFT | wx.ALIGN_CENTER_VERTICAL, 10)
        sizer_4.Add((10, 0), 0, 0, 0)
        sizer_5.Add(self.label_wp_radius, 0, 0, 0)
        sizer_5.Add(self.text_ctrl_wp_radius, 0, 0, 0)
        sizer_5.Add(self.label_loiter_rad, 0, 0, 0)
        sizer_15.Add(self.text_ctrl_loiter_radius, 0, 0, 0)
        sizer_15.Add(self.checkbox_loiter_dir, 0, 0, 0)
        sizer_5.Add(sizer_15, 1, wx.EXPAND, 0)
        sizer_4.Add(sizer_5, 0, wx.EXPAND, 0)
        sizer_4.Add((20, 20), 0, 0, 0)
        sizer_9.Add(self.label_default_alt, 0, 0, 0)
        sizer_9.Add(self.text_ctrl_wp_default_alt, 0, 0, 0)
        sizer_4.Add(sizer_9, 0, wx.EXPAND, 0)
        sizer_4.Add((20, 20), 0, 0, 0)
        sizer_1.Add(self.label_home_location, 0, 0, 0)
        sizer_2.Add(self.label_home_lat, 0, 0, 0)
        sizer_2.Add((40, 0), 0, 0, 0)
        sizer_2.Add(self.label_home_lat_value, 0, 0, 0)
        sizer_1.Add(sizer_2, 1, wx.EXPAND, 0)
        sizer_10.Add(self.label_home_lon, 0, 0, 0)
        sizer_10.Add((36, 0), 0, 0, 0)
        sizer_10.Add(self.label_home_lon_value, 0, 0, 0)
        sizer_1.Add(sizer_10, 1, wx.EXPAND, 0)
        sizer_11.Add(self.label_home_alt, 0, 0, 0)
        sizer_11.Add((11, 0), 0, 0, 0)
        sizer_11.Add(self.label_home_alt_value, 0, 0, 0)
        sizer_1.Add(sizer_11, 1, wx.EXPAND, 0)
        sizer_4.Add(sizer_1, 0, wx.EXPAND, 0)
        sizer_13.Add(self.button_read_wps, 0, 0, 0)
        sizer_13.Add((20, 20), 0, 0, 0)
        sizer_13.Add(self.button_write_wps, 0, 0, 0)
        sizer_12.Add(sizer_13, 1, wx.EXPAND, 0)
        sizer_12.Add((20, 20), 0, 0, 0)
        sizer_14.Add(self.button_load_wp_file, 0, 0, 0)
        sizer_14.Add((20, 20), 0, 0, 0)
        sizer_14.Add(self.button_save_wp_file, 0, 0, 0)
        sizer_12.Add(sizer_14, 1, wx.EXPAND, 0)
        sizer_4.Add(sizer_12, 0, wx.EXPAND, 0)
        sizer_3.Add(sizer_4, 0, wx.EXPAND, 0)
        sizer_3.Add(self.grid_mission, 1, wx.EXPAND, 0)
        sizer_16.Add(self.button_add_wp, 0, 0, 0)
        sizer_3.Add(sizer_16, 0, wx.EXPAND, 0)
        self.SetSizer(sizer_3)
        self.Layout()
        # end wxGlade

    def set_event_queue(self, q):
        self.event_queue = q

    def set_event_queue_lock(self, l):
        self.event_queue_lock = l

    def set_gui_event_queue(self, q):
        self.gui_event_queue = q

    def set_gui_event_queue_lock(self, l):
        self.gui_event_queue_lock = l

    def time_to_process_gui_events(self, evt):
        event_processed = False
        queue_access_start_time = time.time()
        self.gui_event_queue_lock.acquire()
        while self.gui_event_queue.qsize() > 0 and (time.time() < queue_access_start_time) < 0.6:
            event_processed = True
            event = self.gui_event_queue.get()
            if event.get_type() == me_event.MEGE_CLEAR_MISS_TABLE:
                self.grid_mission.ClearGrid()
                if (self.grid_mission.GetNumberRows() > 0):
                    self.grid_mission.DeleteRows(0, 
                            self.grid_mission.GetNumberRows())
                self.grid_mission.SetDefaultColSize(50, True)
                self.grid_mission.SetColSize(ME_COMMAND_COL, 150)
                self.grid_mission.SetColSize(ME_LAT_COL, 100)
                self.grid_mission.SetColSize(ME_LON_COL, 100)
                self.grid_mission.SetColSize(ME_ALT_COL, 75)

                self.grid_mission.ForceRefresh()
            elif event.get_type() == me_event.MEGE_ADD_MISS_TABLE_ROWS:
                num_new_rows = event.get_arg("num_rows")
                if (num_new_rows < 1):
                    continue
                old_num_rows = self.grid_mission.GetNumberRows()
                self.grid_mission.AppendRows(num_new_rows)
                self.prep_new_rows(old_num_rows, num_new_rows)
                self.grid_mission.ForceRefresh()
            elif event.get_type() == me_event.MEGE_SET_MISS_ITEM:
                row = event.get_arg("num") - 1
                command = event.get_arg("command")

                if row == -1:
                    #1st mission item is special: it's the immutable home poitn
                    self.label_home_lat_value.SetLabel(
                            str(event.get_arg("lat")))
                    self.label_home_lon_value.SetLabel(
                            str(event.get_arg("lon")))
                    self.label_home_alt_value.SetLabel(
                            str(event.get_arg("alt")))

                else: #not the first mission item
                    if (me_defines.miss_cmds.has_key(command)):
                        self.grid_mission.SetCellValue(row, ME_COMMAND_COL, 
                                me_defines.miss_cmds[command])
                    else:
                        self.grid_mission.SetCellValue(row, ME_COMMAND_COL,
                                str(command))

                    self.grid_mission.SetCellValue(row, ME_P1_COL, 
                            str(event.get_arg("param1")))
                    self.grid_mission.SetCellValue(row, ME_P2_COL, 
                            str(event.get_arg("param2")))
                    self.grid_mission.SetCellValue(row, ME_P3_COL, 
                            str(event.get_arg("param3")))
                    self.grid_mission.SetCellValue(row, ME_P4_COL, 
                            str(event.get_arg("param4")))
                    self.grid_mission.SetCellValue(row, ME_LAT_COL, 
                            str(event.get_arg("lat")))
                    self.grid_mission.SetCellValue(row, ME_LON_COL, 
                            str(event.get_arg("lon")))
                    self.grid_mission.SetCellValue(row, ME_ALT_COL, 
                            "%.2f" % event.get_arg("alt"))

                    frame_num = event.get_arg("frame") 
                    if (me_defines.frame_enum.has_key(frame_num)): 
                        self.grid_mission.SetCellValue(row, ME_FRAME_COL,
                            me_defines.frame_enum[frame_num])
                    else:
                        self.grid_mission.SetCellValue(row, ME_FRAME_COL, "Und")


            elif event.get_type() == me_event.MEGE_SET_WP_RAD:
                self.text_ctrl_wp_radius.SetValue(str(event.get_arg("wp_rad")))
                self.text_ctrl_wp_radius.SetForegroundColour(wx.Colour(0, 0, 0))
            elif event.get_type() == me_event.MEGE_SET_LOIT_RAD:
                loiter_radius = event.get_arg("loit_rad")    
                self.text_ctrl_loiter_radius.SetValue(
                        str(math.fabs(loiter_radius)))
                self.text_ctrl_loiter_radius.SetForegroundColour(wx.Colour(0, 0, 0))
                if (loiter_radius < 0.0):
                    self.checkbox_loiter_dir.SetValue(False)
                else:
                    self.checkbox_loiter_dir.SetValue(True)
            elif event.get_type() == me_event.MEGE_SET_WP_DEFAULT_ALT:
                self.text_ctrl_wp_default_alt.SetValue(str(
                    event.get_arg("def_wp_alt")))
                self.text_ctrl_wp_default_alt.SetForegroundColour(wx.Colour(0, 0, 0))
            elif event.get_type() == me_event.MEGE_SET_LAST_MAP_CLICK_POS:
                self.last_map_click_pos = event.get_arg("click_pos") 

        self.gui_event_queue_lock.release()
       
        if (event_processed == True):
            #redraw window to apply changes
            self.Refresh()
            self.Update()
        
    def prep_new_row(self, row_num):
        command_choices = me_defines.miss_cmds.values()
        command_choices.sort()
            
        cell_ed = wx.grid.GridCellChoiceEditor(command_choices)
        self.grid_mission.SetCellEditor(row_num, ME_COMMAND_COL, cell_ed)
        self.grid_mission.SetCellValue(row_num, ME_COMMAND_COL, "NAV_WAYPOINT")

        for i in range(1, 7):
            self.grid_mission.SetCellValue(row_num, i, "0.0")

        #set altitude to default:
        self.grid_mission.SetCellValue(row_num, ME_ALT_COL, 
                self.text_ctrl_wp_default_alt.GetValue())

        #populate frm cell editor and set to default value

        frame_cell_ed = wx.grid.GridCellChoiceEditor(me_defines.frame_enum.values())
        self.grid_mission.SetCellEditor(row_num, ME_FRAME_COL, frame_cell_ed)

        # default to previous rows frame
        if row_num > 0:
            frame = self.grid_mission.GetCellValue(row_num-1, ME_FRAME_COL)
            if frame not in ["Rel", "AGL"]:
                frame = "Rel"
        else:
            frame = "Rel"
        self.grid_mission.SetCellValue(row_num, ME_FRAME_COL, frame)

        #this makes newest row always have the cursor in it,
        #making the "Add Below" button work like I want:
        self.grid_mission.SetGridCursor(row_num, ME_COMMAND_COL)
        
    def prep_new_rows(self, start_row, num_rows):
        num_remaining = num_rows
        current_row = start_row
        while num_remaining > 0:
            self.prep_new_row(current_row)
            current_row = current_row + 1
            num_remaining = num_remaining - 1

    def set_modified_state(self, modified):
        if (modified):
            self.label_sync_state.SetLabel("MODIFIED")
            self.label_sync_state.SetForegroundColour(wx.Colour(255, 0, 0))
        else:
            self.label_sync_state.SetLabel("SYNCED")
            self.label_sync_state.SetForegroundColour(wx.Colour(12, 152, 26))

    def read_wp_pushed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_READ_WPS))

        #sneak in some queries about a few other items as well:
        self.event_queue.put(MissionEditorEvent(me_event.MEE_GET_WP_RAD))
        self.event_queue.put(MissionEditorEvent(me_event.MEE_GET_LOIT_RAD))
        self.event_queue.put(MissionEditorEvent(me_event.MEE_GET_WP_DEFAULT_ALT))
        
        self.event_queue_lock.release()
        event.Skip()

        #TODO: the read actually has to succeed before I can say this:
        self.set_modified_state(False)

    def write_wp_pushed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_WRITE_WPS,count=
            self.grid_mission.GetNumberRows()+1))

        #home point first:
        lat = float(self.label_home_lat_value.GetLabel())
        lon = float(self.label_home_lon_value.GetLabel())
        alt = float(self.label_home_alt_value.GetLabel())
        self.event_queue.put(MissionEditorEvent(me_event.MEE_WRITE_WP_NUM,
                                                num=0,cmd_id=16,p1=0.0,p2=0.0,p3=0.0,p4=0.0,
                                                lat=lat,lon=lon,alt=alt,frame=0))
            
        for i in range(0, self.grid_mission.GetNumberRows()):
            cmd_id = me_defines.cmd_reverse_lookup(self.grid_mission.GetCellValue(i,0))
            #don't lock up the misseditor on missing input!
            #anything missing will just be zero
            try:
                p1 = float(self.grid_mission.GetCellValue(i,ME_P1_COL))
            except:
                p1 = 0.0
            try:
                p2 = float(self.grid_mission.GetCellValue(i,ME_P2_COL))
            except:
                p2 = 0.0
            try:
                p3 = float(self.grid_mission.GetCellValue(i,ME_P3_COL))
            except:
                p3 = 0.0
            try:
                p4 = float(self.grid_mission.GetCellValue(i,ME_P4_COL))
            except:
                p4 = 0.0
            try:
                lat = float(self.grid_mission.GetCellValue(i,ME_LAT_COL))
            except:
                lat = 0.0
            try:
                lon = float(self.grid_mission.GetCellValue(i,ME_LON_COL))
            except:
                lon = 0.0
            try:
                alt = float(self.grid_mission.GetCellValue(i,ME_ALT_COL))
            except:
                alt = 0.0
            try:
                frame = float(me_defines.frame_enum_rev[self.grid_mission.GetCellValue(i,ME_FRAME_COL)])
            except:
                frame = 0.0                
                
            self.event_queue.put(MissionEditorEvent(me_event.MEE_WRITE_WP_NUM,
                                                    num=i+1,cmd_id=cmd_id,p1=p1,p2=p2,p3=p3,p4=p4,
                                                    lat=lat,lon=lon,alt=alt,frame=frame))

        self.event_queue_lock.release()

        self.set_modified_state(False)

        event.Skip()

    def load_wp_file_pushed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        fd = wx.FileDialog(self, "Open Mission File", os.getcwd(), "",
                "*.wp|*", wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
        if (fd.ShowModal() == wx.ID_CANCEL):
            return #user changed their mind...
       
        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_LOAD_WP_FILE,
            path=fd.GetPath()))
        self.event_queue_lock.release()
        
        self.last_mission_file_path = fd.GetPath()
        
        event.Skip()

    def add_wp_below_pushed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        row_selected = self.grid_mission.GetGridCursorRow()
        if (row_selected < 0):
            row_selected = self.grid_mission.GetNumberRows()-1
        
        self.grid_mission.InsertRows(row_selected+1)
        self.prep_new_row(row_selected+1)

        #set lat/lon based on last map click position:
        if self.last_map_click_pos is not None:
            self.grid_mission.SetCellValue(row_selected + 1, ME_LAT_COL,
                    str(self.last_map_click_pos[0]))
            self.grid_mission.SetCellValue(row_selected + 1, ME_LON_COL,
                    str(self.last_map_click_pos[1]))            

        #highlight new row
        self.grid_mission.SelectRow(row_selected+1)
        self.set_modified_state(True)
        
        event.Skip()

    def save_wp_file_pushed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        fd = wx.FileDialog(self, "Save Mission File", os.getcwd(),
                self.last_mission_file_path, "*.wp|*",
                wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)
        if (fd.ShowModal() == wx.ID_CANCEL):
            return #user change their mind...

        #ask mp_misseditor module to save file
        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_SAVE_WP_FILE,
            path=fd.GetPath()))
        self.event_queue_lock.release()
        
        self.last_mission_file_path = fd.GetPath()

        event.Skip()

    def on_mission_grid_cell_select(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        command = self.grid_mission.GetCellValue(event.GetRow(), ME_COMMAND_COL)
        self.grid_mission.SetColLabelValue(ME_P1_COL, "P1")
        self.grid_mission.SetColLabelValue(ME_P2_COL, "P2")
        self.grid_mission.SetColLabelValue(ME_P3_COL, "P3")
        self.grid_mission.SetColLabelValue(ME_P4_COL, "P4")
        self.grid_mission.SetColLabelValue(ME_LAT_COL, "Lat")
        self.grid_mission.SetColLabelValue(ME_LON_COL, "Lon")
        self.grid_mission.SetColLabelValue(ME_ALT_COL, "Alt")

        col_labels = me_defines.get_column_labels(command)
        for col in col_labels.keys():
            self.grid_mission.SetColLabelValue(col, col_labels[col])

        event.Skip()
    
    def on_mission_grid_cell_left_click(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        #delete column?
        if (event.GetCol() == ME_DELETE_COL):
            row = event.GetRow()
            dlg = wx.MessageDialog(self, 'Sure you want to delete item ' + str(row+1) + '?', 'Really Delete?', wx.YES_NO | wx.ICON_EXCLAMATION)
            result = dlg.ShowModal()
            
            if (result == wx.ID_YES):
                #delete this row
                self.grid_mission.DeleteRows(row)
                self.set_modified_state(True)
        #up column?
        elif (event.GetCol() == ME_UP_COL):
            row = event.GetRow()
            if (row == 0): #can't go any higher
                return
            #insert a copy of this row above the previous row, then delete this row
            self.grid_mission.InsertRows(row-1)     
            self.prep_new_row(row-1)
            for i in range(0, self.grid_mission.GetNumberCols()):
                self.grid_mission.SetCellValue(row-1, i,
                    self.grid_mission.GetCellValue(row+1,i))
            self.grid_mission.DeleteRows(row+1)
            #move the cursor to where the row moved and highlight the row
            self.grid_mission.SetGridCursor(row-1,ME_UP_COL)
            self.grid_mission.SelectRow(row-1)

            self.set_modified_state(True)
        
            #Return so we don't skip the event so the cursor will go where expected
            return

        #down column?
        elif (event.GetCol() == ME_DOWN_COL):
            row = event.GetRow()
            if (row == self.grid_mission.GetNumberRows() - 1): #can't go lower
                return

            #insert a copy of this row 2 rows down, then delete this row
            self.grid_mission.InsertRows(row+2)
            self.prep_new_row(row+2)
            for i in range(0, self.grid_mission.GetNumberCols()):
                self.grid_mission.SetCellValue(row+2, i,
                    self.grid_mission.GetCellValue(row, i))
            self.grid_mission.DeleteRows(row)

            #move the cursor to where the row moved and highlight the row
            self.grid_mission.SetGridCursor(row+1,ME_DOWN_COL)
            self.grid_mission.SelectRow(row+1)

            self.set_modified_state(True)

            #Return so we don't skip the event so the cursor will go where expected
            return
         
        event.Skip()

    def on_mission_grid_cell_changed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        self.set_modified_state(True) 
        event.Skip()
    def on_wp_radius_changed(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        #change text red
        self.text_ctrl_wp_radius.SetForegroundColour(wx.Colour(255, 0, 0))

        event.Skip()
    def on_wp_radius_enter(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        try:
            wp_radius = float(self.text_ctrl_wp_radius.GetValue())
        except Exception as e:
            print e.strerror
            return

        #text back to black
        self.text_ctrl_wp_radius.SetForegroundColour(wx.Colour(0, 0, 0))

        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_SET_WP_RAD,
            rad=wp_radius))
        self.event_queue_lock.release()

        event.Skip()

    def send_loiter_rad_msg(self):
        try:
            loit_radius = float(self.text_ctrl_loiter_radius.GetValue())
        except Exception as e:
            print e.strerror
            return

        if (not self.checkbox_loiter_dir.IsChecked()):
            loit_radius = loit_radius * -1.0

        #text back to black
        self.text_ctrl_loiter_radius.SetForegroundColour(wx.Colour(0, 0, 0))

        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_SET_LOIT_RAD,
            rad=loit_radius))
        self.event_queue_lock.release()

    def on_loiter_rad_enter(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        self.send_loiter_rad_msg()        

        event.Skip()

    def on_loiter_rad_change(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        #change text red
        self.text_ctrl_loiter_radius.SetForegroundColour(wx.Colour(255, 0, 0))

        event.Skip()
    def on_loiter_dir_cb_change(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        self.send_loiter_rad_msg()
        event.Skip()

    def on_wp_default_alt_enter(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        try:
            def_alt = float(self.text_ctrl_wp_default_alt.GetValue())
        except Exception as e:
            print e.strerror
            return

        #text back to black
        self.text_ctrl_wp_default_alt.SetForegroundColour(wx.Colour(0, 0, 0))

        self.event_queue_lock.acquire()
        self.event_queue.put(MissionEditorEvent(me_event.MEE_SET_WP_DEFAULT_ALT,
            alt=def_alt))
        self.event_queue_lock.release()

        event.Skip()

    def on_wp_default_alt_change(self, event):  # wxGlade: MissionEditorFrame.<event_handler>
        #change text red
        self.text_ctrl_wp_default_alt.SetForegroundColour(wx.Colour(255, 0, 0))

        event.Skip()
# end of class MissionEditorFrame
if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    missionEditor = (None, wx.ID_ANY, "")
    app.SetTopWindow(missionEditor)
    missionEditor.Show()
    app.MainLoop()
