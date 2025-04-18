"""
Terrain navigation UI
"""

from MAVProxy.modules.lib.wx_loader import wx

from MAVProxy.modules.mavproxy_terrainnav import terrainnav_msgs


class TerrainNavFrame(wx.Frame):
    def __init__(self, state, title, size):
        super().__init__(None, title=title, size=size)
        self._state = state

        # control update rate
        self._timer = wx.Timer(self)

        # buttons
        button_size = (120, 25)

        # locations
        self._button_set_start = wx.Button(
            self, id=wx.ID_ANY, label="Set Start", size=button_size
        )
        self._button_set_goal = wx.Button(
            self, id=wx.ID_ANY, label="Set Goal", size=button_size
        )
        self._button_add_rally = wx.Button(
            self, id=wx.ID_ANY, label="Add Rally", size=button_size
        )
        self._button_add_wp = wx.Button(
            self, id=wx.ID_ANY, label="Add Waypoint", size=button_size
        )
        # planning
        self._button_run_planner = wx.Button(
            self, id=wx.ID_ANY, label="Run Planner", size=button_size
        )
        self._button_gen_waypoints = wx.Button(
            self, id=wx.ID_ANY, label="Gen Waypoints", size=button_size
        )
        self._button_clear_path = wx.Button(
            self, id=wx.ID_ANY, label="Clear Path", size=button_size
        )
        self._button_clear_waypoints = wx.Button(
            self, id=wx.ID_ANY, label="Clear Waypoints", size=button_size
        )
        self._button_clear_all = wx.Button(
            self, id=wx.ID_ANY, label="Clear All", size=button_size
        )

        # navigation actions
        self._button_hold = wx.Button(
            self, id=wx.ID_ANY, label="Hold", size=button_size
        )
        self._button_navigate = wx.Button(
            self, id=wx.ID_ANY, label="Navigate", size=button_size
        )
        self._button_rollout = wx.Button(
            self, id=wx.ID_ANY, label="Rollout", size=button_size
        )
        self._button_abort = wx.Button(
            self, id=wx.ID_ANY, label="Abort", size=button_size
        )
        self._button_return = wx.Button(
            self, id=wx.ID_ANY, label="Return", size=button_size
        )

        # terrain visualisation
        self._button_show_contours = wx.Button(
            self, id=wx.ID_ANY, label="Show Contours", size=button_size
        )
        self._button_hide_contours = wx.Button(
            self, id=wx.ID_ANY, label="Hide Contours", size=button_size
        )

        # planner boundary (map extents)
        self._button_show_boundary = wx.Button(
            self, id=wx.ID_ANY, label="Show Boundary", size=button_size
        )
        self._button_hide_boundary = wx.Button(
            self, id=wx.ID_ANY, label="Hide Boundary", size=button_size
        )
        self._button_move_boundary = wx.Button(
            self, id=wx.ID_ANY, label="Move Boundary", size=button_size
        )

        # layout
        self._vert_sizer1 = wx.BoxSizer(wx.VERTICAL)
        self._vert_sizer1.Add(self._button_set_start, proportion=0, flag=wx.EXPAND)
        self._vert_sizer1.Add(self._button_set_goal, proportion=0, flag=wx.EXPAND)
        self._vert_sizer1.Add(self._button_add_rally, proportion=0, flag=wx.EXPAND)
        self._vert_sizer1.Add(self._button_add_wp, proportion=0, flag=wx.EXPAND)

        self._vert_sizer2 = wx.BoxSizer(wx.VERTICAL)
        self._vert_sizer2.Add(self._button_run_planner, proportion=0, flag=wx.EXPAND)
        self._vert_sizer2.Add(self._button_gen_waypoints, proportion=0, flag=wx.EXPAND)
        self._vert_sizer2.Add(self._button_clear_path, proportion=0, flag=wx.EXPAND)
        self._vert_sizer2.Add(
            self._button_clear_waypoints, proportion=0, flag=wx.EXPAND
        )
        self._vert_sizer2.Add(self._button_clear_all, proportion=0, flag=wx.EXPAND)

        self._vert_sizer3 = wx.BoxSizer(wx.VERTICAL)
        self._vert_sizer3.Add(self._button_hold, proportion=0, flag=wx.EXPAND)
        self._vert_sizer3.Add(self._button_navigate, proportion=0, flag=wx.EXPAND)
        self._vert_sizer3.Add(self._button_rollout, proportion=0, flag=wx.EXPAND)
        self._vert_sizer3.Add(self._button_abort, proportion=0, flag=wx.EXPAND)
        self._vert_sizer3.Add(self._button_return, proportion=0, flag=wx.EXPAND)

        self._vert_sizer4 = wx.BoxSizer(wx.VERTICAL)
        self._vert_sizer4.Add(self._button_show_contours, proportion=0, flag=wx.EXPAND)
        self._vert_sizer4.Add(self._button_hide_contours, proportion=0, flag=wx.EXPAND)
        self._vert_sizer4.Add(self._button_show_boundary, proportion=0, flag=wx.EXPAND)
        self._vert_sizer4.Add(self._button_hide_boundary, proportion=0, flag=wx.EXPAND)
        self._vert_sizer4.Add(self._button_move_boundary, proportion=0, flag=wx.EXPAND)

        self._horz_sizer1 = wx.BoxSizer(wx.HORIZONTAL)
        self._horz_sizer1.Add(self._vert_sizer1, proportion=0, flag=wx.EXPAND)
        self._horz_sizer1.Add(self._vert_sizer2, proportion=0, flag=wx.EXPAND)
        self._horz_sizer1.Add(self._vert_sizer3, proportion=0, flag=wx.EXPAND)

        self._horz_sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self._horz_sizer2.Add(self._vert_sizer4, proportion=0, flag=wx.EXPAND)

        # stack rows of buttons
        self._vert_sizer0 = wx.BoxSizer(wx.VERTICAL)
        self._vert_sizer0.Add(self._horz_sizer1, proportion=0, flag=wx.EXPAND)
        self._vert_sizer0.Add(self._horz_sizer2, proportion=0, flag=wx.EXPAND)
        self.SetSizer(self._vert_sizer0)

        # button events
        self.Bind(wx.EVT_BUTTON, self.on_set_start_pushed, self._button_set_start)
        self.Bind(wx.EVT_BUTTON, self.on_set_goal_pushed, self._button_set_goal)
        self.Bind(wx.EVT_BUTTON, self.on_add_rally_pushed, self._button_add_rally)
        self.Bind(wx.EVT_BUTTON, self.on_add_wp_pushed, self._button_add_wp)

        self.Bind(wx.EVT_BUTTON, self.on_run_planner_pushed, self._button_run_planner)
        self.Bind(
            wx.EVT_BUTTON, self.on_gen_waypoints_pushed, self._button_gen_waypoints
        )
        self.Bind(wx.EVT_BUTTON, self.on_clear_path_pushed, self._button_clear_path)
        self.Bind(
            wx.EVT_BUTTON, self.on_clear_waypoints_pushed, self._button_clear_waypoints
        )
        self.Bind(wx.EVT_BUTTON, self.on_clear_all_pushed, self._button_clear_all)

        self.Bind(wx.EVT_BUTTON, self.on_hold_pushed, self._button_hold)
        self.Bind(wx.EVT_BUTTON, self.on_navigate_pushed, self._button_navigate)
        self.Bind(wx.EVT_BUTTON, self.on_rollout_pushed, self._button_rollout)
        self.Bind(wx.EVT_BUTTON, self.on_abort_pushed, self._button_abort)
        self.Bind(wx.EVT_BUTTON, self.on_return_pushed, self._button_return)

        self.Bind(
            wx.EVT_BUTTON, self.on_show_contours_pushed, self._button_show_contours
        )
        self.Bind(
            wx.EVT_BUTTON, self.on_hide_contours_pushed, self._button_hide_contours
        )

        self.Bind(
            wx.EVT_BUTTON, self.on_show_boundary_pushed, self._button_show_boundary
        )
        self.Bind(
            wx.EVT_BUTTON, self.on_hide_boundary_pushed, self._button_hide_boundary
        )
        self.Bind(
            wx.EVT_BUTTON, self.on_move_boundary_pushed, self._button_move_boundary
        )

        # events
        self.Bind(wx.EVT_TIMER, self.on_timer, self._timer)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_CHAR_HOOK, self.on_key_press)

        # restart the timer
        self._timer.Start(milliseconds=100)

    def on_timer(self, event):
        """
        Handle timer events.
        """
        # check for close events
        if self._state.close_event.wait(timeout=0.001):
            # stop the timer and destroy the window - this ensures
            # the window is closed whhen the module is unloaded
            self._timer.Stop()
            self.Destroy()
            return

        # receive data from the module
        while self._state.ui_pipe_recv.poll():
            msg_list = self._state.ui_pipe_recv.recv()
            for msg in msg_list:
                pass

    def on_idle(self, event):
        """
        Handle idle events.
        """
        pass

    def on_key_press(self, event):
        """
        Handle keypress events.
        """
        pass

    def on_set_start_pushed(self, event):
        msg = terrainnav_msgs.SetStart()
        self._state.ui_pipe_send.send(msg)

    def on_set_goal_pushed(self, event):
        msg = terrainnav_msgs.SetGoal()
        self._state.ui_pipe_send.send(msg)

    def on_add_rally_pushed(self, event):
        msg = terrainnav_msgs.AddRally()
        self._state.ui_pipe_send.send(msg)

    def on_add_wp_pushed(self, event):
        msg = terrainnav_msgs.AddWaypoint()
        self._state.ui_pipe_send.send(msg)

    def on_run_planner_pushed(self, event):
        msg = terrainnav_msgs.RunPlanner()
        self._state.ui_pipe_send.send(msg)

    def on_gen_waypoints_pushed(self, event):
        msg = terrainnav_msgs.GenWaypoints()
        self._state.ui_pipe_send.send(msg)

    def on_hold_pushed(self, event):
        msg = terrainnav_msgs.Hold()
        self._state.ui_pipe_send.send(msg)

    def on_navigate_pushed(self, event):
        msg = terrainnav_msgs.Navigate()
        self._state.ui_pipe_send.send(msg)

    def on_rollout_pushed(self, event):
        msg = terrainnav_msgs.Rollout()
        self._state.ui_pipe_send.send(msg)

    def on_abort_pushed(self, event):
        msg = terrainnav_msgs.Abort()
        self._state.ui_pipe_send.send(msg)

    def on_return_pushed(self, event):
        msg = terrainnav_msgs.Return()
        self._state.ui_pipe_send.send(msg)

    def on_show_contours_pushed(self, event):
        msg = terrainnav_msgs.ShowContours()
        self._state.ui_pipe_send.send(msg)

    def on_hide_contours_pushed(self, event):
        msg = terrainnav_msgs.HideContours()
        self._state.ui_pipe_send.send(msg)

    def on_show_boundary_pushed(self, event):
        msg = terrainnav_msgs.ShowBoundary()
        self._state.ui_pipe_send.send(msg)

    def on_hide_boundary_pushed(self, event):
        msg = terrainnav_msgs.HideBoundary()
        self._state.ui_pipe_send.send(msg)

    def on_move_boundary_pushed(self, event):
        msg = terrainnav_msgs.MoveBoundary()
        self._state.ui_pipe_send.send(msg)

    def on_clear_path_pushed(self, event):
        msg = terrainnav_msgs.ClearPath()
        self._state.ui_pipe_send.send(msg)

    def on_clear_waypoints_pushed(self, event):
        msg = terrainnav_msgs.ClearWaypoints()
        self._state.ui_pipe_send.send(msg)

    def on_clear_all_pushed(self, event):
        msg = terrainnav_msgs.ClearAll()
        self._state.ui_pipe_send.send(msg)
