"""
Terrain navigation module
"""

import math
import sys
import time
import threading

from functools import partial

from MAVProxy.mavproxy import MPState

from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.mavproxy_map import mp_slipmap

from MAVProxy.modules.mavproxy_terrainnav import terrainnav_app
from MAVProxy.modules.mavproxy_terrainnav import terrainnav_msgs

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.wxsettings import WXSettings

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

# terrain navigation
from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.grid_map import GridMapSRTM
from terrain_nav_py.path import Path
from terrain_nav_py.path_segment import PathSegment
from terrain_nav_py.terrain_map import TerrainMap
from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt


class TerrainNavModule(mp_module.MPModule):

    # TODO: some of these settings should be extracted from params
    @staticmethod
    def default_settings():
        return mp_settings.MPSettings(
            [
                MPSetting("loiter_agl_alt", float, 60.0),
                MPSetting("loiter_radius", float, 60.0),  # WP_LOITER_RADIUS
                MPSetting("turning_radius", float, 60.0),  # WP_LOITER_RADIUS
                MPSetting("climb_angle_deg", float, 8.0),  # TECS_CLMB_MAX
                MPSetting("max_agl_alt", float, 100.0),
                MPSetting("min_agl_alt", float, 50.0),
                MPSetting("grid_spacing", float, 30.0),
                MPSetting("grid_length", float, 10000.0),
                MPSetting("time_budget", float, 20.0),
                MPSetting("resolution", float, 100.0),
                MPSetting(
                    "wp_generator",
                    str,
                    "UseLoiterToAlt",
                    choice=["SimpleWaypoints", "UseLoiterToAlt"],
                ),
                MPSetting("wp_spacing", float, 60.0),
                MPSetting("wp_min_loiter_angle_deg", float, 45.0),
            ]
        )

    def __init__(self, mpstate: MPState) -> None:
        super().__init__(mpstate, "terrainnav", "terrain navigation module")

        # *** planner settings ***
        self.terrainnav_settings = TerrainNavModule.default_settings()

        # *** commands ***
        cmdname = "terrainnav"
        # TODO: do not support multi-instance or multi-vehicle yet
        # if self.instance > 1:
        #     cmdname += "%u" % self.instance

        self.add_command(
            cmdname,
            self.cmd_terrainnav,
            "terrainnav control",
            [
                "set (TERRAINNAVSETTING)",
                "clear",
            ],
        )
        self.add_completion_function(
            "(TERRAINNAVSETTING)", self.terrainnav_settings.completion
        )

        # add a sub-menu to map and console
        if mp_util.has_wxpython:
            menu = MPMenuSubMenu(
                "TerrainNav",
                items=[
                    # TODO: add menu items
                ],
            )

            map_module = self.module("map")
            if map_module is not None:
                map_module.add_menu(menu)

            console_module = self.module("console")
            if console_module is not None:
                console_module.add_menu(menu)

        # start the terrain nav app
        self.app = terrainnav_app.TerrainNavApp(title="Terrain Navigation")
        self.app.start_ui()

        # *** ui message update settings and state ***
        self._fps = 10.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9
        self._msg_list = []

        # *** planner state ***
        self._grid_map_lat = None
        self._grid_map_lon = None
        self._candidate_path = None

        self.start_latlon = None
        self.goal_latlon = None

        # *** slip map state ***
        self._map_layer_initialised = False
        self._map_layer_id = "terrainnav"
        self._map_start_id = "terrainnav start"
        self._map_goal_id = "terrainnav goal"
        self._map_path_id = "terrainnav path"
        self._map_states_id = "terrainnav states"
        self._map_boundary_id = "terrainnav boundary"
        self._map_circle_linewidth = 2
        self._is_boundary_visible = False

        # *** fence state ***
        self._fence_change_time = 0

        # *** planner multiprocessing ***
        self._planner_process = None
        self._parent_pipe_recv, self._planner_pipe_send = multiproc.Pipe(duplex=False)
        self._planner_pipe_recv, self._parent_pipe_send = multiproc.Pipe(duplex=False)
        self._planner_close_event = multiproc.Event()
        self._planner_close_event.clear()

        # *** planner settings callback - needs pipe ***
        setting_cb = partial(TerrainNavModule.setting_callback, self._parent_pipe_send)
        self.terrainnav_settings.set_callback(setting_cb)

        self.start_planner()

    def mavlink_packet(self, m) -> None:
        """
        Process a mavlink message.
        """
        mtype = m.get_type()

        # TODO: following mavproxy_map which monitors fence updates in
        #       mavlink_packet rather than idle_task
        self.check_reinit_fencepoints()

    def idle_task(self) -> None:
        """
        Called on idle.
        """
        # tell MAVProxy to unload the module if the UI is closed
        if self.app.close_event.wait(timeout=0.001):
            self.needs_unloading = True

        # process messages from the UI
        self.process_ui_msgs()

        # process messages from the planner
        self.process_planner_msgs()

        # send message list via pipe to UI at desired update rate
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to UI
            self.app.parent_pipe_send.send(self._msg_list)

            # reset counters etc.
            self._msg_list = []
            self._last_send = time.time()

    def unload(self):
        """
        Close the app and unload the module.
        """
        self.clear_slip_map()
        self.app.stop_ui()
        self.stop_planner()

    def cmd_terrainnav(self, args):
        """
        terrainnav commands
        """
        usage = "usage: terrainnav <set|clear>"
        if len(args) < 1:
            print(usage)
        elif args[0] == "set":
            self.cmd_set(args)
        elif args[0] == "clear":
            self.cmd_clear(args)
        else:
            print(usage)

    def cmd_set(self, args):
        """
        Modify a setting
        """
        self.terrainnav_settings.command(args[1:])

    # TODO: review various `clear_xxx`` options
    def cmd_clear(self, args):
        """
        Clear current plan
        """
        self.clear_slip_map()

    def process_ui_msgs(self):
        while self.app.parent_pipe_recv.poll():
            msg = self.app.parent_pipe_recv.recv()

            if isinstance(msg, terrainnav_msgs.SetStart):
                self.set_start()
            elif isinstance(msg, terrainnav_msgs.SetGoal):
                self.set_goal()
            elif isinstance(msg, terrainnav_msgs.AddRally):
                if self.is_debug:
                    print("[terrainnav] Add Rally")
            elif isinstance(msg, terrainnav_msgs.AddWaypoint):
                if self.is_debug:
                    print("[terrainnav] Add Waypoint")
            elif isinstance(msg, terrainnav_msgs.RunPlanner):
                self._parent_pipe_send.send(PlannerCmdRunPlanner())
            elif isinstance(msg, terrainnav_msgs.GenWaypoints):
                self.generate_waypoints()
            elif isinstance(msg, terrainnav_msgs.ClearPath):
                self.clear_path()
            elif isinstance(msg, terrainnav_msgs.ClearWaypoints):
                self.clear_waypoints()
            elif isinstance(msg, terrainnav_msgs.ClearAll):
                self.clear_all()
            elif isinstance(msg, terrainnav_msgs.Hold):
                if self.is_debug:
                    print("[terrainnav] Hold")
            elif isinstance(msg, terrainnav_msgs.Navigate):
                if self.is_debug:
                    print("[terrainnav] Navigate")
            elif isinstance(msg, terrainnav_msgs.Rollout):
                if self.is_debug:
                    print("[terrainnav] Rollout")
            elif isinstance(msg, terrainnav_msgs.Abort):
                if self.is_debug:
                    print("[terrainnav] Abort")
            elif isinstance(msg, terrainnav_msgs.Return):
                if self.is_debug:
                    print("[terrainnav] Return")
            elif isinstance(msg, terrainnav_msgs.ShowContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.display_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.HideContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.hide_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.ShowBoundary):
                self.show_planner_boundary()
            elif isinstance(msg, terrainnav_msgs.HideBoundary):
                self.hide_planner_boundary()
            elif isinstance(msg, terrainnav_msgs.MoveBoundary):
                self.move_planner_boundary()
            elif isinstance(msg, terrainnav_msgs.Settings):
                self.settings_dialog()
            else:
                # TODO: raise an exception
                if self.is_debug:
                    print("[terrainnav] unknown message from UI")

    def process_planner_msgs(self):
        while self._parent_pipe_recv.poll():
            msg = self._parent_pipe_recv.recv()

            if isinstance(msg, PlannerStartLatLon):
                if self.is_debug:
                    print(
                        f"[terrainnav] PlannerStartLatLon: {msg.start_latlon}, "
                        f"is_valid: {msg.is_valid}"
                    )

                # validated start location
                if msg.is_valid:
                    self.start_latlon = msg.start_latlon
                else:
                    self.start_latlon = None

                (lat, lon) = msg.start_latlon
                self.draw_start(lat, lon, msg.is_valid)
            elif isinstance(msg, PlannerGoalLatLon):
                if self.is_debug:
                    print(
                        f"[terrainnav] PlannerGoalLatLon: {msg.goal_latlon}, "
                        f"is_valid: {msg.is_valid}"
                    )

                # validated goal location
                if msg.is_valid:
                    self.goal_latlon = msg.goal_latlon
                else:
                    self.goal_latlon = None

                (lat, lon) = msg.goal_latlon
                self.draw_goal(lat, lon, msg.is_valid)
            elif isinstance(msg, PlannerStatus):
                if self.is_debug:
                    print(f"[terrainnav] PlannerStatus: {msg.status}")
            elif isinstance(msg, PlannerPath):
                if self.is_debug:
                    print(f"[terrainnav] PlannerPath: {msg.path}")
                self._candidate_path = msg.path
                self.draw_path(msg.path)
            elif isinstance(msg, PlannerStates):
                if self.is_debug:
                    print(f"[terrainnav] PlannerStates: {msg.states}")
                self.draw_states(msg.states)
            else:
                # TODO: raise an exception
                if self.is_debug:
                    print("[terrainnav] unknown message from planner")

    def init_slip_map_layer(self):
        """
        Initialise a slip map layer for terrain navigation.
        """
        if self._map_layer_initialised:
            return

        map_module = self.module("map")
        if map_module is None:
            return

        slip_layer = mp_slipmap.SlipClearLayer(self._map_layer_id)
        map_module.map.add_object(slip_layer)
        self._map_layer_initialised = True

    def get_map_click_location(self):
        map_module = self.module("map")
        if map_module is None:
            return (None, None)

        return map_module.mpstate.click_location

    def set_start(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._parent_pipe_send.send(PlannerStartLatLon((lat, lon)))

    def draw_start(self, lat, lon, is_valid):
        radius = self.terrainnav_settings.loiter_radius
        colour = (0, 255, 0) if is_valid else (255, 0, 0)
        self.draw_circle(self._map_start_id, lat, lon, radius, colour)

    def set_goal(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._parent_pipe_send.send(PlannerGoalLatLon((lat, lon)))

    def draw_goal(self, lat, lon, is_valid):
        radius = self.terrainnav_settings.turning_radius
        colour = (0, 255, 0) if is_valid else (255, 0, 0)
        self.draw_circle(self._map_goal_id, lat, lon, radius, colour)

    def show_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        self.draw_planner_boundary()
        self._is_boundary_visible = True

    def hide_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_boundary_id)
        self._is_boundary_visible = False

    def move_planner_boundary(self):
        """
        Recentre the terrain map and recalculate.
        """
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._grid_map_lat = lat
        self._grid_map_lon = lon

        # redraw boundary
        if self._is_boundary_visible:
            self.show_planner_boundary()

        self._parent_pipe_send.send(PlannerGridLatLon((lat, lon)))

    @staticmethod
    def setting_callback(pipe, setting):
        """
        Called when a setting is updated

        :param pipe: pipe used to send setting to the planner
        :type pipe: multiproc.Pipe
        :param setting: the setting that has changed
        :type setting: MPSetting
        """
        if setting.name == "loiter_agl_alt":
            pipe.send(PlannerLoiterAglAlt(setting.value))
        elif setting.name == "loiter_radius":
            pipe.send(PlannerLoiterRadius(setting.value))
        elif setting.name == "turning_radius":
            pipe.send(PlannerTurningRadius(setting.value))
        elif setting.name == "climb_angle_deg":
            pipe.send(PlannerClimbAngleDeg(setting.value))
        elif setting.name == "max_agl_alt":
            pipe.send(PlannerMaxAglAlt(setting.value))
        elif setting.name == "min_agl_alt":
            pipe.send(PlannerMinAglAlt(setting.value))
        elif setting.name == "grid_spacing":
            pipe.send(PlannerGridSpacing(setting.value))
        elif setting.name == "grid_length":
            pipe.send(PlannerGridLength(setting.value))
        elif setting.name == "time_budget":
            pipe.send(PlannerTimeBudget(setting.value))

    def settings_dialog(self):
        """
        Open the settings dialog
        """
        WXSettings(self.terrainnav_settings)

    def draw_circle(self, id, lat, lon, radius, colour):
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        slip_circle = mp_slipmap.SlipCircle(
            key=id,
            layer=self._map_layer_id,
            latlon=(lat, lon),
            radius=radius,
            color=colour,
            linewidth=self._map_circle_linewidth,
        )
        map_module.map.add_object(slip_circle)

    def draw_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon
        offset = 0.5 * self.terrainnav_settings.grid_length

        # planner region boundary: NE, NW, SW, SE
        polygon = []
        polygon.append(mp_util.gps_offset(map_lat, map_lon, offset, offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, -offset, offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, -offset, -offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, offset, -offset))

        if len(polygon) > 1:
            colour = (0, 255, 255)
            slip_polygon = mp_slipmap.UnclosedSlipPolygon(
                self._map_boundary_id,
                polygon,
                layer=self._map_layer_id,
                linewidth=1,
                colour=colour,
                showcircles=False,
                showlines=True,
            )
            map_module.map.add_object(slip_polygon)

    def clear_slip_map(self):
        """
        Remove terrain navigation objects from the map.
        """
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_start_id)
        map_module.map.remove_object(self._map_goal_id)
        map_module.map.remove_object(self._map_path_id)
        map_module.map.remove_object(self._map_states_id)
        map_module.map.remove_object(self._map_boundary_id)
        map_module.map.remove_object(self._map_layer_id)

        self._map_layer_initialised = False

    def clear_path(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_path_id)
        map_module.map.remove_object(self._map_states_id)

        self._candidate_path = None

    def clear_waypoints(self):
        # TODO: only remove waypoints created by this module?
        wp_module = self.module("wp")
        if wp_module is None:
            return

        wp_module.wploader.clear()
        wp_module.wploader.expected_count = 0
        self.mpstate.master().waypoint_count_send(0)
        wp_module.loading_waypoints = True

    def clear_start_goal(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_start_id)
        map_module.map.remove_object(self._map_goal_id)

    def clear_all(self):
        self.clear_path()
        self.clear_waypoints()
        self.clear_start_goal()

    def start_planner(self):
        if self.is_planner_alive():
            return

        # get home position
        wp_module = self.module("wp")
        if wp_module is None:
            return
        home = wp_module.get_home()
        if home is None:
            return

        # TODO: pass initial settings as argument to the planner (dict)?
        self._planner_process = TerrainPlanner(
            self._planner_pipe_send,
            self._planner_pipe_recv,
            self._planner_close_event,
        )
        self._planner_process.start()

        # TODO: add support for a batch update
        # send settings
        self._parent_pipe_send.send(
            PlannerLoiterAglAlt(self.terrainnav_settings.loiter_agl_alt)
        )
        self._parent_pipe_send.send(
            PlannerLoiterRadius(self.terrainnav_settings.loiter_radius)
        )
        self._parent_pipe_send.send(
            PlannerTurningRadius(self.terrainnav_settings.turning_radius)
        )
        self._parent_pipe_send.send(
            PlannerClimbAngleDeg(self.terrainnav_settings.climb_angle_deg)
        )
        self._parent_pipe_send.send(
            PlannerMaxAglAlt(self.terrainnav_settings.max_agl_alt)
        )
        self._parent_pipe_send.send(
            PlannerMinAglAlt(self.terrainnav_settings.min_agl_alt)
        )
        self._parent_pipe_send.send(
            PlannerGridSpacing(self.terrainnav_settings.grid_spacing)
        )
        self._parent_pipe_send.send(
            PlannerGridLength(self.terrainnav_settings.grid_length)
        )
        self._parent_pipe_send.send(
            PlannerTimeBudget(self.terrainnav_settings.time_budget)
        )
        self._parent_pipe_send.send(
            PlannerResolution(self.terrainnav_settings.resolution)
        )

        # TODO: find better way to keep map origin in sync.
        self._grid_map_lat = home.x
        self._grid_map_lon = home.y

        # send initial grid map position
        self._parent_pipe_send.send(PlannerGridLatLon((home.x, home.y)))

    def stop_planner(self):
        if not self.is_planner_alive():
            return

        self._planner_close_event.set()
        self._planner_process.join(timeout=2.0)

        if self.is_planner_alive():
            print(
                f"[terrainnav] planner process timed out, killing it", file=sys.stderr
            )
            self.kill_planner()

    def kill_planner(self):
        self._planner_process.terminate()
        self._parent_pipe_recv, self._planner_pipe_send = multiproc.Pipe(duplex=False)
        self._planner_pipe_recv, self._parent_pipe_send = multiproc.Pipe(duplex=False)

    def is_planner_alive(self):
        return self._planner_process is not None and self._planner_process.is_alive()

    def draw_path(self, path):
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        # convert positions [(east, north)] to polygons [(lat, lon)]
        is_path_valid = True
        polygon = []
        for pos in path.position():
            east = pos.x
            north = pos.y
            alt = pos.z
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

            if self.module("terrain") is not None:
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(*point)
                is_path_valid = is_path_valid and alt > ter_alt

        if len(polygon) > 1:
            colour = (0, 255, 0) if is_path_valid else (255, 0, 0)
            slip_polygon = mp_slipmap.SlipPolygon(
                key=self._map_path_id,
                points=polygon,
                layer=self._map_layer_id,
                linewidth=2,
                colour=colour,
                showcircles=False,
            )
            map_module.map.add_object(slip_polygon)

    def draw_states(self, states):
        """
        :param states: list of Dubins airplane states (x, y, z, yaw)
        :type states: list[float, float, float, float]
        """
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        polygon = []
        for i, state in enumerate(states):
            pos = state[:3]
            yaw = state[3]
            east = pos[0]
            north = pos[1]
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

            if self.module("terrain") is not None:
                lat = point[0]
                lon = point[1]
                alt = pos[2]
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(lat, lon)
                agl_alt = alt - ter_alt
                if self.is_debug:
                    print(
                        f"[terrainnav] "
                        f"state: {i}, east: {east:.2f}, north: {north:.2f}, "
                        f"lat: {lat:.6f}, lon: {lon:.6f}, wp_alt: {alt:.2f}, "
                        f"ter_alt: {ter_alt:.2f}, agl_alt: {agl_alt:.2f}"
                    )

        if len(polygon) > 1:
            colour = (255, 0, 255)
            slip_polygon = mp_slipmap.SlipPolygon(
                key=self._map_states_id,
                points=polygon,
                layer=self._map_layer_id,
                linewidth=2,
                colour=colour,
                showcircles=True,
                showlines=False,
            )
            map_module.map.add_object(slip_polygon)

    def generate_waypoints(self):
        wp_gen = self.terrainnav_settings.wp_generator
        if wp_gen == "SimpleWaypoints":
            self._wp_gen_simple_waypoints()
        elif wp_gen == "UseLoiterToAlt":
            self._wp_gen_use_loiter_to_alt()
        else:
            print(f"[terrainnav] invalid WP generator: {wp_gen}")

    def _wp_gen_home(self, seq, home):
        """
        Create a waypoint for the home position (the first item in a mission)
        """
        p1 = 0.0  # hold
        p2 = 0.0  # accept radius
        p3 = 0.0  # pass radius
        p4 = 0.0  # yaw at waypoint
        mission_item = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=self.mpstate.settings.target_system,
            target_component=self.mpstate.settings.target_component,
            seq=seq,
            frame=home.frame,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=p1,
            param2=p2,
            param3=p3,
            param4=p4,
            x=home.x,
            y=home.y,
            z=home.z,
        )
        return mission_item

    @staticmethod
    def loiter_dir_enu(position, velocity, centre):
        """
        Calculate the loiter direction in ENU frame given a loiter centre
        and position and velocity on the perimeter.

        :param position: position on the loiter circle
        :type position: Vector3
        :param velocity: velocity on the loiter circle
        :type velocity: Vector3
        :param centre: centre of the loiter circle
        :type centre: Vector3
        :return: 1 for CCW and -1 for CW
        """
        cen_2d = Vector3(centre.x, centre.y, 0.0)
        pos_2d = Vector3(position.x, position.y, 0.0)
        tan_2d = Vector3(velocity.x, velocity.y, 0.0).normalized()
        rad_2d = (pos_2d - cen_2d).normalized()
        dir_3d = rad_2d % tan_2d
        loiter_dir = -1.0 if dir_3d.z < 0.0 else 1.0
        return loiter_dir

    def _wp_gen_start_loiter(self, seq):
        """
        Create a waypoint for the start loiter.

        :param seq: mission item sequence assigned to this waypoint
        :type seq: int
        :return: A MAVLink mission item
        """
        path = self._candidate_path
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        if path is None or map_lat is None or map_lon is None:
            return

        terrain_module = self.module("terrain")
        if terrain_module is None:
            return None
        elevation_model = terrain_module.ElevationModel

        if self.start_latlon is None:
            return None
        (wp_lat, wp_lon) = self.start_latlon

        # calculate the loiter direction
        (start_x, start_y) = TerrainPlanner.latlon_to_enu(
            map_lat, map_lon, wp_lat, wp_lon
        )
        state = path.first_segment().first_state()
        loiter_dir_frd = -1.0 * TerrainNavModule.loiter_dir_enu(
            state.position, state.velocity, Vector3(start_x, start_y, 0.0)
        )

        ter_alt = elevation_model.GetElevation(wp_lat, wp_lon)
        wp_alt = ter_alt + self.terrainnav_settings.loiter_agl_alt
        p1 = 1.0  # heading required: 0: no, 1: yes
        p2 = loiter_dir_frd * self.terrainnav_settings.loiter_radius  # radius
        p3 = 0.0
        p4 = 1.0  # loiter exit location: 1: line between exit and next wp.
        mission_item = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=self.mpstate.settings.target_system,
            target_component=self.mpstate.settings.target_component,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
            current=0,
            autocontinue=1,
            param1=p1,
            param2=p2,
            param3=p3,
            param4=p4,
            x=wp_lat,
            y=wp_lon,
            z=wp_alt,
        )
        return mission_item

    def _wp_gen_goal_loiter(self, seq):
        """
        Create a waypoint for the goal loiter.

        :param seq: mission item sequence assigned to this waypoint
        :type seq: int
        :return: A MAVLink mission item
        """
        path = self._candidate_path
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        if path is None or map_lat is None or map_lon is None:
            return

        terrain_module = self.module("terrain")
        if terrain_module is None:
            return None
        elevation_model = terrain_module.ElevationModel

        if self.goal_latlon is None:
            return None
        (wp_lat, wp_lon) = self.goal_latlon

        # calculate the loiter direction
        (start_x, start_y) = TerrainPlanner.latlon_to_enu(
            map_lat, map_lon, wp_lat, wp_lon
        )
        state = path.last_segment().last_state()
        loiter_dir_frd = -1.0 * TerrainNavModule.loiter_dir_enu(
            state.position, state.velocity, Vector3(start_x, start_y, 0.0)
        )

        ter_alt = elevation_model.GetElevation(wp_lat, wp_lon)
        wp_alt = ter_alt + self.terrainnav_settings.loiter_agl_alt
        p1 = 0.0  # empty
        p2 = 0.0  # empty
        p3 = loiter_dir_frd * self.terrainnav_settings.loiter_radius  # radius
        p4 = float("nan")  # desired yaw angle
        mission_item = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=self.mpstate.settings.target_system,
            target_component=self.mpstate.settings.target_component,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            current=0,
            autocontinue=1,
            param1=p1,
            param2=p2,
            param3=p3,
            param4=p4,
            x=wp_lat,
            y=wp_lon,
            z=wp_alt,
        )
        return mission_item

    def _wp_gen_simple_waypoints(self):
        path = self._candidate_path
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        if path is None or map_lat is None or map_lon is None:
            return

        wp_module = self.module("wp")
        if wp_module is None:
            return

        home = wp_module.get_home()
        if home is None:
            return

        # target system and component for mission items
        sys_id = self.mpstate.settings.target_system
        cmp_id = self.mpstate.settings.target_component

        mission_items = []
        wp_num = 0

        add_home = True
        add_start_loiter = True
        add_goal_loiter = True

        if add_home:
            mission_item = self._wp_gen_home(wp_num, home)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        if add_start_loiter:
            mission_item = self._wp_gen_start_loiter(wp_num)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        # sample waypoints along the path
        wp_spacing = self.terrainnav_settings.wp_spacing
        wp_num_total = 0
        wp_positions = []
        for i, segment in enumerate(path._segments):
            count = segment.state_count()
            length = segment.get_length()
            # dt = segment.dt
            dt = length / count
            wp_num = max(int(length / wp_spacing), 1)
            stride = count // wp_num
            filtered_positions = segment.position()[::stride]

            # skip first point of next Dubins curve to avoid duplicates
            if (i % 3 == 0) and (i != 0):
                filtered_positions = filtered_positions[:-1]
            wp_positions.extend(filtered_positions)
            wp_num = len(filtered_positions)
            wp_num_total += wp_num
            if self.is_debug:
                print(
                    f"[terrainnav] "
                    f"segment[{i}]: count: {count}, length: {length:.2f}, dt: {dt:.2f}, "
                    f"wp_num: {wp_num}, wp_num_total: {wp_num_total}, stride: {stride}"
                )

        # convert positions [(east, north, alt)] to locations [(lat, lon, alt)]
        for i, pos in enumerate(wp_positions):
            east = pos.x
            north = pos.y
            wp_alt = pos.z
            (wp_lat, wp_lon) = mp_util.gps_offset(map_lat, map_lon, east, north)

            if self.is_debug and self.module("terrain") is not None:
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(wp_lat, wp_lon)
                agl_alt = wp_alt - ter_alt
                print(
                    f"[terrainnav] "
                    f"wp: {wp_num}, east: {east:.2f}, north: {north:.2f}, "
                    f"lat: {wp_lat:.6f}, lon: {wp_lon:.6f}, wp_alt: {wp_alt:.2f}, "
                    f"ter_alt: {ter_alt:.2f}, agl_alt: {agl_alt:.2f}"
                )

            pass_radius = 0.0

            # mission item MAV_CMD_NAV_WAYPOINT (16)
            p1 = 0.0  # hold
            p2 = 0.0  # accept radius
            p3 = 0.0  # pass_radius # pass radius - not working?
            p4 = 0.0  # end_yaw_deg # yaw at waypoint - not working?
            mission_item = mavutil.mavlink.MAVLink_mission_item_message(
                target_system=sys_id,
                target_component=cmp_id,
                seq=wp_num,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0,
                autocontinue=1,
                param1=p1,
                param2=p2,
                param3=p3,
                param4=p4,
                x=wp_lat,
                y=wp_lon,
                z=wp_alt,
            )
            mission_items.append(mission_item)
            wp_num += 1

        if add_goal_loiter:
            mission_item = self._wp_gen_goal_loiter(wp_num)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        # prepare waypoints for load
        wp_module.wploader.clear()
        wp_module.wploader.expected_count = len(mission_items)
        self.mpstate.master().waypoint_count_send(len(mission_items))
        for w in mission_items:
            wp_module.wploader.add(w)
            wsend = wp_module.wploader.wp(w.seq)
            if self.mpstate.settings.wp_use_mission_int:
                wsend = wp_module.wp_to_mission_item_int(w)
            self.mpstate.master().mav.send(wsend)

            # tell the wp module to expect some waypoints
            wp_module.loading_waypoints = True

    def _wp_gen_use_loiter_to_alt(self):
        path = self._candidate_path
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        if path is None or map_lat is None or map_lon is None:
            return

        wp_module = self.module("wp")
        if wp_module is None:
            return

        home = wp_module.get_home()
        if home is None:
            return

        # target system and component for mission items
        sys_id = self.mpstate.settings.target_system
        cmp_id = self.mpstate.settings.target_component

        mission_items = []
        wp_num = 0

        add_home = True
        add_start_loiter = True
        add_goal_loiter = True

        if add_home:
            mission_item = self._wp_gen_home(wp_num, home)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        if add_start_loiter:
            mission_item = self._wp_gen_start_loiter(wp_num)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        # add path mission items
        for i, segment in enumerate(path._segments):
            position3 = segment.first_state().position
            tangent3 = (segment.first_state().velocity).normalized()
            curvature = segment.curvature
            position2 = Vector3(position3.x, position3.y, 0.0)
            tangent2 = Vector3(tangent3.x, tangent3.y, 0.0)

            start_alt = segment.first_state().position.z

            (end_lat, end_lon) = mp_util.gps_offset(
                map_lat,
                map_lon,
                segment.last_state().position.x,
                segment.last_state().position.y,
            )
            end_alt = segment.last_state().position.z
            end_yaw_deg = math.degrees(segment.last_state().attitude.euler[2])

            segment_delta_alt = end_alt - start_alt
            segment_length = segment.get_length()
            segment_gamma = math.atan2(segment_delta_alt, segment_length)
            segment_gamma_deg = math.degrees(segment_gamma)

            if self.is_debug:
                print(
                    f"[terrainnav] "
                    f"[{wp_num}] delta_alt: {segment_delta_alt:.2f}, "
                    f"length: {segment_length:.2f}, gamma_deg: {segment_gamma_deg:.2f}"
                )

            pass_radius = 0.0
            if curvature != 0.0:
                # curvature in ENU, CCW is negative, CW is positive
                pass_radius = 1.0 if curvature < 0 else -1.0

                # do not add a loiter for short turns
                radius = 1.0 / math.fabs(curvature)
                phi = segment_length / radius
                min_phi = math.radians(self.terrainnav_settings.wp_min_loiter_angle_deg)

                if phi >= min_phi:
                    arc_centre = PathSegment.get_arc_centre(
                        position2, tangent2, curvature
                    )
                    (cen_lat, cen_lon) = mp_util.gps_offset(
                        map_lat,
                        map_lon,
                        arc_centre.x,
                        arc_centre.y,
                    )

                    # MAV_CMD_NAV_LOITER_TO_ALT (31)
                    p1 = 1.0  # heading required: 0: no, 1: yes
                    p2 = -1.0 / curvature  # radius: > 0 loiter CW, < 0 loiter CCW
                    p3 = 0.0
                    p4 = 1.0  # loiter exit location: 1: line between exit and next wp.
                    mission_item = mavutil.mavlink.MAVLink_mission_item_message(
                        target_system=sys_id,
                        target_component=cmp_id,
                        seq=wp_num,
                        frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                        command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
                        current=0,
                        autocontinue=1,
                        param1=p1,
                        param2=p2,
                        param3=p3,
                        param4=p4,
                        x=cen_lat,
                        y=cen_lon,
                        z=end_alt,
                    )
                    mission_items.append(mission_item)
                    wp_num += 1
            else:
                # mission item MAV_CMD_NAV_WAYPOINT (16)
                p1 = 0.0  # hold
                p2 = 0.0  # accept radius
                p3 = 0.0  # pass_radius # pass radius - not working?
                p4 = 0.0  # end_yaw_deg # yaw at waypoint - not working?
                mission_item = mavutil.mavlink.MAVLink_mission_item_message(
                    target_system=sys_id,
                    target_component=cmp_id,
                    seq=wp_num,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    current=0,
                    autocontinue=1,
                    param1=p1,
                    param2=p2,
                    param3=p3,
                    param4=p4,
                    x=end_lat,
                    y=end_lon,
                    z=end_alt,
                )
                mission_items.append(mission_item)
                wp_num += 1

        if add_goal_loiter:
            mission_item = self._wp_gen_goal_loiter(wp_num)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        # prepare waypoints for load
        wp_module.wploader.clear()
        wp_module.wploader.expected_count = len(mission_items)
        self.mpstate.master().waypoint_count_send(len(mission_items))
        for w in mission_items:
            wp_module.wploader.add(w)
            wsend = wp_module.wploader.wp(w.seq)
            if self.mpstate.settings.wp_use_mission_int:
                wsend = wp_module.wp_to_mission_item_int(w)
            self.mpstate.master().mav.send(wsend)

            # tell the wp module to expect some waypoints
            wp_module.loading_waypoints = True

    @property
    def is_debug(self):
        return self.mpstate.settings.moddebug > 1

    def check_reinit_fencepoints(self):
        # NOTE: see: mavproxy_map check_redisplay_fencepoints
        fence_module = self.module("fence")
        if fence_module is not None:
            if hasattr(fence_module, "last_change"):
                # new fence module
                last_change = fence_module.last_change()
            else:
                # old fence module
                last_change = fence_module.fenceloader.last_change
            if self._fence_change_time != last_change:
                self._fence_change_time = last_change
                # send polyfences to planner process
                msg = PlannerPolyFences()
                msg.exclusion_polygons = fence_module.exclusion_polygons()
                msg.inclusion_polygons = fence_module.inclusion_polygons()
                msg.exclusion_circles = fence_module.exclusion_circles()
                msg.inclusion_circles = fence_module.inclusion_circles()
                self._parent_pipe_send.send(msg)


# data messages [in]


class PlannerStartLatLon:
    def __init__(self, start_latlon, is_valid=False):
        self.start_latlon = start_latlon
        self.is_valid = is_valid


class PlannerGoalLatLon:
    def __init__(self, goal_latlon, is_valid=False):
        self.goal_latlon = goal_latlon
        self.is_valid = is_valid


class PlannerLoiterAglAlt:
    def __init__(self, loiter_agl_alt):
        self.loiter_agl_alt = loiter_agl_alt


class PlannerLoiterRadius:
    def __init__(self, loiter_radius):
        self.loiter_radius = loiter_radius


class PlannerTurningRadius:
    def __init__(self, turning_radius):
        self.turning_radius = turning_radius


class PlannerClimbAngleDeg:
    def __init__(self, climb_angle_deg):
        self.climb_angle_deg = climb_angle_deg


class PlannerMaxAglAlt:
    def __init__(self, max_agl_alt):
        self.max_agl_alt = max_agl_alt


class PlannerMinAglAlt:
    def __init__(self, min_agl_alt):
        self.min_agl_alt = min_agl_alt


class PlannerGridLatLon:
    def __init__(self, grid_latlon):
        self.grid_latlon = grid_latlon


class PlannerGridSpacing:
    def __init__(self, grid_spacing):
        self.grid_spacing = grid_spacing


class PlannerGridLength:
    def __init__(self, grid_length):
        self.grid_length = grid_length


class PlannerTimeBudget:
    def __init__(self, time_budget):
        self.time_budget = time_budget


class PlannerResolution:
    def __init__(self, resolution):
        self.resolution = resolution


class PlannerPolyFences:
    def __init__(self):
        self.exclusion_polygons = []
        self.inclusion_polygons = []
        self.exclusion_circles = []
        self.inclusion_circles = []


# data messages [out]


class PlannerStatus:
    def __init__(self, status):
        self.status = status


class PlannerPath:
    def __init__(self, path):
        self.path = path


class PlannerStates:
    def __init__(self, states):
        self.states = states


# command messages


class PlannerCmdRunPlanner:
    def __init__(self):
        pass


class TerrainPlanner(multiproc.Process):
    # NOTE: the planner process cannot be daemonic, because it prevents
    #       the terrain tile downloader from running (daemonic processes
    #        are not alloed to have children).
    def __init__(self, pipe_send, pipe_recv, close_event):
        super().__init__(name="TerrainPlanner", daemon=False)

        self._pipe_send = pipe_send
        self._pipe_recv = pipe_recv
        self._close_event = close_event

        # thread to process incoming messages
        self._message_thread = None
        self._lock = multiproc.Lock()

        # *** process state ***
        self._do_init_terrain_map = False
        self._do_init_planner = False
        self._do_update_start_pos = False
        self._do_update_goal_pos = False
        self._do_run_planner = False

        # *** planner settings ***
        self._loiter_agl_alt = 60.0
        self._loiter_radius = 60.0
        self._turning_radius = 60.0
        self._climb_angle_deg = 8.0
        self._max_agl_alt = 100.0
        self._min_agl_alt = 50.0
        self._grid_spacing = 30.0
        self._grid_length = 10000.0
        self._time_budget = 20.0
        self._resolution = 100.0

        # *** planner state ***
        self._start_latlon = (None, None)
        self._start_pos_enu = (None, None)
        self._start_is_valid = False
        self._goal_latlon = (None, None)
        self._goal_pos_enu = (None, None)
        self._goal_is_valid = False
        self._grid_map = None
        self._grid_map_lat = None
        self._grid_map_lon = None
        self._terrain_map = None
        self._da_space = None
        self._planner_mgr = None

        # *** fences ***
        self._exclusion_polygons = []
        self._inclusion_polygons = []
        self._exclusion_circles = []
        self._inclusion_circles = []

    def run(self):
        # start threads
        self.start_message_thread()

        # monitor events
        while True:
            # check for close event
            if self._close_event.is_set():
                break

            try:
                # copy shared state
                self._lock.acquire()
                do_init_terrain_map = self._do_init_terrain_map
                do_init_planner = self._do_init_planner
                do_update_start_pos = self._do_update_start_pos
                do_update_goal_pos = self._do_update_goal_pos
                do_run_planner = self._do_run_planner
                self._lock.release()

                # run planner operations
                if do_init_terrain_map:
                    self.init_terrain_map()

                    self._lock.acquire()
                    self._do_init_terrain_map = False
                    self._lock.release()

                if do_init_planner:
                    self.init_planner()

                    self._lock.acquire()
                    self._do_init_planner = False
                    self._lock.release()

                if do_update_start_pos:
                    self._lock.acquire()
                    (lat, lon) = self._start_latlon
                    self._lock.release()

                    self.set_start_pos_enu(lat, lon)

                    self._lock.acquire()
                    self._do_update_start_pos = False
                    self._lock.release()

                if do_update_goal_pos:
                    self._lock.acquire()
                    (lat, lon) = self._goal_latlon
                    self._lock.release()

                    self.set_goal_pos_enu(lat, lon)

                    self._lock.acquire()
                    self._do_update_goal_pos = False
                    self._lock.release()

                if do_run_planner:
                    self.run_planner()

                    self._lock.acquire()
                    self._do_run_planner = False
                    self._lock.release()

            except ValueError as e:
                print(f"[TerrainPlanner] {e}")
                self._lock.release()
            except Exception as e:
                print(f"[TerrainPlanner] exception in main loop, stopping: {e}")
                self._lock.release()
                break

            time.sleep(0.01)

    def start_message_thread(self):
        if self._message_thread:
            return

        t = threading.Thread(target=self.process_messages, name="MessageThread")
        t.daemon = True
        self._message_thread = t
        t.start()

    def process_messages(self):
        """
        Process incoming messages
        """
        while True:
            # receive data from parent process
            while self._pipe_recv.poll():
                msg = self._pipe_recv.recv()

                if isinstance(msg, PlannerStartLatLon):
                    self.on_start_lat_lon(msg)
                elif isinstance(msg, PlannerGoalLatLon):
                    self.on_goal_lat_lon(msg)
                elif isinstance(msg, PlannerLoiterAglAlt):
                    self.on_loiter_agl_alt(msg)
                elif isinstance(msg, PlannerLoiterRadius):
                    self.on_loiter_radius(msg)
                elif isinstance(msg, PlannerTurningRadius):
                    self.on_turning_radius(msg)
                elif isinstance(msg, PlannerClimbAngleDeg):
                    self.on_climb_angle_deg(msg)
                elif isinstance(msg, PlannerMaxAglAlt):
                    self.on_max_agl_alt(msg)
                elif isinstance(msg, PlannerMinAglAlt):
                    self.on_min_agl_alt(msg)
                elif isinstance(msg, PlannerGridLatLon):
                    self.on_grid_latlon(msg)
                elif isinstance(msg, PlannerGridSpacing):
                    self.on_grid_spacing(msg)
                elif isinstance(msg, PlannerGridLength):
                    self.on_grid_length(msg)
                elif isinstance(msg, PlannerTimeBudget):
                    self.on_time_budget(msg)
                elif isinstance(msg, PlannerResolution):
                    self.on_resolution(msg)
                elif isinstance(msg, PlannerPolyFences):
                    self.on_polyfences(msg)
                elif isinstance(msg, PlannerCmdRunPlanner):
                    self.on_cmd_run_planner(msg)

            # update at 100 Hz
            time.sleep(0.01)

    def on_start_lat_lon(self, msg):
        self._lock.acquire()
        self._start_latlon = msg.start_latlon
        self._do_update_start_pos = True
        self._lock.release()

    def on_goal_lat_lon(self, msg):
        self._lock.acquire()
        self._goal_latlon = msg.goal_latlon
        self._do_update_goal_pos = True
        self._lock.release()

    def on_loiter_agl_alt(self, msg):
        self._lock.acquire()
        self._loiter_agl_alt = msg.loiter_agl_alt
        self._do_update_start_pos = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_loiter_radius(self, msg):
        self._lock.acquire()
        self._loiter_radius = msg.loiter_radius
        self._do_init_planner = True
        self._do_update_start_pos = True
        self._lock.release()
        # TODO: recalculate start and goal positions

    def on_turning_radius(self, msg):
        self._lock.acquire()
        self._turning_radius = msg.turning_radius
        self._do_init_planner = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_climb_angle_deg(self, msg):
        self._lock.acquire()
        self._climb_angle_deg = msg.climb_angle_deg
        self._do_init_planner = True
        self._lock.release()

    def on_max_agl_alt(self, msg):
        self._lock.acquire()
        self._max_agl_alt = msg.max_agl_alt
        self._do_init_planner = True
        self._lock.release()

    def on_min_agl_alt(self, msg):
        self._lock.acquire()
        self._min_agl_alt = msg.min_agl_alt
        self._do_init_planner = True
        self._lock.release()

    def on_grid_latlon(self, msg):
        self._lock.acquire()
        self._grid_map_lat = msg.grid_latlon[0]
        self._grid_map_lon = msg.grid_latlon[1]
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._do_update_start_pos = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_grid_spacing(self, msg):
        self._lock.acquire()
        self._grid_spacing = msg.grid_spacing
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._lock.release()

    def on_grid_length(self, msg):
        self._lock.acquire()
        self._grid_length = msg.grid_length
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._lock.release()

    def on_time_budget(self, msg):
        self._lock.acquire()
        self._time_budget = msg.time_budget
        self._lock.release()

    def on_resolution(self, msg):
        self._lock.acquire()
        self._resolution = msg.resolution
        self._do_init_planner = True
        self._lock.release()

    def on_polyfences(self, msg):
        self._lock.acquire()
        self._exclusion_polygons = msg.exclusion_polygons
        self._inclusion_polygons = msg.inclusion_polygons
        self._exclusion_circles = msg.exclusion_circles
        self._inclusion_circles = msg.inclusion_circles
        self._do_init_planner = True
        self._lock.release()

    def on_cmd_run_planner(self, msg):
        self._lock.acquire()
        self._do_run_planner = True
        self._lock.release()

    def init_terrain_map(self):
        if not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        self._grid_map = GridMapSRTM(
            map_lat=self._grid_map_lat, map_lon=self._grid_map_lon
        )
        self._grid_map.setGridSpacing(self._grid_spacing)
        self._grid_map.setGridLength(self._grid_length)

        # TODO: set up distance layer (too slow in current version)
        # if self.is_debug:
        #     print(f"[TerrainPlanner] calculating distance-surface...", end="")
        # self._grid_map.addLayerDistanceTransform(surface_distance=self.terrainnav_settings.min_agl_alt)

        self._terrain_map = TerrainMap()
        self._terrain_map.setGridMap(self._grid_map)

        self._lock.release()

    def init_planner(self):
        # NOTE: initialisation ordering
        #
        # - create the state space
        # - set the map
        # - set altitude limits
        # - set bounds
        # - configureProblem:
        #   requires:
        #     - map
        #     - bounds (altitude limits)
        #   creates:
        #     - default planner
        #     - default objective
        #     - terrain collision validatity checker
        #     - planner data
        # - set start and goal states
        # - setup problem
        #   - (re-runs configureProblem internally)

        self._lock.acquire()

        # check the terrain map has been initialised
        if self._terrain_map is None:
            self._lock.release()
            return

        # recreate planner, as inputs may change
        self._da_space = DubinsAirplaneStateSpace(
            turningRadius=self._turning_radius,
            gam=math.radians(self._climb_angle_deg),
        )
        self._planner_mgr = TerrainOmplRrt(self._da_space)
        self._planner_mgr.setMap(self._terrain_map)
        self._planner_mgr.setAltitudeLimits(
            max_altitude=self._max_agl_alt,
            min_altitude=self._min_agl_alt,
        )
        self._planner_mgr.setBoundsFromMap(self._terrain_map.getGridMap())

        # run initial configuration so we can finish setting up fences etc.
        self._planner_mgr.configureProblem()

        # update problem
        problem = self._planner_mgr.getProblemSetup()

        # set fences - must called be after configureProblem
        problem.setExclusionPolygons(
            TerrainPlanner.polyfences_polygon_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._exclusion_polygons
            )
        )
        problem.setInclusionPolygons(
            TerrainPlanner.polyfences_polygon_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._inclusion_polygons
            )
        )
        problem.setExclusionCircles(
            TerrainPlanner.polyfences_circle_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._exclusion_circles
            )
        )
        problem.setInclusionCircles(
            TerrainPlanner.polyfences_circle_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._inclusion_circles
            )
        )

        # adjust validity checking resolution
        resolution_requested = self._resolution / self._grid_length
        problem.setStateValidityCheckingResolution(resolution_requested)

        self._lock.release()

    def run_planner(self):
        self._lock.acquire()

        # check start position is valid
        if not self._start_is_valid:
            msg = PlannerStatus(status="INVALID_START")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # check goal position is valid
        if not self._goal_is_valid:
            msg = PlannerStatus(status="INVALID_GOAL")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # set up problem and run
        self._planner_mgr.setupProblem2(
            self._start_pos_enu,
            self._goal_pos_enu,
            self._loiter_radius,
        )

        # run the solver
        candidate_path = Path()
        try:
            self._planner_mgr.Solve1(
                time_budget=self._time_budget,
                path=candidate_path,
            )
        except RuntimeError as e:
            # TODO: append error message
            msg = PlannerStatus(status="PLANNER_EXCEPTION")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # return if no solution
        if not self._planner_mgr.getProblemSetup().haveSolutionPath():
            msg = PlannerStatus(status="NO_SOLUTION")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # TODO: replace string with an enum class for planner status
        msg = PlannerStatus(status="OK")
        self._pipe_send.send(msg)

        # send path
        msg = PlannerPath(path=candidate_path)
        self._pipe_send.send(msg)

        # send states
        # NOTE: cannot send ompl states (cannot be pickled)
        solution_path = self._planner_mgr.getProblemSetup().getSolutionPath()
        ompl_states = solution_path.getStates()
        states = []
        for state in ompl_states:
            pos = TerrainOmplRrt.dubinsairplanePosition(state)
            yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
            states.append([pos[0], pos[1], pos[2], yaw])
        msg = PlannerStates(states=states)
        self._pipe_send.send(msg)

        self._lock.release()

    def have_gridmap_latlon(self):
        # check the grid lat and lon have been set
        self._lock.acquire()
        result = self._grid_map_lat is not None and self._grid_map_lon is not None
        self._lock.release()
        return result

    def set_start_pos_enu(self, lat, lon):
        if lat is None or lon is None or not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainPlanner.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        try:
            elevation = self._grid_map.atPosition("elevation", (east, north))
        except ValueError as e:
            print(f"[TerrainPlanner] unable to set start position: {e}")
            self._lock.release()
            return

        self._start_pos_enu = [
            east,
            north,
            elevation + self._loiter_agl_alt,
        ]

        # check valid
        radius = self._loiter_radius
        self._start_is_valid = self._planner_mgr.validateCircle(
            self._start_pos_enu, radius
        )

        # send validated position
        self._pipe_send.send(PlannerStartLatLon((lat, lon), self._start_is_valid))

        self._lock.release()

    def set_goal_pos_enu(self, lat, lon):
        if lat is None or lon is None or not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainPlanner.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        try:
            elevation = self._grid_map.atPosition("elevation", (east, north))
        except ValueError as e:
            print(f"[TerrainPlanner] unable to set goal position: {e}")
            self._lock.release()
            return

        self._goal_pos_enu = [
            east,
            north,
            elevation + self._loiter_agl_alt,
        ]

        # check valid
        radius = self._turning_radius
        self._goal_is_valid = self._planner_mgr.validateCircle(
            self._goal_pos_enu, radius
        )

        # send validated position
        self._pipe_send.send(PlannerGoalLatLon((lat, lon), self._goal_is_valid))

        self._lock.release()

    @staticmethod
    def latlon_to_enu(origin_lat, origin_lon, lat, lon):
        distance = mp_util.gps_distance(origin_lat, origin_lon, lat, lon)
        bearing_deg = mp_util.gps_bearing(origin_lat, origin_lon, lat, lon)
        bearing_rad = math.radians(bearing_deg)
        east = distance * math.sin(bearing_rad)
        north = distance * math.cos(bearing_rad)
        return (east, north)

    @staticmethod
    def polyfences_polygon_to_enu(origin_lat, origin_lon, polygons):
        """
        Convert polyfences polygones to ENU point polygons.

        :param origin_lat: latitude of the grid map origin
        :type origin_lat: float
        :param origin_lon: longitude of the grid map origin
        :type origin_lon: float
        :param polygons: list of MAVLink polyfences
        :return: list of polygons in ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        polygons_enu = []
        for polygon in polygons:
            points_enu = []
            for point in polygon:
                lat = point.x
                lon = point.y
                if point.get_type() == "MISSION_ITEM_INT":
                    lat *= 1e-7
                    lon *= 1e-7
                point_enu = TerrainPlanner.latlon_to_enu(
                    origin_lat, origin_lon, lat, lon
                )
                points_enu.append(point_enu)
            polygons_enu.append(points_enu)
        return polygons_enu

    @staticmethod
    def polyfences_circle_to_enu(origin_lat, origin_lon, circles):
        circles_enu = []
        for circle in circles:
            lat = circle.x
            lon = circle.y
            if circle.get_type() == "MISSION_ITEM_INT":
                lat *= 1e-7
                lon *= 1e-7
            (east, north) = TerrainPlanner.latlon_to_enu(
                origin_lat, origin_lon, lat, lon
            )
            radius = circle.param1
            circles_enu.append((east, north, radius))
        return circles_enu
