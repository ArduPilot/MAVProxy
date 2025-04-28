"""
Shearwater SmartFlight module

./Tools/autotest/sim_vehicle.py --debug -v plane --console --map --custom-location="21.306831, -157.945131, 5, 90" --no-configure --no-rebuild --mavcesium
"""

import json
import math
import requests
import os

from enum import Enum
from pathlib import Path
from typing import List, Optional
from dataclasses import dataclass, asdict
from dotenv import load_dotenv
from uuid import UUID, uuid4

from pymavlink import mavutil
from MAVProxy.mavproxy import MPState
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util


if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_menu import MPMenuItem

load_dotenv()  # take environment variables

SAMPLE_REQ_FILE = Path(__file__).parent / "sample_request.json"

URL = "https://api.shearwater.ai/api/planning"
KEY = os.environ["SMARTFLIGHT_BEARER_TOKEN"]
# These match colors from https://planning.shearwater.ai/
RGB_OPERATION_AREA = (46, 139, 87) # The planner's computational search area, "Sea Green"
RGB_EXCLUSION = (178, 34, 34) # Exclusions (similar to fence excludes), "Fire Brick"
RGB_PLAN_FASTEST = (255, 215, 0) # Shortest time to completion, "Gold"
RGB_PLAN_SHORTEST = (111, 177, 244) # Shortest distance, "Maya Blue"
RGB_PLAN_EFFICIENT = (124, 252, 0)  # Least energy, "Lawn Green"


class PathType(Enum):
    DISTANCE = 1
    TRAVELTIME = 2
    ENERGY = 3

PATH_COLORS = {
    PathType.DISTANCE: RGB_PLAN_SHORTEST,
    PathType.TRAVELTIME: RGB_PLAN_FASTEST,
    PathType.ENERGY: RGB_PLAN_EFFICIENT,
}

@dataclass
class Point3D:
    lon: float # rad
    lat: float # rad
    alt: float 

@dataclass
class AltitudeLimits:
    minAgl: float
    maxAgl: float
    maxAmsl: float


@dataclass
class Vehicle:
    id: str
    cruiseSpeed: float
    maxFlightSpeed: float
    minFlightSpeed: float
    maxClimbRate: float
    maxDescentRate: float

@dataclass
class Coordinate:
    latitude: float # rad
    longitude: float # rad
    height: float

class SmartFlightPlannerModule(mp_module.MPModule):

    @staticmethod
    def default_settings():
        return mp_settings.MPSettings(
            [
                MPSetting("alt_limit.min_agl", int, 75),
                MPSetting("alt_limit.max_agl", int, 250),
                MPSetting("alt_limit.max_amsl", int, 2000),
                MPSetting("loiter_radius", float, 60.0),  # WP_LOITER_RADIUS, only used for visualization
            ]
        )
    def __init__(self, mpstate: MPState) -> None:
        super().__init__(mpstate, "smartflight", "SmartFlight path planning module")
        self._key: str = KEY
        self.cache_dir: Path = Path("planning_cache")
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # *** planner settings ***
        self.smartflight_planner_settings = SmartFlightPlannerModule.default_settings()

        self._uuid: Optional[UUID] = None
        self._timestamp: Optional[str] = None
        self._missionArea: List[Point3D] = []
        self._keepOutAreas: List[List[Point3D]] = []
        self._altitudeLimits: Optional[AltitudeLimits] = AltitudeLimits(
            self.smartflight_planner_settings.get("alt_limit.min_agl"),
            self.smartflight_planner_settings.get("alt_limit.max_agl"),
            self.smartflight_planner_settings.get("alt_limit.max_amsl"))
        self._vehicle: Optional[Vehicle] = None
        self._departure: Optional[Coordinate] = None
        self._destination: Optional[Coordinate] = None
        self._departureTime: Optional[int] = None
        self._plan = None # The large JSON plan response object

        self.cmdname = "smartflight"

        # *** slip map state ***
        self._map_layer_initialised = False
        self._map_layer_id = self.cmdname
        self._map_boundary_id = f"{self.cmdname} boundary"
        self._map_start_id = f"{self.cmdname} start"
        self._map_goal_id = f"{self.cmdname} goal"
        self._is_boundary_visible = False
        self._is_start_visible = False
        self._is_goal_visible = False
        self._map_circle_linewidth = 2
        # SmartFlight supports multiple path types, each of which can be displayed with a different ID/color
        self._map_path_ids: dict[PathType, str] = {}
        for path_type in PathType:
            self._map_path_ids[path_type] = f"{self.cmdname} path {path_type.name.lower()}"

        # If the user changes the start, goal, or planner settings, we need to recompute the goal with a new UUID.
        self._needs_planning = False

        # TODO move from JSON initial plan to querying UAV params and using inclusion 
        # fence as operational area.
        self.req_from_json(SAMPLE_REQ_FILE)

        # ** CLI interface **
        self.add_command(
            self.cmdname,
            self.cmd_smartflight,
            f"{self.cmdname} control",
            [
                "set (SMARTFLIGHT_SETTING)",
                "show (SMARTFLIGHT_SHOW)",
                "compute",
                "set_start",
                "set_goal",
                "draw (SMARTFLIGHT_DRAW)",
                "execute (SMARTFLIGHT_EXECUTE)",
                # TODO add "info" to get the plan metadata (distance, time to completion)
            ],
        )
        self.add_completion_function(
            "(SMARTFLIGHT_SETTING)", self.smartflight_planner_settings.completion
        )
        self.add_completion_function(
            "(SMARTFLIGHT_SHOW)", self.show_completion
        )
        self.add_completion_function(
            "(SMARTFLIGHT_DRAW)", self.draw_completion
        )

        # add a sub-menu to map and console
        if mp_util.has_wxpython:
            map_menu = MPMenuSubMenu(
                "SmartFlight",
                items=[
                    MPMenuSubMenu('Show', [
                        MPMenuItem('Show Boundary', 'Show SmartFlight Planner Boundary', f'# {self.cmdname} show boundary'),
                        MPMenuItem('Show Start', 'Show SmartFlight Planner Start Circle', f'# {self.cmdname} show start'),
                        MPMenuItem('Show Goal', 'Show SmartFlight Planner Destination Circle', f'# {self.cmdname} show goal'),
                        MPMenuItem('Show Plans', 'Show SmartFlight path plan', f'# {self.cmdname} show plan'),
                    ]),
                    MPMenuSubMenu('Hide', [
                        MPMenuItem('Hide Boundary', 'Hide SmartFlight Planner Boundary', f'# {self.cmdname} hide boundary'),
                        MPMenuItem('Hide Start', 'Hide SmartFlight Planner Start Circle', f'# {self.cmdname} hide start'),
                        MPMenuItem('Hide Goal', 'Hide SmartFlight Planner Destination Circle', f'# {self.cmdname} hide goal'),
                        MPMenuItem('Hide Plans', 'Hide SmartFlight path plan', f'# {self.cmdname} hide plan'),
                    ]),
                    MPMenuSubMenu('Set', [
                        MPMenuItem('Set Start', 'Set the SmartFlight start location', f'# {self.cmdname} set_start'),
                        MPMenuItem('Set Goal', 'Set the SmartFlight end location', f'# {self.cmdname} set_goal'),
                    ]),
                    MPMenuItem('Draw Boundary', 'Set the SmartFlight operational area', f'# {self.cmdname} draw boundary'),
                ],
            )

            console_menu = MPMenuSubMenu(
                "SmartFlight",
                items=[
                    MPMenuItem('Compute Plan', 'Compute the route from the start to goal', f'# {self.cmdname} compute'),
                    MPMenuSubMenu('Execute', [
                        MPMenuItem('Distance Optimized Plan', 'Fly the distance optimized path plan', f'# {self.cmdname} execute distance-optimized'),
                        MPMenuItem('Time Optimized Plan', 'Fly the quickest time optimized path plan', f'# {self.cmdname} execute time-optimized'),
                        MPMenuItem('Energy Optimized Plan', 'Fly the energy optimized path plan', f'# {self.cmdname} execute energy-optimized'),
                    ])
                ],
            )

            map_module = self.module("map")
            if map_module is not None:
                map_module.add_menu(map_menu)

            console_module = self.module("console")
            if console_module is not None:
                console_module.add_menu(console_menu)
        
        self.smartflight_planner_settings.set_callback(self.setting_callback)

    def show_completion(self, _):
        return ["boundary", "start", "goal", "plan"]

    def draw_completion(self, _):
        return ["boundary"]
    
    def execute_completion(self, _):
        return ["distance-optimized", "time-optimized", "energy-optimized"]

    def setting_callback(self, setting: MPSetting):
        """
        Called when a setting is updated

        :param setting: the setting that has changed
        :type setting: MPSetting
        """

        if setting.name == "alt_limit.min_agl":
            self.altitude_limits.minAgl = setting.value
        elif setting.name == "alt_limit.max_agl":
            self.altitude_limits.maxAgl = setting.value
        elif setting.name == "alt_limit.max_amsl":
            self.altitude_limits.maxAmsl = setting.value
        else:
            raise NotImplementedError
        
        self._needs_planning = True

    def cmd_smartflight(self, args):
        """
        smartflight commands
        """
        usage = f"usage: {self.cmdname} <set|show|compute|execute|set_start|set_goal>"
        if len(args) < 1:
            print(usage)
        elif args[0] == "set":
            self.cmd_set(args[1:])
        elif args[0] == "show":
            self.cmd_show(args[1:])
        elif args[0] == "compute":
            self.cmd_compute()
        elif args[0] == "execute":
            self.cmd_execute(args[1:])
        elif args[0] == "set_start":
            self.set_start()
        elif args[0] == "set_goal":
            self.set_goal()
        elif args[0] == "draw":
            self.cmd_draw(args[1:])
        else:
            print(usage)

    def cmd_set(self, args):
        """
        Modify a setting
        """
        self.smartflight_planner_settings.command(args)

    def cmd_show(self, args):
        if not args:
            print(f"Usage: {self.cmdname} show <boundary|start|goal|plan>")
            return

        if args[0] == "boundary":
            self.show_planner_boundary()
        elif args[0] == "start":
            self.show_start()
        elif args[0] == "goal":
            self.show_goal()
        elif args[0] == "plan":
            if len(args) == 1:
                self.show_plan(["all"])
                return
            self.show_plan(args[1:])
        else:
            print(f"Unknown thing to show: {args[0]}")

    def cmd_compute(self):
        if self._needs_planning:
            self.uuid = uuid4()
            print(f"Created new plan with UUID: {self.uuid}")
        self._plan = self.post_plan() # TODO parse return

        if len(self._plan["paths"]) == 0:
            notifications = self._plan["notifications"]
            warnings = notifications["warnings"]
            if warnings is not None:
                for warning in warnings:
                    print(f"Warning: {warning}")
            cautions = notifications["cautions"]
            if cautions is not None:
                for caution in cautions:
                    print(f"Caution: {caution}")
            advisories = notifications["advisories"]
            if advisories is not None:
                for advisory in advisories:
                    print(f"Advisory: {advisory}")
        else:
            print("Plan computed!")
        self._needs_planning = False

    def cmd_execute(self, args):
        usage = f"Usage: {self.cmdname} execute <distance-optimized|energy-optimized|time-optimized>"
        if not args:
            print(usage)
            return
        
        if args[0] == "distance-optimized":
            self.execute_plan(PathType.DISTANCE)
        elif args[0] == "energy-optimized":
            self.execute_plan(PathType.ENERGY)
        elif args[0] == "time-optimized":
            self.execute_plan(PathType.TRAVELTIME)
        else:
            print(f"Unknown execution strategy: '{args[0]}'")
            print(usage)
            raise NotImplementedError

    def execute_plan(self, plan_type: PathType):
        if self._plan is None:
            print("Can't execute. The plan is not yet computed.")
            return
        elif self._needs_planning:
            print("Planner parameters have changed. The plan needs to be recomputed.")
            return

        self._wp_gen_simple_waypoints(plan_type)
        if self.mpstate.status.armed:
            self.master.set_mode("AUTO")
        else:
            print("Can't execute. Need to be armed.")

    def _wp_gen_simple_waypoints(self, plan_type: PathType):

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
        add_start_loiter = False
        add_goal_loiter = False

        if add_home:
            mission_item = self._wp_gen_home(wp_num, home)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        if add_start_loiter:
            raise NotImplementedError
            mission_item = self._wp_gen_start_loiter(wp_num)
            if mission_item is not None:
                mission_items.append(mission_item)
                wp_num += 1

        # Convert SmartFlight plan into MAVLink mission items
        if plan_type == PathType.DISTANCE:
            plan_type = "distance"
        elif plan_type == PathType.ENERGY:
            plan_type ="energy"
        elif plan_type == PathType.TRAVELTIME:
            plan_type = "travelTime"
        else:
            raise NotImplementedError
        
        print(f"Selecting path type optimized for {plan_type}")
        
        sf_wps = next(path["flightRoute"]["waypoints"] for path in self._plan["paths"] if path["type"] == plan_type)
        
        for wp in sf_wps:
            sf_coord = wp["coord"] # radians, x,y,z order

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
                x=math.degrees(sf_coord[1]),
                y=math.degrees(sf_coord[0]),
                z=sf_coord[2],
            )
            mission_items.append(mission_item)
            wp_num += 1

        if add_goal_loiter:
            raise NotImplementedError
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

    def show_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        self.draw_planner_boundary()
        self._is_boundary_visible = True

    def show_start(self):
        map_module = self.module("map")
        if map_module is None:
            return

        self.draw_start()
        self._is_start_visible = True

    def show_goal(self):
        map_module = self.module("map")
        if map_module is None:
            return

        self.draw_goal()
        self._is_goal_visible = True

    def show_plan(self, args):
        if self._plan is None:
            print("Compute the plan before you try to show it")
            return
        
        for plan_type in args:
            if plan_type == "all":
                for path in self._plan["paths"]:
                    path_type = path['type']
                    self.draw_path(path["flightRoute"]["waypoints"], PathType[path_type.upper()])
            else:
                try:
                    wps = next(path["flightRoute"]["waypoints"] for path in self._plan["paths"] if path["type"] == plan_type)
                except StopIteration:
                    print(f"You can only show plans of type {[path['type'] for path in self._plan['paths']]}")
                    return
                self.draw_path(wps, PathType[plan_type.upper()])

    def draw_path(self, waypoints: list[dict], path_type: PathType):
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        polygon = []
        for waypoint in waypoints:
            lon_rad, lat_rad, alt = waypoint["coord"]
            polygon.append((math.degrees(lat_rad), math.degrees(lon_rad)))

        if len(polygon) > 1:
            colour = PATH_COLORS[path_type]
            slip_polygon = mp_slipmap.SlipPolygon(
                key=self._map_path_ids[path_type],
                points=polygon,
                layer=self._map_layer_id,
                linewidth=2,
                colour=colour,
                showcircles=False,
            )
            map_module.map.add_object(slip_polygon)

    def hide_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_boundary_id)
        self._is_boundary_visible = False

    def draw_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        polygon = [(math.degrees(v.lat), math.degrees(v.lon)) for v in self.mission_area]
        
        if len(polygon) > 1:
            slip_polygon = mp_slipmap.UnclosedSlipPolygon(
                self._map_boundary_id,
                polygon,
                layer=self._map_layer_id,
                linewidth=5,
                colour=RGB_OPERATION_AREA,
                showcircles=False,
                showlines=True,
            )
            map_module.map.add_object(slip_polygon)

    def draw_start(self):
        radius = self.smartflight_planner_settings.loiter_radius
        colour = (0, 255, 0)
        self.draw_circle(self._map_start_id, math.degrees(self._departure.latitude), math.degrees(self._departure.longitude), radius, colour)

    def draw_goal(self):

        # TODO handle self._destination being None

        radius = self.smartflight_planner_settings.loiter_radius
        colour = (0, 255, 0)
        self.draw_circle(self._map_goal_id, math.degrees(self._destination.latitude), math.degrees(self._destination.longitude), radius, colour)

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

    def get_map_click_location(self):
        map_module = self.module("map")
        if map_module is None:
            return (None, None)

        return map_module.mpstate.click_location

    def set_start(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return
        
        self.departure.latitude = math.radians(lat)
        self.departure.longitude = math.radians(lon)

        self.draw_start()
        self._needs_planning = True

    def set_goal(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return
        
        self.destination.latitude = math.radians(lat)
        self.destination.longitude = math.radians(lon)

        self.draw_goal()
        self._needs_planning = True

    def boundary_draw_callback(self, points):
        # TODO determine what the planner expects in the alt field.
        alt = 0.0
        self.mission_area = [Point3D(math.radians(p[1]), math.radians(p[0]), alt) for p in points]
        self.show_planner_boundary()
        self._needs_planning = True

    def cmd_draw(self, args):
        usage = f"usage: {self.cmdname} draw <boundary>"
        if len(args) != 1:
            print(usage)
            return
        
        if 'draw_lines' not in self.mpstate.map_functions:
            print("No map drawing available")
            return

        if args[0] == "boundary":
            print("Drawing boundary on map")

            self.mpstate.map_functions['draw_lines'](self.boundary_draw_callback,
                                        colour=RGB_OPERATION_AREA)

    @property
    def uuid(self) -> Optional[UUID]:
        return self._uuid

    @uuid.setter
    def uuid(self, value: UUID) -> None:
        if isinstance(value, UUID):
            self._uuid = value
        else:
            raise ValueError(value)

    @property
    def timestamp(self) -> Optional[str]:
        return self._timestamp

    @timestamp.setter
    def timestamp(self, timestamp: str):
        self._timestamp = timestamp

    @property
    def mission_area(self) -> Optional[List[Point3D]]:
        return self._missionArea

    @mission_area.setter
    def mission_area(self, points: List[Point3D]):
        self._missionArea = points

    @property
    def keep_out_areas(self) -> Optional[List[List[Point3D]]]:
        return self._keepOutAreas

    @keep_out_areas.setter
    def keep_out_areas(self, areas: List[List[Point3D]]):
        self._keepOutAreas = areas

    @property
    def altitude_limits(self) -> AltitudeLimits:
        return self._altitudeLimits

    @altitude_limits.setter
    def altitude_limits(self, limits: AltitudeLimits):
        self._altitudeLimits = limits

    @property
    def vehicle(self)-> Vehicle:
        return self._vehicle
    
    @vehicle.setter
    def vehicle(self, vehicle: Vehicle):
        self._vehicle = vehicle

    @property
    def departure(self) -> Coordinate:
        return self._departure

    @departure.setter
    def departure(self, departure: Coordinate):
        self._departure = departure

    @property
    def destination(self) -> Coordinate:
        return self._destination

    @destination.setter
    def destination(self, destination: Coordinate):
        self._destination = destination

    @property
    def departure_time(self) -> int:
        return self._departureTime

    @departure_time.setter
    def departure_time(self, departure_time: int):
        self._departureTime = departure_time

    def _serialize(self) -> dict:
        return {
            "uuid": str(self._uuid),
            "timestamp": self._timestamp,
            "missionArea": [[p.lon, p.lat, p.alt] for p in self._missionArea],
            "keepOutAreas": [[[p.lon, p.lat, p.alt] for p in area] for area in self._keepOutAreas],
            "altitudeLimits": asdict(self._altitudeLimits) if self._altitudeLimits else None,
            "vehicle": asdict(self._vehicle) if self._vehicle else None,
            "departure": asdict(self._departure) if self._departure else None,
            "destination": asdict(self._destination) if self._destination else None,
            "departureTime": self._departureTime
        }

    def _get_cache_path(self) -> Path:
        if not self.uuid:
            raise ValueError("UUID must be set before accessing cache path")
        return self.cache_dir / f"{self.uuid}.json"

    def load_from_cache(self) -> Optional[dict]:
        cache_path = self._get_cache_path()
        if cache_path.exists():
            print(f"✅ Loaded cached result for UUID {self.uuid}")
            return json.loads(cache_path.read_text())
        return None

    def save_to_cache(self, response_json: dict):
        cache_path = self._get_cache_path()
        cache_path.write_text(json.dumps(response_json, indent=2))
        print(f"💾 Saved response to {cache_path}")

    def post_plan(self) -> Optional[dict]:
        if not self.uuid:
            raise ValueError("UUID must be set before posting plan")

        cached = self.load_from_cache()
        if cached:
            return cached

        headers = {
            'Content-Type': 'application/json',
            'Authorization': self._key
        }

        data = self._serialize()
        print("📡 Sending request to server...")
        # TODO kick this off into a separate thread, prevent multiple requests before completion.
        response = requests.post(URL, headers=headers, data=json.dumps(data))

        if response.status_code in (200, 202):
            result = response.json()
            self.save_to_cache(result)
            return result
        else:
            print(f"❌ Error from server: {response.status_code}")
            print(response.text)
            return None
    
    def _load_json_from_file(self, filepath: Path):
        return json.loads(filepath.read_text())
        
    def _load_from_dict(self, req: dict) -> None:
        self.uuid = UUID(req["uuid"])
        self.timestamp = req["timestamp"]
        self.mission_area = [Point3D(*pt) for pt in req["missionArea"]]
        self.keep_out_areas = req["keepOutAreas"]
        self.altitude_limits = AltitudeLimits(**req["altitudeLimits"])
        self.vehicle = Vehicle(**req["vehicle"])
        self.departure = Coordinate(**req["departure"])
        self.destination = Coordinate(**req["destination"])
        self.departure_time = req["departureTime"]
        
    def req_from_json(self, filepath: Path) -> None:
        if (data := self._load_json_from_file(filepath)) != None:
            print(f"DATA: {data}")
            self._load_from_dict(data)
        else:
            print(f"Can't load {filepath}")
