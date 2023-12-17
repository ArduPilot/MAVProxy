'''
AI Chat Module OpenAi interface
Randy Mackay, December 2023

OpenAI Assistant API: https://platform.openai.com/docs/api-reference/assistants
OpenAI Assistant Playground: https://platform.openai.com/playground
MAVProxy chat wiki: https://ardupilot.org/mavproxy/docs/modules/chat.html
'''

from pymavlink import mavutil
import time, re
from datetime import datetime
import json
import math

try:
    from openai import OpenAI
except:
    print("chat: failed to import openai. See https://ardupilot.org/mavproxy/docs/modules/chat.html")
    exit()

class chat_openai():
    def __init__(self, mpstate, status_cb=None):
        # keep reference to mpstate
        self.mpstate = mpstate

        # keep reference to status callback
        self.status_cb = status_cb

        # initialise OpenAI connection
        self.client = None
        self.assistant = None
        self.assistant_thread = None

    # check connection to OpenAI assistant and connect if necessary
    # returns True if connection is good, False if not
    def check_connection(self):
        # create connection object
        if self.client is None:
            try:
                self.client = OpenAI()
            except:
                print("chat: failed to connect to OpenAI")
                return False

        # check connection again just to be sure
        if self.client is None:
            print("chat: failed to connect to OpenAI")
            return False

        # get assistant id
        if self.assistant is None:
            # get list of available assistants
            my_assistants = self.client.beta.assistants.list()
            if my_assistants is None or my_assistants.data is None or len(my_assistants.data) == 0:
                print("chat: no assistants available")
                return False

            # search for assistant with the expected name
            for existing_assistant in my_assistants.data:
                if existing_assistant.name == "ArduPilot Vehicle Control via MAVLink":
                    self.assistant = existing_assistant
                    break

            # raise error if assistant not found
            if self.assistant is None:
                print("chat: failed to connect to OpenAI assistant")
                return False

        # create new thread
        if self.assistant_thread is None:
            self.assistant_thread = self.client.beta.threads.create()
            if self.assistant_thread is None:
                return "chat: failed to create assistant thread"

        # if we got this far the connection must be good
        return True

    # set the OpenAI API key
    def set_api_key(self, api_key_str):
        self.client = OpenAI(api_key = api_key_str)
        self.assistant = None
        self.assistant_thread = None

    # send text to assistant
    def send_to_assistant(self, text):
        # check connection
        if not self.check_connection():
            return "chat: failed to connect to OpenAI"

        # create a new message
        input_message = self.client.beta.threads.messages.create(
            thread_id=self.assistant_thread.id,
            role="user",
            content=text
        )
        if input_message is None:
            return "chat: failed to create input message"

        # create a run
        self.run = self.client.beta.threads.runs.create(
            thread_id=self.assistant_thread.id,
            assistant_id=self.assistant.id
        )
        if self.run is None:
            return "chat: failed to create run"

        # wait for run to complete
        run_done = False
        while not run_done:
            # wait for one second
            time.sleep(0.1)

            # retrieve the run        
            latest_run = self.client.beta.threads.runs.retrieve(
                thread_id=self.assistant_thread.id,
                run_id=self.run.id
            )

            # check run status
            if latest_run.status in ["queued", "in_progress", "cancelling"]:
                run_done = False
            elif latest_run.status in ["cancelled", "failed", "completed", "expired"]:
                run_done = True
            elif latest_run.status in ["requires_action"]:
                self.handle_function_call(latest_run)
                run_done = False
            else:
                print("chat: unrecognised run status" + latest_run.status)
                run_done = True

            # send status to status callback
            self.send_status(latest_run.status)

        # retrieve messages on the thread
        reply_messages = self.client.beta.threads.messages.list(self.assistant_thread.id, order = "asc", after=input_message.id)
        if reply_messages is None:
            return "chat: failed to retrieve messages"

        # concatenate all messages into a single reply skipping the first which is our question
        reply = ""
        need_newline = False
        for message in reply_messages.data:
            reply = reply + message.content[0].text.value
            if need_newline:
                reply = reply + "\n"
            need_newline = True

        if reply is None or reply == "":
            return "chat: failed to retrieve latest reply"
        return reply

    # handle function call request from assistant
    # on success this returns the text response that should be sent to the assistant, returns None on failure
    def handle_function_call(self, run):

        # sanity check required action (this should never happen)
        if run.required_action is None:
            print("chat::handle_function_call: assistant function call empty")
            return None

        # check format
        if run.required_action.submit_tool_outputs is None:
            print("chat::handle_function_call: submit tools outputs empty")
            return None

        tool_outputs = []
        for tool_call in run.required_action.submit_tool_outputs.tool_calls:
            # init output to None
            output = "invalid function call"
            recognised_function = False

            # get current date and time
            if tool_call.function.name == "get_current_datetime":
                recognised_function = True
                output = self.get_formatted_date()

            # get vehicle type
            if tool_call.function.name == "get_vehicle_type":
                recognised_function = True
                output = json.dumps(self.get_vehicle_type())

            # get vehicle state including armed, mode
            if tool_call.function.name == "get_vehicle_state":
                recognised_function = True
                output = json.dumps(self.get_vehicle_state())

            # get vehicle location and yaw
            if tool_call.function.name == "get_vehicle_location_and_yaw":
                recognised_function = True
                output = json.dumps(self.get_vehicle_location_and_yaw())

            # get_location_plus_offset
            if tool_call.function.name == "get_location_plus_offset":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                except:
                    print("chat::handle_function_call: get_location_plus_offset: failed to parse arguments")
                    output = "get_location_plus_offset: failed to parse arguments"
                try:
                    output = json.dumps(self.get_location_plus_offset(arguments))
                except:
                    print("chat::handle_function_call: get_location_plus_offset: failed to calc location")
                    output = "get_location_plus_offset: failed to get location"

            # send mavlink command_int
            if tool_call.function.name == "send_mavlink_command_int":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.send_mavlink_command_int(arguments)
                except:
                    print("chat::handle_function_call: failed to parse arguments")

            # send mavlink set_position_target_global_int
            if tool_call.function.name == "send_mavlink_set_position_target_global_int":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.send_mavlink_set_position_target_global_int(arguments)
                except:
                    print("chat: send_mavlink_set_position_target_global_int: failed to parse arguments")


            # get a list of mavlink message names that can be retrieved using the get_mavlink_message function
            if tool_call.function.name == "get_available_mavlink_messages":
                recognised_function = True
                output = self.get_available_mavlink_messages()

            # get mavlink message from vehicle
            if tool_call.function.name == "get_mavlink_message":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.get_mavlink_message(arguments)
                except:
                    output = "get_mavlink_message: failed to retrieve message"
                    print("chat: get_mavlink_message: failed to retrieve message")

            # get all parameters from vehicle
            if tool_call.function.name == "get_all_parameters":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.get_all_parameters(arguments)
                except:
                    output = "get_all_parameters: failed to retrieve parameters"
                    print("chat: get_all_parameters: failed to retrieve parameters")

            # get a vehicle parameter's value
            if tool_call.function.name == "get_parameter":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.get_parameter(arguments)
                except:
                    output = "get_parameter: failed to retrieve parameter value"
                    print("chat: get_parameters: failed to retrieve parameter value")

            # set a vehicle parameter's value
            if tool_call.function.name == "set_parameter":
                recognised_function = True
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    output = self.set_parameter(arguments)
                except:
                    output = "set_parameter: failed to set parameter value"
                    print("chat: set_parameter: failed to set parameter value")

            if not recognised_function:
                print("chat: handle_function_call: unrecognised function call: " + tool_call.function.name)
                output = "unrecognised function call: " + tool_call.function.name

            # append output to list of outputs
            tool_outputs.append({"tool_call_id": tool_call.id, "output": output})

        # send function replies to assistant
        try:
            run_reply = self.client.beta.threads.runs.submit_tool_outputs(
                thread_id=run.thread_id,
                run_id=run.id,
                tool_outputs=tool_outputs
            )
        except:
            print("chat: error replying to function call")
            print(tool_outputs)

    # get the current date and time in the format, Saturday, June 24, 2023 6:14:14 PM
    def get_formatted_date(self):
        return datetime.now().strftime("%A, %B %d, %Y %I:%M:%S %p")

    # get vehicle vehicle type (e.g. "Copter", "Plane", "Rover", "Boat", etc)
    def get_vehicle_type(self):
        # get vehicle type from latest HEARTBEAT message
        hearbeat_msg = self.mpstate.master().messages.get('HEARTBEAT', None)
        vehicle_type_str = "unknown"
        if hearbeat_msg is not None:
            if hearbeat_msg.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                     mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
                                     mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
                                     mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR]:
                vehicle_type_str = 'Plane'
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
                vehicle_type_str = 'Rover'
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
                vehicle_type_str = 'Boat'
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
                vehicle_type_str = 'Sub'
            if hearbeat_msg.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                     mavutil.mavlink.MAV_TYPE_COAXIAL,
                                     mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                     mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                     mavutil.mavlink.MAV_TYPE_TRICOPTER,
                                     mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
                    vehicle_type_str = "Copter"
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_HELICOPTER:
                vehicle_type_str = "Heli"
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
                vehicle_type_str = "Tracker"
            if hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_AIRSHIP:
                vehicle_type_str = "Blimp"
        return {
            "vehicle_type": vehicle_type_str
        }

    # get vehicle state including armed, mode
    def get_vehicle_state(self):
        # get mode from latest HEARTBEAT message
        hearbeat_msg = self.mpstate.master().messages.get('HEARTBEAT', None)
        if hearbeat_msg is None:
            mode_number = 0
            print ("chat: get_vehicle_state: vehicle mode is unknown")
        else:
            mode_number = hearbeat_msg.custom_mode
        return {
            "armed": (self.mpstate.master().motors_armed() > 0),
            "mode": mode_number
        }

    # return the vehicle's location and yaw
    def get_vehicle_location_and_yaw(self):
        lat_deg = 0
        lon_deg = 0
        alt_amsl_m = 0
        alt_rel_m = 0
        yaw_deg = 0
        gpi = self.mpstate.master().messages.get('GLOBAL_POSITION_INT', None)
        if gpi:
            lat_deg = gpi.lat * 1e-7,
            lon_deg = gpi.lon * 1e-7,
            alt_amsl_m = gpi.alt * 1e-3,
            alt_rel_m = gpi.relative_alt * 1e-3
            yaw_deg = gpi.hdg * 1e-2
        location = {
            "latitude": lat_deg,
            "longitude": lon_deg,
            "altitude_amsl": alt_amsl_m,
            "altitude_above_home": alt_rel_m,
            "yaw" : yaw_deg
        }
        return location

    # Calculate the latitude and longitude given distances (in meters) North and East
    def get_location_plus_offset(self, arguments):
        lat = arguments.get("latitude", 0)
        lon = arguments.get("longitude", 0)
        dist_north = arguments.get("distance_north", 0)
        dist_east = arguments.get("distance_east", 0)
        lat_lon_to_meters_scaling = 89.8320495336892 * 1e-7
        lat_diff = dist_north * lat_lon_to_meters_scaling
        lon_diff = dist_east * lat_lon_to_meters_scaling / max(0.01, math.cos(math.radians((lat+lat_diff)/2)))
        return {
            "latitude": self.wrap_latitude(lat + lat_diff),
            "longitude": self.wrap_longitude(lon + lon_diff)
        }

    # send a mavlink command_int message to the vehicle
    def send_mavlink_command_int(self, arguments):
        target_system = arguments.get("target_system", 1)
        target_component = arguments.get("target_component", 1)
        frame = arguments.get("frame", 0)
        if ("command" not in arguments):
            return "command_int not sent.  command field required"
        command = arguments.get("command", 0)
        current = arguments.get("current", 0)
        autocontinue = arguments.get("autocontinue", 0)
        param1 = arguments.get("param1", 0)
        param2 = arguments.get("param2", 0)
        param3 = arguments.get("param3", 0)
        param4 = arguments.get("param4", 0)
        x = arguments.get("x", 0)
        y = arguments.get("y", 0)
        z = arguments.get("z", 0)
        self.mpstate.master().mav.command_int_send(target_system, target_component, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)
        return "command_int sent"

    # send a mavlink send_mavlink_set_position_target_global_int message to the vehicle
    def send_mavlink_set_position_target_global_int(self, arguments):
        if arguments is None:
            return "send_mavlink_set_position_target_global_int: arguments is None"
        time_boot_ms = arguments.get("time_boot_ms", 0)
        target_system = arguments.get("target_system", 1)
        target_component = arguments.get("target_component", 1)
        coordinate_frame = arguments.get("coordinate_frame", 5)
        type_mask = arguments.get("type_mask", 0)
        lat_int = arguments.get("lat_int", 0)
        lon_int = arguments.get("lon_int", 0)
        alt = arguments.get("alt", 0)
        vx = arguments.get("vx", 0)
        vy = arguments.get("vy", 0)
        vz = arguments.get("vz", 0)
        afx = arguments.get("afx", 0)
        afy = arguments.get("afy", 0)
        afz = arguments.get("afz", 0)
        yaw = arguments.get("yaw", 0)
        yaw_rate = arguments.get("yaw_rate", 0)
        self.mpstate.master().mav.set_position_target_global_int_send(time_boot_ms, target_system, target_component, coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate)
        return "set_position_target_global_int sent"

    # get a list of mavlink message names that can be retrieved using the get_mavlink_message function
    def get_available_mavlink_messages(self):
        # check if no messages available
        if self.mpstate.master().messages is None or len(self.mpstate.master().messages) == 0:
            return "get_available_mavlink_messages: no messages available"

        # retrieve each available message's name
        mav_msg_names = []
        for msg in self.mpstate.master().messages:
            # append all message names except MAV
            if msg != "MAV":
                mav_msg_names.append(msg)

        # return list of message names
        try:
            return json.dumps(mav_msg_names)
        except:
            return "get_available_mavlink_messages: failed to convert message name list to json"

    # get a mavlink message including all fields and values sent by the vehicle
    def get_mavlink_message(self, arguments):
        if arguments is None:
            return "get_mavlink_message: arguments is None"

        # retrieve requested message's name
        mav_msg_name = arguments.get("message", None)
        if mav_msg_name is None:
            return "get_mavlink_message: message not specified"

        # retrieve message
        mav_msg = self.mpstate.master().messages.get(mav_msg_name, None)
        if mav_msg is None:
            return "get_mavlink_message: message not found"

        # convert message to json
        try:
            return json.dumps(mav_msg.to_dict())
        except:
            return "get_mavlink_message: failed to convert message to json"

    # get all available parameters names and their values
    def get_all_parameters(self, arguments):
        # check if any parameters are available
        if self.mpstate.mav_param is None or len(self.mpstate.mav_param) == 0:
            return "get_all_parameters: no parameters are available"
        param_list = {}
        for param_name in sorted(self.mpstate.mav_param.keys()):
            param_list[param_name] = self.mpstate.mav_param.get(param_name)
        try:
            return json.dumps(param_list)
        except:
            return "get_all_parameters: failed to convert parameter list to json"

    # get a vehicle parameter's value
    def get_parameter(self, arguments):
        param_name = arguments.get("name", None)
        if param_name is None:
            print("get_parameter: name not specified")
            return "get_parameter: name not specified"

        # start with empty parameter list
        param_list = {}

        # handle param name containing regex
        if self.contains_regex(param_name):
            pattern = re.compile(param_name)
            for existing_param_name in sorted(self.mpstate.mav_param.keys()):
                if pattern.match(existing_param_name) is not None:
                    param_value = self.mpstate.functions.get_mav_param(existing_param_name, None)
                    if param_value is None:
                        print("chat: get_parameter unable to get " + existing_param_name)
                    else:
                        param_list[existing_param_name] = param_value
        else:
            # handle simple case of a single parameter name
            param_value = self.mpstate.functions.get_mav_param(param_name, None)
            if param_value is None:
                return "get_parameter: " + param_name + " parameter not found"
            param_list[param_name] = param_value

        try:
            return json.dumps(param_list)
        except:
            return "get_parameter: failed to convert parameter list to json"

    # set a vehicle parameter's value
    def set_parameter(self, arguments):
        param_name = arguments.get("name", None)
        if param_name is None:
            return "set_parameter: parameter name not specified"
        param_value = arguments.get("value", None)
        if param_value is None:
            return "set_parameter: value not specified"
        self.mpstate.functions.param_set(param_name, param_value, retries=3)
        return "set_parameter: parameter value set"

    # wrap latitude to range -90 to 90
    def wrap_latitude(self, latitude_deg):
        if latitude_deg > 90:
            return 180 - latitude_deg
        if latitude_deg < -90:
            return -(180 + latitude_deg)
        return latitude_deg
    
    # wrap longitude to range -180 to 180
    def wrap_longitude(self, longitude_deg):
        if longitude_deg > 180:
            return longitude_deg - 360
        if longitude_deg < -180:
            return longitude_deg + 360
        return longitude_deg

    # send status to chat window via callback
    def send_status(self, status):
        if self.status_cb is not None:
            self.status_cb(status)

    # returns true if string contains regex characters
    def contains_regex(self, string):
        regex_characters = ".^$*+?{}[]\|()"
        for x in regex_characters:
            if string.count(x):
                return True
        return False
