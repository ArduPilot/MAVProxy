#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
WebSocket Module
Jim Heising
August 2024
'''

from MAVProxy.modules.lib import mp_module, mp_settings
from websockets.sync.server import serve
from threading import Thread
from websockets import ConnectionClosedOK
from pymavlink import mavutil
import json
import traceback

mavlink_map = mavutil.mavlink.mavlink_map
message_name_2_type_cache = {}

# Some examples of JSON MAVLink messages
# {"mavpackettype": "COMMAND_LONG", "target_system":0, "target_component": 0, "command": 400, "confirmation": 0, "param1": 1, "param2": 0, "param3": 0, "param4": 0, "param5": 0, "param6": 0, "param7": 0}
# {"mavpackettype": "PARAM_REQUEST_LIST", "target_system":1, "target_component": 1}
# {"mavpackettype": "PARAM_REQUEST_READ", "target_system":1, "target_component": 1, "param_id": "", "param_index":1}

# Convert the name of a MAVLink message into its python class type
def get_mavlink_msg_type_from_name(name):
    # Check to see if this is cached already
    msg_type = message_name_2_type_cache.get(name)

    if msg_type:
        return msg_type

    for msg_id in mavlink_map.keys():
        msg_type = mavlink_map[msg_id]
        mav_cmd_name = msg_type.msgname if hasattr(msg_type, "msgname") else msg_type.name
        if mav_cmd_name == name:
            # Cache this for future use
            message_name_2_type_cache[name] = msg_type
            return msg_type


# Convert a JSON message into a proper MAVLink message
def json_to_mavlink(json_string):
    # Parse our JSON string
    mavlink_dict = json.loads(json_string)

    # Get the name of the MAVLink packet type
    requested_cmd_name = mavlink_dict["mavpackettype"]

    # Check to see if the message type is known
    msg_type = get_mavlink_msg_type_from_name(requested_cmd_name)

    if msg_type:
        # If we get here, we've found a mavlink message that matches our mavpackettype name
        # Now we can parse out our JSON attributes in their proper order create our MAVLink message
        msg_args = []
        for field_name in msg_type.fieldnames:
            field_value = mavlink_dict.get(field_name)
            # Convert strings to bytes
            if isinstance(field_value, str):
                field_value = field_value.encode("ascii")
            msg_args.append(field_value)

        # Here we actually create the actual MAVLink message from the python class
        return msg_type(*msg_args)


class WebSocket(mp_module.MPModule):
    def __init__(self, mpstate):
        super(WebSocket, self).__init__(mpstate, "ws", "WebSocket Module")
        self.ws_settings = mp_settings.MPSettings(
            [('port', int, 3000), ('api_key', str, None)])
        self.add_command('ws', self.cmd_ws, 'WebSocket control',
                         ["<start>",
                          "<stop>",
                          "set (WEBSOCKETSETTING)"])
        self.add_completion_function('(WEBSOCKETSETTING)',
                                     self.ws_settings.completion)

        self.run_thread = None
        self.ws_server = None
        self.ws_connections = []

    def process_ws_input_msg(self, message, websocket):
        links = self.mpstate.mav_master

        for link in links:
            try:
                ml_msg = None
                # Should we turn a JSON value into a MAVLink message?
                if websocket.request.path == "/json":
                    ml_msg = json_to_mavlink(message)
                # Or should we use the bytes as is?
                else:
                    # NOTE: This is not fully tested yet
                    ml_msg = link.mav.decode(str.encode(message))

                # Finally send our MAVLink message to our autopilot
                if ml_msg:
                    link.mav.send(ml_msg)
            except:
                traceback.print_exc()

    def ws_server_handler(self, websocket):
        # Check to see if we should authenticate an X-API-KEY header before allowing this connection
        if self.ws_settings.api_key:
            api_key = websocket.request.headers.get("X-API-KEY")
            if api_key != self.ws_settings.api_key:
                websocket.respond(401, "Unauthorized")
                websocket.close()
                return

        self.ws_connections.append(websocket)
        print("WebSocket client connected")
        while True:
            try:
                message = websocket.recv()
                self.process_ws_input_msg(message, websocket)
            except ConnectionClosedOK:
                print("WebSocket client disconnected")
                break

        self.ws_connections.remove(websocket)

    def ws_server_thread(self):
        self.cmd_stop()
        self.ws_server = serve(self.ws_server_handler, "", self.ws_settings.port)
        print("WebSocket server started on port %d" % self.ws_settings.port)
        self.ws_server.serve_forever()

    def cmd_ws(self, args):
        if len(args) <= 0:
            print("Usage: ws <start|stop|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.cmd_stop()
        elif args[0] == "set":
            self.ws_settings.command(args[1:])

    def cmd_start(self):
        self.run_thread = Thread(target=self.ws_server_thread)
        self.run_thread.start()

    def cmd_stop(self):
        self.ws_connections = []

        if self.ws_server:
            self.ws_server.shutdown()
            self.ws_server = None

    def unload(self):
        self.cmd_stop()

    def mavlink_packet(self, msg):
        json_msg = None
        raw_msg = None

        # Every time we get a MAVLink packet, we loop through all of our active websocket connections and send the results
        for connection in self.ws_connections:
            if connection.request.path == "/json":
                if json_msg is None:
                    try:
                        # Convert our MAVLink message into JSON
                        json_msg = json.dumps(msg.to_dict(), allow_nan=False, skipkeys=True)
                    except:
                        traceback.print_exc()
                        continue
                connection.send(json_msg)
            else:
                if raw_msg is None:
                    # Get the raw bytes
                    raw_msg = msg.get_msgbuf()
                connection.send(raw_msg)


def init(mpstate):
    return WebSocket(mpstate)
