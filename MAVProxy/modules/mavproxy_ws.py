#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
WebSocket Module
Jim Heising
Sept 2024
'''

from MAVProxy.modules.lib import mp_module, mp_settings
from websockets.sync.server import serve
from threading import Thread
from websockets import ConnectionClosedOK
from pymavlink import mavutil
import json

mavlink_map = mavutil.mavlink.mavlink_map

# {"mavpackettype": "COMMAND_LONG", "target_system":0, "target_component": 0, "command": 400, "confirmation": 0, "param1": 1, "param2": 0, "param3": 0, "param4": 0, "param5": 0, "param6": 0, "param7": 0}

def json_to_mavlink(json_string):
    # Parse our JSON string
    mavlink_dict = json.loads(json_string)

    # Get the name of the mav packet type
    requested_cmd_name = mavlink_dict["mavpackettype"]

    # Loop through each mavlink message type and find one that matches our name
    for msg_id in mavlink_map.keys():
        msg_type = mavlink_map[msg_id]
        mav_cmd_name = msg_type.msgname if hasattr(msg_type, "msgname") else msg_type.name
        if mav_cmd_name == requested_cmd_name:
            # If we get here, we've found a mavlink message that matches our mavpackettype name
            # Now we can parse out our JSON attributes in their proper order create our MAVLink message
            msg_args = []
            for field_name in msg_type.fieldnames:
                field_value = mavlink_dict.get(field_name)
                msg_args.append(field_value)
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

    def process_ws_in_msg(self, message, websocket):
        links = self.mpstate.mav_master

        for link in links:
            ml_msg = None
            if websocket.request.path == "/json":
                ml_msg = json_to_mavlink(message)
            else:
                ml_msg = link.mav.decode(str.encode(message))

            # Finally send our MAVLink message to our AutoPilot
            if ml_msg:
                link.mav.send(ml_msg)

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
                self.process_ws_in_msg(message, websocket)
            except ConnectionClosedOK:
                print("WebSocket client disconnected")
                break

        self.ws_connections.remove(websocket)

    def run_ws_server(self):
        self.cmd_stop()
        self.ws_server = serve(self.ws_server_handler, "", self.ws_settings.port)
        print("WebSocket server started on port %d" % self.ws_settings.port)
        self.ws_server.serve_forever()

    def cmd_start(self):
        self.run_thread = Thread(target=self.run_ws_server)
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

        for connection in self.ws_connections:
            if connection.request.path == "/json":
                if json_msg is None:
                    json_msg = msg.to_json()
                connection.send(json_msg)
            else:
                if raw_msg is None:
                    raw_msg = msg.get_msgbuf()
                connection.send(raw_msg)


def init(mpstate):
    return WebSocket(mpstate)
