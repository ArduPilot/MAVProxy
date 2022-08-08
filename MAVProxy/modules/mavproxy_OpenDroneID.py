'''
Support for OpenDroneID
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import time

class OpenDroneIDModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(OpenDroneIDModule, self).__init__(mpstate, "OpenDroneID", "OpenDroneID Support", public = True)
        self.add_command('opendroneid', self.cmd_opendroneid, "opendroneid control",
                         ["<status>", "set (OPENDRONEIDSETTING)"])

        self.OpenDroneID_settings = mp_settings.MPSettings([
            ("rate_hz", float, 0.2),
            # BASIC_ID
            ("UAS_ID_type", int, 0),
            ("UAS_ID", str, ""),
            ("UA_type", int, 0),
            # SELF_ID
            ("description_type", int, 0),
            ("description", str, ""),
            # SYSTEM
            ("area_count", int, 1),
            ("area_radius", int, 0),
            ("area_ceiling", int, -1000),
            ("area_floor", int, -1000),
            ("category_eu", int, 0),
            ("class_eu", int, 0),
            ("classification_type", int, 0),
            # OPERATOR_ID
            ("operator_location_type", int, 0),
            ("operator_id_type", int, 0),
            ("operator_id", str, ""),
            ])
        self.add_completion_function('(OPENDRONEIDSETTING)',
                                     self.OpenDroneID_settings.completion)
        self.last_send_s = time.time()
        self.next_msg = 0
        self.operator_latitude = 0
        self.operator_longitude = 0
        self.operator_altitude_geo = 0

    def cmd_opendroneid(self, args):
        '''opendroneid command parser'''
        usage = "usage: opendroneid <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.OpenDroneID_settings.command(args[1:])
        else:
            print(usage)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        pass

    def send_basic_id(self):
        '''send BASIC_ID'''
        self.master.mav.open_drone_id_basic_id_send(
            self.target_system,
            self.target_component,
            self.id_or_mac(),
            self.OpenDroneID_settings.UAS_ID_type,
            self.OpenDroneID_settings.UA_type,
            self.to_bytes(self.OpenDroneID_settings.UAS_ID, 20))

    def send_system(self):
        '''send SYSTEM'''
        # allow use of fakegps module for testing
        fakegps = self.module("fakegps")
        if fakegps is not None:
            (self.operator_latitude, self.operator_longitude, self.operator_altitude_geo) = fakegps.get_location()

        self.master.mav.open_drone_id_system_send(
            self.target_system,
            self.target_component,
            self.id_or_mac(),
            self.OpenDroneID_settings.operator_location_type,
            self.OpenDroneID_settings.classification_type,
            int(self.operator_latitude*1.0e7),
            int(self.operator_longitude*1.0e7),
            self.OpenDroneID_settings.area_count,
            self.OpenDroneID_settings.area_radius,
            self.OpenDroneID_settings.area_ceiling,
            self.OpenDroneID_settings.area_floor,
            self.OpenDroneID_settings.category_eu,
            self.OpenDroneID_settings.class_eu,
            self.operator_altitude_geo,
            self.timestamp_2019())

    def send_self_id(self):
        '''send SELF_ID'''
        self.master.mav.open_drone_id_self_id_send(
            self.target_system,
            self.target_component,
            self.id_or_mac(),
            self.OpenDroneID_settings.description_type,
            self.to_string(self.OpenDroneID_settings.description, 23))

    def send_operator_id(self):
        '''send OPERATOR_ID'''
        self.master.mav.open_drone_id_operator_id_send(
            self.target_system,
            self.target_component,
            self.id_or_mac(),
            self.OpenDroneID_settings.operator_id_type,
            self.to_string(self.OpenDroneID_settings.operator_id, 20))

    def to_string(self, s, maxlen):
        return s.encode("utf-8")

    def to_bytes(self, s, maxlen):
        b = bytearray(s.encode("utf-8"))
        b = b[:maxlen]
        if len(b) < maxlen:
            b.extend(bytearray([0]*(maxlen-len(b))))
        return b

    def id_or_mac(self):
        return self.to_bytes("", 20)

    def timestamp_2019(self):
        jan_1_2019_s = 1546261200
        return int(time.time() + jan_1_2019_s)

    def idle_task(self):
        '''called on idle'''
        now = time.time()
        if now - self.last_send_s > (1.0/self.OpenDroneID_settings.rate_hz)/4:
            self.last_send_s = now
            if self.next_msg == 0:
                self.send_basic_id()
            elif self.next_msg == 1:
                self.send_system()
            elif self.next_msg == 2:
                self.send_self_id()
            elif self.next_msg == 3:
                self.send_operator_id()
            self.next_msg = (self.next_msg + 1) % 4

def init(mpstate):
    '''initialise module'''
    return OpenDroneIDModule(mpstate)
