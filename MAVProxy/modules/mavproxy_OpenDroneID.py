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

        from MAVProxy.modules.lib.mp_settings import MPSetting
        self.OpenDroneID_settings = mp_settings.MPSettings([
            MPSetting("rate_hz", float, 0.1),
            MPSetting("location_rate_hz", float, 1.0),
            # BASIC_ID
            MPSetting("UAS_ID_type", int, 0, choice=[("None",0), ("SerialNumber",1), ("CAA", 2), ("UTM_ASSIGNED", 3), ("SessionID", 4)]),
            MPSetting("UAS_ID", str, ""),
            MPSetting("UA_type", int, 0, choice=[("None",0), ("Aeroplane",1), ("HeliOrMulti",2),
                                            ("GyroPlane",3), ("HybridLift",4), ("Ornithopter",5),
                                            ("Glider",6), ("Kite",7), ("FreeBalloon",8), ("CaptiveBalloon",9),
                                            ("Airship", 10), ("Parachute",11), ("Rocket",12),
                                            ("TetheredPowered", 13), ("GroundObstacle", 14)]),
            # SELF_ID
            MPSetting("description_type", int, 0, choice=[("Text",0), ("Emergency",1), ("ExtendedStatus", 2)]),
            MPSetting("description", str, ""),
            # SYSTEM
            MPSetting("area_count", int, 1),
            MPSetting("area_radius", int, 0),
            MPSetting("area_ceiling", int, -1000),
            MPSetting("area_floor", int, -1000),
            MPSetting("category_eu", int, 0, choice=[("Undeclared",0), ("Open",1), ("Specific",2), ("Certified",3)]),
            MPSetting("class_eu", int, 0),
            MPSetting("classification_type", int, 0, choice=[("Undeclared",0),("EU",1)]),
            # OPERATOR_ID
            MPSetting("operator_location_type", int, 0, choice=[("Takeoff",0),("LiveGNSS",1),("Fixed",2)]),
            MPSetting("operator_id_type", int, 0),
            MPSetting("operator_id", str, ""),
            ])
        self.add_completion_function('(OPENDRONEIDSETTING)',
                                     self.OpenDroneID_settings.completion)
        self.last_send_s = time.time()
        self.last_loc_send_s = time.time()
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
        if self.mpstate.position is not None:
            pos = self.mpstate.position
            if pos.latitude is not None:
                self.operator_latitude = pos.latitude
                self.operator_longitude = pos.longitude
            if pos.altitude is not None:
                self.operator_altitude_geo = pos.altitude

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

    def send_system_update(self):
        '''send SYSTEM_UPDATE'''
        if self.mpstate.position is not None:
            pos = self.mpstate.position
            if pos.latitude is not None:
                self.operator_latitude = pos.latitude
                self.operator_longitude = pos.longitude
            if pos.altitude is not None:
                self.operator_altitude_geo = pos.altitude

        self.master.mav.open_drone_id_system_update_send(
            self.target_system,
            self.target_component,
            int(self.operator_latitude*1.0e7),
            int(self.operator_longitude*1.0e7),
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
        return int(time.time() - jan_1_2019_s)

    def idle_task(self):
        '''called on idle'''
        now = time.time()
        if now - self.last_loc_send_s > 1.0/self.OpenDroneID_settings.location_rate_hz:
            self.last_loc_send_s = now
            self.send_system_update()
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
