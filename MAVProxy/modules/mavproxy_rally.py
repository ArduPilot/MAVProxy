'''rally point handling via MissionItemProtocol'''

from MAVProxy.modules.lib import mission_item_protocol

from pymavlink import mavutil
from pymavlink import mavwp

from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem


class RallyModule(mission_item_protocol.MissionItemProtocolModule):

    def __init__(self, mpstate):
        '''initialise module'''
        super(RallyModule, self).__init__(
            mpstate,
            "rally",
            "rally point management",
            public=True)

    def command_name(self):
        '''command-line command name'''
        return "rally"

    def cmd_rally_add(self, args):
        '''add a rally point at the last map click position'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        if len(args) < 1:
            alt = self.settings.rallyalt
        else:
            alt = float(args[0])

        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,    # frame
            mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,    # command
            0,    # current
            0,    # autocontinue
            0.0,  # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            alt,                   # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def rally_point(self, i):
        '''return an instance of the old mavlink rally_point message for the
        item at offset i'''
        i = self.wploader.item(i)
        if i is None:
            return None
        lat = i.x
        lng = i.y
        alt = i.z
        if i.get_type() == "MISSION_ITEM":
            lat *= 1e7
            lng *= 1e7
            alt *= 100
        return mavutil.mavlink.MAVLink_rally_point_message(
            i.target_system,
            i.target_component,
            i.seq,
            self.wploader.count(),
            lat,
            lng,
            alt,
            0,
            0,
            0)

    def rally_count(self):
        '''return number of waypoints'''
        return self.wploader.count()

    def commands(self):
        '''returns map from command name to handling function'''
        ret = super(RallyModule, self).commands()
        ret.update({
            'add': self.cmd_rally_add,
            "move": self.cmd_move,  # handled in parent class
            "changealt": self.cmd_changealt,
            "undo": self.cmd_undo,
        })
        return ret

    def mission_ftp_name(self):
        return "@MISSION/rally.dat"

    @staticmethod
    def loader_class():
        return mavwp.MissionItemProtocol_Rally

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_RALLY

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'rally items'

    def itemtype(self):
        '''returns description of item'''
        return 'rally item'

    def mavlink_packet(self, p):
        super(RallyModule, self).mavlink_packet(p)

    def gui_menu_items(self):
        ret = super(RallyModule, self).gui_menu_items()
        ret.extend([
            MPMenuItem(
                'Add', 'Add', '# rally add ',
                handler=MPMenuCallTextDialog(
                    title='Rally Altitude (m)',
                    default=100
                )
            ),
        ])
        return ret


def init(mpstate):
    '''initialise module'''

    # see if pymavlink is new enough to support new protocols:
    oldmodule = "rallypoint_protocol"
    try:
        mavwp.MissionItemProtocol_Rally
    except AttributeError:
        print("pymavlink too old; using old %s module" % oldmodule)
        mpstate.load_module(oldmodule)
        for (m, pm) in mpstate.modules:
            if m.name == "rally":
                return m
        return None

    return RallyModule(mpstate)
