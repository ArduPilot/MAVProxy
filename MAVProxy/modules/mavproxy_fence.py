"""
    MAVProxy geofence module
"""
import copy
import time

from pymavlink import mavextra
from pymavlink import mavutil
from pymavlink import mavwp

from MAVProxy.modules.lib import mission_item_protocol
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem


class FenceModule(mission_item_protocol.MissionItemProtocolModule):
    '''uses common MISSION_ITEM protocol base class to provide fence
    upload/download
    '''

    def __init__(self, mpstate):
        super(FenceModule, self).__init__(mpstate, "fence", "fence point management (new)", public=True)
        self.present = False
        self.enabled = False
        self.healthy = True

    def gui_menu_items(self):
        ret = super(FenceModule, self).gui_menu_items()
        ret.extend([
            MPMenuItem(
                'Add Inclusion Circle', 'Add Inclusion Circle', '# fence addcircle inc ',
                handler=MPMenuCallTextDialog(
                    title='Radius (m)',
                    default=500
                )
            ),
            MPMenuItem(
                'Add Exclusion Circle', 'Add Exclusion Circle', '# fence addcircle exc ',
                handler=MPMenuCallTextDialog(
                    title='Radius (m)',
                    default=500
                )
            ),
            MPMenuItem(
                'Draw Inclusion Poly', 'Draw Inclusion Poly', '# fence draw inc ',
            ),
            MPMenuItem(
                'Draw Exclusion Poly', 'Draw Exclusion Poly', '# fence draw exc ',
            ),
            MPMenuItem(
                'Add Return Point', 'Add Return Point', '# fence addreturnpoint',
            ),
        ])
        return ret

    def command_name(self):
        '''command-line command name'''
        return "fence"

    # def fence_point(self, i):
    #     return self.wploader.fence_point(i)

    def count(self):
        '''return number of waypoints'''
        return self.wploader.count()

    def circles_of_type(self, t):
        '''return a list of Circle fences of a specific type - a single
        MISSION_ITEM'''
        ret = []
        loader = self.wploader
        for i in range(0, loader.count()):
            p = loader.item(i)
            if p is None:
                print("Bad loader item (%u)" % i)
                return []
            if p.command != t:
                continue
            ret.append(p)
        return ret

    def inclusion_circles(self):
        '''return a list of Circle inclusion fences - a single MISSION_ITEM each'''
        return self.circles_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION)

    def exclusion_circles(self):
        '''return a list of Circle exclusion fences - a single MISSION_ITEM each'''
        return self.circles_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)

    def polygons_of_type(self, t):
        '''return a list of polygon fences of a specific type - each a list of
        items'''
        ret = []
        loader = self.wploader
        state_outside = 99
        state_inside = 98
        current_polygon = []
        current_expected_length = 0
        state = state_outside
        for i in range(0, loader.count()):
            p = loader.item(i)
            if p is None:
                print("Bad loader item (%u)" % i)
                return []
            if p.command == t:
                # sanity checks:
                if p.param1 < 3:
                    print("Bad vertex count (%u) in seq=%u" % (p.param1, p.seq))
                    continue
                if state == state_outside:
                    # starting a new polygon
                    state = state_inside
                    current_expected_length = p.param1
                    current_polygon = []
                    current_polygon.append(p)
                    continue
                if state == state_inside:
                    # if the count is different and we're in state
                    # inside then the current polygon is invalid.
                    # Discard it.
                    if p.param1 != current_expected_length:
                        print("Short polygon found, discarding")
                        current_expected_length = p.param1
                        current_polygon = []
                        current_polygon.append(p)
                        continue
                    current_polygon.append(p)
                    if len(current_polygon) == current_expected_length:
                        ret.append(current_polygon)
                        state = state_outside
                    continue
                print("Unknown state (%s)" % str(state))
            else:
                if state == state_inside:
                    if len(current_polygon) != current_expected_length:
                        print("Short polygon found")
                    else:
                        ret.append(current_polygon)
                    state = state_outside
                    continue
                if state == state_outside:
                    continue
                print("Unknown state (%s)" % str(state))
        return ret

    def inclusion_polygons(self):
        '''return a list of polygon inclusion fences - each a list of items'''
        return self.polygons_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

    def exclusion_polygons(self):
        '''return a list of polygon exclusion fences - each a list of items'''
        return self.polygons_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)

    def returnpoint(self):
        '''return a return point if one exists'''
        loader = self.wploader
        for i in range(0, loader.count()):
            p = loader.item(i)
            if p is None:
                print("Bad loader item (%u)" % i)
                return []
            if p.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                continue
            return p
        return None

    def mission_ftp_name(self):
        return "@MISSION/fence.dat"

    @staticmethod
    def loader_class():
        return mavwp.MissionItemProtocol_Fence

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_FENCE

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'fence items'

    def itemtype(self):
        '''returns description of item'''
        return 'fence item'

    def handle_sys_status(self, m):
        '''function to handle SYS_STATUS packets, used by both old and new module'''
        bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        present = ((m.onboard_control_sensors_present & bits) == bits)
        if self.present is False and present is True:
            self.say("fence present")
        elif self.present is True and present is False:
            self.say("fence removed")
        self.present = present

        enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
        if self.enabled is False and enabled is True:
            self.say("fence enabled")
        elif self.enabled is True and enabled is False:
            self.say("fence disabled")
        self.enabled = enabled

        healthy = ((m.onboard_control_sensors_health & bits) == bits)
        if self.healthy is False and healthy is True:
            self.say("fence OK")
        elif self.healthy is True and healthy is False:
            self.say("fence breach")
        self.healthy = healthy

        # console output for fence:
        if not self.present:
            self.console.set_status('Fence', 'FEN', row=0, fg='black')
        elif self.enabled is False:
            self.console.set_status('Fence', 'FEN', row=0, fg='grey')
        elif self.enabled is True and self.healthy is True:
            self.console.set_status('Fence', 'FEN', row=0, fg='green')
        elif self.enabled is True and self.healthy is False:
            self.console.set_status('Fence', 'FEN', row=0, fg='red')

    def mavlink_packet(self, m):
        if m.get_type() == 'SYS_STATUS' and self.message_is_from_primary_vehicle(m):
            self.handle_sys_status(m)
        super(FenceModule, self).mavlink_packet(m)

    def fence_draw_callback(self, points):
        '''callback from drawing a fence'''
        if len(points) < 3:
            print("Fence draw cancelled")
            return
        items = []
        for p in points:
            m = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
                self.drawing_fence_type,    # command
                0,    # current
                0,    # autocontinue
                len(points), # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                int(p[0]*1e7),  # x (latitude)
                int(p[1]*1e7),  # y (longitude)
                0,                     # z (altitude)
                self.mav_mission_type(),
            )
            items.append(m)

        self.append(items)
        self.send_all_items()
        self.wploader.last_change = time.time()

    def cmd_draw(self, args):
        '''convenience / compatability / slow learner command to work like the
        old module - i.e. a single inclusion polyfence'''
        # TODO: emit this only if there actually are complex fences:
        if not self.check_have_list():
            return

        if len(args) == 0:
            print("WARNING!  You want 'fence draw inc' or 'fence draw exc'")
            return
        if args[0] in ("inc", "inclusion"):
            self.drawing_fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            draw_colour = (128, 128, 255)
        elif args[0] in ("exc", "exclusion"):
            self.drawing_fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
            draw_colour = (255, 128, 128)
        else:
            print("fence draw <inc|inclusion|exc|exclusion>")
            return

        if 'draw_lines' not in self.mpstate.map_functions:
            print("No map drawing available")
            return

        self.mpstate.map_functions['draw_lines'](self.fence_draw_callback,
                                                 colour=draw_colour)
        print("Drawing fence on map")

    def cmd_addcircle(self, args):
        '''adds a circle to the map click position of specific type/radius'''
        if not self.check_have_list():
            return
        if len(args) < 2:
            print("Need 2 arguments")
            return
        t = args[0]
        radius = float(args[1])

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        if t in ["inclusion", "inc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        elif t in ["exclusion", "exc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        else:
            print("%s is not one of inclusion|exclusion" % t)
            return

        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
            command,    # command
            0,    # current
            0,    # autocontinue
            radius, # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            0,                     # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def cmd_addreturnpoint(self, args):
        '''adds a returnpoint at the map click location'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
            mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,    # command
            0,    # current
            0,    # autocontinue
            0, # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            0,                     # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def cmd_addpoly(self, args):
        '''adds a number of waypoints equally spaced around a circle around
        click point

        '''
        if not self.check_have_list():
            return
        if len(args) < 1:
            print("Need at least 1 argument (<inclusion|inc|exclusion|exc>", "<radius>" "<pointcount>", "<rotation>")
            return
        t = args[0]
        count = 4
        radius = 20
        rotation = 0
        if len(args) > 1:
            radius = float(args[1])
        if len(args) > 2:
            count = int(args[2])
        if len(args) > 3:
            rotation = float(args[3])

        if count < 3:
            print("Invalid count (%s)" % str(count))
            return
        if radius <= 0:
            print("Invalid radius (%s)" % str(radius))
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        if t in ["inclusion", "inc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        elif t in ["exclusion", "exc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        else:
            print("%s is not one of inclusion|exclusion" % t)
            return

        items = []
        for i in range(0, count):
            (lat, lon) = mavextra.gps_newpos(latlon[0],
                                             latlon[1],
                                             360/float(count)*i + rotation,
                                             radius)

            m = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
                command,    # command
                0,    # current
                0,    # autocontinue
                count, # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                int(lat*1e7),  # x (latitude)
                int(lon*1e7),  # y (longitude)
                0,                     # z (altitude)
                self.mav_mission_type(),
            )
            items.append(m)

        for m in items:
            self.append(m)
        self.send_all_items()

    def cmd_remove(self, args):
        '''deny remove on fence - requires renumbering etc etc'''
        print("remove is not currently supported for fence.  Try removepolygon_point or removecircle")
        if not self.check_have_list():
            return

    def removereturnpoint(self, seq):
        '''remove returnpoint at offset seq'''
        if not self.check_have_list():
            return
        item = self.wploader.item(seq)
        if item is None:
            print("No item %s" % str(seq))
            return

        if item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
            print("Item %u is not a return point" % seq)
            return
        self.wploader.remove(item)
        self.wploader.expected_count -= 1
        self.wploader.last_change = time.time()
        self.send_all_items()

    def removecircle(self, seq):
        '''remove circle at offset seq'''
        if not self.check_have_list():
            return
        item = self.wploader.item(seq)
        if item is None:
            print("No item %s" % str(seq))
            return

        if not self.is_circle_item(item):
            print("Item %u is not a circle" % seq)
            return
        self.wploader.remove(item)
        self.wploader.expected_count -= 1
        self.wploader.last_change = time.time()
        self.send_all_items()

    def is_circle_item(self, item):
        return item.command in [
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
        ]

    def find_polygon_point(self, polygon_start_seq, item_offset):
        '''find polygon point selected on map
        returns first_item, item_offset
        '''
        first_item = self.wploader.item(polygon_start_seq)
        if first_item is None:
            print("No item at %u" % polygon_start_seq)
            return None, None
        if not self.is_polygon_item(first_item):
            print("Item %u is not a polygon vertex" % polygon_start_seq)
            return None, None
        original_count = int(first_item.param1)
        if item_offset == original_count:
            # selecting closing point on polygon selects first point
            item_offset = 0
        if item_offset > original_count:
            print("Out-of-range point")
            return None, None

        return first_item, item_offset
    
    def removepolygon_point(self, polygon_start_seq, item_offset):
        '''removes item at offset item_offset from the polygon starting at
        polygon_start_seq'''
        if not self.check_have_list():
            return

        items_to_set = []

        first_item, item_offset = self.find_polygon_point(polygon_start_seq, item_offset)
        if first_item is None:
            return
        original_count = int(first_item.param1)
        if original_count <= 3:
            print("Too few points to remove one")
            return

        dead_item_walking = self.wploader.item(polygon_start_seq + item_offset)

        # must reduce count in each of the polygons:
        for i in range(int(first_item.param1)):
            item = self.wploader.item(polygon_start_seq+i)
            if int(item.param1) != original_count:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (polygon_start_seq, original_count, i, int(item.param1)))
                return
            item.param1 = item.param1 - 1
            items_to_set.append(item)

        for item in items_to_set:
            w = item
            self.wploader.set(w, w.seq)

        self.wploader.remove(dead_item_walking)
        self.wploader.expected_count -= 1
        self.wploader.last_change = time.time()
        self.send_all_items()

    def addpolygon_point(self, polygon_start_seq, item_offset):
        '''adds item at offset item_offset into the polygon starting at
        polygon_start_seq'''

        if not self.check_have_list():
            return

        items_to_set = []

        first_item = self.wploader.item(polygon_start_seq)
        if (first_item.command not in [
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        ]):
            print("Item %u is not a polygon vertex" % polygon_start_seq)
            return
        original_count = int(first_item.param1)
        if item_offset >= original_count:
            print("Out-of-range point")
            return

        # increase count in each of the polygon vertexes:
        for i in range(int(first_item.param1)):
            item = self.wploader.item(polygon_start_seq+i)
            if int(item.param1) != original_count:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (polygon_start_seq, original_count, i, int(item.param1)))
                return
            item.param1 = item.param1 + 1
            items_to_set.append(item)

        for item in items_to_set:
            w = item
            self.wploader.set(w, w.seq)

        old_item = self.wploader.item(polygon_start_seq + item_offset)
        new_item = copy.copy(old_item)
        # reset latitude and longitude of new item to be half-way
        # between it and the preceeding point
        if item_offset == 0:
            prev_item_offset = original_count-1
        else:
            prev_item_offset = item_offset - 1
        prev_item = self.wploader.item(polygon_start_seq + prev_item_offset)
        new_item.x = (old_item.x + prev_item.x)/2
        new_item.y = (old_item.y + prev_item.y)/2
        self.wploader.insert(polygon_start_seq + item_offset, new_item)
        self.wploader.expected_count += 1
        self.wploader.last_change = time.time()
        self.send_all_items()

    def is_polygon_item(self, item):
        return item.command in [
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
        ]

    def cmd_removepolygon(self, args):
        self.removepolygon(int(args[0]))

    def removepolygon(self, seq):
        '''remove polygon at offset seq'''
        if not self.check_have_list():
            return

        first_item = self.wploader.item(seq)
        if first_item is None:
            print("Invalid item sequence number (%s)" % seq)
            return
        if not self.is_polygon_item(first_item):
            print("Item %u is not a polygon vertex" % seq)
            return

        items_to_remove = []
        for i in range(int(first_item.param1)):
            item = self.wploader.item(seq+i)
            if item is None:
                print("No item %s" % str(i))
                return
            if item.param1 != first_item.param1:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (seq, int(first_item.param1), i, int(item.param1)))
                return
            if not self.is_polygon_item(item):
                print("Item %u point %u is not a polygon vertex" % (seq, i))
                return
            items_to_remove.append(item)

        self.wploader.remove(items_to_remove)
        self.wploader.expected_count -= len(items_to_remove)
        self.wploader.last_change = time.time()
        self.send_all_items()

    def cmd_movepolypoint(self, args):
        '''moves item at offset item_offset in polygon starting at
        polygon_start_seqence to map click point'''
        if not self.check_have_list():
            return

        if len(args) < 2:
            print("Need first polygon point and vertex offset")
            return
        polygon_start_seq = int(args[0])
        item_offset = int(args[1])
        first_item, item_offset = self.find_polygon_point(polygon_start_seq, item_offset)
        if first_item is None:
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        moving_item = self.wploader.item(polygon_start_seq + item_offset)
        moving_item.x = latlon[0]
        moving_item.y = latlon[1]
        if moving_item.get_type() == "MISSION_ITEM_INT":
            moving_item.x *= 1e7
            moving_item.y *= 1e7
            moving_item.x = int(moving_item.x)
            moving_item.y = int(moving_item.y)

        self.wploader.set(moving_item, moving_item.seq)
        self.wploader.last_change = time.time()

        self.send_single_waypoint(moving_item.seq)

    def set_fence_enabled(self, do_enable):
        '''Enable or disable fence'''
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
            0,
            do_enable,
            0,
            0,
            0,
            0,
            0,
            0)

    def cmd_enable(self, args):
        '''enable fence'''
        self.set_fence_enabled(1)

    def cmd_disable(self, args):
        '''disable fence'''
        self.set_fence_enabled(0)

    def commands(self):
        '''returns map from command name to handling function'''
        ret = super(FenceModule, self).commands()
        ret.update({
            'addcircle': (self.cmd_addcircle, ["<inclusion|inc|exclusion|exc>", "RADIUS"]),
            'addpoly': (self.cmd_addpoly, ["<inclusion|inc|exclusion|exc>", "<radius>" "<pointcount>", "<rotation>"]),
            'movepolypoint': (self.cmd_movepolypoint, ["POLY_FIRSTPOINT", "POINT_OFFSET"]),
            'addreturnpoint': (self.cmd_addreturnpoint, []),
            'enable': self.cmd_enable,
            'disable': self.cmd_disable,
            'draw': self.cmd_draw,
            'removepolygon': (self.cmd_removepolygon, ["POLY_FIRSTPOINT"]),
        })
        return ret


def init(mpstate):
    '''initialise module'''
    # see if pymavlink is new enough to support new protocols:
    oldmodule = "fenceitem_protocol"
    try:
        mavwp.MissionItemProtocol_Fence
    except AttributeError:
        print("pymavlink too old; using old %s module" % oldmodule)
        mpstate.load_module(oldmodule)
        for (m, pm) in mpstate.modules:
            if m.name == "fence":
                return m
        return None

    return FenceModule(mpstate)
