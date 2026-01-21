#!/usr/bin/env python
'''locationdb command handling'''

import struct
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib import mp_menu
    from MAVProxy.modules.mavproxy_map import mp_slipmap


class DBItem():
    def __init__(self):
        self.key = 0
        self.timestamp_ms = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.velx = 0
        self.vely = 0
        self.velz = 0
        self.accx = 0
        self.accy = 0
        self.accz = 0
        self.heading = 0
        self.radius = 0
        self.populated_fields = 0
        self.onMap = False


class LocDBModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(LocDBModule, self).__init__(mpstate, "locdb", "LocationDB handling", public=True)
        self.add_command('locdb', self.cmd_locdb, 'locationdb management',
                         ["<list>"])
        self.locdb_ftp_name = "@LOCATIONDB/locationdb.dat"
        self.num_items = 0
        self.background_fetch = True
        self.item_refresh_timer = mavutil.periodic_event(0.5)
        self.dbitems = []

    def cmd_locdb(self, args):
        '''locationdb commands'''
        usage = "usage: locdb <list>"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == "list":
            self.background_fetch = False
            self.dbitems_list()
        else:
            print(usage)

    def dbitems_list(self):
        print("Total DB Items: %u" % self.num_items)
        for item in self.dbitems:
            print("Key:%s, Last Update:%u, Lat:%.2f, Lon:%.2f, Alt:%.2f , Vel:(%.2f, %.2f, %.2f), Acc:(%.2f, %.2f, %.2f), Hdg:%.2f, Rad:%.2f" % ( # noqa
                item.key,
                item.timestamp_ms,
                item.lat,
                item.lon,
                item.alt,
                item.velx,
                item.vely,
                item.velz,
                item.accx,
                item.accy,
                item.accz,
                item.heading,
                item.radius
            ))
        return

    def update_map(self, new_items):
        for mp in self.module_matching('map*'):
            # remove all old items
            for item in self.dbitems:
                if item.onMap:
                    mp.map.remove_object(item.key)

            # show new items
            for new_item in new_items:
                if new_item.onMap:
                    continue

                menu_item = mp_menu.MPMenuItem(
                    'Follow',
                    'Follow this object',
                    '# command_int GLOBAL MAV_CMD_DO_FOLLOW 0 0 0 2 0 0 0 '+str(int(new_item.key, 16))+' 0'
                )
                popup = mp_menu.MPMenuSubMenu('LOCDB', items=[menu_item])
                icon = mp_slipmap.SlipIcon(
                    new_item.key,
                    (new_item.lat * 1e-7, new_item.lon * 1e-7),
                    mp.map.icon('greensinglecopter.png'),
                    layer=3,
                    rotation=new_item.heading*0.01,
                    follow=False,
                    trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                    popup_menu=popup
                )
                mp.map.add_object(icon)
                new_item.onMap = True

        # update the item list
        self.dbitems = new_items

    def create_DBItem_from_packet(self, packet, header_size):
        '''return DBItem by decoding a packet'''
        item = DBItem()
        item.key, item.timestamp_ms, item.populated_fields = (struct.unpack("<IIBB", packet[:header_size]))[:3]
        item.key = hex(item.key)
        packet = packet[header_size:]

        # Populated field bitmask:
        # POS = (1U << 0)
        # VEL = (1U << 1)
        # ACC = (1U << 2)
        # HEADING = (1U << 3)
        # RADIUS = (1U << 4)

        if (item.populated_fields & (1 << 0)) != 0:
            pos_info = packet[:12]
            packet = packet[12:]
            item.lat, item.lon, item.alt = struct.unpack("<fff", pos_info)

        if (item.populated_fields & (1 << 1)) != 0:
            vel_info = packet[:12]
            packet = packet[12:]
            item.velx, item.vely, item.velz = struct.unpack("<fff", vel_info)

        if (item.populated_fields & (1 << 2)) != 0:
            acc_info = packet[:12]
            packet = packet[12:]
            item.accx, item.accy, item.accz = struct.unpack("<fff", acc_info)

        if (item.populated_fields & (1 << 3)) != 0:
            heading_info = packet[:4]
            packet = packet[4:]
            item.heading = (struct.unpack("<f", heading_info))[0]

        if (item.populated_fields & (1 << 4)) != 0:
            radius_info = packet[:4]
            packet = packet[4:]
            item.radius = (struct.unpack("<f", radius_info))[0]

        if len(packet) > 0:
            print("locdb: packet size and field bitmask mismatch")
            return None

        return item

    def ftp_callback(self, fh):
        '''callback from ftp fetch of db items'''
        if fh is None:
            return
        magic = 0x2801
        data = fh.read()
        magic2, num_items = struct.unpack("<HH", data[0:4])
        self.num_items = num_items
        if magic != magic2:
            print("locationdb: bad magic 0x%x expected 0x%x" % (magic2, magic))
            return

        data = data[4:]
        packet_header_size = 10
        items = []
        while len(data) >= packet_header_size:
            packed_header = data[:packet_header_size]
            unpacked_header = struct.unpack("<IIBB", packed_header)
            filled_fields_count = unpacked_header[3]
            compressed_packet_size = packet_header_size + 4 * filled_fields_count
            if len(data) >= compressed_packet_size:
                packet = data[:compressed_packet_size]
                data = data[compressed_packet_size:]
                item = self.create_DBItem_from_packet(packet, packet_header_size)
                if item is not None:
                    items.append(item)

        self.update_map(items)

    def update_items(self):
        ftp = self.mpstate.module('ftp')
        if ftp is None:
            return
        ftp.cmd_get([self.locdb_ftp_name], callback=self.ftp_callback)

    def idle_task(self):
        '''called on idle'''
        if self.item_refresh_timer.trigger():
            self.update_items()


def init(mpstate):
    '''initialise module'''
    return LocDBModule(mpstate)
