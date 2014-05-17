"""
    MAVProxy rally module
"""

from pymavlink import mavwp
import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_menu import *

class RallyModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RallyModule, self).__init__(mpstate, "rally", "rally point control", public = True)
        self.rallyloader = mavwp.MAVRallyLoader(mpstate.status.target_system, mpstate.status.target_component)
        self.add_command('rally', self.cmd_rally, "rally point control", ["<add|clear|list|move|remove>",
                                    "<load|save> (FILENAME)"])
        self.have_list = False

        self.menu_added_console = False
        self.menu_added_map = False
        self.menu = MPMenuSubMenu('Rally',
                                  items=[MPMenuItem('Clear', 'Clear', '# rally clear'),
                                         MPMenuItem('List', 'List', '# rally list'),
                                         MPMenuItem('Load', 'Load', '# rally load ',
                                                    handler=MPMenuCallFileDialog(flags=wx.FD_OPEN,
                                                                                 title='Rally Load',
                                                                                 wildcard='*.rally')),
                                         MPMenuItem('Save', 'Save', '# rally save ',
                                                    handler=MPMenuCallFileDialog(flags=wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT,
                                                                                 title='Rally Save',
                                                                                 wildcard='*.rally')),
                                         MPMenuItem('Add', 'Add', '# rally add ',
                                                    handler=MPMenuCallTextDialog(title='Rally Altitude (m)',
                                                                                 default=100))])


    def idle_task(self):
        '''called on idle'''
        if not self.menu_added_console and self.module('console') is not None:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)
        if not self.menu_added_map and self.module('map') is not None:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)

    def cmd_rally_add(self, args):
        '''handle rally add'''
        if len(args) < 1:
            alt = self.settings.rallyalt
        else:
            alt = float(args[0])

        if not self.have_list:
            print("Please list rally points first")
            return

        if (self.rallyloader.rally_count() > 4):
            print ("Only 5 rally points possible per flight plan.")
            return

        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return

        break_alt = 0.0
        land_hdg = 0.0
        if (len(args) > 2):
            break_alt = float(args[2])
        if (len(args) > 3):
            land_hdg = float(args[3])

        self.rallyloader.create_and_append_rally_point(latlon[0] * 1e7, latlon[1] * 1e7, alt, break_alt, land_hdg, 0)
        self.send_rally_points()
        print("Added Rally point at %s %f" % (str(latlon), alt))

    def cmd_rally_move(self, args):
        '''handle rally move'''
        if len(args) < 1:
            print("Usage: rally move RALLYNUM")
            return
        if not self.have_list:
            print("Please list rally points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.rallyloader.rally_count():
            print("Invalid rally point number %u" % idx)
            return

        rpoint = self.rallyloader.rally_point(idx-1)

        try:
            latlon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return

        oldpos = (rpoint.lat*1e-7, rpoint.lng*1e-7)
        self.rallyloader.move(idx, latlon[0], latlon[1])
        self.send_rally_point(idx-1)
        p = self.fetch_rally_point(idx-1)
        if p.lat != int(latlon[0]*1e7) or p.lng != int(latlon[1]*1e7):
            print("Rally move failed")
            return
        self.rallyloader.reindex()
        print("Moved rally point from %s to %s at %fm" % (str(oldpos), str(latlon), rpoint.alt))


    def cmd_rally(self, args):
        '''rally point commands'''
        #TODO: add_land arg
        if len(args) < 1:
            self.print_usage()
            return

        elif args[0] == "add":
            self.cmd_rally_add(args[1:])

        elif args[0] == "move":
            self.cmd_rally_move(args[1:])

        elif args[0] == "clear":
            self.rallyloader.clear()
            self.mav_param.mavset(self.master,'RALLY_TOTAL',0,3)

        elif args[0] == "remove":
            if not self.have_list:
                print("Please list rally points first")
                return
            if (len(args) < 2):
                print("Usage: rally remove RALLYNUM")
                return
            self.rallyloader.remove(int(args[1]))
            self.send_rally_points()

        elif args[0] == "list":
            self.list_rally_points()
            self.have_list = True

        elif args[0] == "load":
            if (len(args) < 2):
                print("Usage: rally load filename")
                return

            try:
                self.rallyloader.load(args[1])
            except Exception as msg:
                print("Unable to load %s - %s" % (args[1], msg))
                return

            self.send_rally_points()
            self.have_list = True

            print("Loaded %u rally points from %s" % (self.rallyloader.rally_count(), args[1]))

        elif args[0] == "save":
            if (len(args) < 2):
                print("Usage: rally save filename")
                return

            self.rallyloader.save(args[1])

            print("Saved rally file %s" % args[1])

        else:
            self.print_usage()

    def mavlink_packet(self, m):
        '''handle incoming mavlink packet'''
        return #TODO when applicable

    def send_rally_point(self, i):
        '''send rally points from fenceloader'''
        p = self.rallyloader.rally_point(i)
        p.target_system = self.target_system
        p.target_component = self.target_component
        self.master.mav.send(p)

    def send_rally_points(self):
        '''send rally points from fenceloader'''
        self.mav_param.mavset(self.master,'RALLY_TOTAL',self.rallyloader.rally_count(),3)

        for i in range(self.rallyloader.rally_count()):
            self.send_rally_point(i)

    def fetch_rally_point(self, i):
        '''fetch one rally point'''
        self.master.mav.rally_fetch_point_send(self.target_system,
                                                    self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 1:
            p = self.master.recv_match(type='RALLY_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch rally point %u" % i)
            return None
        return p

    def list_rally_points(self):
        self.rallyloader.clear()
        rally_count = self.mav_param.get('RALLY_TOTAL',0)
        if rally_count == 0:
            print("No rally points")
            return
        for i in range(int(rally_count)):
            p = self.fetch_rally_point(i)
            if p is None:
                return
            self.rallyloader.append_rally_point(p)

        for i in range(self.rallyloader.rally_count()):
            p = self.rallyloader.rally_point(i)
            self.console.writeln("lat=%f lng=%f alt=%f break_alt=%f land_dir=%f" % (p.lat * 1e-7, p.lng * 1e-7, p.alt, p.break_alt, p.land_dir))

    def print_usage(self):
        print("Usage: rally <list|load|save|add|remove|move|clear>")

def init(mpstate):
    '''initialise module'''
    return RallyModule(mpstate)
