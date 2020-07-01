"""
    MAVProxy geofence module
"""
import os, time, platform
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class FenceModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FenceModule, self).__init__(mpstate, "fence", "geo-fence management", public = True)
        self.fenceloader_by_sysid = {}
        self.last_fence_breach = 0
        self.last_fence_status = 0
        self.present = False
        self.enabled = False
        self.healthy = True
        self.add_command('fence', self.cmd_fence,
                         "geo-fence management",
                         ["<draw|list|clear|enable|disable|move|remove>",
                          "<load|save> (FILENAME)"])

        self.have_list = False

        if self.continue_mode and self.logdir is not None:
            fencetxt = os.path.join(self.logdir, 'fence.txt')
            if os.path.exists(fencetxt):
                self.fenceloader.load(fencetxt)
                self.have_list = True
                print("Loaded fence from %s" % fencetxt)

        self.menu_added_console = False
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Fence',
                                  items=[MPMenuItem('Clear', 'Clear', '# fence clear'),
                                         MPMenuItem('List', 'List', '# fence list'),
                                         MPMenuItem('Load', 'Load', '# fence load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Fence Load',
                                                                                 wildcard='FenceFiles(*.txt,*.fen)|*.txt;*.fen')),
                                         MPMenuItem('Save', 'Save', '# fence save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Fence Save',
                                                                                 wildcard='FenceFiles(*.txt,*.fen)|*.txt;*.fen')),
                                         MPMenuItem('Draw', 'Draw', '# fence draw')])

    @property
    def fenceloader(self):
        '''fence loader by sysid'''
        if not self.target_system in self.fenceloader_by_sysid:
            self.fenceloader_by_sysid[self.target_system] = mavwp.MAVFenceLoader()
        return self.fenceloader_by_sysid[self.target_system]

    def idle_task(self):
        '''called on idle'''
        if self.module('console') is not None:
            if not self.menu_added_console:
                self.menu_added_console = True
                self.module('console').add_menu(self.menu)
        else:
            self.menu_added_console = False

        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

    def mavlink_packet(self, m):
        '''handle and incoming mavlink packet'''
        if m.get_type() == "FENCE_STATUS":
            self.last_fence_breach = m.breach_time
            self.last_fence_status = m.breach_status
        elif m.get_type() in ['SYS_STATUS']:
            bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

            present = ((m.onboard_control_sensors_present & bits) == bits)
            if self.present == False and present == True:
                self.say("fence present")
            elif self.present == True and present == False:
                self.say("fence removed")
            self.present = present

            enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
            if self.enabled == False and enabled == True:
                self.say("fence enabled")
            elif self.enabled == True and enabled == False:
                self.say("fence disabled")
            self.enabled = enabled

            healthy = ((m.onboard_control_sensors_health & bits) == bits)
            if self.healthy == False and healthy == True:
                self.say("fence OK")
            elif self.healthy == True and healthy == False:
                self.say("fence breach")
            self.healthy = healthy

            #console output for fence:
            if not self.present:
                self.console.set_status('Fence', 'FEN', row=0, fg='black')
            elif self.enabled == False:
                self.console.set_status('Fence', 'FEN', row=0, fg='grey')
            elif self.enabled == True and self.healthy == True:
                self.console.set_status('Fence', 'FEN', row=0, fg='green')
            elif self.enabled == True and self.healthy == False:
                self.console.set_status('Fence', 'FEN', row=0, fg='red')

    def set_fence_enabled(self, do_enable):
        '''Enable or disable fence'''
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0,
            do_enable, 0, 0, 0, 0, 0, 0)

    def cmd_fence_move(self, args):
        '''handle fencepoint move'''
        if len(args) < 1:
            print("Usage: fence move FENCEPOINTNUM")
            return
        if not self.have_list:
            print("Please list fence points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.fenceloader.count():
            print("Invalid fence point number %u" % idx)
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        # note we don't subtract 1, as first fence point is the return point
        self.fenceloader.move(idx, latlon[0], latlon[1])
        if self.send_fence():
            print("Moved fence point %u" % idx)

    def cmd_fence_remove(self, args):
        '''handle fencepoint remove'''
        if len(args) < 1:
            print("Usage: fence remove FENCEPOINTNUM")
            return
        if not self.have_list:
            print("Please list fence points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.fenceloader.count():
            print("Invalid fence point number %u" % idx)
            return

        # note we don't subtract 1, as first fence point is the return point
        self.fenceloader.remove(idx)
        if self.send_fence():
            print("Removed fence point %u" % idx)
        else:
            print("Failed to remove fence point %u" % idx)

    def cmd_fence(self, args):
        '''fence commands'''
        if len(args) < 1:
            self.print_usage()
            return

        if args[0] == "enable":
            self.set_fence_enabled(1)
        elif args[0] == "disable":
            self.set_fence_enabled(0)
        elif args[0] == "load":
            if len(args) != 2:
                print("usage: fence load <filename>")
                return
            self.load_fence(args[1])
        elif args[0] == "list":
            self.list_fence(None)
        elif args[0] == "move":
            self.cmd_fence_move(args[1:])
        elif args[0] == "remove":
            self.cmd_fence_remove(args[1:])
        elif args[0] == "save":
            if len(args) != 2:
                print("usage: fence save <filename>")
                return
            self.list_fence(args[1])
        elif args[0] == "show":
            if len(args) != 2:
                print("usage: fence show <filename>")
                return
            self.fenceloader.load(args[1])
            self.have_list = True
        elif args[0] == "draw":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return
            self.mpstate.map_functions['draw_lines'](self.fence_draw_callback)
            print("Drawing fence on map")
        elif args[0] == "clear":
            self.param_set('FENCE_TOTAL', 0, 3)
        else:
            self.print_usage()

    def load_fence(self, filename):
        '''load fence points from a file'''
        try:
            self.fenceloader.target_system = self.target_system
            self.fenceloader.target_component = self.target_component
            self.fenceloader.load(filename.strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u geo-fence points from %s" % (self.fenceloader.count(), filename))
        self.send_fence()

    def send_fence(self):
        '''send fence points from fenceloader'''
        # must disable geo-fencing when loading
        self.fenceloader.target_system = self.target_system
        self.fenceloader.target_component = self.target_component
        self.fenceloader.reindex()
        action = self.get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
        self.param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE, 3)
        self.param_set('FENCE_TOTAL', self.fenceloader.count(), 3)
        for i in range(self.fenceloader.count()):
            p = self.fenceloader.point(i)
            self.master.mav.send(p)
            p2 = self.fetch_fence_point(i)
            if p2 is None:
                self.param_set('FENCE_ACTION', action, 3)
                return False
            if (p.idx != p2.idx or
                abs(p.lat - p2.lat) >= 0.00003 or
                abs(p.lng - p2.lng) >= 0.00003):
                print("Failed to send fence point %u" % i)
                self.param_set('FENCE_ACTION', action, 3)
                return False
        self.param_set('FENCE_ACTION', action, 3)
        return True

    def fetch_fence_point(self ,i):
        '''fetch one fence point'''
        self.master.mav.fence_fetch_point_send(self.target_system,
                                                    self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 3:
            p = self.master.recv_match(type='FENCE_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch point %u" % i)
            return None
        return p

    def fence_draw_callback(self, points):
        '''callback from drawing a fence'''
        self.fenceloader.clear()
        if len(points) < 3:
            return
        self.fenceloader.target_system = self.target_system
        self.fenceloader.target_component = self.target_component
        bounds = mp_util.polygon_bounds(points)
        (lat, lon, width, height) = bounds
        center = (lat+width/2, lon+height/2)
        self.fenceloader.add_latlon(center[0], center[1])
        for p in points:
            self.fenceloader.add_latlon(p[0], p[1])
        # close it
        self.fenceloader.add_latlon(points[0][0], points[0][1])
        self.send_fence()
        self.have_list = True

    def list_fence(self, filename):
        '''list fence points, optionally saving to a file'''
        self.fenceloader.clear()
        count = self.get_mav_param('FENCE_TOTAL', 0)
        if count == 0:
            print("No geo-fence points")
            return
        for i in range(int(count)):
            p = self.fetch_fence_point(i)
            if p is None:
                return
            self.fenceloader.add(p)

        if filename is not None:
            try:
                self.fenceloader.save(filename.strip('"'))
            except Exception as msg:
                print("Unable to save %s - %s" % (filename, msg))
                return
            print("Saved %u geo-fence points to %s" % (self.fenceloader.count(), filename))
        else:
            for i in range(self.fenceloader.count()):
                p = self.fenceloader.point(i)
                self.console.writeln("lat=%f lng=%f" % (p.lat, p.lng))
        if self.status.logdir is not None:
            fname = 'fence.txt'
            if self.target_system > 1:
                fname = 'fence_%u.txt' % self.target_system
            fencetxt = os.path.join(self.status.logdir, fname)
            self.fenceloader.save(fencetxt.strip('"'))
            print("Saved fence to %s" % fencetxt)
        self.have_list = True

    def print_usage(self):
        print("usage: fence <enable|disable|list|load|save|clear|draw|move|remove>")

    def unload(self):
        self.remove_command("fence")
        if self.module('console') is not None and self.menu_added_console:
            self.menu_added_console = False
            self.module('console').remove_menu(self.menu)
        if self.module('map') is not None and self.menu_added_map:
            self.menu_added_map = False
            self.module('map').remove_menu(self.menu)
        super(FenceModule, self).unload()

def init(mpstate):
    '''initialise module'''
    return FenceModule(mpstate)
