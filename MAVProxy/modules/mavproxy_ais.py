'''
Support for AIS data
Stephen Dade
June 2021
'''

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib import mp_menu
    from MAVProxy.modules.mavproxy_map import mp_slipmap

class AISVehicle():
    '''a generic AIS threat'''

    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.vehicle_colour = 'green'  # use boat icon for now
        self.vehicle_type = 'boat'
        self.icon = self.vehicle_colour + self.vehicle_type + '.png'
        self.update_time = 0
        self.onMap = False

    def update(self, state, tnow):
        self.state = state
        self.update_time = tnow

    def getThreatRadius(self, default_radius):
        ''' get threat radius, based on ship type'''
        # threat radius is the maximum of distance to AIS location
        if self.state.flags & mavutil.mavlink.AIS_FLAGS_LARGE_BOW_DIMENSION or self.state.flags & mavutil.mavlink.AIS_FLAGS_LARGE_STERN_DIMENSION:
            threat_radius = 511
        elif self.state.flags & mavutil.mavlink.AIS_FLAGS_LARGE_PORT_DIMENSION or self.state.flags & mavutil.mavlink.AIS_FLAGS_LARGE_STARBOARD_DIMENSION:
            threat_radius = 63
        else:
            threat_radius = max(self.state.dimension_bow, self.state.dimension_stern, self.state.dimension_port, self.state.dimension_starboard)
        #default to threat_radius setting
        if threat_radius == 0:
            return default_radius

class AISModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(AISModule, self).__init__(mpstate, "ais", "AIS data support", public=True)
        self.threat_vehicles = {}

        self.add_command('ais', self.cmd_AIS, "ais control",
                         ["<status>", "set (AISSETTING)"])

        self.AIS_settings = mp_settings.MPSettings([("timeout", int, 60),  # seconds
                                                    ("threat_radius", int, 200)])  # meters
        self.add_completion_function('(AISSETTING)',
                                     self.AIS_settings.completion)

        self.threat_timeout_timer = mavutil.periodic_event(2)

        self.update_map_timer = mavutil.periodic_event(1)

        self.tnow = self.get_time()

    def cmd_AIS(self, args):
        '''ais command parser'''
        usage = "usage: ais <set>"
        if len(args) == 0:
            print(usage)
        elif args[0] == "set":
            self.AIS_settings.command(args[1:])
        else:
            print(usage)

    def check_threat_timeout(self):
        '''check and handle threat time out'''
        for id in self.threat_vehicles:
            if self.threat_vehicles[id].update_time == 0:
                self.threat_vehicles[id].update_time = self.get_time()
            dt = self.get_time() - self.threat_vehicles[id].update_time
            if dt > self.AIS_settings.timeout:
                # if the threat has timed out...
                del self.threat_vehicles[id]  # remove the threat from the dict
                for mp in self.module_matching('map*'):
                    # remove the threat from the map
                    mp.map.remove_object(id)
                    mp.map.remove_object(id+":circle")
                # we've modified the dict we're iterating over, so
                # we'll get any more timed-out threats next time we're
                # called:
                return

    def update_map(self):
        '''update the map graphics'''
        for mp in self.module_matching('map*'):
            for id in self.threat_vehicles.keys():
                mstate = self.threat_vehicles[id].state
                threat_radius = self.threat_vehicles[id].getThreatRadius(self.AIS_settings.threat_radius)
                # update if existing object on map, else create
                if self.threat_vehicles[id].onMap:
                    mp.map.set_position(id, (mstate.lat * 1e-7, mstate.lon * 1e-7), 3, rotation=mstate.heading*0.01, label=str(mstate.callsign))
                    # Turns out we can't edit the circle's radius, so have to remove and re-add
                    mp.map.remove_object(id+":circle")
                    mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                            (mstate.lat * 1e-7, mstate.lon * 1e-7),
                                                            threat_radius, (0, 255, 255), linewidth=1))
                else:
                    self.threat_vehicles[id].menu_item = mp_menu.MPMenuItem(name=id, returnkey=None)

                    # draw the vehicle on the map
                    popup = mp_menu.MPMenuSubMenu('AIS', items=[self.threat_vehicles[id].menu_item])
                    icon = mp_slipmap.SlipIcon(
                        id,
                        (mstate.lat * 1e-7, mstate.lon * 1e-7),
                        mp.map.icon(self.threat_vehicles[id].icon),
                        3,
                        rotation=mstate.heading*0.01,
                        follow=False,
                        trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                        popup_menu=popup,
                    )
                    mp.map.add_object(icon)
                    mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                            (mstate.lat * 1e-7, mstate.lon * 1e-7),
                                                            threat_radius, (0, 255, 255), linewidth=1))
                    self.threat_vehicles[id].onMap = True

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "AIS_VESSEL":
            id = 'AIS-' + str(m.MMSI)

            if id not in self.threat_vehicles.keys():  # check to see if the vehicle is in the dict
                # if not then add it
                self.threat_vehicles[id] = AISVehicle(id=id, state=m)
            else:  # the vehicle is in the dict
                # update the dict entry
                self.threat_vehicles[id].update(m, self.get_time())

    def idle_task(self):
        '''called on idle'''
        if self.threat_timeout_timer.trigger():
            self.check_threat_timeout()
        if self.update_map_timer.trigger():
            self.update_map()

def init(mpstate):
    '''initialise module'''
    return AISModule(mpstate)
