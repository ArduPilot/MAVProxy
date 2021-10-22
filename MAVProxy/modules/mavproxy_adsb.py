'''
Support for ADS-B data
Samuel Dudley
Dec 2015
'''
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib import mp_menu
    from MAVProxy.modules.mavproxy_map import mp_slipmap

obc_icons = {
    100 : 'greenplane.png',
    101 : 'greenplane.png',
    102 : 'cloud.png',
    103 : 'migbird.png',
    104 : 'hawk.png'
}

obc_radius = {
    100 : 300,
    101 : 300,
    102 : 173,
    103 : 100,
    104 : 200
}

def get_threat_radius(m):

    if m.emitter_type == 255:
        ''' objectAvoidance Database item, squawk contains radius in cm'''
        return m.squawk * 0.01
    '''get threat radius for an OBC item'''
    return obc_radius.get(m.emitter_type, 0)


def get_threat_icon(m, default_icon):

    if m.emitter_type == 255:
        ''' objectAvoidance Database item, do not draw an icon'''
        return None
    '''get threat radius for an OBC item, else the true ADSB icon'''
    return obc_icons.get(m.emitter_type, default_icon)


class ADSBVehicle(object):
    '''a generic ADS-B threat'''

    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.vehicle_colour = 'green'  # use plane icon for now
        self.vehicle_type = 'plane'
        self.icon = self.vehicle_colour + self.vehicle_type + '.png'
        self.update_time = 0
        self.threat_radius_xy_override = 0
        self.threat_altitude_z_override = 0
        self.is_threat = False
        self.on_map = False

    def update(self, state, tnow):
        '''update the threat state'''
        self.state = state
        self.update_time = tnow

    def getLabel(self):
        return str(self.state.callsign) + "/" + str(int(self.state.altitude * 0.001)) + "m"

class ADSBModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(ADSBModule, self).__init__(mpstate, "adsb", "ADS-B data support", public=True, multi_vehicle=True)
        self.threat_vehicles = {}

        self.add_command('adsb', self.cmd_ADSB, "adsb control",
                         ["<status>", "set (ADSBSETTING)"])

        self.ADSB_settings = mp_settings.MPSettings([("timeout", int, 5),  # seconds
                                                     ("show_threat_radius", bool, False),
                                                     ("threat_radius", int, 200),  # meters (legacy)
                                                     ("threat_radius_xy_override", int, 0),  # meters
                                                     ("threat_altitude_z_override", int, 0)])  # meters
        self.add_completion_function('(ADSBSETTING)',
                                     self.ADSB_settings.completion)

        self.threat_timeout_timer = mavutil.periodic_event(2)
        self.tnow = self.get_time()

        self.update_map_timer = mavutil.periodic_event(1)

        # Dict of last known positions of vehicles, to check distances to ADSB vehicles
        # key is vehicle sysid, val is (long,lat,alt) tuple
        self.vehpos = {}

    def cmd_ADSB(self, args):
        '''adsb command parser'''
        usage = "usage: adsb <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print("total threat count: %u" % (len(self.threat_vehicles)))

            for id in self.threat_vehicles.keys():
                print("id: %s callsign: %s  alt: %.2f" % (id,
                                                          self.threat_vehicles[id].state.callsign,
                                                          self.threat_vehicles[id].state.altitude/1000))
        elif args[0] == "set":
            self.ADSB_settings.command(args[1:])
        else:
            print(usage)

    def check_threat_timeout(self):
        '''check and handle threat time out'''
        for id in self.threat_vehicles.keys():
            if self.threat_vehicles[id].update_time == 0:
                self.threat_vehicles[id].update_time = self.get_time()
            dt = self.get_time() - self.threat_vehicles[id].update_time
            if dt > self.ADSB_settings.timeout:
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
                self.threat_vehicles[id].menu_item = mp_menu.MPMenuItem(name=id, returnkey=None)
                selected_icon = get_threat_icon(self.threat_vehicles[id].state, self.threat_vehicles[id].icon)
                threat_colour = (0, 255, 255) if self.threat_vehicles[id].is_threat == 0 else (255, 0, 0)
                if self.threat_vehicles[id].on_map:
                    # update graphics
                    mp.map.set_position(id, (self.threat_vehicles[id].state.lat * 1e-7, self.threat_vehicles[id].state.lon * 1e-7), rotation=self.threat_vehicles[id].state.heading*0.01)
                    mp.map.remove_object(id+":circle")
                    if self.ADSB_settings.show_threat_radius and mp.map:
                        mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                                (self.threat_vehicles[id].state.lat * 1e-7,
                                                                self.threat_vehicles[id].state.lon * 1e-7),
                                                                self.threat_vehicles[id].threat_radius_xy_override, threat_colour, linewidth=1))
                else:
                    if selected_icon is not None:
                        # draw the vehicle on the map
                        popup = mp_menu.MPMenuSubMenu('ADSB', items=[self.threat_vehicles[id].menu_item])
                        icon = mp.map.icon(selected_icon)
                        
                        mp.map.add_object(mp_slipmap.SlipIcon(id, (self.threat_vehicles[id].state.lat * 1e-7, self.threat_vehicles[id].state.lon * 1e-7),
                                                              icon, layer=3, rotation=self.threat_vehicles[id].state.heading*0.01, follow=False,
                                                              trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                                                              popup_menu=popup, label=self.threat_vehicles[id].getLabel()))
                        if self.ADSB_settings.show_threat_radius:
                            mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                                    (self.threat_vehicles[id].state.lat * 1e-7,
                                                                    self.threat_vehicles[id].state.lon * 1e-7),
                                                                    self.threat_vehicles[id].threat_radius_xy_override, threat_colour, linewidth=1))
                    self.threat_vehicles[id].on_map = True

    def update_threats(self):
        # update the threat radius of all ADSB objects
        for id in self.threat_vehicles.keys():
            # update threat radius. Need to check for (legacy) threat_radius and use that if non-zero
            if self.ADSB_settings.threat_radius_xy_override == 0 and self.ADSB_settings.threat_radius == 0:
                self.threat_vehicles[id].threat_radius_xy_override = get_threat_radius(self.threat_vehicles[id].state)
            elif self.ADSB_settings.threat_radius != 0:
                self.threat_vehicles[id].threat_radius_xy_override = self.ADSB_settings.threat_radius
            else:
                self.threat_vehicles[id].threat_radius_xy_override = self.ADSB_settings.threat_radius_xy_override

            if self.ADSB_settings.threat_altitude_z_override == 0 and self.ADSB_settings.threat_radius == 0:
                self.threat_vehicles[id].threat_altitude_z_override = get_threat_radius(self.threat_vehicles[id].state)
            elif self.ADSB_settings.threat_radius != 0:
                self.threat_vehicles[id].threat_altitude_z_override = self.ADSB_settings.threat_radius
            else:
                self.threat_vehicles[id].threat_altitude_z_override = self.ADSB_settings.threat_altitude_z_override

            # Check if any connected vehicles are inside the threat zone of this ADSB object
            # and mark accordingly
            self.threat_vehicles[id].is_threat = False
            for idveh, posn in self.vehpos.items():
                vdelta = posn[2] - (self.threat_vehicles[id].state.altitude/1000)
                hdelta = mp_util.gps_distance(posn[0], posn[1], self.threat_vehicles[id].state.lat*1.0e-7, self.threat_vehicles[id].state.lon*1.0e-7)
                if abs(vdelta) < self.threat_vehicles[id].threat_altitude_z_override or hdelta < self.threat_vehicles[id].threat_radius_xy_override:
                    self.threat_vehicles[id].is_threat = True

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "ADSB_VEHICLE":
            id = 'ADSB-' + str(m.ICAO_address)
            if id not in self.threat_vehicles.keys():  # check to see if the vehicle is in the dict
                # if not then add it
                self.threat_vehicles[id] = ADSBVehicle(id, m)
            else:  # the vehicle exists in the database
                # update the database entry
                self.threat_vehicles[id].update(m, self.get_time())
        elif m.get_type() == 'GLOBAL_POSITION_INT':
            # Log vehicle positions and compare with ADSB vehicle locations
            self.vehpos[m.get_srcSystem()] = (m.lat*1.0e-7, m.lon*1.0e-7, m.alt/1000)
            self.update_threats()

    def idle_task(self):
        '''called on idle'''
        if self.threat_timeout_timer.trigger():
            self.check_threat_timeout()
        if self.update_map_timer.trigger():
            self.update_map()

def init(mpstate):
    '''initialise module'''
    return ADSBModule(mpstate)
