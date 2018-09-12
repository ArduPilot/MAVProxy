'''
Support for ADS-B data
Samuel Dudley
Dec 2015
'''

from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil

obc_icons = {
    0 : 'greenplane.png',
    1 : 'greenplane.png',
    2 : 'cloud.png',
    3 : 'migbird.png',
    4 : 'hawk.png'
}

obc_radius = {
    0 : 300,
    1 : 300,
    2 : 173,
    3 : 100,
    4 : 200
}
    
def get_threat_radius(obc_type):
    '''get threat radius for an OBC item'''
    if obc_type in obc_radius:
        return obc_radius[obc_type]
    return 0

class ADSBVehicle(object):
    '''a generic ADS-B threat'''

    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.vehicle_colour = 'green'  # use plane icon for now
        self.vehicle_type = 'plane'
        self.icon = self.vehicle_colour + self.vehicle_type + '.png'
        self.update_time = 0
        self.is_evading_threat = False
        self.v_distance = None
        self.h_distance = None
        self.distance = None

    def update(self, state, tnow):
        '''update the threat state'''
        self.state = state
        self.update_time = tnow


class ADSBModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(ADSBModule, self).__init__(mpstate, "adsb", "ADS-B data support", public = True)
        self.threat_vehicles = {}
        self.active_threat_ids = []  # holds all threat ids the vehicle is evading

        self.add_command('adsb', self.cmd_ADSB, "adsb control",
                         ["<status>", "set (ADSBSETTING)"])

        self.ADSB_settings = mp_settings.MPSettings([("timeout", int, 5),  # seconds
                                                     ("threat_radius", int, 200),  # meters
                                                     ("show_threat_radius", bool, False),
                                                     # threat_radius_clear = threat_radius*threat_radius_clear_multiplier
                                                     ("threat_radius_clear_multiplier", int, 2),
                                                     ("show_threat_radius_clear", bool, False)])
        self.add_completion_function('(ADSBSETTING)',
                                     self.ADSB_settings.completion)
        
        self.threat_detection_timer = mavutil.periodic_event(2)
        self.threat_timeout_timer = mavutil.periodic_event(2)
        self.tnow = self.get_time()

    def cmd_ADSB(self, args):
        '''adsb command parser'''
        usage = "usage: adsb <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print("total threat count: %u  active threat count: %u" %
                  (len(self.threat_vehicles), len(self.active_threat_ids)))

            for id in self.threat_vehicles.keys():
                print("id: %s  distance: %.2f m callsign: %s  alt: %.2f" % (id,
                                                                            self.threat_vehicles[id].distance,
                                                                            self.threat_vehicles[id].state['callsign'],
                                                                            self.threat_vehicles[id].state['altitude']))
        elif args[0] == "set":
            self.ADSB_settings.command(args[1:])
        else:
            print(usage)

    def perform_threat_detection(self):
        '''determine threats'''
        # TODO: perform more advanced threat detection
        threat_radius_clear = self.ADSB_settings.threat_radius * \
            self.ADSB_settings.threat_radius_clear_multiplier

        for id in self.threat_vehicles.keys():
            if self.threat_vehicles[id].distance is not None:
                if self.threat_vehicles[id].distance <= self.ADSB_settings.threat_radius and not self.threat_vehicles[id].is_evading_threat:
                    # if the threat is in the threat radius and not currently
                    # known to the module...
                    # set flag to action threat
                    self.threat_vehicles[id].is_evading_threat = True

                if self.threat_vehicles[id].distance > threat_radius_clear and self.threat_vehicles[id].is_evading_threat:
                    # if the threat is known to the module and outside the
                    # threat clear radius...
                    # clear flag to action threat
                    self.threat_vehicles[id].is_evading_threat = False

        self.active_threat_ids = [id for id in self.threat_vehicles.keys(
        ) if self.threat_vehicles[id].is_evading_threat]

    def update_threat_distances(self, latlonalt):
        '''update the distance between threats and vehicle'''
        for id in self.threat_vehicles.keys():
            threat_latlonalt = (self.threat_vehicles[id].state['lat'] * 1e-7,
                                self.threat_vehicles[id].state['lon'] * 1e-7,
                                self.threat_vehicles[id].state['altitude'])

            self.threat_vehicles[id].h_distance = self.get_h_distance(latlonalt, threat_latlonalt)
            self.threat_vehicles[id].v_distance = self.get_v_distance(latlonalt, threat_latlonalt)

            # calculate and set the total distance between threat and vehicle
            self.threat_vehicles[id].distance = sqrt(
                self.threat_vehicles[id].h_distance**2 + (self.threat_vehicles[id].v_distance)**2)

    def get_h_distance(self, latlonalt1, latlonalt2):
        '''get the horizontal distance between threat and vehicle'''
        (lat1, lon1, alt1) = latlonalt1
        (lat2, lon2, alt2) = latlonalt2

        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)

        dLat = lat2 - lat1
        dLon = lon2 - lon1

        # math as per mavextra.distance_two()
        a = sin(0.5 * dLat)**2 + sin(0.5 * dLon)**2 * cos(lat1) * cos(lat2)
        c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a))
        return 6371 * 1000 * c

    def get_v_distance(self, latlonalt1, latlonalt2):
        '''get the horizontal distance between threat and vehicle'''
        (lat1, lon1, alt1) = latlonalt1
        (lat2, lon2, alt2) = latlonalt2
        return alt2 - alt1

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

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "ADSB_VEHICLE":
            id = 'ADSB-' + str(m.ICAO_address)
            if id not in self.threat_vehicles.keys():  # check to see if the vehicle is in the dict
                # if not then add it
                self.threat_vehicles[id] = ADSBVehicle(id=id, state=m.to_dict())
                for mp in self.module_matching('map*'):
                    self.threat_vehicles[id].menu_item = MPMenuItem(name=id, returnkey=None)
                    if m.emitter_type >= 100 and m.emitter_type-100 in obc_icons:
                        icon = mp.map.icon(obc_icons[m.emitter_type-100])
                        threat_radius = get_threat_radius(m.emitter_type-100)
                    else:
                        icon = mp.map.icon(self.threat_vehicles[id].icon)
                        threat_radius = 0
                    popup = MPMenuSubMenu('ADSB', items=[self.threat_vehicles[id].menu_item])
                    # draw the vehicle on the map
                    mp.map.add_object(mp_slipmap.SlipIcon(id, (m.lat * 1e-7, m.lon * 1e-7),
                                                                    icon, layer=3, rotation=m.heading*0.01, follow=False,
                                                                    trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                                                                    popup_menu=popup))
                    if threat_radius > 0:
                        mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                    (m.lat * 1e-7, m.lon * 1e-7),
                                                    threat_radius, (0, 255, 255), linewidth=1))
            else:  # the vehicle is in the dict
                # update the dict entry
                self.threat_vehicles[id].update(m.to_dict(), self.get_time())
                for mp in self.module_matching('map*'):
                    # update the map
                    ground_alt = self.console.ElevationMap.GetElevation(m.lat*1e-7, m.lon*1e-7)
                    alt_amsl = m.altitude * 0.001
                    if alt_amsl > 0:
                        alt = int(alt_amsl - ground_alt)
                        label = str(alt) + "m"
                    else:
                        label = None
                    mp.map.set_position(id, (m.lat * 1e-7, m.lon * 1e-7), rotation=m.heading*0.01, label=label, colour=(0,250,250))
                    mp.map.set_position(id+":circle", (m.lat * 1e-7, m.lon * 1e-7))

    def idle_task(self):
        '''called on idle'''
        if self.threat_timeout_timer.trigger():
            self.check_threat_timeout()

        if self.threat_detection_timer.trigger():
            self.perform_threat_detection()
            # TODO: possibly evade detected threats with ids in
            # self.active_threat_ids


def init(mpstate):
    '''initialise module'''
    return ADSBModule(mpstate)
