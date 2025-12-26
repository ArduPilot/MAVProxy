'''
Support for ADS-B data
Samuel Dudley
Dec 2015
'''

from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
from PIL import ImageColor

obc_icons = {
     99 : 'flag.png',
    100 : 'greenplane.png',
    101 : 'bluecopter.png',
    102 : 'cloud.png',
    103 : 'migbird.png',
    104 : 'hawk.png'
}

obc_radius = {
     99 : 25,
    100 : 609.6, # F3442M-23 standard boundary for "Well Clear" of aircraft = 2000ft
    101 : 300,
    102 : 173,
    103 : 100,
    104 : 200
}

obc_height = {
     14 : 25,
     99 : 25,
    100 : 76,   # F3442M-23 standard vertical boundary for "Well Clear" of aircraft = 250ft
    101 : 100,
    102 : 173,
    103 : 100,
    104 : 200
}

def get_threat_radius(emitter_type, squawk):

    if emitter_type == 255:
        ''' objectAvoidance Database item, squawk contains radius in cm'''
        return squawk * 0.01
    if emitter_type < 14:
        ''' adsb emitters for all crude vehicles are < 14 (UAV) '''
        return 609.6
    '''get threat height for an OBC item'''
    return obc_radius.get(emitter_type,609.6)

def get_threat_height(emitter_type):
    if emitter_type < 14:
        ''' adsb emitters for all crude vehicles are < 14 (UAV) '''
        return 76
    return obc_height.get(emitter_type)

def get_threat_icon(emitter_type, default_icon):

    if emitter_type == 255:
        ''' objectAvoidance Database item, do not draw an icon'''
        return obc_icons.get(99, default_icon)
    '''get threat radius for an OBC item, else the true ADSB icon'''
    return obc_icons.get(emitter_type, default_icon)

''' internally we use these special (invalid according to the ADS-B spec) emitter types for test obstacles '''
def get_internal_emitter(trkn):

    if trkn >= 0xB00000 and  trkn < 0xB10000:
        return 102        # weather
    elif trkn >= 0xB10000 and trkn < 0xB20000:
        return 103        # Migratory Bird
    elif trkn >= 0xB20000 and  trkn < 0xB30000:
        return 104        # Predatory Bird
    elif trkn <= 0x003FFF:
        return 101       # drone
    elif trkn >= 0xA00000:
        return 100       # aircraft
    else:
        return 99        # dummy it for now

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
                                                     ("show_threat_radius_clear", bool, False),
                                                     ("show_callsign", bool, True),
                                                     ("traffic_warning", bool, False),
                                                     ("alt_color1", str, "blue"),
                                                     ("alt_color2", str, "red"),
                                                     ("alt_color_alt_thresh", int, 300),
                                                     ("alt_color_dist_thresh", int, 3000)])
        self.add_completion_function('(ADSBSETTING)',
                                     self.ADSB_settings.completion)
        
        self.threat_detection_timer = mavutil.periodic_event(2)
        self.threat_timeout_timer = mavutil.periodic_event(2)
        self.tnow = self.get_time()
        self.last_traffic = self.tnow

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

    def add_vehicle(self, state):
        '''handle an incoming vehicle packet'''
        id = 'ADSB-' + str(state['ICAO_address'])
        lat = state['lat']
        lon = state['lon']
        altitude_km = state['altitude']
        callsign = state['callsign']
        heading = state['heading']
        emitter_type = get_internal_emitter(state['ICAO_address'])
        squawk = state['squawk']

        if id not in self.threat_vehicles.keys():  # check to see if the vehicle is in the dict
            #print("NEW: ", state)
            # if not then add it
            self.threat_vehicles[id] = ADSBVehicle(id=id, state=state)
            #print("NEW: ", state)
            for mp in self.module_matching('map*'):
                from MAVProxy.modules.lib import mp_menu
                from MAVProxy.modules.mavproxy_map import mp_slipmap
                self.threat_vehicles[id].menu_item = mp_menu.MPMenuItem(name=id, returnkey=None)

                threat_radius = get_threat_radius(emitter_type, squawk)
                selected_icon = get_threat_icon(emitter_type, self.threat_vehicles[id].icon)
                    
                if selected_icon is not None:
                    #print("map add ", state)
                    # draw the vehicle on the map
                    popup = mp_menu.MPMenuSubMenu('ADSB', items=[self.threat_vehicles[id].menu_item])
                    icon = mp.map.icon(selected_icon)
                    mp.map.add_object(mp_slipmap.SlipIcon(id, (lat * 1e-7, lon * 1e-7),
                                                        icon, layer=3, rotation=heading*0.01, follow=False,
                                                        trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                                                        popup_menu=popup))
                if threat_radius > 0:
                    mp.map.add_object(mp_slipmap.SlipCircle(id+":circle", 3,
                                                        (lat * 1e-7, lon * 1e-7),
                                                        threat_radius, (0, 255, 255), linewidth=1))
        else:  # the vehicle is in the dict
            # update the dict entry
            self.threat_vehicles[id].update(state, self.get_time())

        for mp in self.module_matching('map*'):
            # update the map, labelling alt above/below our alt
            GPI = self.master.messages.get("GLOBAL_POSITION_INT", None)
            if GPI is None:
                return
            ref_alt = GPI.alt*0.001
            lat_deg = lat * 1.0e-7
            lon_deg = lon * 1.0e-7
            our_lat_deg = GPI.lat*1.0e-7
            our_lon_deg = GPI.lon*1.0e-7

            dist = mp_util.gps_distance(our_lat_deg, our_lon_deg, lat_deg, lon_deg)
            alt_amsl = altitude_km * 0.001
            color = ImageColor.getrgb(self.ADSB_settings.alt_color1)
            label = ""
            if self.ADSB_settings.show_callsign and (emitter_type < 14 or emitter_type == 100 or emitter_type == 101):
                label = "[%s] " % callsign.rstrip()
            if alt_amsl > 0:
                alt = int(alt_amsl - ref_alt)
                label += self.height_string(alt)
                label += " (AMSL: %s) " % self.height_string(alt_amsl)
                if abs(dist) < get_threat_radius(emitter_type, squawk) and abs(alt) < get_threat_height(emitter_type):
                    tnow = self.get_time()
                    if self.ADSB_settings.traffic_warning and tnow - self.last_traffic > 5:
                        self.last_traffic = tnow
                        self.say("traffic")
                    color = ImageColor.getrgb(self.ADSB_settings.alt_color2)

            mp.map.set_position(id, (lat_deg, lon_deg), rotation=heading*0.01, label=label, colour=color)
            mp.map.set_position(id+":circle", (lat_deg, lon_deg))

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "ADSB_VEHICLE":
            state = m.to_dict()
            self.add_vehicle(state)

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
