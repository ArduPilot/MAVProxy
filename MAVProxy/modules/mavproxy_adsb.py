'''
Support for ADS-B data
Samuel Dudley
Dec 2015
'''

import time
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil


class ADSBVehicle(object):
    '''a generic ADS-B threat'''

    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.vehicle_colour = 'green'  # use plane icon for now
        self.vehicle_type = 'plane'
        self.icon = self.vehicle_colour + self.vehicle_type + '.png'
        self.update_time = time.time()
        self.is_evading_threat = False
        self.v_distance = None
        self.h_distance = None
        self.distance = None

    def update(self, state):
        '''update the threat state'''
        self.state = state
        self.update_time = time.time()


class ADSBModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(ADSBModule, self).__init__(mpstate, "adsb", "ADS-B data support")
        self.threat_vehicles = {}
        self.active_threat_ids = []  # holds all threat ids the vehicle is evading

        self.add_command('adsb', self.cmd_ADSB, ["adsb control",
                                                 "<status>",
                                                 "set (ADSBSETTING)"])

        self.ADSB_settings = mp_settings.MPSettings([("timeout", int, 10),  # seconds
                                                     ("threat_radius", int, 200),  # meters
                                                     ("show_threat_radius", bool, False),
                                                     # threat_radius_clear = threat_radius*threat_radius_clear_multiplier
                                                     ("threat_radius_clear_multiplier", int, 2),
                                                     ("show_threat_radius_clear", bool, False)])
        self.threat_detection_timer = mavutil.periodic_event(2)
        self.threat_timeout_timer = mavutil.periodic_event(2)

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
        current_time = time.time()
        for id in self.threat_vehicles.keys():
            if current_time - self.threat_vehicles[id].update_time > self.ADSB_settings.timeout:
                # if the threat has timed out...
                del self.threat_vehicles[id]  # remove the threat from the dict
                if self.mpstate.map:
                    # remove the threat from the map
                    self.mpstate.map.remove_object(id)
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
                if self.mpstate.map:  # if the map is loaded...
                    icon = self.mpstate.map.icon(self.threat_vehicles[id].icon)
                    popup = MPMenuSubMenu('ADSB', items=[MPMenuItem(name=id, returnkey=None)])
                    # draw the vehicle on the map
                    self.mpstate.map.add_object(mp_slipmap.SlipIcon(id, (m.lat * 1e-7, m.lon * 1e-7),
                                                                    icon, layer=3, rotation=m.heading*0.01, follow=False,
                                                                    trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)),
                                                                    popup_menu=popup))
            else:  # the vehicle is in the dict
                # update the dict entry
                self.threat_vehicles[id].update(m.to_dict())
                if self.mpstate.map:  # if the map is loaded...
                    # update the map
                    self.mpstate.map.set_position(id, (m.lat * 1e-7, m.lon * 1e-7), rotation=m.heading*0.01)

        if m.get_type() == "GLOBAL_POSITION_INT":
            if self.mpstate.map:
                if len(self.active_threat_ids) > 0:
                    threat_circle_width = 2
                else:
                    threat_circle_width = 1
                # update the threat circle on the map
                threat_circle = mp_slipmap.SlipCircle("threat_circle", 3,
                                                      (m.lat * 1e-7, m.lon * 1e-7),
                                                      self.ADSB_settings.threat_radius,
                                                      (0, 255, 255), linewidth=threat_circle_width)
                threat_circle.set_hidden(
                    not self.ADSB_settings.show_threat_radius)  # show the circle?
                self.mpstate.map.add_object(threat_circle)

                # update the threat clear circle on the map
                threat_radius_clear = self.ADSB_settings.threat_radius * \
                    self.ADSB_settings.threat_radius_clear_multiplier
                threat_clear_circle = mp_slipmap.SlipCircle("threat_clear_circle", 3,
                                                            (m.lat * 1e-7,
                                                             m.lon * 1e-7),
                                                            threat_radius_clear,
                                                            (0, 255, 255), linewidth=1)
                # show the circle?
                threat_clear_circle.set_hidden(not self.ADSB_settings.show_threat_radius_clear)
                self.mpstate.map.add_object(threat_clear_circle)

            # we assume this is handled much more oftern than ADS-B messages
            # so update the distance between vehicle and threat here
            self.update_threat_distances((m.lat * 1e-7, m.lon * 1e-7, m.alt * 1e-3))

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
