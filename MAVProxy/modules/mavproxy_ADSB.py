'''Support for ADS-B data'''

import time
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import * #popup menus
from pymavlink import mavextra


class ADSBVehicle(object):
    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.icon = 'greenplane'# use plane icon for now
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
        super(ADSBModule, self).__init__(mpstate, "ADSB", "ADS-B data support")
        self.threat_vehicles = {}
        
        self.add_command('ADSB', self.cmd_ADSB, ["ADSB control",
                                                 "<status>", 
                                                 "set (ADSBSETTING)"])
        
        self.ADSB_settings = mp_settings.MPSettings([ ("timeout", int, 10), #seconds
                                                      ("threat_radius", int, 200), #meters
                                                      ("show_threat_radius", bool, True),
                                                      ("threat_radius_clear_multiplier", int, 2), #threat_radius_clear = threat_radius*threat_radius_clear_multiplier
                                                      ("show_threat_radius_clear", bool, True)]) 
        
        
    def cmd_ADSB(self, args):
        '''ADSB command parser'''
        usage = "usage: ADSB <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print("threat count: %u" % (len(self.threat_vehicles)))
            for id in self.threat_vehicles.keys():
                print("id: %s  distance: %.2f callsign: %s  tslc: %u  lat: %.5f  lon: %.5f  alt: %.2f  cog: %u" % (id,
                                                                                   self.threat_vehicles[id].distance,
                                                                                   self.threat_vehicles[id].state['callsign'],
                                                                                   self.threat_vehicles[id].state['tslc'],     
                                                                                   self.threat_vehicles[id].state['lat'] * 1e-7,
                                                                                   self.threat_vehicles[id].state['lon'] * 1e-7,
                                                                                   self.threat_vehicles[id].state['altitude'],
                                                                                   self.threat_vehicles[id].state['heading']))
        elif args[0] == "set":
            self.ADSB_settings.command(args[1:])
        else:
            print(usage)
            
    def update_threat_distances(self, latlonalt):
        '''update the distance between threats and vehicle'''
        for id in self.threat_vehicles.keys():
            threat_latlonalt = (self.threat_vehicles[id].state['lat'] * 1e-7,
                                self.threat_vehicles[id].state['lon'] * 1e-7,
                                self.threat_vehicles[id].state['altitude'])
                                                                              
            self.threat_vehicles[id].h_distance = self.get_h_distance(latlonalt, threat_latlonalt)
            self.threat_vehicles[id].v_distance = self.get_v_distance(latlonalt, threat_latlonalt)
            
            #calculate and set the total distance between threat and vehicle
            self.threat_vehicles[id].distance = sqrt(self.threat_vehicles[id].h_distance**2 + (self.threat_vehicles[id].v_distance)**2)
            
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
        
        a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
        c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
        return 6371 * 1000 * c
    
    def get_v_distance(self, latlonalt1, latlonalt2):
        '''get the horizontal distance between threat and vehicle'''
        (lat1, lon1, alt1) = latlonalt1
        (lat2, lon2, alt2) = latlonalt2
        return alt2 - alt1
    
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "ADSB_VEHICLE":
            
            id = 'ADSB-' + str(m.ICAO_address)
            if id not in self.threat_vehicles.keys(): #check to see if the vehicle is in the dict
                #if not then add it
                self.threat_vehicles[id] = ADSBVehicle(id = id, state = m.to_dict())
                if self.mpstate.map: #if the map is loaded...
                    icon = self.mpstate.map.icon(self.threat_vehicles[id].icon + '.png')
                    popup = MPMenuSubMenu('ADSB',
                                  items=[MPMenuItem(name=id, returnkey=None)])
                    #draw the vehicle on the map
                    self.mpstate.map.add_object(mp_slipmap.SlipIcon(id, (m.lat*1e-7,m.lon*1e-7), icon, layer=3, rotation=m.heading, follow=False,
                                                trail=mp_slipmap.SlipTrail(colour=(0, 255, 255)), popup_menu=popup))
            else: #the vehicle is in the dict
                #update the dict entry
                self.threat_vehicles[id].update(m.to_dict())
                if self.mpstate.map: #if the map is loaded...
                    #update the map
                    self.mpstate.map.set_position(id, (m.lat*1e-7, m.lon*1e-7), rotation=m.heading)
                    
        elif m.get_type() == "GLOBAL_POSITION_INT":
            if self.mpstate.map:
                #update the threat circle on the map
                threat_circle = mp_slipmap.SlipCircle("threat_circle", 3, (m.lat*1e-7, m.lon*1e-7), self.ADSB_settings.threat_radius,
                                                              (0, 255, 255), linewidth=1, popup_menu=None)
                threat_circle.set_hidden(not self.ADSB_settings.show_threat_radius) #show the circle?
                self.mpstate.map.add_object(threat_circle)
                
                #update the threat clear circle on the map
                threat_radius_clear = self.ADSB_settings.threat_radius*self.ADSB_settings.threat_radius_clear_multiplier
                threat_clear_circle = mp_slipmap.SlipCircle("threat_clear_circle", 3, (m.lat*1e-7, m.lon*1e-7),threat_radius_clear,
                                                              (0, 255, 255), linewidth=1, popup_menu=None)
                threat_clear_circle.set_hidden(not self.ADSB_settings.show_threat_radius_clear) #show the circle?
                self.mpstate.map.add_object(threat_clear_circle)
            
            #we assume this is handled much more oftern than ADS-B messages
            #so update the distance between vehicle and threat here
            self.update_threat_distances((m.lat*1e-7, m.lon*1e-7, m.alt*1e-3))  
        
        else:
            pass
                    
    def idle_task(self):
        '''called on idle'''
        current_time = time.time()
        threat_radius_clear = self.ADSB_settings.threat_radius*self.ADSB_settings.threat_radius_clear_multiplier
        
        for id in self.threat_vehicles.keys():
            
            if self.threat_vehicles[id].distance is not None:
                if self.threat_vehicles[id].distance <= self.ADSB_settings.threat_radius and not self.threat_vehicles[id].is_evading_threat:
                    #if the threat is in the threat radius and not currently known to the module...
                    self.threat_vehicles[id].is_evading_threat = True #set flag to action threat
                    
                if self.threat_vehicles[id].distance > threat_radius_clear and self.threat_vehicles[id].is_evading_threat:
                    #if the threat is known to the module and outside the threat clear radius...
                    self.threat_vehicles[id].is_evading_threat = False #clear flag to action threat
                    
            if current_time - self.threat_vehicles[id].update_time > self.ADSB_settings.timeout:
                #if the treat has timed out...
                del self.threat_vehicles[id] #remove the threat from the dict
                if self.mpstate.map:
                    self.mpstate.map.remove_object(id) #remove the threat from the map
                    
                
def init(mpstate):
    '''initialise module'''
    return ADSBModule(mpstate)
