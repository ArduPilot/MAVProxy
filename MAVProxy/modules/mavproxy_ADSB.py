'''Support for ADS-B data'''

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import * #popup menus

class ADSB_Vehicle(object):
    def __init__(self, id, state ):
        self.id = id
        self.state = state
        self.update_time = time.time()
        
    def update(self, state):
        self.state = state
        self.update_time = time.time()
        
class ADSBModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ADSBModule, self).__init__(mpstate, "ADSB", "ADS-B data support")
        self.threat_vehicles = {}
        self.add_command('ADSB', self.cmd_ADSB, ["ADSB control",
                                                 "<status>", 
                                                 "set (ADSBSETTING)"])
        
        self.ADSB_settings = mp_settings.MPSettings([ ("timeout", int, 10) ])#seconds
        
    def cmd_ADSB(self, args):
        '''ADSB command parser'''
        usage = "usage: ADSB <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print("threat count: %u" % (len(self.threat_vehicles)))
            for id in self.threat_vehicles.keys():
                print("id: %s  callsign: %s  tslc: %u  lat: %.5f  lon: %.5f  alt: %.2f  cog: %u" % (id,
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
    
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "ADSB_VEHICLE":
            id = 'ADSB-' + str(m.ICAO_address)
            if id not in self.threat_vehicles.keys(): #check to see if the vehicle is in the dict
                #if not then add it
                self.threat_vehicles[id] = ADSB_Vehicle(id = id, state = m.to_dict())
                if self.mpstate.map: #if the map is loaded...
                    icon = self.mpstate.map.icon('greenplane' + '.png')# use plane icon for now
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
    
                
    def idle_task(self):
        '''called on idle'''
        current_time = time.time()
        for id in self.threat_vehicles.keys():
            if current_time - self.threat_vehicles[id].update_time > self.ADSB_settings.timeout:
                del self.threat_vehicles[id] #remove the threat from the dict
                if self.mpstate.map: #if the map is loaded...
                    self.mpstate.map.remove_object(id) #remove the threat from the map
            

def init(mpstate):
    '''initialise module'''
    return ADSBModule(mpstate)
