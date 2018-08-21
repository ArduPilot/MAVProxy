'''
generate dynamic obstacles for OBC 2018
'''

import time, pickle
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

import socket, time, random, math

track_count = 0

# object types
DNFZ_types = {
    'Aircraft' : 1,
    'Weather' : 20000,
    'BirdMigrating' : 30000,
    'BirdOfPrey' : 40000
}

gen_settings = mp_settings.MPSettings([("port", int, 45454),
                                       ('debug', int, 0),
                                       ('home_lat', float, -27.298440),
                                       ('home_lon', float, 151.290775),
                                       ('region_width', float, 15000),
                                       ('ground_height', float, 1150),
                                       ('num_aircraft', int, 5),
                                       ('num_bird_prey', int, 5),
                                       ('num_bird_migratory', int, 5),
                                       ('num_weather', int, 5),
                                       ('stop', int, 0)])
                                       
    
class DNFZ:
    '''a dynamic no-fly zone object'''
    def __init__(self, DNFZ_type):
        if not DNFZ_type in DNFZ_types:
            raise('Bad DNFZ type %s' % DNFZ_type)
        self.DNFZ_type = DNFZ_type
        self.pkt = {'category': 0, 'I010': {'SAC': {'val': 4, 'desc': 'System Area Code'}, 'SIC': {'val': 0, 'desc': 'System Identification Code'}}, 'I040': {'TrkN': {'val': 0, 'desc': 'Track number'}}, 'ts': 0, 'len': 25, 'I220': {'RoC': {'val': 0.0, 'desc': 'Rate of Climb/Descent'}}, 'crc': 'B52DA163', 'I130': {'Alt': {'max': 150000.0, 'min': -1500.0, 'val': 0.0, 'desc': 'Altitude'}}, 'I070': {'ToT': {'val': 0.0, 'desc': 'Time Of Track Information'}}, 'I105': {'Lat': {'val': 0, 'desc': 'Latitude in WGS.84 in twos complement. Range -90 < latitude < 90 deg.'}, 'Lon': {'val': 0.0, 'desc': 'Longitude in WGS.84 in twos complement. Range -180 < longitude < 180 deg.'}}, 'I080': {'SRC': {'meaning': '3D radar', 'val': 2, 'desc': 'Source of calculated track altitude for I062/130'}, 'FX': {'meaning': 'end of data item', 'val': 0, 'desc': ''}, 'CNF': {'meaning': 'Confirmed track', 'val': 0, 'desc': ''}, 'SPI': {'meaning': 'default value', 'val': 0, 'desc': ''}, 'MRH': {'meaning': 'Geometric altitude more reliable', 'val': 1, 'desc': 'Most Reliable Height'}, 'MON': {'meaning': 'Multisensor track', 'val': 0, 'desc': ''}}}
        self.speed = 0.0 # m/s
        self.heading = 0.0 # degrees
        self.desired_heading = None
        self.yawrate = 0.0
        # random initial position and heading
        self.randpos()
        self.setheading(random.uniform(0,360))
        global track_count
        track_count += 1
        self.pkt['I040']['TrkN']['val'] = DNFZ_types[self.DNFZ_type] + track_count
        if gen_settings.debug > 0:
            print("track %u" % self.pkt['I040']['TrkN']['val'])

    def distance_from(self, lat, lon):
        '''get distance from a point'''
        lat1 = self.pkt['I105']['Lat']['val']
        lon1 = self.pkt['I105']['Lon']['val']
        return mp_util.gps_distance(lat1, lon1, lat, lon)
        
    def distance_from_home(self):
        '''get distance from home'''
        return self.distance_from(gen_settings.home_lat, gen_settings.home_lon)
        
    def randpos(self):
        '''random initial position'''
        self.setpos(gen_settings.home_lat, gen_settings.home_lon)
        self.move(random.uniform(0, 360), random.uniform(0, gen_settings.region_width))

    def randalt(self):
        '''random initial position'''
        self.setalt(gen_settings.ground_height + random.uniform(100, 1500))

    def move(self, bearing, distance):
        '''move position by bearing and distance'''
        lat = self.pkt['I105']['Lat']['val']
        lon = self.pkt['I105']['Lon']['val']
        (lat, lon) = mp_util.gps_newpos(lat, lon, bearing, distance)
        self.setpos(lat, lon)
        
    def setpos(self, lat, lon):
        self.pkt['I105']['Lat']['val'] = lat
        self.pkt['I105']['Lon']['val'] = lon

    def getalt(self):
        return self.pkt['I130']['Alt']['val']
        
    def setalt(self, alt):
        self.pkt['I130']['Alt']['val'] = alt

    def setclimbrate(self, climbrate):
        self.pkt['I220']['RoC']['val'] = climbrate

    def setyawrate(self, yawrate):
        self.yawrate = yawrate
        
    def setspeed(self, speed):
        self.speed = speed

    def setheading(self, heading):
        self.heading = heading
        while self.heading > 360:
            self.heading -= 360.0
        while self.heading < 0:
            self.heading += 360.0

    def move(self, heading, distance):
        lat = self.pkt['I105']['Lat']['val']
        lon = self.pkt['I105']['Lon']['val']
        (lat, lon) = mp_util.gps_newpos(lat, lon, heading, distance)
        self.setpos(lat, lon)        

    def changealt(self, delta_alt):
        alt = self.pkt['I130']['Alt']['val']
        alt += delta_alt
        self.setalt(alt)

    def rate_of_turn(self, bank=45.0):
        '''return expected rate of turn in degrees/s for given speed in m/s and bank angle in degrees'''
        if abs(self.speed) < 2 or abs(bank) > 80:
            return 0
        ret = degrees(9.81*tan(radians(bank))/self.speed)
        return ret
        
    def update(self, deltat=1.0):
        self.move(self.heading, self.speed * deltat)
        climbrate = self.pkt['I220']['RoC']['val']
        self.changealt(climbrate * deltat)
        if self.desired_heading is None:
            self.setheading(self.heading + self.yawrate * deltat)
        else:
            heading_error = self.desired_heading - self.heading
            while heading_error > 180:
                heading_error -= 360.0
            while heading_error < -180:
                heading_error += 360.0            
            max_turn = self.rate_of_turn() * deltat
            if heading_error > 0:
                turn = min(max_turn, heading_error)
            else:
                turn = max(-max_turn, heading_error)
            self.setheading(self.heading + turn)
            if abs(heading_error) < 0.01:
                self.desired_heading = None

    def __str__(self):
        return str(self.pkt)

    def pickled(self):
        return pickle.dumps(self.pkt)

class Aircraft(DNFZ):
    '''an aircraft that flies in a circuit'''
    def __init__(self, speed=30.0, circuit_width=1000.0):
        DNFZ.__init__(self, 'Aircraft')
        self.setspeed(speed)
        self.circuit_width = circuit_width
        self.dist_flown = 0
        self.randalt()

    def update(self, deltat=1.0):
        '''fly a square circuit'''
        DNFZ.update(self, deltat)
        self.dist_flown += self.speed * deltat
        if self.dist_flown > self.circuit_width:
            self.desired_heading = self.heading + 90
            self.dist_flown = 0
        if self.getalt() < gen_settings.ground_height:
            self.randpos()
            self.randalt()

class BirdOfPrey(DNFZ):
    '''an bird that circles slowly climbing, then dives'''
    def __init__(self):
        DNFZ.__init__(self, 'BirdOfPrey')
        self.setspeed(16.0)
        self.radius = random.uniform(100,200)
        self.time_circling = 0
        self.dive_rate = -30
        self.climb_rate = 5
        self.drift_speed = random.uniform(5,10)
        self.max_alt = gen_settings.ground_height + random.uniform(100, 400)
        self.drift_heading = self.heading
        circumference = math.pi * self.radius * 2
        circle_time = circumference / self.speed
        self.turn_rate = 360.0 / circle_time
        if random.uniform(0,1) < 0.5:
            self.turn_rate = -self.turn_rate

    def update(self, deltat=1.0):
        '''fly circles, then dive'''
        DNFZ.update(self, deltat)
        self.time_circling += deltat
        self.setheading(self.heading + self.turn_rate * deltat)
        self.move(self.drift_heading, self.drift_speed)
        if self.getalt() > self.max_alt or self.getalt() < gen_settings.ground_height:
            if self.getalt() > gen_settings.ground_height:
                self.setclimbrate(self.dive_rate)
            else:
                self.setclimbrate(self.climb_rate)
        if self.getalt() < gen_settings.ground_height:
            self.setalt(gen_settings.ground_height)
        if self.distance_from_home() > gen_settings.region_width:
            self.randpos()
            self.randalt()

class BirdMigrating(DNFZ):
    '''an bird that circles slowly climbing, then dives'''
    def __init__(self):
        DNFZ.__init__(self, 'BirdMigrating')
        self.setspeed(random.uniform(4,16))
        self.setyawrate(random.uniform(-0.2,0.2))
        
    def update(self, deltat=1.0):
        '''fly in long curves'''
        DNFZ.update(self, deltat)
        if self.distance_from_home() > gen_settings.region_width or self.getalt() < gen_settings.ground_height:
            self.randpos()
            self.randalt()

class Weather(DNFZ):
    '''a weather system'''
    def __init__(self):
        DNFZ.__init__(self, 'Weather')
        self.setspeed(random.uniform(1,4))
        self.lifetime = random.uniform(300,600)
        self.setalt(0)
        
    def update(self, deltat=1.0):
        '''straight lines, with short life'''
        DNFZ.update(self, deltat)
        self.lifetime -= deltat
        if self.lifetime <= 0:
            self.randpos()
            self.lifetime = random.uniform(300,600)

class GenobstaclesModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(GenobstaclesModule, self).__init__(mpstate, "genobstacles", "OBC 2018 obstacle generator")

        self.add_command('genobstacles', self.cmd_genobstacles, "obstacle generator",
                         ["<start|stop>","set (GENSETTING)"])

        self.add_completion_function('(GENSETTING)',
                                     gen_settings.completion)
        self.sock = None
        self.aircraft = []
        self.last_t = 0
        self.start()
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Obstacles',
                                    items=[MPMenuItem('Restart', 'Restart', '# genobstacles restart'),
                                           MPMenuItem('Stop', 'Stop', '# genobstacles stop'),
                                           MPMenuItem('Start','Start', '# genobstacles start'),
                                           MPMenuItem('Remove','Remove', '# genobstacles remove'),
                                           MPMenuItem('Drop Cloud','DropCloud', '# genobstacles dropcloud'),
                                           MPMenuItem('Drop Eagle','DropEagle', '# genobstacles dropeagle'),
                                           MPMenuItem('Drop Bird','DropBird', '# genobstacles dropbird'),
                                           MPMenuItem('Drop Plane','DropPlane', '# genobstacles dropplane')])

    def cmd_dropobject(self, obj):
        '''drop an object on the map'''
        latlon = self.module('map').click_position
        if latlon is not None:
            obj.setpos(latlon[0], latlon[1])
            self.aircraft.append(obj)
        

    def cmd_genobstacles(self, args):
        '''genobstacles command parser'''
        usage = "usage: genobstacles <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            gen_settings.command(args[1:])
        elif args[0] == "start":
            self.start()
        elif args[0] == "stop":
            self.stop()
        elif args[0] == "restart":
            self.stop()
            self.start()
        elif args[0] == "remove":
            latlon = self.module('map').click_position
            if latlon is not None:
                closest = None
                closest_distance = 1000
                for a in self.aircraft:
                    dist = a.distance_from(latlon[0], latlon[1])
                    if dist < closest_distance:
                        closest_distance = dist
                        closest = a
                if closest is not None:
                    self.aircraft.remove(closest)
                else:
                    print("No obstacle found at click point")
                    
        elif args[0] == "dropcloud":
            self.cmd_dropobject(Weather())
        elif args[0] == "dropeagle":
            self.cmd_dropobject(BirdOfPrey())
        elif args[0] == "dropbird":
            self.cmd_dropobject(BirdMigrating())
        elif args[0] == "dropplane":
            self.cmd_dropobject(Aircraft())
        else:
            print(usage)

    def start(self):
        '''start sending packets'''
        if self.sock is not None:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.connect(('', gen_settings.port))

        global track_count
        self.aircraft = []
        track_count = 0
        self.last_t = 0

        # some fixed wing aircraft
        for i in range(gen_settings.num_aircraft):
            self.aircraft.append(Aircraft(random.uniform(10, 100), 2000.0))

        # some birds of prey
        for i in range(gen_settings.num_bird_prey):
            self.aircraft.append(BirdOfPrey())

        # some migrating birds
        for i in range(gen_settings.num_bird_migratory):
            self.aircraft.append(BirdMigrating())

        # some weather systems
        for i in range(gen_settings.num_weather):
            self.aircraft.append(Weather())
        print("Started on port %u" % gen_settings.port)

    def stop(self):
        '''stop listening for packets'''
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def mavlink_packet(self, m):
        '''trigger sends from ATTITUDE packets'''
        if m.get_type() != 'ATTITUDE':
            return
        t = m.time_boot_ms * 0.001
        dt = t - self.last_t
        if dt < 0 or dt > 10:
            self.last_t = t
            return
        if dt > 10 or dt < 0.9:
            return
        self.last_t = t
        for a in self.aircraft:
            if not gen_settings.stop:
                a.update(1.0)
            try:
                self.sock.send(a.pickled())
            except Exception as ex:
                pass
        if self.module('map') is not None and not self.menu_added_map:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)
        
        
def init(mpstate):
    '''initialise module'''
    return GenobstaclesModule(mpstate)
