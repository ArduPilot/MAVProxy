"""
    MAVProxy rally module
"""

from pymavlink import mavwp
import time
from MAVProxy.modules.lib import mp_module

class RallyModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RallyModule, self).__init__(mpstate, "rally", "rally point control", public = True)
        self.rallyloader = mavwp.MAVRallyLoader(mpstate.status.target_system, mpstate.status.target_component)
        self.add_command('rally', self.cmd_rally, "rally point control", ["<add|clear|list>",
                                    "<load|save> (FILENAME)"])

    def cmd_rally(self, args):
        '''rally point commands'''
        #TODO: add_land arg
        if(len(args) < 1):
            self.print_usage()
            return
    
        elif(args[0] == "add"):
            if (len(args) < 2):
                print("Usage: rally add ALT <BREAK_ALT> <LAND_HDG>")
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
    
            alt = float(args[1]);
            break_alt = 0.0
            land_hdg = 0.0;
            if (len(args) > 2):
                break_alt = float(args[2])
            if (len(args) > 3):
                land_hdg = float(args[3])
    
            self.rallyloader.create_and_append_rally_point(latlon[0] * 1e7, latlon[1] * 1e7, alt, break_alt, land_hdg, 0)
    
            self.send_rally_points();
    
            print("Added Rally point at %s %f" % (str(latlon), alt))
    
        elif(args[0] == "clear"):
            self.rallyloader.clear()
            self.mav_param.mavset(self.master,'RALLY_TOTAL',0,3)
    
        elif(args[0] == "list"):
            self.list_rally_points()
    
        elif(args[0] == "load"):
            if (len(args) < 2):
                print("Usage: rally load filename")
    
            try:
                self.rallyloader.load(args[1])
            except Exception, msg:
                print("Unable to load %s - %s" % (args[1], msg))
                return
        
            self.send_rally_points()
    
            print("Loaded %u rally points from %s" % (self.rallyloader.rally_count(), args[1]))
    
        elif(args[0] == "save"):
            if (len(args) < 2):
                print("Usage: rally save filename");
    
            self.rallyloader.save(args[1]);
    
            print "Saved rally file ", args[1];
    
        else:
            self.print_usage()
    
    def mavlink_packet(self, m):
        '''handle incoming mavlink packet'''
        return #TODO when applicable
    
    def send_rally_points(self):
        '''send rally points from fenceloader'''
        self.mav_param.mavset(self.master,'RALLY_TOTAL',self.rallyloader.rally_count(),3)
    
        for i in range(self.rallyloader.rally_count()):
            p = self.rallyloader.rally_point(i)
            self.master.mav.send(p)
           
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
        print("Usage: rally <list|load|save|add|clear>")
        
def init(mpstate):
    '''initialise module'''
    return RallyModule(mpstate)       
