"""
  MAVProxy console

  uses lib/console.py for display
"""

from MAVProxy.modules.lib import wxhorizon
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.wxhorizon_util import Attitude, VFR_HUD, Global_Position_INT, BatteryInfo, FlightState, WaypointInfo, FPS

import time

class HorizonModule(mp_module.MPModule):
    def __init__(self, mpstate):
        # Define module load/unload reference and window title
        super(HorizonModule, self).__init__(mpstate, "horizon", "Horizon Indicator", public=True)
        self.mpstate.horizonIndicator = wxhorizon.HorizonIndicator(title='Horizon Indicator')
        self.mode = ''
        self.armed = ''
        self.currentWP = 0
        self.finalWP = 0
        self.currentDist = 0
        self.nextWPTime = 0
        self.speed = 0
        self.wpBearing = 0
        self.msgList = []
        self.lastSend = 0.0
        self.fps = 10.0
        self.sendDelay = (1.0/self.fps)*0.9
        self.add_command('horizon-fps',self.fpsInformation,"Get or change frame rate for horizon. Usage: horizon-fps set <fps>, horizon-fps get. Set fps to zero to get unrestricted framerate.")
        
    def unload(self):
        '''unload module'''
        self.mpstate.horizonIndicator.close()
            
    def fpsInformation(self,args):
        '''fps command'''
        invalidStr = 'Invalid number of arguments. Usage horizon-fps set <fps> or horizon-fps get. Set fps to zero to get unrestricted framerate.'
        if len(args)>0:
            if args[0] == "get":
                '''Get the current framerate.'''
                if (self.fps == 0.0):
                    print('Horizon Framerate: Unrestricted')
                else:
                    print("Horizon Framerate: " + str(self.fps))
            elif args[0] == "set":
                if len(args)==2:
                    self.fps = float(args[1])
                    if (self.fps != 0):
                        self.sendDelay = 1.0/self.fps
                    else:
                        self.sendDelay = 0.0
                    self.msgList.append(FPS(self.fps))
                    if (self.fps == 0.0):
                        print('Horizon Framerate: Unrestricted')
                    else:
                        print("Horizon Framerate: " + str(self.fps))
                else:
                    print(invalidStr)
            else:
                print(invalidStr)
        else:
            print(invalidStr)
            
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        msgType = msg.get_type()
        master = self.master       
        if msgType == 'HEARTBEAT':
            # Update state and mode information
            self.armed = master.motors_armed()
            self.mode = master.flightmode
            # Send Flight State information down pipe
            self.msgList.append(FlightState(self.mode,self.armed))
        elif msgType == 'ATTITUDE':
            # Send attitude information down pipe
            self.msgList.append(Attitude(msg))
        elif msgType == 'VFR_HUD':
            # Send HUD information down pipe
            self.msgList.append(VFR_HUD(msg))
        elif msgType == 'GLOBAL_POSITION_INT':
            # Send altitude information down pipe
            self.msgList.append(Global_Position_INT(msg,time.time()))
        elif msgType == 'SYS_STATUS':
            # Mode and Arm State
            self.msgList.append(BatteryInfo(msg))
        elif msgType in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
            # Waypoints
            self.currentWP = msg.seq
            self.finalWP = self.module('wp').wploader.count()
            self.msgList.append(WaypointInfo(self.currentWP,self.finalWP,self.currentDist,self.nextWPTime,self.wpBearing))
        elif msgType == 'NAV_CONTROLLER_OUTPUT':
            self.currentDist = msg.wp_dist
            self.speed = master.field('VFR_HUD', 'airspeed', 30)
            if self.speed > 1:
                self.nextWPTime = self.currentDist / self.speed
            else:
                self.nextWPTime = '-'
            self.wpBearing = msg.target_bearing
            self.msgList.append(WaypointInfo(self.currentWP,self.finalWP,self.currentDist,self.nextWPTime,self.wpBearing))

    
    def idle_task(self):
        if self.mpstate.horizonIndicator.close_event.wait(0.001):
            self.needs_unloading = True   # tell MAVProxy to unload this module
    
        if (time.time() - self.lastSend) > self.sendDelay:
            self.mpstate.horizonIndicator.parent_pipe_send.send(self.msgList)
            self.msgList = []
            self.lastSend = time.time()
    
def init(mpstate):
    '''initialise module'''
    return HorizonModule(mpstate)
