"""
  MAVProxy console

  uses lib/console.py for display
"""

from MAVProxy.modules.lib import wxhorizon
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.wxhorizon_util import Attitude, VFR_HUD, Global_Position_INT, BatteryInfo, FlightState, WaypointInfo


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
        
    def unload(self):
        '''unload module'''
        self.mpstate.horizonIndicator.close()
            
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        msgType = msg.get_type()
        master = self.master
        if msgType == 'HEARTBEAT':
            # Update state and mode information
            if type(master.motors_armed()) == type(True):
                self.armed = master.motors_armed()
                self.mode = master.flightmode
                # Send Flight State information down pipe
                self.mpstate.horizonIndicator.parent_pipe_send.send(FlightState(self.mode,self.armed))
        elif msgType == 'ATTITUDE':
            # Send attitude information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(Attitude(msg))
        elif msgType == 'VFR_HUD':
            # Send HUD information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(VFR_HUD(msg))
        elif msgType == 'GLOBAL_POSITION_INT':
            # Send altitude information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(Global_Position_INT(msg))
        elif msgType == 'SYS_STATUS':
            # Mode and Arm State
            self.mpstate.horizonIndicator.parent_pipe_send.send(BatteryInfo(msg))
        elif msgType in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
            # Waypoints
            self.currentWP = msg.seq
            self.finalWP = self.module('wp').wploader.count()
            self.mpstate.horizonIndicator.parent_pipe_send.send(WaypointInfo(self.currentWP,self.finalWP,self.currentDist,self.nextWPTime,self.wpBearing))
        elif msgType == 'NAV_CONTROLLER_OUTPUT':
            self.currentDist = msg.wp_dist
            self.speed = master.field('VFR_HUD', 'airspeed', 30)
            if self.speed > 1:
                self.nextWPTime = self.currentDist / self.speed
            else:
                self.nextWPTime = '-'
            self.wpBearing = msg.target_bearing
            self.mpstate.horizonIndicator.parent_pipe_send.send(WaypointInfo(self.currentWP,self.finalWP,self.currentDist,self.nextWPTime,self.wpBearing))
        
def init(mpstate):
    '''initialise module'''
    return HorizonModule(mpstate)
