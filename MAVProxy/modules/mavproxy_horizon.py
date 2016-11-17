"""
  MAVProxy console

  uses lib/console.py for display
"""

from MAVProxy.modules.lib import wxhorizon
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.wxhorizon_util import Attitude, VFR_HUD, Global_Position_INT, BatteryInfo


class HorizonModule(mp_module.MPModule):
    def __init__(self, mpstate):
        # Define module load/unload reference and window title
        super(HorizonModule, self).__init__(mpstate, "horizon", "Horizon Indicator", public=True)
        self.mpstate.horizonIndicator = wxhorizon.HorizonIndicator(title='Horizon Indicator')

    def unload(self):
        '''unload module'''
        self.mpstate.horizonIndicator.close()

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        msgType = msg.get_type()
        if msgType == 'ATTITUDE':
            # Send attitude information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(Attitude(msg))
        elif msgType == 'VFR_HUD':
            # Send HUD information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(VFR_HUD(msg))
        elif msgType == 'GLOBAL_POSITION_INT':
            # Send altitude information down pipe
            self.mpstate.horizonIndicator.parent_pipe_send.send(Global_Position_INT(msg))
        elif msgType == 'SYS_STATUS':
            self.mpstate.horizonIndicator.parent_pipe_send.send(BatteryInfo(msg))
        
def init(mpstate):
    '''initialise module'''
    return HorizonModule(mpstate)
