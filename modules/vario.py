"""
  MAVProxy vario module
  Uses fluidsynth and soundfonts driving alsa
"""

import mavutil, re, os, sys, threading, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib', 'vario'))
import mp_vario
import varioSettings

        
def name():
    '''return module name'''
    return "vario"

def description():
    '''return module description'''
    return "variometer"

def cmd_vario(args):
    '''vario command'''
    return

#===============================================================================
#    state = mpstate.graph_state
# 
#    if len(args) == 0:
#        # list current graphs
#        for i in range(len(state.graphs)):
#            print("Graph %u: %s" % (i, state.graphs[i].fields))
#        return
# 
#    # start a new graph
#    state.graphs.append(Graph(args[:]))
#===============================================================================


def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate

    mpstate.vario_initialised = False

    try:
        settings_path = "unassigned"
        settings_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', "DefaultVarioSettings.xml")
 #        self.settings_path = os.path.join(self.application_path, "DefaultVarioSettings.xml")
        mpstate.varioSettings = varioSettings.parse(settings_path)
    except:
        print("Could not load vario settings at: " + settings_path)
        print("vario not initialised")
    else:

 #       rise_font_path = mpstate.varioSettings.get_risingSoundfont()
 #       fall_font_path = mpstate.varioSettings.get_fallingSoundfont()   
        
        rise_font_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', str(mpstate.varioSettings.get_risingSoundfont()) )
        fall_font_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', str(mpstate.varioSettings.get_fallingSoundfont()) )         
        
        if(not os.path.isfile(rise_font_path)):
            print("did not find soundfont path for rising at " + rise_font_path)
            return        
        if(not os.path.isfile(fall_font_path)):
            print("did not find soundfont path for falling at " + fall_font_path)
            return
          
        mpstate.vario = mp_vario.vario(mpstate, rise_font_path, fall_font_path)
    
                
        mpstate.vario.deadband = float(mpstate.varioSettings.get_Deadband()) * 100.0
        mpstate.vario.minRate = float(mpstate.varioSettings.get_maxFallRate()) * -100.0
        mpstate.vario.maxRate = float(mpstate.varioSettings.get_maxRiseRate()) * 100.0
#        mpstate.vario.maxRate = float(mpstate.varioSettings.m_textCtrlMaxRisingRate.GetValue()) * 100.0
        mpstate.vario.minFallingKey = int(mpstate.varioSettings.get_minFallingKey())
        value = int(str(mpstate.varioSettings.get_maxFallingKey()))
        mpstate.vario.maxFallingKey = value
        mpstate.vario.minRisingKey = int(mpstate.varioSettings.get_minRisingKey())
        value = int(mpstate.varioSettings.get_maxRisingKey())
        mpstate.vario.maxRisingKey = value
            
 #       mpstate.vario.setRisingSoundfont(str(mpstate.varioSettings.get_risingSoundfont()))
 #       mpstate.vario.setFallingSoundfont(str(mpstate.varioSettings.get_fallingSoundfont()))
    
        mpstate.vario_initialised = True
        print("vario initialised")

def unload():
    '''unload module'''
    mpstate.vario_initialised = False
    mpstate.vario.stop()

        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
                #===============================================================
                # if msg and msg.get_type() == "HEARTBEAT":
                #    print(msg)        
                #    system = msg.get_srcSystem()
                #    component = msg.get_srcComponent()
                #    print("Heartbeat from UDB (system %u component %u)" % (system, component))
                #    if((system == mpstate.system) and (component == mpstate.component)):
                #        mpstate.heartbeat_ok = True
                #        mpstate.heartbeat_timer = time.time()
                #===============================================================

    if msg and msg.get_type() == "GLOBAL_POSITION_INT":
        try:
            vz = msg.vz   # vertical velocity in cm/s
            vz = int(-vz)
        except:
            print("decode global vz message fail")
        try:
            if(mpstate.vario_initialised == True):
                mpstate.vario.setRate(vz)
        except:
            print("vario callback fail")
    return
            
#===============================================================================
# 
# class healthcheck(threading.Thread):
#    def __init__(self, _mpstate):
#        threading.Thread.__init__(self)
#        self._stop = threading.Event()
#        global mpstate
#        mpstate = _mpstate
# 
#        
#    def run(mpstate):
#        while ( (not mpstate.status.exit) and (not self._stop.isSet()) ):
#            time.sleep(0.5)
#===============================================================================
        

