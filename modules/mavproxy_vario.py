"""
  MAVProxy vario module
  Uses fluidsynth and soundfonts driving alsa
"""

import mavutil, re, os, sys, threading, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib', 'vario'))
import mp_vario
import varioSettings


class vario_manager(object):
    def __init__(self, mpstate):
        self.mpstate = mpstate
        
        self.unload = threading.Event()
        self.unload.clear()

        self.monitor_thread = threading.Thread(target=self.vario_app)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        self.timeout_time = 0;
        
    def update_timeout(self):
        self.timeout_time = time.time() + 1.0;
    
    def vario_app(self):
        try:
            settings_path = "unassigned"
            settings_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "DefaultVarioSettings.xml")    #os.path.dirname(os.path.realpath(__file__))
     #        self.settings_path = os.path.join(self.application_path, "DefaultVarioSettings.xml")
            self.varioSettings = varioSettings.parse(settings_path)
        except:
            print("Could not load vario settings at: " + settings_path)
            print("vario not initialised")
        else:
    
     #       rise_font_path = mpstate.varioSettings.get_risingSoundfont()
     #       fall_font_path = mpstate.varioSettings.get_fallingSoundfont()   
            
            rise_font_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", str(self.varioSettings.get_risingSoundfont()) )
            fall_font_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", str(self.varioSettings.get_fallingSoundfont()) )         
            
            if(not os.path.isfile(rise_font_path)):
                print("did not find soundfont path for rising at " + rise_font_path)
                return        
            if(not os.path.isfile(fall_font_path)):
                print("did not find soundfont path for falling at " + fall_font_path)
                return
              
            self.vario = mp_vario.vario(mpstate, rise_font_path, fall_font_path)
        
                    
            self.vario.deadband = float(self.varioSettings.get_Deadband()) * 100.0
            self.vario.minRate = float(self.varioSettings.get_maxFallRate()) * -100.0
            self.vario.maxRate = float(self.varioSettings.get_maxRiseRate()) * 100.0
    #        mpstate.vario.maxRate = float(mpstate.varioSettings.m_textCtrlMaxRisingRate.GetValue()) * 100.0
            self.vario.minFallingKey = int(self.varioSettings.get_minFallingKey())
            value = int(str(self.varioSettings.get_maxFallingKey()))
            self.vario.maxFallingKey = value
            self.vario.minRisingKey = int(self.varioSettings.get_minRisingKey())
            value = int(self.varioSettings.get_maxRisingKey())
            self.vario.maxRisingKey = value
                
     #       mpstate.vario.setRisingSoundfont(str(mpstate.varioSettings.get_risingSoundfont()))
     #       mpstate.vario.setFallingSoundfont(str(mpstate.varioSettings.get_fallingSoundfont()))
        
            mpstate.vario_initialised = True
            print("vario initialised")        
        
        while ( (not mpstate.status.exit) and (not self.unload.is_set()) ):        
            time.sleep(0.1)
            if(time.time() > self.timeout_time):
                self.vario.setRate(0)
            
        mpstate.vario_initialised = False
        self.vario.stop()
        self.vario.join()
        self.vario = None
        print("vario closed")        

        
def name():
    '''return module name'''
    return "vario"

def description():
    '''return module description'''
    return "variometer"

def cmd_vario(args):
    '''vario command'''
    if(mpstate.vario_initialised == False):
        return
    usage = "vario param value"
    if(len(args) < 1):
         return
    if(args(0) == "volume"):
        mpstate.vario_manager.vario.setAmplitude(float(args(1)))

    

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
    mpstate.vario_manager = vario_manager(mpstate)
    
    mpstate.command_map['vario'] = (cmd_vario, "vario settings adjust")

def unload():
    '''unload module'''
    mpstate.vario_manager.unload.set()
        
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
                mpstate.vario_manager.vario.setRate(vz)
                mpstate.vario_manager.update_timeout()
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
        

