"""
  MAVProxy realtime graphing module

  uses lib/live_graph.py for display
"""

import mavutil, re, os, sys, threading, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib'))
import pymavario
import varioSettings

        
def name():
    '''return module name'''
    return "vario"

def description():
    '''return module description'''
    return "variometer"

def cmd_vario(args):
    '''vario command'''


def init(_mpstate):
    global mpstate
    mstate = _mpstate
    '''initialise module'''
    
    self

        try:
            self.settings_path = os.path.join(self.application_path, 'modules', 'data', "DefaultVarioSettings.xml")
#            self.settings_path = os.path.join(self.application_path, "DefaultVarioSettings.xml")
            self.Settings = varioSettings.parse(self.settings_path)
        except:
            print("Could not load vario settings at: " + self.settings_path)
        else:
          
    self.vario = pymavario.vario()

            
    self.vario.deadband = float(self.Settings.get_Deadband()) * 100.0
    self.vario.minRate = float(self.Settings.get_maxFallRate()) * -100.0
    self.vario.maxRate = float(self.Settings.get_maxRiseRate()) * 100.0
    self.vario.maxRate = float(self.m_textCtrlMaxRisingRate.GetValue()) * 100.0
    self.vario.minFallingKey = int(self.Settings.get_minFallingKey())
    value = int(str(self.Settings.get_maxFallingKey())
    self.vario.maxFallingKey = value
    self.vario.minRisingKey = int(self.Settings.get_minRisingKey())
    value = int(self.Settings.get_maxRisingKey())
    self.vario.maxRisingKey = value
        
    self.vario.setRisingSoundfont(str(self.Settings.get_risingSoundfont()))
    self.vario.setFallingSoundfont(str(self.Settings.get_fallingSoundfont()))

    print("vario initialised")

def unload():
    '''unload module'''
    self.vario.stop()
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
                #===============================================================
                # if msg and msg.get_type() == "HEARTBEAT":
                #    print(msg)        
                #    system = msg.get_srcSystem()
                #    component = msg.get_srcComponent()
                #    print("Heartbeat from UDB (system %u component %u)" % (system, component))
                #    if((system == self.system) and (component == self.component)):
                #        self.heartbeat_ok = True
                #        self.heartbeat_timer = time.time()
                #===============================================================

    if msg and msg.get_type() == "GLOBAL_POSITION_INT":
        try:
            vz = msg.vz   # vertical velocity in cm/s
            vz = int(-vz)
        except:
            print("decode global vz message fail")
        try:
            self.vario.vario_callback(vz)
        except:
            print("vario callback fail")
            

class healthcheck(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._stop = threading.Event()

        
    def run(self):
        while ( (not mpstate.status.exit) and (not self._stop.isSet()) ):
            time.sleep(0.5)
        
