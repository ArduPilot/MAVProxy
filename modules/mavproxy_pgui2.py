"""
  MAVProxy mixer gui module
  
"""

import mavutil, re, os, sys, threading, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib', 'param'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'param'))

import MAVlinkParameterProcesses2 as MAVLinkProcesses
 
#import subMAVFunctionSettings as MAVFSettingsAPI

        
def name():
    '''return module name'''
    return "pgui"

def description():
    '''return module description'''
    return "parameter settings editor"

def cmd_mixer(args):
    '''pgui command'''
    return


import wx
from ParamMainFrame2 import ParamMainFrame

from optparse import OptionParser

class ParamOptionParser(OptionParser):
    def error(self, msg):
        pass

class pguiApp(wx.App):
#    def __init__( self, mpstate):
#        self.mpstate = mpstate
#        wx.PyApp.__init__(self)

    def set_mpstate(self, mpstate):
        self.mpstate = mpstate
        self.MAVParamProc.set_mpstate( mpstate )
        
    def OnInit(self):
        self.m_frame = ParamMainFrame(None)
        self.MAVParamProc = MAVLinkProcesses.mavlink_parameter_processes(self.m_frame)
        self.m_frame.Show()
        self.SetTopWindow(self.m_frame)
        return True
    
    def stop(self):
        self.MAVParamProc.stop_services()
#        self.m_frame.Close()

class pgui_app_thread(threading.Thread):
    def __init__(self, mpstate):
        threading.Thread.__init__(self)
        
        self.mpstate = mpstate
        
    def stopped(self):
        return not self.isAlive()
        
    def run(self):

        print("pgui2 app thread starting")
                
        self.pgui_app = pguiApp(0)

        self.pgui_app.set_mpstate(self.mpstate)
        
        self.mpstate.pgui_initialised = True
        print("param initialised")

        self.pgui_app.MainLoop()
        
        print("stopping app")
        self.pgui_app.stop()

        print("pgui2 app thread end")
        self.mpstate.pgui_initialised = False
        
def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate

    mpstate.pgui_initialised = False

    mpstate.pgui = pgui_app_thread(mpstate, )
    mpstate.pgui.start()
    

def unload():
    '''unload module'''    
    try:
        if(mpstate.pgui_initialised == True):
            mpstate.pgui_initialised = False
            try:
                mpstate.pgui.stop()
            except:
                print("pgui is already closed")
    except:
        print("pgui already unloaded")
        
    try:
        mpstate.pgui = None
    except:
        print("mpstate.pgui doesn't exist")
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''

    if(mpstate.pgui_initialised == True):
        mpstate.pgui.pgui_app.MAVParamProc.msg_recv(msg)

            
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
        

