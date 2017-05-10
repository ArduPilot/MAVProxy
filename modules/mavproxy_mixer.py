"""
  MAVProxy mixer gui module
  
"""

import mavutil, re, os, sys, threading, time

mpstate = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib', 'mixer'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'mixer'))

from mixer_doc import mixer_document
import MAVlinkProcesses
 
#import subMAVFunctionSettings as MAVFSettingsAPI

        
def name():
    '''return module name'''
    return "mixer"

def description():
    '''return module description'''
    return "mixer settings editor"

def cmd_mixer(args):
    '''mixer command'''
    return


import wx
from MainFrame import MainFrame

from optparse import OptionParser

class MyOptionParser(OptionParser):
    def error(self, msg):
        pass

class pyFEdit(wx.App):
#    def __init__( self, mpstate):
#        self.mpstate = mpstate
#        wx.PyApp.__init__(self)

    def set_mpstate(self, mpstate):
        self.mpstate = mpstate
        self.MAVProc.set_mpstate( mpstate )
        
    def OnInit(self):
        args = sys.argv[1:]
        print(args)
        self.aircraft = filter(lambda x: '--aircraft' in x,args)
        if(len(self.aircraft) < 1):
            self.aircraft = "default"
        else:
            self.aircraft = self.aircraft[0]
            splt = self.aircraft.split("=")
            if(len(splt) == 2):
                self.aircraft = splt[1]
            else:
                self.aircraft = "default"
                
        if(self.aircraft is None):
            self.aircraft = "default"

        if(self.aircraft == ""):
            self.aircraft = "default"
        
        self.mix_doc = mixer_document(self.aircraft)
        
        self.MAVProc = MAVlinkProcesses.mavlink_processes( self.mix_doc )
        self.m_frame = MainFrame(None, self.mix_doc)
        self.m_frame.Show()
        self.SetTopWindow(self.m_frame)
        return True
    
    def stop(self):
        self.MAVProc.stop_services();
        self.mix_doc.m_close()
                
class mixer_gui_thread(threading.Thread):
    def __init__(self, mpstate):
        threading.Thread.__init__(self)
        
        self.mpstate = mpstate

    def stopped(self):
        return not self.isAlive()
        
    def run(self):

        print("mixer gui thread starting")

        time.sleep(2.0)
                
        self.mixer_app = pyFEdit(0)
#        self.mixer_app.set_mpstate(self.mpstate)
#        app.RedirectStdio()
#        self.mixer_app.SetOutputWindowAttributes("pyFEdit")

        self.mixer_app.set_mpstate(self.mpstate)
        
        self.mpstate.mixer_initialised = True
        print("mixer initialised")
        
        time.sleep(2.0)

        self.mixer_app.MainLoop()
        
        time.sleep(2.0)

        print("stopping app")
        self.mixer_app.stop()

        print("mixer gui thread end")
        self.mpstate.mixer_initialised = False
        

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate

    mpstate.mixer_initialised = False

    mpstate.mixgui = mixer_gui_thread(mpstate)
    mpstate.mixgui.start()
    

def unload():
    '''unload module'''
    mpstate.mixer_initialised = False
    mpstate.mixer.stop()
    mpstate.mixer = None
        
def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''

    if(mpstate.mixer_initialised == True):
        mpstate.mixgui.mixer_app.MAVProc.msg_recv(msg)

            
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
        

