#!/usr/bin/env python


import threading
from time import sleep
import os

mpstate = None


class gumstix_manager(object):
    def __init__(self, mpstate):
        self.mpstate = mpstate
        
        self.unload = threading.Event()
        self.unload.clear()
    
        self.wifi_power         = 1
        self.trigger_wifi_off   = 0
        self.trigger_reboot     = 0
        self.trigger_shutdown   = 0
        self.baseboard          = ""
        self.led_state          = 0
        
    def set_baseboard(self, board):
        self.baseboard = board
#        if(board == "tobi"):
#            os.system('echo 1 >  /sys/class/gpio/gpio144/direction')

        
    def start_monitor(self):
        #initialize gumstix monitoring deamon
        self.gumstix_thread = threading.Thread(target=self.gumstix_monitor)
        self.gumstix_thread.daemon = True
        self.gumstix_thread.start()

    def gumstix_monitor(self):
        try:
            exit_state = mpstate.status.exit
        except:
            exit_state = 1
            
        while (not exit_state) and (not self.unload.isSet()):        
            sleep(0.1)
            if(	self.trigger_wifi_off == 1 ):
                f = open('/sys/class/gpio/gpio16/value', 'r')
                wifi_state = str(f.read(1))
                f.close()
                print("reading wifi state as " + wifi_state)
                
                if( wifi_state == str('1') ):
                    print("turning wifi off")
                    os.system('nmcli nm wifi off');
                    sleep(5)
                    os.system('echo 0 >  /sys/class/gpio/gpio16/value')
                                  
#                    f = open('/sys/class/gpio/gpio16/value', 'w')
#                    wifi_state = 0
#                    f.write(wifi_state)
#                    f.close()
                    self.wifi_power = 0
                    
                self.trigger_wifi_off = 0
                self.wifi_power = 0


            if( self.trigger_reboot == 1 ):
                self.trigger_reboot = 0
                os.system("reboot")
                
            if( self.trigger_shutdown == 1 ):
                self.trigger_shutdown = 0
                os.system("shutdown -h")
                
            if(self.baseboard == "pinto"):
                # caution.  led may be on mmc io
                if(self.led_state == 0):
                    os.system('echo 1 >  /sys/class/gpio/gpio21/value')
                    self.led_state = 1
                else:
                    os.system('echo 0 >  /sys/class/gpio/gpio21/value')
                    self.led_state = 0
                      
                if( self.trigger_shutdown == 0 ):
                    for x in range (0, 5):                    
                        f = open('/sys/class/gpio/gpio14/value', 'r')
                        shutdown_button_state = str(f.read(1))
                        f.close()
                    if( shutdown_button_state == str('1') ):
                        continue
                    print("shutdown in " + (6-x) )
                    sleep(1)
#                    if(x == 0):
#                        self.trigger_shutdown = 1       
            if(self.baseboard == "tobi"):
                if( self.trigger_shutdown == 0 ):
                    for x in range (0, 5):                    
                        f = open('/sys/class/gpio/gpio144/value', 'r')
                        shutdown_button_state = str(f.read(1))
                        f.close()
                    if( shutdown_button_state == str('1') ):
                        continue
                    print("shutdown in " + (6-x) )
                    sleep(1)
#                    if(x == 0):
#                        self.trigger_shutdown = 1
            try:
                exit_state = mpstate.status.exit
            except:
                exit_state = 1
        
                

# deamon should exit here after doing wifi shutdown

def name():
    '''return module name'''
    return "gumstix"

def description():
    '''return module description'''
    return "gumstix overo system management"

def cmd_gumstix_shutdown(args):
    mpstate.gumstix.trigger_shutdown = 1

def cmd_gumstix_reboot(args):
    mpstate.gumstix.trigger_reboot = 1

def cmd_gumstix_rfkill(args):
    mpstate.gumstix.trigger_wifi_off = 1

def cmd_gumstix(args):
    if(len(args) < 2):
        return 0
        
    if(args[0] == "board"):
        mpstate.gumstix.set_baseboard(args[1])

def init(_mpstate):
    '''initialise module'''
    global mpstate
        
    mpstate = _mpstate
    
    mpstate.gumstix = gumstix_manager(mpstate)
        
    mpstate.command_map['gumstix_shutdown'] = (cmd_gumstix_shutdown, "gumstix shutdown")
    mpstate.command_map['gumstix_reboot'] = (cmd_gumstix_reboot, "gumstix reboot")
    mpstate.command_map['gumstix_rfkill'] = (cmd_gumstix_rfkill, "gumstix wifi and bluetooth radio kill")
    mpstate.command_map['gumstix'] = (cmd_gumstix, "gumstix preferences and query")

    mpstate.gumstix.start_monitor()
    

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''

    #Use a heartbeat packet to switch off the wifi
    if m.get_type() == "HEARTBEAT":
        if(mpstate.gumstix):
            if(mpstate.gumstix.wifi_power == 1):
                mpstate.gumstix.trigger_wifi_off = 1
            