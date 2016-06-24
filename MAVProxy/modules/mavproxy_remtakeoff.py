#!/usr/bin/env python
'''
Remote Takeoff module
Stephen Dade
June 2016
'''

import sys, os, time
from MAVProxy.modules.lib import mp_remtakeoff
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
from MAVProxy.modules.lib.mp_settings import MPSetting
import multiprocessing, threading

class remtakeoffModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(remtakeoffModule, self).__init__(mpstate, "remtakeoff", "Remote Takeoff module")
        #self.remtakeoff = mp_remtakeoff.RemTakeoffUI()
        #state: 0=not active, 1=countdown, 2=arming, 3=takeoff
        self.remstatus = multiprocessing.Queue()
        self.settings.append(
            MPSetting('remtimer', int, 10, 'Seconds to Takeoff'))
        self.settings.append(
            MPSetting('remtakeoffwp', int, 1, 'Waypoint to begin from'))
        #self.settings.numcells
        self.add_command('remtakeoff', self.cmd_takeoff, "Begin Remote Takeoff Sequence")
        self.add_command('remabort', self.cmd_abort, "Abort Remote Takeoff Sequence")
        
        #try setting up the Rasberry Pi button input
        #Requires the RPi.GPIO library from pip
        #using button 11
        try:
            import RPi.GPIO as GPIO
            GPIO.setup(11, GPIO.IN)
            #200ms software de-bouncing, adding event handler for rising event on button 11
            GPIO.add_event_detect(11, GPIO.RISING, callback=pi_button_rising, bouncetime=200)
            print("Remote takeoff - YES Rpi hardware handling")
        except:
            print("Remote takeoff - no Rpi hardware handling")
            
        #try setting up for Odroid button handling
        #requires https://github.com/hardkernel/WiringPi2-Python
        try:
            import wiringpi2 as wpi
            wpi.wiringPiSetupSys()
            wpi.pinMode(11, 0) #set pin 11 to input
            #wpi.wiringPiISR (11, wpi.INT_EDGE_RISING,  void (*function)(void)) ;
            #note this library doesn't support interrupts, so using a process instead
            Odroidchild = multiprocessing.Process(target=OdroidLoop, args=(11))
            print("Remote takeoff - YES Odroid hardware handling")
            Odroidchild.start()
        except:
            print("Remote takeoff - no Odroid hardware handling")
            
    def OdroidLoop(channel):
        '''Event handling for the Odroid'''
        debounceCount = 0
        while(True):
            if wiringpi.digitalRead(channel) == 1:
                debounceCount = debounceCount+1
            if wiringpi.digitalRead(channel) == 0:
                debounceCount = 0
            #wait for 20*10ms of button push (software debouncing)
            if debounceCount > 20:
                print('RPi button push detected')
                debounceCount = 0
                self.cmd_takeoff()
                break 
            time.sleep(0.01)
        return   

    def pi_button_rising(channel):
        '''Event handler for Rasberry Pi'''
        print('RPi button push detected')
        self.cmd_takeoff()
        
    def cmd_takeoff(self, args):
        '''Begin the takeoff countdown - via new thread'''
        self.child = multiprocessing.Process(target=self.takeoff_child, args=(self.remstatus, self.settings.remtimer))
        self.child.start()
    
    def takeoff_child(self, statusqueue, timer):
        '''Start Takeoff Sequence (thread)'''
        #switch to appropriate waypoint for auto-takeoff
        print("Remote Takeoff - Set WP")
        statusqueue.put('wpset')
                       
        #start the timer 
        print("Remote Takeoff - Wait: %u seconds" % timer)
        for num in range(0, timer):
            time.sleep(1)
            #print("Queue = %u" % statusqueue.qsize())
            if not statusqueue.empty():
                curmsg = statusqueue.get()
                if curmsg == 'abort':
                    print("Remote Takeoff - Aborted")
                    return
                else:
                    #else put the message back in the queue
                    statusqueue.put(curmsg)
        
        print("Remote Takeoff - Arming")
        #arm the vehicle
        statusqueue.put('arm')
        
        #wait for arm confirmation. Or an abort
        while True:
            if not statusqueue.empty():
                curmsg = statusqueue.get()
                if curmsg == 'armed':
                    print("Remote Takeoff - Armed")
                    break 
                elif curmsg == 'abort':
                    print("Remote Takeoff - Aborted")
                    return 
                else:
                    #else put the message back in the queue
                    statusqueue.put(curmsg)
                    time.sleep(1)
                
        #switch to auto and off we go 
        time.sleep(1) 
        print("Remote Takeoff - Switch to AUTO")
        self.remstatus.put('takeoff')
        
        #notify end-of-sequence
        print("Remote Takeoff - Finished")
        return       
        
    def cmd_abort(self, args):
        '''Abort countdown'''
        self.remstatus.put('abort')
        #print("Queue = %u" % self.remstatus.qsize())
        #print("Abort Remote")

    def idle_task(self):
        '''run periodic tasks'''
        #check the queue for any return messages
        if not self.remstatus.empty():
            #if there is an arm message from child thread, command the vehicle to arm
            curmsg = self.remstatus.get()
            if curmsg == 'arm':
                #print("start arm")
                self.master.mav.set_mode_send(self.target_system,
                                      mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                      0)
                self.master.arducopter_arm() 
            elif curmsg == 'wpset':
                #print("setting wp")
                self.master.waypoint_set_current_send(int(self.settings.remtakeoffwp)) 
            elif curmsg == 'takeoff':
                mode_mapping = self.master.mode_mapping()
                modenum = mode_mapping['AUTO']
                self.master.set_mode(modenum)
            else:
                #else put the message back in the queue
                self.remstatus.put(curmsg)
        
        
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        #if not isinstance(self.remtakeoff, mp_remtakeoff.RemTakeoffUI):
        #    return
        #if not self.remtakeoff.is_alive():
        #    return
        
        type = msg.get_type()
        master = self.master
        
        #check for system arming (taken from arm display code in maxproxy_console.py)
        #note it is a 2 stage check
        if type == 'HEARTBEAT':
            isArmed = 0
            isReady = 0
            if self.master.motors_armed():
                isArmed = 1
                #print('isarmed1')
            if 'SYS_STATUS' in self.mpstate.status.msgs:
                if (self.mpstate.status.msgs['SYS_STATUS'].onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) != 0:
                    isReady = 1
                    #print('isarmed2')
            if isArmed == 1 and isReady == 1:
                #print('isarmed')
                self.remstatus.put('armed')
                


def init(mpstate):
    '''initialise module'''
    return remtakeoffModule(mpstate)
    

