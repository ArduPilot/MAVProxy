#!/usr/bin/env python
'''joystick interface module

Contributed by AndrewF:
  http://diydrones.com/profile/AndrewF

'''

import pygame, threading
from time import sleep

mpstate = None
j = None

def get_joystick():
    try:
        joycount = pygame.joystick.get_count()
        print( "number of joysticks is %i" % (joycount) )
        for index in range(joycount):
            j = pygame.joystick.Joystick(index)
            print ("Joystick %i is %s" % (index , j.get_name()))
        
        j = pygame.joystick.Joystick(0) # create a joystick instance
        j.init() # init instance
        print( 'joystick initialized: ' + j.get_name() )
        numhats = j.get_numhats()
        print( 'Joystick has %i hat controls' % (j.get_numhats()) )
        numbuttons = j.get_numbuttons()
        print( 'Joystick has %i buttons' % (numbuttons) )
        numballs = j.get_numballs()
        print( 'Joystick has %i trackballs' % (numballs) )
        numaxis = j.get_numaxes()
        print( 'Joystick has %i axis' % (numaxis) )
        
    except pygame.error:
        print 'joystick not found.'
        return

    while not mpstate.status.exit:        
        sleep(0.005)
        for e in pygame.event.get(): # iterate over event stack
            #the following is somewhat custom for the specific joystick model:
            if e.type == pygame.JOYAXISMOTION:
                if(numaxis >= 4):
                    mpstate.status.override[0] = int(j.get_axis(2)*400 + 1500)
                    mpstate.status.override[1] = int(j.get_axis(3)*400 + 1500)
                    mpstate.status.override[2] = int(-j.get_axis(1)*800 + 1100)
                    mpstate.status.override[3] = int(j.get_axis(0)*400 + 1500)
                    mpstate.override_period.force()
            elif e.type == pygame.JOYBUTTONDOWN:
                mpstate.functions.report_altitude(mpstate.status.altitude)
                for index in range(numbuttons):
                    buttonstate = j.get_button(index)
                continue
            elif e.type == pygame.JOYBUTTONUP:
                continue            
            elif e.type == pygame.JOYHATMOTION:
                for index in range(numhats):
                    hatstate = j.get_hat(index)
                continue
            elif e.type == pygame.JOYBALLMOTION:
                continue
            

def name():
    '''return module name'''
    return "joystick"

def description():
    '''return module description'''
    return "joystick aircraft control"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    #initialize joystick, if available
    pygame.init()
    pygame.joystick.init() # main joystick device system
        
    mpstate = _mpstate

    #get joystick input as a seperate thread to avoid modifying the main code:
    mpstate.status.joystick_thread = threading.Thread(target=get_joystick)
    mpstate.status.joystick_thread.daemon = True
    mpstate.status.joystick_thread.start()
