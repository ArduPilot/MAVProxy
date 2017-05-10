#!/usr/bin/env python
from Tkinter import *
import time, math, threading


mpstate = None
app = None

def name():
    '''return module name'''
    return "APM Checklists"

def description():
    '''return module description'''
    return "Module for auto-checking APM at various stages throughout flight"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    global app

    mpstate = _mpstate

    '''Start up the UI. Note the 100ms time delay to let the UI vars
    initialise before we can start reading them'''
    app = App()
    time.sleep(0.1)

    '''Test a checklist item'''
    app.BootGyro.select()

    print("Module APM Checklists loaded")

'''This class contains the UI in a separate thread'''
class App(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.start()

    def callback(self):
       self.root.quit()

    def run(self):
        '''Set up the frame'''
        self.root = Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.frame = Frame()
        self.root.title("APM Checklists")
        self.frame.grid()

	'''Group Labels'''
	self.BootLabel = Label(self.frame, text="Boot / Before Takeoff")
	self.FlightLabel = Label(self.frame, text="Before Cruise/AUTO")
	self.CruiseLabel = Label(self.frame, text="During Flight - General")
	self.LandLabel = Label(self.frame, text="Before Landing")
	self.BootLabel.grid(row=0, column=0)
	self.FlightLabel.grid(row=0, column=1)
	self.CruiseLabel.grid(row=0, column=2)
	self.LandLabel.grid(row=0, column=3)

	'''Quit Button'''
        self.quitButton = Button ( self.frame, text='Quit', command=self.frame.quit )
	self.quitButton.grid(row=9, column=0)

	'''Before Takeoff'''
	self.BootTrim = Checkbutton(self.frame, text='Trim set from controller', state="disabled")      
	self.BootTrim.grid(row=1, column=0, sticky=W)
	self.BootAvbattery = Checkbutton(self.frame, text='Avionics Battery', state="disabled")      
	self.BootAvbattery.grid(row=2, column=0, sticky=W)
	self.BootCompOffsets = Checkbutton(self.frame, text='Compass Offsets', state="disabled")      
	self.BootCompOffsets.grid(row=3, column=0, sticky=W)
	self.BootAccel = Checkbutton(self.frame, text='Accelerometers Calibrated', state="disabled")      
	self.BootAccel.grid(row=4, column=0, sticky=W)
	self.BootGyro = Checkbutton(self.frame, text='Gyros Calibrated', state="disabled")      
	self.BootGyro.grid(row=5, column=0, sticky=W)
	self.BootAirParams = Checkbutton(self.frame, text='Aircraft Params Loaded', state="disabled")      
	self.BootAirParams.grid(row=6, column=0, sticky=W)
	self.BootRadios = Checkbutton(self.frame, text='Radio Links > 6db margin', state="disabled")      
	self.BootRadios.grid(row=7, column=0, sticky=W)
	self.BootWaypoints = Checkbutton(self.frame, text='Waypoints Loaded', state="disabled")      
	self.BootWaypoints.grid(row=8, column=0, sticky=W)

	'''After takeoff'''
	self.TakeoffSpeed = Checkbutton(self.frame, text='Airspeed > 20 m/s', state="disabled")      
	self.TakeoffSpeed.grid(row=1, column=1, sticky=W)
	self.TakeoffAlt = Checkbutton(self.frame, text='Altitude > 30 m', state="disabled")      
	self.TakeoffAlt.grid(row=2, column=1, sticky=W)
	self.TakeoffWaypoint = Checkbutton(self.frame, text='< 100 degrees to 1st Waypoint', state="disabled")      
	self.TakeoffWaypoint.grid(row=3, column=1, sticky=W)

	'''General - During Flight'''
	self.GeneralAirspeedDiv = Checkbutton(self.frame, text='No Airspeed Sensor Divergence', state="disabled")      
	self.GeneralAirspeedDiv.grid(row=1, column=2, sticky=W)
	self.GeneralHeadingDiv = Checkbutton(self.frame, text='No Heading Sensor Divergence', state="disabled")      
	self.GeneralHeadingDiv.grid(row=2, column=2, sticky=W)
	self.GeneralAltDiv = Checkbutton(self.frame, text='No Altitude Sensor Divergence', state="disabled")      
	self.GeneralAltDiv.grid(row=3, column=2, sticky=W)

        '''finally, start the event loop'''
        self.root.mainloop()


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    '''go through the packed and set the Checkboxes as appropriate - doesn't work yet'''
    '''Boot - Trim'''
    if m.get_type() == 'PARAM_VALUE':
        if str(m.param_id) == 'RC1_TRIM' and m.param_value == 1200:
            app.BootTrim.deselect()
        elif str(m.param_id) == 'RC2_TRIM' and m.param_value == 1200:
            app.BootTrim.deselect()
        elif str(m.param_id) == 'RC3_TRIM' and m.param_value == 1200:
            app.BootTrim.deselect()
        elif str(m.param_id) == 'RC4_TRIM' and m.param_value == 1200:
            app.BootTrim.deselect()
        else:
            app.BootTrim.select()

    '''Boot - Avionics Battery'''
    if m.get_type() == 'SYS_STATUS':
        if str(m.param_id) == 'voltage_battery' and m.param_value < 5000:
            app.BootAvbattery.deselect()
        else:
            app.BootAvbattery.select()

    '''Boot - Compass Offsets'''


