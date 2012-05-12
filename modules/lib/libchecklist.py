#!/usr/bin/env python

"""
  MAVProxy checklist, implemented in a child process  
"""

class CheckItem():
    '''Checklist item'''
    def __init__(self, name, state):
        self.name = name
        self.state = state

class UI():
    '''
    a UI for the MAVProxy checklist
    '''
    def __init__(self,
                 title='MAVProxy: checklist'):
        import multiprocessing
        self.title  = title
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_event = multiprocessing.Event()
        self.close_event.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        from Tkinter import *

        self.root = Tk()

        self.root.grid()
        self.createWidgets(self.root)
        self.on_timer()
        self.root.mainloop()     

    def createWidgets(self, frame):
        from Tkinter import *
	'''Group Labels'''
	BootLabel = Label(frame, text="Boot / Before Takeoff")
	FlightLabel = Label(frame, text="Before Cruise/AUTO")
	CruiseLabel = Label(frame, text="During Flight - General")
	LandLabel = Label(frame, text="Before Landing")
	BootLabel.grid(row=0, column=0)
	FlightLabel.grid(row=0, column=1)
	CruiseLabel.grid(row=0, column=2)
	LandLabel.grid(row=0, column=3)

	'''Before Takeoff'''
	BootTrim = Checkbutton(frame, text='Trim set from controller', state="disabled")      
	BootTrim.grid(row=1, column=0, sticky=W)
	BootAvbattery = Checkbutton(text='Avionics Battery', state="disabled")      
	BootAvbattery.grid(row=2, column=0, sticky=W)
	BootCompOffsets = Checkbutton(text='Compass Offsets', state="disabled")      
	BootCompOffsets.grid(row=3, column=0, sticky=W)
	BootAccel = Checkbutton(text='Accelerometers Calibrated', state="disabled")      
	BootAccel.grid(row=4, column=0, sticky=W)
	BootGyro = Checkbutton(text='Gyros Calibrated', state="disabled")      
	BootGyro.grid(row=5, column=0, sticky=W)
	BootAirParams = Checkbutton(text='Aircraft Params Loaded', state="disabled")      
	BootAirParams.grid(row=6, column=0, sticky=W)
	BootRadios = Checkbutton(text='Radio Links > 6db margin', state="disabled")      
	BootRadios.grid(row=7, column=0, sticky=W)
	BootWaypoints = Checkbutton(text='Waypoints Loaded', state="disabled")      
	BootWaypoints.grid(row=8, column=0, sticky=W)

	'''After takeoff'''
	TakeoffSpeed = Checkbutton(text='Airspeed > 20 m/s', state="disabled")      
	TakeoffSpeed.grid(row=1, column=1, sticky=W)
	TakeoffAlt = Checkbutton(text='Altitude > 30 m', state="disabled")      
	TakeoffAlt.grid(row=2, column=1, sticky=W)
	TakeoffWaypoint = Checkbutton(text='< 100 degrees to 1st Waypoint', state="disabled")      
	TakeoffWaypoint.grid(row=3, column=1, sticky=W)

	'''General - During Flight'''
	GeneralAirspeedDiv = Checkbutton(text='No Airspeed Sensor Divergence', state="disabled")      
	GeneralAirspeedDiv.grid(row=1, column=2, sticky=W)
	GeneralHeadingDiv = Checkbutton(text='No Heading Sensor Divergence', state="disabled")      
	GeneralHeadingDiv.grid(row=2, column=2, sticky=W)
	GeneralAltDiv = Checkbutton(text='No Altitude Sensor Divergence', state="disabled")      
	GeneralAltDiv.grid(row=3, column=2, sticky=W)

    def close(self):
        '''close the console'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def on_timer(self):
        from Tkinter import *
        '''state = self.state'''
        if self.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while self.child_pipe.poll():
            obj = self.child_pipe.recv()
            if isinstance(obj, CheckItem):
                # request to set a checklist item
                '''Go through all the controls in the main window'''
                for child in self.root.winfo_children():

                    '''If the control is a checkbutton and it's name matches, update it'''
                    if isinstance(child, Checkbutton) and obj.name == child.cget('text'):
                        if obj.state == 1:
                            child.select()
                        else:
                            child.deselect()


        print("in here")
        self.root.after(1000, self.on_timer)

    def set_status(self, name, status):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(CheckItem(name, status))

    
if __name__ == "__main__":
    # test the console
    import time

    checklist = UI()
    checklist.set_status("Compass Offsets", 1)
    while checklist.is_alive():
        '''console.write('Tick', fg='red')
        console.write(" %s " % time.asctime())
        console.writeln('tock', bg='yellow')
        console.set_status('GPS', 'GPS: OK', fg='blue', bg='green')
        console.set_status('Link1', 'Link1: OK', fg='green', bg='write')
        console.set_status('Date', 'Date: %s' % time.asctime(), fg='red', bg='write', row=2)'''
        print("main loop")
        time.sleep(0.5)
