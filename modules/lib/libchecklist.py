#!/usr/bin/env python

"""
  MAVProxy checklist, implemented in a child process
  Created by Stephen Dade (stephen_dade@hotmail.com)
"""

class CheckItem():
    '''Checklist item used for information transfer
    between threads/processes/pipes'''
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
        import Tkinter as tk

        self.root = tk.Tk()

        self.root.grid()
        self.createLists()
        self.createWidgets(self.root)
        self.on_timer()
        self.root.mainloop()     

    def createLists(self):
        '''Generate the checklists. Note that:
        0,1 = off/on for auto-ticked items
        2,3 = off/on for manually ticked items'''

        self.bootList = {
        'Trim set from controller':2,
        'Avionics Battery':2,
        'Compass Offsets':2,
        'Accelerometers Calibrated':2,
        'Gyros Calibrated':2,
        'Aircraft Params Loaded':2,
        'Radio Links > 6db margin':2,
        'Waypoints Loaded':2
        }

        self.beforeCruiseList = {
        'Airspeed > 20 m/s':0,
        'Altitude > 30 m':0,
        '< 100 degrees to 1st Waypoint':0
        }

        self.bottleDropList = {
        'Joe found':0,
        'Joe waypoint laid in':0,
        '< 100m to Joe waypoint':0,
        'Bottle drop mechanism activated':0
        }

        self.beforeLanding = {
        '< 100m from airfield home':0
        }

    def createWidgets(self, frame):
        '''Create the controls on the UI'''
        import Tkinter as tk

	'''Group Labels'''
	BootLabel = tk.Label(frame, text="Boot / Before Takeoff")
	FlightLabel = tk.Label(frame, text="Before Cruise/AUTO")
	BottleLabel = tk.Label(frame, text="Bottle Drop")
	LandLabel = tk.Label(frame, text="Before Landing")
	BootLabel.grid(row=0, column=0)
	FlightLabel.grid(row=0, column=1)
	BottleLabel.grid(row=0, column=2)
	LandLabel.grid(row=0, column=3)
        i = 1

        '''Boot checklist'''
        for key in self.bootList:     
            if self.bootList[key] == 0:
                self.bootList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bootList[key], state="disabled", onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=0, sticky='w')
            if self.bootList[key] == 2:
                self.bootList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bootList[key], onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=0, sticky='w')
            i = i+1

	self.BootDone = tk.Button(text='Confirm Checklist Complete', state="active", command=self.bootCheck)      
	self.BootDone.grid(row = i, column=0, sticky='w')
        i=i+1

	'''After takeoff'''
        i=1
        for key in self.beforeCruiseList:     
            if self.beforeCruiseList[key] == 0:
                self.beforeCruiseList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeCruiseList[key], state="disabled", onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=1, sticky='w')
            if self.beforeCruiseList[key] == 2:
                self.beforeCruiseList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeCruiseList[key], onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=1, sticky='w')
            i = i+1

	self.TakeoffDone = tk.Button(text='Confirm Checklist Complete', state="disabled", command=self.takeoffCheck)      
	self.TakeoffDone.grid(row = i, column=1, sticky='w')
        i = i+1

	'''Before bottle drop'''
        i=1
        for key in self.bottleDropList:     
            if self.bottleDropList[key] == 0:
                self.bottleDropList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bottleDropList[key],state="disabled", onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=2, sticky='w')
            if self.bottleDropList[key] == 2:
                self.bottleDropList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bottleDropList[key], onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=2, sticky='w')
            i = i+1

	self.BottleDone = tk.Button(text='Confirm Checklist Complete', state="disabled", command=self.BottleCheck)      
	self.BottleDone.grid(row = i, column=2, sticky='w')
        i = i+1

        '''Before landing'''
        i=1
        for key in self.beforeLanding:     
            if self.beforeLanding[key] == 0:
                self.beforeLanding[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeLanding[key],state="disabled", onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=3, sticky='w')
            if self.beforeLanding[key] == 2:
                self.beforeLanding[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeLanding[key], onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=3, sticky='w')
            i = i+1

	self.LandingButton = tk.Button(text='Confirm Checklist Complete', state="disabled", command=self.landCheck)      
	self.LandingButton.grid(row = i, column=3, sticky='w')
        i = i+1


    def bootCheck(self):
        '''Event for the "Checklist Complete" button for the Before Takeoff section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.bootList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''if we made it here, the checklist is OK'''
        self.TakeoffDone.config(state="normal")
	self.BootDone.config(text='Checklist Completed', state="disabled")   


    def takeoffCheck(self):
        '''Event for the "Checklist Complete" button for the Before Cruise/AUTO section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeCruiseList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''if we made it here, the checklist is OK'''
        self.BottleDone.config(state="normal")
	self.TakeoffDone.config(text='Checklist Completed', state="disabled")  

    def BottleCheck(self):
        '''Event for the "Checklist Complete" button for the Before Bottle Drop section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.bottleDropList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''if we made it here, the checklist is OK'''
        self.LandingButton.config(state="normal")
	self.BottleDone.config(text='Checklist Completed', state="disabled") 

    def landCheck(self):
        '''Event for the "Checklist Complete" button for the Before Landing section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeLanding.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''if we made it here, the checklist is OK'''
	self.LandingButton.config(text='Checklist Completed', state="disabled")
        tkMessageBox.showinfo("Information", "Checklist Completed!")

    def close(self):
        '''close the console'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def on_timer(self):
        '''this timer periodically checks the inter-process pipe
        for any updated checklist items'''
        import Tkinter as tk
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
                    if isinstance(child, tk.Checkbutton) and obj.name == child.cget('text'):
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
