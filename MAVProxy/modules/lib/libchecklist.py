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

    def __init__(self):
        import multiprocessing
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_event = multiprocessing.Event()
        self.close_event.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import Tkinter as tk

        '''curStep is which step in the list we are up to, increments +1 for each list completed
        it is the same as the column number of the checklist item'''
        self.curStep = 0

        self.root = tk.Tk()
        self.root.title("MAVProxy: Checklist")
        self.root.grid()
        self.createLists()
        self.createWidgets(self.root)
        self.on_timer()

        self.root.mainloop()


    def createLists(self):
        '''Generate the checklists. Note that:
        0,1 = off/on for auto-ticked items
        2,3 = off/on for manually ticked items'''

        self.beforeAssemblyList = {
        'Confirm batteries charged':2,
        'No physical damage to airframe':2,
        'All electronics present and connected':2,
        'Joe placed':2,
        'CoG of UAV correct':2,
        'Ground station operational':2
        }

        self.beforeEngineList = {
        'APM Booted':0,
        'Pandaboard Booted':2,
        'Cameras calibrated and capturing':2,
        'GPS lock':0,
        'Altitude lock':0,
        'Flight mode MANUAL':0,
        'Trim set from controller':0,
        'Avionics Battery':0,
        'Compass Calibrated':0,
        'Accelerometers and Gyros Calibrated':0,
        'UAV Level':0,
        'Aircraft Params Loaded':2,
        'Radio Links > 6db margin':0,
        'Waypoints Loaded':0
        }

        self.beforeTakeoffList = {
        'Flight control surfaces responsive':2,
        'Engine throttle responsive':2,
        'Runway clear':2,
        'Compass active':0,
        'IMU OK':2,
        'Set flight timer and alarm':2
        }

        self.beforeCruiseList = {
        'Airspeed > 10 m/s':0,
        'Altitude > 30 m':0,
        '< 100 degrees to 1st Waypoint':2
        }

        self.bottleDropList = {
        'Joe found':2,
        'Joe waypoint laid in':2,
        '< 100m to Joe waypoint':2,
        'Bottle drop mechanism activated':2
        }

        self.beforeLandingList = {
        'APM set to MANUAL mode':2,
        '< 100m from airfield home':2
        }

        self.beforeShutdownList = {
        'Engine cutoff':2,
        'Data downloaded':2
        }

    def createWidgets(self, frame):
        '''Create the controls on the UI'''
        import Tkinter as tk

        '''Group Labels'''
        AssemblyLabel = tk.Label(frame, text="During Assembly")
        EngineLabel = tk.Label(frame, text="Before Engine Start")
        BootLabel = tk.Label(frame, text="Before Takeoff")
        FlightLabel = tk.Label(frame, text="Before Cruise/AUTO")
        '''BottleLabel = tk.Label(frame, text="Bottle Drop")'''
        '''LandLabel = tk.Label(frame, text="Before Landing")'''
        '''ShutdownLabel = tk.Label(frame, text="Before Shutdown")'''
        AssemblyLabel.grid(row=0, column=0)
        EngineLabel.grid(row=0, column=1)
        BootLabel.grid(row=0, column=2)
        FlightLabel.grid(row=0, column=3)
        '''BottleLabel.grid(row=0, column=4)'''
        '''LandLabel.grid(row=0, column=5)'''
        '''ShutdownLabel.grid(row=0, column=6)'''

        '''before assembly checklist'''
        i = 1
        for key in self.beforeAssemblyList:
            if self.beforeAssemblyList[key] == 0:
                self.beforeAssemblyList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeAssemblyList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=0, sticky='w')
            if self.beforeAssemblyList[key] == 2:
                self.beforeAssemblyList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeAssemblyList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=0, sticky='w')
            i = i+1

        self.beforeAssemblyButton = tk.Button(text='Close final hatches', state="active", command=self.beforeAssemblyListCheck)
        self.beforeAssemblyButton.grid(row = i, column=0, sticky='w')

        '''before Engine Start checklist'''
        i = 1
        for key in self.beforeEngineList:
            if self.beforeEngineList[key] == 0:
                self.beforeEngineList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeEngineList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=1, sticky='w')
            if self.beforeEngineList[key] == 2:
                self.beforeEngineList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeEngineList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=1, sticky='w')
            i = i+1

        self.beforeEngineButton = tk.Button(text='Ready for Engine start', state="disabled", command=self.beforeEngineCheck)
        self.beforeEngineButton.grid(row = i, column=1, sticky='w')

        '''before takeoff checklist'''
        i = 1
        for key in self.beforeTakeoffList:
            if self.beforeTakeoffList[key] == 0:
                self.beforeTakeoffList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeTakeoffList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=2, sticky='w')
            if self.beforeTakeoffList[key] == 2:
                self.beforeTakeoffList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeTakeoffList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=2, sticky='w')
            i = i+1

        self.beforeTakeoffButton = tk.Button(text='Ready for Takeoff', state="disabled", command=self.beforeTakeoffCheck)
        self.beforeTakeoffButton.grid(row = i, column=2, sticky='w')

        '''After takeoff'''
        i=1
        for key in self.beforeCruiseList:
            if self.beforeCruiseList[key] == 0:
                self.beforeCruiseList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeCruiseList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=3, sticky='w')
            if self.beforeCruiseList[key] == 2:
                self.beforeCruiseList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeCruiseList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=3, sticky='w')
            i = i+1

        self.beforeCruiseButton = tk.Button(text='Ready for Cruise', state="disabled", command=self.beforeCruiseCheck)
        self.beforeCruiseButton.grid(row = i, column=3, sticky='w')

        '''Before bottle drop'''
        '''i=1
        for key in self.bottleDropList:
            if self.bottleDropList[key] == 0:
                self.bottleDropList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bottleDropList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=4, sticky='w')
            if self.bottleDropList[key] == 2:
                self.bottleDropList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.bottleDropList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=4, sticky='w')
            i = i+1

        self.bottleDropButton = tk.Button(text='Bottle drop completed', state="disabled", command=self.bottleDropCheck)
        self.bottleDropButton.grid(row = i, column=4, sticky='w')'''

        '''Before landing'''
        '''i=1
        for key in self.beforeLandingList:
            if self.beforeLandingList[key] == 0:
                self.beforeLandingList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeLandingList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=5, sticky='w')
            if self.beforeLandingList[key] == 2:
                self.beforeLandingList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeLandingList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=5, sticky='w')
            i = i+1

        self.beforeLandingButton = tk.Button(text='Ready for landing', state="disabled", command=self.beforeLandingCheck)
        self.beforeLandingButton.grid(row = i, column=5, sticky='w')'''

        '''before shutdown checklist'''
        '''i = 1
        for key in self.beforeShutdownList:
            if self.beforeShutdownList[key] == 0:
                self.beforeShutdownList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeShutdownList[key], state="disabled", wraplength=170, justify='left', onvalue=1, offvalue=0)
                aCheckButton.grid(row = i, column=6, sticky='w')
            if self.beforeShutdownList[key] == 2:
                self.beforeShutdownList[key] = tk.IntVar()
                aCheckButton = tk.Checkbutton(text=key, variable=self.beforeShutdownList[key], wraplength=170, justify='left', onvalue=3, offvalue=2)
                aCheckButton.grid(row = i, column=6, sticky='w')
            i = i+1

        self.beforeShutdownButton = tk.Button(text='Shutdown', state="disabled", command=self.beforeShutdownCheck)
        self.beforeShutdownButton.grid(row = i, column=6, sticky='w')'''


    def beforeAssemblyListCheck(self):
        '''Event for the "Checklist Complete" button for the Before Assembly section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeAssemblyList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeEngineButton.config(state="normal")
        self.beforeAssemblyButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 1

    def beforeEngineCheck(self):
        '''Event for the "Checklist Complete" button for the Before Engine Start section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeEngineList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeTakeoffButton.config(state="normal")
        self.beforeEngineButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 2

    def beforeTakeoffCheck(self):
        '''Event for the "Checklist Complete" button for the Before Takeoff section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeTakeoffList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeCruiseButton.config(state="normal")
        self.beforeTakeoffButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 3


    def beforeCruiseCheck(self):
        '''Event for the "Checklist Complete" button for the Before Cruise/AUTO section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeCruiseList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        '''self.bottleDropButton.config(state="normal")'''
        tkMessageBox.showinfo("Information", "Checklist Completed!")
        self.beforeCruiseButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 4

    def bottleDropCheck(self):
        '''Event for the "Checklist Complete" button for the Before Bottle Drop section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.bottleDropList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeLandingButton.config(state="normal")
        self.bottleDropButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 5

    def beforeLandingCheck(self):
        '''Event for the "Checklist Complete" button for the Before Landing section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeLandingList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeShutdownButton.config(state="normal")
        self.beforeLandingButton.config(text='Checklist Completed', state="disabled")
        self.curStep = 6

    def beforeShutdownCheck(self):
        '''Event for the "Checklist Complete" button for the Before Landing section'''
        import Tkinter as tk
        import tkMessageBox

        '''Check all of the checklist for ticks'''
        for key, value in self.beforeShutdownList.items():
            state = value.get()
            if state == 0 or state == 2:
                tkMessageBox.showinfo("Error", "Item not ticked: " + key)
                return

        '''disable all checkboxes in this column'''
        for child in self.root.winfo_children():
            if isinstance(child, tk.Checkbutton) and int(child.grid_info()['column']) == self.curStep:
                child.config(state="disabled")

        '''if we made it here, the checklist is OK'''
        self.beforeShutdownButton.config(text='Checklist Completed', state="disabled")
        tkMessageBox.showinfo("Information", "Checklist Completed!")
        self.curStep = 7

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
                    '''If the control is a checkbutton and it's name matches and we're in the right checklist step, update it'''
                    if (isinstance(child, tk.Checkbutton) and
                        obj.name == child.cget('text') and
                        int(child.grid_info()['column']) == self.curStep):
                        if obj.state == 1:
                            child.select()
                        else:
                            child.deselect()


        '''print("in here")'''
        self.root.after(500, self.on_timer)

    def set_status(self, name, status):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(CheckItem(name, status))


if __name__ == "__main__":
    # test the console
    import time

    checklist = UI()

    while checklist.is_alive():
        checklist.set_status("Compass Offsets", 1)
        time.sleep(0.5)
