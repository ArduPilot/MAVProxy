'''
Swarming support
Stephen Dade, July 2021

Helper GUI for managing leader-follower swarms

'''

import sys
import time
import wx.lib.scrolledpanel as scrolled
from pymavlink import mavutil

from MAVProxy.modules.lib import (icon, mp_module, mp_settings, mp_util,
                                  multiproc, win_layout)
from MAVProxy.modules.lib.wx_loader import wx



def get_vehicle_name(vehtype):
    '''return vehicle type string from a heartbeat'''
    if vehtype == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        return 'Plane'
    if vehtype == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        return 'Rover'
    if vehtype == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
        return 'Boat'
    if vehtype == mavutil.mavlink.MAV_TYPE_SUBMARINE:
        return 'Sub'
    if vehtype in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                   mavutil.mavlink.MAV_TYPE_COAXIAL,
                   mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                   mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                   mavutil.mavlink.MAV_TYPE_TRICOPTER,
                   mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
        return "Copter"
    if vehtype == mavutil.mavlink.MAV_TYPE_HELICOPTER:
        return "Heli"
    if vehtype == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
        return "Tracker"
    if vehtype == mavutil.mavlink.MAV_TYPE_AIRSHIP:
        return "Blimp"
    return "UNKNOWN({0})".format(vehtype)

def parmString(str):
    '''convert string to bytes in python3'''
    if sys.version_info.major < 3:
        return str   
    return bytes(str, 'ascii')

class SwarmUI():
    '''
    Swarm User Interface
    '''

    def __init__(self, parmsToShow, title='MAVProxy: Swarm Control', takeoffalt=10):
        self.title = title
        self.parmsToShow = parmsToShow
        self.takeoffalt = takeoffalt
        self.parent_pipe, self.child_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()

        app = wx.App(False)
        app.frame = SwarmFrame(
            self, title=self.title, params=self.parmsToShow, takeoffalt=self.takeoffalt)
        app.frame.Show()
        app.MainLoop()

    def changesetting(self, setting, val):
        '''Change a setting'''
        if self.child.is_alive():
            self.parent_pipe.send(('changesetting', (setting, val)))

    def close(self):
        '''close the UI'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()

    def updateLayout(self, newLayout):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(('updatelayout', newLayout))

    def updateHB(self, vehHB):
        '''update HB status of vehicles'''
        if self.child.is_alive():
            self.parent_pipe.send(('updateHB', vehHB))

    def onmavlinkpacket(self, pkt):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(('onmavlinkpacket', pkt))


class UnassignedPanel(wx.Panel):
    '''
    A wx.Panel to hold a single unassigned vehicle
    '''
    def __init__(self, state, parent):
        wx.Panel.__init__(self, parent)  # , style=wx.BORDER_SIMPLE)
        self.state = state

        self.sizer = wx.BoxSizer(wx.VERTICAL)

        self.titleText = wx.StaticText(
            self, label="Unassigned Vehicles -->\n\nTools:")
        self.sizer.Add(self.titleText, flag=wx.EXPAND |
                       wx.LEFT | wx.RIGHT, border=5)

        self.getParamsButton = wx.Button(
            self, label="Get offsets", size=wx.Size(100, 50))
        self.Bind(wx.EVT_BUTTON, self.getParams, self.getParamsButton)
        self.resetButton = wx.Button(
            self, label="Reset layout", size=wx.Size(100, 50))
        self.Bind(wx.EVT_BUTTON, self.reset, self.resetButton)

        self.sizer.Add(self.getParamsButton)
        self.sizer.Add(self.resetButton)

        # Do the sizer layout
        self.SetSizer(self.sizer)

    def getParams(self, event):
        '''get offset params again'''
        self.state.child_pipe.send(("getparams", None, None))

    def reset(self, event):
        '''arm/disarm the vehicle'''
        self.state.child_pipe.send(("resetLayout", None, None))


class VehiclePanel(wx.Panel):
    '''
    A wx.panel to hold a single vehicle (leader or follower)
    '''

    def __init__(self, state, parent, sysid, compid, vehtype, isLeader, listFollowers, takeoffalt):
        wx.Panel.__init__(self, parent)
        self.state = state
        self.sysid = sysid
        self.compid = compid
        self.listFollowers = listFollowers
        self.isLeader = isLeader
        self.vehtype = vehtype

        self.takeoffalt = takeoffalt

        self.inLinkLoss = False

        # XYZ offsets. Filled are we get params
        self.offsetValues = [None, None, None]

        self.sizer = wx.BoxSizer(wx.VERTICAL)

        # Status boxes
        if self.isLeader:
            self.title = wx.StaticText(self, label="Leader {0}:{1} {2}".format(
                sysid, compid, get_vehicle_name(vehtype)))
            self.offsets = wx.StaticText(self, label="Offset: N/A")
        else:
            self.title = wx.StaticText(self, label="Veh {0}:{1} {2}".format(
                sysid, compid, get_vehicle_name(vehtype)))
            self.offsets = wx.StaticText(self, label="Offset: xxxxxx")

        self.armmode = wx.StaticText(self, label="armed/mode N/A    ")
        self.thrAlt = wx.StaticText(
            self, label="Alt: {0}m    Thr: {1}%".format(0, 0))
        self.altRel = wx.StaticText(self, label="Rel Alt: {0}m".format(0))
        self.battery = wx.StaticText(self, label="Battery: {0}V".format(0))
        self.status = wx.StaticText(self, label="Status: N/A")
        self.prearm = wx.StaticText(self, label="Prearm: N/A")
        self.statusText = wx.TextCtrl(
            self, style=wx.TE_READONLY | wx.TE_MULTILINE, size=wx.Size(140, 100))

        # Command buttons
        self.doArm = wx.Button(self, label="XXX", size=wx.Size(100, 50))
        self.Bind(wx.EVT_BUTTON, self.arm, self.doArm)
        if self.isLeader:
            self.armSizer = wx.BoxSizer(wx.HORIZONTAL)
            self.doArmAll = wx.Button(self, label="ALL", size=wx.Size(70, 50))
            self.Bind(wx.EVT_BUTTON, self.armAll, self.doArmAll)

        self.doGuided = wx.Button(
            self, label="Mode GUIDED", size=wx.Size(100, 50))
        self.Bind(wx.EVT_BUTTON, self.guided, self.doGuided)
        if self.isLeader:
            self.guidedSizer = wx.BoxSizer(wx.HORIZONTAL)
            self.doGuidedAll = wx.Button(
                self, label="ALL", size=wx.Size(70, 50))
            self.Bind(wx.EVT_BUTTON, self.guidedAll, self.doGuidedAll)

        if vehtype != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.doGuidedTakeoff = wx.Button(
                self, label="GUIDED T/O {0}m".format(self.takeoffalt), size=wx.Size(130, 50))
            self.Bind(wx.EVT_BUTTON, self.guidedTakeoff, self.doGuidedTakeoff)
            if self.isLeader:
                self.takeoffSizer = wx.BoxSizer(wx.HORIZONTAL)
                self.doGuidedTakeoffAll = wx.Button(
                    self, label="ALL".format(self.takeoffalt), size=wx.Size(70, 50))
                self.Bind(wx.EVT_BUTTON, self.guidedTakeoffAll,
                          self.doGuidedTakeoffAll)

        self.doRTL = wx.Button(self, label="Mode RTL", size=wx.Size(100, 50))
        self.Bind(wx.EVT_BUTTON, self.rtl, self.doRTL)
        if self.isLeader:
            self.rtlSizer = wx.BoxSizer(wx.HORIZONTAL)
            self.doRTLAll = wx.Button(self, label="ALL", size=wx.Size(70, 50))
            self.Bind(wx.EVT_BUTTON, self.rtlAll, self.doRTLAll)

        self.doKill = wx.Button(self, label="KILL", size=wx.Size(100, 50))
        self.killTimer = None
        self.Bind(wx.EVT_BUTTON, self.kill, self.doKill)
        if self.isLeader:
            self.killSizer = wx.BoxSizer(wx.HORIZONTAL)
            self.doKillAll = wx.Button(self, label="ALL", size=wx.Size(70, 50))
            self.killAllTimer = None
            self.Bind(wx.EVT_BUTTON, self.killall, self.doKillAll)

        if self.isLeader:
            self.doFollowAll = wx.Button(
                self, label="All Follow Leader", size=wx.Size(130, 50))
            self.Bind(wx.EVT_BUTTON, self.followAll, self.doFollowAll)
            self.doAUTO = wx.Button(
                self, label="Mode AUTO", size=wx.Size(100, 50))
            self.Bind(wx.EVT_BUTTON, self.auto, self.doAUTO)
        else:
            self.doFollow = wx.Button(
                self, label="Mode Follow", size=wx.Size(100, 50))
            self.Bind(wx.EVT_BUTTON, self.follow, self.doFollow)

        # Do the sizer layout
        self.doSizer()

        # get offset params. Needs to be after GUI elements are created
        time.sleep(0.05)
        self.state.child_pipe.send(("getoffsets", self.sysid, self.compid))

    def doSizer(self):
        '''Sort out all the sizers and layout'''
        self.sizer.Add(self.title)
        self.sizer.Add(self.armmode)
        self.sizer.Add(self.thrAlt)
        self.sizer.Add(self.altRel)
        self.sizer.Add(self.battery)
        self.sizer.Add(self.status)
        self.sizer.Add(self.prearm)
        self.sizer.Add(self.offsets)
        self.sizer.Add(self.statusText)

        if self.isLeader:
            self.armSizer.Add(self.doArm)
            self.armSizer.Add(self.doArmAll)
            self.sizer.Add(self.armSizer)
        else:
            self.sizer.Add(self.doArm)

        if self.vehtype != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            if self.isLeader:
                self.takeoffSizer.Add(self.doGuidedTakeoff)
                self.takeoffSizer.Add(self.doGuidedTakeoffAll)
                self.sizer.Add(self.takeoffSizer)
            else:
                self.sizer.Add(self.doGuidedTakeoff)

        if self.isLeader:
            self.guidedSizer.Add(self.doGuided)
            self.guidedSizer.Add(self.doGuidedAll)
            self.sizer.Add(self.guidedSizer)
        else:
            self.sizer.Add(self.doGuided)

        if self.isLeader:
            self.rtlSizer.Add(self.doRTL)
            self.rtlSizer.Add(self.doRTLAll)
            self.sizer.Add(self.rtlSizer)
        else:
            self.sizer.Add(self.doRTL)

        if self.isLeader:
            self.killSizer.Add(self.doKill)
            self.killSizer.Add(self.doKillAll)
            self.sizer.Add(self.killSizer)
        else:
            self.sizer.Add(self.doKill)

        if self.isLeader:
            self.sizer.Add(self.doFollowAll)
            self.sizer.Add(self.doAUTO)
        else:
            self.sizer.Add(self.doFollow)

        self.SetSizer(self.sizer)

    def updatetakeoffalt(self, alt):
        '''update takeoff altitude'''
        self.takeoffalt = alt
        if self.vehtype != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.doGuidedTakeoff.SetLabel(
                "GUIDED T/O {0}m".format(self.takeoffalt))
            if self.isLeader:
                self.doGuidedTakeoffAll.SetLabel("ALL".format(self.takeoffalt))

    def updateData(self, armed, mode):
        '''update the arming/mode status. Only change if required'''
        
        if armed and (self.doArm.GetLabel() in ["ARM", "XXX"] or str(mode) not in self.armmode.GetLabel()):
            self.armmode.SetForegroundColour((0, 200, 0))
            self.armmode.SetLabel("Armed" + "/" + str(mode))
            self.doArm.SetLabel("DISARM")
        elif not armed and (self.doArm.GetLabel() in ["DISARM", "XXX"] or str(mode) not in self.armmode.GetLabel()):
            self.armmode.SetForegroundColour((200, 0, 0))
            self.armmode.SetLabel("Disarmed" + "/" + str(mode))
            self.doArm.SetLabel("ARM")

    def updatethralt(self, throttle, alt):
        '''update throttle and altitude status'''
        self.thrAlt.SetLabel("Alt: {0}m    Thr: {1}%".format(int(alt), throttle))

    def updaterelalt(self, relalt):
        '''update relative altitude'''
        self.altRel.SetLabel("Rel Alt: {0}m".format(int(relalt)))

    def updateStatus(self, status):
        '''update status'''
        self.status.SetLabel("Status: {0}".format(status))
        if status in ["FAILSAFE", "Emergency", "Terminated"]:
            self.status.SetForegroundColour((200, 0, 0))
        elif status == "Flying":
            self.status.SetForegroundColour((0, 200, 0))
        else:
            self.status.SetForegroundColour((120, 120, 0))

    def updateprearm(self, preArmGood):
        '''Update pre-arm status'''
        self.prearm.SetLabel("Prearm: {0}".format(
            "OK" if preArmGood else "BAD"))
        if preArmGood:
            self.prearm.SetForegroundColour((0, 200, 0))
        else:
            self.prearm.SetForegroundColour((200, 0, 0))

    def updatevoltage(self, voltage):
        '''update battery voltage'''
        self.battery.SetLabel("Battery: {0:.1f}V".format(voltage))

    def arm(self, event):
        '''arm/disarm the vehicle'''
        self.state.child_pipe.send(
            ("arm" if self.doArm.GetLabel() == "ARM" else "disarm", self.sysid, self.compid))

    def armAll(self, event):
        '''Arm leader and all followers'''
        self.state.child_pipe.send(("arm", self.sysid, self.compid))
        if len(self.listFollowers) > 0:
            for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                self.state.child_pipe.send(("arm", sysidFollow, compidFollow))

    def guided(self, event):
        '''switch to mode guided'''
        self.state.child_pipe.send(("GUIDED", self.sysid, self.compid))

    def guidedAll(self, event):
        '''mode guided for leader and all followers'''
        self.state.child_pipe.send(("GUIDED", self.sysid, self.compid))
        if len(self.listFollowers) > 0:
            for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                self.state.child_pipe.send(
                    ("GUIDED", sysidFollow, compidFollow))

    def addstatustext(self, text):
        '''have recieved statustext, update relevant control'''
        self.statusText.AppendText('\n' + text)

    def guidedTakeoff(self, event):
        '''switch to guided mode, wait 0.1 sec then nav_takeoff to TAKEOFFALT'''
        self.state.child_pipe.send(("GUIDED", self.sysid, self.compid))
        time.sleep(0.1)
        self.state.child_pipe.send(("takeoff", self.sysid, self.compid))

    def guidedTakeoffAll(self, event):
        '''switch to guided mode, wait 0.1 sec then nav_takeoff to TAKEOFFALT for all vehicles'''
        self.state.child_pipe.send(("GUIDED", self.sysid, self.compid))
        time.sleep(0.1)
        self.state.child_pipe.send(("takeoff", self.sysid, self.compid))

        if len(self.listFollowers) > 0:
            for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                self.state.child_pipe.send(
                    ("GUIDED", sysidFollow, compidFollow))
                time.sleep(0.1)
                self.state.child_pipe.send(
                    ("takeoff", sysidFollow, compidFollow))

    def rtl(self, event):
        '''switch to mode RTL'''
        self.state.child_pipe.send(("RTL", self.sysid, self.compid))

    def rtlAll(self, event):
        '''RTL leader and all followers'''
        self.state.child_pipe.send(("RTL", self.sysid, self.compid))
        if len(self.listFollowers) > 0:
            for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                self.state.child_pipe.send(("RTL", sysidFollow, compidFollow))

    def killTimeout(self, event):
        '''Kill action timeout'''
        self.doKill.SetLabel("KILL")
        self.killTimer = None

    def killAllTimeout(self, event):
        '''Kill all action timeout'''
        self.doKillAll.SetLabel("ALL")
        self.killAllTimer = None

    def kill(self, event):
        '''disarm force (kill) the drone'''
        if self.doKill.GetLabel() == "KILL":
            self.doKill.SetLabel("KILL Sure?")
            # start a 1 sec timer
            self.killTimer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.killTimeout, self.killTimer)
            self.killTimer.Start(1000)
        else:
            self.state.child_pipe.send(("kill", self.sysid, self.compid))

    def killall(self, event):
        '''disarm force (kill) the whole swarm'''
        if self.doKillAll.GetLabel() == "ALL":
            self.doKillAll.SetLabel("ALL Sure?")
            # start a 1 sec timer
            self.killAllTimer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.killAllTimeout, self.killAllTimer)
            self.killAllTimer.Start(1000)
        else:
            self.state.child_pipe.send(("kill", self.sysid, self.compid))
            if len(self.listFollowers) > 0:
                for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                    self.state.child_pipe.send(
                        ("kill", sysidFollow, compidFollow))

    def follow(self, event):
        '''switch to mode follow'''
        self.state.child_pipe.send(("FOLLOW", self.sysid, self.compid))

    def followAll(self, event):
        '''Mode FOLLOW for all followers'''
        if len(self.listFollowers) > 0:
            for (sysidFollow, compidFollow, vehtype) in self.listFollowers:
                self.state.child_pipe.send(
                    ("FOLLOW", sysidFollow, compidFollow))

    def auto(self, event):
        '''switch to mode AUTO'''
        self.state.child_pipe.send(("AUTO", self.sysid, self.compid))

    def updateOffset(self, param, value):
        '''get offset param'''
        if self.isLeader:
            return
        if param == 'FOLL_OFS_X':
            self.offsetValues[0] = value
        elif param == 'FOLL_OFS_Y':
            self.offsetValues[1] = value
        elif param == 'FOLL_OFS_Z':
            self.offsetValues[2] = value
        else:
            return
        # refresh static text
        if self.offsetValues[0] != None and self.offsetValues[1] != None and self.offsetValues[2] != None:
            self.offsets.SetLabel("Offset: {0}m,{1}m,{2}m".format(
                self.offsetValues[0], self.offsetValues[1], self.offsetValues[2]))


class SwarmFrame(wx.Frame):
    '''
    The main frame of the UI. Runs in a subprocess, so need a pipe to communicate with
    '''

    def __init__(self, state, title, params, takeoffalt):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(
            1400, 500), style=wx.DEFAULT_FRAME_STYLE)
        self.panel = scrolled.ScrolledPanel(self, style=wx.FULL_REPAINT_ON_RESIZE)
        self.panel.SetBackgroundColour(wx.WHITE)

        # Icon
        self.SetIcon(icon.SimpleIcon("Swarm").get_ico())

        # Params to show (array)
        self.parmsToShow = params

        self.takeoffalt = takeoffalt

        self.last_layout_send = time.time()

        # layout. Column per leader, with followers underneath
        self.sizer = wx.FlexGridSizer(1, 1, 0, 0)

        # add in the pipe from MAVProxy
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, lambda evt,
                  panel=self.panel: self.on_timer(evt), self.timer)
        self.timer.Start(100)

        #Fires on window resize
        self.Bind(wx.EVT_SIZE, self.OnSize)

        # layout
        self.panel.SetSizer(self.sizer)
        self.panel.Layout()
        self.panel.SetupScrolling()
        self.Show(True)

    def OnSize(self, e):
        #This refresh shouldn't be necessary
        self.Refresh()

        #Pass event up the chain so window still resizes
        e.Skip()

    def on_timer(self, event):
        '''receive messages from MAVProxy and pass on to GUI'''
        state = self.state

        # update saved pos/size of GUI
        now = time.time()
        if now - self.last_layout_send > 1:
            self.last_layout_send = now
            state.child_pipe.send(
                (win_layout.get_wx_window_layout(self), 0, 0))

        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe.poll():
            obj = state.child_pipe.recv()
            if obj[0] == 'updatelayout':
                self.updateLayout(obj[1])
            elif obj[0] == 'onmavlinkpacket':
                self.onmavlinkpacket(obj[1])
            elif obj[0] == 'updateHB':
                self.updateHB(obj[1])
            elif isinstance(obj[0], win_layout.WinLayout):
                win_layout.set_wx_window_layout(self, obj[0])
            elif obj[0] == 'changesetting' and obj[1][0] == 'takeoffalt':
                self.updatetakeoffalt(int(obj[1][1]))

    def updatetakeoffalt(self, alt):
        '''Update the takeoff altitude of the vehicles'''
        children = self.sizer.GetChildren()
        # update panel
        for child in children:
            widget = child.GetWindow()
            if isinstance(widget, VehiclePanel):
                widget.updatetakeoffalt(alt)

    def updateHB(self, vehHB):
        '''Update the GUI panels if we've lost link to vehicle
        Panel goes red if more than 4 sec since last HB'''
        children = self.sizer.GetChildren()
        # update panel
        for child in children:
            widget = child.GetWindow()
            if isinstance(widget, VehiclePanel):
                if (widget.sysid, widget.compid) in vehHB.keys():
                    # put to red if more than 4 sec, else no colour. Only change if required.
                    if vehHB[(widget.sysid, widget.compid)] + 4 < time.time() and widget.GetBackgroundColour() != wx.RED:
                        widget.SetBackgroundColour(wx.RED)
                    elif vehHB[(widget.sysid, widget.compid)] + 4 > time.time() and widget.GetBackgroundColour() != wx.WHITE:
                        widget.SetBackgroundColour(wx.WHITE)

    def updateLayout(self, layout):
        '''Update (recreate) the GUI layout, based on known vehicles'''
        leaders = []  # array of (sysid, compid, followid, vehtype)

        # create list of leaders, another vehicle has it's sysid as leader
        for veh in layout:
            for vehL in layout:
                if vehL[2] == veh[0] and veh not in leaders:
                    leaders.append(veh)

        # create list of followers
        followers = {}  # dict of key=leadersysid, (sysid, compid, vehtype)
        maxfollowers = 0
        for veh in layout:
            # don't include leaders in the follower list
            if veh[2] != 0:
                if veh[2] not in followers.keys():
                    followers[veh[2]] = [(veh[0], veh[1], veh[3])]
                else:
                    followers[veh[2]].append((veh[0], veh[1], veh[3]))
                if len(followers[veh[2]]) > maxfollowers:
                    maxfollowers = len(followers[veh[2]])

        # sort followers by sysid increasing
        for fwr in followers.keys():
            followers[fwr].sort(key=lambda tup: tup[0])

        # Any unassigned vehicles: Leader with 0 followers OR leader not present
        # OR vehicles not present in either list (leftovers)
        unassignedVeh = []  # (sysid, compid, vehtype)

        # Leader with no followers
        for index, veh in enumerate(leaders):
            if veh[0] not in followers.keys():
                del leaders[index]
                unassignedVeh.append((veh[0], veh[1], veh[3]))

        # Not present in either list
        allfollowersflat = []
        for fl in followers.items():
            for vv in fl[1]:
                allfollowersflat.append(vv[0])
        for veh in layout:
            if (veh not in leaders) and veh[0] not in allfollowersflat:
                unassignedVeh.append((veh[0], veh[1], veh[3]))

        # Leader not present
        for leader in followers.copy().keys():
            if leader not in [i[0] for i in leaders]:
                for veh in followers[leader]:
                    unassignedVeh.append(veh)
                del followers[leader]  # followers.remove(leader)

        self.sizer.Clear(True)
        colsVeh = max(maxfollowers, len(unassignedVeh))
        self.sizer = wx.FlexGridSizer((len(leaders)*2) + 1, colsVeh+1+1, 5, 0)

        # start populating the grid
        # for each row:
        for leader in leaders:
            panelLeader = VehiclePanel(
                self.state, self.panel, leader[0], leader[1], leader[3], True, followers[leader[0]], self.takeoffalt)
            self.sizer.Add(panelLeader)

            # add vertical line
            line = wx.StaticLine(self.panel, style=wx.LI_VERTICAL)
            self.sizer.Add(line, proportion=0, flag=wx.EXPAND |
                           wx.LEFT | wx.RIGHT, border=5)

            for follower in range(colsVeh):
                if leader[0] in followers.keys() and len(followers[leader[0]]) > follower:
                    panelFollower = VehiclePanel(self.state, self.panel, followers[leader[0]][follower][0], followers[
                                                 leader[0]][follower][1], followers[leader[0]][follower][2], False, None, self.takeoffalt)
                else:
                    panelFollower = wx.StaticText(self.panel, label="N/A")
                self.sizer.Add(panelFollower, flag=wx.EXPAND |
                               wx.LEFT | wx.RIGHT, border=5)

            # add horizontal line, column by column
            for follower in range(colsVeh+1+1):
                line = wx.StaticLine(self.panel, style=wx.LI_HORIZONTAL)
                self.sizer.Add(line, proportion=0,
                               flag=wx.EXPAND | wx.ALL, border=0)

        # add "unassigned" row
        panelUnassigned = UnassignedPanel(self.state, self.panel)
        self.sizer.Add(panelUnassigned, flag=wx.EXPAND |
                       wx.LEFT | wx.RIGHT, border=5)
        line = wx.StaticLine(self.panel, style=wx.LI_VERTICAL)
        self.sizer.Add(line, proportion=0, flag=wx.EXPAND |
                       wx.LEFT | wx.RIGHT, border=5)

        for veh in unassignedVeh:
            panelunassigned = VehiclePanel(
                self.state, self.panel, veh[0], veh[1], veh[2], False, None, self.takeoffalt)
            self.sizer.Add(panelunassigned)

        self.panel.SetSizer(self.sizer)
        self.panel.Layout()
        self.panel.SetupScrolling()

    def onmavlinkpacket(self, msg):
        ''' on new mavlink packet, update relevant GUI elements'''
        children = self.sizer.GetChildren()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()
        mtype = msg.get_type()

        for child in children:
            widget = child.GetWindow()
            # update arm and mode status on leader
            if isinstance(widget, VehiclePanel) and widget.sysid == sysid and widget.compid == compid:
                if mtype == 'HEARTBEAT':
                    mode_map = mavutil.mode_mapping_bynumber(msg.type)
                    isArmed = True if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 128 else False
                    widget.updateData(isArmed, mode_map[msg.custom_mode])
                    sysStatus = msg.system_status
                    if sysStatus == mavutil.mavlink.MAV_STATE_BOOT:
                        widget.updateStatus("Booting")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_CALIBRATING:
                        widget.updateStatus("Calibrating")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_STANDBY:
                        widget.updateStatus("On Ground")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_ACTIVE:
                        widget.updateStatus("Flying")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_CRITICAL:
                        widget.updateStatus("FAILSAFE")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_EMERGENCY:
                        widget.updateStatus("Emergency")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_POWEROFF:
                        widget.updateStatus("Poweroff")
                    elif sysStatus == mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION:
                        widget.updateStatus("Terminated")
                elif mtype == 'VFR_HUD':
                    widget.updatethralt(msg.throttle, msg.alt)
                elif mtype == 'GLOBAL_POSITION_INT':
                    widget.updaterelalt(msg.relative_alt * 1.0e-3)
                elif mtype == "SYS_STATUS":
                    preArmGood = ((msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
                                  == mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
                    widget.updateprearm(preArmGood)
                    widget.updatevoltage(float(msg.voltage_battery)/1000)
                elif mtype == "STATUSTEXT" and msg.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                    # Only pass on warning messages or more severe
                    widget.addstatustext(msg.text)
                elif mtype == 'PARAM_VALUE' and msg.param_id in self.parmsToShow:
                    widget.updateOffset(msg.param_id, int(msg.param_value))


class swarm(mp_module.MPModule):
    '''
    The MAVProxy module that manages the GUI
    '''
    def __init__(self, mpstate):
        '''Initialise module'''
        super(swarm, self).__init__(mpstate, "swarm",
                                    "swarm module", multi_vehicle=True)

        # array of tuples for (SYSID, COMPID, FOLL_SYSID, veh_type) of all detected vehicles
        self.vehicleListing = []

        self.add_command('swarm', self.cmd_swarm, "swarm control",
                         ["<status>", "set (SWARMSETTING)"])

        self.swarm_settings = mp_settings.MPSettings(
            [("takeoffalt", int, 10)])  # meters
        self.add_completion_function('(SWARMSETTING)',
                                     self.swarm_settings.completion)

        # Which params to show on the GUI per vehicle
        self.parmsToShow = ["FOLL_OFS_X", "FOLL_OFS_Y", "FOLL_OFS_Z"]

        # time of last HB for each vehicle. Key is tuple of sysid,compid. Value is time of last HB
        self.vehicleLastHB = {}
        self.needHBupdate_timer = mavutil.periodic_event(1)

        # Periodic event to update the GUI
        self.needGUIupdate = False
        self.needGUIupdate_timer = mavutil.periodic_event(1)

        # Periodic event to send param (offset) requests (5 Hz)
        self.requestParams_timer = mavutil.periodic_event(5)

        # Periodic event re-get params (0.1 Hz)
        self.RerequestParams_timer = mavutil.periodic_event(0.1)

        # List of any vehicles we still need to get params for
        self.vehParamsToGet = []

        # All vehicle positions. Dict. Key is sysid, value is tuple of (lat,lon,alt)
        self.allVehPos = {}

        self.validVehicles = frozenset([mavutil.mavlink.MAV_TYPE_FIXED_WING,
                              mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                              mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                              mavutil.mavlink.MAV_TYPE_SUBMARINE,
                              mavutil.mavlink.MAV_TYPE_QUADROTOR,
                              mavutil.mavlink.MAV_TYPE_COAXIAL,
                              mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                              mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                              mavutil.mavlink.MAV_TYPE_TRICOPTER,
                              mavutil.mavlink.MAV_TYPE_HELICOPTER,
                              mavutil.mavlink.MAV_TYPE_DODECAROTOR,
                              mavutil.mavlink.MAV_TYPE_AIRSHIP])

        # The GUI
        self.gui = SwarmUI(
            self.parmsToShow, takeoffalt=self.swarm_settings.get('takeoffalt'))

    def cmd_swarm(self, args):
        '''swarm command parser'''
        usage = "usage: swarm <set>"
        if len(args) == 0:
            print(usage)
        elif args[0] == "set":
            self.swarm_settings.command(args[1:])
            if len(args) == 3:
                self.gui.changesetting(args[1], args[2])
        else:
            print(usage)

    def set_layout(self, layout):
        '''set window layout'''
        self.gui.parent_pipe.send([layout])

    def idle_task(self):
        '''run on idle'''
        # send updated HB stats to GUI every 1 sec
        if self.needHBupdate_timer.trigger():
            self.gui.updateHB(self.vehicleLastHB)

        # do we need to update the GUI?
        if self.needGUIupdate_timer.trigger() and self.needGUIupdate:
            self.gui.updateLayout(self.vehicleListing)
            self.needGUIupdate = False

        # do we need to get any vehicle follow sysid params?
        # only send param requests 1 per 0.1sec, to avoid link flooding
        if self.requestParams_timer.trigger() and len(self.vehParamsToGet) > 0:
            (sysid, compid) = self.vehParamsToGet.pop(0)
            self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.param_request_read_send(
                sysid, compid, parmString("FOLL_SYSID"), -1))

        if self.RerequestParams_timer.trigger():
            # If any in vehicleListing are missing their FOLL_SYSID, re-request
            for veh in self.vehicleListing:
                if veh[2] == 0:
                    self.vehParamsToGet.append(((veh[0], veh[1])))

        # execute any commands from GUI via parent_pipe
        if self.gui.parent_pipe.poll():
            (cmd, sysid, compid) = self.gui.parent_pipe.recv()
            if isinstance(cmd, win_layout.WinLayout):
                win_layout.set_layout(cmd, self.set_layout)
            if cmd == "arm":
                self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.command_long_send(
                    # self.master.mav.command_long_send(
                    sysid,  # target_system
                    compid,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                    0,  # confirmation
                    1,  # param1 (1 to indicate arm)
                    0,  # param2  (all other params meaningless)
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0))  # param7
            elif cmd == "disarm":
                self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.command_long_send(
                    sysid,  # target_system
                    compid,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                    0,  # confirmation
                    0,  # param1 (1 to indicate arm)
                    0,  # param2  (all other params meaningless)
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0))  # param7
            elif cmd == "takeoff":
                self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.command_long_send(
                    sysid,  # target_system
                    compid,  # target_component
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
                    0,  # confirmation
                    0,  # param1
                    0,  # param2
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    self.swarm_settings.takeoffalt))  # param7
            elif cmd in ["FOLLOW", "RTL", "AUTO", "GUIDED"]:
                mode_mapping = self.master.mode_mapping()
                self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.command_long_send(sysid,
                                                                                          compid,
                                                                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                                                                          0,
                                                                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                          mode_mapping[cmd],
                                                                                          0,
                                                                                          0,
                                                                                          0,
                                                                                          0,
                                                                                          0))
            elif cmd == "kill":
                self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.command_long_send(
                    sysid,  # target_system
                    compid,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                    0,  # confirmation
                    0,  # param1 (0 to indicate disarm)
                    21196,  # param2 (indicates force disarm)
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0))  # param7
            elif cmd == 'getoffsets':
                # time.sleep(0.1)
                for parm in self.parmsToShow:
                    self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.param_request_read_send(
                        sysid, compid, parmString(parm), -1))
            elif cmd == 'resetLayout':
                self.vehicleListing = []
                self.vehicleLastHB = {}
                self.needGUIupdate = True
                # self.gui.updateLayout(self.vehicleListing)
            elif cmd == 'getparams':
                for (sysid, compid, foll_sysid, veh_type) in self.vehicleListing:
                    # time.sleep(0.1)
                    for parm in self.parmsToShow:
                        self.mpstate.foreach_mav(sysid, compid, lambda mav: mav.param_request_read_send(
                            sysid, compid, parmString(parm), -1))

    def mavlink_packet(self, m):
        '''handle incoming mavlink packets'''
        mtype = m.get_type()
        sysid = m.get_srcSystem()
        compid = m.get_srcComponent()

        # add to GUI if vehicle not seen before
        if mtype == 'HEARTBEAT' and m.type in self.validVehicles and not (sysid in [sysidList[0] for sysidList in self.vehicleListing] and compid in [compidList[1] for compidList in self.vehicleListing]):
            self.vehicleListing.append((sysid, compid, 0, m.type))
            # figure out leader for vehicle - check FOLL_SYSID
            self.vehParamsToGet.append((sysid, compid))
            self.needGUIupdate = True
            self.vehicleLastHB[(sysid, compid)] = time.time()
        # Only send these packets on if the vehicle is already in the list
        elif (sysid, compid) in self.vehicleLastHB.keys():
            # update time vehicle was last seen
            if mtype == 'HEARTBEAT':
                self.vehicleLastHB[(sysid, compid)] = time.time()

            # updated leader information from vehicle
            elif mtype == 'PARAM_VALUE' and m.param_id == "FOLL_SYSID":
                for i in range(0, len(self.vehicleListing)):
                    # only update if the leader ID has changed (avoids unessariliy refreshing the UI)
                    if self.vehicleListing[i][0] == sysid and self.vehicleListing[i][1] == compid and self.vehicleListing[i][2] != int(m.param_value):
                        self.vehicleListing[i] = (self.vehicleListing[i][0], self.vehicleListing[i][1], int(
                            m.param_value), self.vehicleListing[i][3])
                        # get GUI to update layout
                        self.needGUIupdate = True
                        break

            # pass to gui elements. Only send relevant packets, otherwise will slow the GUI
            if mtype in ['HEARTBEAT', 'VFR_HUD', 'GLOBAL_POSITION_INT', "SYS_STATUS", "STATUSTEXT", 'PARAM_VALUE']:
                self.gui.onmavlinkpacket(m)


def init(mpstate):
    '''initialise module'''
    return swarm(mpstate)
