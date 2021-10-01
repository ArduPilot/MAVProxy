#!/usr/bin/env python

'''
Swarming support
Stephen Dade, July 2021

Helper functions for managing leader-follower swarms

'''

import time
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_util

class swarmUI():
    '''
    Swarm GUI
    '''
    def __init__(self, title='MAVProxy: Swarm Control'):
        self.title  = title
        self.menu_callback = None
        self.parent_pipe,self.child_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()

        app = wx.App(False)
        app.frame = swarmFrame(state=self, title=self.title)
        app.frame.Show()
        app.MainLoop()

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

    def onmavlinkpacket(self, pkt):
        '''set a status value'''
        if self.child.is_alive():
            self.parent_pipe.send(('onmavlinkpacket', pkt))
            
class followerPanel(wx.Panel):
    """ Custom panel for all follower controls """
    def __init__(self, parent, sysid, compid):
        # sysid and compid are of the follower
        wx.Panel.__init__(self, parent)
        
        self.sysid = sysid
        self.compid = compid
        
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.title = wx.StaticText(self, label = "Veh\n{0}:{1}".format(sysid, compid))
        self.armmode = wx.StaticText(self, label = "armed\nmode123456")
        
        self.sizer.Add(self.title)
        self.sizer.Add(self.armmode)
        self.SetSizer(self.sizer)
        
    def updateData(self, armed, mode):
        # update the arming/mode status
        self.armmode.SetLabel(str(armed) + "\n" + str(mode))

class leaderPanel(wx.Panel):
    """ Custom panel for all leader controls """
    def __init__(self, parent, sysid, compid):
        # sysid and compid are of the follower
        wx.Panel.__init__(self, parent)
        
        self.sysid = sysid
        self.compid = compid
        
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.title = wx.StaticText(self, label = "Leader\n{0}:{1}".format(sysid, compid))
        self.armmode = wx.StaticText(self, label = "armed\nmode123456")
        
        self.sizer.Add(self.title)
        self.sizer.Add(self.armmode)
        self.SetSizer(self.sizer)

    def updateData(self, armed, mode):
        # update the arming/mode status
        self.armmode.SetLabel(str(armed) + "\n" + str(mode))
                    
class swarmFrame(wx.Frame):
    """ The main frame of the UI. Runs in a subprocess, so need a pipe to communicate with"""

    def __init__(self, state, title):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=(1100,400), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)
        self.panel = wx.Panel(self)

        #layout. Column per leader, with followers underneath
        self.sizer = wx.FlexGridSizer(1, 1, 10, 10)
        
        #add in the pipe from MAVProxy
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, lambda evt, panel=self.panel: self.on_timer(evt), self.timer)
        self.timer.Start(100)

        # layout
        self.panel.SetSizer(self.sizer)
        self.panel.Layout()
        self.Show(True)


    #Receive messages from MAVProxy and process them
    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        while state.child_pipe.poll():
            obj = state.child_pipe.recv()
            if obj[0] == 'updatelayout':
                self.updateLayout(obj[1])
            if obj[0] == 'onmavlinkpacket':
                self.onmavlinkpacket(obj[1])
                            
    def updateLayout(self, layout):
        print("here")
        print(layout)
        leaders = [] #array of (sysid, compid, followid)
        #followers = [] 
        
        # create list of leaders
        for veh in layout:
            if veh[2] != 0:
                for vehL in layout:
                    if vehL[0] == veh[2] and vehL not in leaders:
                        leaders.append(vehL)
                        
        # create list of followers
        followers = {} #dict of key=leadersysid, (sysid, compid, followid)
        maxfollowers = 0
        for veh in layout:
            # don't include leaders in the follower list
            if veh[2] != 0:
                if veh[2] not in followers.keys():
                    followers[veh[2]] = [(veh[0], veh[1])]
                else:
                    followers[veh[2]].append((veh[0], veh[1]))
                if len(followers[veh[2]]) > maxfollowers:
                    maxfollowers = len(followers[veh[2]])
                    
            
        self.sizer.Clear(True)
        self.sizer = wx.FlexGridSizer(len(leaders), maxfollowers+1, 10, 10)
        
        #start populating the grid
        #for each row:
        for leader in leaders:
            #panelLeader = wx.StaticText(self.panel, label = "Leader: ({0},{1})".format(leader[0],leader[1]))
            panelLeader = leaderPanel(self.panel, leader[0], leader[1])
            self.sizer.Add(panelLeader)
            for follower in range(maxfollowers):
                if leader[0] in followers.keys() and len(followers[leader[0]]) > follower:
                    #panel = vehPanel(self.panel, label = "Veh: {0}".format(followers[leader[0]][follower]))
                    panelFollower = followerPanel(self.panel, followers[leader[0]][follower][0], followers[leader[0]][follower][1])
                else:
                    panelFollower = wx.StaticText(self.panel, label = "N/A")
                self.sizer.Add(panelFollower)
                        
        self.panel.SetSizer(self.sizer)
        self.panel.Layout()
        
    def onmavlinkpacket(self, msg):
        """ on new pmavlink packet, update relevant GUI elements"""
        children = self.sizer.GetChildren()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()
        mtype = msg.get_type()
        
        for child in children:
            widget = child.GetWindow()
            # update arm and mode status on leader
            if (isinstance(widget, leaderPanel) or isinstance(widget, followerPanel)) and widget.sysid == sysid and widget.compid == compid and mtype == 'HEARTBEAT':
                mode_map = mavutil.mode_mapping_bynumber(msg.type)
                isArmed = "Armed" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 128 else "Disarmed"
                widget.updateData(isArmed, mode_map[msg.custom_mode])
                
        
class swarm(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(swarm, self).__init__(mpstate, "swarm", "swarm module",multi_vehicle=True)
        
        # array of tuples for (SYSID, COMPID, FOLL_SYSID) of all detected vehicles
        self.vehicleListing = []
        
        self.validVehicles = [mavutil.mavlink.MAV_TYPE_FIXED_WING,
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
                              mavutil.mavlink.MAV_TYPE_AIRSHIP]
                              
        # The GUI
        self.gui = swarmUI()

        self.swarm_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ('leader', int, 1),
             ('cmddelay', int, 1),
            ]
        )
        self.add_command('swarm',
                         self.cmd_swarm,
                         "swarm control",
                         ['set (SWARMSETTING)',
                          'armfollowers',
                          'armall',
                          'disarmfollowers',
                          'disarmall',
                          'modefollowers',
                          'modeall'
                         ])
        
    def usage(self):
        '''show help on command line options'''
        return "Usage: swarm <set|armfollowers|armall|disarmfollowers|disarmall|modefollowers|modeall>"

    def cmd_send(self, doLeader, doFollowers, args):
        '''send command to leader and/or followers'''
        saved_target = self.mpstate.settings.target_system
        linkmod = self.module('link')
        
        for v in sorted(self.mpstate.vehicle_list):
            if doLeader and int(v) == self.swarm_settings.leader:
                linkmod.cmd_vehicle([str(v)])
                self.mpstate.functions.process_stdin(' '.join(args), True)
                time.sleep(self.swarm_settings.cmddelay/1000)
            elif doFollowers and int(v) != self.swarm_settings.leader:
                linkmod.cmd_vehicle([str(v)])
                self.mpstate.functions.process_stdin(' '.join(args), True)
                time.sleep(self.swarm_settings.cmddelay/1000)
        linkmod.cmd_vehicle([str(saved_target)])

    def cmd_swarm(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "set":
            self.swarm_settings.command(args[1:])
        elif args[0] == "armfollowers":
            # arm all the followers
            self.cmd_send(False, True, ["arm", "throttle"])
        elif args[0] == "armall":
            # arm all
            self.cmd_send(True, True, ["arm", "throttle"])
        elif args[0] == "disarmfollowers":
            # disarm all the followers
            self.cmd_send(False, True, ["disarm"])
        elif args[0] == "disarmall":
            # disarm all
            self.cmd_send(True, True, ["disarm"])
        elif args[0] == "modefollowers" and len(args) == 2:
            # set mode for followers
            self.cmd_send(False, True, ["mode"] + [args[1]])
        elif args[0] == "modefollowers":
            print("Usage: swarm modefollowers MODE")
        elif args[0] == "modeall"and len(args) == 2:
            # set mode for all
            self.cmd_send(True, True, ["mode"] + [args[1]])
        elif args[0] == "modeall":
            print("Usage: swarm modeall MODE")
                                    
    def idle_task(self):
        '''called rapidly by mavproxy'''

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        sysid = m.get_srcSystem()
        compid = m.get_srcComponent()
        
        # add new vehicle on new heartbeat
        if mtype == 'HEARTBEAT' and m.type in self.validVehicles:
            if not sysid in [sysidList[0] for sysidList in self.vehicleListing]:
                self.vehicleListing.append((sysid, compid, 0))
                print("New vehicle {0}:{1}".format(sysid, compid))
                # get leader for vehicle
                self.getLeaderUpdate(sysid, compid)
            
        # updated leader information from vehicle
        if mtype == 'PARAM_VALUE' and m.param_id == "FOLL_SYSID":
            print("Got leader ID {0} for vehicle {1}".format(int(m.param_value), m.get_srcSystem()))
            for i in range(0, len(self.vehicleListing)):
                if self.vehicleListing[i][0] == sysid and self.vehicleListing[i][1] == compid:
                    self.vehicleListing[i] = (sysid, compid, int(m.param_value))
                    break
            # get GUI to update layout
            self.gui.updateLayout(self.vehicleListing)
            
        # pass to gui elements
        self.gui.onmavlinkpacket(m)

    def getLeaderUpdate(self, sysid=None, compid=None):
        '''Ask for updated FOLL_SYSID params for all vehicles, or a specific sysid/compid'''
        if sysid is not None and compid is not None:
            for mst in self.mpstate.mav_master:
                mst.mav.param_request_read_send(sysid, compid, bytes("FOLL_SYSID",'ascii'), -1)
        else:
            for veh in self.vehicleListing:
                for mst in self.mpstate.mav_master:
                    mst.mav.param_request_read_send(self.vehicleListing[0], self.vehicleListing[1], bytes("FOLL_SYSID",'ascii'), -1)
                
def init(mpstate):
    '''initialise module'''
    return swarm(mpstate)
