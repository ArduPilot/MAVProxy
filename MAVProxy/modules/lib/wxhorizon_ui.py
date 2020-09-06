import time
from MAVProxy.modules.lib.wxhorizon_util import Attitude, VFR_HUD, Global_Position_INT, BatteryInfo, FlightState, WaypointInfo, FPS
from MAVProxy.modules.lib.wx_loader import wx
import math, time

import matplotlib
matplotlib.use('wxAgg')
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.pyplot import Polygon
import matplotlib.patheffects as PathEffects
from matplotlib import patches
import matplotlib as mpl

class HorizonFrame(wx.Frame):
    """ The main frame of the horizon indicator."""

    def __init__(self, state, title):
        self.state = state
        # Create Frame and Panel(s)
        wx.Frame.__init__(self, None, title=title)
        state.frame = self

        # Initialisation
        self.initData()
        self.initUI()
        self.startTime = time.time()
        self.nextTime = 0.0
        self.fps = 10.0

    def initData(self):
        # Initialise Attitude
        self.pitch = 0.0  # Degrees
        self.roll = 0.0   # Degrees
        self.yaw = 0.0    # Degrees
        
        # History Values
        self.oldRoll = 0.0 # Degrees
        
        # Initialise Rate Information
        self.airspeed = 0.0 # m/s
        self.relAlt = 0.0 # m relative to home position
        self.relAltTime = 0.0 # s The time that the relative altitude was recorded
        self.climbRate = 0.0 # m/s
        self.altHist = [] # Altitude History
        self.timeHist = [] # Time History
        self.altMax = 0.0 # Maximum altitude since startup
        
        # Initialise HUD Info
        self.heading = 0.0 # 0-360
        
        # Initialise Battery Info
        self.voltage = 0.0
        self.current = 0.0
        self.batRemain = 0.0

        # Initialise Mode and State
        self.mode = 'UNKNOWN'
        self.armed = ''
        self.safetySwitch = ''
        
        # Intialise Waypoint Information
        self.currentWP = 0
        self.finalWP = 0
        self.wpDist = 0
        self.nextWPTime = 0
        self.wpBearing = 0
    

    def initUI(self):
        # Create Event Timer and Bindings
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_CHAR_HOOK,self.on_KeyPress)

        # Create Panel
        self.panel = wx.Panel(self)
        self.vertSize = 0.09
        self.resized = False
        
        # Create Matplotlib Panel
        self.createPlotPanel()

        # Fix Axes - vertical is of length 2, horizontal keeps the same lengthscale
        self.rescaleX()
        self.calcFontScaling()        
        
        # Create Horizon Polygons
        self.createHorizonPolygons()
        
        # Center Pointer Marker
        self.thick = 0.015
        self.createCenterPointMarker()
        
        # Pitch Markers
        self.dist10deg = 0.2 # Graph distance per 10 deg
        self.createPitchMarkers()
        
        # Add Roll, Pitch, Yaw Text
        self.createRPYText()
        
        # Add Airspeed, Altitude, Climb Rate Text
        self.createAARText()
        
        # Create Heading Pointer
        self.createHeadingPointer()
        
        # Create North Pointer
        self.createNorthPointer()
        
        # Create Battery Bar
        self.batWidth = 0.1
        self.batHeight = 0.2
        self.rOffset = 0.35
        self.createBatteryBar()
        
        # Create Mode & State Text
        self.createStateText()
        
        # Create Waypoint Text
        self.createWPText()
        
        # Create Waypoint Pointer
        self.createWPPointer()
        
        # Create Altitude History Plot
        self.createAltHistoryPlot()
        
        # Show Frame
        self.Show(True)
        self.pending = []
    
    def createPlotPanel(self):
        '''Creates the figure and axes for the plotting panel.'''
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        self.canvas = FigureCanvas(self,-1,self.figure)
        self.canvas.SetSize(wx.Size(300,300))
        self.axes.axis('off')
        self.figure.subplots_adjust(left=0,right=1,top=1,bottom=0)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas,1,wx.EXPAND,wx.ALL)
        self.SetSizerAndFit(self.sizer)
        self.Fit()
        
    def rescaleX(self):
        '''Rescales the horizontal axes to make the lengthscales equal.'''
        self.ratio = self.figure.get_size_inches()[0]/float(self.figure.get_size_inches()[1])
        self.axes.set_xlim(-self.ratio,self.ratio)
        self.axes.set_ylim(-1,1)
        
    def calcFontScaling(self):
        '''Calculates the current font size and left position for the current window.'''
        self.ypx = self.figure.get_size_inches()[1]*self.figure.dpi
        self.xpx = self.figure.get_size_inches()[0]*self.figure.dpi
        self.fontSize = self.vertSize*(self.ypx/2.0)
        self.leftPos = self.axes.get_xlim()[0]
        self.rightPos = self.axes.get_xlim()[1]
    
    def checkReszie(self):
        '''Checks if the window was resized.'''
        if not self.resized:
            oldypx = self.ypx
            oldxpx = self.xpx
            self.ypx = self.figure.get_size_inches()[1]*self.figure.dpi
            self.xpx = self.figure.get_size_inches()[0]*self.figure.dpi
            if (oldypx != self.ypx) or (oldxpx != self.xpx):
                self.resized = True
            else:
                self.resized = False
    
    def createHeadingPointer(self):
        '''Creates the pointer for the current heading.'''
        self.headingTri = patches.RegularPolygon((0.0,0.80),3,0.05,color='k',zorder=4)
        self.axes.add_patch(self.headingTri)
        self.headingText = self.axes.text(0.0,0.675,'0',color='k',size=self.fontSize,horizontalalignment='center',verticalalignment='center',zorder=4)
    
    def adjustHeadingPointer(self):
        '''Adjust the value of the heading pointer.'''
        self.headingText.set_text(str(self.heading))
        self.headingText.set_size(self.fontSize) 
    
    def createNorthPointer(self):
        '''Creates the north pointer relative to current heading.'''
        self.headingNorthTri = patches.RegularPolygon((0.0,0.80),3,0.05,color='k',zorder=4)
        self.axes.add_patch(self.headingNorthTri)
        self.headingNorthText = self.axes.text(0.0,0.675,'N',color='k',size=self.fontSize,horizontalalignment='center',verticalalignment='center',zorder=4)    

    def adjustNorthPointer(self):
        '''Adjust the position and orientation of
        the north pointer.'''
        self.headingNorthText.set_size(self.fontSize) 
        headingRotate = mpl.transforms.Affine2D().rotate_deg_around(0.0,0.0,self.heading)+self.axes.transData
        self.headingNorthText.set_transform(headingRotate)
        if (self.heading > 90) and (self.heading < 270):
            headRot = self.heading-180
        else:
            headRot = self.heading
        self.headingNorthText.set_rotation(headRot)
        self.headingNorthTri.set_transform(headingRotate)
        # Adjust if overlapping with heading pointer
        if (self.heading <= 10.0) or (self.heading >= 350.0):
            self.headingNorthText.set_text('')
        else:
            self.headingNorthText.set_text('N')
            
    def toggleWidgets(self,widgets):
        '''Hides/shows the given widgets.'''
        for wig in widgets:
            if wig.get_visible():
                wig.set_visible(False)
            else:
                wig.set_visible(True)
    
    def createRPYText(self):
        '''Creates the text for roll, pitch and yaw.'''
        self.rollText = self.axes.text(self.leftPos+(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0),'Roll:   %.2f' % self.roll,color='w',size=self.fontSize)
        self.pitchText = self.axes.text(self.leftPos+(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0),'Pitch: %.2f' % self.pitch,color='w',size=self.fontSize)
        self.yawText = self.axes.text(self.leftPos+(self.vertSize/10.0),-0.97,'Yaw:   %.2f' % self.yaw,color='w',size=self.fontSize)
        self.rollText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.pitchText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.yawText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        
    def updateRPYLocations(self):
        '''Update the locations of roll, pitch, yaw text.'''
        # Locations
        self.rollText.set_position((self.leftPos+(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0)))
        self.pitchText.set_position((self.leftPos+(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0)))
        self.yawText.set_position((self.leftPos+(self.vertSize/10.0),-0.97))
        # Font Size
        self.rollText.set_size(self.fontSize)
        self.pitchText.set_size(self.fontSize)
        self.yawText.set_size(self.fontSize)
        
    def updateRPYText(self):
        'Updates the displayed Roll, Pitch, Yaw Text'
        self.rollText.set_text('Roll:   %.2f' % self.roll)
        self.pitchText.set_text('Pitch: %.2f' % self.pitch)
        self.yawText.set_text('Yaw:   %.2f' % self.yaw)
        
    def createCenterPointMarker(self):
        '''Creates the center pointer in the middle of the screen.'''
        self.axes.add_patch(patches.Rectangle((-0.75,-self.thick),0.5,2.0*self.thick,facecolor='orange',zorder=3))
        self.axes.add_patch(patches.Rectangle((0.25,-self.thick),0.5,2.0*self.thick,facecolor='orange',zorder=3))
        self.axes.add_patch(patches.Circle((0,0),radius=self.thick,facecolor='orange',edgecolor='none',zorder=3))
        
    def createHorizonPolygons(self):
        '''Creates the two polygons to show the sky and ground.'''
        # Sky Polygon
        vertsTop = [[-1,0],[-1,1],[1,1],[1,0],[-1,0]]
        self.topPolygon = Polygon(vertsTop,facecolor='dodgerblue',edgecolor='none')
        self.axes.add_patch(self.topPolygon)
        # Ground Polygon
        vertsBot = [[-1,0],[-1,-1],[1,-1],[1,0],[-1,0]]
        self.botPolygon = Polygon(vertsBot,facecolor='brown',edgecolor='none')
        self.axes.add_patch(self.botPolygon)
        
    def calcHorizonPoints(self):
        '''Updates the verticies of the patches for the ground and sky.'''
        ydiff = math.tan(math.radians(-self.roll))*float(self.ratio)
        pitchdiff = self.dist10deg*(self.pitch/10.0)
        # Draw Polygons
        vertsTop = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,1),(self.ratio,1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]
        vertsBot = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,-1),(self.ratio,-1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]
        if (self.roll > 90) or (self.roll < -90):
            vertsTop = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,-1),(self.ratio,-1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]
            vertsBot = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,1),(self.ratio,1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]
        self.topPolygon.set_xy(vertsTop)
        self.botPolygon.set_xy(vertsBot)  
    
    def createPitchMarkers(self):
        '''Creates the rectangle patches for the pitch indicators.'''
        self.pitchPatches = []
        # Major Lines (multiple of 10 deg)
        for i in [-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9]:
            width = self.calcPitchMarkerWidth(i)
            currPatch = patches.Rectangle((-width/2.0,self.dist10deg*i-(self.thick/2.0)),width,self.thick,facecolor='w',edgecolor='none')
            self.axes.add_patch(currPatch)
            self.pitchPatches.append(currPatch)
        # Add Label for +-30 deg
        self.vertSize = 0.09
        self.pitchLabelsLeft = []
        self.pitchLabelsRight = []
        i=0
        for j in [-90,-60,-30,30,60,90]:
            self.pitchLabelsLeft.append(self.axes.text(-0.55,(j/10.0)*self.dist10deg,str(j),color='w',size=self.fontSize,horizontalalignment='center',verticalalignment='center'))
            self.pitchLabelsLeft[i].set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
            self.pitchLabelsRight.append(self.axes.text(0.55,(j/10.0)*self.dist10deg,str(j),color='w',size=self.fontSize,horizontalalignment='center',verticalalignment='center'))
            self.pitchLabelsRight[i].set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
            i += 1
        
    def calcPitchMarkerWidth(self,i):
        '''Calculates the width of a pitch marker.'''
        if (i % 3) == 0:
            if i == 0:
                width = 1.5
            else:
                width = 0.9
        else:
            width = 0.6
            
        return width
        
    def adjustPitchmarkers(self):
        '''Adjusts the location and orientation of pitch markers.'''
        pitchdiff = self.dist10deg*(self.pitch/10.0)
        rollRotate = mpl.transforms.Affine2D().rotate_deg_around(0.0,-pitchdiff,self.roll)+self.axes.transData
        j=0
        for i in [-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9]:
            width = self.calcPitchMarkerWidth(i)
            self.pitchPatches[j].set_xy((-width/2.0,self.dist10deg*i-(self.thick/2.0)-pitchdiff))
            self.pitchPatches[j].set_transform(rollRotate)
            j+=1
        # Adjust Text Size and rotation
        i=0
        for j in [-9,-6,-3,3,6,9]:
                self.pitchLabelsLeft[i].set_y(j*self.dist10deg-pitchdiff)
                self.pitchLabelsRight[i].set_y(j*self.dist10deg-pitchdiff)
                self.pitchLabelsLeft[i].set_size(self.fontSize)
                self.pitchLabelsRight[i].set_size(self.fontSize)
                self.pitchLabelsLeft[i].set_rotation(self.roll)
                self.pitchLabelsRight[i].set_rotation(self.roll)
                self.pitchLabelsLeft[i].set_transform(rollRotate)
                self.pitchLabelsRight[i].set_transform(rollRotate)
                i += 1 
                
    def createAARText(self):
        '''Creates the text for airspeed, altitude and climb rate.'''
        self.airspeedText = self.axes.text(self.rightPos-(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0),'AS:   %.1f m/s' % self.airspeed,color='w',size=self.fontSize,ha='right')
        self.altitudeText = self.axes.text(self.rightPos-(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0),'ALT: %.1f m   ' % self.relAlt,color='w',size=self.fontSize,ha='right')
        self.climbRateText = self.axes.text(self.rightPos-(self.vertSize/10.0),-0.97,'CR:   %.1f m/s' % self.climbRate,color='w',size=self.fontSize,ha='right')
        self.airspeedText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.altitudeText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.climbRateText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        
    def updateAARLocations(self):
        '''Update the locations of airspeed, altitude and Climb rate.'''
        # Locations
        self.airspeedText.set_position((self.rightPos-(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0)))
        self.altitudeText.set_position((self.rightPos-(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0)))
        self.climbRateText.set_position((self.rightPos-(self.vertSize/10.0),-0.97))
        # Font Size
        self.airspeedText.set_size(self.fontSize)
        self.altitudeText.set_size(self.fontSize)
        self.climbRateText.set_size(self.fontSize)
        
    def updateAARText(self):
        'Updates the displayed airspeed, altitude, climb rate Text'
        self.airspeedText.set_text('AR:   %.1f m/s' % self.airspeed)
        self.altitudeText.set_text('ALT: %.1f m   ' % self.relAlt)
        self.climbRateText.set_text('CR:   %.1f m/s' % self.climbRate)
    
    def createBatteryBar(self):
        '''Creates the bar to display current battery percentage.'''
        self.batOutRec = patches.Rectangle((self.rightPos-(1.3+self.rOffset)*self.batWidth,1.0-(0.1+1.0+(2*0.075))*self.batHeight),self.batWidth*1.3,self.batHeight*1.15,facecolor='darkgrey',edgecolor='none')
        self.batInRec = patches.Rectangle((self.rightPos-(self.rOffset+1+0.15)*self.batWidth,1.0-(0.1+1+0.075)*self.batHeight),self.batWidth,self.batHeight,facecolor='lawngreen',edgecolor='none')
        self.batPerText = self.axes.text(self.rightPos - (self.rOffset+0.65)*self.batWidth,1-(0.1+1+(0.075+0.15))*self.batHeight,'%.f' % self.batRemain,color='w',size=self.fontSize,ha='center',va='top')
        self.batPerText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.voltsText = self.axes.text(self.rightPos-(self.rOffset+1.3+0.2)*self.batWidth,1-(0.1+0.05+0.075)*self.batHeight,'%.1f V' % self.voltage,color='w',size=self.fontSize,ha='right',va='top')
        self.ampsText = self.axes.text(self.rightPos-(self.rOffset+1.3+0.2)*self.batWidth,1-self.vertSize-(0.1+0.05+0.1+0.075)*self.batHeight,'%.1f A' % self.current,color='w',size=self.fontSize,ha='right',va='top')
        self.voltsText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.ampsText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        
        self.axes.add_patch(self.batOutRec)
        self.axes.add_patch(self.batInRec)
        
    def updateBatteryBar(self):
        '''Updates the position and values of the battery bar.'''
        # Bar
        self.batOutRec.set_xy((self.rightPos-(1.3+self.rOffset)*self.batWidth,1.0-(0.1+1.0+(2*0.075))*self.batHeight))
        self.batInRec.set_xy((self.rightPos-(self.rOffset+1+0.15)*self.batWidth,1.0-(0.1+1+0.075)*self.batHeight))
        self.batPerText.set_position((self.rightPos - (self.rOffset+0.65)*self.batWidth,1-(0.1+1+(0.075+0.15))*self.batHeight))
        self.batPerText.set_fontsize(self.fontSize)
        self.voltsText.set_text('%.1f V' % self.voltage)
        self.ampsText.set_text('%.1f A' % self.current)
        self.voltsText.set_position((self.rightPos-(self.rOffset+1.3+0.2)*self.batWidth,1-(0.1+0.05)*self.batHeight))
        self.ampsText.set_position((self.rightPos-(self.rOffset+1.3+0.2)*self.batWidth,1-self.vertSize-(0.1+0.05+0.1)*self.batHeight))
        self.voltsText.set_fontsize(self.fontSize)
        self.ampsText.set_fontsize(self.fontSize)
        if self.batRemain >= 0:
            self.batPerText.set_text(int(self.batRemain))
            self.batInRec.set_height(self.batRemain*self.batHeight/100.0)
            if self.batRemain/100.0 > 0.5:
                self.batInRec.set_facecolor('lawngreen')
            elif self.batRemain/100.0 <= 0.5 and self.batRemain/100.0 > 0.2:
                self.batInRec.set_facecolor('yellow')
            elif self.batRemain/100.0 <= 0.2 and self.batRemain >= 0.0:
                self.batInRec.set_facecolor('r')
        elif self.batRemain == -1:
            self.batInRec.set_height(self.batHeight)
            self.batInRec.set_facecolor('k')
        
    def createStateText(self):
        '''Creates the mode and arm state text.'''
        self.modeText = self.axes.text(self.leftPos+(self.vertSize/10.0),0.97,'UNKNOWN',color='grey',size=1.5*self.fontSize,ha='left',va='top')
        self.modeText.set_path_effects([PathEffects.withStroke(linewidth=self.fontSize/10.0,foreground='black')])
        
    def updateStateText(self):
        '''Updates the mode and colours red or green depending on arm state.'''
        self.modeText.set_position((self.leftPos+(self.vertSize/10.0),0.97))
        self.modeText.set_text(self.mode)
        self.modeText.set_size(1.5*self.fontSize)
        if self.armed:
            self.modeText.set_color('red')
            self.modeText.set_path_effects([PathEffects.withStroke(linewidth=self.fontSize/10.0,foreground='yellow')])
        elif (self.armed == False):
            self.modeText.set_color('lightgreen')
            self.modeText.set_bbox(None)
            self.modeText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='black')])
        else:
            # Fall back if unknown
            self.modeText.set_color('grey')
            self.modeText.set_bbox(None)
            self.modeText.set_path_effects([PathEffects.withStroke(linewidth=self.fontSize/10.0,foreground='black')])
        
    def createWPText(self):
        '''Creates the text for the current and final waypoint,
        and the distance to the new waypoint.'''
        self.wpText = self.axes.text(self.leftPos+(1.5*self.vertSize/10.0),0.97-(1.5*self.vertSize)+(0.5*self.vertSize/10.0),'0/0\n(0 m, 0 s)',color='w',size=self.fontSize,ha='left',va='top')
        self.wpText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='black')])
        
    def updateWPText(self):
        '''Updates the current waypoint and distance to it.''' 
        self.wpText.set_position((self.leftPos+(1.5*self.vertSize/10.0),0.97-(1.5*self.vertSize)+(0.5*self.vertSize/10.0)))
        self.wpText.set_size(self.fontSize)
        if type(self.nextWPTime) is str:
            self.wpText.set_text('%.f/%.f\n(%.f m, ~ s)' % (self.currentWP,self.finalWP,self.wpDist))
        else:
            self.wpText.set_text('%.f/%.f\n(%.f m, %.f s)' % (self.currentWP,self.finalWP,self.wpDist,self.nextWPTime))
    
    def createWPPointer(self):
        '''Creates the waypoint pointer relative to current heading.'''
        self.headingWPTri = patches.RegularPolygon((0.0,0.55),3,0.05,facecolor='lime',zorder=4,ec='k')
        self.axes.add_patch(self.headingWPTri)
        self.headingWPText = self.axes.text(0.0,0.45,'1',color='lime',size=self.fontSize,horizontalalignment='center',verticalalignment='center',zorder=4)   
        self.headingWPText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')]) 

    def adjustWPPointer(self):
        '''Adjust the position and orientation of
        the waypoint pointer.'''
        self.headingWPText.set_size(self.fontSize) 
        headingRotate = mpl.transforms.Affine2D().rotate_deg_around(0.0,0.0,-self.wpBearing+self.heading)+self.axes.transData
        self.headingWPText.set_transform(headingRotate)
        angle = self.wpBearing - self.heading
        if angle < 0:
            angle += 360
        if (angle > 90) and (angle < 270):
            headRot = angle-180
        else:
            headRot = angle
        self.headingWPText.set_rotation(-headRot)
        self.headingWPTri.set_transform(headingRotate)
        self.headingWPText.set_text('%.f' % (angle))
    
    def createAltHistoryPlot(self):
        '''Creates the altitude history plot.'''
        self.altHistRect = patches.Rectangle((self.leftPos+(self.vertSize/10.0),-0.25),0.5,0.5,facecolor='grey',edgecolor='none',alpha=0.4,zorder=4)
        self.axes.add_patch(self.altHistRect)
        self.altPlot, = self.axes.plot([self.leftPos+(self.vertSize/10.0),self.leftPos+(self.vertSize/10.0)+0.5],[0.0,0.0],color='k',marker=None,zorder=4)
        self.altMarker, = self.axes.plot(self.leftPos+(self.vertSize/10.0)+0.5,0.0,marker='o',color='k',zorder=4)
        self.altText2 = self.axes.text(self.leftPos+(4*self.vertSize/10.0)+0.5,0.0,'%.f m' % self.relAlt,color='k',size=self.fontSize,ha='left',va='center',zorder=4)
    
    def updateAltHistory(self):
        '''Updates the altitude history plot.'''
        self.altHist.append(self.relAlt)
        self.timeHist.append(self.relAltTime)
        
        # Delete entries older than x seconds
        histLim = 10
        currentTime = time.time()
        point = 0
        for i in range(0,len(self.timeHist)):
            if (self.timeHist[i] > (currentTime - 10.0)):
                break
        # Remove old entries
        self.altHist = self.altHist[i:]
        self.timeHist = self.timeHist[i:]
        
        # Transform Data
        x = []
        y = []
        tmin = min(self.timeHist)
        tmax = max(self.timeHist)
        x1 = self.leftPos+(self.vertSize/10.0)
        y1 = -0.25
        altMin = 0
        altMax = max(self.altHist)
        # Keep alt max for whole mission
        if altMax > self.altMax:
            self.altMax = altMax
        else:
            altMax = self.altMax
        if tmax != tmin:
            mx = 0.5/(tmax-tmin)
        else:
            mx = 0.0
        if altMax != altMin:
            my = 0.5/(altMax-altMin)
        else:
            my = 0.0
        for t in self.timeHist:
            x.append(mx*(t-tmin)+x1)
        for alt in self.altHist:
            val = my*(alt-altMin)+y1
            # Crop extreme noise
            if val < -0.25:
                val = -0.25
            elif val > 0.25:
                val = 0.25
            y.append(val)
        # Display Plot
        self.altHistRect.set_x(self.leftPos+(self.vertSize/10.0))
        self.altPlot.set_data(x,y)
        self.altMarker.set_data(self.leftPos+(self.vertSize/10.0)+0.5,val)
        self.altText2.set_position((self.leftPos+(4*self.vertSize/10.0)+0.5,val))
        self.altText2.set_size(self.fontSize)
        self.altText2.set_text('%.f m' % self.relAlt)
        
    # =============== Event Bindings =============== #    
    def on_idle(self, event):
        '''To adjust text and positions on rescaling the window when resized.'''
        # Check for resize
        self.checkReszie()
        
        if self.resized:
            # Fix Window Scales 
            self.rescaleX()
            self.calcFontScaling()
            
            # Recalculate Horizon Polygons
            self.calcHorizonPoints()
            
            # Update Roll, Pitch, Yaw Text Locations
            self.updateRPYLocations()
            
            # Update Airpseed, Altitude, Climb Rate Locations
            self.updateAARLocations()
            
            # Update Pitch Markers
            self.adjustPitchmarkers()
            
            # Update Heading and North Pointer
            self.adjustHeadingPointer()
            self.adjustNorthPointer()
            
            # Update Battery Bar
            self.updateBatteryBar()
            
            # Update Mode and State
            self.updateStateText()
            
            # Update Waypoint Text
            self.updateWPText()
            
            # Adjust Waypoint Pointer
            self.adjustWPPointer()
            
            # Update History Plot
            self.updateAltHistory()
            
            # Update Matplotlib Plot
            self.canvas.draw()
            self.canvas.Refresh()
            
            self.resized = False
        
        time.sleep(0.05)
 
    def on_timer(self, event):
        '''Main Loop.'''
        state = self.state
        self.loopStartTime = time.time()
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        
        # Check for resizing
        self.checkReszie()
        if self.resized:
            self.on_idle(0)
        
        # Get attitude information
        while state.child_pipe_recv.poll():           
            objList = state.child_pipe_recv.recv()
            for obj in objList:
                self.calcFontScaling()
                if isinstance(obj,Attitude):
                    self.oldRoll = self.roll
                    self.pitch = obj.pitch*180/math.pi
                    self.roll = obj.roll*180/math.pi
                    self.yaw = obj.yaw*180/math.pi
                    
                    # Update Roll, Pitch, Yaw Text Text
                    self.updateRPYText()
                    
                    # Recalculate Horizon Polygons
                    self.calcHorizonPoints()
                    
                    # Update Pitch Markers
                    self.adjustPitchmarkers()
                
                elif isinstance(obj,VFR_HUD):
                    self.heading = obj.heading
                    self.airspeed = obj.airspeed
                    self.climbRate = obj.climbRate
                    
                    # Update Airpseed, Altitude, Climb Rate Locations
                    self.updateAARText()
                    
                    # Update Heading North Pointer
                    self.adjustHeadingPointer()
                    self.adjustNorthPointer()
                
                elif isinstance(obj,Global_Position_INT):
                    self.relAlt = obj.relAlt
                    self.relAltTime = obj.curTime
                    
                    # Update Airpseed, Altitude, Climb Rate Locations
                    self.updateAARText()
                    
                    # Update Altitude History
                    self.updateAltHistory()
                    
                elif isinstance(obj,BatteryInfo):
                    self.voltage = obj.voltage
                    self.current = obj.current
                    self.batRemain = obj.batRemain
                    
                    # Update Battery Bar
                    self.updateBatteryBar()
                    
                elif isinstance(obj,FlightState):
                    self.mode = obj.mode
                    self.armed = obj.armState
                    
                    # Update Mode and Arm State Text
                    self.updateStateText()
                    
                elif isinstance(obj,WaypointInfo):
                    self.currentWP = obj.current
                    self.finalWP = obj.final
                    self.wpDist = obj.currentDist
                    self.nextWPTime = obj.nextWPTime
                    if obj.wpBearing < 0.0:
                        self.wpBearing = obj.wpBearing + 360
                    else:
                        self.wpBearing = obj.wpBearing
                    
                    # Update waypoint text
                    self.updateWPText()
                    
                    # Adjust Waypoint Pointer
                    self.adjustWPPointer()
                    
                elif isinstance(obj, FPS):
                    # Update fps target
                    self.fps = obj.fps
                
                
        # Quit Drawing if too early
        if (time.time() > self.nextTime):                     
            # Update Matplotlib Plot
            self.canvas.draw()
            self.canvas.Refresh()                 
                 
            self.Refresh()
            self.Update()
            
            # Calculate next frame time
            if (self.fps > 0):
                fpsTime = 1/self.fps
                self.nextTime = fpsTime + self.loopStartTime
            else:
                self.nextTime = time.time()
                
    def on_KeyPress(self,event):
        '''To adjust the distance between pitch markers.'''
        if event.GetKeyCode() == wx.WXK_UP:
            self.dist10deg += 0.1
            print('Dist per 10 deg: %.1f' % self.dist10deg)
        elif event.GetKeyCode() == wx.WXK_DOWN:
            self.dist10deg -= 0.1
            if self.dist10deg <= 0:
                self.dist10deg = 0.1
            print('Dist per 10 deg: %.1f' % self.dist10deg)
        # Toggle Widgets
        elif event.GetKeyCode() == 49: # 1
            widgets = [self.modeText,self.wpText]
            self.toggleWidgets(widgets)  
        elif event.GetKeyCode() == 50: # 2
            widgets = [self.batOutRec,self.batInRec,self.voltsText,self.ampsText,self.batPerText]
            self.toggleWidgets(widgets)     
        elif event.GetKeyCode() == 51: # 3
            widgets = [self.rollText,self.pitchText,self.yawText]
            self.toggleWidgets(widgets)                 
        elif event.GetKeyCode() == 52: # 4
            widgets = [self.airspeedText,self.altitudeText,self.climbRateText]
            self.toggleWidgets(widgets)
        elif event.GetKeyCode() == 53: # 5
            widgets = [self.altHistRect,self.altPlot,self.altMarker,self.altText2]
            self.toggleWidgets(widgets)
        elif event.GetKeyCode() == 54: # 6
            widgets = [self.headingTri,self.headingText,self.headingNorthTri,self.headingNorthText,self.headingWPTri,self.headingWPText]
            self.toggleWidgets(widgets)
            
        # Update Matplotlib Plot
        self.canvas.draw()
        self.canvas.Refresh()                       
        
        self.Refresh()
        self.Update()
