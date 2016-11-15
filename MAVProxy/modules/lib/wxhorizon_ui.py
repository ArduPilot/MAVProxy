import time
from wxhorizon_util import Attitude
from wx_loader import wx
import math

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


    def initData(self):
        # Initialise Attitude
        self.pitch = 0  # Degrees
        self.roll = 0   # Degrees
        self.yaw = 0    # Degrees
        
        # History Values
        self.oldRoll = 0 # Degrees
    

    def initUI(self):
        # Create Event Timer and Bindings
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_CHAR_HOOK,self.on_KeyPress)

        # Create Panel
        self.panel = wx.Panel(self)
        
        # Create Matplotlib Panel
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

        # Fix Axes - vertical is of length 2, horizontal keeps the same lengthscale
        self.rescaleX()
        
        # Sky Polygon
        vertsTop = [[-1,0],[-1,1],[1,1],[1,0],[-1,0]]
        self.topPolygon = Polygon(vertsTop,facecolor='cyan',edgecolor='none')
        self.axes.add_patch(self.topPolygon)
        # Ground Polygon
        vertsBot = [[-1,0],[-1,-1],[1,-1],[1,0],[-1,0]]
        self.botPolygon = Polygon(vertsBot,facecolor='brown',edgecolor='none')
        self.axes.add_patch(self.botPolygon)
        
        # Markers
        self.axes.plot([-1,-1,1,1],[-1,1,1,-1],'ro')
        
        # Center Pointer Marker
        self.thick = 0.015
        self.axes.add_patch(patches.Rectangle((-0.75,-self.thick),0.5,2.0*self.thick,facecolor='orange',zorder=3))
        self.axes.add_patch(patches.Rectangle((0.25,-self.thick),0.5,2.0*self.thick,facecolor='orange',zorder=3))
        self.axes.add_patch(patches.Circle((0,0),radius=self.thick,facecolor='orange',edgecolor='none',zorder=3))
        
        # Pitch Markers
        self.dist10deg = 0.2 # Graph distance per 10 deg
        self.createPitchmMarkers()
        
        # Add Roll, Pitch, Yaw Text
        self.vertSize = 0.09
        ypx = self.figure.get_size_inches()[1]*self.figure.dpi
        self.fontSize = self.vertSize*(ypx/2.0)
        leftPos = self.axes.get_xlim()[0]
        self.rollText = self.axes.text(leftPos+(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0),'Roll:   %.2f' % self.roll,color='w',size=self.fontSize)
        self.pitchText = self.axes.text(leftPos+(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0),'Pitch: %.2f' % self.pitch,color='w',size=self.fontSize)
        self.yawText = self.axes.text(leftPos+(self.vertSize/10.0),-0.97,'Yaw:   %.2f' % self.yaw,color='w',size=self.fontSize)
        self.rollText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.pitchText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        self.yawText.set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
        
        # Show Frame
        self.Show(True)
        self.pending = []
        
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
        
    def createPitchmMarkers(self):
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
        ypx = self.figure.get_size_inches()[1]*self.figure.dpi
        self.fontSize = self.vertSize*(ypx/2.0)
        self.pitchLabelsLeft = []
        self.pitchLabelsRight = []
        i=0
        for j in [-90,-60,-30,30,60,90]:
            self.pitchLabelsLeft.append(self.axes.text(-0.55,(j/10.0)*self.dist10deg,str(j),color='w',size=self.fontSize,horizontalalignment='center',verticalalignment='center'))
            self.pitchLabelsLeft[i].set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
            self.pitchLabelsRight.append(self.axes.text(0.55,(j/10.0)*self.dist10deg,str(j),color='w',size=self.fontSize,horizontalalignment='center',verticalalignment='center'))
            self.pitchLabelsRight[i].set_path_effects([PathEffects.withStroke(linewidth=1,foreground='k')])
            i += 1
        
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
        
    def rescaleX(self):
        '''Rescales the horizontal axes to make the lengthscales equal.'''
        self.ratio = self.figure.get_size_inches()[0]/float(self.figure.get_size_inches()[1])
        self.axes.set_xlim(-self.ratio,self.ratio)
        self.axes.set_ylim(-1,1)
        
    def calcHorizonPoints(self):
        '''Updates the verticies of the patches for the ground and sky.'''
        self.ratio = self.figure.get_size_inches()[0]/float(self.figure.get_size_inches()[1])
        ydiff = math.tan(math.radians(-self.roll))*float(self.ratio)
        pitchdiff = self.dist10deg*(self.pitch/10.0)
        # Sky Polygon
        vertsTop = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,1),(self.ratio,1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]       
        self.topPolygon.set_xy(vertsTop)
        # Ground Polygon
        vertsBot = [(-self.ratio,ydiff-pitchdiff),(-self.ratio,-1),(self.ratio,-1),(self.ratio,-ydiff-pitchdiff),(-self.ratio,ydiff-pitchdiff)]       
        self.botPolygon.set_xy(vertsBot)       
        
    def updateAttitudeText(self):
        'Updates the displayed attitude Text'
        self.rollText.set_text('Roll:   %.2f' % self.roll)
        self.pitchText.set_text('Pitch: %.2f' % self.pitch)
        self.yawText.set_text('Yaw:   %.2f' % self.yaw)

    def updateRPYLocations(self):
        '''Update the locations of rol, pitch, yaw text.'''
        leftPos = self.axes.get_xlim()[0]
        # Locations
        self.rollText.set_position((leftPos+(self.vertSize/10.0),-0.97+(2*self.vertSize)-(self.vertSize/10.0)))
        self.pitchText.set_position((leftPos+(self.vertSize/10.0),-0.97+self.vertSize-(0.5*self.vertSize/10.0)))
        self.yawText.set_position((leftPos+(self.vertSize/10.0),-0.97))
        # Font Size
        ypx = self.figure.get_size_inches()[1]*self.figure.dpi
        self.fontSize = self.vertSize*(ypx/2.0)
        self.rollText.set_size(self.fontSize)
        self.pitchText.set_size(self.fontSize)
        self.yawText.set_size(self.fontSize)

    def on_idle(self, event):
        # Fix Window Scales 
        self.rescaleX()
        
        # Recalculate Horizon Polygons
        self.calcHorizonPoints()
        
        # Update Roll, Pitch, Yaw Text Locations
        self.updateRPYLocations()
        
        # Update Pitch Markers
        self.adjustPitchmarkers()
        
        # Update Matplotlib Plot
        self.canvas.draw()
        self.canvas.Refresh()
        
        
        time.sleep(0.05)
 
    def on_timer(self, event):
        state = self.state
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return
        # Get attitude information
        while state.child_pipe_recv.poll():
            obj = state.child_pipe_recv.recv()
            if isinstance(obj,Attitude):
                self.oldRoll = self.roll
                self.pitch = obj.pitch*180/math.pi
                self.roll = obj.roll*180/math.pi
                self.yaw = obj.yaw*180/math.pi
                
                # Update Displayed Text
                self.updateAttitudeText()
                
                # Recalculate Horizon Polygons
                self.calcHorizonPoints()
                
                # Update Pitch Markers
                self.adjustPitchmarkers()
                
                # Update Matplotlib Plot
                self.canvas.draw()
                self.canvas.Refresh()
                
        self.Refresh()
        self.Update()
                
    def on_KeyPress(self,event):
        if event.GetKeyCode() == wx.WXK_UP:
            self.dist10deg += 0.1
            print 'Dist per 10 deg: %.1f' % self.dist10deg      
        elif event.GetKeyCode() == wx.WXK_DOWN:
            self.dist10deg -= 0.1
            if self.dist10deg < 0:
                self.dist10deg = 0
            print 'Dist per 10 deg: %.1f' % self.dist10deg      

