#!/usr/bin/env python
'''
mavgui - a Qt GUI for MAVLink

operates by using both the mavproxy netconsole plugin, and the --out parameter
eg: 
python mavproxy.py --out=127.0.0.1:6678  xxxxx whatever else you want here. xxxx 
MAV> module load netconsole


Copyright David Bussenschutt 2012
Released under the GNU GPL version 3 or later

'''
import sys
from PyQt4 import QtCore, QtGui, uic , QtNetwork
from PyQt4.QtCore import QSocketNotifier, SIGNAL
from PyQt4.QtGui import QApplication, QMainWindow
from BuzzGraphicsView import BuzzGraphicsView

from functools import partial


# this is a work-in-progress, so please dont judge me on the code, or the weird comments or reminders etc


#from mavgui3 import Ui_BuzzMainWindow  # old way is to pull in raw output from mavgui3.ui/mavgui3.py to inherit from
# or new way ( below) is to load the .ui on the fly with uic module... as we do below
# hints:
# http://stackoverflow.com/questions/2398800/linking-a-qtdesigner-ui-file-to-python-pyqt
#http://www.riverbankcomputing.co.uk/static/Docs/PyQt4/html/designer.html



import socket,errno, time
import sys, os, select, re

# find the mavlink.py module
for d in [ 'pymavlink',
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'pymavlink'),
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..','..', 'pymavlink'),
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'mavlink', 'pymavlink'),
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'mavlink', 'pymavlink') ]:
    if os.path.exists(d):
        sys.path.insert(0, d)
        if os.name == 'nt':
            try:
                # broken python compilation of mavlink.py on windows!
                os.unlink(os.path.join(d, 'mavlink.pyc'))
            except:
                pass

# add modules path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules', 'lib'))


import mavutil, mavwp




class OurMainWindow(QMainWindow): #, Ui_BuzzMainWindow):  # multiple inherit , works fine, alternate way.


    # convert a number from an arbitrary min-max range INTO a 1000-2000 PWM /RC range
    # "servo" must be between min and max.
    def scale_rc(self,servo, min, max):
        '''scale a PWM value'''
        # default to servo range of 1000 to 2000
        min_pwm  = 1000
        max_pwm  = 2000
        
        # percentage of original throw
        p = (servo-min) / float(max-min_pwm)
        
        v = min_pwm + p*(max_pwm-min_pwm)
        
        if v < min_pwm:
            v = min_pwm
        if v > max_pwm:
            v = max_pwm
            
        print "scaled version: %d" % v    
            
        return v


    def constant(self):
		"""
		Start the timer/s
		"""
		self.ctimer.start(0)   # as often as we can! 
		self.ptimer.start(100) 

    def constantUpdate(self):
		"""
		slot for constant timer timeout
		"""
		#print "many hz"
		self.check_networks()
		
    def UDPUpdate(self):
		"""
		slot for UDPUpdate timer timeout
		"""
		#print "one hz"
		#self.oneHertzUpdate()
		pass
		
    def __init__(self):
          QMainWindow.__init__(self)
        
          self._ail = 1500   # range from 1000 -> 2000
          self._ele = 1500
          self._thr = 1500
          self._rud = 1500
          self.sock = None;
      
          self.consoletimeout = time.time()   
    
          # Set up the user interface from Designer, including attaching signals and the like
          uic.loadUi('mavgui.ui', self)

#         self.setupUi(self) use this if doing multiple inheritance from Ui_BuzzMainWindow in mavgui3.py
          
          QtCore.QObject.connect(self.LeftGraphicsView,QtCore.SIGNAL("moved"),self.leftstuff )
          QtCore.QObject.connect(self.RightGraphicsView,QtCore.SIGNAL("moved"),self.rightstuff )
          
          self.LeftGraphicsView.setMaximumSize(180,180)
          self.RightGraphicsView.setMaximumSize(180,180)
          
          self.Lscene = QtGui.QGraphicsScene(self)
          self.Rscene = QtGui.QGraphicsScene(self)
          
          #Lscene.setSceneRect(0,0,600,400)
          #Rscene.setSceneRect(0,0,600,400)
          
          self.LeftGraphicsView.setScene(self.Lscene)
          self.RightGraphicsView.setScene(self.Rscene)
         
          self.Litem = QtGui.QGraphicsEllipseItem(-20, -20, 10, 10) # x, y, w, h
          self.Ritem = QtGui.QGraphicsEllipseItem(-20, -20, 10, 10) # x, y, w, h
          
          #Litem.setPos(90,90)
          #Ritem.setPos(90,90)

           # this changes it from click-n-move events only to any mouse-over events at all
          #self.Litem.setAcceptHoverEvents(1);
          #self.Ritem.setAcceptHoverEvents(1);
          
          
          # MAV data coming over UDP from MAVProxy or similar. 
          self.simudp = False
          

          self.ctimer = QtCore.QTimer()
          self.ptimer = QtCore.QTimer()
		
		  # constant timer
          QtCore.QObject.connect(self.ctimer, QtCore.SIGNAL("timeout()"), self.constantUpdate)
		  # per-second timer
          QtCore.QObject.connect(self.ptimer, QtCore.SIGNAL("timeout()"), self.UDPUpdate)
          
          
          QtCore.QMetaObject.connectSlotsByName(self)
          
          
          self.constant();

      
          self.Lscene.addItem(self.Litem)
          self.Rscene.addItem(self.Ritem)
          
          # create a 1Hz timer, and connect it to the function we want to trigger
          #self.animator=QtCore.QTimer()
          #self.animator.timeout.connect(self.animate)
          #self.animate()  # do it once by hand, just to start with 
         
	  self.listen_udp() 

#    def readalldata(fd):
#	print "readalldata! woot"
#	try:
#	    self.m.recv();
#	except Exception:
#	    return
#	if m.first_byte: 
#		m.auto_mavlink_version(s)
#	msgs = m.mav.parse_buffer(s)
#	if msgs:
#          for msg in msgs:
#            if getattr(m, '_timestamp', None) is None:
#                m.post_message(msg)
#            if msg.get_type() == "BAD_DATA":
#                #if opts.show_errors:
#                #    mpstate.console.writeln("MAV error: %s" % msg)
#                #mpstate.status.mav_error += 1
#		pass
#       #bufferSize = 1024
#        #while True:
#        #        data = os.read(fd, bufferSize)
#        #        if not data:
#        #                break
#        #        print 'data read:'
#
    def master_callback(self,m):
    	'''process mavlink message m on master, sending any messages to recipients'''
    
    	mtype = m.get_type()
    
    	print "mavi packet type: %s" % mtype
    	
    	if mtype == 'HEARTBEAT':
    		#mpstate.status.last_heartbeat = time.time()
    		#master.last_heartbeat = mpstate.status.last_heartbeat
    		pass
    
    	elif mtype == 'STATUSTEXT':
    		#mpstate.status.last_apm_msg = m.text
    		pass
    	
    	elif mtype == 'PARAM_VALUE':
    		# x = = m.param_value
    		pass
    	
    	elif mtype == 'SERVO_OUTPUT_RAW':
     		#x= m.servo1_raw m.servo2_yaw m.servo3_yaw m.servo4_yaw -- etc
    		pass
    	
    	elif mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
    		# m.count
    		pass
    
    	elif mtype in ['WAYPOINT', 'MISSION_ITEM']:
    		# x = m.seq 
    		pass
    
    	elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
    		#  blah with m
    		pass
    
    	elif mtype == "SYS_STATUS":
    		# get battery info from m
    		# m.voltage_battery
    		# m.current_battery
    		# m.
    		self.Vcc = m.voltage_battery
    		pass
    
    	elif mtype == "VFR_HUD":
    		# display alt from m.alt, and prolly other stuff
    		# m.airspeed
    		# m.groundspeed
    		# m.heading
    		# m.throttle
    		# m.alt
    		# m.climb
    		self.update_one_text_field(self.Alt,m.alt)
 
    		self.update_one_text_field(self.AirSpeed,m.airspeed)
    		self.update_one_text_field(self.GPSSpeed, m.groundspeed)
    		self.update_one_text_field(self.Heading, m.heading)
    		
    		print "%d" % m.throttle
    		#self.update_one_data_field(self.LverticalSlider, self.scale_rc(m.throttle,0,100) )
    		pass
    
    	elif mtype == "GPS_RAW":
    		m.fix_type != 2  # GPS fix lost
    		m.fix_type == 2  # GPS OK
    		pass
    
    	elif mtype == "GPS_RAW_INT":
    		m.fix_type != 3  # GPS fix lost
    		m.fix_type == 3  # GPS OK		
    		pass
    	
    	elif mtype == "RC_CHANNELS_RAW":
    		# x = m.chan7_raw #	 and other channels!  etc
    		pass
    
    	elif mtype == "NAV_CONTROLLER_OUTPUT":
    		# x = m.wp_dist
    		pass
    	
    	elif mtype == "FENCE_STATUS":
    		# x = m.breach_time
    		# y = m.breach_status
    		# breach_type
    		# if m.breach_status == mavutil.mavlink.FENCE_BREACH_NONE: # Fence OK! 
    		pass
    	
    	elif mtype == "SCALED_PRESSURE":
    	  
    	  #ground_press = get_mav_param('GND_ABS_PRESS', None)
    	  #ground_temperature = get_mav_param('GND_TEMP', None)
    	 # if ground_press is not None and ground_temperature is not None:
           	#         altitude = None
           	#         try:
           	 #            scaling = ground_press / (m.press_abs*100)
           	 #            temp = ground_temperature + 273.15
           	 #            altitude = math.log(scaling) * temp * 29271.267 * 0.001
           	 #        except ValueError:
           	 #            pass
           	 #        except ZeroDivisionError:
           	 #            pass
           	 #	     if altitude is not None:
          	 #     	         # blah    report_altitude(altitude)
    		         pass
    
    	elif mtype == "BAD_DATA":
    		# blah m.data
    		pass
    
    	elif mtype in [ 'HEARTBEAT', 'GLOBAL_POSITION', 'RC_CHANNELS_SCALED',
                        'ATTITUDE', 'RC_CHANNELS_RAW', 'GPS_STATUS', 'WAYPOINT_CURRENT',
                        'SERVO_OUTPUT_RAW', 'VFR_HUD',
                        'GLOBAL_POSITION_INT', 'RAW_PRESSURE', 'RAW_IMU',
                        'WAYPOINT_ACK', 'MISSION_ACK',
                        'NAV_CONTROLLER_OUTPUT', 'GPS_RAW', 'GPS_RAW_INT', 'WAYPOINT',
                        'SCALED_PRESSURE', 'SENSOR_OFFSETS', 'MEMINFO', 'AP_ADC',
                        'FENCE_POINT', 'FENCE_STATUS', 'DCM', 'RADIO', 'AHRS', 'HWSTATUS', 'SIMSTATE', 'PPP' ]:
    		# blah do print pack type mtype ignored deliberately!
           	 	pass
    
    	else:
            	print "Got MAVLink msg: %s" % m
   
    def update_one_text_field(self, _field, _contents):
        _field.setText(QtGui.QApplication.translate("MainWindow", "%s" % _contents, None, QtGui.QApplication.UnicodeUTF8)) 

    def update_one_data_field(self, _field, _contents):
        _field.setValue(_contents) 
        


    def read_all_q(self):
	while self.udpSocket.hasPendingDatagrams():
            datagram = QtCore.QByteArray()
            datagram.resize(self.udpSocket.pendingDatagramSize())
            sender = QtNetwork.QHostAddress()
            data   = self.udpSocket.readDatagram(datagram.size())
            ip     = str(data[1].toString())
            msg    = str(data[0])

	    s = msg

	    #print "data: ip: %s  msg: %s " % ( ip, msg ) 
	    self.m.pre_message()
	    if len(s) == 0:
              return None
	    if self.m.first_byte:
		self.m.auto_mavlink_version(s)
	    msg = self.m.mav.parse_buffer(s)
	    if msg is not None:
            	for m in msg:
               	 #self.m.post_message(m)
               	 self.master_callback(m)
                 self.simudp = time.time()
		 #
            	#return msg[0]
	    #return None


    def check_networks(self):

        if ( time.time() - self.simudp > 1 ):
            self.simudp = False

        if ( self.simudp != False ): 
            self.infobar.setText(QtGui.QApplication.translate("MainWindow", "OK! UDP data from MAVProxy on port 6678...", None, QtGui.QApplication.UnicodeUTF8))
        else:
            self.infobar.setText(QtGui.QApplication.translate("MainWindow", "Waiting for UDP data from MAVProxy on port 6678...", None, QtGui.QApplication.UnicodeUTF8))

          
    def listen_udp(self):
	print "listening udp!"
	# for now, we'll just accept a UDP stream: 
	self.m = mavutil.mavudp('localhost:6666', input=True);
	self.m.close() # don't listen for stuff yourself, but we'll pass it in later from Qt
	#self.m.mav.set_callback(master_callback, self.m)
	self.m.linknum = 1
	self.m.linkerror = False
	self.m.link_delayed = False
	self.m.last_heartbeat = 0
	self.m.highest_usec = 0

	# Qt method: 
	self.udpSocket = QtNetwork.QUdpSocket(self)
	self.udpSocket.bind(QtNetwork.QHostAddress(QtNetwork.QHostAddress.Any),6678); 
	self.connect(self.udpSocket, QtCore.SIGNAL("readyRead()"),self.read_all_q)
	

	# emits activate() signal on any socket event
	#self.unotifier =  QSocketNotifier(self.m.port, QSocketNotifier.Read)
        #QtCore.QObject.connect(self.unotifier,QtCore.SIGNAL("activated(int)"),self.readalldata )
	
        #status_period = mavutil.periodic_event(1.0)
	#msg_period = mavutil.periodic_event(1.0/15)
 
	#mpstate.mav_master.append(m)

 
    def leftstuff(self, point): # we are passed a QPoint here!
           #point.x and point.y vary from 0-180, so we scale them from 1000->2000
           x = float(point.x())
           x = x/180*1000
           x +=1000 # centre x around 1500, low value is 1000, at the left
           y = float(point.y())
           y = y/180*1000
           y= 2000-y  # make low value be at bottom, not top.
           # constrain it to 1000-2000, no matter! 
           if x < 1000: x = 1000
           if x > 2000: x = 2000
           if y < 1000: y = 1000
           if y > 2000: y = 2000       
           self.rud(x)
           self.thr(y)
           #print "L: %d %d" % ( x ,y )
           self.LhorizontalSlider.setValue(x)
           self.LverticalSlider.setValue(y)
           pass 
           
    def rightstuff(self, point): # we are passed a QPoint here!
           #point.x and point.y vary from 0-180, so we scale them from 1000->2000
           x = float(point.x())
           x = x/180*1000
           x +=1000 # centre x around 1500, low value is 1000, at the left
           y = float(point.y())
           y = y/180*1000
           y= 2000-y  # make low value be at bottom, not top.
           # constrain it to 1000-2000, no matter! 
           if x < 1000: x = 1000
           if x > 2000: x = 2000
           if y < 1000: y = 1000
           if y > 2000: y = 2000       
           self.ail(x)
           self.ele(2000-(y-1000))
           #print "R: %d %d" % ( x ,y )
           self.RhorizontalSlider.setValue(x)
           self.RverticalSlider.setValue(2000-(y-1000))
           pass 


    def bind(self):
      # Create a TCP/IP socket
      if self.sock is None:
        try:
          self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
          self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
          self.sock.setblocking(0) # non-blocking mode
        except socket.error, msg:
          self.sock = None
          print msg
            
    def tcp_connect( self): 
           # Connect the socket to the port where the server is listening
          self.server_address = ('localhost', 45678)
          print >>sys.stderr, 'connecting to %s port %s' % self.server_address
          
        # This is kinda ugly, but a non-blocking connect() returns
        # errno 36 ('Operation now in progress') and we don't want to
        # catch that as an error.  So we let select() find a trashed
        # socket if it wants.    
          try:
            self.sock.connect(self.server_address)
          except socket.error, msg:
             if msg[0] == errno.EINPROGRESS:
                # 'Operation in Progress' is OK
                pass
             else :
              self.sock.close()
              self.sock = None
              print msg
              
   	       # wait for the connect to succeed, or time out in 2s
          try:
   	       (sread, swrite, serror) = select.select( [], [self.sock], [], 2 )
          except select.error:
   	      # there was a problem ...
             print "Select error %s: %s" (select.error)
             self.valid = None
             return
   
       	  if swrite.count(self.sock) > 0 :
       	  # Check the socket for an error
       	  # we got the connection...
       	    i = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
       	    if 0 == i:
                   self.valid = 1
       	    self.sock.setblocking(1)
            return
       
       	    # otherwise, we hit the timeout
            self.valid = None


    def send(self, message):
    
        # TODO before we send, we should read any pending incoming data, and throw it away, yes? 
 
        # Send data
        #message = 'auto'
        print >>sys.stderr, 'sending "%s"' % message
     #   self.tcp_connect()
     
        if self.sock is None:
            self.bind()
            self.tcp_connect()
        if self.sock is None:
            print 'could not open socket'
            
        success = None
        try:
            success = self.sock.sendall(message)  # returns 'None' on success, exception on error
        except socket.error as e:     
            if e.errno == errno.EPIPE:  # EPIPE is remote disconnect, we are OK with this, we just close socket, assuming we are done. 
                self.sock.close()
                self.sock = None
            if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK]:  
                raise

        # Look for the response in at most the next 2 seconds worth of output!   
    def expect(self, find, timeout=2):  
        amount_received = 0
        amount_expected = 2  # the "ok" sring.
      
        data = True;
        block = "";
        ret = False;        
                
        # more than 2 secs since last update of network traffic?  
        self.consoletimeout = time.time()   
        
        
        sread = swrite =serror = None    
    
        # check for timeout
        if sread == []: 
            print >>sys.stderr, 'timeout, no match found'
            self.consoletimeout = False
            return False

        while ( time.time() - self.consoletimeout < timeout  ): 
          # non-blocking read loop, does no giveup on exceptions, only on successful zero-byte reads. 
          try:
            # poll, not wait - timeout of zero means non-blocking poll
            (sread, swrite, serror) = select.select( [self.sock], [], [], 0 )
            if sread != []:
                data = self.sock.recv(1024)
                if not data: break
                print >>sys.stderr, 'received "%s"' % data
                amount_received += len(data)
                m = re.search(find, data, re.MULTILINE)
                block += data;
                if m != None: # match found? 
                    print >>sys.stderr, 'match found'
                    break
                    ret = True;
          except socket.error as e:
            if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                raise
            print >>sys.stderr, 'nothing received yet'

        if ( time.time() - self.consoletimeout > timeout ):
                print >>sys.stderr, 'timeout, no match found'
                self.consoletimeout = False

#   if remote abort, timeout, or other exception, then try match on entire block. 
        m = re.search(find, block, re.MULTILINE)
        if m != None: # match found? 
            ret = True;
               
                
        print >>sys.stderr, 'received "%s"' % block
       
        self.BigText.setPlainText(QtGui.QApplication.translate("MainWindow", block, None, QtGui.QApplication.UnicodeUTF8))

        # self.close();
        return ret

    def close(self):
     if self.sock != None:
        self.sock.close()
     self.sock = None
        
    def reconnect(self):
        self.close()
        self.bind()
        self.tcp_connect()
          
    def auto(self):
        self.send("auto\n\n")
        self.expect("Got MAVLink msg");
        pass
        
    def leftrc(self):
       # self.send("auto\n\n")
       # self.expect("Got MAVLink msg");
        print("leftrc")   
        pass    
        
    def pwm_direction(self,direction):
        self.pwm_direction = direction
        #self.send("auto\n\n")
        #self.expect("Got MAVLink msg");
        pass
        
    def manual(self):
        self.send("manual\n\n")
        self.expect("Got MAVLink msg");
        pass

    def rtl(self):
        self.send("rtl\n\n")
        self.expect("Got MAVLink msg");
        pass
        
    def stabilise(self):  #mode "STABILIZE" 
        self.send("rc 8 1400\n\n")
        self.expect("");
        pass
        
    def circle(self):
        self.send("loiter\n\n")
        self.expect("Got MAVLink msg");
        pass
        
    def guided(self):
        self.send("guided\n\n")
        self.expect("Got MAVLink msg");
        pass        

    def exitnow(self):
        self.close()
        sys.exit(0)
        pass 
        
    def airhold(self):
        self.airhold_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))
        
        self.send("auto")
        self.expect("Got MAVLink msg");
        pass
    
    def comshold(self):
        self.comhold_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))
        self.send("manual")
        self.expect("Got MAVLink msg");
        pass
    
    def miss1(self):
        self.miss1_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))   
        self.send("help")
        self.expect("waypoint management");
        pass
    
    def miss2(self):
        self.miss2_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))
        pass
    

    def miss3(self):
        self.miss3_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))
        pass
    

    def bottleaway(self):
        self.bottledrop_out.setText(QtGui.QApplication.translate("MainWindow", "woot!", None, QtGui.QApplication.UnicodeUTF8))  
        pass
    

    def chuteaway(self):
        self.chutedrop_out.setText(QtGui.QApplication.translate("MainWindow", "woot!.", None, QtGui.QApplication.UnicodeUTF8))
        self.close()
        pass
    

    def rud(self,val): # channel 4
        if ( self._rud != val):
            self._rud = val
            self.send("rc 4 %d" % val) 

    def thr(self,val): # channel 3 
        if ( self._thr != val):
            self._thr = val
            self.send("rc 3 %d" % val) 

    def ail(self,val): # channel 1 
        if ( self._ail != val):
            self._ail = val
            self.send("rc 1 %d" % val) 

    def ele(self,val): # channel 2
        if ( self._ele != val):
            self._ele = val
            self.send("rc 2 %d" % val) 
   

app = QApplication(sys.argv)
window = OurMainWindow()

window.show()
sys.exit(app.exec_())
