#!/usr/bin/env python
'''dump raw state information our GUI, hopefully connected to netconsole/mavgui'''

import time, math, mavutil, pickle, sys

mpstate = None

##class netstate_report(object):
#   def __init__(self):
#        self.last_report = 0
#        self.ok = True
#        self.value = 0

class netstate_state(object):
    def __init__(self):

        self.last_watch = 0
 
        self.Mode = ""
        self.GPS = ""
        self.Vcc = ""
        self.Radio = ""
        self.Heading = ""
        self.GPSHeading = ""
        self.Alt = ""
        self.GPSAlt = ""
        self.AirSpeed = ""
        self.GPSSpeed = ""
        self.Thr = ""
        self.Roll = ""
        self.Pitch = ""
        self.WP = ""
        self.Heartbeat = 0
        self.GroundAlt = 0
        
 #       self.Linkdown = 0
 #       self.linkup = 0
          
        

def name():
    '''return module name'''
    return "netstate"

def description():
    '''return module description'''
    return "monitor sensor consistancy"

# pull basic data from mavproxy state, into out local state, so we can stream it easily.  :-) 
def cmd_netstate():
    global mpstate
    #if mpstate.master().WIRE_PROTOCOL_VERSION == '1.0':
    #    gps_heading = mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01
    #else:
    #    gps_heading = mpstate.status.msgs['GPS_RAW'].hdg


    self.Heading = mpstate.status.msgs['VFR_HUD'].heading
    self.GPSHeading =gps_heading
    self.Alt = mpstate.status.altitude
    self.GPSAlt = mpstate.netstate_state.gps_alt
    self.Roll = math.degrees(mpstate.status.msgs['ATTITUDE'].roll)
    self.Pitch = math.degrees(mpstate.status.msgs['ATTITUDE'].pitch)
    self.AirSpeed = mpstate.status.msgs['VFR_HUD'].airspeed
    self.GPSSpeed = mpstate.status.msgs['VFR_HUD'].groundspeed
    self.Thr  = mpstate.status.msgs['VFR_HUD'].throttle
                     

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.netstate_state = netstate_state()
  #  mpstate.command_map['netstate'] = (cmd_netstate, "show key netstate")
    #mpstate.command_map['speed'] = (cmd_speed, "enable/disable speed report")

    if 'GPS_RAW' in mpstate.status.msgs:
        # cope with reload
         gps = mpstate.status.msgs['GPS_RAW']
         mpstate.netstate_state.GroundAlt = gps.alt - mpstate.status.altitude

    if 'GPS_RAW_INT' in mpstate.status.msgs:
        # cope with reload
        gps = mpstate.status.msgs['GPS_RAW_INT']
        mpstate.netstate_state.GroundAlt = (gps.alt/1.0e3) - mpstate.status.altitude
        
    # populate initial data from mavlink     
    #cmd_netstate()

def unload():
    global mpstate
    mpstate.netstate_state = False
    mpstate.command_map['netstate'] = False
   # mpstate.command_map['speed'] = False

def angle_diff(angle1, angle2):
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

#def report(name, ok, msg=None, deltat=20):
#    '''report a sensor error'''
#    r = mpstate.netstate_state.reports[name]
#    if time.time() < r.last_report + deltat:
#        r.ok = ok
#        return
#    r.last_report = time.time()
#    if ok and not r.ok:
#        mpstate.functions.say("%s OK" % name)
#    r.ok = ok
#    if not r.ok:
#        mpstate.functions.say(msg)

#def report_change(name, value, maxdiff=1, deltat=10):
#    '''report a sensor change'''
#    r = mpstate.netstate_state.reports[name]
#    if time.time() < r.last_report + deltat:
#        return
#    r.last_report = time.time()
#    if math.fabs(r.value - value) < maxdiff:
#        return
#    r.value = value
#    mpstate.functions.say("%s %u" % (name, value))

def check_heading(m):
    '''check heading discrepancy'''
    if 'GPS_RAW' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW']
        if gps.v < 3:
            return
        diff = math.fabs(angle_diff(m.heading, gps.hdg))
    elif 'GPS_RAW_INT' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW_INT']
        if gps.vel < 300:
            return
        diff = math.fabs(angle_diff(m.heading, gps.cog/100.0))
    else:
        return
   # report('heading', diff<20, 'heading error %u' % diff)

def check_altitude(m):
    '''check altitude discrepancy'''
    if 'GPS_RAW' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW']
        if gps.fix_type != 2:
            return
        v = gps.v
        alt = gps.alt
    elif 'GPS_RAW_INT' in mpstate.status.msgs:
        gps = mpstate.status.msgs['GPS_RAW_INT']
        if gps.fix_type != 3:
            return
        v = gps.vel/100
        alt = gps.alt/1000
    else:
        return

    if v > mpstate.netstate_state.max_speed:
        mpstate.netstate_state.max_speed = v
    if mpstate.netstate_state.max_speed < 5:
        mpstate.netstate_state.ground_alt = alt
        return
    mpstate.netstate_state.gps_alt = alt - mpstate.netstate_state.ground_alt
    diff = math.fabs(mpstate.netstate_state.gps_alt - mpstate.status.altitude)
   # report('altitude', diff<30, 'altitude error %u' % diff)

def mavlink_packet(msg):
    global mpstate

    '''handle an incoming mavlink packet'''
#    if msg.get_type() == 'VFR_HUD' and ( 'GPS_RAW' in mpstate.status.msgs or 'GPS_RAW_INT' in mpstate.status.msgs ):
#       check_heading(msg)
#       check_altitude(msg)
#        report_change('speed', m.groundspeed, maxdiff=2, deltat=2)

 #   print "blerg" 
    
    #print pickle.dump(msg,sys.stdout)
    '''    
    print "%s!%s!%s!%s!%s!%s!%s!%s!%s!%s!%s!%s!%s!%s!%d!%d!" %  ( mpstate.netstate_state.Mode,
            mpstate.netstate_state.GPS,mpstate.netstate_state.Vcc,
            mpstate.netstate_state.Radio,mpstate.netstate_state.Heading,  
            mpstate.netstate_state.GPSHeading,mpstate.netstate_state.Alt,
            mpstate.netstate_state.GPSAlt,mpstate.netstate_state.AirSpeed,     
            mpstate.netstate_state.GPSSpeed,mpstate.netstate_state.Thr,
            mpstate.netstate_state.Roll,mpstate.netstate_state.Pitch,
            mpstate.netstate_state.WP,mpstate.netstate_state.Heartbeat,
            mpstate.netstate_state.GroundAlt ) 
    '''
    # do periodic report to GUI immediatetly on loading module! 
    if time.time() > mpstate.netstate_state.last_watch+1:
        mpstate.netstate_state.last_watch = time.time()
        cmd_netstate([])
        # now blurt out all the data to the GUI: 
       # print pickle.dump(mpstate.netstate_state,sys.stdout)      
 
    master = mpstate.master()
    # add some status fields
    if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
        if ((msg.fix_type == 3 and master.mavlink10()) or
            (msg.fix_type == 2 and not master.mavlink10())):
            mpstate.netstate_state.GPS = 'OK'
        else:
            mpstate.netstate_state.GPS = '%u' % msg.fix_type
        if master.mavlink10():
            gps_heading = int(mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01)
        else:
            gps_heading = mpstate.status.msgs['GPS_RAW'].hdg
        mpstate.netstate_state.Heading = '%s/%u' % (master.field('VFR_HUD', 'heading', '-'), gps_heading)
    elif type == 'VFR_HUD':
        mpstate.netstate_state.Mode = '%s' % master.flightmode
        if master.mavlink10():
            alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
        else:
            alt = master.field('GPS_RAW', 'alt', 0)
        mpstate.netstate_state.Alt = '%u/%.0f' % (mpstate.status.altitude, alt)
        mpstate.netstate_state.AirSpeed = '%u' % msg.airspeed
        mpstate.netstate_state.GPSSpeed = '%u' % msg.groundspeed
        mpstate.netstate_state.Thr = '%u' % msg.throttle
    elif type == 'ATTITUDE':
        mpstate.netstate_state.Roll = '%u' % math.degrees(msg.roll)
        mpstate.netstate_state.Pitch = '%u' % math.degrees(msg.pitch)
    elif type == 'HWSTATUS':
 #       if msg.Vcc >= 4600 and msg.Vcc <= 5100:
 #           fg = 'green'
 #       else:
 #           fg = 'red'
        mpstate.netstate_state.Vcc = '%.2f' % (msg.Vcc * 0.001)
    elif type == 'RADIO':
  #      if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
  #          fg = 'red'
 #       else:
 #           fg = 'black'
        mpstate.netstate_state.Radio = '%u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise)
        
        
    elif type == 'HEARTBEAT':
          
        mpstate.netstate_state.Heatbeat = time.time()  # remember last good heartbeat
        
         
        
 #       for m in mpstate.mav_master:
 #           linkdelay = (mpstate.status.highest_usec - m.highest_usec)*1e-6            
 #           linkline = "Link %u " % (m.linknum+1)
 #           if m.linkerror:
 #               linkline += "down"
 #               fg = 'red'
 #           elif master.link_delayed:
 #               linkline += "delayed %.2fs" % linkdelay
 #               fg = 'yellow'
 #           else:
 #               linkline += "OK (%u pkts, %.2fs delay, %u lost)" % (m.mav_count, linkdelay, m.mav_loss)
 #               fg = 'darkgreen'
 #     #      mpstate.netstate_state.Link = %u'%m.linknum, linkline, row=1, fg=fg)
 #           mpstate.console.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
    elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
        mpstate.netstate_state.WP = '%u' % msg.seq
    elif type == 'NAV_CONTROLLER_OUTPUT':
#        mpstate.netstate_state.WPDist = 'Distance %u' % msg.wp_dist)
#        mpstate.netstate_state.WPBearing = 'Bearing %u' % msg.target_bearing)
#        mpstate.netstate_state.AltError = 'AltError %d' % msg.alt_error)
#        mpstate.netstate_state.AspdError = 'AspdError %.1f' % (msg.aspd_error*0.01))
        pass
        
        #
        #
        #
        ##
        #
        #
        #
        #
        ##
        
        