#!/usr/bin/env python
'''waypoint command handling'''

import time, os, fnmatch
from pymavlink import mavutil, mavwp

class wp_state(object):
    def __init__(self):
        self.wp_op = None
        self.wp_save_filename = None
        self.wploader = mavwp.MAVWPLoader()
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.last_waypoint = 0
        self.wp_period = mavutil.periodic_event(0.5)

def name():
    '''return module name'''
    return "wp"

def description():
    '''return module description'''
    return "waypoint handling"

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.wp_state = wp_state()
    state = mpstate.wp_state
    mpstate.command_map['wp'] = (cmd_wp,       'waypoint management')
    mpstate.completions['wp'] = ["<list|clear>",
                                 "<load|update|save> (FILENAME)"]
    if mpstate.continue_mode and mpstate.status.logdir != None:
        waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
        if os.path.exists(waytxt):
            state.wploader.load(waytxt)
            print("Loaded waypoints from %s" % waytxt)


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.wp_state
    mtype = m.get_type()
    if mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
        if state.wp_op is None:
            mpstate.console.error("No waypoint load started")
        else:
            state.wploader.clear()
            state.wploader.expected_count = m.count
            mpstate.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                             time.asctime(time.localtime(m._timestamp)),
                                                                             time.asctime()))
            mpstate.master().waypoint_request_send(0)

    elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and state.wp_op != None:
        if m.seq > state.wploader.count():
            mpstate.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, state.wploader.count()))
        elif m.seq < state.wploader.count():
            # a duplicate
            pass
        else:
            state.wploader.add(m)
        if m.seq+1 < state.wploader.expected_count:
            mpstate.master().waypoint_request_send(m.seq+1)
        else:
            if state.wp_op == 'list':
                for i in range(state.wploader.count()):
                    w = state.wploader.wp(i)
                    print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                        w.command, w.frame, w.x, w.y, w.z,
                        w.param1, w.param2, w.param3, w.param4,
                        w.current, w.autocontinue))
                if mpstate.status.logdir != None:
                    waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
                    save_waypoints(waytxt)
                    print("Saved waypoints to %s" % waytxt)
            elif state.wp_op == "save":
                save_waypoints(state.wp_save_filename)
            state.wp_op = None

    elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
        process_waypoint_request(m, mpstate.master())

    elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
        if m.seq != state.last_waypoint:
            state.last_waypoint = m.seq
            mpstate.functions.say("waypoint %u" % m.seq,priority='message')



def idle_task():
    '''handle missing waypoints'''
    state = mpstate.wp_state
    if state.wp_period.trigger():
        # cope with packet loss fetching mission
        if mpstate.master().time_since('MISSION_ITEM') >= 2 and state.wploader.count() < getattr(state.wploader,'expected_count',0):
            seq = state.wploader.count()
            print("re-requesting WP %u" % seq)
            mpstate.master().waypoint_request_send(seq)

def process_waypoint_request(m, master):
    '''process a waypoint request from the master'''
    state = mpstate.wp_state
    if (not state.loading_waypoints or
        time.time() > state.loading_waypoint_lasttime + 10.0):
        state.loading_waypoints = False
        mpstate.console.error("not loading waypoints")
        return
    if m.seq >= state.wploader.count():
        mpstate.console.error("Request for bad waypoint %u (max %u)" % (m.seq, state.wploader.count()))
        return
    wp = state.wploader.wp(m.seq)
    wp.target_system = mpstate.status.target_system
    wp.target_component = mpstate.status.target_component
    mpstate.master().mav.send(state.wploader.wp(m.seq))
    state.loading_waypoint_lasttime = time.time()
    mpstate.console.writeln("Sent waypoint %u : %s" % (m.seq, state.wploader.wp(m.seq)))
    if m.seq == state.wploader.count() - 1:
        state.loading_waypoints = False
        mpstate.console.writeln("Sent all %u waypoints" % state.wploader.count())

def load_waypoints(filename):
    '''load waypoints from a file'''
    state = mpstate.wp_state
    state.wploader.target_system = mpstate.status.target_system
    state.wploader.target_component = mpstate.status.target_component
    try:
        state.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u waypoints from %s" % (state.wploader.count(), filename))

    mpstate.master().waypoint_clear_all_send()
    if state.wploader.count() == 0:
        return

    state.loading_waypoints = True
    state.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(state.wploader.count())

def update_waypoints(filename, wpnum):
    '''update waypoints from a file'''
    state = mpstate.wp_state
    state.wploader.target_system = mpstate.status.target_system
    state.wploader.target_component = mpstate.status.target_component
    try:
        state.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    if state.wploader.count() == 0:
        print("No waypoints found in %s" % filename)
        return
    if wpnum == -1:
        print("Loaded %u updated waypoints from %s" % (state.wploader.count(), filename))
    elif wpnum >= state.wploader.count():
        print("Invalid waypoint number %u" % wpnum)
        return
    else:
        print("Loaded updated waypoint %u from %s" % (wpnum, filename))

    state.loading_waypoints = True
    state.loading_waypoint_lasttime = time.time()
    if wpnum == -1:
        start = 0
        end = state.wploader.count()-1
    else:
        start = wpnum
        end = wpnum
    mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         start, end)

def save_waypoints(filename):
    '''save waypoints to a file'''
    state = mpstate.wp_state
    try:
        state.wploader.save(filename)
    except Exception, msg:
        print("Failed to save %s - %s" % (filename, msg))
        return
    print("Saved %u waypoints to %s" % (state.wploader.count(), filename))

def wp_draw_callback(points):
    '''callback from drawing waypoints'''
    state = mpstate.wp_state
    if len(points) < 3:
        return
    from MAVProxy.modules.lib import mp_util
    home = state.wploader.wp(0)
    state.wploader.clear()
    state.wploader.target_system = mpstate.status.target_system
    state.wploader.target_component = mpstate.status.target_component
    state.wploader.add(home)
    for p in points:
        state.wploader.add_latlonalt(p[0], p[1], mpstate.settings.wpalt)
    mpstate.master().waypoint_clear_all_send()
    if state.wploader.count() == 0:
        return
    state.loading_waypoints = True
    state.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(state.wploader.count())

def wp_loop():
    '''close the loop on a mission'''
    state = mpstate.wp_state
    loader = state.wploader
    if loader.count() < 2:
        print("Not enough waypoints (%u)" % loader.count())
        return
    wp = loader.wp(loader.count()-2)
    if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
        print("Mission is already looped")
        return
    wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_DO_JUMP,
                                                      0, 1, 1, -1, 0, 0, 0, 0, 0)
    loader.add(wp)
    loader.add(loader.wp(1))
    state.loading_waypoints = True
    state.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(state.wploader.count())
    print("Closed loop on mission")

def set_home_location():
    '''set home location from last map click'''
    state = mpstate.wp_state
    try:
        latlon = mpstate.map_state.click_position
    except Exception:
        print("No map available")
        return
    lat = float(latlon[0])
    lon = float(latlon[1])
    if state.wploader.count() == 0:
        state.wploader.add_latlonalt(lat, lon, 0)
    w = state.wploader.wp(0)
    w.x = lat
    w.y = lon
    state.wploader.set(w, 0)
    state.loading_waypoints = True
    state.loading_waypoint_lasttime = time.time()
    mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         0, 0)
    

def cmd_wp(args):
    '''waypoint commands'''
    state = mpstate.wp_state
    if len(args) < 1:
        print("usage: wp <list|load|update|save|set|clear|loop>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: wp load <filename>")
            return
        load_waypoints(args[1])
    elif args[0] == "update":
        if len(args) < 2:
            print("usage: wp update <filename> <wpnum>")
            return
        if len(args) == 3:
            wpnum = int(args[2])
        else:
            wpnum = -1
        update_waypoints(args[1], wpnum)
    elif args[0] == "list":
        state.wp_op = "list"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: wp save <filename>")
            return
        state.wp_save_filename = args[1]
        state.wp_op = "save"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "savelocal":
        if len(args) != 2:
            print("usage: wp savelocal <filename>")
            return
        state.wploader.save(args[1])
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: wp show <filename>")
            return
        state.wploader.load(args[1])
    elif args[0] == "set":
        if len(args) != 2:
            print("usage: wp set <wpindex>")
            return
        mpstate.master().waypoint_set_current_send(int(args[1]))
    elif args[0] == "clear":
        mpstate.master().waypoint_clear_all_send()
    elif args[0] == "draw":
        if not 'draw_lines' in mpstate.map_functions:
            print("No map drawing available")
            return        
        if state.wploader.count() == 0:
            print("Need home location - refresh waypoints")
            return
        if len(args) > 1:
            mpstate.settings.wpalt = int(args[1])
        mpstate.map_functions['draw_lines'](wp_draw_callback)
        print("Drawing waypoints on map at altitude %d" % mpstate.settings.wpalt)
    elif args[0] == "sethome":
        set_home_location()        
    elif args[0] == "loop":
        wp_loop()        
    else:
        print("Usage: wp <list|load|save|set|show|clear|draw|loop>")


