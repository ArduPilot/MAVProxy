"""
    MAVProxy rally module
"""

from pymavlink import mavwp
import time

class rally_state(object):
    def __init__(self):
        self.rallyloader = mavwp.MAVRallyLoader(mpstate.status.target_system, mpstate.status.target_component)
        return 

def name():
    '''return module name'''
    return "rally"

def description():
    '''return module description'''
    return "rally point control"

def init(_mpstate):
    '''initialize module'''
    global mpstate
    mpstate = _mpstate
    mpstate.rally_state = rally_state()
    mpstate.command_map['rally'] = (cmd_rally, "rally point control")
    mpstate.completions["rally"] = ["<add|clear|list>",
                                    "<load|save> (FILENAME)"]

def cmd_rally(args):
    '''rally point commands'''
    #TODO: add_land arg
    if(len(args) < 1):
        print_usage()
        return

    elif(args[0] == "add"):
        if (len(args) < 2):
            print("Usage: rally add ALT <BREAK_ALT> <LAND_HDG>")
            return

        if (mpstate.rally_state.rallyloader.rally_count() > 4):
            print ("Only 5 rally points possible per flight plan.")
            return

        try:
            latlon = mpstate.map_state.click_position
        except Exception:
            print("No map available")
            return
        if latlon is None:
            print("No map click position available")
            return

        alt = float(args[1]);
        break_alt = 0.0
        land_hdg = 0.0;
        if (len(args) > 2):
            break_alt = float(args[2])
        if (len(args) > 3):
            land_hdg = float(args[3])

        mpstate.rally_state.rallyloader.create_and_append_rally_point(latlon[0] * 1e7, latlon[1] * 1e7, alt, break_alt, land_hdg, 0)

        send_rally_points();

        print("Added Rally point at %s %f" % (str(latlon), alt))

    elif(args[0] == "clear"):
        mpstate.rally_state.rallyloader.clear()
        mpstate.mav_param.mavset(mpstate.master(),'RALLY_TOTAL',0,3)

    elif(args[0] == "list"):
        list_rally_points()

    elif(args[0] == "load"):
        if (len(args) < 2):
            print("Usage: rally load filename")

        try:
            mpstate.rally_state.rallyloader.load(args[1])
        except Exception, msg:
            print("Unable to load %s - %s" % (args[1], msg))
            return
    
        send_rally_points()

        print("Loaded %u rally points from %s" % (mpstate.rally_state.rallyloader.rally_count(), args[1]))

    elif(args[0] == "save"):
        if (len(args) < 2):
            print("Usage: rally save filename");

        mpstate.rally_state.rallyloader.save(args[1]);

        print "Saved rally file ", args[1];

    else:
        print_usage()

def mavlink_packet(m):
    '''handle incoming mavlink packet'''
    return #TODO when applicable

def send_rally_points():
    '''send rally points from fenceloader'''
    mpstate.mav_param.mavset(mpstate.master(),'RALLY_TOTAL',mpstate.rally_state.rallyloader.rally_count(),3)

    for i in range(mpstate.rally_state.rallyloader.rally_count()):
        p = mpstate.rally_state.rallyloader.rally_point(i)
        mpstate.master().mav.send(p)
       
def fetch_rally_point(i):
    '''fetch one rally point'''
    mpstate.master().mav.rally_fetch_point_send(mpstate.status.target_system,
                                                mpstate.status.target_component, i)
    tstart = time.time()
    p = None
    while time.time() - tstart < 1:
        p = mpstate.master().recv_match(type='RALLY_POINT', blocking=False)
        if p is not None:
            break
        time.sleep(0.1)
        continue
    if p is None:
        mpstate.console.error("Failed to fetch rally point %u" % i)
        return None
    return p

def list_rally_points():
    mpstate.rally_state.rallyloader.clear()
    rally_count = mpstate.mav_param.get('RALLY_TOTAL',0)
    if rally_count == 0:
        print("No rally points")
        return
    for i in range(int(rally_count)):
        p = fetch_rally_point(i)
        if p is None:
            return
        mpstate.rally_state.rallyloader.append_rally_point(p)

    for i in range(mpstate.rally_state.rallyloader.rally_count()):
        p = mpstate.rally_state.rallyloader.rally_point(i)
        mpstate.console.writeln("lat=%f lng=%f alt=%f break_alt=%f land_dir=%f" % (p.lat * 1e-7, p.lng * 1e-7, p.alt, p.break_alt, p.land_dir))

def print_usage():
    print("Usage: rally <list|load|save|add|clear>")
