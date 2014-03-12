"""
    MAVProxy geofence module
"""
import os, time
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util

class fence_state(object):
    def __init__(self):
        self.fenceloader = mavwp.MAVFenceLoader()
        self.fence_enabled = False
        self.last_fence_breach = 0
        self.last_fence_status = 0
        return

def name():
    '''return module description'''
    return "fence"

def description():
    '''return module description'''
    return "geo-fence management"

def init(_mpstate):
    '''initialize module'''
    global mpstate
    mpstate = _mpstate
    mpstate.fence = fence_state()
    mpstate.command_map['fence'] = (cmd_fence, "geo-fence management")
    mpstate.completions["fence"] = ["<draw|list|clear>",
                                    "<load|save> (FILENAME)"]
    if mpstate.continue_mode and mpstate.status.logdir != None:
        fencetxt = os.path.join(mpstate.status.logdir, 'fence.txt')
        if os.path.exists(fencetxt):
            mpstate.fence.fenceloader.load(fencetxt)
            print("Loaded fence from %s" % fencetxt)

def mavlink_packet(m):
    '''handle and incoming mavlink packet'''
    if m.get_type() == "FENCE_STATUS":
        if not mpstate.fence.fence_enabled:
            mpstate.fence.fence_enabled = True
            mpstate.functions.say("fence enabled")
        if mpstate.fence.last_fence_breach != m.breach_time:
            mpstate.functions.say("fence breach")
        if mpstate.fence.last_fence_status != m.breach_status:
            if m.breach_status == mavutil.mavlink.FENCE_BREACH_NONE:
                mpstate.functions.say("fence OK")
        mpstate.fence.last_fence_breach = m.breach_time
        mpstate.fence.last_fence_status = m.breach_status

def cmd_fence(args):
    '''fence commands'''
    if len(args) < 1:
        print_usage()
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: fence load <filename>")
            return
        load_fence(args[1])
    elif args[0] == "list":
        list_fence(None)
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: fence save <filename>")
            return
        list_fence(args[1])
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: fence show <filename>")
            return
        mpstate.fence.fenceloader.load(args[1])
    elif args[0] == "draw":
        if not 'draw_lines' in mpstate.map_functions:
            print("No map drawing available")
            return        
        mpstate.map_functions['draw_lines'](fence_draw_callback)
        print("Drawing fence on map")
    elif args[0] == "clear":
        mpstate.mav_param.mavset(mpstate.master(),'FENCE_TOTAL', 0, 3)
    else:
        print("Usage: fence <list|load|save|show|clear|draw>")

def load_fence(filename):
    '''load fence points from a file'''
    try:
        mpstate.fence.fenceloader.target_system = mpstate.status.target_system
        mpstate.fence.fenceloader.target_component = mpstate.status.target_component
        mpstate.fence.fenceloader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u geo-fence points from %s" % (mpstate.fence.fenceloader.count(), filename))
    send_fence()

def send_fence():
    '''send fence points from fenceloader'''
    # must disable geo-fencing when loading
    action = mpstate.mav_param.get('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
    mpstate.mav_param.mavset(mpstate.master(), 'FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE, 3)
    mpstate.mav_param.mavset(mpstate.master(), 'FENCE_TOTAL', mpstate.fence.fenceloader.count(), 3)
    for i in range(mpstate.fence.fenceloader.count()):
        p = mpstate.fence.fenceloader.point(i)
        mpstate.master().mav.send(p)
        p2 = fetch_fence_point(i)
        if p2 is None:
            mpstate.mav_param.mavset(mpstate.master(), 'FENCE_ACTION', action, 3)
            return
        if (p.idx != p2.idx or
            abs(p.lat - p2.lat) >= 0.00003 or
            abs(p.lng - p2.lng) >= 0.00003):
            print("Failed to send fence point %u" % i)
            mpstate.mav_param.mavset(mpstate.master(), 'FENCE_ACTION', action, 3)
            return
    mpstate.mav_param.mavset(mpstate.master(),'FENCE_ACTION', action, 3)

def fetch_fence_point(i):
    '''fetch one fence point'''
    mpstate.master().mav.fence_fetch_point_send(mpstate.status.target_system,
                                                mpstate.status.target_component, i)
    tstart = time.time()
    p = None
    while time.time() - tstart < 1:
        p = mpstate.master().recv_match(type='FENCE_POINT', blocking=False)
        if p is not None:
            break
        time.sleep(0.1)
        continue
    if p is None:
        mpstate.console.error("Failed to fetch point %u" % i)
        return None
    return p

def fence_draw_callback(points):
    '''callback from drawing a fence'''
    mpstate.fence.fenceloader.clear()
    if len(points) < 3:
        return
    mpstate.fence.fenceloader.target_system = mpstate.status.target_system
    mpstate.fence.fenceloader.target_component = mpstate.status.target_component
    bounds = mp_util.polygon_bounds(points)
    (lat, lon, width, height) = bounds
    center = (lat+width/2, lon+height/2)
    mpstate.fence.fenceloader.add_latlon(center[0], center[1])
    for p in points:
        mpstate.fence.fenceloader.add_latlon(p[0], p[1])
    # close it
    mpstate.fence.fenceloader.add_latlon(points[0][0], points[0][1])
    send_fence()

def list_fence(filename):
    '''list fence points, optionally saving to a file'''
    mpstate.fence.fenceloader.clear()
    count = mpstate.mav_param.get('FENCE_TOTAL', 0)
    if count == 0:
        print("No geo-fence points")
        return
    for i in range(int(count)):
        p = fetch_fence_point(i)
        if p is None:
            return
        mpstate.fence.fenceloader.add(p)

    if filename is not None:
        try:
            mpstate.fence.fenceloader.save(filename)
        except Exception, msg:
            print("Unable to save %s - %s" % (filename, msg))
            return
        print("Saved %u geo-fence points to %s" % (mpstate.fence.fenceloader.count(), filename))
    else:
        for i in range(mpstate.fence.fenceloader.count()):
            p = mpstate.fence.fenceloader.point(i)
            mpstate.console.writeln("lat=%f lng=%f" % (p.lat, p.lng))
    if mpstate.status.logdir != None:
        fencetxt = os.path.join(mpstate.status.logdir, 'fence.txt')
        mpstate.fence.fenceloader.save(fencetxt)
        print("Saved fence to %s" % fencetxt)

def print_usage():
    print("usage: fence <list|load|save|clear|draw>")


