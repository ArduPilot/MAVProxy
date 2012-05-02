#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, cv

# use the camera code from the cuav repo (see githib.com/tridge)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'camera'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'image'))
import chameleon, scanner

mpstate = None

class camera_state(object):
    def __init__(self):
        self.save_dir = "/data/tmp"
        self.running = False
        self.unload = threading.Event()
        self.unload.clear()
        self.capture_thread = None
        self.save_thread = None
        self.capture_count = 0
        self.error_count = 0
        self.error_msg = None
        self.region_count = 0
        self.fps = 0
        self.cam = None
        self.save_queue = Queue.Queue()
        self.scan_queue = Queue.Queue()
        self.viewing = False
        

def name():
    '''return module name'''
    return "camera"

def description():
    '''return module description'''
    return "ptgrey camera control"

def cmd_camera(args):
    '''camera commands'''
    state = mpstate.camera_state
    if args[0] == "start":
        state.colour = 0
        state.depth = 8
        try:
            # try colour first
            print("trying colour")
            state.cam = chameleon.open(1, state.depth)
            state.colour = 1
        except chameleon.error:
            # fallback to mono
            print("trying mono")
            state.cam = chameleon.open(0, state.depth)
            state.colour = 0
        state.capture_count = 0
        state.error_count = 0
        state.error_msg = None
        state.running = True
        print("started camera running")
    elif args[0] == "stop":
        if state.cam is not None:
            state.running = False
            time.sleep(0.3)
            if state.cam is not None:
                chameleon.close(state.cam)
                state.cam = None
            print("stopped camera running")
    elif args[0] == "status":
        print("Captured %u images %u errors %u regions %.1f fps" % (
            state.capture_count, state.error_count, state.region_count, state.fps))
    elif args[0] == "view":
        if not state.viewing:
            print("Starting image viewer")
        state.viewing = True
    elif args[0] == "noview":
        if state.viewing:
            print("Stopping image viewer")
        state.viewing = False



def capture_thread():
    '''camera capture thread'''
    state = mpstate.camera_state
    t1 = time.time()
    while not mpstate.camera_state.unload.wait(0.01):
        if not state.running:
            t1 = time.time()
            continue
        try:
            status = chameleon.trigger(state.cam)
            im = numpy.zeros((960,1280),dtype='uint8')
            (shutter, ftime) = chameleon.capture(state.cam, im)
            state.save_queue.put(im)
            state.scan_queue.put(im)
            state.capture_count += 1
            t2 = time.time()
            state.fps = 1.0 / (t2-t1)
            t1 = t2
        except chameleon.error, msg:
            state.error_count += 1
            state.error_msg = msg



def save_thread():
    '''image save thread'''
    state = mpstate.camera_state
    count = 0
    while not state.unload.wait(0.01):
        if state.save_queue.empty():
            continue
        im = state.save_queue.get()
        chameleon.save_pgm(state.cam, 'tmp/i%u.pgm' % count, im)
        count += 1
        # don't allow the queue to get too large, drop half the images
        # when larger than 50
        if state.save_queue.qsize() > 50:
            state.save_queue.get()
            count += 1

def scan_thread():
    '''image scanning thread'''
    state = mpstate.camera_state

    im_640 = numpy.zeros((480,640,3),dtype='uint8')
    im_marked = numpy.zeros((480,640,3),dtype='uint8')

    view_window = False
    
    while not state.unload.wait(0.01):
        if state.scan_queue.empty():
            continue
        im = state.scan_queue.get()
        scanner.debayer(im, im_640)
        regions = scanner.scan(im_640, im_marked)
        state.region_count += len(regions)
        if state.viewing:
            if not view_window:
                view_window = True
                cv.NamedWindow('Viewer')
                key = cv.WaitKey(1)
            mat = cv.fromarray(im_marked)
            img = cv.GetImage(mat)
            cv.ShowImage('Viewer', img)
            key = cv.WaitKey(1)
        else:
            if view_window:
                view_window = False
                cv.DestroyWindow('Viewer')
                for i in range(5):
                    # OpenCV bug - need to wait multiple times on destroy for all
                    # events to be processed
                    key = cv.WaitKey(1)



def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.camera_state = camera_state()
    mpstate.command_map['camera'] = (cmd_camera, "camera control")

    # start capture thread
    mpstate.camera_state.capture_thread = threading.Thread(target=capture_thread)
    mpstate.camera_state.capture_thread.daemon = True
    mpstate.camera_state.capture_thread.start()

    # start save thread
    mpstate.camera_state.save_thread = threading.Thread(target=save_thread)
    mpstate.camera_state.save_thread.daemon = True
    mpstate.camera_state.save_thread.start()

    # start scan thread
    mpstate.camera_state.scan_thread = threading.Thread(target=scan_thread)
    mpstate.camera_state.scan_thread.daemon = True
    mpstate.camera_state.scan_thread.start()

    print("camera initialised")


def unload():
    '''unload module'''
    mpstate.camera_state.unload.set()
    mpstate.camera_state.capture_thread.join(2.0)
    mpstate.camera_state.save_thread.join(2.0)
    mpstate.camera_state.scan_thread.join(2.0)
    if state.cam is not None:
        print("closing camera")
        chameleon.close(state.cam)
        state.cam = None
    print('camera unload OK')


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    pass
