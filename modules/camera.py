#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, cv, socket, errno, cPickle, signal, struct

# use the camera code from the cuav repo (see githib.com/tridge)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'camera'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'image'))
import chameleon, scanner, mavutil

mpstate = None

class camera_state(object):
    def __init__(self):
        self.running = False
        self.unload = threading.Event()
        self.unload.clear()

        self.capture_thread = None
        self.save_thread = None
        self.scan_thread = None
        self.transmit_thread = None
        self.view_thread = None

        self.capture_count = 0
        self.error_count = 0
        self.error_msg = None
        self.region_count = 0
        self.fps = 0
        self.cam = None
        self.save_queue = Queue.Queue()
        self.scan_queue = Queue.Queue()
        self.transmit_queue = Queue.Queue()
        self.viewing = False
        self.depth = 8
        self.gcs_address = None
        self.gcs_view_port = 7543
        self.brightness = 1.0

        # setup directory for images
        self.camera_dir = os.path.join(os.path.dirname(mpstate.logfile_name),
                                      "camera")
        try:
            os.mkdir(self.camera_dir)
        except OSError as e:
            if not e.errno in [ errno.EEXIST ]:
                raise

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
        if state.capture_thread is None:
            state.capture_thread = start_thread(capture_thread)
            state.save_thread = start_thread(save_thread)
            state.scan_thread = start_thread(scan_thread)
            state.transmit_thread = start_thread(transmit_thread)
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
        if state.view_thread is None:
            state.view_thread = start_thread(view_thread)
        state.viewing = True
    elif args[0] == "noview":
        if state.viewing:
            print("Stopping image viewer")
        state.viewing = False
    elif args[0] == "gcs":
        if len(args) != 2:
            print("usage: camera gcs <IPADDRESS>")
            return
        state.gcs_address = args[1]
    elif args[0] == "brightness":
        if len(args) != 2:
            print("usage: camera brightness <BRIGHTNESS>")
            return
        state.brightness = float(args[1])
    else:
        print("usage: camera <start|stop|status|view|noview|gcs|brightness>")



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
            if state.depth == 16:
                im = numpy.zeros((960,1280),dtype='uint16')
            else:
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

def save_aircraft_data(filename):
    '''save critical data needed for geo-referencing'''
    output = mavutil.mavlogfile(filename, write=True)
    for mtype in [ 'ATTITUDE', 'GPS_RAW', 'VFR_HUD' ]:
        master = mpstate.master()
        if mtype in master.messages:
            m = master.messages[mtype]
            timestamp = getattr(m, '_timestamp', 0)
            output.write(struct.pack('>Q', timestamp*1.0e6))
            output.write(m.get_msgbuf())
    output.close()

def save_thread():
    '''image save thread'''
    state = mpstate.camera_state
    count = 0
    while not state.unload.wait(0.01):
        if state.save_queue.empty():
            continue
        im = state.save_queue.get()
        chameleon.save_pgm(state.cam, 
                           '%s/raw%u.pgm' % (state.camera_dir, count), 
                           im)
        save_aircraft_data('%s/raw%u.log' % (state.camera_dir, count))
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
    counter = 0

    while not state.unload.wait(0.01):
        if state.scan_queue.empty():
            continue
        im = state.scan_queue.get()
        if state.depth == 16:
            scanner.debayer_16_8(im, im_640)
        else:
            scanner.debayer(im, im_640)

        regions = scanner.scan(im_640, im_marked)

        state.region_count += len(regions)
        if len(regions) > 0:
            cv.SaveImage('%s/marked%u.pnm' % (state.camera_dir, counter), 
                         im_marked)
            counter += 1
        jpeg = scanner.jpeg_compress(im_marked)
        state.transmit_queue.put((regions, jpeg))


def transmit_thread():
    '''thread for image transmit to GCS'''
    state = mpstate.camera_state

    connected = False

    i = 0
    while not state.unload.wait(0.01):
        if state.transmit_queue.empty():
            continue
        (regions, jpeg) = state.transmit_queue.get()

        if not connected:
            # try to connect if the GCS viewer is ready
            if state.gcs_address is None:
                continue
            try:
                port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                port.connect((state.gcs_address, state.gcs_view_port))
            except socket.error as e:
                if e.errno in [ errno.EHOSTUNREACH, errno.ECONNREFUSED ]:
                    continue
                raise
            connected = True
        try:
            port.send(cPickle.dumps((regions, jpeg), protocol=cPickle.HIGHEST_PROTOCOL))
        except socket.error:
            port.close()
            port = None
            connected = False

        # local save
        jfile = open('%s/j%u.jpg' % (state.camera_dir, i), "w")
        jfile.write(jpeg)
        jfile.close()
        i += 1

def view_thread():
    '''image viewing thread - this runs on the ground station'''
    state = mpstate.camera_state
    view_window = False

    connected = False
    port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    port.bind(("", state.gcs_view_port))
    port.listen(1)
    port.setblocking(1)
    sock = None
    pfile = None
    counter = 0
    view_window = False

    while not state.unload.wait(0.01):
        if state.viewing:
            if not view_window:
                view_window = True
                cv.NamedWindow('Viewer')
                key = cv.WaitKey(1)
            if not connected:
                try:
                    (sock, remote) = port.accept()
                    pfile = sock.makefile()
                except socket.error as e:
                    continue
                connected = True
            try:
                (regions, jpeg) = cPickle.load(pfile)
            except Exception:
                sock.close()
                pfile = None
                connected = False

            filename = '%s/v%u.jpg' % (state.camera_dir, counter)
            counter += 1
            jfile = open(filename, "w")
            jfile.write(jpeg)
            jfile.close()
            img = cv.LoadImage(filename)
            cv.ConvertScale(img, img, scale=state.brightness)
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


def start_thread(fn):
    '''start a thread running'''
    t = threading.Thread(target=fn)
    t.daemon = True
    t.start()
    return t

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.camera_state = camera_state()
    mpstate.command_map['camera'] = (cmd_camera, "camera control")
    state = mpstate.camera_state
    print("camera initialised")


def unload():
    '''unload module'''
    mpstate.camera_state.unload.set()
    if mpstate.camera_state.capture_thread is not None:
        mpstate.camera_state.capture_thread.join(1.0)
        mpstate.camera_state.save_thread.join(1.0)
        mpstate.camera_state.scan_thread.join(1.0)
        mpstate.camera_state.transmit_thread.join(1.0)
    if mpstate.camera_state.view_thread is not None:
        mpstate.camera_state.view_thread.join(1.0)
    if state.cam is not None:
        print("closing camera")
        chameleon.close(state.cam)
        state.cam = None
    print('camera unload OK')


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    pass
