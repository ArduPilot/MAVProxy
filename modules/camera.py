#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, cv, socket, errno, cPickle, signal, struct

# use the camera code from the cuav repo (see githib.com/tridge)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'camera'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'image'))
import chameleon, scanner, mavutil

mpstate = None

def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

class camera_state(object):
    def __init__(self):
        self.running = False
        self.unload = threading.Event()
        self.unload.clear()

        self.capture_thread = None
        self.save_thread = None
        self.scan_thread1 = None
        self.scan_thread2 = None
        self.transmit_thread = None
        self.view_thread = None

        self.capture_count = 0
        self.scan_count = 0
        self.error_count = 0
        self.error_msg = None
        self.region_count = 0
        self.fps = 0
        self.scan_fps = 0
        self.cam = None
        self.save_queue = Queue.Queue()
        self.scan_queue = Queue.Queue()
        self.transmit_queue = Queue.Queue()
        self.viewing = False
        self.depth = 8
        self.gcs_address = None
        self.gcs_view_port = 7543
        self.capture_brightness = 1000
        self.gamma = 950
        self.brightness = 1.0
        self.full_resolution = 0
        self.quality = 95

        self.last_watch = 0
        self.frame_loss = 0
        self.colour = 1

        # setup directory for images
        self.camera_dir = os.path.join(os.path.dirname(mpstate.logfile_name),
                                      "camera")

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
        state.capture_count = 0
        state.error_count = 0
        state.error_msg = None
        state.running = True
        if state.capture_thread is None:
            state.capture_thread = start_thread(capture_thread)
            state.save_thread = start_thread(save_thread)
            state.scan_thread1 = start_thread(scan_thread)
            state.scan_thread2 = start_thread(scan_thread)
            state.transmit_thread = start_thread(transmit_thread)
        print("started camera running")
    elif args[0] == "stop":
        state.running = False
        print("stopped camera capture")
    elif args[0] == "status":
        print("Captured %u images  %u errors  %u scanned  %u regions %.1f fps  %.1f scan_fps  %u lost" % (
            state.capture_count, state.error_count, state.scan_count, state.region_count, 
            state.fps, state.scan_fps, state.frame_loss))
    elif args[0] == "queue":
        print("scan %u  save %u  transmit %u" % (
                state.scan_queue.qsize(),
                state.save_queue.qsize(),
                state.transmit_queue.qsize()))
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
            print("brightness=%f" % state.brightness)
        else:
            state.brightness = float(args[1])
    elif args[0] == "capbrightness":
        if len(args) != 2:
            print("capbrightness=%u" % state.capture_brightness)
        else:
            state.capture_brightness = int(args[1])
    elif args[0] == "gamma":
        if len(args) != 2:
            print("gamma=%u" % state.gamma)
        else:
            state.gamma = int(args[1])
    elif args[0] == "fullres":
        if len(args) != 2 or args[1] not in ['0','1']:
            print("usage: camera fullres <0/1>")
            return
        state.full_resolution = int(args[1])
    else:
        print("usage: camera <start|stop|status|view|noview|gcs|brightness|capbrightness>")


def get_base_time():
  '''we need to get a baseline time from the camera. To do that we trigger
  in single shot mode until we get a good image, and use the time we 
  triggered as the base time'''
  state = mpstate.camera_state
  frame_time = None
  error_count = 0

  print('Opening camera')
  h = chameleon.open(state.colour, state.depth, state.capture_brightness)

  print('Getting camare base_time')
  while frame_time is None:
    try:
      base_time = time.time()
      im = numpy.zeros((960,1280),dtype='uint8' if state.depth==8 else 'uint16')
      chameleon.trigger(h, False)
      frame_time, frame_counter, shutter = chameleon.capture(h, 1000, im)
      base_time -= frame_time
    except chameleon.error:
      print('failed to capture')
      error_count += 1
      if error_count > 3:
        error_count = 0
        print('re-opening camera')
        chameleon.close(h)
        h = chameleon.open(state.colour, state.depth, state.capture_brightness)
  print('base_time=%f' % base_time)
  return h, base_time, frame_time

def capture_thread():
    '''camera capture thread'''
    state = mpstate.camera_state
    t1 = time.time()
    last_frame_counter = 0
    h = None
    last_gamma = 0

    raw_dir = os.path.join(state.camera_dir, "raw")
    mkdir_p(raw_dir)

    while not mpstate.camera_state.unload.wait(0.02):
        if not state.running:            
            t1 = time.time()
            if h is not None:
                chameleon.close(h)
                h = None
            continue
        try:
            if h is None:
                h, base_time, last_frame_time = get_base_time()
                # put into continuous mode
                chameleon.trigger(h, True)

            frame_time = time.time()
            if state.depth == 16:
                im = numpy.zeros((960,1280),dtype='uint16')
            else:
                im = numpy.zeros((960,1280),dtype='uint8')
            if last_gamma != state.gamma:
                chameleon.set_gamma(h, state.gamma)
                last_gamma = state.gamma
            frame_time, frame_counter, shutter = chameleon.capture(h, 1000, im)
            if frame_time < last_frame_time:
                base_time += 128
            if last_frame_counter != 0:
                state.frame_loss += frame_counter - (last_frame_counter+1)
                
            state.save_queue.put((base_time+frame_time,im))
            state.scan_queue.put((base_time+frame_time,im))
            state.capture_count += 1
            t2 = time.time()
            state.fps = 1.0 / (t2-t1)
            t1 = t2

            last_frame_time = frame_time
            last_frame_counter = frame_counter
        except chameleon.error, msg:
            state.error_count += 1
            state.error_msg = msg
    if h is not None:
        chameleon.close(h)

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

def timestamp(frame_time):
    '''return a localtime timestamp with 0.01 second resolution'''
    hundredths = int(frame_time * 100.0) % 100
    return "%s%02u" % (time.strftime("%Y%m%d%H%M%S", time.localtime(frame_time)), hundredths)

def save_thread():
    '''image save thread'''
    state = mpstate.camera_state
    raw_dir = os.path.join(state.camera_dir, "raw")
    mkdir_p(raw_dir)
    while not state.unload.wait(0.02):
        if state.save_queue.empty():
            continue
        (frame_time,im) = state.save_queue.get()
        rawname = "raw%s" % timestamp(frame_time)
        chameleon.save_pgm('%s/%s.pgm' % (raw_dir, rawname), im)
        save_aircraft_data('%s/%s.log' % (raw_dir, rawname))

def scan_thread():
    '''image scanning thread'''
    state = mpstate.camera_state

    while not state.unload.wait(0.02):
        try:
            (frame_time,im) = state.scan_queue.get(timeout=0.2)
        except Queue.Empty:
            continue

        t1 = time.time()
        im_640 = numpy.zeros((480,640,3),dtype='uint8')
        scanner.debayer(im, im_640)
        regions = scanner.scan(im_640)
        t2 = time.time()
        state.scan_fps = 1.0 / (t2-t1)
        state.scan_count += 1

        state.region_count += len(regions)
        state.transmit_queue.put((frame_time, regions, im, im_640))

def transmit_thread():
    '''thread for image transmit to GCS'''
    state = mpstate.camera_state

    connected = False

    while not state.unload.wait(0.02):
        if state.transmit_queue.empty():
            continue
        # only send the latest image to the ground station
        while state.transmit_queue.qsize() > 0:
            (frame_time, regions, im, im_640) = state.transmit_queue.get()
        if state.full_resolution:
            # we're transmitting a full size image
            im_colour = numpy.zeros((960,1280,3),dtype='uint8')
            scanner.debayer_full(im, im_colour)
            jpeg = scanner.jpeg_compress(im_colour, state.quality)
        else:
            # compress a 640x480 image
            jpeg = scanner.jpeg_compress(im_640, state.quality)

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
            port.send(cPickle.dumps((frame_time, regions, jpeg), protocol=cPickle.HIGHEST_PROTOCOL))
        except socket.error:
            port.close()
            port = None
            connected = False

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
    view_window = False
    image_count = 0
    region_count = 0

    view_dir = os.path.join(state.camera_dir, "view")
    mkdir_p(view_dir)

    mpstate.console.set_status('Images', 'Images %u' % image_count, row=6)
    mpstate.console.set_status('Regions', 'Regions %u' % region_count, row=6)

    while not state.unload.wait(0.02):
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
                (frame_time, regions, jpeg) = cPickle.load(pfile)
            except Exception:
                sock.close()
                pfile = None
                connected = False
                continue

            filename = '%s/v%s.jpg' % (view_dir, timestamp(frame_time))
            chameleon.save_file(filename, jpeg)
            img = cv.LoadImage(filename)
            if img.width == 640:
                region_scale = 1
            else:
                region_scale = 2
            for r in regions:
                (x1,y1,x2,y2) = [x * region_scale for x in r]
                cv.Rectangle(img, (x1,y1), (x2,y2), (255,0,0), 2)
            cv.ConvertScale(img, img, scale=state.brightness)
            cv.ShowImage('Viewer', img)
            key = cv.WaitKey(1)

            image_count += 1
            region_count += len(regions)
            mpstate.console.set_status('Images', 'Images %u' % image_count)
            mpstate.console.set_status('Regions', 'Regions %u' % region_count)
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
    state.running = False
    mpstate.camera_state.unload.set()
    if mpstate.camera_state.capture_thread is not None:
        mpstate.camera_state.capture_thread.join(1.0)
        mpstate.camera_state.save_thread.join(1.0)
        mpstate.camera_state.scan_thread1.join(1.0)
        mpstate.camera_state.scan_thread2.join(1.0)
        mpstate.camera_state.transmit_thread.join(1.0)
    if mpstate.camera_state.view_thread is not None:
        mpstate.camera_state.view_thread.join(1.0)
    print('camera unload OK')


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.camera_state
    if mpstate.status.watch in ["camera","queue"] and time.time() > state.last_watch+1:
        state.last_watch = time.time()
        cmd_camera(["status" if mpstate.status.watch == "camera" else "queue"])
