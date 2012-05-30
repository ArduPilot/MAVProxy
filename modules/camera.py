#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, cv, errno, cPickle, signal, struct, fcntl, select, cStringIO

# use the camera code from the cuav repo (see githib.com/tridge)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'camera'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'image'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'cuav', 'lib'))
import scanner, mavutil, cuav_mosaic, mav_position, cuav_util, cuav_joe, block_xmit

# allow for replaying of previous flights
if os.getenv('FAKE_CHAMELEON'):
    print("Loaded fake chameleon backend")
    import fake_chameleon as chameleon
else:
    import chameleon

mpstate = None

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
        self.bandwidth = 120000
        self.capture_brightness = 150
        self.gamma = 950
        self.lens = 5.0
        self.brightness = 1.0
        self.quality = 75
        self.jpeg_size = 0
        self.xmit_queue = 0

        self.last_watch = 0
        self.frame_loss = 0
        self.colour = 1
        self.boundary = None

        # setup directory for images
        self.camera_dir = os.path.join(os.path.dirname(mpstate.logfile_name),
                                      "camera")
        cuav_util.mkdir_p(self.camera_dir)

        self.mpos = mav_position.MavInterpolator(backlog=500)
        self.joelog = cuav_joe.JoeLog(os.path.join(self.camera_dir, 'joe.log'))


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
        print("Cap %u imgs  %u err %u scan  %u regions %.0f jsize %.0f xmitq %u lst %u sq" % (
            state.capture_count, state.error_count, state.scan_count, state.region_count, 
            state.jpeg_size, state.xmit_queue, state.frame_loss, state.scan_queue.qsize()))
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
    elif args[0] == "quality":
        if len(args) != 2:
            print("quality=%u" % state.quality)
        else:
            state.quality = int(args[1])
    elif args[0] == "bandwidth":
        if len(args) != 2:
            print("bandwidth=%u" % state.bandwidth)
        else:
            state.bandwidth = int(args[1])
    elif args[0] == "boundary":
        if len(args) != 2:
            print("boundary=%s" % state.boundary)
        else:
            state.boundary = args[1]
    else:
        print("usage: camera <start|stop|status|view|noview|gcs|brightness|capbrightness|boundary|bandwidth>")


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
      im = numpy.zeros((960,1280),dtype='uint8' if state.depth==8 else 'uint16')
      base_time = time.time()
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
    cuav_util.mkdir_p(raw_dir)

    gammalog = open(os.path.join(state.camera_dir, "gamma.log"), "w")

    while not mpstate.camera_state.unload.wait(0.02):
        if not state.running:            
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
                
            gammalog.write('%f %s %u %u\n' % (frame_time+base_time,
                                              cuav_util.frame_time(frame_time+base_time),
                                              frame_counter,
                                              state.gamma))
            gammalog.flush()

            state.save_queue.put((base_time+frame_time,im))
            state.scan_queue.put((base_time+frame_time,im))
            state.capture_count += 1
            state.fps = 1.0/(frame_time - last_frame_time)

            last_frame_time = frame_time
            last_frame_counter = frame_counter
        except chameleon.error, msg:
            state.error_count += 1
            state.error_msg = msg
    if h is not None:
        chameleon.close(h)

def save_thread():
    '''image save thread'''
    state = mpstate.camera_state
    raw_dir = os.path.join(state.camera_dir, "raw")
    cuav_util.mkdir_p(raw_dir)
    while not state.unload.wait(0.02):
        if state.save_queue.empty():
            continue
        (frame_time,im) = state.save_queue.get()
        rawname = "raw%s" % cuav_util.frame_time(frame_time)
        chameleon.save_pgm('%s/%s.pgm' % (raw_dir, rawname), im)

def scan_thread():
    '''image scanning thread'''
    state = mpstate.camera_state

    while not state.unload.wait(0.02):
        try:
            # keep the queue size below 100, so we don't run out of memory
            if state.scan_queue.qsize() > 100:
                (frame_time,im) = state.scan_queue.get(timeout=0.2)
            (frame_time,im) = state.scan_queue.get(timeout=0.2)
        except Queue.Empty:
            continue

        t1 = time.time()
        im_full = numpy.zeros((960,1280,3),dtype='uint8')
        im_640 = numpy.zeros((480,640,3),dtype='uint8')
        scanner.debayer_full(im, im_full)
        scanner.downsample(im_full, im_640)
        regions = scanner.scan(im_640)
        t2 = time.time()
        state.scan_fps = 1.0 / (t2-t1)
        state.scan_count += 1

        state.region_count += len(regions)
        if state.transmit_queue.qsize() < 100:
            state.transmit_queue.put((frame_time, regions, im_full, im_640))

def log_joe_position(frame_time, regions, filename=None):
    '''add to joe.log if possible'''
    state = mpstate.camera_state
    try:
        pos = state.mpos.position(frame_time, 0)
        state.joelog.add_regions(frame_time, regions, pos, filename)
        return pos
    except mav_position.MavInterpolatorException:
        return None


class ImagePacket:
    '''a jpeg image sent to the ground station'''
    def __init__(self, frame_time, jpeg):
        self.frame_time = frame_time
        self.jpeg = jpeg

class ThumbPacket:
    '''a thumbnail region sent to the ground station'''
    def __init__(self, frame_time, regions, thumb, frame_loss, xmit_queue):
        self.frame_time = frame_time
        self.regions = regions
        self.thumb = thumb
        self.frame_loss = frame_loss
        self.xmit_queue = xmit_queue
        

def transmit_thread():
    '''thread for image transmit to GCS'''
    state = mpstate.camera_state

    tx_count = 0
    skip_count = 0
    bsend = block_xmit.BlockSender(0, state.bandwidth)

    while not state.unload.wait(0.02):
        bsend.tick()
        if state.transmit_queue.empty():
            continue

        (frame_time, regions, im_full, im_640) = state.transmit_queue.get()
        log_joe_position(frame_time, regions)

        state.xmit_queue = bsend.sendq_size()

        jpeg = None

        if len(regions) > 0 and bsend.sendq_size() < 2000:
            # send a region message with thumbnails to the ground station
            thumb = cuav_mosaic.CompositeThumbnail(im_640, regions, quality=state.quality)
            bsend.set_bandwidth(state.bandwidth)
            pkt = ThumbPacket(frame_time, regions, thumb, state.frame_loss, state.xmit_queue)

            # send matches with a higher priority
            bsend.send(cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL),
                       dest=(state.gcs_address, state.gcs_view_port),
                       priority=1)

        # send the image itself, compressing according to the queue size
        if len(regions) > 0:
            # this image had a Joe in it. Unless the sendq is very large, send the image in
            # full resolution with high quality
            if state.xmit_queue > 300:
                # nobody is listening
                jpeg = None
            elif state.xmit_queue > 50:
                # very large queue, send in half quality
                jpeg = scanner.jpeg_compress(im_640, int(state.quality*0.1))
            elif state.xmit_queue > 20:
                # very large queue, send in half quality
                jpeg = scanner.jpeg_compress(im_640, int(state.quality*0.3))
            elif state.xmit_queue > 10 or state.transmit_queue.qsize() > 50:
                # very large queue, send in half quality
                jpeg = scanner.jpeg_compress(im_640, int(state.quality*0.5))
            elif state.xmit_queue > 5:
                # moderate queue, send in normal quality
                jpeg = scanner.jpeg_compress(im_640, state.quality)
            else:
                # small queue, send in full res
                jpeg = scanner.jpeg_compress(im_full, max(state.quality, min(state.quality*2, 80)))
        else:
            # this image didn't have a Joe. Don't send at all unless the send queue is quite small.
            # if we do send, then send at low res
            if state.xmit_queue < 5 or (tx_count+skip_count) % 15 == 0:
                jpeg = scanner.jpeg_compress(im_640, state.quality)

        if jpeg is None:
            skip_count += 1
            continue

        # keep filtered image size
        state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(jpeg)
        
        tx_count += 1

        if state.gcs_address is None:
            continue
        bsend.set_bandwidth(state.bandwidth)
        pkt = ImagePacket(frame_time, jpeg)
        bsend.send(cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL),
                   dest=(state.gcs_address, state.gcs_view_port))

def view_thread():
    '''image viewing thread - this runs on the ground station'''
    import cuav_mosaic
    state = mpstate.camera_state
    view_window = False

    bsend = block_xmit.BlockSender(state.gcs_view_port, state.bandwidth)

    view_window = False
    image_count = 0
    region_count = 0
    mosaic = None
    view_dir = os.path.join(state.camera_dir, "view")
    thumb_dir = os.path.join(state.camera_dir, "thumb")
    cuav_util.mkdir_p(view_dir)
    cuav_util.mkdir_p(thumb_dir)

    mpstate.console.set_status('Images', 'Images %u' % image_count, row=6)
    mpstate.console.set_status('Lost', 'Lost %u' % 0, row=6)
    mpstate.console.set_status('Regions', 'Regions %u' % region_count, row=6)
    mpstate.console.set_status('JPGSize', 'JPG Size %.0f' % 0.0, row=6)
    mpstate.console.set_status('XMITQ', 'XMITQ %.0f' % 0.0, row=6)

    while not state.unload.wait(0.02):
        if state.viewing:
            bsend.tick()
            cv.WaitKey(1)
            if not view_window:
                view_window = True
                cv.NamedWindow('Viewer')
                key = cv.WaitKey(1)
                mosaic = cuav_mosaic.Mosaic(lens=state.lens)
                if state.boundary is not None:
                    boundary = cuav_util.polygon_load(state.boundary)
                    mosaic.set_boundary(boundary)
            buf = bsend.recv(0)
            if buf is None:
                continue
            try:
                obj = cPickle.loads(str(buf))
                if obj == None:
                    continue
            except Exception as e:
                continue

            if isinstance(obj, ThumbPacket):
                # we've received a set of thumbnails from the plane for a positive hit

                # save the thumbnails
                filename = '%s/v%s.jpg' % (thumb_dir, cuav_util.frame_time(obj.frame_time))
                chameleon.save_file(filename, obj.thumb)
                composite = cv.LoadImage(filename)
                thumbs = cuav_mosaic.ExtractThumbs(composite, len(obj.regions))

                # log the joe positions
                filename = '%s/v%s.jpg' % (view_dir, cuav_util.frame_time(obj.frame_time))
                pos = log_joe_position(obj.frame_time, obj.regions, filename)

                # update the mosaic and map
                mosaic.add_regions(obj.regions, thumbs, filename, pos=pos)

                # update console display
                region_count += len(obj.regions)
                state.frame_loss = obj.frame_loss
                state.xmit_queue = obj.xmit_queue

                mpstate.console.set_status('Lost', 'Lost %u' % state.frame_loss)
                mpstate.console.set_status('Regions', 'Regions %u' % region_count)
                mpstate.console.set_status('XMITQ', 'XMITQ %.0f' % state.xmit_queue)

            if isinstance(obj, ImagePacket):
                # we have an image from the plane

                # save it to disk
                filename = '%s/v%s.jpg' % (view_dir, cuav_util.frame_time(obj.frame_time))
                chameleon.save_file(filename, obj.jpeg)
                img = cv.LoadImage(filename)
                    
                # work out where we were at the time
                try:
                    pos = state.mpos.position(obj.frame_time, 0)
                except mav_position.MavInterpolatorException:
                    pos = None
                if pos:
                    mosaic.add_image(filename, img, pos)

                img = cv.LoadImage(filename)
                if img.width == 1280:
                    display_img = cv.CreateImage((640, 480), 8, 3)
                    cv.Resize(img, display_img)
                else:
                    display_img = img

                cv.ConvertScale(display_img, display_img, scale=state.brightness)
                cv.ShowImage('Viewer', display_img)

                # update console
                image_count += 1
                state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(obj.jpeg)
                mpstate.console.set_status('Images', 'Images %u' % image_count)
                mpstate.console.set_status('JPGSize', 'JPG Size %.0f' % state.jpeg_size)
                
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
    # update position interpolator
    state.mpos.add_msg(m)
