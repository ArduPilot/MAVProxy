#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, cv, errno, cPickle, signal, struct, fcntl, select, cStringIO

# use the camera code from the cuav repo (see githib.com/tridge)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'cuav', 'camera'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'cuav', 'image'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'cuav', 'lib'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'lib'))

import scanner, mavutil, cuav_mosaic, mav_position, cuav_util, cuav_joe, block_xmit, mp_image, cuav_region
from cam_params import CameraParams

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
        self.bandwidth = 40000
        self.capture_brightness = 150
        self.gamma = 950
        self.c_params = CameraParams(lens=4.0)
        self.brightness = 1.0
        self.quality = 75
        self.jpeg_size = 0
        self.xmit_queue = 0
        self.efficiency = 1.0

        self.last_watch = 0
        self.frame_loss = 0
        self.colour = 1
        self.boundary = None
        self.boundary_polygon = None
        self.packet_loss = 0
        self.save_pgm = True

        self.bandwidth_used = 0
        self.rtt_estimate = 0
        self.transmit = True

        self.roll_stabilised = True

        self.minscore = 3
        self.altitude = None
        
        # setup directory for images
        self.camera_dir = os.path.join(os.path.dirname(mpstate.logfile_name),
                                      "camera")
        cuav_util.mkdir_p(self.camera_dir)

        self.mpos = mav_position.MavInterpolator(backlog=5000, gps_lag=0.3)
        self.joelog = cuav_joe.JoeLog(os.path.join(self.camera_dir, 'joe.log'), append=mpstate.continue_mode)
        # load camera params
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..',
                            'cuav', 'data', 'chameleon1_arecont0.json')
        self.c_params.load(path)


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
        print("Cap %u imgs  %u err %u scan  %u regions %.0f jsize %.0f xmitq %u lst %u sq %.1f eff" % (
            state.capture_count, state.error_count, state.scan_count, state.region_count, 
            state.jpeg_size, state.xmit_queue, state.frame_loss, state.scan_queue.qsize(), state.efficiency))
    elif args[0] == "queue":
        print("scan %u  save %u  transmit %u  eff %.1f  bw %.1f  rtt %.1f" % (
                state.scan_queue.qsize(),
                state.save_queue.qsize(),
                state.transmit_queue.qsize(),
                state.efficiency,
                state.bandwidth_used,
                state.rtt_estimate))
    elif args[0] == "view":
        if mpstate.map is None:
            print("Please load map module first")
            return
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
    elif args[0] == "loss":
        if len(args) != 2:
            print("packet_loss=%u" % state.packet_loss)
        else:
            state.packet_loss = int(args[1])
    elif args[0] == "save":
        if len(args) != 2:
            print("save_pgm=%s" % str(state.save_pgm))
        else:
            state.save_pgm = bool(int(args[1]))
    elif args[0] == "transmit":
        if len(args) != 2:
            print("transmit=%s" % str(state.transmit))
        else:
            state.transmit = bool(int(args[1]))
    elif args[0] == "minscore":
        if len(args) != 2:
            print("minscore=%u" % state.minscore)
        else:
            state.minscore = int(args[1])
    elif args[0] == "altitude":
        if len(args) != 2:
            print("altitude=%u" % state.altitude)
        else:
            state.altitude = int(args[1])
    elif args[0] == "boundary":
        if len(args) != 2:
            print("boundary=%s" % state.boundary)
        else:
            state.boundary = args[1]
            state.boundary_polygon = cuav_util.polygon_load(state.boundary)
    else:
        print("usage: camera <start|stop|status|view|noview|gcs|brightness|capbrightness|boundary|bandwidth|transmit|loss|save|minscore|altitude>")


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

    if mpstate.continue_mode:
        mode = 'a'
    else:
        mode = 'w'
    gammalog = open(os.path.join(state.camera_dir, "gamma.log"), mode=mode)

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

            capture_time = time.time()
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
                
            gammalog.write('%f %f %f %s %u %u\n' % (frame_time,
                                                    frame_time+base_time,
                                                    capture_time,
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
        if state.save_pgm:
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
        regions = cuav_region.RegionsConvert(scanner.scan(im_640))
        t2 = time.time()
        state.scan_fps = 1.0 / (t2-t1)
        state.scan_count += 1

        regions = cuav_region.filter_regions(im_full, regions, min_score=state.minscore)

        state.region_count += len(regions)
        if state.transmit_queue.qsize() < 100:
            state.transmit_queue.put((frame_time, regions, im_full, im_640))

def get_plane_position(frame_time,roll=None):
    '''get a MavPosition object for the planes position if possible'''
    state = mpstate.camera_state
    try:
        pos = state.mpos.position(frame_time, 0,roll=roll)
        return pos
    except mav_position.MavInterpolatorException as e:
        print str(e)
        return None

def log_joe_position(pos, frame_time, regions, filename=None, thumb_filename=None):
    '''add to joe.log if possible, returning a list of (lat,lon) tuples
    for the positions of the identified image regions'''
    state = mpstate.camera_state
    return state.joelog.add_regions(frame_time, regions, pos, filename, thumb_filename, altitude=state.altitude)


class ImagePacket:
    '''a jpeg image sent to the ground station'''
    def __init__(self, frame_time, jpeg, xmit_queue, pos):
        self.frame_time = frame_time
        self.jpeg = jpeg
        self.xmit_queue = xmit_queue
        self.pos = pos

class ThumbPacket:
    '''a thumbnail region sent to the ground station'''
    def __init__(self, frame_time, regions, thumb, frame_loss, xmit_queue, pos):
        self.frame_time = frame_time
        self.regions = regions
        self.thumb = thumb
        self.frame_loss = frame_loss
        self.xmit_queue = xmit_queue
        self.pos = pos
        

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
        if state.roll_stabilised:
            roll=0
        else:
            roll=None
        pos = get_plane_position(frame_time, roll=roll)

        # this adds the latlon field to the regions
        log_joe_position(pos, frame_time, regions)

        # filter out any regions outside the boundary
        if state.boundary_polygon:
            regions = cuav_region.filter_boundary(regions, state.boundary_polygon, pos)

        state.xmit_queue = bsend.sendq_size()
        state.efficiency = bsend.get_efficiency()
        state.bandwidth_used = bsend.get_bandwidth_used()
        state.rtt_estimate = bsend.get_rtt_estimate()

        jpeg = None

        if len(regions) > 0 and bsend.sendq_size() < 2000:
            # send a region message with thumbnails to the ground station
            thumb = cuav_mosaic.CompositeThumbnail(cv.GetImage(cv.fromarray(im_full)),
                                                   regions, quality=state.quality, thumb_size=80)
            bsend.set_bandwidth(state.bandwidth)
            bsend.set_packet_loss(state.packet_loss)
            pkt = ThumbPacket(frame_time, regions, thumb, state.frame_loss, state.xmit_queue, pos)

            # send matches with a higher priority
            if state.transmit:
                bsend.send(cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL),
                           dest=(state.gcs_address, state.gcs_view_port),
                           priority=1)
                bsend.tick()

        # Base how many images we send on the send queue size
        send_frequency = state.xmit_queue // 3
        if send_frequency == 0 or (tx_count+skip_count) % send_frequency == 0:
            jpeg = scanner.jpeg_compress(im_640, state.quality)

        if jpeg is None:
            skip_count += 1
            continue

        # keep filtered image size
        state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(jpeg)
        
        tx_count += 1

        if state.gcs_address is None:
            continue
        bsend.set_packet_loss(state.packet_loss)
        bsend.set_bandwidth(state.bandwidth)
        pkt = ImagePacket(frame_time, jpeg, state.xmit_queue, pos)
        str = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
        bsend.send(str,
                   dest=(state.gcs_address, state.gcs_view_port))

def reload_mosaic(mosaic):
    '''reload state into mosaic'''
    state = mpstate.camera_state
    regions = []
    last_thumbfile = None
    last_joe = None
    joes = cuav_joe.JoeIterator(state.joelog.filename)
    for joe in joes:
        print joe
        if joe.thumb_filename == last_thumbfile or last_thumbfile is None:
            regions.append(joe.r)
            last_joe = joe
            last_thumbfile = joe.thumb_filename
        else:
            try:
                composite = cv.LoadImage(last_joe.thumb_filename)
                thumbs = cuav_mosaic.ExtractThumbs(composite, len(regions))
                mosaic.add_regions(regions, thumbs, last_joe.image_filename, last_joe.pos)
            except Exception:
                pass                
            regions = []
            last_joe = None
            last_thumbfile = None
    if last_joe:
        try:
            composite = cv.LoadImage(last_joe.thumb_filename)
            thumbs = cuav_mosaic.ExtractThumbs(composite, len(regions))
            mosaic.add_regions(regions, thumbs, last_joe.image_filename, last_joe.pos)
        except Exception:
            pass
        


def view_thread():
    '''image viewing thread - this runs on the ground station'''
    import cuav_mosaic
    state = mpstate.camera_state

    bsend = block_xmit.BlockSender(state.gcs_view_port, state.bandwidth)

    view_window = False
    image_count = 0
    thumb_count = 0
    image_total_bytes = 0
    jpeg_total_bytes = 0
    thumb_total_bytes = 0
    region_count = 0
    mosaic = None
    view_dir = os.path.join(state.camera_dir, "view")
    thumb_dir = os.path.join(state.camera_dir, "thumb")
    cuav_util.mkdir_p(view_dir)
    cuav_util.mkdir_p(thumb_dir)

    img_window = mp_image.MPImage(title='Camera')

    mpstate.console.set_status('Images', 'Images %u' % image_count, row=6)
    mpstate.console.set_status('Lost', 'Lost %u' % 0, row=6)
    mpstate.console.set_status('Regions', 'Regions %u' % region_count, row=6)
    mpstate.console.set_status('JPGSize', 'JPGSize %.0f' % 0.0, row=6)
    mpstate.console.set_status('XMITQ', 'XMITQ %.0f' % 0.0, row=6)

    mpstate.console.set_status('Thumbs', 'Thumbs %u' % thumb_count, row=7)
    mpstate.console.set_status('ThumbSize', 'ThumbSize %.0f' % 0.0, row=7)
    mpstate.console.set_status('ImageSize', 'ImageSize %.0f' % 0.0, row=7)

    while not state.unload.wait(0.02):
        if state.viewing:
            bsend.tick(packet_count=1000)
            if not view_window:
                view_window = True
                mosaic = cuav_mosaic.Mosaic(slipmap=mpstate.map, C=state.c_params)
                if state.boundary_polygon is not None:
                    mosaic.set_boundary(state.boundary_polygon)
                if mpstate.continue_mode:
                    print("MOSAIC RELOAD")
                    reload_mosaic(mosaic)
                else:
                    print("NO MOSAIC RELOAD")

            # check for keyboard events
            mosaic.check_events()

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
                thumb_total_bytes += len(buf)

                # save the thumbnails
                thumb_filename = '%s/v%s.jpg' % (thumb_dir, cuav_util.frame_time(obj.frame_time))
                chameleon.save_file(thumb_filename, obj.thumb)
                composite = cv.LoadImage(thumb_filename)
                thumbs = cuav_mosaic.ExtractThumbs(composite, len(obj.regions))

                # log the joe positions
                filename = '%s/v%s.jpg' % (view_dir, cuav_util.frame_time(obj.frame_time))
                pos = obj.pos
                log_joe_position(pos, obj.frame_time, obj.regions, filename, thumb_filename)

                # update the mosaic and map
                mosaic.add_regions(obj.regions, thumbs, filename, pos=pos)

                # update console display
                region_count += len(obj.regions)
                state.frame_loss = obj.frame_loss
                state.xmit_queue = obj.xmit_queue
                thumb_count += 1

                mpstate.console.set_status('Lost', 'Lost %u' % state.frame_loss)
                mpstate.console.set_status('Regions', 'Regions %u' % region_count)
                mpstate.console.set_status('XMITQ', 'XMITQ %.0f' % state.xmit_queue)
                mpstate.console.set_status('Thumbs', 'Thumbs %u' % thumb_count)
                mpstate.console.set_status('ThumbSize', 'ThumbSize %.0f' % (thumb_total_bytes/thumb_count))

            if isinstance(obj, ImagePacket):
                # we have an image from the plane
                image_total_bytes += len(buf)

                state.xmit_queue = obj.xmit_queue
                mpstate.console.set_status('XMITQ', 'XMITQ %.0f' % state.xmit_queue)

                # save it to disk
                filename = '%s/v%s.jpg' % (view_dir, cuav_util.frame_time(obj.frame_time))
                chameleon.save_file(filename, obj.jpeg)
                img = cv.LoadImage(filename)
                if img.width == 1280:
                    display_img = cv.CreateImage((640, 480), 8, 3)
                    cv.Resize(img, display_img)
                else:
                    display_img = img

                mosaic.add_image(obj.frame_time, filename, obj.pos)

                cv.ConvertScale(display_img, display_img, scale=state.brightness)
                img_window.set_image(display_img, bgr=True)

                # update console
                image_count += 1
                jpeg_total_bytes += len(obj.jpeg)
                state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(obj.jpeg)
                mpstate.console.set_status('Images', 'Images %u' % image_count)
                mpstate.console.set_status('JPGSize', 'JPG Size %.0f' % (jpeg_total_bytes/image_count))
                mpstate.console.set_status('ImageSize', 'ImageSize %.0f' % (image_total_bytes/image_count))
                
        else:
            if view_window:
                view_window = False


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
