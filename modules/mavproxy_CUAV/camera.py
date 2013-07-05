#!/usr/bin/env python
'''camera control for ptgrey chameleon camera'''

import time, threading, sys, os, numpy, Queue, errno, cPickle, signal, struct, fcntl, select, cStringIO
import cv2.cv as cv

from cuav.image import scanner
from pymavlink import mavutil
from cuav.lib import cuav_mosaic, mav_position, cuav_util, cuav_joe, block_xmit, cuav_region
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_image
from cuav.camera.cam_params import CameraParams
from MAVProxy.modules.mavproxy_map import mp_slipmap

# allow for replaying of previous flights
if os.getenv('FAKE_CHAMELEON'):
    print("Loaded fake chameleon backend")
    import cuav.camera.fake_chameleon as chameleon
else:
    import cuav.camera.chameleon as chameleon

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

        self.settings = mp_settings.MPSettings(
            [ ('depth', int, 8),
              ('gcs_address', str, None),
              ('gcs_view_port', int, 7543),
              ('bandwidth',  int, 40000),
              ('bandwidth2', int, 2000),
              ('capture_brightness', int, 150),
              ('gamma', int, 950),
              ('brightness', float, 1.0),
              ('quality', int, 75),
              ('save_pgm', int, 1),
              ('transmit', int, 1),
              ('roll_stabilised', int, 1),
              ('minscore', int, 75),
              ('minscore2', int, 500),
              ('altitude', int, None),
              ('send1', int, 1),
              ('send2', int, 1),
              ('maxqueue1', int, None),
              ('maxqueue2', int, 30),
              ('thumbsize', int, 60),
              ('packet_loss', int, 0),             
              ('gcs_slave', str, None),
              ('filter_type', str, 'simple')  
              ]
            )

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
        
        self.c_params = CameraParams(lens=4.0)
        self.jpeg_size = 0
        self.xmit_queue = 0
        self.xmit_queue2 = 0
        self.efficiency = 1.0

        self.last_watch = 0
        self.frame_loss = 0
        self.boundary = None
        self.boundary_polygon = None

        self.bandwidth_used = 0
        self.rtt_estimate = 0
        self.bsocket = None
        self.bsend2 = None
        self.bsend_slave = None
        
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


class MavSocket:
    '''map block_xmit onto MAVLink data packets'''
    def __init__(self, master):
        self.master = master
        self.incoming = []

    def sendto(self, buf, dest):
        if len(buf) <= 16:
            self.master.mav.data16_send(0, len(buf), buf)
        if len(buf) <= 32:
            self.master.mav.data32_send(0, len(buf), buf)
        elif len(buf) <= 64:
            self.master.mav.data64_send(0, len(buf), buf)
        elif len(buf) <= 96:
            self.master.mav.data96_send(0, len(buf), buf)
        else:
            print("PACKET TOO LARGE %u" % len(buf))
            raise RuntimeError('packet too large %u' % len(buf))

    def recvfrom(self, size):
        if len(self.incoming) == 0:
            return ('', 'mavlink')
        m = self.incoming.pop(0)
        buf = bytes(m.data[:m.len])
        return (buf, 'mavlink')


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
        print("Cap imgs:%u err:%u scan:%u regions:%u jsize:%.0f xmitq:%u/%u lst:%u sq:%.1f eff:%.2f" % (
            state.capture_count, state.error_count, state.scan_count, state.region_count, 
            state.jpeg_size,
            state.xmit_queue, state.xmit_queue2, state.frame_loss, state.scan_queue.qsize(), state.efficiency))
        if state.bsend2:
            state.bsend2.report(detailed=False)
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
    elif args[0] == "set":
        if len(args) < 3:
            state.settings.show_all()
        else:
            state.settings.set(args[1], args[2])
    elif args[0] == "boundary":
        if len(args) != 2:
            print("boundary=%s" % state.boundary)
        else:
            state.boundary = args[1]
            state.boundary_polygon = cuav_util.polygon_load(state.boundary)
            if mpstate.map is not None:
                mpstate.map.add_object(mp_slipmap.SlipPolygon('boundary', state.boundary_polygon, layer=1, linewidth=2, colour=(0,0,255)))
                
    else:
        print("usage: camera <start|stop|status|view|noview|boundary|set>")


def cmd_remote(args):
    '''camera commands'''
    state = mpstate.camera_state
    cmd = " ".join(args)
    if state.bsend2 is None:
        print("bsend2 not initialised")
        return
    pkt = CommandPacket(cmd)
    buf = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
    state.bsend2.send(buf, priority=10000)

def get_base_time():
  '''we need to get a baseline time from the camera. To do that we trigger
  in single shot mode until we get a good image, and use the time we 
  triggered as the base time'''
  state = mpstate.camera_state
  frame_time = None
  error_count = 0

  print('Opening camera')
  h = chameleon.open(1, state.settings.depth, state.settings.capture_brightness)

  print('Getting camare base_time')
  while frame_time is None:
    try:
      im = numpy.zeros((960,1280),dtype='uint8' if state.settings.depth==8 else 'uint16')
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
        h = chameleon.open(1, state.settings.depth, state.settings.capture_brightness)
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
            if state.settings.depth == 16:
                im = numpy.zeros((960,1280),dtype='uint16')
            else:
                im = numpy.zeros((960,1280),dtype='uint8')
            if last_gamma != state.settings.gamma:
                chameleon.set_gamma(h, state.settings.gamma)
                last_gamma = state.settings.gamma
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
                                                    state.settings.gamma))
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
    frame_count = 0
    while not state.unload.wait(0.02):
        if state.save_queue.empty():
            continue
        (frame_time,im) = state.save_queue.get()
        rawname = "raw%s" % cuav_util.frame_time(frame_time)
        frame_count += 1
        if state.settings.save_pgm != 0:
            if frame_count % state.settings.save_pgm == 0:
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

        regions = cuav_region.filter_regions(im_full, regions, min_score=min(state.settings.minscore,state.settings.minscore2),
                                             filter_type=state.settings.filter_type)

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
    return state.joelog.add_regions(frame_time, regions, pos, filename, thumb_filename, altitude=state.settings.altitude)


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

class CommandPacket:
    '''a command to run on the plane'''
    def __init__(self, command):
        self.command = command

class CommandResponse:
    '''a command response from the plane'''
    def __init__(self, response):
        self.response = response


def transmit_thread():
    '''thread for image transmit to GCS'''
    state = mpstate.camera_state

    tx_count = 0
    skip_count = 0
    bsend = block_xmit.BlockSender(0, bandwidth=state.settings.bandwidth, debug=False)
    state.bsocket = MavSocket(mpstate.mav_master[0])
    state.bsend2 = block_xmit.BlockSender(mss=96, sock=state.bsocket, dest_ip='mavlink', dest_port=0, backlog=5, debug=False)
    state.bsend2.set_bandwidth(state.settings.bandwidth2)

    while not state.unload.wait(0.02):
        bsend.tick(packet_count=1000, max_queue=state.settings.maxqueue1)
        state.bsend2.tick(packet_count=1000, max_queue=state.settings.maxqueue2)
        check_commands()
        if state.transmit_queue.empty():
            continue

        (frame_time, regions, im_full, im_640) = state.transmit_queue.get()
        if state.settings.roll_stabilised:
            roll=0
        else:
            roll=None
        pos = get_plane_position(frame_time, roll=roll)

        # this adds the latlon field to the regions
        log_joe_position(pos, frame_time, regions)

        # filter out any regions outside the boundary
        if state.boundary_polygon:
            regions = cuav_region.filter_boundary(regions, state.boundary_polygon, pos)
            regions = cuav_region.filter_regions(im_full, regions, min_score=state.settings.minscore,
                                                 filter_type=state.settings.filter_type)

        state.xmit_queue = bsend.sendq_size()
        state.xmit_queue2 = state.bsend2.sendq_size()
        state.efficiency = bsend.get_efficiency()
        state.bandwidth_used = bsend.get_bandwidth_used()
        state.rtt_estimate = bsend.get_rtt_estimate()

        jpeg = None

        if len(regions) > 0:
            lowscore = 0
            highscore = 0
            for r in regions:
                lowscore = min(lowscore, r.score)
                highscore = max(highscore, r.score)
                
            if state.settings.transmit:
                # send a region message with thumbnails to the ground station
                thumb = None
                if state.settings.send1:
                    thumb_img = cuav_mosaic.CompositeThumbnail(cv.GetImage(cv.fromarray(im_full)),
                                                               regions,
                                                               thumb_size=state.settings.thumbsize)
                    thumb = scanner.jpeg_compress(numpy.ascontiguousarray(cv.GetMat(thumb_img)), state.settings.quality)

                    pkt = ThumbPacket(frame_time, regions, thumb, state.frame_loss, state.xmit_queue, pos)

                    buf = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
                    bsend.set_bandwidth(state.settings.bandwidth)
                    bsend.set_packet_loss(state.settings.packet_loss)
                    bsend.send(buf,
                               dest=(state.settings.gcs_address, state.settings.gcs_view_port),
                               priority=1)
                # also send thumbnails via 900MHz telemetry
                if state.settings.send2 and highscore >= state.settings.minscore2:
                    if thumb is None or lowscore < state.settings.minscore2:
                        # remove some of the regions
                        regions = cuav_region.filter_regions(im_full, regions, min_score=state.settings.minscore2,
                                                             filter_type=state.settings.filter_type)
                        thumb_img = cuav_mosaic.CompositeThumbnail(cv.GetImage(cv.fromarray(im_full)),
                                                                   regions,
                                                                   thumb_size=state.settings.thumbsize)
                        thumb = scanner.jpeg_compress(numpy.ascontiguousarray(cv.GetMat(thumb_img)), state.settings.quality)
                        pkt = ThumbPacket(frame_time, regions, thumb, state.frame_loss, state.xmit_queue, pos)

                        buf = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
                    state.bsend2.set_bandwidth(state.settings.bandwidth2)
                    state.bsend2.send(buf, priority=highscore)

        # Base how many images we send on the send queue size
        send_frequency = state.xmit_queue // 3
        if send_frequency == 0 or (tx_count+skip_count) % send_frequency == 0:
            jpeg = scanner.jpeg_compress(im_640, state.settings.quality)

        if jpeg is None:
            skip_count += 1
            continue

        # keep filtered image size
        state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(jpeg)
        
        tx_count += 1

        if state.settings.gcs_address is None:
            continue
        bsend.set_packet_loss(state.settings.packet_loss)
        bsend.set_bandwidth(state.settings.bandwidth)
        pkt = ImagePacket(frame_time, jpeg, state.xmit_queue, pos)
        str = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
        bsend.send(str,
                   dest=(state.settings.gcs_address, state.settings.gcs_view_port))

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
                mosaic.set_brightness(state.settings.brightness)
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
            mosaic.set_brightness(state.settings.brightness)
            mosaic.add_regions(regions, thumbs, last_joe.image_filename, last_joe.pos)
        except Exception:
            pass
        


def view_thread():
    '''image viewing thread - this runs on the ground station'''
    from cuav.lib import cuav_mosaic
    state = mpstate.camera_state

    bsend = block_xmit.BlockSender(state.settings.gcs_view_port, bandwidth=state.settings.bandwidth)
    state.bsocket = MavSocket(mpstate.mav_master[0])
    state.bsend2 = block_xmit.BlockSender(mss=96, sock=state.bsocket, dest_ip='mavlink', dest_port=0, backlog=5, debug=False)
    state.bsend2.set_bandwidth(state.settings.bandwidth2)

    view_window = False
    image_count = 0
    thumb_count = 0
    image_total_bytes = 0
    jpeg_total_bytes = 0
    thumb_total_bytes = 0
    region_count = 0
    mosaic = None
    thumbs_received = set()
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

    ack_time = time.time()

    while not state.unload.wait(0.02):
        if state.viewing:
            tnow = time.time()
            if tnow - ack_time > 0.1:
                bsend.tick(packet_count=1000, max_queue=state.settings.maxqueue1)
                state.bsend2.tick(packet_count=1000, max_queue=state.settings.maxqueue2)
                if state.bsend_slave is not None:
                    state.bsend_slave.tick(packet_count=1000)
                ack_time = tnow
            if not view_window:
                view_window = True
                mosaic = cuav_mosaic.Mosaic(slipmap=mpstate.map, C=state.c_params)
                if state.boundary_polygon is not None:
                    mosaic.set_boundary(state.boundary_polygon)
                if mpstate.continue_mode:
                    reload_mosaic(mosaic)

            # check for keyboard events
            mosaic.check_events()

            buf = bsend.recv(0)
            if buf is None:
                buf = state.bsend2.recv(0)
            if buf is None:
                continue
            try:
                obj = cPickle.loads(str(buf))
                if obj == None:
                    continue
            except Exception as e:
                continue

            if state.settings.gcs_slave is not None:
                if state.bsend_slave is None:
                    state.bsend_slave = block_xmit.BlockSender(0, bandwidth=state.settings.bandwidth*10, debug=False)
                state.bsend_slave.send(buf,
                                       dest=(state.settings.gcs_slave, state.settings.gcs_view_port),
                                       priority=1)

            if isinstance(obj, ThumbPacket):
                # we've received a set of thumbnails from the plane for a positive hit
                if obj.frame_time in thumbs_received:
                    continue
                thumbs_received.add(obj.frame_time)

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
                mosaic.set_brightness(state.settings.brightness)
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

                cv.ConvertScale(display_img, display_img, scale=state.settings.brightness)
                img_window.set_image(display_img, bgr=True)

                # update console
                image_count += 1
                jpeg_total_bytes += len(obj.jpeg)
                state.jpeg_size = 0.95 * state.jpeg_size + 0.05 * len(obj.jpeg)
                mpstate.console.set_status('Images', 'Images %u' % image_count)
                mpstate.console.set_status('JPGSize', 'JPG Size %.0f' % (jpeg_total_bytes/image_count))
                mpstate.console.set_status('ImageSize', 'ImageSize %.0f' % (image_total_bytes/image_count))

            if isinstance(obj, CommandResponse):
                print('REMOTE: %s' % obj.response)
                
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
    mpstate.command_map['remote'] = (cmd_remote, "remote command")
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

def check_commands():
    '''check for remote commands'''
    state = mpstate.camera_state
    if state.bsend2 is None:
        return
    buf = state.bsend2.recv(0)
    if buf is None:
        return
    try:
        obj = cPickle.loads(str(buf))
        if obj == None:
            return
    except Exception as e:
        return

    if isinstance(obj, CommandPacket):
        stdout_saved = sys.stdout
        buf = cStringIO.StringIO()
        sys.stdout = buf
        mpstate.functions.process_stdin(obj.command)
        sys.stdout = stdout_saved
        pkt = CommandResponse(str(buf.getvalue()))
        buf = cPickle.dumps(pkt, cPickle.HIGHEST_PROTOCOL)
        state.bsend2.send(buf, priority=10000)
        state.bsend2.set_bandwidth(state.settings.bandwidth2)


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.camera_state
    if mpstate.status.watch in ["camera","queue"] and time.time() > state.last_watch+1:
        state.last_watch = time.time()
        cmd_camera(["status" if mpstate.status.watch == "camera" else "queue"])
    # update position interpolator
    state.mpos.add_msg(m)
    if m.get_type() in [ 'DATA16', 'DATA32', 'DATA64', 'DATA96' ]:
        if state.bsocket is not None:
            state.bsocket.incoming.append(m)
