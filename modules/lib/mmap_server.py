import BaseHTTPServer
import json
import os.path
import threading
import urlparse

DOC_DIR = os.path.join(os.path.dirname(__file__), 'mmap_app')


class Server(BaseHTTPServer.HTTPServer):
  def __init__(self, handler, address='', port=9999, module_state=None):
    BaseHTTPServer.HTTPServer.__init__(self, (address, port), handler)
    self.allow_reuse_address = True
    self.module_state = module_state


def content_type_for_file(path):
  content_types = {
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.html': 'text/html; charset=utf-8'}
  root, ext = os.path.splitext(path)
  if ext in content_types:
    return content_types[ext]
  else:
    return 'text/plain; charset=utf-8'


class Handler(BaseHTTPServer.BaseHTTPRequestHandler):
  def log_request(code, size=None):
    pass

  def do_POST(self):
    # Expects a JSON string in the body.
    content_len = int(self.headers.getheader('content-length'))
    post_body = self.rfile.read(content_len)
    command = json.loads(post_body)
    self.server.module_state.command(command)

  def do_GET(self):
    scheme, host, path, params, query, frag = urlparse.urlparse(self.path)
    ps = path.split('/')
    # API: /mavlink/mtype
    if len(ps) == 3 and ps[1] == 'mavlink':
      mtype = ps[2]
      msgs = self.server.module_state.messages
      if msgs.has_message(mtype):
        (t, n, m) = msgs.get_message(mtype)
        mdict = m.to_dict()
        resp = {'time_usec': t,
                'index': n,
                'msg': mdict}
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(resp))
    elif path == '/data':
      state = self.server.module_state
      data = {'lat': state.lat,
              'lon': state.lon,
              'heading': state.heading,
              'pitch': state.pitch,
              'roll': state.roll,
              'yaw': state.yaw,
              'alt': state.alt,
              'airspeed': state.airspeed,
              'groundspeed': state.groundspeed,
              'gps_fix_type': state.gps_fix_type,
              'flight_mode': state.flight_mode,
              'wp_change_time': state.wp_change_time}
      if state.client_waypoint:
        data['client_waypoint'] = state.client_waypoint
      msgs = self.server.module_state.messages
      if msgs.has_message('STATUSTEXT'):
        (unused_t, seq, m) = msgs.get_message('STATUSTEXT')
        data['status_text'] = {'severity': m.severity,
                               'text': m.text,
                               'seq': seq}
      self.send_response(200)
      # http://www.ietf.org/rfc/rfc4627.txt says application/json.
      self.send_header('Content-type', 'application/json')
      self.end_headers()
      self.wfile.write(json.dumps(data))
    else:
      # Remove leading '/'.
      path = path[1:]
      # Ignore all directories.  E.g.  for ../../bar/a.txt serve
      # DOC_DIR/a.txt.
      unused_head, path = os.path.split(path)
      # for / serve index.html.
      if path == '':
        path = 'index.html'
      content = None
      error = None
      try:
        with open(os.path.join(DOC_DIR, path), 'rb') as f:
          content = f.read()
      except IOError, e:
        error = str(e)
      if content:
        self.send_response(200)
        self.send_header('Content-type', content_type_for_file(path))
        self.end_headers()
        self.wfile.write(content)
      else:
        self.send_response(404)
        self.end_headers()
        self.wfile.write('Error: %s' % (error,))


def start_server(address, port, module_state):
  server = Server(
    Handler, address=address, port=port, module_state=module_state)
  server_thread = threading.Thread(target=server.serve_forever)
  server_thread.daemon = True
  server_thread.start()
  return server
