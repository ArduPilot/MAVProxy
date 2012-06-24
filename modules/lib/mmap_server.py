import BaseHTTPServer
import json
import os.path
import thread
import urlparse

DOC_DIR = os.path.join(os.path.dirname(__file__), 'mmap_app')


class Server(BaseHTTPServer.HTTPServer):
  def __init__(self, handler, address='', port=9999, module_state=None):
    BaseHTTPServer.HTTPServer.__init__(self, (address, port), handler)
    self.allow_reuse_address = True
    self.module_state = module_state


class Handler(BaseHTTPServer.BaseHTTPRequestHandler):
  def do_GET(self):
    scheme, host, path, params, query, frag = urlparse.urlparse(self.path)
    if path == '/data':
      state = self.server.module_state
      data = {'lat': state.lat,
              'lon': state.lon,
              'heading': state.heading,
              'alt': state.alt,
              'airspeed': state.airspeed,
              'groundspeed': state.groundspeed}
      self.send_response(200)
      self.end_headers()
      self.wfile.write(json.dumps(data))
    else:
      path = path[1:]
      self.send_response(200)
      self.end_headers()
      with open(os.path.join(DOC_DIR, path), 'rb') as f:
        self.wfile.write(f.read())


def start_server(address, port, module_state):
  server = Server(
    Handler, address=address, port=port, module_state=module_state)
  thread.start_new_thread(server.serve_forever, ())
  return server
