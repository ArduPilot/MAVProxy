import BaseHTTPServer
import cgi
import json
import signal
import thread
import time
import urlparse


class Server():
  def __init__(self, handler, port=9999, mp_state=None):
    self.server = BaseHTTPServer.HTTPServer(('', port), handler)
    self.port = port
    self.allow_reuse_address = True
    self.done = False

  def serve(self):
    signal.signal(signal.SIGINT, self.terminate)
    thread.start_new_thread(self.server.serve_forever, ())
    while not self.done:
      try:
        time.sleep(0.3)
      except IOError:
        pass
    self.server.server_close()

  def start(self):
    thread.start_new_thread(self.serve, self)

  def terminate(self, unused_sig_num, unused_frame):
    self.done = True

  def loop(self):
    url = "http://127.0.0.1:%s" % self.port
    print "server running on port %s" % self.port
    print "documentation and usage is available at %s/\n\n" % url

    self.serve()


class Handler(BaseHTTPServer.BaseHTTPRequestHandler):
  def __init__(self, request, client_address, server):
    self.ctx = {}
    self.points = {}
    BaseHTTPServer.BaseHTTPRequestHandler.__init__(
      self, request, client_address, server)

  def do_GET(self):
    #scheme, host, path, params, query, frag = urlparse.urlparse(self.path)
    #params = cgi.parse_qs(query)
    pass


def main():
  server = Server(Handler, 9999)
  server.loop()


if __name__ == '__main__':
  main()
